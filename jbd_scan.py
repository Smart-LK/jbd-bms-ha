#!/usr/bin/env python3
"""
jbd_scan.py v2.0 - JBD BMS Protocol Scanner & Diagnostic Tool
==============================================================
Testeaza protocoalele JBD si logheza RAW + parsed.

Protocol JBD (confirmat din joba-1/Joba_JbdBms/jbdbms.h si JBD Protocol V4):
  Request standard:    DD A5 [CMD] [LEN] [DATA] [CHK_H] [CHK_L] 77
  Request adresabil:   DD [ADDR] A5 [CMD] [LEN] [DATA] [CHK_H] [CHK_L] 77
  Response:            DD [CMD] [STATUS] [LEN] [DATA] [CHK_H] [CHK_L] 77
  STATUS: 0x00=OK, 0x80=error
  CHECKSUM: -sum(CMD, LEN, DATA...) in 16-bit two's complement

Status struct layout (din jbdbms.h, confirmat):
  [0:2]  voltage           uint16 10mV
  [2:4]  current           int16  10mA  (+charge, -discharge)
  [4:6]  remainingCapacity uint16 10mAh
  [6:8]  nominalCapacity   uint16 10mAh
  [8:10] cycles            uint16
  [10:12] productionDate   uint16 (bits: year[15:9] month[8:5] day[4:0])
  [12:14] balanceLow       uint16 bit per cell 1-16
  [14:16] balanceHigh      uint16 bit per cell 17-32
  [16:18] fault            uint16 bit field
  [18]   version           uint8  (high nibble=major, low=minor)
  [19]   currentCapacity   uint8  %
  [20]   mosfetStatus      uint8  (bit0=CHG, bit1=DSG, 1=ON)
  [21]   cells             uint8
  [22]   ntcs              uint8
  [23+]  temperatures      N x uint16, unit 0.1K absolute (0.1K=deciKelvin)
                           Formula: celsius = (raw - 2731) / 10.0

NOTA UP16S015 RS485:
  Modelul UP16S015 (parallel pack BMS) poate folosi varianta adresabila:
  DD [ADDR] A5 [CMD] ... unde ADDR=01 pentru primul BMS
  CAUZA TIMEOUT v7.0 scan: am trimis DD A5 03 (fara ADDR byte)
  Variante testate in acest scan: standard + adresabil ADDR=01 + ADDR=00

Rulare:
  pip install pyserial
  python3 jbd_scan.py /dev/ttyUSB1                        # testeaza 9600 si 19200
  python3 jbd_scan.py /dev/ttyUSB1 --baud 9600            # baud specific
  python3 jbd_scan.py /dev/ttyUSB1 --baud 19200 --addr 1  # adresa specifica
  python3 jbd_scan.py --scan                              # listeaza porturi

Ruleaza CAND BATERIA SE INCARCA sau SE DESCARCA pentru a verifica curentul!
La 100% SOC idle curentul poate fi 0.00A - e corect.

Autor: Smart-LK / Claude Sonnet, mai 2026
"""

import argparse
import glob
import logging
import os
import struct
import sys
import time
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("EROARE: pip install pyserial")
    sys.exit(1)

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "jbd_scan.log")

def setup_logger():
    fmt = logging.Formatter("%(asctime)s %(message)s", datefmt="%H:%M:%S.%f")
    log = logging.getLogger("scan")
    log.setLevel(logging.DEBUG)
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(fmt); log.addHandler(ch)
    fh = logging.FileHandler(LOG_FILE, mode="w", encoding="utf-8")
    fh.setFormatter(fmt); log.addHandler(fh)
    return log

log = logging.getLogger("scan")
p = lambda msg="": log.info(msg)

# =============================================================================
# CHECKSUM JBD DD/A5/77
# Covers: CMD, LEN, DATA bytes (NOT start DD, NOT direction A5/5A, NOT stop 77)
# From jbdbms.h: genCrc(cmd, len, data)
# =============================================================================

def jbd_checksum(payload: bytes) -> tuple[int, int]:
    """
    Checksum JBD: -sum(payload) in 16-bit two's complement
    payload = bytes([CMD, LEN, DATA...])
    """
    s   = sum(payload) & 0xFFFF
    chk = (~s + 1) & 0xFFFF
    return (chk >> 8) & 0xFF, chk & 0xFF


def build_read_standard(cmd: int) -> bytes:
    """Request standard: DD A5 CMD 00 CHK_H CHK_L 77"""
    h, l = jbd_checksum(bytes([cmd, 0x00]))
    return bytes([0xDD, 0xA5, cmd, 0x00, h, l, 0x77])


def build_read_addressed(addr: int, cmd: int) -> bytes:
    """
    Request adresabil (RS485 UP series): DD ADDR A5 CMD 00 CHK_H CHK_L 77
    Checksum still covers only [CMD, 0x00] (ADDR byte is not in checksum)
    Ref: 'instead of dd a5, packets start dd 01 a5 where 01 is the bank address'
    """
    h, l = jbd_checksum(bytes([cmd, 0x00]))
    return bytes([0xDD, addr & 0xFF, 0xA5, cmd, 0x00, h, l, 0x77])


def build_mos_cmd(xx: int) -> bytes:
    """MOS write: DD 5A E1 02 00 XX CHK_H CHK_L 77"""
    data = bytes([0x00, xx])
    h, l = jbd_checksum(bytes([0xE1, 0x02]) + data)
    return bytes([0xDD, 0x5A, 0xE1, 0x02]) + data + bytes([h, l, 0x77])

# =============================================================================
# CRC16 MODBUS (pentru protocolul 0x78 proprietar din addon v6.x)
# =============================================================================

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def build_request_78(addr: int, start_reg: int, end_reg: int) -> bytes:
    frame = bytes([addr, 0x78,
        (start_reg >> 8) & 0xFF, start_reg & 0xFF,
        (end_reg   >> 8) & 0xFF, end_reg   & 0xFF,
        0x00, 0x00])
    crc = crc16_modbus(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

# =============================================================================
# RECEIVE
# =============================================================================

def recv_raw(ser, timeout: float = 3.0) -> bytes:
    buf = bytearray(); start = time.time()
    while time.time() - start < timeout:
        chunk = ser.read(256)
        if chunk: buf.extend(chunk); start = time.time()
        elif buf: break
        time.sleep(0.02)
    return bytes(buf)


def recv_dd_response(ser, expected_cmd: int, timeout: float = 3.0) -> tuple:
    """
    Primeste raspuns DD/CMD/STATUS/LEN/DATA/CHK/77.
    Suporta atat raspunsul standard cat si cel adresabil.
    Returneaza (payload, status_str) sau (None, eroare).
    """
    buf = bytearray(); start = time.time()
    while time.time() - start < timeout:
        chunk = ser.read(256)
        if chunk: buf.extend(chunk)
        while len(buf) > 0 and buf[0] != 0xDD: buf = buf[1:]
        if len(buf) < 4: time.sleep(0.01); continue
        # Detecteaza daca e format standard (DD CMD ...) sau adresabil (DD ADDR CMD ...)
        # Standard:  DD [CMD] [STATUS] [LEN] ...
        # Adresabil: DD [ADDR] [CMD] [STATUS] [LEN] ...
        # Diferenta: in standard buf[1]=CMD, in adresabil buf[1]=ADDR (!=0xDD,!=cmd)
        # Incercam ambele formate
        for offset in [0, 1]:  # 0=standard, 1=adresabil
            if len(buf) < 4 + offset: continue
            cmd    = buf[1 + offset]
            status = buf[2 + offset]
            length = buf[3 + offset]
            if cmd != expected_cmd: continue
            total = (1 + offset) + 1 + 1 + 1 + length + 2 + 1  # DD[+ADDR]+CMD+STATUS+LEN+DATA+CHK+77
            if len(buf) < total: break
            if buf[total - 1] != 0x77: break
            frame = bytes(buf[:total])
            buf   = buf[total:]
            if status != 0x00: return None, f"BMS_ERROR 0x{status:02X}"
            data_start = 1 + offset + 3  # after DD[+ADDR]+CMD+STATUS+LEN
            data  = frame[data_start:data_start + length]
            chk_h = frame[data_start + length]
            chk_l = frame[data_start + length + 1]
            exp_h, exp_l = jbd_checksum(bytes([cmd, length]) + data)
            ok_str = "OK" if (exp_h==chk_h and exp_l==chk_l) else f"CHK_MISMATCH(exp={exp_h:02X}{exp_l:02X} got={chk_h:02X}{chk_l:02X})"
            fmt = "adresabil" if offset==1 else "standard"
            return data, f"{ok_str} ({fmt} format)"
        time.sleep(0.01)
    return None, "TIMEOUT"

# =============================================================================
# PARSE DD/A5/77 STATUS (0x03) - conform jbdbms.h Status_t
# =============================================================================

FAULT_BITS = {
    0x0001: "cell_overvoltage",    0x0002: "cell_undervoltage",
    0x0004: "pack_overvoltage",    0x0008: "pack_undervoltage",
    0x0010: "chg_overtemp",       0x0020: "chg_undertemp",
    0x0040: "dchg_overtemp",      0x0080: "dchg_undertemp",
    0x0100: "chg_overcurrent",    0x0200: "dchg_overcurrent",
    0x0400: "short_circuit",      0x0800: "frontend_ic_error",
    0x1000: "sw_lock_mos",
}


def parse_status_0x03(data: bytes) -> dict:
    """
    Parse DD 0x03 response payload (Status_t din jbdbms.h).

    Layout EXACT din jbdbms.h:
    [0:2]  voltage            uint16  10mV
    [2:4]  current            int16   10mA  (+charge, -discharge)
    [4:6]  remainingCapacity  uint16  10mAh
    [6:8]  nominalCapacity    uint16  10mAh
    [8:10] cycles             uint16
    [10:12] productionDate    uint16
    [12:14] balanceLow        uint16
    [14:16] balanceHigh       uint16
    [16:18] fault             uint16
    [18]   version            uint8
    [19]   currentCapacity    uint8   %
    [20]   mosfetStatus       uint8   (bit0=CHG, bit1=DSG, 1=ON)
    [21]   cells              uint8
    [22]   ntcs               uint8
    [23+]  temperatures       N*2 bytes, unit 0.1K absolute
           Formula: celsius = (raw - 2731) / 10.0
           Exemplu: 0x0B98=2968 -> (2968-2731)/10 = 23.7C
    """
    MIN = 23
    if len(data) < MIN:
        return {"error": f"date prea scurte: {len(data)} < {MIN}"}

    r = {}
    r['voltage_v']  = round(struct.unpack('>H', data[0:2])[0] / 100.0, 2)  # 10mV -> V

    # Current SIGNED int16, 10mA -> A
    # NOTA: la 100% SOC idle, curentul POATE fi 0.0A (corect!)
    # Ruleaza scanul cand bateria e activ incarcata/descarcata pt verificare
    current_raw  = struct.unpack('>h', data[2:4])[0]
    r['current_a']           = round(current_raw / 100.0, 2)
    r['current_raw_signed']  = current_raw
    r['current_raw_hex']     = f"0x{struct.unpack('>H', data[2:4])[0]:04X}"
    r['current_direction']   = "CHARGING(+)" if current_raw > 0 else ("DISCHARGING(-)" if current_raw < 0 else "IDLE(0) - normal la 100% SOC!")

    r['cap_remaining_ah'] = round(struct.unpack('>H', data[4:6])[0] * 10 / 1000, 2)
    r['cap_nominal_ah']   = round(struct.unpack('>H', data[6:8])[0] * 10 / 1000, 2)
    r['cycles']           = struct.unpack('>H', data[8:10])[0]

    date_raw = struct.unpack('>H', data[10:12])[0]
    try:
        r['production_date'] = f"{2000+(date_raw>>9):04d}-{(date_raw>>5)&0x0F:02d}-{date_raw&0x1F:02d}"
    except:
        r['production_date'] = f"raw=0x{date_raw:04X}"

    r['balance_low_hex']  = f"0x{struct.unpack('>H', data[12:14])[0]:04X}"
    r['balance_high_hex'] = f"0x{struct.unpack('>H', data[14:16])[0]:04X}"

    fault = struct.unpack('>H', data[16:18])[0]
    r['fault_raw']    = f"0x{fault:04X}"
    r['fault_active'] = [desc for mask, desc in FAULT_BITS.items() if fault & mask] or ["none"]

    sw = data[18]
    r['software_version'] = f"V{sw >> 4}.{sw & 0x0F}"
    r['soc_pct']          = data[19]

    fet = data[20]
    r['fet_raw']       = f"0x{fet:02X}"
    r['charge_mos']    = "ON" if (fet & 0x01) else "OFF"  # bit0
    r['discharge_mos'] = "ON" if (fet & 0x02) else "OFF"  # bit1

    r['num_cells'] = data[21]
    num_ntc = data[22]; r['num_ntc'] = num_ntc

    # Temperaturi: unit 0.1K absolut
    # FORMULA: celsius = (raw - 2731) / 10.0 (din jbdbms.h: deciCelsius = deciKelvin - 2731)
    # Exemplu din PDF: 0x0B98=2968 -> (2968-2731)/10 = 23.7C
    temps = []
    for i in range(num_ntc):
        off = 23 + i * 2
        if off + 2 <= len(data):
            raw_t = struct.unpack('>H', data[off:off+2])[0]
            temps.append({
                'raw': raw_t, 'hex': f"0x{raw_t:04X}",
                'celsius': round((raw_t - 2731) / 10.0, 1),
            })
    r['temperatures'] = temps
    r['power_w'] = round(r['voltage_v'] * r['current_a'], 1)
    return r


def parse_cells_0x04(data: bytes) -> dict:
    """Parse DD 0x04 response payload (Cells_t din jbdbms.h)."""
    if len(data) < 2 or len(data) % 2 != 0:
        return {"error": f"invalid len={len(data)}"}
    n = len(data) // 2
    cells = [struct.unpack('>H', data[i*2:i*2+2])[0] for i in range(n)]
    return {
        'num_cells': n, 'cells_mv': cells,
        'min_mv': min(cells), 'max_mv': max(cells),
        'delta_mv': max(cells) - min(cells),
        'avg_mv': round(sum(cells) / n, 1),
        'sum_mv': sum(cells),
    }


def parse_hw_0x05(data: bytes) -> dict:
    """Parse DD 0x05 response (Hardware_t din jbdbms.h)."""
    try: hw = data.decode('ascii', errors='replace').strip()
    except: hw = data.hex()
    return {'hardware_id': hw, 'raw_hex': data.hex()}

# =============================================================================
# SCANARE PROTOCOL DD/A5/77 (standard + adresabil)
# =============================================================================

def scan_dd_protocol(ser, baud: int, addr: int = None):
    """
    Testeaza protocolul DD/A5/77.
    addr=None: testeaza varianta standard (fara byte adresa)
    addr=N:    testeaza varianta adresabila (DD ADDR A5 CMD ...)
    """
    if addr is None:
        label = "DD/A5/77 Standard"
        build_req = lambda cmd: build_read_standard(cmd)
    else:
        label = f"DD/A5/77 Adresabil ADDR=0x{addr:02X}"
        build_req = lambda cmd: build_read_addressed(addr, cmd)

    p(); p("="*65)
    p(f"  {label} @ {baud} bps")
    p("="*65)
    results = {}

    for cmd, name in [(0x03, "Basic Info & Status"), (0x04, "Cell Voltages"), (0x05, "Hardware Version")]:
        req = build_req(cmd)
        p(); p(f"  [{label[:5]}-{cmd:02X}] READ 0x{cmd:02X} - {name}")
        p(f"  TX ({len(req)} bytes): {req.hex(' ').upper()}")

        ser.reset_input_buffer(); ser.write(req)
        time.sleep(0.2)
        data, status = recv_dd_response(ser, cmd)

        if data is not None:
            p(f"  RX ({len(data)} bytes): {data.hex(' ').upper()}")
            p(f"  Checksum+Format: {status}")

            if cmd == 0x03:
                parsed = parse_status_0x03(data)
                results['cmd03'] = {'status': status, 'raw': data.hex(), 'parsed': parsed}
                p(); p("  === PARSED 0x03 (Status_t din jbdbms.h) ===")
                p(f"  Tensiune:           {parsed.get('voltage_v')} V")
                p(f"  Curent (signed):    {parsed.get('current_a')} A")
                p(f"    hex: {parsed.get('current_raw_hex')} = {parsed.get('current_raw_signed')} (10mA units)")
                p(f"    directie: {parsed.get('current_direction')}")
                p(f"  Putere:             {parsed.get('power_w')} W")
                p(f"  SoC:                {parsed.get('soc_pct')} %")
                p(f"  Cap. ramasa:        {parsed.get('cap_remaining_ah')} Ah  (raw x10mAh)")
                p(f"  Cap. nominala:      {parsed.get('cap_nominal_ah')} Ah  (raw x10mAh)")
                p(f"  Cicluri:            {parsed.get('cycles')}")
                p(f"  Data productie:     {parsed.get('production_date')}")
                p(f"  Balance low:        {parsed.get('balance_low_hex')}")
                p(f"  Balance high:       {parsed.get('balance_high_hex')}")
                p(f"  Fault raw:          {parsed.get('fault_raw')}")
                p(f"  Faulturi active:    {', '.join(parsed.get('fault_active', []))}")
                p(f"  Versiune SW:        {parsed.get('software_version')}")
                p(f"  FET status:         raw={parsed.get('fet_raw')} CHG={parsed.get('charge_mos')} DSG={parsed.get('discharge_mos')}")
                p(f"  Numar celule:       {parsed.get('num_cells')}")
                p(f"  Numar NTC:          {parsed.get('num_ntc')}")
                p(f"  NOTA temperaturi:   formula din jbdbms.h: (raw-2731)/10 = Celsius")
                for i, t in enumerate(parsed.get('temperatures', [])):
                    p(f"  Temp T{i+1}:           {t['celsius']} C  (raw={t['raw']} = {t['hex']})")

            elif cmd == 0x04:
                parsed = parse_cells_0x04(data)
                results['cmd04'] = {'status': status, 'raw': data.hex(), 'parsed': parsed}
                p(); p("  === PARSED 0x04 ===")
                p(f"  Numar celule:  {parsed.get('num_cells')}")
                p(f"  Min:   {parsed.get('min_mv')} mV")
                p(f"  Max:   {parsed.get('max_mv')} mV")
                p(f"  Delta: {parsed.get('delta_mv')} mV")
                p(f"  Avg:   {parsed.get('avg_mv')} mV")
                p(f"  Sum:   {parsed.get('sum_mv')} mV = {parsed.get('sum_mv')/1000:.2f} V")
                for i, v in enumerate(parsed.get('cells_mv', [])):
                    p(f"  C{i+1:02d}: {v} mV")

            elif cmd == 0x05:
                parsed = parse_hw_0x05(data)
                results['cmd05'] = {'status': status, 'raw': data.hex(), 'parsed': parsed}
                p(); p("  === PARSED 0x05 ===")
                p(f"  Hardware ID: '{parsed.get('hardware_id')}'")
                p(f"  Raw hex: {parsed.get('raw_hex')}")
        else:
            p(f"  ESUAT: {status}")
            results[f'cmd{cmd:02x}'] = {'status': status, 'error': True}
        time.sleep(0.3)

    # MOS frames (fara trimitere)
    p(); p("  MOS frames (FARA TRIMITERE):")
    for xx, desc in [(0x00,"release all"),(0x01,"CHG off"),(0x02,"DSG off"),(0x03,"both off")]:
        p(f"  XX=0x{xx:02X} ({desc:12s}): {build_mos_cmd(xx).hex(' ').upper()}")

    ok = sum(1 for r in results.values() if not r.get('error'))
    p(); p(f"  === SUMAR {label}: {ok}/{len(results)} comenzi OK ===")
    if ok >= 2:
        p(f"  >>> FUNCTIONAL! Foloseste acest protocol in addon.")
        if results.get('cmd03') and not results['cmd03'].get('error'):
            pr = results['cmd03']['parsed']
            p(f"  >>> Curent: {pr.get('current_a')} A  ({pr.get('current_direction')})")
    else:
        p(f"  >>> NEFUNCTIONAL pe acest port/baud/adresa")
    return results

# =============================================================================
# SCANARE PROTOCOL 0x78 REGISTRI (protocol proprietar addon v6.x)
# =============================================================================

def scan_protocol_78(ser, baud: int):
    p(); p("="*65)
    p(f"  PROTOCOL 0x78 Registri (addon v6.x) @ {baud} bps")
    p("="*65)
    req = build_request_78(0x01, 0x1000, 0x10A0)
    p(f"  TX: {req.hex(' ').upper()}")
    ser.reset_input_buffer(); ser.write(req); time.sleep(0.5)
    raw = recv_raw(ser, timeout=4.0)

    if not raw:
        p("  TIMEOUT"); return {'ok': False}

    p(f"  RX ({len(raw)} bytes):")
    for i in range(0, len(raw), 16):
        ch = raw[i:i+16]
        p(f"    {i:04X}: {' '.join(f'{b:02X}' for b in ch):<48}")

    if len(raw) < 10 or raw[0] != 0x01 or raw[1] != 0x78:
        p("  Header invalid"); return {'ok': False}

    data_len = struct.unpack('>H', raw[6:8])[0]
    payload  = raw[8:8+data_len]
    p(f"  Header OK: data_len={data_len}")

    if len(payload) >= 4:
        v_raw   = struct.unpack('>H', payload[0:2])[0]
        i_raw_s = struct.unpack('>h', payload[2:4])[0]  # SIGNED
        p(); p("  === DATE 0x78 (payload offset 0-3) ===")
        p(f"  [0:2] Tensiune:  0x{v_raw:04X} = {v_raw} -> {v_raw/100.0:.2f} V")
        p(f"  [2:4] Curent:    0x{struct.unpack('>H', payload[2:4])[0]:04X} = {i_raw_s} (signed) -> {i_raw_s/100.0:.2f} A")
        p(f"         Directie: {'CHARGING(+)' if i_raw_s>0 else 'DISCHARGING(-)' if i_raw_s<0 else 'IDLE - corect la baterie plina!'}")

        if len(payload) > 8:
            nc_raw = struct.unpack('>H', payload[6:8])[0]
            p(f"  [6:8] = 0x{nc_raw:04X} = {nc_raw} -> v6.x il folosea ca curent GRESIT!")
            p(f"         v6.x: ({nc_raw}-37403)/114.2 = {(nc_raw-37403)/114.2:.2f} A  <- INCORECT")
            p(f"         v7.1: [2:4] signed / 100 = {i_raw_s/100.0:.2f} A  <- CORECT")

        p(f"  Tensiune verificare: {v_raw/100.0:.2f} V (matches BMS display?)")
        p(f"  Curent: {i_raw_s/100.0:.2f} A  (0.0A = normal la 100% SOC/idle!)")

    return {'ok': True, 'voltage': raw[8]/100.0 if len(raw)>8 else None}

# =============================================================================
# VERIFICARE CHECKSUM
# =============================================================================

def verify_checksum_examples():
    p(); p("="*65); p("  VERIFICARE CHECKSUM - Exemple din protocol v4 PDF")
    p("  Confirmat cu implementarea din joba-1/Joba_JbdBms/jbdbms.h")
    p("="*65)
    tests = [
        ("Read 0x03 standard",    build_read_standard(0x03),   bytes([0xDD,0xA5,0x03,0x00,0xFF,0xFD,0x77])),
        ("Read 0x04 standard",    build_read_standard(0x04),   bytes([0xDD,0xA5,0x04,0x00,0xFF,0xFC,0x77])),
        ("Read 0x05 standard",    build_read_standard(0x05),   bytes([0xDD,0xA5,0x05,0x00,0xFF,0xFB,0x77])),
        ("Read 0x03 addr=01",     build_read_addressed(0x01,0x03), None),  # no reference to compare
        ("MOS DSG off (XX=02)",   build_mos_cmd(0x02),         bytes([0xDD,0x5A,0xE1,0x02,0x00,0x02,0xFF,0x1B,0x77])),
    ]
    for name, built, expected in tests:
        p(f"  {name}:")
        p(f"    Construit: {built.hex(' ').upper()}")
        if expected:
            match = built == expected
            p(f"    Asteptat:  {expected.hex(' ').upper()}")
            p(f"    Status:    {'OK' if match else 'MISMATCH'}")
        else:
            p(f"    (fara referinta de comparatie - nou in v2.0)")

# =============================================================================
# PORT SERIAL
# =============================================================================

def scan_ports() -> list:
    ports = []
    by_id = "/dev/serial/by-id/"
    if os.path.isdir(by_id):
        for link in sorted(glob.glob(by_id + "*")):
            try:
                real = os.path.realpath(link)
                p(f"  by-id: {os.path.basename(link)} -> {real}"); ports.append(real)
            except: pass
    for dev in sorted(glob.glob("/dev/ttyUSB*")):
        if dev not in ports: p(f"  ttyUSB: {dev}"); ports.append(dev)
    for pi in serial.tools.list_ports.comports():
        if pi.device not in ports: p(f"  {pi.device} [{pi.description}]"); ports.append(pi.device)
    return list(dict.fromkeys(ports))

# =============================================================================
# MAIN
# =============================================================================

def main():
    setup_logger()
    parser = argparse.ArgumentParser(description="JBD BMS Protocol Scanner v2.0")
    parser.add_argument("port",    nargs="?", default=None)
    parser.add_argument("--baud",  type=int,  default=None, help="Baud (implicit: testeaza 9600 si 19200)")
    parser.add_argument("--addr",  type=lambda x:int(x,0), default=None,
                        help="Adresa RS485 specifica (implicit: testeaza standard + 0x00 + 0x01)")
    parser.add_argument("--proto", choices=["dd","78","all"], default="all")
    parser.add_argument("--scan",  action="store_true")
    args = parser.parse_args()

    p("="*65); p("  JBD BMS Protocol Scanner v2.0")
    p(f"  Data: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    p(f"  Log:  {LOG_FILE}")
    p("="*65)
    p("  IMPORTANT: Ruleaza cand bateria se incarca/descarca activ!")
    p("  La 100% SOC idle curentul = 0.00A este CORECT.")

    verify_checksum_examples()

    if args.scan:
        p(); p("  Porturi seriale disponibile:")
        ports = scan_ports()
        if not ports: p("  (niciun port gasit)")
        return

    if not args.port:
        p(); p("  EROARE: Specifica portul! (sau --scan)")
        sys.exit(1)

    bauds   = [args.baud] if args.baud else [9600, 19200]
    # Adrese de testat: standard (None), addr=0x00, addr=0x01
    addrs   = [args.addr] if args.addr is not None else [None, 0x00, 0x01]
    all_res = {}

    for baud in bauds:
        p(); p("="*65); p(f"  PORT: {args.port} @ {baud} bps"); p("="*65)
        try:
            ser = serial.Serial(port=args.port, baudrate=baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.1)
            p(f"  Serial OK")
        except Exception as e: p(f"  EROARE: {e}"); continue

        time.sleep(0.3); res = {}

        if args.proto in ("dd", "all"):
            for addr in addrs:
                key = f"dd_addr_{addr if addr is not None else 'none'}"
                res[key] = scan_dd_protocol(ser, baud, addr)
                time.sleep(0.5)

        if args.proto in ("78", "all"):
            res['proto_78'] = scan_protocol_78(ser, baud)

        all_res[baud] = res
        ser.close(); time.sleep(0.5)

    # Sumar
    p(); p("="*65); p("  SUMAR FINAL"); p("="*65)
    best = None
    for baud, res in all_res.items():
        for key, r in res.items():
            if key.startswith('dd_'):
                c03 = r.get('cmd03', {}); c04 = r.get('cmd04', {})
                ok03 = not c03.get('error', True); ok04 = not c04.get('error', True)
                addr_str = key.replace('dd_addr_', 'addr=')
                p(f"  @ {baud} {key}: 0x03={'OK' if ok03 else 'FAIL'} 0x04={'OK' if ok04 else 'FAIL'}")
                if ok03 and ok04 and best is None:
                    best = (baud, addr_str, 'DD/A5/77')
            elif key == 'proto_78':
                p(f"  @ {baud} 0x78: {'OK' if r.get('ok') else 'FAIL'}")
                if r.get('ok') and best is None:
                    best = (baud, 'N/A', '0x78')

    p()
    if best:
        baud, addr_str, proto = best
        p(f"  RECOMANDAT: Protocol {proto} @ {baud} bps ({addr_str})")
        if proto == 'DD/A5/77':
            p(f"  Addon v7.x: schimba protocolul la DD/A5/77 cu baud_rate={baud}")
        else:
            p(f"  Addon v7.1 cu 0x78 protocol, baud_rate={baud}")
    else:
        p("  NICIUN PROTOCOL NU A RASPUNS!")
        p("  Verifica: portul serial, cablul, baud rate, adresa")

    p(); p(f"  Log complet: {LOG_FILE}"); p("="*65)

if __name__ == "__main__": main()
