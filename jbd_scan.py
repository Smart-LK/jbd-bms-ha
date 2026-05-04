#!/usr/bin/env python3
"""
jbd_scan.py v2.1 - JBD BMS Protocol Scanner & Diagnostic Tool
==============================================================
Testeaza exhaustiv toate variantele de protocol si logheza TOT ce primeste.

Variante testate pentru protocolul DD/A5/77:
  1. Standard:           DD A5 CMD 00 CHK_H CHK_L 77  (big-endian checksum)
  2. LE checksum:        DD A5 CMD 00 CHK_L CHK_H 77  (little-endian checksum)
  3. Addr=00 standard:   DD 00 A5 CMD 00 CHK_H CHK_L 77
  4. Addr=01 standard:   DD 01 A5 CMD 00 CHK_H CHK_L 77
  5. Addr=00 LE chk:     DD 00 A5 CMD 00 CHK_L CHK_H 77
  6. Addr=01 LE chk:     DD 01 A5 CMD 00 CHK_L CHK_H 77

Baud rates testate: 9600 si 19200 (sau specificat cu --baud)

IMPORTANT: Ruleaza CAND BATERIA SE INCARCA sau DESCARCA activ!
La 100% SOC idle curentul = 0.00A este CORECT.

Rulare:
  pip install pyserial
  python3 jbd_scan.py /dev/ttyUSB1              # testeaza 9600 + 19200
  python3 jbd_scan.py /dev/ttyUSB1 --baud 9600
  python3 jbd_scan.py /dev/ttyUSB1 --baud 19200
  python3 jbd_scan.py --scan                    # listeaza porturi

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
# FRAME BUILDERS
# =============================================================================

def jbd_chk_be(payload: bytes) -> tuple[int, int]:
    """Standard JBD checksum: big-endian (CHK_H, CHK_L)"""
    s   = sum(payload) & 0xFFFF
    chk = (~s + 1) & 0xFFFF
    return (chk >> 8) & 0xFF, chk & 0xFF

def jbd_chk_le(payload: bytes) -> tuple[int, int]:
    """Little-endian variant: (CHK_L, CHK_H) - reversed"""
    h, l = jbd_chk_be(payload)
    return l, h  # swapped

def build_dd_request(cmd: int, chk_fn, addr: int = None) -> bytes:
    """
    Construieste request DD/A5/77.
    addr=None -> standard (fara byte adresa)
    addr=N    -> adresabil (DD ADDR A5 CMD ...)
    chk_fn    -> jbd_chk_be (standard) sau jbd_chk_le (LE variant)
    """
    h, l = chk_fn(bytes([cmd, 0x00]))
    if addr is None:
        return bytes([0xDD, 0xA5, cmd, 0x00, h, l, 0x77])
    else:
        return bytes([0xDD, addr & 0xFF, 0xA5, cmd, 0x00, h, l, 0x77])

def build_mos_cmd(xx: int) -> bytes:
    """MOS write: DD 5A E1 02 00 XX CHK_H CHK_L 77"""
    data = bytes([0x00, xx])
    h, l = jbd_chk_be(bytes([0xE1, 0x02]) + data)
    return bytes([0xDD, 0x5A, 0xE1, 0x02]) + data + bytes([h, l, 0x77])

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

def recv_raw_all(ser, timeout: float = 3.0) -> bytes:
    """Citeste TOT ce vine pe serial in timeout secunde. Fara filtrare."""
    buf = bytearray(); start = time.time(); last_rx = time.time()
    while time.time() - start < timeout:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
            last_rx = time.time()
        else:
            # Daca am primit ceva si e liniste > 0.5s, consideram raspunsul complet
            if buf and (time.time() - last_rx > 0.5):
                break
        time.sleep(0.02)
    return bytes(buf)

def parse_raw_response(raw: bytes, expected_cmd: int) -> dict:
    """
    Incearca sa parseze raspunsul in mai multe formate:
    - DD/A5/77 standard (big-endian chk, CMD la offset 1)
    - DD/A5/77 adresabil (CMD la offset 2, cu ADDR la offset 1)
    - Frame inversat (incepe cu 0x77)
    - Orice alta secventa care contine 0xDD
    """
    result = {
        'raw_hex': raw.hex(' ').upper() if raw else '',
        'len': len(raw),
        'format_detected': None,
        'data': None,
    }

    if not raw:
        result['error'] = 'empty'
        return result

    # Afiseaza hex dump
    p(f"  RX ({len(raw)} bytes):")
    for i in range(0, len(raw), 16):
        ch = raw[i:i+16]
        ascii_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in ch)
        p(f"    {i:04X}: {' '.join(f'{b:02X}' for b in ch):<48} |{ascii_part}|")

    # Analiza byte-cu-byte
    p(f"  Analiza byte-cu-byte:")
    p(f"    byte[0]  = 0x{raw[0]:02X} {'= START 0xDD ✓' if raw[0]==0xDD else '≠ 0xDD'}")
    if len(raw) > 1:
        p(f"    byte[1]  = 0x{raw[1]:02X} {'= CMD 0x03 (standard)' if raw[1]==0x03 else f'= 0x{raw[1]:02X}'}")
    if len(raw) > 6:
        p(f"    byte[-1] = 0x{raw[-1]:02X} {'= STOP 0x77 ✓' if raw[-1]==0x77 else '≠ 0x77'}")
        p(f"    byte[-2] = 0x{raw[-2]:02X}  (CHK_L)")
        p(f"    byte[-3] = 0x{raw[-3]:02X}  (CHK_H)")

    # Cauta 0xDD in raspuns
    dd_positions = [i for i, b in enumerate(raw) if b == 0xDD]
    p(f"  Pozitii 0xDD in raspuns: {dd_positions if dd_positions else 'nicaieri'} ← START byte")
    p(f"  Pozitii 0x77 in raspuns: {[i for i, b in enumerate(raw) if b==0x77]} ← STOP byte")
    p(f"  Pozitii 0xA5 in raspuns: {[i for i, b in enumerate(raw) if b==0xA5]} ← READ direction")

    # Incearca variante de parsare
    for start_offset in range(min(4, len(raw))):
        buf = raw[start_offset:]
        if len(buf) < 7: continue

        # Detecteaza: DD CMD STATUS LEN DATA CHK_H CHK_L 77
        if buf[0] == 0xDD:
            for cmd_offset in [1, 2]:  # standard sau adresabil
                if len(buf) <= cmd_offset + 3: continue
                cmd    = buf[cmd_offset]
                status = buf[cmd_offset + 1]
                length = buf[cmd_offset + 2]
                total  = cmd_offset + 3 + length + 2 + 1
                if len(buf) < total: continue
                if buf[total - 1] != 0x77: continue
                if cmd != expected_cmd: continue

                data  = buf[cmd_offset + 3:cmd_offset + 3 + length]
                chk_h = buf[cmd_offset + 3 + length]
                chk_l = buf[cmd_offset + 3 + length + 1]
                exp_h, exp_l = jbd_chk_be(bytes([cmd, length]) + data)

                fmt = f"DD/A5/77 {'adresabil' if cmd_offset==2 else 'standard'}"
                if exp_h == chk_h and exp_l == chk_l:
                    p(f"  FORMAT DETECTAT: {fmt} cu checksum BE CORECT ✓")
                    result['format_detected'] = fmt + ' BE'
                    result['data'] = data
                    result['status'] = status
                    return result
                else:
                    # Incearca si LE
                    if exp_l == chk_h and exp_h == chk_l:
                        p(f"  FORMAT DETECTAT: {fmt} cu checksum LE CORECT ✓")
                        result['format_detected'] = fmt + ' LE'
                        result['data'] = data
                        result['status'] = status
                        return result
                    else:
                        p(f"  Posibil {fmt}: CMD=0x{cmd:02X} STATUS=0x{status:02X} LEN={length} "
                          f"CHK_got={chk_h:02X}{chk_l:02X} CHK_exp_BE={exp_h:02X}{exp_l:02X} "
                          f"CHK_exp_LE={exp_l:02X}{exp_h:02X}")

    result['format_detected'] = 'unknown'
    return result


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
    """Parse DD 0x03 payload per jbdbms.h Status_t."""
    if len(data) < 23:
        return {"error": f"prea scurt: {len(data)} < 23"}
    r = {}
    r['voltage_v']          = round(struct.unpack('>H', data[0:2])[0] / 100.0, 2)
    current_raw             = struct.unpack('>h', data[2:4])[0]
    r['current_a']          = round(current_raw / 100.0, 2)
    r['current_raw']        = current_raw
    r['current_raw_hex']    = f"0x{struct.unpack('>H', data[2:4])[0]:04X}"
    r['current_dir']        = "CHARGE(+)" if current_raw > 0 else ("DISCHARGE(-)" if current_raw < 0 else "IDLE(0) - corect la SOC=100%!")
    r['cap_remaining_ah']   = round(struct.unpack('>H', data[4:6])[0] * 10 / 1000, 2)
    r['cap_nominal_ah']     = round(struct.unpack('>H', data[6:8])[0] * 10 / 1000, 2)
    r['cycles']             = struct.unpack('>H', data[8:10])[0]
    date_raw                = struct.unpack('>H', data[10:12])[0]
    try:    r['production_date'] = f"{2000+(date_raw>>9):04d}-{(date_raw>>5)&0x0F:02d}-{date_raw&0x1F:02d}"
    except: r['production_date'] = f"raw=0x{date_raw:04X}"
    r['balance_low_hex']    = f"0x{struct.unpack('>H', data[12:14])[0]:04X}"
    r['balance_high_hex']   = f"0x{struct.unpack('>H', data[14:16])[0]:04X}"
    fault                   = struct.unpack('>H', data[16:18])[0]
    r['fault_raw']          = f"0x{fault:04X}"
    r['faults']             = [desc for mask, desc in FAULT_BITS.items() if fault & mask] or ["none"]
    sw                      = data[18]
    r['sw_version']         = f"V{sw>>4}.{sw&0x0F}"
    r['soc_pct']            = data[19]
    fet                     = data[20]
    r['fet_raw']            = f"0x{fet:02X}"
    r['charge_mos']         = "ON" if (fet & 0x01) else "OFF"
    r['discharge_mos']      = "ON" if (fet & 0x02) else "OFF"
    r['num_cells']          = data[21]
    r['num_ntc']            = data[22]
    # Temperaturi: 0.1K absolut, celsius = (raw - 2731) / 10 (din jbdbms.h deciCelsius)
    r['temperatures']       = []
    for i in range(data[22]):
        off = 23 + i * 2
        if off + 2 <= len(data):
            raw_t = struct.unpack('>H', data[off:off+2])[0]
            r['temperatures'].append({'raw': raw_t, 'hex': f"0x{raw_t:04X}", 'celsius': round((raw_t-2731)/10.0, 1)})
    r['power_w'] = round(r['voltage_v'] * r['current_a'], 1)
    return r

def parse_cells_0x04(data: bytes) -> dict:
    if len(data) < 2 or len(data) % 2 != 0: return {"error": f"len={len(data)}"}
    n = len(data) // 2
    cells = [struct.unpack('>H', data[i*2:i*2+2])[0] for i in range(n)]
    return {'n': n, 'cells': cells, 'min': min(cells), 'max': max(cells),
            'delta': max(cells)-min(cells), 'avg': round(sum(cells)/n, 1), 'sum': sum(cells)}

def parse_hw_0x05(data: bytes) -> str:
    try:    return data.decode('ascii', errors='replace').strip()
    except: return data.hex()

# =============================================================================
# TEST DD/A5/77 CU CAPTURA RAW
# =============================================================================

def test_dd_variant(ser, req: bytes, label: str, expected_cmd: int, timeout: float = 3.0) -> dict:
    """
    Trimite cererea si captureaza RAW tot ce vine.
    Incearca sa parseze in orice format posibil.
    """
    p(f"  --- {label} ---")
    p(f"  TX ({len(req)} bytes): {req.hex(' ').upper()}")
    ser.reset_input_buffer()
    ser.write(req)
    # Pauza scurta pentru flush
    time.sleep(0.1)
    # Citeste raw, fara filtrare
    raw = recv_raw_all(ser, timeout=timeout)
    if not raw:
        p(f"  RX: NIMIC (timeout {timeout}s)")
        return {'ok': False, 'raw': b''}

    result = parse_raw_response(raw, expected_cmd)
    if result.get('data') is not None:
        p(f"  >>> SUCCES! Format: {result['format_detected']}")
        return {'ok': True, 'raw': raw, 'data': result['data'], 'status': result.get('status')}
    return {'ok': False, 'raw': raw}


def scan_all_dd_variants(ser, baud: int, cmds=(0x03, 0x04, 0x05)):
    """Testeaza TOATE variantele DD/A5/77 pentru fiecare comanda."""
    p(); p("="*65)
    p(f"  DD/A5/77 - Toate variantele @ {baud} bps")
    p("="*65)

    # Defineste toate variantele de request
    variants = [
        ("Standard BE",       None,  jbd_chk_be),
        ("Standard LE chk",   None,  jbd_chk_le),
        ("Addr=0x00 BE",      0x00,  jbd_chk_be),
        ("Addr=0x00 LE chk",  0x00,  jbd_chk_le),
        ("Addr=0x01 BE",      0x01,  jbd_chk_be),
        ("Addr=0x01 LE chk",  0x01,  jbd_chk_le),
    ]

    found_variant = None

    for cmd, name in [(0x03, "Basic Info"), (0x04, "Cell Voltages"), (0x05, "HW Version")]:
        if cmd not in cmds:
            continue
        p(); p(f"  === CMD 0x{cmd:02X} - {name} ===")

        for var_name, addr, chk_fn in variants:
            req = build_dd_request(cmd, chk_fn, addr)
            label = f"{var_name} (0x{cmd:02X})"
            res = test_dd_variant(ser, req, label, cmd, timeout=2.5)
            time.sleep(0.3)

            if res['ok']:
                data = res['data']
                p(f"  PARSE 0x{cmd:02X}:")
                if cmd == 0x03:
                    pr = parse_status_0x03(data)
                    p(f"    V={pr.get('voltage_v')}V I={pr.get('current_a')}A ({pr.get('current_dir')})")
                    p(f"    SoC={pr.get('soc_pct')}% Cap={pr.get('cap_remaining_ah')}Ah")
                    p(f"    CHG={pr.get('charge_mos')} DSG={pr.get('discharge_mos')}")
                    for t in pr.get('temperatures', []):
                        p(f"    Temp: {t['celsius']}C (raw={t['hex']})")
                elif cmd == 0x04:
                    pr = parse_cells_0x04(data)
                    p(f"    {pr.get('n')} celule: min={pr.get('min')}mV max={pr.get('max')}mV delta={pr.get('delta')}mV")
                    for i, v in enumerate(pr.get('cells', [])):
                        p(f"    C{i+1:02d}: {v} mV")
                elif cmd == 0x05:
                    p(f"    HW: '{parse_hw_0x05(data)}'")

                if found_variant is None:
                    found_variant = (baud, var_name, cmd)

    return found_variant

# =============================================================================
# SCANARE PROTOCOL 0x78 CU PARSARE COMPLETA
# =============================================================================

def scan_protocol_78(ser, baud: int) -> dict:
    p(); p("="*65)
    p(f"  PROTOCOL 0x78 Registri @ {baud} bps")
    p("="*65)
    req = build_request_78(0x01, 0x1000, 0x10A0)
    p(f"  TX: {req.hex(' ').upper()}")
    ser.reset_input_buffer(); ser.write(req)
    time.sleep(0.5)
    raw = recv_raw_all(ser, timeout=4.0)

    if not raw:
        p("  TIMEOUT"); return {'ok': False}

    p(f"  RX ({len(raw)} bytes):")
    for i in range(0, len(raw), 16):
        ch = raw[i:i+16]
        ascii_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in ch)
        p(f"    {i:04X}: {' '.join(f'{b:02X}' for b in ch):<48} |{ascii_part}|")

    if len(raw) < 10 or raw[0] != 0x01 or raw[1] != 0x78:
        p("  Header 0x78 invalid"); return {'ok': False}

    data_len = struct.unpack('>H', raw[6:8])[0]
    payload  = raw[8:8 + data_len]
    p(f"  Header OK: data_len={data_len}, payload={len(payload)} bytes")

    if len(payload) < 4:
        return {'ok': False}

    # Parsare completa payload
    p(); p("  === PARSARE COMPLETA PAYLOAD 0x78 ===")

    v_raw   = struct.unpack('>H', payload[0:2])[0]
    i_raw_s = struct.unpack('>h', payload[2:4])[0]
    p(f"  [0:2]   0x{v_raw:04X} = {v_raw}  -> tensiune: {v_raw/100.0:.2f} V")
    p(f"  [2:4]   0x{struct.unpack('>H', payload[2:4])[0]:04X} = {i_raw_s} (signed) -> curent: {i_raw_s/100.0:.2f} A")
    p(f"          directie: {'CHARGE(+)' if i_raw_s>0 else 'DISCHARGE(-)' if i_raw_s<0 else 'IDLE(0) - normal la 100% SOC'}")

    for off in range(4, min(24, len(payload)), 2):
        raw_val = struct.unpack('>H', payload[off:off+2])[0]
        p(f"  [{off}:{off+2}]   0x{raw_val:04X} = {raw_val:6d}  (unsigned) / {struct.unpack('>h', payload[off:off+2])[0]:7d} (signed)")

    # SOC confirmat
    if len(payload) > 24:
        soc_val = struct.unpack('>H', payload[22:24])[0] & 0xFF
        p(f"  [22:24] 0x{struct.unpack('>H', payload[22:24])[0]:04X} = {soc_val} -> SOC: {soc_val}% {'✓' if soc_val <= 100 else '?'}")

    # Num celule
    if len(payload) > 68:
        num_cells = struct.unpack('>H', payload[66:68])[0]
        p(f"  [66:68] 0x{num_cells:04X} = {num_cells} -> num_cells={'✓ ' + str(num_cells) if 0 < num_cells <= 32 else '?'}")

        # Tensiuni celule
        if 0 < num_cells <= 32:
            cells_end = 68 + num_cells * 2
            if len(payload) >= cells_end:
                cells = [struct.unpack('>H', payload[68+i*2:70+i*2])[0] for i in range(num_cells)]
                p(f"  Celule ({num_cells}): min={min(cells)} max={max(cells)} delta={max(cells)-min(cells)} sum={sum(cells)}mV={sum(cells)/1000:.2f}V")
                for i, v in enumerate(cells):
                    p(f"    C{i+1:02d}: {v} mV")

                # NTC
                if len(payload) > cells_end + 2:
                    num_ntc = struct.unpack('>H', payload[cells_end:cells_end+2])[0]
                    p(f"  num_ntc={num_ntc}")
                    for i in range(min(num_ntc, 5)):
                        off = cells_end + 2 + i * 2
                        if off + 2 <= len(payload):
                            raw_t = struct.unpack('>H', payload[off:off+2])[0]
                            # Formula (raw-500)/10:
                            t500 = round((raw_t - 500) / 10.0, 1)
                            # Formula (raw-2731)/10 (standard jbdbms.h):
                            t2731 = round((raw_t - 2731) / 10.0, 1)
                            p(f"  NTC T{i+1}: raw=0x{raw_t:04X}={raw_t} -> (raw-500)/10={t500}C  (raw-2731)/10={t2731}C")
                            p(f"             Probabil corect: {'(raw-500)/10=' + str(t500) + 'C' if 0 < t500 < 80 else ''} {'(raw-2731)/10=' + str(t2731) + 'C' if 0 < t2731 < 80 else ''}")

    # Device name ASCII
    # Cauta sirul "JBD" in payload
    jbd_pos = payload.find(b'JBD')
    if jbd_pos >= 0:
        name_bytes = payload[jbd_pos:jbd_pos+20].split(b'\x00')[0]
        try:    name = name_bytes.decode('ascii')
        except: name = name_bytes.hex()
        p(f"  Device name la offset {jbd_pos}: '{name}'")

    # v6.x vs v7.1 comparatie
    if len(payload) > 8:
        nc_raw = struct.unpack('>H', payload[6:8])[0]
        p(); p("  === COMPARATIE v6.x vs v7.1 ===")
        p(f"  v6.x (GRESIT): payload[6:8]=0x{nc_raw:04X}={nc_raw}, ({nc_raw}-37403)/114.2 = {(nc_raw-37403)/114.2:.2f} A")
        p(f"  v7.1 (CORECT): payload[2:4] signed = {i_raw_s}/100 = {i_raw_s/100.0:.2f} A")

    return {'ok': True, 'voltage': v_raw/100.0, 'current': i_raw_s/100.0, 'payload': payload}

# =============================================================================
# VERIFICARE CHECKSUM
# =============================================================================

def verify_checksums():
    p(); p("="*65); p("  VERIFICARE CHECKSUM - BE vs LE"); p("="*65)
    for cmd in [0x03, 0x04, 0x05]:
        req_be = build_dd_request(cmd, jbd_chk_be)
        req_le = build_dd_request(cmd, jbd_chk_le)
        p(f"  CMD 0x{cmd:02X}:")
        p(f"    Standard BE: {req_be.hex(' ').upper()}")
        p(f"    LE checksum: {req_le.hex(' ').upper()} (CHK bytes inversate)")
    p(f"  MOS (DSG off): {build_mos_cmd(0x02).hex(' ').upper()}")

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
    parser = argparse.ArgumentParser(description="JBD BMS Protocol Scanner v2.1")
    parser.add_argument("port",    nargs="?",  default=None)
    parser.add_argument("--baud",  type=int,   default=None, help="Baud rate (implicit: testeaza 9600 si 19200)")
    parser.add_argument("--proto", choices=["dd","78","all"], default="all")
    parser.add_argument("--scan",  action="store_true")
    args = parser.parse_args()

    p("="*65); p("  JBD BMS Protocol Scanner v2.1")
    p(f"  Data: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    p(f"  Log:  {LOG_FILE}"); p("="*65)
    p("  IMPORTANT: Ruleaza cand bateria se incarca/descarca activ!")
    p("  La 100% SOC idle, curentul 0.00A este CORECT.")

    verify_checksums()

    if args.scan:
        p(); p("  Porturi seriale disponibile:")
        if not scan_ports(): p("  (niciun port gasit)")
        return

    if not args.port:
        p(); p("  EROARE: Specifica portul! (sau --scan)"); sys.exit(1)

    bauds   = [args.baud] if args.baud else [9600, 19200]
    all_res = {}

    for baud in bauds:
        p(); p("="*65); p(f"  PORT: {args.port} @ {baud} bps 8N1"); p("="*65)
        try:
            ser = serial.Serial(port=args.port, baudrate=baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.05)
            p(f"  Serial OK @ {baud} bps")
        except Exception as e:
            p(f"  EROARE: {e}"); continue

        time.sleep(0.3); res = {}

        if args.proto in ("dd", "all"):
            found = scan_all_dd_variants(ser, baud)
            res['dd'] = found

        time.sleep(0.5)

        if args.proto in ("78", "all"):
            res['78'] = scan_protocol_78(ser, baud)

        all_res[baud] = res
        ser.close(); time.sleep(0.5)

    # Sumar final
    p(); p("="*65); p("  SUMAR FINAL"); p("="*65)
    for baud, res in all_res.items():
        dd_ok = res.get('dd')
        ok78  = res.get('78', {}).get('ok', False)
        p(f"  @ {baud} bps:")
        p(f"    DD/A5/77: {'OK - varianta: ' + str(dd_ok[1]) if dd_ok else 'TIMEOUT pe toate variantele'}")
        p(f"    0x78:     {'OK' if ok78 else 'FAIL'}")

    # Recomandare
    p()
    dd_found = next((r['dd'] for r in all_res.values() if r.get('dd')), None)
    ok78_found = next((b for b, r in all_res.items() if r.get('78', {}).get('ok')), None)

    if dd_found:
        baud, variant, cmd = dd_found
        p(f"  RECOMANDAT: DD/A5/77 @ {baud} bps varianta '{variant}'")
        p(f"  Actualizeaza addon la v7.x cu baud_rate={baud}")
    elif ok78_found:
        p(f"  RECOMANDAT: Protocol 0x78 @ {ok78_found} bps")
        p(f"  Foloseste addon v7.1 (0x78 protocol) cu baud_rate={ok78_found}")
    else:
        p("  NICIUN PROTOCOL NU A RASPUNS! Verifica portul, cablul, baud rate.")

    p(); p(f"  Log: {LOG_FILE}"); p("="*65)

if __name__ == "__main__": main()
