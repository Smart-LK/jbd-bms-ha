#!/usr/bin/env python3
"""
jbd_scan.py - JBD BMS Protocol Scanner & Diagnostic Tool
=========================================================
Testeaza ambele protocoale JBD si logheza TOT ce primeste de la BMS.
Folosit INAINTE de update-ul addon-ului la v7.0.0 pentru a confirma
ca protocolul DD/A5/77 standard functioneaza pe hardware-ul tau.

Protocoale testate:
  A) DD/A5/77 Standard (protocol oficial PDF, comenzile 0x03, 0x04, 0x05)
  B) 0x78 Register (protocolul vechi folosit de addon v6.x)

Rezultat: fisier jbd_scan.log cu tot traficul raw + date parsate.

Rulare:
  pip install pyserial
  python3 jbd_scan.py /dev/ttyUSB0
  python3 jbd_scan.py /dev/ttyUSB0 --baud 19200
  python3 jbd_scan.py /dev/ttyUSB0 --baud 9600 --proto a    # doar DD/A5/77
  python3 jbd_scan.py /dev/ttyUSB0 --baud 9600 --proto b    # doar 0x78
  python3 jbd_scan.py --scan                                 # listeaza porturi

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
    print("EROARE: pyserial nu e instalat. Ruleaza: pip install pyserial")
    sys.exit(1)

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "jbd_scan.log")

def setup_logger():
    fmt = logging.Formatter("%(asctime)s %(message)s", datefmt="%H:%M:%S.%f")
    log = logging.getLogger("scan")
    log.setLevel(logging.DEBUG)
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(fmt)
    log.addHandler(ch)
    fh = logging.FileHandler(LOG_FILE, mode="w", encoding="utf-8")
    fh.setFormatter(fmt)
    log.addHandler(fh)
    return log

log = logging.getLogger("scan")
p = lambda msg="": log.info(msg)

# ── Checksum DD/A5/77 ─────────────────────────────────────────────────────────

def jbd_checksum(payload: bytes) -> tuple[int, int]:
    s   = sum(payload) & 0xFFFF
    chk = (~s + 1) & 0xFFFF
    return (chk >> 8) & 0xFF, chk & 0xFF

def build_read_std(cmd: int) -> bytes:
    h, l = jbd_checksum(bytes([cmd, 0x00]))
    return bytes([0xDD, 0xA5, cmd, 0x00, h, l, 0x77])

def build_mos_cmd(xx: int) -> bytes:
    data = bytes([0x00, xx])
    h, l = jbd_checksum(bytes([0xE1, 0x02]) + data)
    return bytes([0xDD, 0x5A, 0xE1, 0x02]) + data + bytes([h, l, 0x77])

# ── CRC16 Modbus (protocol 0x78) ──────────────────────────────────────────────

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

# ── Receive ───────────────────────────────────────────────────────────────────

def recv_raw(ser, timeout: float = 3.0) -> bytes:
    buf = bytearray(); start = time.time()
    while time.time() - start < timeout:
        chunk = ser.read(256)
        if chunk: buf.extend(chunk); start = time.time()
        elif buf: break
        time.sleep(0.02)
    return bytes(buf)

def recv_std_response(ser, expected_cmd: int, timeout: float = 3.0) -> tuple:
    buf = bytearray(); start = time.time()
    while time.time() - start < timeout:
        chunk = ser.read(256)
        if chunk: buf.extend(chunk)
        while len(buf) > 0 and buf[0] != 0xDD: buf = buf[1:]
        if len(buf) < 4: time.sleep(0.01); continue
        cmd = buf[1]; status = buf[2]; length = buf[3]
        total = 4 + length + 2 + 1
        if len(buf) < total: time.sleep(0.01); continue
        if buf[total-1] != 0x77: buf = buf[1:]; continue
        frame = bytes(buf[:total]); buf = buf[total:]
        if cmd != expected_cmd: continue
        if status != 0x00: return None, f"BMS_ERROR 0x{status:02X}"
        data = frame[4:4+length]
        chk_h = frame[4+length]; chk_l = frame[5+length]
        exp_h, exp_l = jbd_checksum(bytes([cmd, length]) + data)
        ok_str = "OK" if (exp_h==chk_h and exp_l==chk_l) else f"CHK_MISMATCH(exp={exp_h:02X}{exp_l:02X} got={chk_h:02X}{chk_l:02X})"
        return data, ok_str
    return None, "TIMEOUT"

# ── Parse 0x03 ────────────────────────────────────────────────────────────────

PROT_BITS = {
    0:"Cell overvoltage", 1:"Cell undervoltage", 2:"Pack overvoltage",
    3:"Pack undervoltage", 4:"Chg overtemp", 5:"Chg undertemp",
    6:"Dchg overtemp", 7:"Dchg undertemp", 8:"Chg overcurrent",
    9:"Dchg overcurrent", 10:"Short circuit", 11:"Frontend IC error",
    12:"Software lock MOS",
}

def parse_cmd03(data: bytes) -> dict:
    if len(data) < 23: return {"error": f"too short: {len(data)}"}
    r = {}
    r['voltage_v']  = round(struct.unpack('>H', data[0:2])[0] / 100.0, 2)
    current_raw     = struct.unpack('>h', data[2:4])[0]          # SIGNED int16
    r['current_a']  = round(current_raw / 100.0, 2)
    r['current_raw_signed'] = current_raw
    r['current_raw_hex']    = f"0x{struct.unpack('>H', data[2:4])[0]:04X}"
    r['current_dir']        = "charging(+)" if current_raw >= 0 else "discharging(-)"
    r['cap_remaining_ah']   = round(struct.unpack('>H', data[4:6])[0] * 10 / 1000, 2)
    r['cap_nominal_ah']     = round(struct.unpack('>H', data[6:8])[0] * 10 / 1000, 2)
    r['cycles']             = struct.unpack('>H', data[8:10])[0]
    date_raw = struct.unpack('>H', data[10:12])[0]
    try: r['production_date'] = f"{2000+(date_raw>>9):04d}-{(date_raw>>5)&0x0F:02d}-{date_raw&0x1F:02d}"
    except: r['production_date'] = f"raw=0x{date_raw:04X}"
    r['balance_low']  = f"0x{struct.unpack('>H', data[12:14])[0]:04X}"
    r['balance_high'] = f"0x{struct.unpack('>H', data[14:16])[0]:04X}"
    prot = struct.unpack('>H', data[16:18])[0]
    r['protection_raw']    = f"0x{prot:04X}"
    r['protection_active'] = [desc for bit, desc in PROT_BITS.items() if prot & (1 << bit)] or ["none"]
    sw = data[18]
    r['software_version']  = f"V{sw >> 4}.{sw & 0x0F}"
    r['soc_pct']           = data[19]
    fet = data[20]
    r['fet_raw']      = f"0x{fet:02X}"
    r['charge_mos']   = "ON" if (fet & 0x01) else "OFF"
    r['discharge_mos']= "ON" if (fet & 0x02) else "OFF"
    r['num_cells']    = data[21]
    num_ntc = data[22]; r['num_ntc'] = num_ntc
    temps = []
    for i in range(num_ntc):
        off = 23 + i * 2
        if off + 2 <= len(data):
            raw_t = struct.unpack('>H', data[off:off+2])[0]
            temps.append({'raw': raw_t, 'raw_hex': f"0x{raw_t:04X}", 'celsius': round((raw_t-2731)/10.0, 1)})
    r['temperatures'] = temps
    r['power_w'] = round(r['voltage_v'] * r['current_a'], 1)
    return r

def parse_cmd04(data: bytes) -> dict:
    if len(data) < 2 or len(data) % 2 != 0: return {"error": f"invalid len={len(data)}"}
    n = len(data) // 2
    cells = [struct.unpack('>H', data[i*2:i*2+2])[0] for i in range(n)]
    return {'num_cells': n, 'cells_mv': cells,
            'min_mv': min(cells), 'max_mv': max(cells),
            'delta_mv': max(cells)-min(cells), 'avg_mv': round(sum(cells)/n, 1)}

def parse_cmd05(data: bytes) -> dict:
    try: ver = data.decode('ascii', errors='replace').strip()
    except: ver = data.hex()
    return {'hardware_version': ver, 'raw_hex': data.hex()}

# ── Scan porturi ─────────────────────────────────────────────────────────────

def scan_ports() -> list:
    ports = []
    by_id = "/dev/serial/by-id/"
    if os.path.isdir(by_id):
        for link in sorted(glob.glob(by_id + "*")):
            try:
                real = os.path.realpath(link)
                p(f"  by-id: {os.path.basename(link)} -> {real}")
                ports.append(real)
            except: pass
    for dev in sorted(glob.glob("/dev/ttyUSB*")):
        if dev not in ports: p(f"  ttyUSB: {dev}"); ports.append(dev)
    for pi in serial.tools.list_ports.comports():
        if pi.device not in ports: p(f"  serial: {pi.device} [{pi.description}]"); ports.append(pi.device)
    return list(dict.fromkeys(ports))

# ── Protocol A: DD/A5/77 ─────────────────────────────────────────────────────

def scan_protocol_a(ser, baud: int) -> dict:
    p(); p("="*65)
    p(f"  PROTOCOL A: DD/A5/77 Standard @ {baud} bps")
    p("="*65)
    results = {}

    for cmd, name in [(0x03, "Basic Info"), (0x04, "Cell Voltages"), (0x05, "HW Version")]:
        req = build_read_std(cmd)
        p(); p(f"  [A-{cmd:02X}] READ 0x{cmd:02X} - {name}")
        p(f"  TX: {req.hex(' ').upper()}")
        ser.reset_input_buffer(); ser.write(req); time.sleep(0.2)
        data, status = recv_std_response(ser, cmd)
        if data is not None:
            p(f"  RX ({len(data)} bytes): {data.hex(' ').upper()}")
            p(f"  Checksum: {status}")
            if cmd == 0x03:
                parsed = parse_cmd03(data)
                results['cmd03'] = {'status': status, 'raw': data.hex(), 'parsed': parsed}
                p(); p("  === PARSED 0x03 ===")
                p(f"  Tensiune:          {parsed.get('voltage_v')} V")
                p(f"  Curent (SIGNED):   {parsed.get('current_a')} A  ({parsed.get('current_dir')})")
                p(f"    raw hex:         {parsed.get('current_raw_hex')} = {parsed.get('current_raw_signed')} (signed)")
                p(f"  Putere:            {parsed.get('power_w')} W")
                p(f"  SoC:               {parsed.get('soc_pct')} %")
                p(f"  Cap. ramasa:       {parsed.get('cap_remaining_ah')} Ah")
                p(f"  Cap. nominala:     {parsed.get('cap_nominal_ah')} Ah")
                p(f"  Cicluri:           {parsed.get('cycles')}")
                p(f"  Data productie:    {parsed.get('production_date')}")
                p(f"  Balance:           low={parsed.get('balance_low')} high={parsed.get('balance_high')}")
                p(f"  Protectie raw:     {parsed.get('protection_raw')}")
                p(f"  Protectii active:  {', '.join(parsed.get('protection_active', []))}")
                p(f"  Versiune SW:       {parsed.get('software_version')}")
                p(f"  FET:               raw={parsed.get('fet_raw')} CHG={parsed.get('charge_mos')} DSG={parsed.get('discharge_mos')}")
                p(f"  Celule:            {parsed.get('num_cells')} strings, {parsed.get('num_ntc')} NTC")
                for i, t in enumerate(parsed.get('temperatures', [])):
                    p(f"  Temp T{i+1}:          {t['celsius']} C  (raw={t['raw']} = {t['raw_hex']})")
            elif cmd == 0x04:
                parsed = parse_cmd04(data)
                results['cmd04'] = {'status': status, 'raw': data.hex(), 'parsed': parsed}
                p(); p("  === PARSED 0x04 ===")
                p(f"  Numar celule: {parsed.get('num_cells')}")
                p(f"  Min: {parsed.get('min_mv')} mV  Max: {parsed.get('max_mv')} mV  Delta: {parsed.get('delta_mv')} mV  Avg: {parsed.get('avg_mv')} mV")
                for i, v in enumerate(parsed.get('cells_mv', [])):
                    p(f"  C{i+1:02d}: {v} mV")
            elif cmd == 0x05:
                parsed = parse_cmd05(data)
                results['cmd05'] = {'status': status, 'raw': data.hex(), 'parsed': parsed}
                p(); p("  === PARSED 0x05 ===")
                p(f"  Hardware version: '{parsed.get('hardware_version')}'")
        else:
            p(f"  ESUAT: {status}")
            results[f'cmd{cmd:02x}'] = {'status': status, 'error': True}
        time.sleep(0.3)

    # MOS frames (fara aplicare)
    p(); p("  [A-MOS] Frame MOS (FARA APLICARE - doar verificare constructie):")
    for xx, desc in [(0x00,"release all"),(0x01,"CHG off"),(0x02,"DSG off"),(0x03,"both off")]:
        p(f"  XX=0x{xx:02X} ({desc:12s}): {build_mos_cmd(xx).hex(' ').upper()}")

    ok = sum(1 for r in results.values() if not r.get('error'))
    p(); p(f"  === SUMAR A: {ok}/{len(results)} comenzi reusita ===")
    if results.get('cmd03') and not results['cmd03'].get('error'):
        pr = results['cmd03']['parsed']
        p(f"  Curent corect: {pr.get('current_a')} A  Putere: {pr.get('power_w')} W")
        p(f"  Protocol A {'FUNCTIONAL ✓' if ok >= 2 else 'PARTIAL'}")
    else:
        p("  Protocol A NEFUNCTIONAL pe acest port/baud")
    return results

# ── Protocol B: 0x78 Register ────────────────────────────────────────────────

def scan_protocol_b(ser, baud: int) -> dict:
    p(); p("="*65)
    p(f"  PROTOCOL B: 0x78 Register (addon v6.x) @ {baud} bps")
    p("="*65)
    req = build_request_78(0x01, 0x1000, 0x10A0)
    p(f"  TX: {req.hex(' ').upper()}")
    ser.reset_input_buffer(); ser.write(req); time.sleep(0.5)
    raw = recv_raw(ser, timeout=4.0)
    results = {}

    if raw:
        p(f"  RX ({len(raw)} bytes):")
        for i in range(0, len(raw), 16):
            ch = raw[i:i+16]
            p(f"    {i:04X}: {' '.join(f'{b:02X}' for b in ch):<48}")
        if len(raw) >= 10 and raw[0] == 0x01 and raw[1] == 0x78:
            data_len = struct.unpack('>H', raw[6:8])[0]
            payload  = raw[8:8+data_len]
            p(f"  Header OK: data_len={data_len}")
            if len(payload) >= 8:
                v_raw   = struct.unpack('>H', payload[0:2])[0]
                nc_raw  = struct.unpack('>H', payload[6:8])[0]
                i_raw_s = struct.unpack('>h', payload[2:4])[0]
                p(); p("  === ANALIZA COMPARATIVA v6.x vs v7.0 ===")
                p(f"  bytes[0:2]=0x{v_raw:04X} -> tensiune: {v_raw/100.0:.2f} V")
                p(f"  bytes[2:4]=0x{struct.unpack('>H', payload[2:4])[0]:04X} -> curent CORECT (signed): {i_raw_s/100.0:.2f} A")
                p(f"  bytes[6:8]=0x{nc_raw:04X} -> cap_nominala: {nc_raw*10/1000:.2f} Ah (NU curent!)")
                p(f"  v6.x formula gresita: ({nc_raw}-37403)/114.2 = {(nc_raw-37403)/114.2:.2f} A")
                p(f"  v7.0 formula corecta: {i_raw_s}/100 = {i_raw_s/100.0:.2f} A")
                results['0x78'] = {'ok': True,
                    'voltage_v': v_raw/100.0,
                    'current_wrong_v6': (nc_raw-37403)/114.2,
                    'current_correct': i_raw_s/100.0}
        else:
            p("  Header invalid"); results['0x78'] = {'ok': False}
    else:
        p("  TIMEOUT"); results['0x78'] = {'ok': False, 'error': 'timeout'}

    p(); p(f"  === SUMAR B: Protocol 0x78 {'FUNCTIONAL' if results.get('0x78',{}).get('ok') else 'NEFUNCTIONAL'} ===")
    return results

# ── Verificare exemple PDF ────────────────────────────────────────────────────

def verify_pdf_examples():
    p(); p("="*65); p("  VERIFICARE CHECKSUM - Exemple din protocol PDF"); p("="*65)
    examples = [
        ("READ 0x03", bytes([0xDD,0xA5,0x03,0x00,0xFF,0xFD,0x77]), build_read_std(0x03)),
        ("READ 0x04", bytes([0xDD,0xA5,0x04,0x00,0xFF,0xFC,0x77]), build_read_std(0x04)),
        ("READ 0x05", bytes([0xDD,0xA5,0x05,0x00,0xFF,0xFB,0x77]), build_read_std(0x05)),
        ("MOS XX=02 (DSG off)", bytes([0xDD,0x5A,0xE1,0x02,0x00,0x02,0xFF,0x1B,0x77]), build_mos_cmd(0x02)),
    ]
    for name, expected, built in examples:
        match = built == expected
        p(f"  {name}:")
        p(f"    Asteptat: {expected.hex(' ').upper()}")
        p(f"    Calculat: {built.hex(' ').upper()}")
        p(f"    Status:   {'OK' if match else 'MISMATCH'}")

# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    setup_logger()
    parser = argparse.ArgumentParser(description="JBD BMS Protocol Scanner")
    parser.add_argument("port",    nargs="?", default=None)
    parser.add_argument("--baud",  type=int,  default=None, help="Baud rate (implicit: testeaza 9600 si 19200)")
    parser.add_argument("--addr",  type=lambda x: int(x,0), default=1)
    parser.add_argument("--proto", choices=["a","b","all"], default="all")
    parser.add_argument("--scan",  action="store_true", help="Listeaza porturi disponibile")
    args = parser.parse_args()

    p("="*65); p("  JBD BMS Protocol Scanner")
    p(f"  Data: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    p(f"  Log:  {LOG_FILE}"); p("="*65)

    verify_pdf_examples()

    if args.scan:
        p(); p("  Porturi seriale disponibile:")
        ports = scan_ports()
        if not ports: p("  (niciun port gasit)")
        p(); p("  Exemplu: python3 jbd_scan.py /dev/ttyUSB0")
        return

    if not args.port:
        p(); p("  EROARE: Specifica portul serial!"); p("  Porneste cu --scan pentru porturi disponibile.")
        sys.exit(1)

    bauds = [args.baud] if args.baud else [9600, 19200]
    all_results = {}

    for baud in bauds:
        p(); p("="*65); p(f"  PORT: {args.port} @ {baud} bps"); p("="*65)
        try:
            ser = serial.Serial(port=args.port, baudrate=baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.1)
            p(f"  Serial OK")
        except Exception as e:
            p(f"  EROARE: {e}"); continue

        time.sleep(0.3); res = {}
        if args.proto in ("a","all"): res['proto_a'] = scan_protocol_a(ser, baud)
        time.sleep(0.5)
        if args.proto in ("b","all"): res['proto_b'] = scan_protocol_b(ser, baud)
        all_results[baud] = res; ser.close(); time.sleep(0.5)

    p(); p("="*65); p("  SUMAR FINAL SI RECOMANDARI"); p("="*65)
    best_baud = None
    for baud, res in all_results.items():
        pa = res.get('proto_a', {})
        pb = res.get('proto_b', {})
        a03 = not pa.get('cmd03', {}).get('error', True) and pa.get('cmd03', {}).get('status','') not in ('','TIMEOUT')
        a04 = not pa.get('cmd04', {}).get('error', True) and pa.get('cmd04', {}).get('status','') not in ('','TIMEOUT')
        b78 = pb.get('0x78', {}).get('ok', False)
        p(f"  @ {baud} bps:  Proto A: 0x03={'OK' if a03 else 'FAIL'} 0x04={'OK' if a04 else 'FAIL'}  |  Proto B 0x78: {'OK' if b78 else 'FAIL'}")
        if a03 and a04 and best_baud is None: best_baud = baud

    p()
    if best_baud:
        p(f"  RECOMANDAT: Protocol A (DD/A5/77) @ {best_baud} bps")
        p(f"  Config addon v7.0.0: baud_rate: {best_baud}")
    else:
        p("  ATENTIE: Niciun protocol nu a raspuns complet!")
        p("  Verifica portul serial, cablul, adresa Modbus (--addr 0)")

    p(); p(f"  Log: {LOG_FILE}"); p("="*65)

if __name__ == "__main__": main()
