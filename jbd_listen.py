#!/usr/bin/env python3
"""
jbd_listen.py v2.0 - Passive JBD BMS Serial Listener (NO TX)
=============================================================
Asculta pasiv tot ce trimite BMS-ul pe port, FARA a trimite nimic.

Analiza log 2026-05-10 (ttyUSB1 @ 19200 bps):
  - Protocol DD/77: ABSENT
  - Frame detectat: 10 bytes, repetat la ~0.6s, counter B0 ciclează 0x02-0x0F
  - Structura: [B0] [0x45] [00 00] [00 54] [00 00] [CRC_L] [CRC_H]
  - Ipoteza: Modbus RTU cu FC=0x45 (user-defined, range 65-72)
             sau protocol proprietar invertor-BMS pe magistrala RS485
  - Ultima pereche de bytes = CRC16 Modbus (validat de script)

Ce face v2.0:
  - Detecteaza frame-uri DD/77 si 0x78 (ca v1.0)
  - ADAUGAT: detecteaza si valideaza frame-uri de 10 bytes (pattern FC45)
  - ADAUGAT: verifica CRC16 Modbus al frame-urilor 10-byte
  - ADAUGAT: statistici pe frame-urile necunoscute (counter, data bytes)
  - ADAUGAT: sumar final cu interpretarea protocolului detectat

Rulare:
  pip install pyserial
  python3 jbd_listen.py /dev/ttyUSB1 --baud 19200 --duration 60
  python3 jbd_listen.py /dev/ttyUSB0 --baud 9600
  python3 jbd_listen.py --scan

Autor: Smart-LK / Claude Sonnet, mai 2026
"""

import argparse
import glob
import logging
import os
import struct
import sys
import time
from collections import defaultdict
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("EROARE: pip install pyserial")
    sys.exit(1)

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "jbd_listen.log")


def setup_logger():
    fmt = logging.Formatter("%(asctime)s %(message)s", datefmt="%H:%M:%S.%f")
    log = logging.getLogger("listen")
    log.setLevel(logging.DEBUG)
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(fmt)
    log.addHandler(ch)
    fh = logging.FileHandler(LOG_FILE, mode="w", encoding="utf-8")
    fh.setFormatter(fmt)
    log.addHandler(fh)
    return log


log = logging.getLogger("listen")
p = lambda msg="": log.info(msg)

# =============================================================================
# CRC16 MODBUS RTU
# =============================================================================

def crc16_modbus(data: bytes) -> int:
    """CRC16 Modbus RTU: poly=0xA001, init=0xFFFF, little-endian output."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc  # low byte first in frame: frame[-2]=crc&0xFF, frame[-1]=(crc>>8)&0xFF


# =============================================================================
# CHECKSUM JBD DD/A5/77
# =============================================================================

def jbd_checksum(payload: bytes) -> tuple:
    s = sum(payload) & 0xFFFF
    chk = (~s + 1) & 0xFFFF
    return (chk >> 8) & 0xFF, chk & 0xFF


# =============================================================================
# PARSE DD/77 FRAMES
# =============================================================================

FAULT_BITS = {
    0x0001: "cell_overvoltage",   0x0002: "cell_undervoltage",
    0x0004: "pack_overvoltage",   0x0008: "pack_undervoltage",
    0x0010: "chg_overtemp",       0x0020: "chg_undertemp",
    0x0040: "dchg_overtemp",      0x0080: "dchg_undertemp",
    0x0100: "chg_overcurrent",    0x0200: "dchg_overcurrent",
    0x0400: "short_circuit",      0x0800: "frontend_ic_error",
    0x1000: "sw_lock_mos",
}


def parse_status_0x03(data: bytes) -> dict:
    """
    Parse DD 0x03 response payload.
    Layout:
      [0:2]  voltage            uint16  10mV
      [2:4]  current            int16   10mA  (SIGNED: +=charge, -=discharge)
      [4:6]  remainingCapacity  uint16  10mAh
      [6:8]  nominalCapacity    uint16  10mAh
      [8:10] cycles             uint16
      [10:12] productionDate    uint16
      [12:14] balanceLow        uint16
      [14:16] balanceHigh       uint16
      [16:18] fault             uint16
      [18]   version            uint8
      [19]   currentCapacity    uint8   %
      [20]   mosfetStatus       uint8   (bit0=CHG ON, bit1=DSG ON)
      [21]   cells              uint8
      [22]   ntcs               uint8
      [23+]  temperatures       N*2 bytes, 0.1K absolut -> (raw-2731)/10 = Celsius
    """
    if len(data) < 23:
        return {"error": f"prea scurt: {len(data)} < 23"}
    r = {}
    r['voltage_v']        = round(struct.unpack('>H', data[0:2])[0] / 100.0, 2)
    current_raw           = struct.unpack('>h', data[2:4])[0]   # SIGNED int16
    r['current_a']        = round(current_raw / 100.0, 2)
    r['current_raw']      = current_raw
    r['current_hex']      = f"0x{struct.unpack('>H', data[2:4])[0]:04X}"
    r['current_dir']      = ("CHARGING(+)" if current_raw > 0
                             else "DISCHARGING(-)" if current_raw < 0 else "IDLE(0)")
    r['cap_remaining_ah'] = round(struct.unpack('>H', data[4:6])[0] * 10 / 1000, 2)
    r['cap_nominal_ah']   = round(struct.unpack('>H', data[6:8])[0] * 10 / 1000, 2)
    r['cycles']           = struct.unpack('>H', data[8:10])[0]
    date_raw = struct.unpack('>H', data[10:12])[0]
    try:
        r['production_date'] = f"{2000+(date_raw>>9):04d}-{(date_raw>>5)&0x0F:02d}-{date_raw&0x1F:02d}"
    except Exception:
        r['production_date'] = f"raw=0x{date_raw:04X}"
    fault = struct.unpack('>H', data[16:18])[0]
    r['fault_raw']    = f"0x{fault:04X}"
    r['fault_active'] = [desc for mask, desc in FAULT_BITS.items() if fault & mask] or ["none"]
    sw = data[18]
    r['sw_version']    = f"V{sw >> 4}.{sw & 0x0F}"
    r['soc_pct']       = data[19]
    fet = data[20]
    r['charge_mos']    = "ON" if (fet & 0x01) else "OFF"
    r['discharge_mos'] = "ON" if (fet & 0x02) else "OFF"
    r['num_cells']     = data[21]
    num_ntc            = data[22]
    r['num_ntc']       = num_ntc
    temps = []
    for i in range(num_ntc):
        off = 23 + i * 2
        if off + 2 <= len(data):
            raw_t = struct.unpack('>H', data[off:off+2])[0]
            temps.append(round((raw_t - 2731) / 10.0, 1))
    r['temperatures'] = temps
    r['power_w'] = round(r['voltage_v'] * r['current_a'], 1)
    return r


def parse_cells_0x04(data: bytes) -> dict:
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
    try:
        hw = data.decode('ascii', errors='replace').strip()
    except Exception:
        hw = data.hex()
    return {'hardware_id': hw}


# =============================================================================
# FRAME EXTRACTORS
# =============================================================================

def extract_dd_frames(buf: bytearray) -> tuple:
    """Cauta frame-uri DD...77 (standard si adresabil)."""
    frames = []
    i = 0
    while i < len(buf):
        if buf[i] != 0xDD:
            i += 1
            continue
        found = False
        for offset in [0, 1]:
            hdr     = i + 1 + offset
            len_pos = hdr + 2
            if len_pos >= len(buf):
                break
            dlen  = buf[len_pos]
            total = 1 + offset + 1 + 1 + 1 + dlen + 2 + 1
            end   = i + total
            if end > len(buf):
                break
            if buf[end - 1] != 0x77:
                continue
            frame      = bytes(buf[i:end])
            cmd        = frame[1 + offset]
            status     = frame[2 + offset]
            length     = frame[3 + offset]
            data_start = 1 + offset + 3
            data       = frame[data_start:data_start + length]
            chk_h      = frame[data_start + length]
            chk_l      = frame[data_start + length + 1]
            exp_h, exp_l = jbd_checksum(bytes([cmd, length]) + data)
            chk_ok = (exp_h == chk_h and exp_l == chk_l)
            frames.append({
                'raw': frame, 'cmd': cmd, 'status': status, 'data': data,
                'chk_ok': chk_ok,
                'format': 'adresabil' if offset == 1 else 'standard',
                'addr': buf[i + 1] if offset == 1 else None,
            })
            buf   = buf[end:]
            i     = 0
            found = True
            break
        if not found:
            i += 1
    return frames, buf


def extract_78_frames(buf: bytearray) -> tuple:
    """Cauta frame-uri protocol 0x78."""
    frames = []
    i = 0
    while i < len(buf) - 1:
        if buf[i + 1] != 0x78:
            i += 1
            continue
        if len(buf) - i < 10:
            break
        data_len = struct.unpack('>H', buf[i+6:i+8])[0]
        if data_len > 512:
            i += 1
            continue
        total = 10 + data_len
        if i + total > len(buf):
            break
        frame = bytes(buf[i:i+total])
        frames.append({'raw': frame, 'addr': frame[0], 'data_len': data_len, 'data': frame[8:-2]})
        buf = buf[i+total:]
        i   = 0
    return frames, buf


def extract_10byte_frames(buf: bytearray) -> tuple:
    """
    Detecteaza frame-uri de 10 bytes identificate in log-ul din 2026-05-10:
      [B0] [0x45] [00] [00] [00] [0x54] [00] [00] [CRC_L] [CRC_H]
    unde B0 cicleaza 0x02-0x0F si ultimele 2 bytes sunt CRC16 Modbus al primilor 8 bytes.

    NOTA: Detectia nu impune valori fixe pentru B2-B7 (unele frame-uri pot diferi).
    Detectia se face pe lungime fixa (10 bytes) + validare CRC16.
    """
    frames = []
    i = 0
    while i <= len(buf) - 10:
        # Cauta al doilea byte = 0x45 (marker constant observat)
        if buf[i + 1] != 0x45:
            i += 1
            continue
        frame     = bytes(buf[i:i+10])
        crc_recv  = struct.unpack('<H', frame[-2:])[0]   # little-endian
        crc_calc  = crc16_modbus(frame[:-2])
        crc_ok    = (crc_recv == crc_calc)
        frames.append({
            'raw':      frame,
            'b0':       frame[0],          # counter / slave addr
            'fc':       frame[1],          # 0x45 = FC user-defined
            'data':     frame[2:8],        # 6 bytes payload
            'crc_recv': crc_recv,
            'crc_calc': crc_calc,
            'crc_ok':   crc_ok,
        })
        buf = buf[i+10:]
        i   = 0
    return frames, buf


# =============================================================================
# PRINT HELPERS
# =============================================================================

def hexdump(data: bytes, prefix: str = "  ") -> list:
    lines = []
    for i in range(0, len(data), 16):
        chunk    = data[i:i+16]
        hex_part = ' '.join(f'{b:02X}' for b in chunk)
        asc_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        lines.append(f"{prefix}{i:04X}: {hex_part:<48}  {asc_part}")
    return lines


def print_dd_frame(frm: dict, idx: int):
    raw      = frm['raw']
    cmd      = frm['cmd']
    chk_str  = "CHK OK" if frm['chk_ok'] else "CHK FAIL!"
    addr_str = f" ADDR=0x{frm['addr']:02X}" if frm['addr'] is not None else ""
    p(f"  [{idx:03d}] DD frame CMD=0x{cmd:02X} STATUS=0x{frm['status']:02X} "
      f"LEN={len(frm['data'])} {chk_str} ({frm['format']}{addr_str})")
    p(f"       RAW: {raw.hex(' ').upper()}")

    if not frm['chk_ok']:
        p(f"       ATENTIE: checksum invalid")
        return
    if frm['status'] == 0x80:
        p(f"       BMS ERROR response")
        return

    data = frm['data']
    if cmd == 0x03:
        pr = parse_status_0x03(data)
        if 'error' in pr:
            p(f"       Parse error: {pr['error']}"); return
        p(f"       === STATUS 0x03 ===")
        p(f"       Tensiune:      {pr['voltage_v']} V")
        p(f"       Curent:        {pr['current_a']} A  (raw={pr['current_raw']} = {pr['current_hex']}, {pr['current_dir']})")
        p(f"       Putere:        {pr['power_w']} W")
        p(f"       SoC:           {pr['soc_pct']} %")
        p(f"       Cap. ramasa:   {pr['cap_remaining_ah']} Ah")
        p(f"       Cap. nominala: {pr['cap_nominal_ah']} Ah")
        p(f"       Cicluri:       {pr['cycles']}")
        p(f"       Data prod.:    {pr['production_date']}")
        p(f"       Fault:         {pr['fault_raw']} -> {', '.join(pr['fault_active'])}")
        p(f"       SW version:    {pr['sw_version']}")
        p(f"       CHG MOS:       {pr['charge_mos']}  DSG MOS: {pr['discharge_mos']}")
        p(f"       Celule:        {pr['num_cells']}  NTC: {pr['num_ntc']}")
        for i, t in enumerate(pr['temperatures']):
            p(f"       Temp T{i+1}:       {t} C")
    elif cmd == 0x04:
        pr = parse_cells_0x04(data)
        if 'error' in pr:
            p(f"       Parse error: {pr['error']}"); return
        p(f"       === CELULE 0x04 ({pr['num_cells']} celule) ===")
        p(f"       Min: {pr['min_mv']} mV  Max: {pr['max_mv']} mV  "
          f"Delta: {pr['delta_mv']} mV  Avg: {pr['avg_mv']} mV")
        p(f"       Suma: {pr['sum_mv']} mV = {pr['sum_mv']/1000:.3f} V")
        p(f"       {'  '.join(f'C{i+1:02d}:{v}' for i, v in enumerate(pr['cells_mv']))}")
    elif cmd == 0x05:
        pr = parse_hw_0x05(data)
        p(f"       HW ID: '{pr['hardware_id']}'")
    else:
        p(f"       Date CMD 0x{cmd:02X} ({len(data)} bytes):")
        for line in hexdump(data, "       "):
            p(line)


def print_10byte_frame(frm: dict, idx: int, verbose: bool = True):
    crc_str = "CRC16-Modbus OK ✓" if frm['crc_ok'] else f"CRC FAIL (recv=0x{frm['crc_recv']:04X} calc=0x{frm['crc_calc']:04X})"
    if verbose:
        d = frm['data']
        # Interpretari posibile ale celor 6 bytes de payload
        reg_addr = struct.unpack('>H', d[0:2])[0]  # bytes 2-3
        reg_val  = struct.unpack('>H', d[2:4])[0]  # bytes 4-5
        ext_val  = struct.unpack('>H', d[4:6])[0]  # bytes 6-7
        p(f"  [{idx:03d}] Frame 10B FC=0x{frm['fc']:02X} B0=0x{frm['b0']:02X}  {crc_str}")
        p(f"       RAW: {frm['raw'].hex(' ').upper()}")
        p(f"       B0=0x{frm['b0']:02X}({frm['b0']:3d})  FC=0x{frm['fc']:02X}  "
          f"payload: {d.hex(' ').upper()}")
        p(f"       Interpret: RegAddr=0x{reg_addr:04X}({reg_addr})  "
          f"RegVal=0x{reg_val:04X}({reg_val})  Ext=0x{ext_val:04X}({ext_val})")
    else:
        p(f"  [{idx:03d}] Frame 10B B0=0x{frm['b0']:02X}  {crc_str}  "
          f"data={frm['data'].hex(' ').upper()}")


# =============================================================================
# LISTENER PASIV
# =============================================================================

def listen_passive(port: str, baud: int, duration: int):
    p()
    p("=" * 65)
    p(f"  ASCULTARE PASIVA: {port} @ {baud} bps | {duration}s")
    p(f"  Nu se trimite NIMIC pe port!")
    p("=" * 65)

    try:
        ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0.1,
        )
        p(f"  Port deschis OK")
    except Exception as e:
        p(f"  EROARE: {e}")
        return {'ok': False, 'error': str(e), 'baud': baud}

    buf               = bytearray()
    total_bytes       = 0
    frame_count_dd    = 0
    frame_count_78    = 0
    frame_count_10b   = 0
    crc_ok_count      = 0
    crc_fail_count    = 0
    b0_seen           = defaultdict(int)   # B0 -> count
    data_seen         = defaultdict(int)   # data_hex -> count
    start             = time.time()
    last_activity     = None
    last_10b_verbose  = 0    # timestamp al ultimei afisari verbose 10-byte
    no_data_warned    = False

    p(f"  Incep ascultarea...")
    p()

    try:
        while time.time() - start < duration:
            elapsed = time.time() - start
            chunk   = ser.read(256)

            if chunk:
                ts            = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                total_bytes  += len(chunk)
                last_activity = time.time()
                no_data_warned = False
                buf.extend(chunk)

                p(f"  [{ts}] RX {len(chunk):3d} bytes (total: {total_bytes}):")
                for line in hexdump(chunk, "    "):
                    p(line)

                # --- DD/77 ---
                dd_frames, buf = extract_dd_frames(buf)
                for frm in dd_frames:
                    frame_count_dd += 1
                    p()
                    p(f"  *** Frame DD/77 #{frame_count_dd} ***")
                    print_dd_frame(frm, frame_count_dd)
                    p()

                # --- 0x78 ---
                frames_78, buf = extract_78_frames(buf)
                for frm in frames_78:
                    frame_count_78 += 1
                    p()
                    p(f"  *** Frame 0x78 #{frame_count_78}: ADDR=0x{frm['addr']:02X} LEN={frm['data_len']} ***")
                    if len(frm['data']) >= 4:
                        v_raw = struct.unpack('>H', frm['data'][0:2])[0]
                        i_raw = struct.unpack('>h', frm['data'][2:4])[0]
                        p(f"       V={v_raw/100:.2f}V  I={i_raw/100:.2f}A (signed)")
                    p()

                # --- 10-byte FC45 (pattern identificat in log 2026-05-10) ---
                frames_10b, buf = extract_10byte_frames(buf)
                for frm in frames_10b:
                    frame_count_10b += 1
                    b0_seen[frm['b0']] += 1
                    data_seen[frm['data'].hex()] += 1
                    if frm['crc_ok']:
                        crc_ok_count += 1
                    else:
                        crc_fail_count += 1

                    # Afisare verbosa doar la prima aparitie a unui B0 nou,
                    # sau daca nu am mai afișat de >5s
                    is_new_b0 = (b0_seen[frm['b0']] == 1)
                    now = time.time()
                    verbose = is_new_b0 or (now - last_10b_verbose > 5.0)
                    if verbose:
                        last_10b_verbose = now
                        p()
                        print_10byte_frame(frm, frame_count_10b, verbose=True)
                    # Altfel: un singur rând compact
                    # (nu spam pentru frame-uri repetate)

            else:
                if total_bytes == 0 and elapsed > 10 and not no_data_warned:
                    p(f"  [!] 10s, niciun byte. Verifica port/baud/cablu.")
                    no_data_warned = True
                elif total_bytes > 0 and last_activity and time.time() - last_activity > 5:
                    p(f"  [i] {time.time()-last_activity:.0f}s de la ultimul byte...")
                    last_activity = time.time()

            if len(buf) > 256:
                p(f"  [!] Buffer nedecodat: {len(buf)} bytes. Primii 64:")
                for line in hexdump(bytes(buf[:64]), "    "):
                    p(line)
                buf = buf[-128:]

    except KeyboardInterrupt:
        p()
        p("  Intrerupt (Ctrl+C)")
    finally:
        ser.close()

    elapsed_total = time.time() - start

    p()
    p("=" * 65)
    p(f"  SUMAR @ {baud} bps ({elapsed_total:.1f}s):")
    p(f"  Total bytes RX:    {total_bytes}  ({total_bytes/elapsed_total:.1f} bytes/s)")
    p(f"  Frame-uri DD/77:   {frame_count_dd}")
    p(f"  Frame-uri 0x78:    {frame_count_78}")
    p(f"  Frame-uri 10-byte: {frame_count_10b}")
    if frame_count_10b > 0:
        p(f"    CRC16 Modbus OK:  {crc_ok_count} / {frame_count_10b}")
        p(f"    CRC16 Modbus FAIL:{crc_fail_count} / {frame_count_10b}")
        p(f"    Valori B0 vazute: {sorted(b0_seen.keys())} "
          f"= [{', '.join(f'0x{b:02X}' for b in sorted(b0_seen.keys()))}]")
        p(f"    Variante payload (hex):")
        for dh, cnt in sorted(data_seen.items(), key=lambda x: -x[1])[:10]:
            # Interpreta payload-ul ca register addr + val + ext
            dbytes   = bytes.fromhex(dh)
            reg_addr = struct.unpack('>H', dbytes[0:2])[0]
            reg_val  = struct.unpack('>H', dbytes[2:4])[0]
            ext_val  = struct.unpack('>H', dbytes[4:6])[0]
            p(f"      {dh}  x{cnt:3d}  RegAddr={reg_addr:5d}  RegVal={reg_val:5d}  Ext={ext_val:5d}")

        # Concluzie protocol
        p()
        if crc_ok_count == frame_count_10b:
            p(f"  >>> CONFIRMAT: Frame-urile 10-byte sunt Modbus RTU valide!")
            p(f"  >>> FC=0x45 (user-defined 69) pe magistrala RS485 invertor-BMS")
            p(f"  >>> Acesta NU este protocolul DD/77 al BMS-ului JBD.")
            p(f"  >>> Pentru date BMS foloseste portul UART/BT cu protocolul DD.")
        elif crc_ok_count > frame_count_10b // 2:
            p(f"  >>> PROBABIL Modbus RTU: {crc_ok_count}/{frame_count_10b} CRC OK")
            p(f"  >>> Frame-urile cu CRC FAIL pot fi date de aliniere gresita.")
        else:
            p(f"  >>> ATENTIE: CRC16 Modbus NU se potriveste pe majoritatea frame-urilor.")
            p(f"  >>> Protocol necunoscut sau aliniere frame gresita.")

    if buf:
        p(f"  Buffer ramas: {len(buf)} bytes: {bytes(buf[:64]).hex(' ').upper()}")
    p("=" * 65)

    return {
        'ok': total_bytes > 0,
        'total_bytes': total_bytes,
        'frame_count_dd': frame_count_dd,
        'frame_count_78': frame_count_78,
        'frame_count_10b': frame_count_10b,
        'crc_ok_count': crc_ok_count,
        'crc_fail_count': crc_fail_count,
        'b0_seen': dict(b0_seen),
        'baud': baud,
    }


# =============================================================================
# PORT SCANNER
# =============================================================================

def scan_ports():
    p("  Porturi seriale disponibile:")
    ports = []
    by_id = "/dev/serial/by-id/"
    if os.path.isdir(by_id):
        for link in sorted(glob.glob(by_id + "*")):
            try:
                real = os.path.realpath(link)
                p(f"  by-id: {os.path.basename(link)} -> {real}")
                ports.append(real)
            except Exception:
                pass
    for dev in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyS*") + glob.glob("/dev/rfcomm*")):
        if dev not in ports:
            p(f"  {dev}")
            ports.append(dev)
    for pi in serial.tools.list_ports.comports():
        if pi.device not in ports:
            p(f"  {pi.device} [{pi.description}]")
            ports.append(pi.device)
    if not ports:
        p("  (niciun port gasit)")
    return ports


# =============================================================================
# MAIN
# =============================================================================

def main():
    setup_logger()
    parser = argparse.ArgumentParser(description="JBD BMS Passive Listener v2.0")
    parser.add_argument("port",       nargs="?", default=None)
    parser.add_argument("--baud",     type=int,  default=None,
                        help="Baud rate (implicit: testeaza 9600 si 19200)")
    parser.add_argument("--duration", type=int,  default=60,
                        help="Durata ascultare in secunde (implicit: 60)")
    parser.add_argument("--scan",     action="store_true")
    args = parser.parse_args()

    p("=" * 65)
    p("  JBD BMS Passive Listener v2.0")
    p(f"  Data: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    p(f"  Log:  {LOG_FILE}")
    p("=" * 65)
    p("  MOD PASIV - nu se trimite NIMIC pe port serial!")
    p("  Detectie: DD/77 + 0x78 + 10-byte FC45 (Modbus CRC16 validat)")
    p()

    if args.scan:
        scan_ports()
        return

    if not args.port:
        p("  EROARE: Specifica portul sau --scan")
        p("  Exemplu (portul cu broadcast): python3 jbd_listen.py /dev/ttyUSB1 --baud 19200")
        p("  Exemplu (portul BT/DD):        python3 jbd_listen.py /dev/ttyUSB0 --baud 9600")
        sys.exit(1)

    bauds   = [args.baud] if args.baud else [9600, 19200]
    results = []

    for baud in bauds:
        r = listen_passive(args.port, baud, args.duration)
        results.append(r)
        # Opreste la primul baud unde gasim frame-uri valide
        if r.get('ok') and (r.get('frame_count_dd', 0) + r.get('frame_count_10b', 0) > 0):
            p(f"  Frame-uri detectate la {baud} bps.")
            break

    p()
    p("=" * 65)
    p("  SUMAR FINAL")
    p("=" * 65)
    for r in results:
        if not r.get('ok'):
            p(f"  @ {r.get('baud')} bps: FARA DATE")
        else:
            p(f"  @ {r['baud']} bps: {r['total_bytes']} bytes | "
              f"DD={r.get('frame_count_dd',0)} | "
              f"0x78={r.get('frame_count_78',0)} | "
              f"10B={r.get('frame_count_10b',0)} "
              f"(CRC OK={r.get('crc_ok_count',0)})")
    p(f"  Log: {LOG_FILE}")
    p("=" * 65)


if __name__ == "__main__":
    main()
