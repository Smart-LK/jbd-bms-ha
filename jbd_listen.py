#!/usr/bin/env python3
"""
jbd_listen.py v1.0 - Passive JBD BMS Serial Listener (NO TX)
=============================================================
Ascultă pasiv tot ce trimite BMS-ul pe port, FARA a trimite nimic.
Folosit pentru:
  - port cu broadcast spontan (portul USB original)
  - port UART/Bluetooth unde BMS trimite date periodic

Ce face:
  - Deschide portul in mod READ-ONLY (nu trimite nimic)
  - Inregistreaza TOATE datele brute cu timestamp
  - Detecteaza automat frame-uri DD/77 si 0x78
  - Parseaza frame-urile DD 0x03 (status) si 0x04 (celule)
  - Afiseaza hex dump + decoded in timp real
  - Salveaza log complet in jbd_listen.log

Rulare:
  pip install pyserial
  python3 jbd_listen.py /dev/ttyUSB0
  python3 jbd_listen.py /dev/ttyUSB0 --baud 9600
  python3 jbd_listen.py /dev/ttyUSB0 --baud 19200 --duration 60
  python3 jbd_listen.py --scan            # listeaza porturile disponibile

Baud rate implicit: testeaza 9600 si 19200 (30s fiecare).
Durata implicita: 60s per baud (sau --duration N secunde).

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
# CHECKSUM JBD DD/A5/77
# =============================================================================

def jbd_checksum(payload: bytes) -> tuple:
    """Checksum JBD: -sum(payload) in 16-bit two's complement."""
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
    Parse DD 0x03 response payload (Status_t din jbdbms.h).
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
      [23+]  temperatures       N*2 bytes, unit 0.1K absolut
             Formula: celsius = (raw - 2731) / 10.0
    """
    if len(data) < 23:
        return {"error": f"prea scurt: {len(data)} < 23"}

    r = {}
    r['voltage_v']       = round(struct.unpack('>H', data[0:2])[0] / 100.0, 2)

    # Curent SIGNED int16, unitate 10mA
    current_raw          = struct.unpack('>h', data[2:4])[0]  # '>h' = big-endian signed int16
    r['current_a']       = round(current_raw / 100.0, 2)
    r['current_raw']     = current_raw
    r['current_hex']     = f"0x{struct.unpack('>H', data[2:4])[0]:04X}"
    r['current_dir']     = ("CHARGING(+)" if current_raw > 0
                            else "DISCHARGING(-)" if current_raw < 0
                            else "IDLE(0)")

    r['cap_remaining_ah'] = round(struct.unpack('>H', data[4:6])[0] * 10 / 1000, 2)
    r['cap_nominal_ah']   = round(struct.unpack('>H', data[6:8])[0] * 10 / 1000, 2)
    r['cycles']           = struct.unpack('>H', data[8:10])[0]

    date_raw = struct.unpack('>H', data[10:12])[0]
    try:
        r['production_date'] = f"{2000 + (date_raw >> 9):04d}-{(date_raw >> 5) & 0x0F:02d}-{date_raw & 0x1F:02d}"
    except Exception:
        r['production_date'] = f"raw=0x{date_raw:04X}"

    fault = struct.unpack('>H', data[16:18])[0]
    r['fault_raw']    = f"0x{fault:04X}"
    r['fault_active'] = [desc for mask, desc in FAULT_BITS.items() if fault & mask] or ["none"]

    sw = data[18]
    r['sw_version']     = f"V{sw >> 4}.{sw & 0x0F}"
    r['soc_pct']        = data[19]
    fet = data[20]
    r['charge_mos']     = "ON" if (fet & 0x01) else "OFF"
    r['discharge_mos']  = "ON" if (fet & 0x02) else "OFF"
    r['num_cells']      = data[21]
    num_ntc             = data[22]
    r['num_ntc']        = num_ntc

    temps = []
    for i in range(num_ntc):
        off = 23 + i * 2
        if off + 2 <= len(data):
            raw_t = struct.unpack('>H', data[off:off + 2])[0]
            temps.append(round((raw_t - 2731) / 10.0, 1))
    r['temperatures'] = temps
    r['power_w'] = round(r['voltage_v'] * r['current_a'], 1)
    return r


def parse_cells_0x04(data: bytes) -> dict:
    if len(data) < 2 or len(data) % 2 != 0:
        return {"error": f"invalid len={len(data)}"}
    n = len(data) // 2
    cells = [struct.unpack('>H', data[i * 2:i * 2 + 2])[0] for i in range(n)]
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
# FRAME DETECTOR - cauta frame-uri DD/77 intr-un buffer
# =============================================================================

def extract_dd_frames(buf: bytearray) -> tuple:
    """
    Cauta toate frame-urile DD...77 complete din buffer.
    Returneaza (frames, buf_ramas).
    Frame format:
      DD [CMD|ADDR] [STATUS|A5] [LEN|CMD] [DATA] [CHK_H] [CHK_L] 77
    Suporta ambele variante: standard (DD CMD STATUS LEN ...)
                             si adresabil (DD ADDR CMD STATUS LEN ...)
    """
    frames = []
    i = 0
    while i < len(buf):
        # Cauta byte-ul de start DD
        if buf[i] != 0xDD:
            i += 1
            continue

        # Incearca sa identifice lungimea frame-ului
        # Strategia: cauta secventa valida DD + LEN + 77 la pozitia corecta
        found = False
        for offset in [0, 1]:  # 0=standard, 1=adresabil (cu ADDR byte)
            # Standard:   buf[i]=DD, buf[i+1]=CMD,  buf[i+2]=STATUS, buf[i+3]=LEN
            # Adresabil:  buf[i]=DD, buf[i+1]=ADDR, buf[i+2]=CMD,    buf[i+3]=STATUS, buf[i+4]=LEN
            hdr = i + 1 + offset  # pozitia CMD in buffer
            len_pos = hdr + 2     # pozitia LEN in buffer
            if len_pos >= len(buf):
                break
            dlen = buf[len_pos]
            # Frame total: DD [+ADDR] CMD STATUS LEN [DATA*dlen] CHK_H CHK_L 77
            total = 1 + offset + 1 + 1 + 1 + dlen + 2 + 1
            end = i + total
            if end > len(buf):
                break  # frame incomplet, asteptam mai multe date
            if buf[end - 1] != 0x77:
                continue  # nu se termina cu 0x77, nu e frame valid

            frame = bytes(buf[i:end])
            cmd = frame[1 + offset]
            status = frame[2 + offset]
            length = frame[3 + offset]
            data_start = 1 + offset + 3
            data = frame[data_start:data_start + length]
            chk_h = frame[data_start + length]
            chk_l = frame[data_start + length + 1]

            # Verifica checksum (acoperind CMD, LEN, DATA)
            exp_h, exp_l = jbd_checksum(bytes([cmd, length]) + data)
            chk_ok = (exp_h == chk_h and exp_l == chk_l)

            frames.append({
                'raw': frame,
                'cmd': cmd,
                'status': status,
                'data': data,
                'chk_ok': chk_ok,
                'format': 'adresabil' if offset == 1 else 'standard',
                'addr': buf[i + 1] if offset == 1 else None,
            })
            buf = buf[end:]
            i = 0
            found = True
            break

        if not found:
            i += 1

    return frames, buf


def extract_78_frames(buf: bytearray) -> tuple:
    """
    Cauta frame-uri protocol 0x78 in buffer.
    Format: [ADDR] 0x78 [SR_H] [SR_L] [ER_H] [ER_L] [LEN_H] [LEN_L] [DATA...] [CRC_L] [CRC_H]
    """
    frames = []
    i = 0
    while i < len(buf) - 1:
        if buf[i + 1] != 0x78:
            i += 1
            continue
        if len(buf) - i < 10:
            break
        data_len = struct.unpack('>H', buf[i + 6:i + 8])[0]
        if data_len > 512:
            i += 1
            continue
        total = 10 + data_len
        if i + total > len(buf):
            break
        frame = bytes(buf[i:i + total])
        frames.append({
            'raw': frame,
            'addr': frame[0],
            'data_len': data_len,
            'data': frame[8:-2],
        })
        buf = buf[i + total:]
        i = 0
    return frames, buf


# =============================================================================
# PRINT HELPERS
# =============================================================================

def hexdump(data: bytes, prefix: str = "  ") -> list:
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i:i + 16]
        hex_part = ' '.join(f'{b:02X}' for b in chunk)
        asc_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        lines.append(f"{prefix}{i:04X}: {hex_part:<48}  {asc_part}")
    return lines


def print_dd_frame(frm: dict, idx: int):
    raw = frm['raw']
    cmd = frm['cmd']
    chk_str = "CHK OK" if frm['chk_ok'] else "CHK FAIL!"
    fmt = frm['format']
    addr_str = f" ADDR=0x{frm['addr']:02X}" if frm['addr'] is not None else ""
    p(f"  [{idx:03d}] DD frame CMD=0x{cmd:02X} STATUS=0x{frm['status']:02X} "
      f"LEN={len(frm['data'])} {chk_str} ({fmt}{addr_str})")
    p(f"       RAW: {raw.hex(' ').upper()}")

    if not frm['chk_ok']:
        p(f"       ATENTIE: checksum invalid - date corupte sau frame detectat gresit")
        return

    if frm['status'] == 0x80:
        p(f"       BMS ERROR response")
        return

    data = frm['data']
    if cmd == 0x03:
        pr = parse_status_0x03(data)
        if 'error' in pr:
            p(f"       Parse error: {pr['error']}")
            return
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
            p(f"       Parse error: {pr['error']}")
            return
        p(f"       === CELULE 0x04 ({pr['num_cells']} celule) ===")
        p(f"       Min: {pr['min_mv']} mV  Max: {pr['max_mv']} mV  "
          f"Delta: {pr['delta_mv']} mV  Avg: {pr['avg_mv']} mV")
        p(f"       Suma: {pr['sum_mv']} mV = {pr['sum_mv']/1000:.3f} V")
        cell_str = '  '.join(f"C{i+1:02d}:{v}" for i, v in enumerate(pr['cells_mv']))
        p(f"       {cell_str}")

    elif cmd == 0x05:
        pr = parse_hw_0x05(data)
        p(f"       === HW ID 0x05 ===")
        p(f"       Hardware ID: '{pr['hardware_id']}'")

    else:
        p(f"       Date CMD 0x{cmd:02X} ({len(data)} bytes):")
        for line in hexdump(data, "       "):
            p(line)


# =============================================================================
# LISTENER PASIV
# =============================================================================

def listen_passive(port: str, baud: int, duration: int):
    """
    Asculta pasiv portul serial pentru 'duration' secunde.
    Nu trimite NIMIC.
    """
    p()
    p("=" * 65)
    p(f"  ASCULTARE PASIVA: {port} @ {baud} bps | {duration}s")
    p(f"  IMPORTANT: Nu se trimite nimic pe port!")
    p("=" * 65)

    try:
        ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        p(f"  Port deschis OK")
    except Exception as e:
        p(f"  EROARE deschidere port: {e}")
        return {'ok': False, 'error': str(e)}

    buf = bytearray()
    total_bytes = 0
    frame_count = 0
    raw_chunks = []  # lista de (timestamp, bytes)
    start = time.time()
    last_activity = None
    no_data_warned = False

    p(f"  Incep ascultarea...")
    p()

    try:
        while time.time() - start < duration:
            elapsed = time.time() - start
            remaining = duration - elapsed

            chunk = ser.read(256)
            if chunk:
                ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                total_bytes += len(chunk)
                last_activity = time.time()
                no_data_warned = False
                buf.extend(chunk)
                raw_chunks.append((ts, bytes(chunk)))

                # Afiseaza raw hex al chunk-ului
                p(f"  [{ts}] RX {len(chunk):3d} bytes (total: {total_bytes}):")
                for line in hexdump(chunk, "    "):
                    p(line)

                # Incearca sa extraga frame-uri DD/77
                dd_frames, buf = extract_dd_frames(buf)
                for frm in dd_frames:
                    frame_count += 1
                    p()
                    p(f"  *** Frame DD/77 #{frame_count} detectat ***")
                    print_dd_frame(frm, frame_count)
                    p()

                # Incearca sa extraga frame-uri 0x78
                frames_78, buf = extract_78_frames(buf)
                for frm in frames_78:
                    frame_count += 1
                    p()
                    p(f"  *** Frame 0x78 #{frame_count} detectat ***")
                    p(f"       ADDR=0x{frm['addr']:02X} LEN={frm['data_len']}")
                    if frm['data']:
                        v_raw = struct.unpack('>H', frm['data'][0:2])[0] if len(frm['data']) >= 2 else 0
                        i_raw = struct.unpack('>h', frm['data'][2:4])[0] if len(frm['data']) >= 4 else 0
                        p(f"       Tensiune: {v_raw/100:.2f} V  Curent: {i_raw/100:.2f} A (signed)")
                    p()

            else:
                # Nicio data
                if total_bytes == 0 and elapsed > 10 and not no_data_warned:
                    p(f"  [!] 10s scurse, niciun byte primit. Verifica portul/baud/cablul.")
                    no_data_warned = True
                elif total_bytes > 0 and last_activity and time.time() - last_activity > 5:
                    p(f"  [i] {time.time()-last_activity:.0f}s de la ultimul byte...")
                    last_activity = time.time()  # reset ca sa nu spameze

            # Daca buf-ul ramas e prea mare si nu a gasit frame-uri, afiseaza-l
            if len(buf) > 256:
                p(f"  [!] Buffer nedecodat: {len(buf)} bytes. Primii 64:")
                for line in hexdump(bytes(buf[:64]), "    "):
                    p(line)
                # Arunca datele vechi pentru a nu bloca memoria
                buf = buf[-128:]

    except KeyboardInterrupt:
        p()
        p("  Intrerupt de utilizator (Ctrl+C)")
    finally:
        ser.close()

    elapsed_total = time.time() - start
    p()
    p("=" * 65)
    p(f"  SUMAR @ {baud} bps:")
    p(f"  Durata:          {elapsed_total:.1f}s")
    p(f"  Total bytes RX:  {total_bytes}")
    p(f"  Frame-uri gasite:{frame_count}")
    if total_bytes > 0:
        p(f"  Rata medie:      {total_bytes / elapsed_total:.1f} bytes/s")
    if buf:
        p(f"  Buffer ramas:    {len(buf)} bytes (nedecodat)")
        p(f"    HEX: {bytes(buf[:128]).hex(' ').upper()}")
    p("=" * 65)

    return {
        'ok': total_bytes > 0,
        'total_bytes': total_bytes,
        'frame_count': frame_count,
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

    parser = argparse.ArgumentParser(
        description="JBD BMS Passive Listener v1.0 - citeste fara TX"
    )
    parser.add_argument("port",       nargs="?", default=None,   help="Port serial (ex: /dev/ttyUSB0)")
    parser.add_argument("--baud",     type=int,  default=None,   help="Baud rate (implicit: testeaza 9600 si 19200)")
    parser.add_argument("--duration", type=int,  default=60,     help="Durata ascultare in secunde (implicit: 60)")
    parser.add_argument("--scan",     action="store_true",       help="Listeaza porturile disponibile")
    args = parser.parse_args()

    p("=" * 65)
    p("  JBD BMS Passive Listener v1.0")
    p(f"  Data: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    p(f"  Log:  {LOG_FILE}")
    p("=" * 65)
    p("  MOD PASIV - nu se trimite NIMIC pe port serial!")
    p("  Frame-urile DD/77 si 0x78 sunt detectate automat.")
    p()

    if args.scan:
        scan_ports()
        return

    if not args.port:
        p("  EROARE: Specifica portul serial sau --scan")
        p("  Exemplu: python3 jbd_listen.py /dev/ttyUSB0")
        p("  Exemplu: python3 jbd_listen.py /dev/ttyUSB0 --baud 9600 --duration 30")
        sys.exit(1)

    bauds = [args.baud] if args.baud else [9600, 19200]
    results = []

    for baud in bauds:
        r = listen_passive(args.port, baud, args.duration)
        results.append(r)
        if r.get('ok') and r.get('frame_count', 0) > 0:
            p(f"  Frame-uri gasite la {baud} bps - opresc testarea altor baud-uri.")
            break
        if r.get('ok') and r.get('total_bytes', 0) > 0:
            p(f"  Date primite la {baud} bps dar fara frame-uri decodabile.")
            p(f"  Verifica log-ul pentru dump-ul raw.")
            # Continuam sa testam si urmatorul baud

    p()
    p("=" * 65)
    p("  SUMAR FINAL")
    p("=" * 65)
    for r in results:
        if not r.get('ok'):
            p(f"  @ {r.get('baud')} bps: FARA DATE (eroare port sau niciun byte)")
        else:
            p(f"  @ {r['baud']} bps: {r['total_bytes']} bytes, {r['frame_count']} frame-uri")
    p(f"  Log complet: {LOG_FILE}")
    p("=" * 65)


if __name__ == "__main__":
    main()
