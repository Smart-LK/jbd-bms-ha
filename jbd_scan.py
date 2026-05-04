#!/usr/bin/env python3
"""
jbd_scan.py v3.0 - JBD BMS Protocol Scanner (clean, standard only)
=========================================================================
Testeaza STRICT protocolul JBD standard DD/A5/77 si 0x78.
Fara variante de adresa - doar formatul oficial din protocol.

Raspunsurile care NU contin 0xDD sau 0x77 sunt trafic strain:
  - Trafic de la alt device pe bus (ex: SRNE inverter broadcast)
  - Broadcast autonom 0x78 al BMS-ului (periodic, independent)
  - Zgomot de linie (baud rate gresit)
  NU sunt raspunsuri la comenzile noastre!

Rulare:
  pip install pyserial
  python3 jbd_scan.py /dev/ttyUSB1              # testeaza 9600 + 19200
  python3 jbd_scan.py /dev/ttyUSB1 --baud 19200
  python3 jbd_scan.py --scan

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
    print("EROARE: pip install pyserial"); sys.exit(1)

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "jbd_scan.log")

def setup_logger():
    fmt = logging.Formatter("%(asctime)s %(message)s", datefmt="%H:%M:%S.%f")
    log = logging.getLogger("scan")
    log.setLevel(logging.DEBUG)
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(fmt); log.addHandler(ch)
    fh = logging.FileHandler(LOG_FILE, mode="w", encoding="utf-8")
    fh.setFormatter(fmt); log.addHandler(fh)

log = logging.getLogger("scan")
p = lambda msg="": log.info(msg)

# ---- Checksum JBD ----------------------------------------------------------

def jbd_checksum(payload: bytes):
    s   = sum(payload) & 0xFFFF
    chk = (~s + 1) & 0xFFFF
    return (chk >> 8) & 0xFF, chk & 0xFF

def build_dd(cmd: int) -> bytes:
    """Standard request: DD A5 CMD 00 CHK_H CHK_L 77"""
    h, l = jbd_checksum(bytes([cmd, 0x00]))
    return bytes([0xDD, 0xA5, cmd, 0x00, h, l, 0x77])

def build_mos(xx: int) -> bytes:
    data = bytes([0x00, xx])
    h, l = jbd_checksum(bytes([0xE1, 0x02]) + data)
    return bytes([0xDD, 0x5A, 0xE1, 0x02]) + data + bytes([h, l, 0x77])

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def build_78(addr, s1, s2) -> bytes:
    f = bytes([addr, 0x78, (s1>>8)&0xFF, s1&0xFF, (s2>>8)&0xFF, s2&0xFF, 0, 0])
    c = crc16(f); return f + bytes([c&0xFF, (c>>8)&0xFF])

# ---- Receive ----------------------------------------------------------------

def recv_all(ser, timeout=3.0) -> bytes:
    buf = bytearray(); start = time.time(); last = time.time()
    while time.time() - start < timeout:
        ch = ser.read(256)
        if ch: buf.extend(ch); last = time.time()
        elif buf and (time.time()-last > 0.5): break
        time.sleep(0.02)
    return bytes(buf)

def hexdump(data: bytes, indent="    "):
    for i in range(0, len(data), 16):
        ch = data[i:i+16]
        asc = ''.join(chr(b) if 32<=b<127 else '.' for b in ch)
        p(f"{indent}{i:04X}: {' '.join(f'{b:02X}' for b in ch):<48} |{asc}|")

# ---- Analiza raw response ---------------------------------------------------

FAULT = {0x0001:"cell_OV",0x0002:"cell_UV",0x0004:"pack_OV",0x0008:"pack_UV",
          0x0010:"chg_OT",0x0020:"chg_UT",0x0040:"dchg_OT",0x0080:"dchg_UT",
          0x0100:"chg_OC",0x0200:"dchg_OC",0x0400:"short",0x0800:"ic_err",0x1000:"sw_lock"}

def analyse(raw: bytes, cmd: int) -> dict:
    """Analizeaza raw bytes: e raspuns JBD valid sau trafic strain?"""
    if not raw:
        p(f"  RX: NIMIC (timeout)")
        return {'valid': False}

    p(f"  RX ({len(raw)} bytes):")
    hexdump(raw)

    dd_pos = [i for i, b in enumerate(raw) if b == 0xDD]
    p77_pos = [i for i, b in enumerate(raw) if b == 0x77]

    p(f"  Structura:")
    p(f"    0xDD la pozitia: {dd_pos if dd_pos else 'ABSENT - NU e raspuns JBD!'}")
    p(f"    0x77 la pozitia: {p77_pos if p77_pos else 'ABSENT - NU e raspuns JBD!'}")

    if not dd_pos or not p77_pos:
        p(f"  >>> Trafic strain (alt device pe bus sau broadcast autonom)")
        if len(raw)>1 and raw[1]==0x78:
            p(f"  >>> Detectat frame 0x78 (addr=0x{raw[0]:02X}) - BMS broadcast autonom")
        return {'valid': False, 'foreign': True}

    # Incearca parsare DD/A5/77
    for i in dd_pos:
        rest = raw[i:]
        if len(rest) < 7: continue
        rcmd=rest[1]; status=rest[2]; length=rest[3]
        total = 4+length+2+1
        if len(rest)<total or rest[total-1]!=0x77: continue
        if rcmd != cmd: continue
        data  = rest[4:4+length]
        ch    = rest[4+length]; cl = rest[5+length]
        eh, el = jbd_checksum(bytes([rcmd, length]) + data)
        ok = (eh==ch and el==cl)
        p(f"  >>> RASPUNS JBD VALID la offset {i}! STATUS=0x{status:02X} LEN={length} CHK={'OK' if ok else 'GRESIT'}")
        return {'valid': True, 'data': data, 'status': status, 'chk_ok': ok}

    p(f"  >>> 0xDD/0x77 prezente dar frame invalid pentru CMD=0x{cmd:02X}")
    return {'valid': False}

# ---- Parse JBD responses ----------------------------------------------------

def print_status(data: bytes):
    if len(data) < 23: p(f"    Prea scurt: {len(data)}"); return
    v=struct.unpack('>H',data[0:2])[0]; i=struct.unpack('>h',data[2:4])[0]
    cap=struct.unpack('>H',data[4:6])[0]; nom=struct.unpack('>H',data[6:8])[0]
    cyc=struct.unpack('>H',data[8:10])[0]; dr=struct.unpack('>H',data[10:12])[0]
    fault=struct.unpack('>H',data[16:18])[0]; sw=data[18]; soc=data[19]; fet=data[20]
    nc=data[21]; ntc=data[22]
    p(f"    Tensiune: {v/100.0:.2f}V  Curent: {i/100.0:.2f}A  Putere: {v*i/10000.0:.1f}W")
    p(f"    SOC: {soc}%  Cap.ramasa: {cap*10/1000:.2f}Ah  Cap.nominala: {nom*10/1000:.2f}Ah")
    p(f"    Cicluri: {cyc}")
    try: p(f"    Data productie: {2000+(dr>>9):04d}-{(dr>>5)&0x0F:02d}-{dr&0x1F:02d}")
    except: p(f"    Data prod raw: 0x{dr:04X}")
    p(f"    Fault: 0x{fault:04X} = {[d for m,d in FAULT.items() if fault&m] or ['none']}")
    p(f"    FET: 0x{fet:02X}  CHG={'ON' if fet&1 else 'OFF'}  DSG={'ON' if fet&2 else 'OFF'}")
    p(f"    Celule: {nc}  NTC: {ntc}")
    p(f"    Temperaturi (formula (raw-2731)/10 per jbdbms.h):")
    for j in range(ntc):
        off=23+j*2
        if off+2<=len(data):
            rt=struct.unpack('>H',data[off:off+2])[0]
            p(f"      T{j+1}: raw=0x{rt:04X}={rt} -> {(rt-2731)/10.0:.1f}C")

def print_cells(data: bytes):
    if len(data)<2 or len(data)%2!=0: p(f"    Invalid len={len(data)}"); return
    n=len(data)//2; cells=[struct.unpack('>H',data[i*2:i*2+2])[0] for i in range(n)]
    p(f"    {n} celule: min={min(cells)} max={max(cells)} delta={max(cells)-min(cells)} sum={sum(cells)}mV={sum(cells)/1000:.2f}V")
    for i,v in enumerate(cells): p(f"      C{i+1:02d}: {v} mV")

# ---- Scan DD/A5/77 ----------------------------------------------------------

def scan_dd(ser, baud: int) -> dict:
    p(); p("="*65)
    p(f"  DD/A5/77 Standard (fara adresa) @ {baud} bps")
    p("="*65)
    p("  NOTE: Raspuns valid = contine 0xDD la inceput si 0x77 la sfarsit")
    p("        Orice altceva = trafic strain pe bus (nu e raspuns la cererea noastra)")
    results={}
    for cmd,name,pfn in [(0x03,"Basic Info",print_status),(0x04,"Cell Voltages",print_cells),(0x05,"HW Version",None)]:
        req=build_dd(cmd)
        p(); p(f"  --- CMD 0x{cmd:02X}: {name} ---")
        p(f"  TX: {req.hex(' ').upper()}")
        ser.reset_input_buffer(); ser.write(req)
        time.sleep(0.15)
        raw=recv_all(ser, timeout=3.0)
        res=analyse(raw, cmd); results[f'cmd{cmd:02x}']=res
        if res.get('valid') and res.get('data'):
            p(f"  Parse:")
            if pfn: pfn(res['data'])
            elif cmd==0x05:
                try: p(f"    HW ID: '{res['data'].decode('ascii',errors='replace').strip()}'")
                except: p(f"    HW raw: {res['data'].hex()}")
        time.sleep(0.5)
    ok=sum(1 for r in results.values() if r.get('valid'))
    p(); p(f"  SUMAR DD/A5/77 @ {baud}: {ok}/3 raspunsuri JBD valide")
    if ok==0: p("  >>> DD/A5/77 NU functioneaza pe aceasta interfata")
    return results

# ---- Scan 0x78 --------------------------------------------------------------

def scan_78(ser, baud: int) -> dict:
    p(); p("="*65); p(f"  PROTOCOL 0x78 @ {baud} bps"); p("="*65)
    req=build_78(0x01, 0x1000, 0x10A0)
    p(f"  TX: {req.hex(' ').upper()}")
    ser.reset_input_buffer(); ser.write(req); time.sleep(0.5)
    raw=recv_all(ser, timeout=4.0)
    if not raw: p("  TIMEOUT"); return {'ok':False}
    p(f"  RX ({len(raw)} bytes):"); hexdump(raw)
    if len(raw)<10 or raw[0]!=0x01 or raw[1]!=0x78: p("  Header invalid"); return {'ok':False}
    dl=struct.unpack('>H',raw[6:8])[0]; pl=raw[8:8+dl]
    p(f"  Header OK: data_len={dl}")
    if len(pl)<70: return {'ok':False}
    v=struct.unpack('>H',pl[0:2])[0]; i=struct.unpack('>h',pl[2:4])[0]
    soc=struct.unpack('>H',pl[22:24])[0]&0xFF
    p(f"  Tensiune: {v/100.0:.2f}V  Curent: {i/100.0:.2f}A  SOC: {soc}%")
    nc=struct.unpack('>H',pl[66:68])[0]
    p(f"  Num celule: {nc}")
    if 0<nc<=32:
        ce=68+nc*2
        if len(pl)>=ce:
            cells=[struct.unpack('>H',pl[68+i*2:70+i*2])[0] for i in range(nc)]
            p(f"  Celule: min={min(cells)} max={max(cells)} delta={max(cells)-min(cells)} sum={sum(cells)/1000:.2f}V")
            for i,v in enumerate(cells): p(f"    C{i+1:02d}: {v} mV")
            if len(pl)>ce+2:
                nntc=struct.unpack('>H',pl[ce:ce+2])[0]
                p(f"  NTC: {nntc}")
                for j in range(min(nntc,5)):
                    off=ce+2+j*2
                    if off+2<=len(pl):
                        rt=struct.unpack('>H',pl[off:off+2])[0]
                        t500=round((rt-500)/10.0,1); t2731=round((rt-2731)/10.0,1)
                        p(f"    T{j+1}: raw={rt}  (raw-500)/10={t500}C  (raw-2731)/10={t2731}C")
                        p(f"         Corect: {'(raw-500)/10='+str(t500)+'C' if 0<t500<80 else '(raw-2731)/10='+str(t2731)+'C' if 0<t2731<80 else '?'}")
    jbd=pl.find(b'JBD')
    if jbd>=0:
        nm=pl[jbd:jbd+20].split(b'\x00')[0]
        try: p(f"  Device: '{nm.decode('ascii')}'")
        except: pass
    p(f"  SUMAR 0x78 @ {baud}: FUNCTIONAL")
    return {'ok':True,'v':v/100.0,'i':i/100.0,'soc':soc}

# ---- Porturi ----------------------------------------------------------------

def list_ports():
    ports=[]
    by_id="/dev/serial/by-id/"
    if os.path.isdir(by_id):
        for lk in sorted(glob.glob(by_id+"*")):
            try: p(f"  by-id: {os.path.basename(lk)} -> {os.path.realpath(lk)}"); ports.append(os.path.realpath(lk))
            except: pass
    for d in sorted(glob.glob("/dev/ttyUSB*")):
        if d not in ports: p(f"  {d}"); ports.append(d)
    return list(dict.fromkeys(ports))

# ---- Main -------------------------------------------------------------------

def main():
    setup_logger()
    parser=argparse.ArgumentParser(description="JBD BMS Scanner v3.0")
    parser.add_argument("port",nargs="?",default=None)
    parser.add_argument("--baud",type=int,default=None)
    parser.add_argument("--proto",choices=["dd","78","all"],default="all")
    parser.add_argument("--scan",action="store_true")
    args=parser.parse_args()
    p("="*65); p("  JBD BMS Protocol Scanner v3.0")
    p(f"  Data: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    p(f"  Log: {LOG_FILE}"); p("="*65)
    p("  Requests JBD standard (fara adresa):")
    for cmd in [0x03,0x04,0x05]: p(f"    {build_dd(cmd).hex(' ').upper()}")
    p(f"  MOS DSG off: {build_mos(0x02).hex(' ').upper()}")
    if args.scan: p(); p("  Porturi:"); list_ports(); return
    if not args.port: p(); p("  EROARE: Specifica portul!"); sys.exit(1)
    bauds=[args.baud] if args.baud else [9600,19200]
    all_res={}
    for baud in bauds:
        p(); p("="*65); p(f"  PORT: {args.port} @ {baud} bps"); p("="*65)
        try:
            ser=serial.Serial(port=args.port,baudrate=baud,bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0.05)
            p("  Serial OK")
        except Exception as e: p(f"  EROARE: {e}"); continue
        time.sleep(0.3); res={}
        if args.proto in ("dd","all"): res['dd']=scan_dd(ser,baud)
        time.sleep(0.5)
        if args.proto in ("78","all"): res['78']=scan_78(ser,baud)
        all_res[baud]=res; ser.close(); time.sleep(0.5)
    p(); p("="*65); p("  SUMAR FINAL"); p("="*65)
    for baud,res in all_res.items():
        dd_ok=sum(1 for r in res.get('dd',{}).values() if r.get('valid'))
        ok78=res.get('78',{}).get('ok',False)
        p(f"  @ {baud}: DD/A5/77={'OK ('+str(dd_ok)+'/3)' if dd_ok else 'FAIL'}  0x78={'OK' if ok78 else 'FAIL'}")
    p()
    dd_any=any(sum(1 for r in res.get('dd',{}).values() if r.get('valid'))>0 for res in all_res.values())
    ok78b=next((b for b,r in all_res.items() if r.get('78',{}).get('ok')),None)
    if dd_any: p("  RECOMANDAT: DD/A5/77 functional -> addon v7.x")
    elif ok78b: p(f"  RECOMANDAT: 0x78 @ {ok78b} bps -> addon v7.1")
    else: p("  ATENTIE: Niciun protocol nu a raspuns!")
    p(); p(f"  Log: {LOG_FILE}"); p("="*65)

if __name__=="__main__": main()
