#!/usr/bin/env python3
"""
jbd_bms_mqtt.py  v7.0 - Protocol JBD Standard DD/A5/77
=======================================================
Implementare exhaustiva conform protocolului oficial furnizat:
  JDB RS485/RS232/UART/Bluetooth Communication Protocol V4

Comenzi implementate:
  READ  0x03 - Basic information and status (poll fiecare ciclu)
  READ  0x04 - Battery cell voltages (poll fiecare ciclu)
  READ  0x05 - Hardware version number (citit o data la startup)
  WRITE 0xE1 - Control MOS charge/discharge via MQTT

===========================================================================
PROTOCOL FRAME STRUCTURE
===========================================================================
Request  (host -> BMS): DD A5 [CMD] [LEN=0]          [CHK_H] [CHK_L] 77
Response (BMS -> host): DD [CMD] [STATUS] [LEN] [DATA] [CHK_H] [CHK_L] 77
Write    (host -> BMS): DD 5A [CMD] [LEN] [DATA]      [CHK_H] [CHK_L] 77

STATUS: 0x00 = OK, 0x80 = error

CHECKSUM covers [CMD, LEN, DATA...] (not START=DD, not STATE=A5/5A, not STOP=77):
  sum_val  = sum(CMD, LEN, DATA...) & 0xFFFF
  checksum = (~sum_val + 1) & 0xFFFF   <- two's complement negation
  CHK_H = checksum >> 8
  CHK_L = checksum & 0xFF

Examples (from protocol PDF):
  Read 0x03: DD A5 03 00 FF FD 77   (sum=0x03, ~0x03+1=0xFFFD)
  Read 0x04: DD A5 04 00 FF FC 77   (sum=0x04, ~0x04+1=0xFFFC)
  Read 0x05: DD A5 05 00 FF FB 77   (sum=0x05, ~0x05+1=0xFFFB)
  MOS (DSG off): DD 5A E1 02 00 02 FF 1B 77 (sum=E1+02+00+02=0xE5, ~0xE5+1=0xFF1B)

===========================================================================
CURRENT (in 0x03 response, bytes 2-3):
  Type: signed int16 big-endian, unit 10mA
  POSITIVE = charging (curent intra in baterie)
  NEGATIVE = discharging (curent iese din baterie)
  Formula: current_A = struct.unpack('>h', data[2:4])[0] / 100.0
  NU necesita calibrare hardware - valoare directa din protocol!

  Exemplu din PDF: 0xF824 = -2012 (signed) -> -2012 * 10mA = -20.12A (discharge)

TEMPERATURES (in 0x03 response, dupa datele fixe):
  Unit: 0.1 Kelvin absolut
  Formula: temp_C = (raw - 2731) / 10.0
  Exemplu: 0x0B98 = 2968 -> (2968 - 2731) / 10 = 23.7C

FET STATUS (in 0x03 response, byte 20):
  bit0 = charge MOS    (1=ON/enabled, 0=OFF/disabled)
  bit1 = discharge MOS (1=ON/enabled, 0=OFF/disabled)

MOS CONTROL COMMAND (WRITE 0xE1, data=[0x00, XX]):
  XX=0x00: release all (ambele MOS enabled - normal)
  XX=0x01: disable charging MOS only
  XX=0x02: disable discharging MOS only
  XX=0x03: disable both MOS

PROTECTION STATUS (in 0x03 response, bytes 16-17):
  16 biti, fiecare bit = o stare de protectie (vezi PROT_BITS)

CHANGELOG:
  v7.0 - RESCRIS COMPLET - implementare protocol DD/A5/77 standard
         ELIMINAT CURRENT_OFFSET/CURRENT_SCALE (era complet gresit)
         Curentul = signed int16 bytes[2:4] / 100.0 - fara calibrare
         FIX: in v6.x curentul era citit de la bytes[6:8] = Nominal Capacity!
         ADAUGAT READ 0x04 - citire tensiuni individuale celule
         ADAUGAT READ 0x05 - versiune hardware (citit la startup)
         ADAUGAT senzori: protection_active, protection_status_text,
           production_date, software_version, balance_active,
           cell_avg_mv, hardware_version
         ADAUGAT toate bitii de protectie ca binary_sensor individual
         MOS command refacut conform protocol (DD 5A E1 02 00 XX ...)
         Baud rate default: 9600 (standard protocol PDF)
         Nota: daca BMS-ul e configurat la alta rata (ex. 19200),
           schimba baud_rate in config
  v6.3 - fix CURRENT_SCALE (obsolet, eliminat in v7.0)
  v6.2 - serial_port_by_id
  v6.0 - config din options.json
  v5.0 - control MOS DD A5

Autor: Smart-LK / Claude Sonnet, mai 2026
"""

import serial
import serial.tools.list_ports
import struct
import time
import json
import logging
import sys
import os
import glob
import threading
import paho.mqtt.client as mqtt

# =============================================================================
# CONFIG
# =============================================================================

DEFAULTS = {
    "serial_port":       "/dev/ttyUSB0",
    "serial_port_by_id": "",
    "baud_rate":         9600,       # Standard protocol = 9600 bps
    "poll_interval":     30,
    "num_cells":         9,
    "mqtt_host":         "core-mosquitto",
    "mqtt_port":         1883,
    "mqtt_user":         "mqtt_local",
    "mqtt_password":     "mqtt2026vidra",
    "mqtt_prefix":       "jbd_bms",
    "device_name":       "Acumulator JBD",
    "device_id":         "jbd_bms_vidra",
    "log_level":         "INFO",
}

def load_config() -> dict:
    cfg = dict(DEFAULTS)
    options_path = "/data/options.json"
    if os.path.exists(options_path):
        try:
            with open(options_path) as f:
                cfg.update(json.load(f))
        except Exception as e:
            print(f"[WARN] Nu pot citi options.json: {e}, folosesc defaults")
    return cfg

cfg = load_config()

BAUD_RATE     = int(cfg["baud_rate"])
POLL_INTERVAL = int(cfg["poll_interval"])
NUM_CELLS     = int(cfg["num_cells"])
MQTT_HOST     = cfg["mqtt_host"]
MQTT_PORT     = int(cfg["mqtt_port"])
MQTT_USER     = cfg["mqtt_user"]
MQTT_PASS     = cfg["mqtt_password"]
MQTT_PREFIX   = cfg["mqtt_prefix"]
DEVICE_NAME   = cfg["device_name"]
DEVICE_ID     = cfg["device_id"]

log_level = getattr(logging, cfg.get("log_level", "INFO").upper(), logging.INFO)
logging.basicConfig(
    level=log_level,
    format='%(asctime)s | %(levelname)s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[logging.StreamHandler(sys.stdout)]
)
log = logging.getLogger(__name__)

# =============================================================================
# PROTOCOL CONSTANTS
# =============================================================================

CMD_BASIC   = 0x03   # Read basic information and status
CMD_CELLS   = 0x04   # Read battery cell voltages
CMD_VERSION = 0x05   # Read hardware version number
CMD_MOS     = 0xE1   # Write: control MOS

START_BYTE  = 0xDD
STATE_READ  = 0xA5
STATE_WRITE = 0x5A
STOP_BYTE   = 0x77

# Protection status bits (bytes 16-17 of 0x03 response)
PROT_BITS = {
    0:  ("cell_overvoltage",          "Cell overvoltage"),
    1:  ("cell_undervoltage",         "Cell undervoltage"),
    2:  ("pack_overvoltage",          "Pack overvoltage"),
    3:  ("pack_undervoltage",         "Pack undervoltage"),
    4:  ("charge_overtemperature",    "Charge over-temperature"),
    5:  ("charge_undertemperature",   "Charge under-temperature"),
    6:  ("discharge_overtemperature", "Discharge over-temperature"),
    7:  ("discharge_undertemperature","Discharge under-temperature"),
    8:  ("charge_overcurrent",        "Charge overcurrent"),
    9:  ("discharge_overcurrent",     "Discharge overcurrent"),
    10: ("short_circuit",             "Short circuit"),
    11: ("frontend_ic_error",         "Front-end IC error"),
    12: ("software_lock_mos",         "Software lock MOS"),
}

# MOS control codes (XX byte in WRITE 0xE1 command)
MOS_RELEASE_ALL = 0x00   # Release software control (both MOS enabled)
MOS_DISABLE_CHG = 0x01   # Disable charge MOS
MOS_DISABLE_DSG = 0x02   # Disable discharge MOS
MOS_DISABLE_ALL = 0x03   # Disable both

# =============================================================================
# PROTOCOL LAYER: CHECKSUM, BUILD FRAMES, PARSE RESPONSES
# =============================================================================

def _jbd_checksum(payload: bytes) -> tuple[int, int]:
    """
    Calculeaza checksum-ul JBD pentru [CMD, LEN, DATA...].
    Returneaza (CHK_H, CHK_L).
    Formula: sum(bytes) -> ~sum + 1 (two's complement in 16-bit)
    """
    s   = sum(payload) & 0xFFFF
    chk = (~s + 1) & 0xFFFF
    return (chk >> 8) & 0xFF, chk & 0xFF


def build_read_request(cmd: int) -> bytes:
    """
    Construieste un request de citire:
    DD A5 [CMD] 00 [CHK_H] [CHK_L] 77
    Checksum over [CMD, 0x00]
    """
    chk_h, chk_l = _jbd_checksum(bytes([cmd, 0x00]))
    return bytes([START_BYTE, STATE_READ, cmd, 0x00, chk_h, chk_l, STOP_BYTE])


def build_write_mos(xx: int) -> bytes:
    """
    Construieste comanda MOS write:
    DD 5A E1 02 00 [XX] [CHK_H] [CHK_L] 77
    Checksum over [E1, 02, 00, XX]
    XX: 0x00=release, 0x01=CHG off, 0x02=DSG off, 0x03=both off
    """
    data         = bytes([0x00, xx & 0xFF])
    chk_h, chk_l = _jbd_checksum(bytes([CMD_MOS, len(data)]) + data)
    return bytes([START_BYTE, STATE_WRITE, CMD_MOS, len(data)]) + data + bytes([chk_h, chk_l, STOP_BYTE])


# Pre-built requests
REQ_BASIC   = build_read_request(CMD_BASIC)
REQ_CELLS   = build_read_request(CMD_CELLS)
REQ_VERSION = build_read_request(CMD_VERSION)


def read_response(ser, expected_cmd: int, timeout: float = 3.0) -> bytes | None:
    """
    Citeste si valideaza un raspuns DD/A5/77 de la BMS.
    Returneaza payload-ul de date (bytes[4:4+length]),
    sau None la timeout/eroare de status.
    """
    buf   = bytearray()
    start = time.time()

    while time.time() - start < timeout:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)

        # Resync pe byte-ul de start 0xDD
        while len(buf) > 0 and buf[0] != START_BYTE:
            buf = buf[1:]

        # Minim: DD + cmd + status + length = 4 bytes
        if len(buf) < 4:
            time.sleep(0.01)
            continue

        cmd    = buf[1]
        status = buf[2]
        length = buf[3]

        # Frame total: DD + cmd + status + len + data + CHK_H + CHK_L + 77
        total = 4 + length + 2 + 1
        if len(buf) < total:
            time.sleep(0.01)
            continue

        # Verifica stop byte
        if buf[total - 1] != STOP_BYTE:
            log.debug(f"Stop byte invalid 0x{buf[total-1]:02X} la pos {total-1}, resync")
            buf = buf[1:]
            continue

        frame = bytes(buf[:total])
        buf   = buf[total:]

        # Comanda asteptata?
        if cmd != expected_cmd:
            log.debug(f"CMD neasteptat 0x{cmd:02X} != 0x{expected_cmd:02X}, skip")
            continue

        # Status
        if status != 0x00:
            log.warning(f"BMS eroare cmd=0x{cmd:02X} status=0x{status:02X}")
            return None

        # Extrage si verifica checksum
        data  = frame[4:4 + length]
        chk_h = frame[4 + length]
        chk_l = frame[5 + length]
        exp_h, exp_l = _jbd_checksum(bytes([cmd, length]) + data)
        if exp_h != chk_h or exp_l != chk_l:
            log.warning(f"Checksum cmd=0x{cmd:02X}: expected 0x{exp_h:02X}{exp_l:02X} "
                        f"got 0x{chk_h:02X}{chk_l:02X} - continuam oricum")

        return data

    log.warning(f"Timeout cmd=0x{expected_cmd:02X}")
    return None

# =============================================================================
# PARSE 0x03 - BASIC INFORMATION AND STATUS
# =============================================================================

def _decode_production_date(raw: int) -> str:
    """Decode data productie din format JBD 2-byte packed."""
    try:
        day   = raw & 0x1F
        month = (raw >> 5) & 0x0F
        year  = 2000 + (raw >> 9)
        return f"{year:04d}-{month:02d}-{day:02d}"
    except Exception:
        return f"raw=0x{raw:04X}"


def parse_basic_info(data: bytes) -> dict | None:
    """
    Parseaza payload-ul raspunsului 0x03 (Basic Information).

    Structura (conform protocol v4):
    Off  Size  Description
    0-1   2    Total voltage, unit 10mV, unsigned BE
    2-3   2    Current, unit 10mA, SIGNED BE  (+ = charge, - = discharge)
    4-5   2    Remaining capacity, unit 10mAh
    6-7   2    Nominal capacity, unit 10mAh
    8-9   2    Cycles (count)
    10-11 2    Production date (packed: day[4:0] month[8:5] year[15:9])
    12-13 2    Balance status low  (bit per cell, cells 1-16)
    14-15 2    Balance status high (bit per cell, cells 17-32)
    16-17 2    Protection status (bit field, 13 bits defined)
    18    1    Software version (0x10 = V1.0)
    19    1    RSOC - remaining capacity %
    20    1    FET status: bit0=CHG MOS (1=ON), bit1=DSG MOS (1=ON)
    21    1    Number of battery strings (cells)
    22    1    Number of NTC sensors
    23+   2*N  NTC temperatures, unit 0.1K (subtract 273.1 for Celsius)
    """
    MIN_LEN = 23
    if len(data) < MIN_LEN:
        log.warning(f"parse_basic_info: date prea scurte {len(data)} < {MIN_LEN}")
        return None

    r = {}

    # Tensiune totala: unit 10mV -> V
    r['voltage'] = round(struct.unpack('>H', data[0:2])[0] / 100.0, 2)

    # Curent: SIGNED int16, unit 10mA -> A
    # + = incarcare (charging is positive per protocol)
    # - = descarcare
    current_raw  = struct.unpack('>h', data[2:4])[0]   # SIGNED
    r['current']     = round(current_raw / 100.0, 2)
    r['current_raw'] = current_raw

    # Capacitate
    r['capacity_remaining_ah'] = round(struct.unpack('>H', data[4:6])[0]  * 10 / 1000, 2)
    r['capacity_nominal_ah']   = round(struct.unpack('>H', data[6:8])[0]  * 10 / 1000, 2)

    # Cicluri si data productie
    r['cycles']          = struct.unpack('>H', data[8:10])[0]
    r['production_date'] = _decode_production_date(struct.unpack('>H', data[10:12])[0])

    # Balance status
    bal_low  = struct.unpack('>H', data[12:14])[0]
    bal_high = struct.unpack('>H', data[14:16])[0]
    r['balance_bits']   = (bal_high << 16) | bal_low
    r['balance_active'] = r['balance_bits'] != 0

    # Protectie
    prot_raw = struct.unpack('>H', data[16:18])[0]
    r['protection_status_raw'] = prot_raw
    for bit, (key, _) in PROT_BITS.items():
        r[key] = bool(prot_raw & (1 << bit))
    r['protection_active'] = prot_raw != 0
    active = [desc for bit, (key, desc) in PROT_BITS.items() if prot_raw & (1 << bit)]
    r['protection_status_text'] = ', '.join(active) if active else "OK"

    # Versiune software: nibble high = major, nibble low = minor
    sw = data[18]
    r['software_version'] = f"V{sw >> 4}.{sw & 0x0F}"

    # RSOC
    r['soc'] = data[19]

    # FET status
    fet = data[20]
    r['charge_mos']     = bool(fet & 0x01)
    r['discharge_mos']  = bool(fet & 0x02)
    r['fet_status_raw'] = fet

    # Celule si NTC
    r['num_cells'] = data[21]
    num_ntc = data[22]
    r['num_ntc'] = num_ntc

    # Temperaturi NTC: unit 0.1K absolut, formula: T_C = (raw - 2731) / 10
    r['temperatures'] = []
    for i in range(num_ntc):
        off = 23 + i * 2
        if off + 2 <= len(data):
            raw_t = struct.unpack('>H', data[off:off+2])[0]
            r['temperatures'].append(round((raw_t - 2731) / 10.0, 1))

    # Putere calculata: P = V * I
    r['power'] = round(r['voltage'] * r['current'], 1)

    return r

# =============================================================================
# PARSE 0x04 - CELL VOLTAGES
# =============================================================================

def parse_cell_voltages(data: bytes) -> list[int] | None:
    """
    Parseaza payload-ul raspunsului 0x04 (Cell Voltages).
    Data length = N * 2 (N = numar celule)
    Fiecare celula: 2 bytes, unsigned BE, unit mV
    """
    if len(data) < 2 or len(data) % 2 != 0:
        log.warning(f"parse_cell_voltages: date invalide len={len(data)}")
        return None
    n = len(data) // 2
    return [struct.unpack('>H', data[i*2:i*2+2])[0] for i in range(n)]

# =============================================================================
# PARSE 0x05 - HARDWARE VERSION
# =============================================================================

def parse_hardware_version(data: bytes) -> str:
    """
    Parseaza payload-ul raspunsului 0x05 (Hardware Version).
    Date = caractere ASCII, max 31.
    Exemplu: '0123456789' sau 'LH-XXXX'
    """
    try:
        return data.decode('ascii', errors='replace').strip()
    except Exception:
        return data.hex()

# =============================================================================
# BMS READ / WRITE FUNCTIONS
# =============================================================================

serial_lock = threading.Lock()
ser_global  = None
mos_state   = {'charge': True, 'discharge': True}


def _send_receive(ser, request: bytes, expected_cmd: int, timeout: float = 3.0) -> bytes | None:
    """Trimite request si asteapta raspuns (thread-safe)."""
    with serial_lock:
        ser.reset_input_buffer()
        ser.write(request)
        time.sleep(0.1)
        return read_response(ser, expected_cmd, timeout)


def read_basic_info(ser) -> dict | None:
    """READ 0x03 - Basic information and status."""
    data = _send_receive(ser, REQ_BASIC, CMD_BASIC)
    if data is None:
        return None
    return parse_basic_info(data)


def read_cell_voltages(ser) -> list[int] | None:
    """READ 0x04 - Battery cell voltages."""
    data = _send_receive(ser, REQ_CELLS, CMD_CELLS)
    if data is None:
        return None
    return parse_cell_voltages(data)


def read_hardware_version(ser) -> str | None:
    """READ 0x05 - Hardware version (apelat o data la startup)."""
    data = _send_receive(ser, REQ_VERSION, CMD_VERSION)
    if data is None:
        return None
    return parse_hardware_version(data)


def send_mos_command(ser, charge_on: bool, discharge_on: bool) -> bool:
    """
    WRITE 0xE1 - Control MOS charge/discharge.
    charge_on=True    = CHG MOS enabled (incarcare permisa)
    discharge_on=True = DSG MOS enabled (descarcare permisa)
    """
    xx = MOS_RELEASE_ALL
    if not charge_on:    xx |= MOS_DISABLE_CHG
    if not discharge_on: xx |= MOS_DISABLE_DSG

    cmd_bytes = build_write_mos(xx)
    log.info(f"MOS write: CHG={'ON' if charge_on else 'OFF'} "
             f"DSG={'ON' if discharge_on else 'OFF'} "
             f"XX=0x{xx:02X} | TX: {cmd_bytes.hex(' ').upper()}")

    with serial_lock:
        ser.reset_input_buffer()
        ser.write(cmd_bytes)
        time.sleep(0.3)
        ser.timeout = 1.5
        resp = bytearray(ser.read(16))

    if resp:
        log.info(f"MOS raspuns: {resp.hex(' ').upper()}")
        # Verifica frame: DD E1 00 00 CHK_H CHK_L 77
        if len(resp) >= 7 and resp[0] == 0xDD and resp[1] == CMD_MOS and resp[2] == 0x00:
            log.info("MOS confirmat OK")
            return True
        log.warning("MOS: raspuns neclar")
    else:
        log.warning("MOS: fara confirmare de la BMS")
    return False


def read_all(ser) -> dict | None:
    """
    Poll complet: READ 0x03 (basic) + READ 0x04 (cells).
    Combina rezultatele. Returneaza None daca 0x03 esueaza.
    """
    basic = read_basic_info(ser)
    if basic is None:
        return None
    time.sleep(0.15)

    cells = read_cell_voltages(ser)
    if cells:
        basic['cell_voltages'] = cells
        basic['cell_min_mv']   = min(cells)
        basic['cell_max_mv']   = max(cells)
        basic['cell_delta_mv'] = max(cells) - min(cells)
        basic['cell_avg_mv']   = round(sum(cells) / len(cells), 1)
    else:
        log.warning("0x04 celule esuate, continuam fara tensiuni individuale")

    # Sincronizam starea MOS locala cu BMS-ul
    mos_state['charge']    = basic.get('charge_mos',    mos_state['charge'])
    mos_state['discharge'] = basic.get('discharge_mos', mos_state['discharge'])

    log.info(
        f"V={basic.get('voltage')}V "
        f"I={basic.get('current')}A "
        f"P={basic.get('power')}W "
        f"SoC={basic.get('soc')}% "
        f"Cap={basic.get('capacity_remaining_ah')}Ah "
        f"CHG={'ON' if basic.get('charge_mos') else 'OFF'} "
        f"DSG={'ON' if basic.get('discharge_mos') else 'OFF'} "
        f"Prot={basic.get('protection_status_text')}"
    )
    if cells:
        log.info(f"Celule: min={basic['cell_min_mv']}mV max={basic['cell_max_mv']}mV "
                 f"delta={basic['cell_delta_mv']}mV avg={basic['cell_avg_mv']}mV")

    return basic

# =============================================================================
# PORT SERIAL
# =============================================================================

def scan_serial_ports() -> dict:
    result  = {}
    by_id   = "/dev/serial/by-id/"
    if os.path.isdir(by_id):
        for link in sorted(glob.glob(by_id + "*")):
            try:
                result[os.path.basename(link)] = os.path.realpath(link)
            except Exception:
                pass
    return result


def resolve_serial_port(cfg: dict) -> str:
    ports = scan_serial_ports()
    log.info("--- Dispozitive seriale ---")
    for name, path in ports.items():
        log.info(f"  by-id: {name} -> {path}")
    tty = sorted(glob.glob("/dev/ttyUSB*"))
    if tty: log.info(f"  ttyUSB: {', '.join(tty)}")
    log.info("---------------------------")

    by_id_name = cfg.get("serial_port_by_id", "").strip()
    if by_id_name:
        p = f"/dev/serial/by-id/{by_id_name}"
        if os.path.exists(p):
            resolved = os.path.realpath(p)
            log.info(f"Folosesc by-id: {by_id_name} -> {resolved}")
            return p
        log.warning(f"serial_port_by_id '{by_id_name}' nu exista! "
                    f"Disponibile: {list(ports.keys())}")
    return cfg["serial_port"]

# =============================================================================
# MQTT AUTO-DISCOVERY
# =============================================================================

def publish_discovery(client, data: dict):
    """
    Publica configuratia HA auto-discovery pentru toti senzorii.
    Senzori publicati:
      - Din 0x03: voltage, current, power, soc, capacitate, cicluri,
          temperaturi NTC, status FET, stare protectie (text + biti individuali),
          software version, production date, balance active
      - Din 0x04: tensiuni individuale celule + min/max/delta/avg
      - Din 0x05: hardware version
      - Switches: control CHG/DSG MOS
    """
    num_cells = data.get('num_cells', NUM_CELLS)
    num_ntc   = data.get('num_ntc', 0)
    device = {
        "identifiers":  [DEVICE_ID],
        "name":         DEVICE_NAME,
        "manufacturer": "JBD",
        "model":        f"9S3P ({num_cells}S)",
        "sw_version":   data.get('software_version', ''),
        "hw_version":   data.get('hardware_version', ''),
    }
    avail = {
        "availability_topic":    f"{MQTT_PREFIX}/status",
        "payload_available":     "online",
        "payload_not_available": "offline",
    }
    state = f"{MQTT_PREFIX}/state"

    def pub_sensor(uid, name, tmpl, unit=None, dc=None, sc="measurement",
                   icon=None, ent_cat=None, precision=None):
        c = {
            "unique_id":      f"{DEVICE_ID}_{uid}",
            "name":           name,
            "state_topic":    state,
            "value_template": tmpl,
            "device":         device,
            "state_class":    sc,
            **avail
        }
        if unit:      c["unit_of_measurement"] = unit
        if dc:        c["device_class"] = dc
        if icon:      c["icon"] = icon
        if ent_cat:   c["entity_category"] = ent_cat
        if precision: c["suggested_display_precision"] = precision
        client.publish(f"homeassistant/sensor/{DEVICE_ID}/{uid}/config",
                       json.dumps(c), retain=True)

    def pub_binary(uid, name, tmpl, dc=None, icon=None, ent_cat=None):
        c = {
            "unique_id":      f"{DEVICE_ID}_{uid}",
            "name":           name,
            "state_topic":    state,
            "value_template": tmpl,
            "payload_on":     "True",
            "payload_off":    "False",
            "device":         device,
            **avail
        }
        if dc:      c["device_class"] = dc
        if icon:    c["icon"] = icon
        if ent_cat: c["entity_category"] = ent_cat
        client.publish(f"homeassistant/binary_sensor/{DEVICE_ID}/{uid}/config",
                       json.dumps(c), retain=True)

    def pub_switch(uid, name, state_tmpl, cmd_topic, icon=None):
        c = {
            "unique_id":      f"{DEVICE_ID}_{uid}",
            "name":           name,
            "state_topic":    state,
            "value_template": state_tmpl,
            "command_topic":  cmd_topic,
            "payload_on":     "ON",
            "payload_off":    "OFF",
            "state_on":       "True",
            "state_off":      "False",
            "device":         device,
            **avail
        }
        if icon: c["icon"] = icon
        client.publish(f"homeassistant/switch/{DEVICE_ID}/{uid}/config",
                       json.dumps(c), retain=True)

    # ── Masuratori principale (din 0x03) ──────────────────────────────────────
    pub_sensor("voltage",  "BMS Tensiune Pack",     "{{ value_json.voltage }}",              "V",  "voltage",  icon="mdi:lightning-bolt",      precision=2)
    pub_sensor("current",  "BMS Curent",            "{{ value_json.current }}",              "A",  "current",  icon="mdi:current-dc",          precision=2)
    pub_sensor("power",    "BMS Putere",            "{{ value_json.power }}",                "W",  "power",    icon="mdi:flash",               precision=1)
    pub_sensor("soc",      "BMS SoC",               "{{ value_json.soc }}",                 "%",  "battery",  icon="mdi:battery")
    pub_sensor("cap_rem",  "BMS Capacitate Ramasa", "{{ value_json.capacity_remaining_ah }}","Ah", None,       icon="mdi:battery-50",          precision=1)
    pub_sensor("cap_nom",  "BMS Capacitate Nominala","{{ value_json.capacity_nominal_ah }}", "Ah", None,       icon="mdi:battery",             precision=1)
    pub_sensor("cycles",   "BMS Cicluri",           "{{ value_json.cycles }}",               None, None, "total_increasing", icon="mdi:cached")

    # ── Temperaturi NTC ───────────────────────────────────────────────────────
    for i in range(max(num_ntc, 1)):
        pub_sensor(f"temp_t{i+1}", f"BMS Temp T{i+1}",
                   f"{{{{ value_json.temperatures[{i}] if value_json.temperatures is defined "
                   f"and value_json.temperatures | length > {i} else None }}}}",
                   "\u00b0C", "temperature", icon="mdi:thermometer")

    # ── Tensiuni celule (din 0x04) ────────────────────────────────────────────
    pub_sensor("cell_min",   "BMS Celula Min",   "{{ value_json.cell_min_mv }}",   "mV", None, icon="mdi:battery-arrow-down-outline")
    pub_sensor("cell_max",   "BMS Celula Max",   "{{ value_json.cell_max_mv }}",   "mV", None, icon="mdi:battery-arrow-up-outline")
    pub_sensor("cell_delta", "BMS Delta Celule", "{{ value_json.cell_delta_mv }}", "mV", None, icon="mdi:delta")
    pub_sensor("cell_avg",   "BMS Celula Medie", "{{ value_json.cell_avg_mv }}",   "mV", None, icon="mdi:battery-medium", precision=1)

    for i in range(num_cells):
        pub_sensor(
            f"cell_{i+1:02d}", f"BMS Celula {i+1:02d}",
            f"{{{{ value_json.cell_voltages[{i}] "
            f"if value_json.cell_voltages is defined "
            f"and value_json.cell_voltages | length > {i} else None }}}}",
            "mV", None, icon="mdi:battery-outline", ent_cat="diagnostic"
        )

    # ── Stare operationala (din 0x03) ─────────────────────────────────────────
    pub_sensor("prot_text",  "BMS Stare Protectie",   "{{ value_json.protection_status_text }}",
               None, None, sc="", icon="mdi:shield-check", ent_cat="diagnostic")
    pub_sensor("sw_version", "BMS Versiune Software",  "{{ value_json.software_version }}",
               None, None, sc="", icon="mdi:chip", ent_cat="diagnostic")
    pub_sensor("hw_version", "BMS Versiune Hardware",   "{{ value_json.hardware_version }}",
               None, None, sc="", icon="mdi:chip", ent_cat="diagnostic")
    pub_sensor("prod_date",  "BMS Data Productie",      "{{ value_json.production_date }}",
               None, None, sc="", icon="mdi:calendar", ent_cat="diagnostic")

    # ── Binary sensors: MOS status ────────────────────────────────────────────
    pub_binary("chg_mos_status", "BMS Charge MOS",    "{{ value_json.charge_mos }}",
               icon="mdi:battery-charging-outline")
    pub_binary("dsg_mos_status", "BMS Discharge MOS", "{{ value_json.discharge_mos }}",
               icon="mdi:battery-minus-outline")

    # ── Binary sensors: protectii ─────────────────────────────────────────────
    pub_binary("protection_active", "BMS Protectie Activa",
               "{{ value_json.protection_active }}", dc="problem", icon="mdi:shield-alert")
    pub_binary("balance_active",    "BMS Echilibrare Activa",
               "{{ value_json.balance_active }}", icon="mdi:scale-balance")

    for bit, (key, desc) in PROT_BITS.items():
        pub_binary(f"prot_{key}", f"BMS {desc}",
                   f"{{{{ value_json.{key} }}}}",
                   dc="problem", icon="mdi:alert-circle", ent_cat="diagnostic")

    # ── Switches: control MOS ─────────────────────────────────────────────────
    pub_switch("chg_mos_ctrl", "BMS Charge MOS Control",
               "{{ value_json.charge_mos }}",
               f"{MQTT_PREFIX}/mos/charge/set", icon="mdi:battery-charging")
    pub_switch("dsg_mos_ctrl", "BMS Discharge MOS Control",
               "{{ value_json.discharge_mos }}",
               f"{MQTT_PREFIX}/mos/discharge/set", icon="mdi:battery-minus")

    log.info(f"Auto-discovery publicat: {num_cells} celule, {num_ntc} NTC, "
             f"{len(PROT_BITS)} biti protectie")

# =============================================================================
# MQTT CALLBACKS
# =============================================================================

def on_message(client, userdata, msg):
    global mos_state, ser_global
    payload = msg.payload.decode(errors='ignore').strip()
    topic   = msg.topic
    log.info(f"MQTT CMD: {topic} = {payload}")

    new_chg = mos_state['charge']
    new_dsg = mos_state['discharge']

    if topic == f"{MQTT_PREFIX}/mos/charge/set":
        new_chg = (payload.upper() == "ON")
    elif topic == f"{MQTT_PREFIX}/mos/discharge/set":
        new_dsg = (payload.upper() == "ON")
    else:
        log.warning(f"Topic necunoscut: {topic}")
        return

    if ser_global:
        ok = send_mos_command(ser_global, new_chg, new_dsg)
        if ok:
            mos_state['charge']    = new_chg
            mos_state['discharge'] = new_dsg
    else:
        log.warning("Serial indisponibil pentru comanda MOS")

# =============================================================================
# MAIN
# =============================================================================

def main():
    global ser_global

    SERIAL_PORT = resolve_serial_port(cfg)

    log.info("=" * 65)
    log.info("  JBD BMS MQTT Bridge v7.0")
    log.info("  Protocol: DD/A5/77 standard (JDB Communication Protocol V4)")
    log.info(f"  Serial: {SERIAL_PORT} @ {BAUD_RATE} bps 8N1")
    log.info(f"  MQTT:   {MQTT_HOST}:{MQTT_PORT} prefix={MQTT_PREFIX}")
    log.info(f"  Cells:  {NUM_CELLS} | Poll: {POLL_INTERVAL}s")
    log.info(f"  Requests:")
    log.info(f"    0x03: {REQ_BASIC.hex(' ').upper()}")
    log.info(f"    0x04: {REQ_CELLS.hex(' ').upper()}")
    log.info(f"    0x05: {REQ_VERSION.hex(' ').upper()}")
    log.info("=" * 65)

    # MQTT
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=DEVICE_ID)
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.will_set(f"{MQTT_PREFIX}/status", "offline", retain=True)
    client.on_message = on_message

    try:
        client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
        client.subscribe(f"{MQTT_PREFIX}/mos/charge/set")
        client.subscribe(f"{MQTT_PREFIX}/mos/discharge/set")
        client.loop_start()
        client.publish(f"{MQTT_PREFIX}/status", "online", retain=True)
        log.info("MQTT OK")
    except Exception as e:
        log.error(f"MQTT eroare: {e}"); sys.exit(1)

    # Serial
    try:
        ser_global = serial.Serial(
            port=SERIAL_PORT, baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0.1
        )
        log.info(f"Serial OK: {SERIAL_PORT} @ {BAUD_RATE}")
    except Exception as e:
        log.error(f"Serial eroare: {e}"); client.loop_stop(); sys.exit(1)

    time.sleep(0.5)

    # Startup: READ 0x05 (versiune hardware - o singura data)
    hw_version = read_hardware_version(ser_global)
    if hw_version:
        log.info(f"Hardware version (0x05): '{hw_version}'")
    else:
        hw_version = "N/A"
        log.warning("0x05 hardware version: nu a raspuns")
    time.sleep(0.2)

    # Prima citire completa
    data = read_all(ser_global)
    if data:
        data['hardware_version'] = hw_version
        publish_discovery(client, data)
        client.publish(f"{MQTT_PREFIX}/state", json.dumps(data, default=str))
        log.info("Prima citire OK")
    else:
        log.warning("Prima citire esuata - auto-discovery cu date implicite")
        publish_discovery(client, {
            'num_cells': NUM_CELLS, 'num_ntc': 2,
            'software_version': '', 'hardware_version': hw_version,
        })

    # Loop principal
    consecutive_errors = 0
    try:
        while True:
            time.sleep(POLL_INTERVAL)
            data = read_all(ser_global)
            if data:
                consecutive_errors = 0
                data['hardware_version'] = hw_version
                client.publish(f"{MQTT_PREFIX}/state", json.dumps(data, default=str))
                client.publish(f"{MQTT_PREFIX}/status", "online", retain=True)
            else:
                consecutive_errors += 1
                log.warning(f"Citire esuata ({consecutive_errors} consecutive)")
                client.publish(f"{MQTT_PREFIX}/status", "degraded", retain=True)
                if consecutive_errors >= 5:
                    log.error("5 erori -> reconectare serial")
                    try:
                        ser_global.close(); time.sleep(3)
                        ser_global = serial.Serial(
                            port=SERIAL_PORT, baudrate=BAUD_RATE,
                            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE, timeout=0.1
                        )
                        consecutive_errors = 0
                        log.info("Serial reconectat")
                    except Exception as e:
                        log.error(f"Reconectare esuata: {e}")

    except KeyboardInterrupt:
        log.info("Oprire")
    finally:
        client.publish(f"{MQTT_PREFIX}/status", "offline", retain=True)
        client.loop_stop()
        if ser_global and ser_global.is_open:
            ser_global.close()


if __name__ == '__main__':
    main()
