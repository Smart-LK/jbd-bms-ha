#!/usr/bin/env python3
"""
jbd_bms_mqtt.py  v7.1 - Protocol 0x78 Proprietar JBD (confirmat functional)
=============================================================================
Dispozitiv confirmat: JBD BMS 9S3P, /dev/ttyUSB1 @ 19200 bps
  Protocolul DD/A5/77 standard (PDF) = TIMEOUT pe acest hardware
  Protocolul 0x78 proprietar = FUNCTIONAL (confirmat scan 2026-05-04)

Protocol 0x78:
  Request: [ADDR] 0x78 [START_H] [START_L] [END_H] [END_L] 0x00 0x00 [CRC_L] [CRC_H]
  Response: [ADDR] 0x78 [START_H] [START_L] [END_H] [END_L] [LEN_H] [LEN_L] [DATA...] [CRC_L] [CRC_H]
  CRC: CRC16 Modbus (poly=0xA001, init=0xFFFF)

Register map confirmat din scan 2026-05-04 (payload 140 bytes @ 0x1000-0x10A0):
  Offset 0-1:   Tensiune totala    x0.01V unsigned  (0x0BFE=3070 -> 30.70V)
  Offset 2-3:   Curent             x0.01A SIGNED    (0x0000=0A; neg=descarcare)
                  !! v6.x citea gresit de la offset 6-7 !!
  Offset 22-23: SOC                %                (0x0064=100%)
  Offset 66-67: Numar celule       uint16           (0x0009=9)
  Offset 68+:   N x tensiune celula mV uint16       (9 celule, suma=30704mV=30.70V)
  Offset 68+N*2:    Numar NTC      uint16           (0x0003=3)
  Offset 68+N*2+2+: M x temperatura uint16          (raw-500)/10 = Celsius
                  !! v6.x folosea formula (raw-500)/10 corect dar offset gresit !!
  Offset 102+:  Device name       ASCII string      (JBD87654321)

Changelog:
  v7.1 - REVERT la protocol 0x78 (confirmat functional pe hardware)
         FIX: curentul = offset 2-3 signed int16 x0.01A
         FIX: SOC citit din offset 22 (0x0064=100%)
         FIX: temperatura = (raw-500)/10 Celsius (confirmat)
         ADAUGAT: parsare completa din registri confirmati
         v7.0 - ELIMINAT (DD/A5/77 = TIMEOUT pe HW-ul testat)
  v6.3 - fix CURRENT_SCALE (obsolet)
  v6.x - protocol 0x78 dar offset curent gresit (bytes[6:8] in loc de bytes[2:4])

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
    "baud_rate":         19200,      # Confirmat 19200 bps pe hardware testat
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
# PROTOCOL 0x78 - CRC16 + REQUEST/RESPONSE
# =============================================================================

def _crc16(data: bytes) -> int:
    """CRC16 Modbus (poly=0xA001, init=0xFFFF, LSB first)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_request_78(addr: int, start_reg: int, end_reg: int) -> bytes:
    frame = bytes([
        addr, 0x78,
        (start_reg >> 8) & 0xFF, start_reg & 0xFF,
        (end_reg   >> 8) & 0xFF, end_reg   & 0xFF,
        0x00, 0x00
    ])
    crc = _crc16(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


REQ_MAIN = build_request_78(0x01, 0x1000, 0x10A0)


def _recv_78_response(ser, slave_addr: int, timeout: float = 4.0) -> bytes | None:
    """
    Citeste raspunsul protocol 0x78.
    Format: [ADDR] [0x78] [START_H] [START_L] [END_H] [END_L] [LEN_H] [LEN_L] [DATA...] [CRC_L] [CRC_H]
    """
    buf = bytearray(); start = time.time(); ignored = 0
    while time.time() - start < timeout:
        chunk = ser.read(256)
        if chunk: buf.extend(chunk)
        i = 0
        while i < len(buf):
            if buf[i] != slave_addr: ignored += 1; i += 1; continue
            rest = buf[i:]
            if len(rest) < 8: break
            if rest[1] != 0x78: ignored += 1; i += 1; continue
            data_len = struct.unpack('>H', rest[6:8])[0]
            if data_len > 512: i += 1; continue
            total = 8 + data_len + 2
            if len(rest) < total: break
            frame = bytes(rest[:total])
            crc_recv = struct.unpack('<H', frame[-2:])[0]
            crc_calc = _crc16(frame[:-2])
            if crc_recv != crc_calc: i += 1; continue
            if ignored > 0: log.debug(f"Bus: ignorat {ignored}b")
            return frame[8:-2]
            i += 1
        time.sleep(0.01)
    if ignored > 0: log.debug(f"Bus: ignorat {ignored}b (timeout)")
    return None

# =============================================================================
# MOS CONTROL - DD 5A E1
# =============================================================================

def _jbd_checksum(payload: bytes) -> tuple[int, int]:
    s = sum(payload) & 0xFFFF; chk = (~s + 1) & 0xFFFF
    return (chk >> 8) & 0xFF, chk & 0xFF


def build_mos_cmd(xx: int) -> bytes:
    data = bytes([0x00, xx & 0xFF])
    chk_h, chk_l = _jbd_checksum(bytes([0xE1, len(data)]) + data)
    return bytes([0xDD, 0x5A, 0xE1, len(data)]) + data + bytes([chk_h, chk_l, 0x77])

# =============================================================================
# PARSE PAYLOAD 0x78
# =============================================================================

def parse_payload_78(data: bytes, num_cells_cfg: int = 9) -> dict | None:
    """
    Parseaza payload-ul raspunsului protocol 0x78.

    Layout confirmat din scan 2026-05-04 (payload 140 bytes):
      [0:2]    Tensiune    unsigned int16, x0.01V
      [2:4]    Curent      SIGNED int16,   x0.01A  (+=charge, -=discharge)
      [22:24]  SOC         uint16, %
      [66:68]  Num_cells   uint16
      [68:68+N*2]    Cell voltages  N x uint16, mV
      [68+N*2:68+N*2+2]  Num_NTC   uint16
      [68+N*2+2+i*2] Temp i   uint16, (raw-500)/10 = Celsius
      [102+]   Device name ASCII
    """
    if not data or len(data) < 70:
        log.warning(f"Payload prea scurt: {len(data) if data else 0} < 70")
        return None

    r = {}
    r['voltage']  = round(struct.unpack('>H', data[0:2])[0] / 100.0, 2)

    # Curent SIGNED int16, unit 0.01A
    # CONFIRMAT: 0x0000=0A la idle/full (scan 2026-05-04) ✓
    current_raw   = struct.unpack('>h', data[2:4])[0]
    r['current']  = round(current_raw / 100.0, 2)
    r['current_raw'] = current_raw
    r['current_dir'] = "charge(+)" if current_raw > 0 else ("discharge(-)" if current_raw < 0 else "idle")

    # SOC: CONFIRMAT offset 22-23: 0x0064=100% ✓
    r['soc'] = struct.unpack('>H', data[22:24])[0] & 0xFF

    # Capacitati (offset 10-13, estimate)
    if len(data) > 14:
        r['capacity_nominal_ah'] = round(struct.unpack('>H', data[10:12])[0] * 10 / 1000, 1)
        r['capacity_full_ah']    = round(struct.unpack('>H', data[12:14])[0] * 10 / 1000, 1)

    # Numar celule: CONFIRMAT offset 66-67: 0x0009=9 ✓
    if len(data) < 68: return r
    num_cells = struct.unpack('>H', data[66:68])[0]
    if num_cells == 0 or num_cells > 32:
        log.warning(f"num_cells={num_cells} invalid, fallback la {num_cells_cfg}")
        num_cells = num_cells_cfg
    r['num_cells'] = num_cells

    cells_end = 68 + num_cells * 2
    if len(data) < cells_end: return r

    # Tensiuni celule: CONFIRMAT suma=30704mV=30.70V ✓
    cells = [struct.unpack('>H', data[68+i*2:70+i*2])[0] for i in range(num_cells)]
    r['cell_voltages'] = cells
    r['cell_min_mv']   = min(cells)
    r['cell_max_mv']   = max(cells)
    r['cell_delta_mv'] = max(cells) - min(cells)
    r['cell_avg_mv']   = round(sum(cells) / len(cells), 1)

    # Temperaturi NTC: formula (raw-500)/10 CONFIRMATA
    # 0x02B2=690 -> (690-500)/10 = 19.0°C ✓ (display BMS: 20-21°C) ✓
    if len(data) > cells_end + 2:
        num_ntc = struct.unpack('>H', data[cells_end:cells_end+2])[0]
        r['num_ntc'] = num_ntc
        temps = []
        for i in range(min(num_ntc, 5)):
            off = cells_end + 2 + i * 2
            if off + 2 <= len(data):
                raw_t = struct.unpack('>H', data[off:off+2])[0]
                temps.append(round((raw_t - 500) / 10.0, 1))
        r['temperatures'] = temps
    else:
        r['temperatures'] = []

    r['power'] = round(r['voltage'] * r['current'], 1)

    # Device name ASCII: CONFIRMAT "JBD87654321" la offset 102 ✓
    if len(data) > 112:
        try:
            name = data[102:120].split(b'\x00')[0].decode('ascii', errors='replace').strip()
            if name and all(32 <= ord(c) < 127 for c in name):
                r['device_name_bms'] = name
        except Exception:
            pass

    return r

# =============================================================================
# BMS READ + MOS
# =============================================================================

serial_lock = threading.Lock()
ser_global  = None
mos_state   = {'charge': True, 'discharge': True}


def read_bms(ser) -> dict | None:
    with serial_lock:
        ser.reset_input_buffer(); ser.write(REQ_MAIN)
        time.sleep(0.3)
        payload = _recv_78_response(ser, slave_addr=0x01, timeout=4.0)
    if payload is None:
        log.warning("0x78: fara raspuns"); return None
    data = parse_payload_78(payload, num_cells_cfg=NUM_CELLS)
    if data is None:
        log.warning("0x78: parsare esuata"); return None
    data['charge_mos']    = mos_state['charge']
    data['discharge_mos'] = mos_state['discharge']
    log.info(f"V={data.get('voltage')}V I={data.get('current')}A ({data.get('current_dir')}) "
             f"P={data.get('power')}W SoC={data.get('soc')}% "
             f"CHG={'ON' if data.get('charge_mos') else 'OFF'} DSG={'ON' if data.get('discharge_mos') else 'OFF'}")
    if data.get('cell_voltages'):
        log.info(f"Celule: min={data['cell_min_mv']}mV max={data['cell_max_mv']}mV "
                 f"delta={data['cell_delta_mv']}mV avg={data['cell_avg_mv']}mV")
    if data.get('temperatures'):
        log.info(f"Temp: {data['temperatures']} C")
    return data


def send_mos_command(ser, charge_on: bool, discharge_on: bool) -> bool:
    xx = 0x00
    if not charge_on:    xx |= 0x01
    if not discharge_on: xx |= 0x02
    cmd = build_mos_cmd(xx)
    log.info(f"MOS: CHG={'ON' if charge_on else 'OFF'} DSG={'ON' if discharge_on else 'OFF'} "
             f"XX=0x{xx:02X} TX: {cmd.hex(' ').upper()}")
    with serial_lock:
        ser.reset_input_buffer(); ser.write(cmd)
        time.sleep(0.3); ser.timeout = 1.5
        resp = bytearray(ser.read(16))
    if resp:
        log.info(f"MOS raspuns: {resp.hex(' ').upper()}")
        if len(resp) >= 7 and resp[0]==0xDD and resp[1]==0xE1 and resp[2]==0x00:
            log.info("MOS OK"); return True
        log.warning("MOS raspuns neclar")
    else: log.warning("MOS: fara confirmare")
    return False

# =============================================================================
# PORT SERIAL
# =============================================================================

def resolve_serial_port(cfg: dict) -> str:
    ports = {}
    by_id = "/dev/serial/by-id/"
    if os.path.isdir(by_id):
        for link in sorted(glob.glob(by_id + "*")):
            try: ports[os.path.basename(link)] = os.path.realpath(link)
            except: pass
    log.info("--- Porturi seriale ---")
    for name, path in ports.items(): log.info(f"  by-id: {name} -> {path}")
    tty = sorted(glob.glob("/dev/ttyUSB*"))
    if tty: log.info(f"  ttyUSB: {', '.join(tty)}")
    log.info("-----------------------")
    by_id_name = cfg.get("serial_port_by_id", "").strip()
    if by_id_name:
        p = f"/dev/serial/by-id/{by_id_name}"
        if os.path.exists(p):
            log.info(f"Folosesc by-id: {by_id_name} -> {os.path.realpath(p)}")
            return p
        log.warning(f"by-id '{by_id_name}' nu exista!")
    return cfg["serial_port"]

# =============================================================================
# MQTT DISCOVERY
# =============================================================================

def publish_discovery(client, data: dict):
    num_cells = data.get('num_cells', NUM_CELLS)
    num_ntc   = len(data.get('temperatures', []))
    device    = {"identifiers": [DEVICE_ID], "name": DEVICE_NAME,
                 "manufacturer": "JBD", "model": f"9S3P ({num_cells}S)",
                 "hw_version": data.get('device_name_bms', '')}
    avail = {"availability_topic": f"{MQTT_PREFIX}/status",
             "payload_available": "online", "payload_not_available": "offline"}
    state = f"{MQTT_PREFIX}/state"

    def pub_sensor(uid, name, tmpl, unit=None, dc=None, sc="measurement",
                   icon=None, ent_cat=None, precision=None):
        c = {"unique_id": f"{DEVICE_ID}_{uid}", "name": name, "state_topic": state,
             "value_template": tmpl, "device": device, "state_class": sc, **avail}
        if unit:      c["unit_of_measurement"] = unit
        if dc:        c["device_class"] = dc
        if icon:      c["icon"] = icon
        if ent_cat:   c["entity_category"] = ent_cat
        if precision: c["suggested_display_precision"] = precision
        client.publish(f"homeassistant/sensor/{DEVICE_ID}/{uid}/config", json.dumps(c), retain=True)

    def pub_binary(uid, name, tmpl, dc=None, icon=None, ent_cat=None):
        c = {"unique_id": f"{DEVICE_ID}_{uid}", "name": name, "state_topic": state,
             "value_template": tmpl, "payload_on": "True", "payload_off": "False",
             "device": device, **avail}
        if dc:      c["device_class"] = dc
        if icon:    c["icon"] = icon
        if ent_cat: c["entity_category"] = ent_cat
        client.publish(f"homeassistant/binary_sensor/{DEVICE_ID}/{uid}/config", json.dumps(c), retain=True)

    def pub_switch(uid, name, state_tmpl, cmd_topic, icon=None):
        c = {"unique_id": f"{DEVICE_ID}_{uid}", "name": name, "state_topic": state,
             "value_template": state_tmpl, "command_topic": cmd_topic,
             "payload_on": "ON", "payload_off": "OFF",
             "state_on": "True", "state_off": "False", "device": device, **avail}
        if icon: c["icon"] = icon
        client.publish(f"homeassistant/switch/{DEVICE_ID}/{uid}/config", json.dumps(c), retain=True)

    # Masuratori principale
    pub_sensor("voltage", "BMS Tensiune Pack",       "{{ value_json.voltage }}",           "V",  "voltage", icon="mdi:lightning-bolt", precision=2)
    pub_sensor("current", "BMS Curent",              "{{ value_json.current }}",           "A",  "current", icon="mdi:current-dc",    precision=2)
    pub_sensor("power",   "BMS Putere",              "{{ value_json.power }}",             "W",  "power",   icon="mdi:flash",         precision=1)
    pub_sensor("soc",     "BMS SoC",                 "{{ value_json.soc }}",              "%",  "battery", icon="mdi:battery")
    pub_sensor("cap_nom", "BMS Capacitate Nominala", "{{ value_json.capacity_nominal_ah }}","Ah", None,      icon="mdi:battery",       precision=1)
    pub_sensor("cap_full","BMS Capacitate Full",     "{{ value_json.capacity_full_ah }}",  "Ah", None,      icon="mdi:battery-check", precision=1)

    # Temperaturi NTC
    for i in range(max(num_ntc, 1)):
        pub_sensor(f"temp_t{i+1}", f"BMS Temp T{i+1}",
                   f"{{{{ value_json.temperatures[{i}] if value_json.temperatures is defined "
                   f"and value_json.temperatures | length > {i} else None }}}}",
                   "\u00b0C", "temperature", icon="mdi:thermometer")

    # Tensiuni celule
    pub_sensor("cell_min",   "BMS Celula Min",   "{{ value_json.cell_min_mv }}",   "mV", None, icon="mdi:battery-arrow-down-outline")
    pub_sensor("cell_max",   "BMS Celula Max",   "{{ value_json.cell_max_mv }}",   "mV", None, icon="mdi:battery-arrow-up-outline")
    pub_sensor("cell_delta", "BMS Delta Celule", "{{ value_json.cell_delta_mv }}", "mV", None, icon="mdi:delta")
    pub_sensor("cell_avg",   "BMS Celula Medie", "{{ value_json.cell_avg_mv }}" ,  "mV", None, icon="mdi:battery-medium", precision=1)
    for i in range(num_cells):
        pub_sensor(f"cell_{i+1:02d}", f"BMS Celula {i+1:02d}",
                   f"{{{{ value_json.cell_voltages[{i}] if value_json.cell_voltages is defined "
                   f"and value_json.cell_voltages | length > {i} else None }}}}",
                   "mV", None, icon="mdi:battery-outline", ent_cat="diagnostic")

    # Stare
    pub_sensor("device_name_bms", "BMS HW Name", "{{ value_json.device_name_bms }}",
               None, None, sc="", icon="mdi:chip", ent_cat="diagnostic")
    pub_binary("chg_mos_status", "BMS Charge MOS",    "{{ value_json.charge_mos }}",    icon="mdi:battery-charging-outline")
    pub_binary("dsg_mos_status", "BMS Discharge MOS", "{{ value_json.discharge_mos }}", icon="mdi:battery-minus-outline")
    pub_switch("chg_mos_ctrl", "BMS Charge MOS Control",
               "{{ value_json.charge_mos }}", f"{MQTT_PREFIX}/mos/charge/set", icon="mdi:battery-charging")
    pub_switch("dsg_mos_ctrl", "BMS Discharge MOS Control",
               "{{ value_json.discharge_mos }}", f"{MQTT_PREFIX}/mos/discharge/set", icon="mdi:battery-minus")

    log.info(f"Auto-discovery publicat: {num_cells} celule, {num_ntc} NTC")

# =============================================================================
# MQTT CALLBACKS
# =============================================================================

def on_message(client, userdata, msg):
    global mos_state, ser_global
    payload = msg.payload.decode(errors='ignore').strip(); topic = msg.topic
    log.info(f"MQTT CMD: {topic} = {payload}")
    new_chg = mos_state['charge']; new_dsg = mos_state['discharge']
    if topic == f"{MQTT_PREFIX}/mos/charge/set": new_chg = (payload.upper() == "ON")
    elif topic == f"{MQTT_PREFIX}/mos/discharge/set": new_dsg = (payload.upper() == "ON")
    else: log.warning(f"Topic necunoscut: {topic}"); return
    if ser_global:
        ok = send_mos_command(ser_global, new_chg, new_dsg)
        if ok: mos_state['charge'] = new_chg; mos_state['discharge'] = new_dsg
    else: log.warning("Serial indisponibil")

# =============================================================================
# MAIN
# =============================================================================

def main():
    global ser_global
    SERIAL_PORT = resolve_serial_port(cfg)
    log.info("="*65); log.info("  JBD BMS MQTT Bridge v7.1")
    log.info("  Protocol: 0x78 proprietar (confirmat functional)")
    log.info(f"  Serial: {SERIAL_PORT} @ {BAUD_RATE} bps")
    log.info(f"  REQ_MAIN: {REQ_MAIN.hex(' ').upper()}"); log.info("="*65)

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
    except Exception as e: log.error(f"MQTT: {e}"); sys.exit(1)

    try:
        ser_global = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0.1)
        log.info(f"Serial OK: {SERIAL_PORT} @ {BAUD_RATE}")
    except Exception as e: log.error(f"Serial: {e}"); client.loop_stop(); sys.exit(1)

    time.sleep(0.5)
    data = read_bms(ser_global)
    if data:
        publish_discovery(client, data)
        client.publish(f"{MQTT_PREFIX}/state", json.dumps(data, default=str))
        log.info("Prima citire OK")
    else:
        log.warning("Prima citire esuata")
        publish_discovery(client, {'num_cells': NUM_CELLS, 'temperatures': [0,0,0]})

    consecutive_errors = 0
    try:
        while True:
            time.sleep(POLL_INTERVAL)
            data = read_bms(ser_global)
            if data:
                consecutive_errors = 0
                client.publish(f"{MQTT_PREFIX}/state", json.dumps(data, default=str))
                client.publish(f"{MQTT_PREFIX}/status", "online", retain=True)
            else:
                consecutive_errors += 1
                log.warning(f"Citire esuata ({consecutive_errors})")
                client.publish(f"{MQTT_PREFIX}/status", "degraded", retain=True)
                if consecutive_errors >= 5:
                    log.error("5 erori -> reconectare")
                    try:
                        ser_global.close(); time.sleep(3)
                        ser_global = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE,
                            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE, timeout=0.1)
                        consecutive_errors = 0; log.info("Serial reconectat")
                    except Exception as e: log.error(f"Reconectare: {e}")
    except KeyboardInterrupt: log.info("Oprire")
    finally:
        client.publish(f"{MQTT_PREFIX}/status", "offline", retain=True)
        client.loop_stop()
        if ser_global and ser_global.is_open: ser_global.close()

if __name__ == '__main__': main()
