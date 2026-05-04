"""
jbd_bms_mqtt.py  v6.3 - Protocol JBD 0x78 + Control MOS DD A5
=============================================================
- Citeste date BMS via protocol 0x78 broadcast
- Publica senzori + binary_sensor MOS in HA via MQTT
- Control MOS (charge/discharge ON/OFF) via comanda DD A5 0x5A 0xE1
- MQTT switch pentru control din HA
- Config din /data/options.json (HA addon) sau constante hardcodate (fallback)
- Suport serial_port_by_id: foloseste /dev/serial/by-id/ pentru port fix

Changelog:
  v6.3 - FIX: CURRENT_SCALE corectat de la 114.2 la 212.8
         Calibrat din masuratori reale: BMS display=4.1A, HA=7.64A (2026-05-02)
         Calcul: raw-offset = 7.64*114.2 = 872.5; SCALE = 872.5/4.1 = 212.8
         Power = voltage * current se corecteaza automat (123W vs 230W anterior)
         NOTA: CURRENT_OFFSET (37403) presupus corect (calibrat la 0A anterior)
         Recomandat: verificare cu clampmetru la un curent cunoscut
  v6.2 - serial_port_by_id, scan porturi la startup
  v6.1 - fix KeyError options.json cu DEFAULTS + update()
  v6.0 - config din /data/options.json, log_level configurabil
  v5.0 - control MOS DD A5, MQTT switch

Comanda control MOS (DD A5 write):
  DD 5A E1 02 00 [XX] [CHK_H] [CHK_L] 77
  XX: 0x00=release all, 0x01=disable CHG, 0x02=disable DSG, 0x03=disable both
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

# --- CALIBRARE HARDWARE (constante specifice acestui BMS) ---------------------
#
# CURRENT_OFFSET: valoarea raw ADC cand curentul e 0A (calibrat anterior)
# CURRENT_SCALE:  LSB/A - cat de multi "pasi" ADC corespund unui Amper
#
# Istoric calibrare:
#   v6.2 si anterior: SCALE=114.2 -> dadea 7.64A cand BMS display arata 4.1A
#   v6.3: SCALE=212.8 -> calculat din discrepanta observata 2026-05-02:
#     raw_estimat = 7.64 * 114.2 + 37403 = 38275.5
#     SCALE_corect = (38275.5 - 37403) / 4.1 = 872.5 / 4.1 = 212.8
#
# Verificare recomandata: masurare cu clampmetru la curent cunoscut
# si ajustare CURRENT_SCALE = (raw - CURRENT_OFFSET) / I_real_A
#
CURRENT_OFFSET = 37403   # raw ADC la 0A (nemodificat)
CURRENT_SCALE  = 212.8   # LSB/A (corectat v6.3: era 114.2)
# ------------------------------------------------------------------------------

# --- DEFAULTS -----------------------------------------------------------------
DEFAULTS = {
    "serial_port":       "/dev/ttyUSB0",
    "serial_port_by_id": "",
    "baud_rate":         19200,
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
                loaded = json.load(f)
            cfg.update(loaded)
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

# ------------------------------------------------------------------------------

log_level = getattr(logging, cfg.get("log_level", "INFO").upper(), logging.INFO)
logging.basicConfig(
    level=log_level,
    format='%(asctime)s | %(levelname)s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[logging.StreamHandler(sys.stdout)]
)
log = logging.getLogger(__name__)

# --- REZOLVARE PORT SERIAL ----------------------------------------------------

def scan_serial_ports() -> dict:
    result = {}
    by_id_dir = "/dev/serial/by-id/"
    if os.path.isdir(by_id_dir):
        for link in sorted(glob.glob(by_id_dir + "*")):
            try:
                target = os.path.realpath(link)
                result[os.path.basename(link)] = target
            except Exception:
                pass
    return result


def resolve_serial_port(cfg: dict) -> str:
    ports_by_id = scan_serial_ports()

    log.info("--- Dispozitive seriale disponibile ---")
    if ports_by_id:
        for name, path in ports_by_id.items():
            log.info(f"  by-id: {name} -> {path}")
    else:
        log.info("  /dev/serial/by-id/ gol sau indisponibil")

    tty_list = sorted(glob.glob("/dev/ttyUSB*"))
    if tty_list:
        log.info(f"  ttyUSB: {', '.join(tty_list)}")
    log.info("---------------------------------------")

    by_id_name = cfg.get("serial_port_by_id", "").strip()
    if by_id_name:
        by_id_path = f"/dev/serial/by-id/{by_id_name}"
        if os.path.exists(by_id_path):
            resolved = os.path.realpath(by_id_path)
            log.info(f"Folosesc by-id: {by_id_name} -> {resolved}")
            return by_id_path
        else:
            log.warning(f"serial_port_by_id '{by_id_name}' nu exista!")
            log.warning(f"Disponibile: {list(ports_by_id.keys())}")
            log.warning(f"Fallback la serial_port: {cfg['serial_port']}")

    return cfg["serial_port"]

# ------------------------------------------------------------------------------

mos_state = {'charge': True, 'discharge': True}
serial_lock = threading.Lock()
ser_global = None

# --- CRC16 MODBUS -------------------------------------------------------------

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

# --- PROTOCOL 0x78 READ -------------------------------------------------------

def build_request_78(addr, start_reg, end_reg):
    frame = bytes([
        addr, 0x78,
        (start_reg >> 8) & 0xFF, start_reg & 0xFF,
        (end_reg   >> 8) & 0xFF, end_reg   & 0xFF,
        0x00, 0x00
    ])
    crc = crc16(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

REQ_MAIN = build_request_78(0x01, 0x1000, 0x10A0)


def read_response_78(ser, timeout=4.0):
    ser.timeout = 0.1
    buf = bytearray()
    start = time.time()

    while time.time() - start < timeout:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)

        while len(buf) >= 2:
            if buf[0] == 0x01 and buf[1] == 0x78:
                break
            buf = buf[1:]

        if len(buf) < 8:
            continue

        data_len = struct.unpack('>H', buf[6:8])[0]
        total = 8 + data_len + 2

        if total > 512:
            buf = buf[1:]
            continue

        if len(buf) < total:
            continue

        frame = bytes(buf[:total])
        buf = buf[total:]

        crc_recv = struct.unpack('<H', frame[-2:])[0]
        crc_calc = crc16(frame[:-2])
        if crc_recv != crc_calc:
            log.warning(f"CRC gresit: 0x{crc_recv:04X} != 0x{crc_calc:04X}")
            continue

        return frame[8:-2]

    return None

# --- PROTOCOL DD A5 CONTROL MOS -----------------------------------------------

def da_a5_checksum(data: bytes) -> bytes:
    s = sum(data) & 0xFFFF
    chk = (~s + 1) & 0xFFFF
    return bytes([(chk >> 8) & 0xFF, chk & 0xFF])


def build_mos_command(charge_on: bool, discharge_on: bool) -> bytes:
    xx = 0x00
    if not charge_on:
        xx |= 0x01
    if not discharge_on:
        xx |= 0x02
    payload = bytes([0xE1, 0x02, 0x00, xx])
    chk = da_a5_checksum(payload)
    return bytes([0xDD, 0x5A]) + payload + chk + bytes([0x77])


def send_mos_command(ser, charge_on: bool, discharge_on: bool):
    cmd = build_mos_command(charge_on, discharge_on)
    log.info(f"MOS CMD: CHG={'ON' if charge_on else 'OFF'} DSG={'ON' if discharge_on else 'OFF'} | TX: {cmd.hex(' ').upper()}")
    with serial_lock:
        ser.reset_input_buffer()
        ser.write(cmd)
        time.sleep(0.5)
        ser.timeout = 1.0
        resp = bytearray(ser.read(16))
        if resp:
            if 0xDD in resp and 0xE1 in resp:
                log.info(f"MOS confirmat: {resp.hex(' ').upper()}")
            else:
                log.warning(f"Raspuns MOS neclar: {resp.hex(' ').upper()}")
        else:
            log.warning("Fara confirmare MOS")

# --- PARSE PAYLOAD ------------------------------------------------------------

def parse_payload(data: bytes) -> dict:
    if not data or len(data) < 94:
        log.warning(f"Payload scurt: {len(data) if data else 0} bytes")
        return {}

    r = {}
    r['voltage'] = struct.unpack('>H', data[0:2])[0] / 100.0

    # Curent: raw ADC 16-bit unsigned, calibrat cu CURRENT_OFFSET si CURRENT_SCALE
    # Pozitiv = descarcare, negativ = incarcare (conventie JBD 0x78)
    # v6.3: CURRENT_SCALE corectat la 212.8 (era 114.2, dadea ~2x prea mare)
    current_raw = struct.unpack('>H', data[6:8])[0]
    r['current']     = round((current_raw - CURRENT_OFFSET) / CURRENT_SCALE, 2)
    r['current_raw'] = current_raw

    r['soc'] = round(struct.unpack('>H', data[8:10])[0] / 100.0, 1)
    r['capacity_remaining'] = struct.unpack('>H', data[10:12])[0] / 100.0
    r['capacity_full']      = struct.unpack('>H', data[12:14])[0] / 100.0
    r['capacity_nominal']   = struct.unpack('>H', data[14:16])[0] / 100.0

    def temp(raw): return round((raw - 500) / 10.0, 1)
    r['temp_mos']     = temp(struct.unpack('>H', data[16:18])[0])
    r['temp_ambient'] = temp(struct.unpack('>H', data[18:20])[0])
    r['soh']          = struct.unpack('>H', data[22:24])[0]
    r['cycles']       = struct.unpack('>H', data[36:38])[0]

    num_cells = struct.unpack('>H', data[66:68])[0]
    r['num_cells'] = num_cells

    cells = []
    for i in range(min(num_cells, 16)):
        off = 68 + i * 2
        if off + 2 <= len(data):
            cells.append(struct.unpack('>H', data[off:off+2])[0])
    r['cell_voltages'] = cells
    if cells:
        r['cell_min_mv']   = min(cells)
        r['cell_max_mv']   = max(cells)
        r['cell_delta_mv'] = max(cells) - min(cells)

    temps_cell = []
    for i in range(3):
        off = 88 + i * 2
        if off + 2 <= len(data):
            temps_cell.append(temp(struct.unpack('>H', data[off:off+2])[0]))
    r['temperatures'] = temps_cell

    # Power calculat din tensiune si curent (automat corectat cu CURRENT_SCALE nou)
    r['power']         = round(r['voltage'] * r['current'], 1)
    r['charge_mos']    = mos_state['charge']
    r['discharge_mos'] = mos_state['discharge']

    return r


def read_bms(ser) -> dict:
    with serial_lock:
        ser.reset_input_buffer()
        ser.write(REQ_MAIN)
        time.sleep(0.3)
        payload = read_response_78(ser, timeout=4.0)

    if not payload:
        log.warning("Fara raspuns la request principal")
        return {}

    data = parse_payload(payload)
    if data:
        log.info(f"Pack: {data.get('voltage')}V | I={data.get('current')}A | "
                 f"SoC={data.get('soc')}% | P={data.get('power')}W | "
                 f"raw_I={data.get('current_raw')} | "
                 f"Delta={data.get('cell_delta_mv')}mV | "
                 f"CHG={'ON' if data.get('charge_mos') else 'OFF'} DSG={'ON' if data.get('discharge_mos') else 'OFF'}")
    return data

# --- MQTT DISCOVERY -----------------------------------------------------------

def publish_discovery(client, num_cells, num_temps):
    device = {
        "identifiers": [DEVICE_ID],
        "name": DEVICE_NAME,
        "manufacturer": "JBD",
        "model": f"9S3P ({num_cells}S)"
    }
    avail = {
        "availability_topic": f"{MQTT_PREFIX}/status",
        "payload_available": "online",
        "payload_not_available": "offline"
    }

    def pub_sensor(uid, name, tmpl, unit=None, dc=None, sc="measurement", icon=None):
        c = {"unique_id": f"{DEVICE_ID}_{uid}", "name": name,
             "state_topic": f"{MQTT_PREFIX}/state", "value_template": tmpl,
             "device": device, "state_class": sc, **avail}
        if unit: c["unit_of_measurement"] = unit
        if dc:   c["device_class"] = dc
        if icon: c["icon"] = icon
        client.publish(f"homeassistant/sensor/{DEVICE_ID}/{uid}/config", json.dumps(c), retain=True)

    pub_sensor("voltage",    "BMS Tensiune Pack",       "{{ value_json.voltage }}",            "V",   "voltage")
    pub_sensor("current",    "BMS Curent",              "{{ value_json.current }}",            "A",   "current")
    pub_sensor("soc",        "BMS SoC",                 "{{ value_json.soc }}",               "%",   "battery")
    pub_sensor("soh",        "BMS SoH",                 "{{ value_json.soh }}",               "%",   None, icon="mdi:heart-pulse")
    pub_sensor("power",      "BMS Putere",              "{{ value_json.power }}",              "W",   "power")
    pub_sensor("cycles",     "BMS Cicluri",             "{{ value_json.cycles }}",             None,  None, icon="mdi:cached")
    pub_sensor("cap_rem",    "BMS Capacitate Ramasa",   "{{ value_json.capacity_remaining }}", "Ah",  None, icon="mdi:battery-50")
    pub_sensor("cap_full",   "BMS Capacitate Totala",   "{{ value_json.capacity_full }}",      "Ah",  None, icon="mdi:battery")
    pub_sensor("temp_mos",   "BMS Temp MOS",            "{{ value_json.temp_mos }}",           "°C",  "temperature")
    pub_sensor("temp_amb",   "BMS Temp Ambient",        "{{ value_json.temp_ambient }}",       "°C",  "temperature")
    pub_sensor("cell_min",   "BMS Celula Min",          "{{ value_json.cell_min_mv }}",        "mV",  None, icon="mdi:battery-arrow-down")
    pub_sensor("cell_max",   "BMS Celula Max",          "{{ value_json.cell_max_mv }}",        "mV",  None, icon="mdi:battery-arrow-up")
    pub_sensor("cell_delta", "BMS Delta Celule",        "{{ value_json.cell_delta_mv }}",      "mV",  None, icon="mdi:delta")
    pub_sensor("current_raw","BMS Curent Raw ADC",      "{{ value_json.current_raw }}",        None,  None, "measurement", icon="mdi:chip")

    for i in range(min(num_temps, 3)):
        pub_sensor(f"temp_t{i+1}", f"BMS Temp T{i+1}",
                   f"{{{{ value_json.temperatures[{i}] }}}}", "°C", "temperature")

    for i in range(num_cells):
        pub_sensor(f"cell_{i+1}", f"BMS Celula {i+1}",
                   f"{{{{ value_json.cell_voltages[{i}] }}}}", "mV", None, icon="mdi:battery-outline")

    def pub_binary(uid, name, tmpl, icon=None):
        c = {"unique_id": f"{DEVICE_ID}_{uid}", "name": name,
             "state_topic": f"{MQTT_PREFIX}/state", "value_template": tmpl,
             "payload_on": True, "payload_off": False, "device": device, **avail}
        if icon: c["icon"] = icon
        client.publish(f"homeassistant/binary_sensor/{DEVICE_ID}/{uid}/config", json.dumps(c), retain=True)

    pub_binary("chg_mos_status", "BMS Charge MOS Status", "{{ value_json.charge_mos }}", "mdi:electric-switch")
    pub_binary("dsg_mos_status", "BMS Discharge MOS Status", "{{ value_json.discharge_mos }}", "mdi:electric-switch")

    def pub_switch(uid, name, cmd_topic, state_tmpl, icon=None):
        c = {"unique_id": f"{DEVICE_ID}_{uid}", "name": name,
             "state_topic": f"{MQTT_PREFIX}/state", "value_template": state_tmpl,
             "command_topic": cmd_topic, "payload_on": "ON", "payload_off": "OFF",
             "state_on": True, "state_off": False, "device": device, **avail}
        if icon: c["icon"] = icon
        client.publish(f"homeassistant/switch/{DEVICE_ID}/{uid}/config", json.dumps(c), retain=True)

    pub_switch("chg_mos_ctrl", "BMS Charge MOS",
               f"{MQTT_PREFIX}/mos/charge/set", "{{ value_json.charge_mos }}", "mdi:battery-charging")
    pub_switch("dsg_mos_ctrl", "BMS Discharge MOS",
               f"{MQTT_PREFIX}/mos/discharge/set", "{{ value_json.discharge_mos }}", "mdi:battery-minus")

    log.info("Auto-discovery publicat")

# --- MQTT CALLBACKS -----------------------------------------------------------

def on_message(client, userdata, msg):
    global mos_state, ser_global
    payload = msg.payload.decode().strip()
    topic   = msg.topic
    log.info(f"CMD primit: {topic} = {payload}")

    if topic == f"{MQTT_PREFIX}/mos/charge/set":
        mos_state['charge'] = (payload == "ON")
        send_mos_command(ser_global, mos_state['charge'], mos_state['discharge'])
    elif topic == f"{MQTT_PREFIX}/mos/discharge/set":
        mos_state['discharge'] = (payload == "ON")
        send_mos_command(ser_global, mos_state['charge'], mos_state['discharge'])

    client.publish(f"{MQTT_PREFIX}/mos/charge/state", "ON" if mos_state['charge'] else "OFF", retain=True)
    client.publish(f"{MQTT_PREFIX}/mos/discharge/state", "ON" if mos_state['discharge'] else "OFF", retain=True)

# --- MAIN ---------------------------------------------------------------------

def main():
    global ser_global

    SERIAL_PORT = resolve_serial_port(cfg)

    log.info("=" * 60)
    log.info(f"  JBD BMS MQTT Bridge v6.3")
    log.info(f"  Serial: {SERIAL_PORT} @ {BAUD_RATE} bps 8N1")
    log.info(f"  MQTT:   {MQTT_HOST}:{MQTT_PORT} prefix={MQTT_PREFIX}")
    log.info(f"  Cells:  {NUM_CELLS}")
    log.info(f"  Calibrare: OFFSET={CURRENT_OFFSET} SCALE={CURRENT_SCALE} LSB/A")
    log.info(f"  TX: {REQ_MAIN.hex(' ').upper()}")
    log.info("=" * 60)

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
        log.error(f"MQTT eroare: {e}")
        sys.exit(1)

    try:
        ser_global = serial.Serial(
            port=SERIAL_PORT, baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0.1
        )
        log.info(f"Serial OK: {SERIAL_PORT}")
    except Exception as e:
        log.error(f"Serial eroare: {e}")
        client.loop_stop()
        sys.exit(1)

    time.sleep(1)

    data = read_bms(ser_global)
    if data:
        publish_discovery(client, data.get('num_cells', NUM_CELLS), len(data.get('temperatures', [])))
        client.publish(f"{MQTT_PREFIX}/state", json.dumps(data))
        log.info("Prima citire OK!")
    else:
        log.warning("Prima citire esuata.")
        publish_discovery(client, NUM_CELLS, 3)

    try:
        while True:
            time.sleep(POLL_INTERVAL)
            data = read_bms(ser_global)
            if data:
                client.publish(f"{MQTT_PREFIX}/state", json.dumps(data))
                client.publish(f"{MQTT_PREFIX}/status", "online", retain=True)
            else:
                log.warning("Citire esuata")
                client.publish(f"{MQTT_PREFIX}/status", "degraded", retain=True)

    except KeyboardInterrupt:
        log.info("Oprire")
    finally:
        client.publish(f"{MQTT_PREFIX}/status", "offline", retain=True)
        client.loop_stop()
        ser_global.close()


if __name__ == '__main__':
    main()
