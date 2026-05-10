#!/usr/bin/env python3
"""
jbd_bms_mqtt.py  v2.0  -  JBD SRNE Broadcast Pasiv -> MQTT
============================================================
Port confirmat: /dev/ttyUSB1 @ 19200 bps, MOD PASIV (fara TX)

Protocol identificat din log 2026-05-10 (102/102 CRC OK):
---------------------------------------------------------------
FC=0x45 NU exista in protocolul SRNE standard (SRNE-MODBUS.pdf v3.9).
Functiile SRNE standard sunt: 03H, 06H, 10H, 78H, 79H.
=> Acesta este un broadcast PROPRIETAR JBD in modul compatibilitate SRNE.
   Semnificatia campurilor word1/word2/word3 este NECUNOSCUTA din documentatie.

Frame fix de 10 bytes, CRC16 Modbus (poly=0xA001, LSB first) VALIDAT:

  Byte 0:    pack_addr  uint8   adresa pack (0x02..0x0F), cicleaza x3 per val
  Byte 1:    0x45       marker fix FC proprietar JBD-SRNE
  Bytes 2-3: word1      uint16 big-endian, valoare constanta: 0x0000 (=0)
  Bytes 4-5: word2      uint16 big-endian, valoare constanta: 0x0054 (=84)
  Bytes 6-7: word3      uint16 big-endian, valoare constanta: 0x0000 (=0)
  Bytes 8-9: CRC16 Modbus al bytes 0-7 (LSB first)

  Semnificatia word1/word2/word3: NECUNOSCUTA - nu se presupune nimic.
  Valorile sunt publicate ca decimal si hex pentru analiza ulterioara.

MQTT publicat (prefix/state - JSON):
  pack_addr        adresa ultimului frame valid (uint8 decimal)
  pack_addr_hex    adresa hex, ex: "0x05"
  pack_addrs_seen  lista tuturor adreselor distincte vazute, ex: [2,3,4,...,15]
  word1_dec        bytes 2-3 uint16 decimal
  word1_hex        bytes 2-3 hex, ex: "0x0000"
  word2_dec        bytes 4-5 uint16 decimal
  word2_hex        bytes 4-5 hex, ex: "0x0054"
  word3_dec        bytes 6-7 uint16 decimal
  word3_hex        bytes 6-7 hex, ex: "0x0000"
  frame_hex        frame complet 10 bytes, ex: "05 45 00 00 00 54 00 00 D5 20"
  crc_ok_count     total frame-uri CRC valid de la pornire
  crc_fail_count   total frame-uri CRC invalid de la pornire
  frames_per_min   rata medie frame-uri/minut
  last_seen        timestamp ISO ultima receptie valida

Autor: Smart-LK / Claude Sonnet, mai 2026
"""

import json
import logging
import os
import struct
import sys
import time
from datetime import datetime

import paho.mqtt.client as mqtt
import serial

# =============================================================================
# CONFIG
# =============================================================================

DEFAULTS = {
    "serial_port":      "/dev/ttyUSB1",
    "baud_rate":        19200,
    "mqtt_host":        "core-mosquitto",
    "mqtt_port":        1883,
    "mqtt_user":        "mqtt_local",
    "mqtt_password":    "mqtt2026vidra",
    "mqtt_prefix":      "jbd_bms",
    "device_name":      "Acumulator JBD",
    "device_id":        "jbd_bms_vidra",
    "publish_interval": 10,   # secunde intre publicari MQTT
    "log_level":        "INFO",
}


def load_config() -> dict:
    cfg = dict(DEFAULTS)
    path = "/data/options.json"
    if os.path.exists(path):
        try:
            with open(path) as f:
                cfg.update(json.load(f))
        except Exception as e:
            print(f"[WARN] options.json: {e}, folosesc defaults")
    return cfg


cfg              = load_config()
SERIAL_PORT      = cfg["serial_port"]
BAUD_RATE        = int(cfg["baud_rate"])
MQTT_HOST        = cfg["mqtt_host"]
MQTT_PORT        = int(cfg["mqtt_port"])
MQTT_USER        = cfg["mqtt_user"]
MQTT_PASS        = cfg["mqtt_password"]
MQTT_PREFIX      = cfg["mqtt_prefix"]
DEVICE_NAME      = cfg["device_name"]
DEVICE_ID        = cfg["device_id"]
PUBLISH_INTERVAL = int(cfg["publish_interval"])

log_level = getattr(logging, cfg.get("log_level", "INFO").upper(), logging.INFO)
logging.basicConfig(
    level=log_level,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger(__name__)

# =============================================================================
# CRC16 MODBUS
# =============================================================================

def crc16_modbus(data: bytes) -> int:
    """CRC16 Modbus RTU: poly=0xA001, init=0xFFFF, LSB first in frame."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

# =============================================================================
# FRAME DETECTION
# =============================================================================

def extract_frames(buf: bytearray) -> tuple:
    """
    Cauta frame-uri JBD-SRNE de 10 bytes cu CRC16 Modbus valid.
    Criteriu de detectie: buf[i+1] == 0x45 si CRC valid.
    Returneaza (lista_frame_uri, buffer_ramas).
    """
    frames = []
    i = 0
    while i <= len(buf) - 10:
        if buf[i + 1] != 0x45:
            i += 1
            continue
        candidate = bytes(buf[i:i + 10])
        crc_recv  = struct.unpack("<H", candidate[8:10])[0]
        crc_calc  = crc16_modbus(candidate[:8])
        if crc_recv == crc_calc:
            frames.append({
                "raw":       candidate,
                "pack_addr": candidate[0],
                "word1":     struct.unpack(">H", candidate[2:4])[0],
                "word2":     struct.unpack(">H", candidate[4:6])[0],
                "word3":     struct.unpack(">H", candidate[6:8])[0],
            })
            buf = buf[i + 10:]
            i   = 0
        else:
            i += 1
    return frames, buf

# =============================================================================
# MQTT DISCOVERY
# =============================================================================

def publish_discovery(client: mqtt.Client):
    device = {
        "identifiers":  [DEVICE_ID],
        "name":         DEVICE_NAME,
        "manufacturer": "JBD",
        "model":        "JBD SRNE broadcast",
    }
    avail = {
        "availability_topic":    f"{MQTT_PREFIX}/status",
        "payload_available":     "online",
        "payload_not_available": "offline",
    }
    st = f"{MQTT_PREFIX}/state"

    def sensor(uid, name, tmpl, unit=None, dc=None, icon=None, ent_cat=None):
        c = {
            "unique_id":      f"{DEVICE_ID}_{uid}",
            "name":           name,
            "state_topic":    st,
            "value_template": tmpl,
            "state_class":    "measurement",
            "device":         device,
            **avail,
        }
        if unit:    c["unit_of_measurement"] = unit
        if dc:      c["device_class"]         = dc
        if icon:    c["icon"]                  = icon
        if ent_cat: c["entity_category"]       = ent_cat
        client.publish(
            f"homeassistant/sensor/{DEVICE_ID}/{uid}/config",
            json.dumps(c), retain=True
        )

    # Campuri publicate - fara interpretare, doar raw
    sensor("pack_addr",    "JBD SRNE Pack Addr",
           "{{ value_json.pack_addr }}", icon="mdi:identifier")
    sensor("word1_dec",    "JBD SRNE Word1 (dec)",
           "{{ value_json.word1_dec }}", icon="mdi:numeric")
    sensor("word2_dec",    "JBD SRNE Word2 (dec)",
           "{{ value_json.word2_dec }}", icon="mdi:numeric")
    sensor("word3_dec",    "JBD SRNE Word3 (dec)",
           "{{ value_json.word3_dec }}", icon="mdi:numeric")
    sensor("crc_ok_count", "JBD SRNE CRC OK total",
           "{{ value_json.crc_ok_count }}", icon="mdi:check-circle-outline",
           ent_cat="diagnostic")
    sensor("frames_per_min", "JBD SRNE Frames/min",
           "{{ value_json.frames_per_min }}", icon="mdi:timer-outline",
           ent_cat="diagnostic")

    log.info("MQTT Discovery publicat")

# =============================================================================
# MAIN LOOP
# =============================================================================

def main():
    log.info("=" * 60)
    log.info("  JBD BMS MQTT Bridge v2.0")
    log.info("  Protocol: JBD-SRNE broadcast pasiv (FC=0x45, 10 bytes)")
    log.info(f"  Port: {SERIAL_PORT} @ {BAUD_RATE} bps")
    log.info("  MOD PASIV - nu se trimite nimic pe port!")
    log.info("=" * 60)

    # --- MQTT ---
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=DEVICE_ID)
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.will_set(f"{MQTT_PREFIX}/status", "offline", retain=True)
    try:
        client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
        client.loop_start()
        client.publish(f"{MQTT_PREFIX}/status", "online", retain=True)
        log.info("MQTT conectat OK")
    except Exception as e:
        log.error(f"MQTT eroare: {e}")
        sys.exit(1)

    # --- Serial ---
    try:
        ser = serial.Serial(
            port=SERIAL_PORT, baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0.1,
        )
        log.info(f"Port serial deschis: {SERIAL_PORT} @ {BAUD_RATE}")
    except Exception as e:
        log.error(f"Serial eroare: {e}")
        client.loop_stop()
        sys.exit(1)

    # --- State ---
    buf             = bytearray()
    crc_ok_count    = 0
    crc_fail_count  = 0
    addrs_seen      = set()
    last_frame      = None      # ultimul frame valid
    last_publish    = 0.0
    start_time      = time.time()
    discovery_done  = False

    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)

            frames, buf = extract_frames(buf)

            for frm in frames:
                crc_ok_count += 1
                addrs_seen.add(frm["pack_addr"])
                last_frame = frm
                log.debug(
                    f"Frame OK: addr=0x{frm['pack_addr']:02X} "
                    f"W1={frm['word1']}(0x{frm['word1']:04X}) "
                    f"W2={frm['word2']}(0x{frm['word2']:04X}) "
                    f"W3={frm['word3']}(0x{frm['word3']:04X})"
                )

            # Publica periodic
            now = time.time()
            if last_frame and (now - last_publish) >= PUBLISH_INTERVAL:
                if not discovery_done:
                    publish_discovery(client)
                    discovery_done = True

                elapsed_min = (now - start_time) / 60.0
                fpm = round(crc_ok_count / elapsed_min, 1) if elapsed_min > 0 else 0

                frm  = last_frame
                raw  = frm["raw"]
                state = {
                    "pack_addr":     frm["pack_addr"],
                    "pack_addr_hex": f"0x{frm['pack_addr']:02X}",
                    "pack_addrs_seen": sorted(list(addrs_seen)),
                    "word1_dec":     frm["word1"],
                    "word1_hex":     f"0x{frm['word1']:04X}",
                    "word2_dec":     frm["word2"],
                    "word2_hex":     f"0x{frm['word2']:04X}",
                    "word3_dec":     frm["word3"],
                    "word3_hex":     f"0x{frm['word3']:04X}",
                    "frame_hex":     raw.hex(" ").upper(),
                    "crc_ok_count":  crc_ok_count,
                    "crc_fail_count": crc_fail_count,
                    "frames_per_min": fpm,
                    "last_seen":     datetime.now().isoformat(timespec="seconds"),
                }

                client.publish(f"{MQTT_PREFIX}/state", json.dumps(state), retain=True)
                client.publish(f"{MQTT_PREFIX}/status", "online", retain=True)
                last_publish = now

                log.info(
                    f"Publicat: addr=0x{frm['pack_addr']:02X} "
                    f"W1={frm['word1']}  W2={frm['word2']}  W3={frm['word3']}  "
                    f"CRC_OK={crc_ok_count}  fpm={fpm}"
                )

            # Buffer blocat fara frame-uri: curata dupa 1KB
            if len(buf) > 1024:
                log.warning(f"Buffer >1KB fara frame-uri detectate, resetez. "
                            f"Primii 20 bytes: {bytes(buf[:20]).hex(' ').upper()}")
                buf = bytearray()

            time.sleep(0.02)

    except KeyboardInterrupt:
        log.info("Oprire manuala")
    finally:
        client.publish(f"{MQTT_PREFIX}/status", "offline", retain=True)
        client.loop_stop()
        if ser.is_open:
            ser.close()
        log.info(f"Stop. Total frame-uri CRC OK: {crc_ok_count}, FAIL: {crc_fail_count}")


if __name__ == "__main__":
    main()
