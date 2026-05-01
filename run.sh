#!/bin/sh
set -e
echo "[JBD BMS] Pornire..."
while true; do
    python3 /jbd_bms_mqtt.py || true
    echo "[JBD BMS] Script oprit, restart in 10 secunde..."
    sleep 10
done
