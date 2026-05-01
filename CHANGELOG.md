# CHANGELOG

## [1.3.0] - 2026-05-01
### Adăugat
- `serial_port_by_id`: identificare port serial prin `/dev/serial/by-id/` — stabil la reboot
- Scan automat al tuturor porturilor seriale la pornire, vizibil în Log tab
- `README.md` cu instrucțiuni instalare și configurare
- `repository.json` pentru HA addon repository

### Modificat
- `config.yaml` v1.3.0
- `jbd_bms_mqtt.py` v6.2

---

## [1.2.0] - 2026-05-01
### Adăugat
- Toate opțiunile configurabile din HA Configuration tab
- `serial_port`, `baud_rate`, `poll_interval`, `num_cells`, `mqtt_*`, `device_*`, `log_level`

### Reparat
- Fix `KeyError` la `options.json` incomplet: pattern `DEFAULTS + cfg.update(loaded)`

---

## [1.1.0] - 2026-04-30
### Adăugat
- Prima versiune cu `options` parțiale în `config.yaml`

---

## [1.0.0] - 2026-04-30
### Inițial
- Config hardcodat în script
- Protocol JBD 0x78, control MOS DD A5
- MQTT auto-discovery HA
