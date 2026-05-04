# JBD BMS MQTT HA Addon - Changelog

## [7.0.0] - 2026-05-04

### Rescris complet - Protocol DD/A5/77 standard

#### Fixed (bug-uri critice eliminate)
- **CURENTUL ERA COMPLET GRESIT** in toate versiunile anterioare:
  - v6.x citea `data[6:8]` = Nominal Capacity (nu Current!)
  - Aplica `CURRENT_OFFSET=37403` si `CURRENT_SCALE=114.2` inventate
  - Rezultat: 7.64A in HA vs 4.1A pe display BMS (1.86x eroare)
- Protocol schimbat de la proprietar `0x78` (Modbus-like) la **DD/A5/77 standard**
- Checksum: `(~sum + 1) & 0xFFFF` conform protocol (verificat cu exemplele din PDF)

#### Added
- **READ 0x03** complet conform protocol:
  - Curent: `signed int16 big-endian`, unit `10mA`, pozitiv=incarcare
  - Formula: `current_A = struct.unpack('>h', data[2:4])[0] / 100.0`
  - Toate campurile: voltage, current, power, SoC, capacitate, cicluri
  - Data productie (format packed 2-byte JBD)
  - Balance status (bit per celula)
  - Status protectie (16 biti, 13 definiti)
  - Versiune software (nibble high.nibble low)
  - Status FET (bit0=CHG, bit1=DSG)
  - Temperaturi NTC (unit 0.1K absolut -> Celsius)
- **READ 0x04** - tensiuni individuale celule (unit mV, big-endian unsigned)
  - cell_min_mv, cell_max_mv, cell_delta_mv, cell_avg_mv
  - Senzori HA per celula (C01..C09)
- **READ 0x05** - versiune hardware (ASCII string, citit la startup)
- **WRITE 0xE1** - control MOS (DD 5A E1 02 00 XX CHK_H CHK_L 77)
  - XX=0x00: release all (normal)
  - XX=0x01: CHG off
  - XX=0x02: DSG off
  - XX=0x03: both off
- Binary sensors pentru **toti biti de protectie** (13 protectii definite)
- Senzori diagnostici: hardware_version, software_version, production_date

#### Changed
- Baud rate default: 9600 (standard protocol; daca BMS e la alta rata, schimba in config)
- Frame parser cu resync pe 0xDD + verificare stop byte 0x77
- Checksum non-fatal (log warning, nu rejecteaza date)
- Reconectare automata serial dupa 5 erori consecutive

---

## [6.3] - 2026-05-04
- Fix CURRENT_SCALE 114.2 -> 212.8 (paliativ, eliminat in v7.0)

## [6.2] - 2026-05-03
- serial_port_by_id

## [6.0] - 2026-05-02
- Config din /data/options.json

## [5.0] - 2026-05-01
- Control MOS DD A5

## [1.0] - 2026-05-01
- Versiune initiala
