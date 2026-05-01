# JBD BMS MQTT — Home Assistant Addon

Addon Home Assistant pentru citirea datelor de la un BMS JBD via RS485 și publicarea lor în MQTT cu auto-discovery complet.

---

## Hardware testat

| Componentă | Model |
|------------|-------|
| BMS | JBD SP15S001 (protocol 0x78 broadcast) |
| Acumulator | 9S3P LiFePO4 |
| Adaptor serial | FTDI FT232R USB-UART (recomandat) |
| Alternativ | CH340 USB-Serial (fără serial number unic — nu suportă `by-id`) |

---

## Schema de conectare BMS → HA

```
┌─────────────────────────────────────────────────────────┐
│                    BMS JBD SP15S001                      │
│                                                         │
│   Port RS485 / UART                                     │
│   ┌──────┐                                              │
│   │ TX   │──────────────────────────────┐               │
│   │ RX   │──────────────────────────┐   │               │
│   │ GND  │───────────────────────┐  │   │               │
│   └──────┘                       │  │   │               │
└─────────────────────────────────│──│───│───────────────┘
                                  │  │   │
                                  │  │   │
┌─────────────────────────────────│──│───│───────────────┐
│   Adaptor USB-Serial (FTDI)     │  │   │               │
│   ┌──────┐                      │  │   │               │
│   │ GND  │──────────────────────┘  │   │               │
│   │ RX   │─────────────────────────┘   │               │
│   │ TX   │──────────────────────────────┘               │
│   │ USB  │                                              │
│   └──────┘                                              │
└──────────────────────────────────────────────────────────┘
         │
         │ USB
         ▼
┌─────────────────────────────────────────────────────────┐
│   Home Assistant OS (Dell Wyse 5070)                    │
│   /dev/serial/by-id/                                    │
│   usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0          │
│                                                         │
│   ┌──────────────────────────────────────────────┐      │
│   │  Addon: JBD BMS MQTT                         │      │
│   │  Protocol: 0x78 broadcast, 19200bps 8N1      │      │
│   │  Poll: 30s                                   │      │
│   └──────────────────┬───────────────────────────┘      │
│                      │ MQTT                             │
│   ┌──────────────────▼───────────────────────────┐      │
│   │  Broker: core-mosquitto                       │      │
│   │  Topic: jbd_bms/state                        │      │
│   └──────────────────┬───────────────────────────┘      │
│                      │                                  │
│   ┌──────────────────▼───────────────────────────┐      │
│   │  Home Assistant                               │      │
│   │  Auto-discovery → Entități HA                │      │
│   └──────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────┘
```

### Pinout BMS JBD (port comunicație)

| Pin BMS | Semnal | Conectare |
|---------|--------|-----------|
| TX | Date ieșire BMS | → RX adaptor serial |
| RX | Date intrare BMS | → TX adaptor serial |
| GND | Masă | → GND adaptor serial |

> **Atenție:** BMS-ul JBD folosește UART TTL (3.3V sau 5V), nu RS485 diferențial. Adaptorul USB-Serial simplu (FTDI/CH340) este suficient — nu este nevoie de convertor RS485.

---

## Instalare în Home Assistant

### 1. Adaugă repository-ul
Settings → Add-ons → Add-on Store → ⋮ → Repositories → adaugă:
```
https://github.com/Smart-LK/jbd-bms-ha
```

### 2. Instalează addon-ul
Refresh → apare **JBD BMS MQTT** → Install → Rebuild dacă e necesar

### 3. Configurare
Înainte de pornire, mergi la tab-ul **Configuration** și setează:

| Câmp | Descriere | Valoare implicită |
|------|-----------|-------------------|
| `serial_port` | Port fallback (dacă `by-id` e gol) | `/dev/ttyUSB0` |
| `serial_port_by_id` | **Recomandat** — nume stabil din `/dev/serial/by-id/` | *(gol)* |
| `baud_rate` | Viteză comunicație | `19200` |
| `poll_interval` | Interval citire în secunde | `30` |
| `num_cells` | Număr celule BMS | `9` |
| `mqtt_host` | Broker MQTT | `core-mosquitto` |
| `mqtt_port` | Port broker MQTT | `1883` |
| `mqtt_user` | User MQTT | `mqtt_local` |
| `mqtt_password` | Parolă MQTT | — |
| `mqtt_prefix` | Prefix topic MQTT | `jbd_bms` |
| `device_name` | Nume dispozitiv în HA | `Acumulator JBD` |
| `device_id` | ID unic dispozitiv | `jbd_bms_vidra` |
| `log_level` | Nivel logare | `INFO` |

### 4. Identificare port serial (recomandat)

Din **SSH terminal HA**:
```bash
ls /dev/serial/by-id/
```

Exemplu output:
```
usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0
usb-1a86_USB_Serial-if00-port0
```

Copiezi numele FTDI-ului în câmpul `serial_port_by_id`. Astfel portul rămâne fix la orice reboot, indiferent de ordinea de enumerare USB.

> **De ce `by-id`?** Adaptoarele CH340 (chip chinezesc ieftin) nu au serial number unic și nu apar în `by-id`. La reboot pot schimba numărul (`ttyUSB0` ↔ `ttyUSB1`). Adaptoarele FTDI au serial number unic și sunt întotdeauna identificabile stabil.

---

## Entități publicate în Home Assistant

### Senzori
| Entitate | Unitate | Descriere |
|----------|---------|-----------|
| BMS Tensiune Pack | V | Tensiunea totală a pack-ului |
| BMS Curent | A | Curent (+ = descărcare, - = încărcare) |
| BMS SoC | % | State of Charge |
| BMS SoH | % | State of Health |
| BMS Putere | W | Putere instantanee |
| BMS Cicluri | — | Număr cicluri complete |
| BMS Capacitate Rămasă | Ah | Capacitate disponibilă |
| BMS Capacitate Totală | Ah | Capacitate totală curentă |
| BMS Temp MOS | °C | Temperatura tranzistoarelor MOS |
| BMS Temp Ambient | °C | Temperatura ambientală BMS |
| BMS Temp T1/T2/T3 | °C | Temperaturi celule |
| BMS Celula Min | mV | Tensiunea minimă pe celulă |
| BMS Celula Max | mV | Tensiunea maximă pe celulă |
| BMS Delta Celule | mV | Diferența max-min între celule |
| BMS Celula 1..9 | mV | Tensiune individuală fiecare celulă |

### Switch-uri (control MOS)
| Entitate | Descriere |
|----------|-----------|
| BMS Charge MOS | Activare/dezactivare încărcare |
| BMS Discharge MOS | Activare/dezactivare descărcare |

> **Prerequisit control MOS:** Comutatoarele funcționează doar dacă BMS-ul suportă comanda `DD A5 E1`. Verifică în Log tab că răspunsul MOS e confirmat.

---

## Protocol comunicație

- **Protocol citire:** JBD 0x78 broadcast request → răspuns cu date complete pack
- **Protocol scriere MOS:** DD A5 write (comanda `0x5A 0xE1`)
- **Viteză:** 19200 bps, 8N1, fără flow control
- **Request principal:** `01 78 10 00 10 A0 00 00 7F B2`

---

## Changelog

Vezi [CHANGELOG.md](CHANGELOG.md)
