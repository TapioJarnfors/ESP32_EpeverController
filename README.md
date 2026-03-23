# ESP32 Epever Controller

ESP32-based master controller for the Epever Tracer 1210A solar charge controller, designed for maintenance charging of tractor batteries.

## Overview

This project uses an ESP32 to communicate with an Epever Tracer 1210A charge controller via Modbus RTU protocol. The system monitors and manages the charging process for tractor battery maintenance.

## Hardware

- **Microcontroller**: ESP32
- **Charge Controller**: Epever Tracer 1210A
- **Communication**: RS-485 (Modbus RTU)
  - UART2 RX: GPIO 16
  - UART2 TX: GPIO 17
  - Baud rate: 115200, 8N1

## Features

- Modbus RTU communication with Epever Tracer 1210A
- Real-time solar panel, battery, and load monitoring
- MQTT integration with Home Assistant autodiscovery
- Dual MQTT broker support:
  - **Mosquitto** (local/LAN, no TLS)
  - **HiveMQ Cloud** (cloud-based, TLS encrypted)
- WiFi connectivity
- 14 sensor metrics published to MQTT
- CRC16 validation and frame monitoring

## Building

Prerequisites: ESP-IDF v5.5.2

```bash
idf.py build
idf.py -p COM8 flash monitor
```

## Configuration

### First-Time Setup

1. **Configure your credentials** (required before first build):
```bash
idf.py menuconfig
```
Navigate to: `Epever Controller Configuration`

2. **Set WiFi credentials**:
   - WiFi SSID: Your network name
   - WiFi Password: Your network password

3. **Choose MQTT broker** and configure:
   
   **Option A: Mosquitto (Local/LAN)**
   - Select "Mosquitto (Local/LAN)"
   - Set MQTT Broker URI (e.g., `mqtt://192.168.1.210:1883`)
   - Set username/password if your broker requires authentication
   
   **Option B: HiveMQ Cloud**
   - Select "HiveMQ Cloud"
   - Set HiveMQ Cloud Host (e.g., `xxxxxxxx.s1.eu.hivemq.cloud`)
   - Set Port: `8883` (default)
   - Set HiveMQ Username and Password from your HiveMQ Cloud console

4. **Save and exit** menuconfig (press S, then Q)

### Security Note

⚠️ **Your credentials are stored locally in `sdkconfig`** which is git-ignored.
- `sdkconfig` - Contains your actual credentials (NOT committed to git)
- `sdkconfig.defaults` - Contains placeholder values (committed to git)

Never commit `sdkconfig` with real credentials to version control.

## Documentation

See [.doc/EpeverChargeControllerModbusRegisterMap.md](.doc/EpeverChargeControllerModbusRegisterMap.md) for Modbus register details.

## Project Status

Work in progress - to be continued!
