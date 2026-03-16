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

- Modbus RTU frame sniffing and monitoring
- Real-time frame timestamp tracking
- CRC16 validation
- Binary data logging over USB serial (460800 baud)
- Hex dump output for debugging

## Building

Prerequisites: ESP-IDF v5.5.2

```bash
idf.py build
idf.py -p COM8 flash monitor
```

## Configuration

Monitor baud rate is set to 115200 in `sdkconfig`:
```
CONFIG_ESPTOOLPY_MONITOR_BAUD=115200
```

Debug logging can be enabled/disabled in [main.c](main/main.c):
```c
#define DEBUG true  // Set to false for production
```

## Documentation

See [.doc/EpeverChargeControllerModbusRegisterMap.md](.doc/EpeverChargeControllerModbusRegisterMap.md) for Modbus register details.

## Project Status

Work in progress - to be continued!
