# Epever Charge Controller Modbus Register Map

This document extracts all Modbus register addresses and their meanings from the provided Python driver code and official Epever protocol documentation.

## Real-Time Data (Input Registers - Function Code 0x04)

| Register Address | Method / Name | Description | Scale/Format |
|-----------------|--------|-------------|--------------|
| **PV Array** |
| `0x3100` | `get_solar_voltage()` | PV array input voltage | ÷100 (Volts) |
| `0x3101` | `get_solar_current()` | PV array input current | ÷100 (Amps) |
| `0x3102-0x3103` | `get_solar_power()` | PV array input power (32-bit) | ÷100 (Watts) |
| **Battery** |
| `0x3106-0x3107` | `get_battery_power()` | Battery charging power (32-bit) | ÷100 (Watts) |
| `0x3110` | `get_battery_temperature()` | Battery temperature | ÷100, signed (°C) |
| `0x3111` | Not in driver | Temperature inside controller | ÷100, signed (°C) |
| `0x3112` | Not in driver | Power components temperature | ÷100, signed (°C) |
| `0x311A` | `get_battery_state_of_charge()` | Battery state of charge | ÷100 (%) |
| `0x311B` | `get_remote_battery_temperature()` | Remote battery temperature sensor | ÷100, signed (°C) |
| `0x311D` | `get_battery_real_rated_voltage()` | Current system rated voltage (12V=1200, 24V=2400, etc.) | ÷100 (Volts) |
| `0x3200` | `get_battery_status()` | Battery status register | bitmapped |
| `0x331A` | `get_battery_voltage()` | Battery voltage | ÷100 (Volts) |
| `0x331B-0x331C` | `get_battery_current()` | Net battery current (+ = charging, - = discharging) (32-bit) | ÷100, signed (Amps) |
| `0x331D` | Not in driver | Battery temperature (alternate address) | ÷100, signed (°C) |
| `0x331E` | Not in driver | Ambient temperature | ÷100, signed (°C) |
| **Load** |
| `0x310C` | `get_load_voltage()` | Load output voltage | ÷100 (Volts) |
| `0x310D` | `get_load_current()` | Load output current | ÷100 (Amps) |
| `0x310E-0x310F` | `get_load_power()` | Load output power (32-bit) | ÷100 (Watts) |
| **Historical Data (Today)** |
| `0x3300` | `get_maximum_pv_voltage_today()` | Maximum PV voltage today | ÷100 (Volts) |
| `0x3301` | `get_minimum_pv_voltage_today()` | Minimum PV voltage today | ÷100 (Volts) |
| `0x3302` | `get_maximum_battery_voltage_today()` | Maximum battery voltage today | ÷100 (Volts) |
| `0x3303` | `get_minimum_battery_voltage_today()` | Minimum battery voltage today | ÷100 (Volts) |
| `0x3304-0x3305` | `get_consumed_energy_today()` | Consumed energy today (32-bit) | ÷100 (kWh) |
| `0x330C-0x330D` | `get_generated_energy_today()` | Generated energy today (32-bit) | ÷100 (kWh) |
| **Historical Data (Month/Year)** |
| `0x3306-0x3307` | `get_consumed_energy_this_month()` | Consumed energy this month (32-bit) | ÷100 (kWh) |
| `0x3308-0x3309` | `get_consumed_energy_this_year()` | Consumed energy this year (32-bit) | ÷100 (kWh) |
| `0x330A-0x330B` | `get_total_consumed_energy()` | Total consumed energy (lifetime) (32-bit) | ÷100 (kWh) |
| `0x330E-0x330F` | `get_generated_energy_this_month()` | Generated energy this month (32-bit) | ÷100 (kWh) |
| `0x3310-0x3311` | `get_generated_energy_this_year()` | Generated energy this year (32-bit) | ÷100 (kWh) |
| `0x3312-0x3313` | `get_total_generated_energy()` | Total generated energy (lifetime) (32-bit) | ÷100 (kWh) |
| `0x3314-0x3315` | Not in driver | Carbon dioxide reduction (32-bit) | ÷100 (Tons) |
| **Status Registers** |
| `0x2000` | `is_device_over_temperature()` | Bit 2: Over temperature inside device | bit read |
| `0x200C` | `is_night()` | Day/night status (1=Night, 0=Day) | bit read |
| `0x3201` | `get_charging_equipment_status()` | Charging equipment status | bitmapped |
| `0x3202` | `get_discharging_equipment_status()` | Discharging equipment status | bitmapped |

---

## Device Information & Ratings (Holding Registers - Function Code 0x03)

| Register Address | Method / Name | Description | Scale/Format |
|-----------------|--------|-------------|--------------|
| **PV Array Ratings** |
| `0x3000` | Not in driver | PV array rated voltage | ÷100 (Volts) |
| `0x3001` | Not in driver | PV array rated current | ÷100 (Amps) |
| `0x3002-0x3003` | Not in driver | PV array rated power (32-bit) | ÷100 (Watts) |
| **Battery Ratings** |
| `0x3004` | Not in driver | Battery rated voltage | ÷100 (Volts) |
| `0x3005` | `get_rated_charging_current()` | Rated charging current to battery | ÷100 (Amps) |
| `0x3006-0x3007` | Not in driver | Battery rated power (32-bit) | ÷100 (Watts) |
| `0x3008` | Not in driver | Charging mode (0=Disc, 1=PWM, 2=MPPT) | raw |
| **Load Ratings** |
| `0x300E` | `get_rated_load_current()` | Rated load current | ÷100 (Amps) |
| **Battery Configuration** |
| `0x9000` | `get_battery_type()` | Battery type setting (0=User, 1=Sealed, 2=GEL, 3=Flooded) | mapped to names |
| `0x9001` | `get/set_battery_capacity()` | Battery capacity | raw (Ah) |
| `0x9002` | `get/set_temperature_compensation_coefficient()` | Temperature compensation coefficient | ÷100 (mV/°C/2V) |
| `0x9067` | `get_battery_rated_voltage()` | Battery rated voltage code (0=Auto, 1=12V, 2=24V, 3=36V, 4=48V, 5=60V, 6=110V, 7=120V, 8=220V, 9=240V) | mapped to voltages |
| **Battery Voltage Control Registers** |
| `0x9003` | `get_over_voltage_disconnect_voltage()` | Over voltage disconnect | ÷100 (Volts) |
| `0x9004` | `get_charging_limit_voltage()` | Charging limit voltage | ÷100 (Volts) |
| `0x9005` | `get_over_voltage_reconnect_voltage()` | Over voltage reconnect | ÷100 (Volts) |
| `0x9006` | `get_equalize_charging_voltage()` | Equalize charging voltage | ÷100 (Volts) |
| `0x9007` | `get_boost_charging_voltage()` | Boost charging voltage | ÷100 (Volts) |
| `0x9008` | `get_float_charging_voltage()` | Float charging voltage | ÷100 (Volts) |
| `0x9009` | `get_boost_reconnect_charging_voltage()` | Boost reconnect voltage | ÷100 (Volts) |
| `0x900A` | `get_low_voltage_reconnect_voltage()` | Low voltage reconnect | ÷100 (Volts) |
| `0x900B` | `get_under_voltage_recover_voltage()` | Under voltage recover | ÷100 (Volts) |
| `0x900C` | `get_under_voltage_warning_voltage()` | Under voltage warning | ÷100 (Volts) |
| `0x900D` | `get_low_voltage_disconnect_voltage()` | Low voltage disconnect | ÷100 (Volts) |
| `0x900E` | `get_discharging_limit_voltage()` | Discharging limit voltage | ÷100 (Volts) |
| **Temperature Settings** |
| `0x9017` | Not in driver | Battery temperature warning upper limit | ÷100 (°C) |
| `0x9018` | Not in driver | Battery temperature warning lower limit | ÷100 (°C) |
| `0x9019` | Not in driver | Controller inner temperature upper limit | ÷100 (°C) |
| `0x901A` | Not in driver | Controller inner temperature recovery | ÷100 (°C) |
| `0x901B` | Not in driver | Power component temperature upper limit | ÷100 (°C) |
| `0x901C` | Not in driver | Power component temperature recovery | ÷100 (°C) |
| **Timing & Day/Night Settings** |
| `0x9013-0x9015` | `get/set_rtc()` | Real time clock (sec/min, hour/day, month/year) | bitmapped |
| `0x9016` | Not in driver | Equalize charging cycle interval | raw (days) |
| `0x901E` | Not in driver | Night Time Threshold Voltage (NTTV) | ÷100 (Volts) |
| `0x901F` | Not in driver | Night delay time | raw (minutes) |
| `0x9020` | Not in driver | Day Time Threshold Voltage (DTTV) | ÷100 (Volts) |
| `0x9021` | Not in driver | Day delay time | raw (minutes) |
| `0x9065` | Not in driver | Length of night (hour/minute) | bitmapped |
| **Load Control Settings** |
| `0x903D` | Not in driver | Load control modes (0=Manual, 1=Light ON/OFF, 2=Light ON+Timer, 3=Time Control) | raw |
| `0x903E` | Not in driver | Working time length 1 (hour/minute) | bitmapped |
| `0x903F` | Not in driver | Working time length 2 (hour/minute) | bitmapped |
| `0x9042-0x9044` | Not in driver | Turn on timing 1 (sec/min/hour) | raw |
| `0x9045-0x9047` | Not in driver | Turn off timing 1 (sec/min/hour) | raw |
| `0x9048-0x904A` | Not in driver | Turn on timing 2 (sec/min/hour) | raw |
| `0x904B-0x904D` | Not in driver | Turn off timing 2 (sec/min/hour) | raw |
| `0x9069` | Not in driver | Load timing control selection (0=one timer, 1=two timers) | raw |
| **Charging Parameters** |
| `0x906B` | `get_equalize_duration()` | Equalize duration | raw (minutes) |
| `0x906C` | `get_boost_duration()` | Boost duration | raw (minutes) |
| `0x906D` | `get_battery_discharge()` | Battery discharge percentage setting | ÷100 (%) |
| `0x906E` | `get_battery_charge()` | Battery charge percentage setting | ÷100 (%) |
| `0x9070` | `get_charging_mode()` | Management mode (0=Voltage compensation, 1=SOC) | raw |
| **Miscellaneous** |
| `0x901D` | Not in driver | Line impedance | ÷100 (milliohm) |
| `0x9063` | Not in driver | LCD backlight time | raw (seconds) |
| `0x9066` | Not in driver | Main power supply configuration (1=Battery, 2=AC-DC) | raw |
| `0x906A` | `get_default_load_on_off_in_manual_mode()` | Default load state in manual mode | 0=OFF, 1=ON |

---

## Control Coils (Function Code 0x05/0x01)

| Address | Name | Description | Values |
|---------|------|-------------|--------|
| `0x00` | Charging device on/off | Enable/disable charging | 0=OFF, 1=ON |
| `0x01` | Output control mode | Manual/automatic mode selection | 0=Auto, 1=Manual |
| `0x02` | Manual load control | Control load when in manual mode | 0=OFF, 1=ON |
| `0x03` | Default load control | Default load state | 0=OFF, 1=ON |
| `0x05` | Load test mode | Enable load test mode | 0=Disable, 1=Enable |
| `0x06` | Force load on/off | Temporary load control for testing | 0=OFF, 1=ON |
| `0x0D` | Restore system defaults | Reset to factory defaults | 0=No, 1=Yes |
| `0x0E` | Clear statistics | Reset energy generation statistics | 0=No, 1=Yes |

---

## Bitmapped Register Details

### Battery Status Register (`0x3200`)

| Bits | Field | Values |
|------|-------|--------|
| D15 | Wrong identification for rated voltage | 0=Normal, 1=Error |
| D8 | Battery inner resistance abnormal | 0=Normal, 1=Abnormal |
| D7-4 | Temperature warning status | 0=NORMAL, 1=OVER_TEMP, 2=LOW_TEMP |
| D3-0 | Battery status | 0=NORMAL, 1=OVER_VOLTAGE, 2=UNDER_VOLTAGE, 3=OVER_DISCHARGE, 4=FAULT |

### Charging Equipment Status (`0x3201`)

| Bits | Field | Values |
|------|-------|--------|
| D15-14 | Input voltage status | 0=NORMAL, 1=NO_INPUT_POWER, 2=HIGHER_INPUT, 3=INPUT_VOLTAGE_ERROR |
| D13 | Charging MOSFET short circuit | 0=Normal, 1=Short |
| D12 | Charging/anti-reverse MOSFET open circuit | 0=Normal, 1=Open |
| D11 | Anti-reverse MOSFET short circuit | 0=Normal, 1=Short |
| D10 | Input over current | 0=Normal, 1=Over current |
| D9 | Load over current | 0=Normal, 1=Over current |
| D8 | Load short circuit | 0=Normal, 1=Short |
| D7 | Load MOSFET short circuit | 0=Normal, 1=Short |
| D6 | Disequilibrium in three circuits | 0=Normal, 1=Error |
| D4 | PV input short circuit | 0=Normal, 1=Short |
| D3-2 | Charging status | 0=NO_CHARGING, 1=FLOAT, 2=BOOST, 3=EQUALIZATION |
| D1 | Fault | 0=No fault, 1=Fault |
| D0 | Running | 0=Standby, 1=Running |

### Discharging Equipment Status (`0x3202`)

| Bits | Field | Values |
|------|-------|--------|
| D15-14 | Output voltage status | 0=NORMAL, 1=LOW, 2=HIGH, 3=NO_ACCESS |
| D13-12 | Output power level | 0=LIGHT, 1=MODERATE, 2=RATED, 3=OVERLOAD |
| D11 | Short circuit | 0=Normal, 1=Short |
| D10 | Unable to discharge | 0=Normal, 1=Cannot discharge |
| D9 | Unable to stop discharging | 0=Normal, 1=Cannot stop |
| D8 | Output voltage abnormal | 0=Normal, 1=Abnormal |
| D7 | Input over voltage | 0=Normal, 1=Over voltage |
| D6 | High voltage side short circuit | 0=Normal, 1=Short |
| D5 | Boost over voltage | 0=Normal, 1=Over voltage |
| D4 | Output over voltage | 0=Normal, 1=Over voltage |
| D1 | Fault | 0=No fault, 1=Fault |
| D0 | Running | 0=Standby, 1=Running |

---

## Notes

- **Function Codes Used:**
  - `0x01`: Read Coils (control registers)
  - `0x03`: Read Holding Registers (configuration/ratings)
  - `0x04`: Read Input Registers (real-time data)
  - `0x05`: Write Single Coil (control operations)
  - `0x06`: Write Single Register (configuration)
  - `0x10`: Write Multiple Registers (used in `write_registers`)

- **Data Formats:**
  - 16-bit registers with ÷100 scaling for voltage/current/power
  - 32-bit values use two consecutive registers (low address = low 16 bits)
  - Signed values (two's complement) for temperature and battery current
  - Energy values typically in 0.01 kWh units (÷100 = kWh)

- **Register Access Notes:**
  - Addresses `0x3000-0x300E` contain device ratings (read-only)
  - Addresses `0x9000-0x9070` are configurable settings
  - Coils `0x00-0x0E` provide direct control functions
  - Some registers may vary by controller model/series (Tracer, Xtra, etc.)

- **Retry Logic:** All read operations retry up to 5 times with 200ms delay