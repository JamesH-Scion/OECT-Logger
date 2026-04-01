# OECT Logger

A two-channel organic electrochemical transistor (OECT) data logger built on the **Adafruit Feather ESP32 v2**. The logger applies precise excitation voltages to OECT devices, measures the resulting drain current through a transimpedance amplifier (TIA), logs all data to a microSD card with a real-time timestamp, and exposes a Wi-Fi web interface for in-field configuration and data download.

---

## Features

- **Dual-channel OECT measurement** — simultaneous drain current and excitation voltage logging on two independent channels
- **High-resolution ADC** — 24-bit ADS122C04 with 8-sample oversampling and EMA filtering on all channels
- **Closed-loop DAC control** — MCP4725 DACs servo'd to the user's excitation voltage target via ADC feedback
- **Timestamped SD logging** — one CSV file per power-on session, named `OECTLoggerN-YYMMDD-hhmmss.csv`
- **On-demand Wi-Fi access point** — activated by a button press; auto-disables after 10 minutes to preserve power
- **Web interface** — set excitation voltages, sync the RTC, and download log files from any browser
- **mDNS** — accessible at `http://logger.local` without needing to know the IP address
- **Light sleep between samples** — ESP32 enters light sleep between task deadlines for reduced battery consumption
- **Battery monitoring** — voltage divider on A13 with NeoPixel colour indication (green / yellow / orange)
- **Auto device naming** — each logger identifies itself by MAC address and takes a name like `OECTLogger14`

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| Microcontroller | Adafruit Feather ESP32 v2 | WROOM module, built-in LiPo charger |
| ADC | Texas Instruments ADS122C04 | 24-bit, 4-ch, I²C (`0x40`) |
| DAC Ch1 | Microchip MCP4725 | 12-bit, I²C (`0x60`) |
| DAC Ch2 | Microchip MCP4725 | 12-bit, I²C (`0x61`) |
| RTC | DS3231 | I²C, battery-backed |
| Environmental | Bosch BME280 | Temperature / Humidity / Pressure, I²C (`0x77`) |
| Storage | microSD card | SPI via SdFat, CS on A5 |
| Status LED | Built-in NeoPixel | `PIN_NEOPIXEL` |
| User button | Tactile switch | GPIO 38, active-low |

> **Voltage reference note:** The ADS122C04 uses AVDD (3.3 V rail) as its reference. For best noise performance, consider routing a stable external reference to the REFP pin.

---

## Signal Chain

```
OECT device
    │
    ├─► Gate/Drain ──► TIA (1 kΩ) ──► AIN0 / AIN1  (drain current)
    │
    └─► Gate voltage ─────────────► AIN2 / AIN3  (excitation voltage monitor)

MCP4725 DAC ──► [op-amp buffer + 2.5 V offset] ──► OECT Gate/Source
```

The ADC converts voltages in the range 0–5 V with a 2.5 V midpoint offset, giving a symmetric ±2.5 V measurement window.

---

## Software Architecture

The firmware is task-based using **TaskScheduler**. The ESP32 sleeps between task deadlines.

| Task | Period | Description |
|---|---|---|
| `readADCTask` | 10 s | Measure all 4 ADC channels (AIN0–AIN3), apply EMA filtering |
| `sendSensorDataTask` | 60 s | Read BME280, format and write row to SD card, print to serial |
| `readUserInputTask` | 10 s | Apply new voltage targets from web UI, save to NVS |
| `readBatteryTask` | 10 min | Sample battery voltage, update status LED colour |
| `blinkLEDTask` | 1 s | Toggle NeoPixel at the current status colour |
| `checkButtonTask` | 1 s | Detect button press, activate Wi-Fi AP |
| `wifiTimerTask` | 10 min (once) | Deactivate Wi-Fi after timeout |

### Measurement quality features

- **8-sample oversampling** on every ADC conversion (`getADCReading(8)`)
- **EMA filter (α = 0.1)** on all four channels for high-frequency noise rejection
- **Dummy conversion + 50 ms settle** after every MUX switch to eliminate input transients
- **Time-aligned logging** — excitation voltage and drain current are all measured within the same `readADCTask` execution

---

## CSV Log Format

Each row in the output file contains a full set of simultaneous measurements:

```
timestamp, raw_1, filt_1, voltage_1, current_1, raw_2, filt_2, voltage_2, current_2, vex2, vex1, temp, rh, pressure
```

| Column | Units | Description |
|---|---|---|
| `timestamp` | ISO 8601 | RTC timestamp (DS3231) |
| `raw_1` | counts | Raw 24-bit ADC value, Ch1 |
| `filt_1` | counts | EMA-filtered ADC value, Ch1 |
| `voltage_1` | V | TIA output voltage, Ch1 |
| `current_1` | mA | Drain current, Ch1 (1 kΩ TIA) |
| `raw_2` | counts | Raw 24-bit ADC value, Ch2 |
| `filt_2` | counts | EMA-filtered ADC value, Ch2 |
| `voltage_2` | V | TIA output voltage, Ch2 |
| `current_2` | mA | Drain current, Ch2 (1 kΩ TIA) |
| `vex2` | V | Measured excitation voltage, Ch2 |
| `vex1` | V | Measured excitation voltage, Ch1 |
| `temp` | °C | BME280 temperature |
| `rh` | % | BME280 relative humidity |
| `pressure` | hPa | BME280 atmospheric pressure |

---

## Web Interface

Press **button GPIO 38** to activate the Wi-Fi access point. The AP name matches the device name (e.g., `OECTLogger14`). Connect and navigate to:

```
http://logger.local
```

or the direct IP `192.168.4.1`.

| Endpoint | Method | Description |
|---|---|---|
| `/` | GET | Main dashboard — set voltages, view live data, sync RTC |
| `/setValues` | GET | Set `vex1` and `vex2` target voltages |
| `/userUpdate` | GET | Trigger immediate DAC update and NVS save |
| `/updateData` | GET | JSON snapshot of current sensor values |
| `/rtctime` | GET | Current RTC time as plain text |
| `/setrtc` | GET | Set RTC from Unix epoch (`?epoch=...`) |
| `/list_files` | GET | Browse CSV files on the SD card |
| `/download` | GET | Download a named file (`?name=...`) |

The AP automatically deactivates after **10 minutes** to preserve battery. The device resumes light sleep scheduling after deactivation.

---

## Device Naming

Each logger is identified by its Wi-Fi MAC address. Known devices are mapped in the `devices[]` table in `main.cpp`:

```cpp
DeviceMapping devices[] = {
  {"F4:65:0B:33:FE:04", 5},  // OECTLogger5
  ...
  {"F4:65:0B:31:C8:B0", 18}, // OECTLogger18
};
```

To register a new logger, add its MAC address and desired number to this table. Unrecognised devices fall back to `OECTLoggerXX` where `XX` is the last two hex digits of the MAC.

---

## Getting Started

### 1. Install PlatformIO

Install the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode) for VS Code.

### 2. Clone and open

```bash
git clone https://github.com/JamesH-Scion/OECT-Logger.git
cd OECT-Logger
code .
```

### 3. Build and upload

```bash
pio run --target upload --environment adafruit_feather_esp32_v2
```

Or use the PlatformIO toolbar in VS Code.

### 4. First boot

- The NeoPixel cycles purple → orange on startup, then turns **green** when sampling begins.
- Open the serial monitor at **115 200 baud** to see the device name, MAC address, and live readings.
- If the RTC has lost power, it will be initialised from the firmware compile time. Use the web interface to correct it.

### 5. Set excitation voltages

- Press the **button on GPIO 38** to start the Wi-Fi AP.
- Connect to the SSID matching the device name.
- Navigate to `http://logger.local`.
- Enter target voltages for Ch1 and Ch2 and click **Submit**. Values are saved to non-volatile storage and persist across reboots.

---

## NeoPixel Status Colours

| Colour | Meaning |
|---|---|
| 🟣 Purple | Startup / initialising |
| 🟠 Orange | Low battery (≤ 3.4 V) |
| 🟡 Yellow | Medium battery (3.4–3.7 V) |
| 🟢 Green | Normal operation, good battery (> 3.7 V) |
| 🔵 Blue | Hardware initialisation in progress |
| 🩵 Cyan | Wi-Fi access point active |
| 🔴 Red | SD write error |

---

## Dependencies

Managed automatically by PlatformIO via `platformio.ini`:

| Library | Source |
|---|---|
| SparkFun ADS122C04 ADC Arduino Library | `sparkfun/SparkFun ADS122C04 ADC Arduino Library` |
| SdFat | `greiman/SdFat` |
| RTClib | `adafruit/RTClib` |
| Adafruit MCP4725 | `adafruit/Adafruit MCP4725` |
| Adafruit NeoPixel | `adafruit/Adafruit NeoPixel` |
| Adafruit BME280 Library | `adafruit/Adafruit BME280 Library` |
| TaskScheduler | `arkhipenko/TaskScheduler` |
| Adafruit Unified Sensor | `adafruit/Adafruit Unified Sensor` |
| ESPAsyncWebServer | `ESP32Async/ESPAsyncWebServer` |
| AsyncTCP | `ESP32Async/AsyncTCP` |

---

## License

Internal Scion research tool. Contact the repository owner for licensing terms.
