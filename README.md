# PLA — Parachute Landing Assistant

An open-source skydiving altitude alerter built on ESP32-S3. It uses a TFmini Plus laser rangefinder and IMU to measure your true above-ground-level height, then alerts you with two-stage audio warnings as you approach landing.

## How It Works

The device is worn at waist level. A downward-facing laser rangefinder measures distance to the ground. The IMU corrects for body tilt (cosine compensation), and a configurable height offset subtracts the waist-to-ground distance so that `h = 0` means your feet are on the ground.

A Kalman filter fuses IMU-predicted altitude with laser measurements for smooth, responsive height tracking. Speed compensation dynamically raises alert thresholds — the faster you descend, the earlier the alert triggers (adding ~0.3 seconds of reaction time).

**Alert Stages:**
- **Stage 1** (configurable, default 4.0m): Intermittent beeping
- **Stage 2** (configurable, default 1.2m): Continuous tone
- Alerts stop when you flip the ARM switch off after landing

**Configuration** is done over WiFi — long-press a button, connect your phone, adjust parameters in a browser, and save. Parameters are stored in non-volatile memory.

## Hardware

| Component | Model | Notes |
|-----------|-------|-------|
| MCU | ESP32-S3 DevKit | Any ESP32-S3 board with USB-C |
| IMU | ICM-42688-P | SPI, 8g/2000dps, 100Hz |
| Laser Rangefinder | TFmini Plus | UART 115200, 0.1–12m range, 100Hz |
| Display | SSD1680 E-Paper | 152×152, SPI, only refreshes on boot/config |
| Buzzer | Active buzzer 3.3V | GPIO driven |
| ARM Switch | SPST toggle | Directly wired to GPIO, LOW = armed |
| WiFi Button | Momentary push button | Long press 6s to enable WiFi config |

**Approximate total cost: ~¥200 / $30 USD** (excluding 3D-printed enclosure)

## Wiring

```
ESP32-S3 Pin Assignments
═══════════════════════════════════════════════

IMU (ICM-42688-P) — SPI2
  SCK  → GPIO 48
  MOSI → GPIO 38
  MISO → GPIO 47
  CS   → GPIO 6

E-Paper (SSD1680) — SPI3
  SCK  → GPIO 13
  MOSI → GPIO 14
  CS   → GPIO 8
  DC   → GPIO 9
  RST  → GPIO 10
  BUSY → GPIO 7

TFmini Plus — UART1 (115200 baud)
  ESP TX (GPIO 43) → TFmini RX (Green wire)
  ESP RX (GPIO 44) → TFmini TX (White wire)
  5V               → TFmini VCC (Red wire)
  GND              → TFmini GND (Black wire)

Buzzer
  Signal → GPIO 4

WiFi Config Button
  GPIO 46 → Button → GND (internal pull-up)

ARM Switch
  GPIO 17 (D8) → Switch → GND (internal pull-up)
  Switch CLOSED (LOW) = ARMED
  Switch OPEN  (HIGH) = DISARMED
```

### TFmini Plus Wiring Detail

```
TFmini Connector (GH1.25-4P)
Pin 1 (Red)   → 5V  ⚠️ Must be 5V, not 3.3V!
Pin 2 (White) → GPIO 44 (ESP32 RX)
Pin 3 (Green) → GPIO 43 (ESP32 TX)
Pin 4 (Black) → GND
```

> **Important:** The TFmini Plus requires 5V power supply (550mW nominal, 140mA peak). It will not function on 3.3V.

## Build & Flash

### Prerequisites

- [ESP-IDF v5.5.x](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/)
- ESP32-S3 board with USB-C

### Steps

```bash
# Clone the repo
git clone https://github.com/YOUR_USERNAME/PLA.git
cd PLA

# Set target
idf.py set-target esp32s3

# Build
idf.py build

# Flash
idf.py -p COMX flash monitor
```

> **Note:** The console UART uses USB CDC (GPIO 43/44 are used by TFmini). Make sure `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` in your `sdkconfig`.

## Usage

### First Boot

1. Power on with ARM switch **OFF** (GPIO 17 open)
2. Device beeps once → IMU calibration starts (10 seconds, keep still)
3. Long beep → calibration done, e-paper shows parameters
4. Device enters low-power wait

### WiFi Configuration

1. Long-press WiFi button (GPIO 46) for **6 seconds**
2. Buzzer confirms, e-paper shows SSID and password
3. Connect phone to WiFi:
   - **SSID:** `PLA-Config`
   - **Password:** `QQ123456`
4. Open browser → `192.168.4.1`
5. Adjust parameters and save
6. WiFi automatically closes, e-paper shows updated parameters

### Configurable Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| Wing Loading | 0.93 | 0.5–2.0 | Your canopy wing loading (lb/ft²) |
| Height Offset | 85 cm | 30–150 cm | Waist-to-ground distance |
| Stage 1 Height | 4.0 m | 2–8 m | Intermittent beep threshold |
| Stage 2 Height | 1.2 m | 0.5–3 m | Continuous tone threshold |

### Arming (Before Jump)

1. Flip ARM switch to ON (GPIO 17 → GND)
2. System wakes IMU and TFmini
3. Waits up to 15 seconds for sensor convergence
4. Two short beeps → **ARMED**
5. If sensors fail → silent (no beeps), not armed

### In Flight

The system continuously calculates AGL height with tilt compensation and speed compensation:

```
AGL = (laser_distance × cos(tilt)) − height_offset

Effective Stage 1 threshold = stage1_h + |vertical_speed| × 0.3
Effective Stage 2 threshold = stage2_h + |vertical_speed| × 0.3
```

The `× 0.3` factor adds approximately 0.3 seconds of reaction time — descending faster triggers alerts earlier.

### After Landing

Flip ARM switch OFF → buzzer stops, sensors enter low-power mode.

### Crash Reboot Protection

If the device unexpectedly reboots while ARM is ON (power glitch), it detects this on boot and immediately enters ARMED mode without calibration, with triple-beep warning.

## Architecture

```
Core 0                          Core 1
┌─────────────────┐    ┌──────────────────────┐
│ task_armed       │    │ task_buzzer           │
│  100Hz loop      │    │  Drives buzzer output │
│  IMU read        │    │                      │
│  Mahony AHRS     │    │ task_lidar           │
│  Kalman filter   │    │  TFmini UART read    │
│  flare_update()  │    │                      │
│                  │    │ task_wifi_btn         │
│                  │    │  Long-press detect    │
│                  │    │  Config save handler  │
│                  │    │                      │
│                  │    │ task_arm              │
│                  │    │  ARM switch monitor   │
└─────────────────┘    └──────────────────────┘
```

## File Structure

```
PLA/
├── main/
│   └── main.c          # Single-file firmware (all logic)
├── CMakeLists.txt
├── README.md
└── LICENSE
```

## Known Limitations

- TFmini Plus max range is 12m (sufficient for canopy flight below 12m AGL)
- Highly reflective surfaces (mirrors, wet tarmac) may cause laser errors
- E-paper only refreshes on boot, WiFi open, and config save (by design — no distraction in flight)
- WiFi password is hardcoded; all devices share the same default password

## Contributing

Pull requests welcome. Key areas for improvement:

- 3D-printed enclosure design
- Support for other laser rangefinders (e.g., VL53L5CX for shorter range)
- Bluetooth configuration app (replace WiFi portal)
- Flight data logging to SPI flash
- Per-device unique WiFi password (based on MAC address)

## Safety Disclaimer

**This is an experimental aid, not a certified aviation instrument.** Always rely on your primary altimeter and training. The developer assumes no liability for use of this device in skydiving or any other activity. Test thoroughly on the ground before any airborne use.

## License

MIT License — see [LICENSE](LICENSE) for details.
