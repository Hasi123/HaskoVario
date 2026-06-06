# HaskoVario / GNUVario — Code Architecture

HaskoVario is an Arduino-based **variometer** for paragliding and hang-gliding. It measures barometric altitude and vertical acceleration, fuses them with a Kalman filter, and provides real-time audio (via a buzzer), GPS-based IGC flight logging (SD card), and Bluetooth telemetry (LXNAV or LK8000 sentence formats).

## Project Structure

```
HaskoVario/
├── variometer/                   # Main firmware sketch (variometer.ino)
├── libraries/                    # All custom libraries
│   ├── VarioSettings/            # Central configuration (compile-time settings)
│   ├── kalmanvert/               # 2-state Kalman filter (position + velocity)
│   ├── ms5611/                   # MS5611 barometric pressure sensor driver
│   ├── MPU6050/                  # MPU6050 IMU + DMP firmware loader
│   ├── beeper/                   # Audio feedback (climb/sink tones)
│   ├── FlightHistory/            # Circular buffer for climb rate & glide ratio
│   ├── IntTW/                    # Interrupt-driven I2C (TWI) master library
│   ├── NmeaParser/               # GPS NMEA sentence parser ($RMC/$GGA)
│   ├── SerialNmea/               # Interrupt-driven UART with NMEA filtering
│   ├── GpsSentences/             # Output sentence generators (IGC, LK8, LXWP0)
│   ├── LightFat16/               # Ultra-light FAT16 + SD card driver
│   ├── varioPower/               # Power management (LDO, button, watchdog, low-bat)
│   ├── marioSounds/              # Startup/shutdown/low-battery melodies
│   ├── LPtoneAC/                 # High-quality push-pull tone generation (timer1)
│   ├── wserial/                  # Lightweight write-only software serial
│   ├── igcrypto/                 # HMAC-SHA256 IGC G record signing
│   └── dmp_compress/             # DMP firmware compression utilities
├── tools/                        # PC-side utilities (igc_verify.py)
├── IGCs/                         # Sample signed flight logs
└── schematic.pdf                 # Hardware schematic
```

## Configuration: `VarioSettings.h`

All hardware configuration happens at compile time in `libraries/VarioSettings/VarioSettings.h`. Key categories:

| Category | Examples |
|----------|----------|
| **Feature flags** | `HAVE_SPEAKER`, `HAVE_ACCELEROMETER`, `HAVE_SCREEN`, `HAVE_GPS`, `HAVE_SDCARD`, `HAVE_BLUETOOTH`, `HAVE_VOLTAGE_DIVISOR`, `HAVE_IGC_SECURITY` |
| **IGC header fields** | `VARIOMETER_MODEL`, `VARIOMETER_PILOT_NAME`, `VARIOMETER_GLIDER_NAME`, `VARIOMETER_GLIDER_ID`, `VARIOMETER_FIRMWARE_VERSION`, `VARIOMETER_HARDWARE_VERSION` |
| **IGC security** | `VARIOMETER_HMAC_KEY` (32-byte HMAC-SHA256 key) |
| **Beep thresholds** | `VARIOMETER_SINKING_THRESHOLD` (-0.3 m/s), `VARIOMETER_CLIMBING_THRESHOLD` (+0.8 m/s), `VARIOMETER_NEAR_CLIMBING_SENSITIVITY` |
| **Flight detection** | `FLIGHT_START_MIN_TIMESTAMP` (30s), velocity thresholds, `FLIGHT_START_MIN_SPEED` (15 km/h GPS) |
| **GPS/baro calibration** | `VARIOMETER_GPS_ALTI_CALIBRATION_PRECISION_THRESHOLD` (HDOP < 1.5) |
| **Bluetooth** | Sentence type selector (`VARIOMETER_SENT_LXNAV_SENTENCE` / `VARIOMETER_SENT_LK8000_SENTENCE`), baud rate |
| **Pin assignments** | CS pins for SD card, interrupt pin for MPU6050 |
| **Climb/glide integration** | `VARIOMETER_CLIMB_RATE_INTEGRATION_TIME` (6s), `VARIOMETER_GLIDE_RATIO_INTEGRATION_TIME` (30s) |

## Main Sketch: `variometer/variometer.ino`

### Global State Machine

The variometer transitions through four states:

1. **`VARIOMETER_STATE_INITIAL`** (0) — Waiting for GPS date to be received
2. **`VARIOMETER_STATE_DATE_RECORDED`** (1) — GPS date received, waiting to calibrate baro with GPS altitude
3. **`VARIOMETER_STATE_CALIBRATED`** (2) — Baro zeroed to GPS altitude. Without GPS, this is the starting state.
4. **`VARIOMETER_STATE_FLIGHT_STARTED`** (3) — Flight detected; SD card recording, near-climbing alarms activated.

### `setup()` — Initialization Sequence

1. **`varioPower.init()`** — Power on LDO, check button for shutdown, start watchdog
2. **SPI initialization** — Enable CS lines for SD card and screen
3. **SD card** — `file.init()` (if HAVE_SDCARD + HAVE_GPS)
4. **Screen** — `screen.begin(VARIOSCREEN_CONTRAST)`
5. **GPS / Bluetooth UART** — `serialNmea.begin(GPS_BLUETOOTH_BAUDS, ...)`
6. **I2C** — `I2C::begin()` initializes TWI hardware and timer interrupts
7. **MS5611** — `ms.init()` resets sensor, reads PROM calibration
8. **MPU6050** — `mpu.init()` loads DMP firmware, configures FIFO
9. **MPU6050 interrupt** — `attachInterrupt(digitalPinToInterrupt(MPU6050_INTERRUPT_PIN), getSensors, RISING)`
10. **Boot sound** — `marioSounds.bootUp()`
11. **Firmware update check** — `varioPower.updateFW()` checks for FIRM.HEX on SD card
12. **Calibration** — `mpu.calibrate()` if held upside-down
13. **Kalman filter init** — Wait for first sensor data, `kalmanvert.init()`
14. **History init** — `history.init(firstAlti, millis())`

### `loop()` — Main Execution Cycle

The main loop is driven by the `I2C::newData` flag, which is set by the ISR (`I2C::intHandler()`) triggered by the MPU6050 interrupt. The flag cycles through values 0→5, orchestrating a lockstep sensor pipeline:

```
I2C::newData:
  0 → idle (waiting)
  1 → ms.update()              — compute compensated pressure/temp from MS5611
  2 → alt = ms.getAltitude()   — get barometric altitude
  3 → vertAccel = mpu.getVertaccel() — get vertical acceleration from DMP
  4 → kalmanvert.update1(vertAccel, millis()) — Kalman time-update (predict)
  5 → kalmanvert.update2(alt)  — Kalman measurement-update (correct)
      beeper.setVelocity(kalmanvert.getVelocity()) — feed velocity to beeper
      reset to 0
```

#### Each loop iteration also performs:

| Component | Conditions | What it does |
|-----------|-----------|--------------|
| **History** | Always (`HAVE_GPS` or integrated climb rate) | `history.setAlti(calibratedPosition, millis())` |
| **Screen digits** | `HAVE_SCREEN` | Updates altitude and vario digit displays |
| **VarioPower** | Always | `varioPower.update()` — watchdog reset, button check, low-voltage monitoring |
| **Beeper** | `HAVE_SPEAKER` | `beeper.update()` — drives toneAC based on current velocity |
| **Bluetooth** | `HAVE_BLUETOOTH` | Sends vario NMEA sentence every `VARIOMETER_SENTENCE_DELAY` ms |
| **GPS** | `HAVE_GPS` | Parses RMC/GGA sentences, calibrates baro, detects flight start |
| **SD card (IGC)** | `HAVE_SDCARD` + `HAVE_GPS` | Writes IGC B-records from GGA data |
| **Low-freq screen** | `HAVE_SCREEN` | Updates time, elapsed time, satellite count, battery (on page transitions) |
| **Screen scheduler** | `HAVE_SCREEN` | `varioScreen.displayStep()` — renders visible page elements |

## Sensor Pipeline: `I2CHelper` + `IntTW`

The I2C subsystem (in `libraries/IntTW/`) is the heart of sensor coordination:

- **`IntTW`** — Interrupt-driven TWI (I2C) master. Executes command sequences from a buffer with interrupt-level bus handling.
- **`I2C` (static class)** — High-level wrapper. Provides `begin()`, `sendCMD()`, `read24()`, `readWord()`, `writeByte()`, etc.
- **`I2C::intHandler()`** — Called from the MPU6050 interrupt ISR. Orchestrates the MS5611 + MPU6050 pipeline in a state machine that runs entirely at interrupt level. Sets `I2C::newData` to advance the main loop's processing.

The timer interrupt (`TIMER2_COMPA`) schedules MS5611 conversion timing (8.44ms for pressure, 1.18ms for temperature) to ensure correct ADC readout.

## Kalman Filter: `kalmanvert`

A **2-state discrete Kalman filter** estimating **vertical position** and **vertical velocity**:

**State vector:** `x = [position, velocity]^T`

**Time-update (`update1`):**
```
x_k|k-1 = F * x_k-1|k-1 + G * a_m
P_k|k-1 = F * P_k-1|k-1 * F^T + Q
```
Where `a_m` is measured vertical acceleration (from MPU6050 DMP), `F` is the kinematic matrix `[[1, dt], [0, 1]]`, and `G = [0.5*dt^2, dt]^T`.

**Measurement-update (`update2`):**
```
K = P * H^T * (H * P * H^T + R)^-1
x = x + K * (z - H * x)
P = (I - K * H) * P
```
Where `z` is measured barometric altitude (from MS5611), `H = [1, 0]`.

**Calibration:** `calibratePosition(newPosition)` adds an offset (`calibrationDrift`) to the position estimate without modifying velocity or covariance. This allows GPS altitude to bias the baro zero without disturbing the velocity estimate.

## Audio Feedback: `beeper`

The beeper maps vertical velocity to four beep zones:

| Zone | Velocity Range | Sound |
|------|---------------|-------|
| **Sinking** | `< SINKING_THRESHOLD` (e.g. -0.3 m/s) | Continuous low tone, pitch varies with sink rate |
| **Silent** | Between sinking and climbing thresholds | No sound |
| **Gliding / Near-climbing** | Near but below climbing threshold | Periodic short beeps (100ms on / 1400ms off) |
| **Climbing** | `> CLIMBING_THRESHOLD` (e.g. +0.8 m/s) | Periodic beeps with velocity-dependent pitch |

The climbing beep pattern cycles every 1.0 vertical meter (duration = 1.0 / climbRate) for intuitive audio altitude feedback.

## Display: `varioscreen`

The Nokia 5110 (PCD8544) screen uses a **screen object model**:

- **`VarioScreen`** — Low-level PCD8544 SPI driver
- **`VarioScreenObject`** — Abstract base with `display()` / `update()` / `reset()`
- **`ScreenDigit`** — Numeric display using stabilized `FPSDigit`
- **Unit objects** — `MSUnit` ("m/s"), `MUnit` ("m"), `KMHUnit` ("km/h"), `GRUnit` (glide ratio)
- **Status objects** — `BATLevel` (battery bar), `SATLevel` (satellite bar), `ScreenTime` (HH:MM:SS), `ScreenElapsedTime`, `ScreenMuteIndicator`
- **`ScreenScheduler`** — Page manager. Base page (page 0) shows altitude + vario. Alternate page (page 1, only with GPS) shows time + speed + glide ratio + satellite count.

## GPS / Bluetooth Serial: `SerialNmea`

**`SerialNmea`** is an interrupt-driven UART manager that shares the hardware serial port between GPS (RX) and Bluetooth (TX):

- **Receive ISR** (`rxCompleteVect`) — Reads bytes into a 128-byte ring buffer. Validates NMEA checksums in real-time (detects `$` start, verifies `*XX` checksum). Marks complete sentences as available by type (RMC / GGA).
- **Transmit ISR** (`udrEmptyVect`) — Transmits buffered bytes for Bluetooth output.
- **Lock mechanism** — `lockRMC()` / `lockGGA()` claim complete sentences for reading. `lock()` / `write()` / `release()` handle Bluetooth output interleaved with GPS reading.

## GPS Data Flow

```
GPS Module (UART)
    ↓ (interrupts)
SerialNmea (ISR, validates NMEA checksums)
    ↓ (lockRMC / lockGGA)
NmeaParser (extracts time, date, altitude, speed, satellite count, HDOP)
    ↓
├── kalmanvert.calibratePosition(gpsAlti) — when HDOP < threshold
├── screenTime.setTime(nmeaParser.time)
├── ScreenElapsedTime / SATLevel updates
├── SpeedFlightHistory::getGlideRatio(speed, timestamp)
└── IGC file creation (date → filename: DDMMYY00.IGC, DDMMYY01.IGC, ...)
```

## IGC Flight Logging (SD Card)

When GPS is available and the variometer is calibrated, an IGC file is
created on the SD card (FAT16). File naming uses the date + auto-increment:
`DDMMYY00.IGC`, `DDMMYY01.IGC`, etc.

### IGC Header (PROGMEM)

The IGC header (pilot name, glider model, instrument ID, etc.) is stored
as a **single PROGMEM string** assembled at compile time from the defines
in `VarioSettings.h`. At file creation, the header is written byte-by-byte
via `pgm_read_byte()`, with a 6-byte placeholder (`000000`) at the date
position overwritten by the actual GPS date. The header contains 14 HF
records conforming to IGC spec A3.2.4.

### B Records (Buffered Write)

Each GGA sentence from the GPS is converted to an IGC B-record using
`IGCSentence`. To prevent corrupted records (e.g. during GPS signal loss),
the entire B record (max 48 bytes) is buffered on the stack during GGA
parsing. It is flushed to the SD card and fed to the HMAC engine **only
after** `NmeaParser::satelliteCount > 0` is confirmed — guaranteeing zero
incomplete B records on disk.

### IGC Security (HMAC-SHA256 G Record)

When `HAVE_IGC_SECURITY` is enabled, the firmware computes an
HMAC-SHA256 digest incrementally as bytes are written to the IGC file:

- **Header bytes** — fed to the HMAC engine during file creation
- **B-record bytes** — fed each time a valid GGA sentence is parsed

On shutdown, a **two-phase sequence** finalizes the file:

1. `varioPower.update()` detects the shutdown trigger (long-press or
   low voltage) and sets `shutdownPending = true` instead of sleeping.
2. The main loop calls `finalizeIGCFile()`: HMAC is finalized, and the
   256-bit digest is written as two `G<32 hex chars>\r\n` lines
   (64 hex chars total, split per IGC line length limits).
3. `file.sync()` flushes the block buffer to the SD card.
4. `varioPower.completeShutdown()` then disables all peripherals and
   enters `SLEEP_MODE_PWR_DOWN`.

The crypto implementation is in `libraries/igcrypto/` (SHA-256 with
rolling `W[16]` window, HMAC-SHA256 wrapper, PROGMEM key accessor).

### Firmware Updates

The SD card also supports **firmware updates**: holding the power button
at startup causes the bootloader to reflash from `FIRM.HEX` on the card.

## Bluetooth Sentences

At compile time, one of two sentence formats is selected:

- **`$LXWP0`** (LXNAV) — `$LXWP0,Y,,,<vario>,,,,,,,,*<checksum>`
- **`$LK8EX1`** (LK8000) — `$LK8EX1,,<alti>,<vario>,99,999,*<checksum>`

The variometer alternates between parsing GPS sentences and sending its own vario NMEA sentence over Bluetooth. The sentence is sent every `VARIOMETER_SENTENCE_DELAY` ms (default 500ms).

## Power Management: `varioPower`

- **LDO control** — Pin PD5 controls an external low-dropout regulator. Pulled high to power the system, driven low to completely cut power.
- **Button** — Long press (>1s) triggers shutdown. During shutdown: plays melody, disables peripherals, sets all pins to input, enters `SLEEP_MODE_PWR_DOWN`. Wake on button LOW level interrupt → watchdog reset.
- **Watchdog** — Reset every loop iteration. If not reset within ~1s, the MCU reboots.
- **Low-voltage** — Monitors battery (ADC on A1). At < 3.45V: periodic double-beep. At < 3.3V: immediate shutdown.
- **Firmware update** — Button held for >1s at startup → `varioPower.updateFW()` checks for `FIRM.HEX` on SD card and jumps to bootloader (address 0x7800).

---

*Generated from the HaskoVario codebase at `/home/david/opencode_dev/HaskoVario`.*
