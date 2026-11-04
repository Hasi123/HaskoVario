# HaskoVario — How It Works

HaskoVario is a zero-latency audio variometer with GPS flight logging for
paragliding and hang gliding, running on an ATmega328P at 8 MHz with 2 KB of
RAM. It fuses barometric altitude (MS5611) with vertical acceleration (MPU6050
DMP) in a Kalman filter, beeps the climb rate on a piezo, and writes signed
IGC files to an SD card.

Everything runs on one CPU without an operating system. The code keeps the
sensor path in interrupts and splits all slow work in `loop()` into short
pieces, so that the beeper is updated often enough to sound clean. The
[timing section](#main-loop-timing-and-the-beeper) explains the budget.

## Hardware and pins

| Function | Pin | Notes |
|---|---|---|
| MPU6050 interrupt | PD2 (INT0) | DMP data ready, 100 Hz, latched until any MPU read |
| Power button | PD3 (INT1) | pull-up, wake from sleep |
| Regulator enable | PD5 | high = system powered |
| I2C (MS5611, MPU6050) | PC4/PC5 | 400 kHz, interrupt driven (`IntTW`) |
| GPS (Quectel L80) | PD0 (RX) | 9600 baud, hardware UART |
| GNSS configuration | PC2 (A2) | software serial TX, only used at boot. Not yet wired to the L80 RX (see TODO.md) |
| UART TX | PD1 | Bluetooth module, not fitted (`HAVE_BLUETOOTH` off) |
| SD card | PB0 (CS), SPI | FAT16, 4 MHz SPI |
| Piezo | PB1/PB2 (OC1A/OC1B) | Timer1 push-pull PWM (`toneAC`) |
| Battery voltage | PC1 (A1) | internal 1.1 V reference |

## Project structure

```
HaskoVario/
├── variometer/variometer.ino   # setup(), loop(), IGC file creation/finalisation
├── libraries/
│   ├── VarioSettings/          # all compile-time configuration
│   ├── IntTW/                  # interrupt-driven I2C master + sensor chain (I2CHelper)
│   ├── ms5611/                 # barometer driver (compensation, altitude)
│   ├── MPU6050/                # IMU driver, DMP loader, calibration, vertical accel
│   ├── kalmanvert/             # 2-state Kalman filter (altitude, vertical speed)
│   ├── beeper/                 # climb/sink sound patterns
│   ├── LPtoneAC/               # Timer1 tone generation
│   ├── marioSounds/            # boot/shutdown/low-battery sounds
│   ├── varioPower/             # power on/off, watchdog, battery monitor, FW update jump
│   ├── SerialNmea/             # UART interrupt with NMEA filtering (RMC/GGA)
│   ├── NmeaParser/             # date, time, speed, altitude, HDOP from RMC/GGA
│   ├── GpsSentences/           # IGC header + B record builder, LXWP0/LK8EX1 (Bluetooth)
│   ├── LightFat16/             # minimal SD card + FAT16 file writer
│   ├── igcrypto/               # SHA-256 / HMAC-SHA256 for the IGC G record
│   ├── wserial/                # write-only software serial (GNSS config)
│   ├── digit/                  # number to digits (Bluetooth sentences)
│   └── dmp_compress/           # PC-side tool lib that produced the compressed DMP image
├── tools/
│   ├── igc_verify.py           # checks the G record of an IGC file
│   └── crypto_test/            # PC test of igcrypto against Python hashlib/hmac
├── igc specifications/         # IGC and CIVL specifications used for compliance
└── TODO.md                     # open items
```

## Configuration: `VarioSettings.h`

All settings are compile-time `#define`s in
`libraries/VarioSettings/VarioSettings.h`:

| Group | Settings (current value) |
|---|---|
| Features | `HAVE_SPEAKER`, `HAVE_ACCELEROMETER`, `HAVE_GPS`, `HAVE_SDCARD`, `HAVE_IGC_SECURITY`, `HAVE_GNSS_CONFIG` on; `HAVE_BLUETOOTH` off |
| IGC header | `VARIOMETER_MANUFACTURER`, `VARIOMETER_MODEL`, pilot, glider type/ID, firmware/hardware version |
| IGC security | `VARIOMETER_HMAC_KEY` (32 bytes), `IGC_SHA256_ROUNDS_PER_STEP` (4) |
| Beeps | sink −2.5 m/s, climb +0.2 m/s, near-climb sensitivity 0.5 m/s, volume 10 |
| Flight start | >15 s after power-on, \|vario\| > 0.5 m/s, GPS speed > 10 km/h |
| Baro calibration | GPS altitude used when HDOP < 3.50 (`350`) |
| Hardware | SD CS pin 8, GNSS config pin 16 (PC2), 9600 baud, I2C 400 kHz |

## Boot: `setup()`

1. `varioPower.init()`: switch the regulator on, wait 1 s. If the button is no
   longer held, go straight back to sleep. Start the watchdog (4 s).
2. SD card: set the CS line, `file.init()`.
3. UART: `serialNmea.begin(9600)` with receive enabled.
4. GNSS configuration (`HAVE_GNSS_CONFIG`): four NMEA commands over the
   software serial on PC2. They disable GPTXT and PPS, enable low-power mode,
   and limit the output to RMC + GGA. Each byte is sent with interrupts off,
   about 80 ms in total.
5. I2C, `ms.init()` (reset, read PROM coefficients),
   `mpu.init()` (reset, load calibration from EEPROM, upload the DMP image,
   enable FIFO + DMP interrupt), then attach INT0 → `I2C::intHandler()`.
6. Boot melody. If the button is still held → jump to the SD bootloader
   (`0x7800`) which flashes `FIRM.HEX`.
7. If the vario is upside down and still → 6-position accelerometer and gyro
   calibration, saved to EEPROM, then reset.
8. Wait for the first sensor sample and start the Kalman filter at that
   altitude.

## Execution model

There is no scheduler. Work is done in three places:

| Context | Trigger | Work | Duration |
|---|---|---|---|
| INT0 ISR | MPU6050 DMP, every 10 ms | starts the I2C chain | few µs |
| TWI ISR | each I2C byte (~25 µs apart) | runs the I2C chain, parses results | ~10 µs each, ~50 per 10 ms |
| USART RX ISR | each GPS byte (~1 ms apart) | NMEA filtering into a ring buffer | few µs |
| Timer0 ISR | 1 ms | `millis()` | few µs |
| `loop()` | always | sensor math, beeper, GPS parsing, IGC, power | see below |

**Rule:** all I2C traffic happens inside the interrupt chain. `loop()` never
starts an I2C transfer while INT0 is attached (the only exception is
`I2C::kick()`, which runs with interrupts disabled and only when the bus is
idle). `IntTW` has no locking, so a blocking transfer from `loop()` could
collide with the chain. That was the cause of the "beeps go strange after a
while" bug.

## Sensor pipeline (interrupts)

The MPU6050 DMP produces one 32-byte FIFO packet (quaternion, accelerometer,
gyro) every 10 ms and raises INT0. The pin is latched until any MPU read, so
**every interrupt must lead to an MPU read**, or no further interrupt will come.

`I2C::intHandler()` (`libraries/IntTW/I2CHelper.cpp`) starts a chain of
non-blocking transfers. Each step is started from the completion callback of
the previous one, inside the TWI interrupt:

```
INT0 ──► intHandler()
           │  bus still busy? ──► pendingInt = 1, restart when the chain ends
           ▼
         MS5611: read ADC (result of the conversion started 10 ms ago),
                 start next conversion (pressure, temperature every 20th)
           ▼  mpuGetFIFOcount()
         record the MS5611 value (d1 or d2), read MPU FIFO count (keep bus)
           ▼  mpuCheckFIFOcount()
         count == 0           ──► stop, chainEnd()
         count % 32 != 0, >256 ──► reset FIFO (DMP keeps running) ──► chainEnd()
         otherwise            ──► read all packets, keep the newest
           ▼  dataReady()
         mpu.parseFIFO(), I2C::newData = 1, chainEnd()
```

- The MS5611 runs at OSR 4096 (max 9.04 ms conversion). Starting the next
  conversion in each chain and reading it 10 ms later uses the DMP interrupt
  as the conversion timer. That is why the DMP rate must stay at 100 Hz or
  slower.
- The MS5611 reading is recorded before the MPU step, so an MPU failure
  cannot mix up temperature and pressure.
- The FIFO reset writes `USER_CTRL = DMP_EN|FIFO_RST`, then
  `DMP_EN|FIFO_EN`, as two separate transfers. They have to be separate
  because `IntTW` merges consecutive writes to the same device into one
  transaction. The DMP is not stopped, so its orientation should stay intact
  (still to be confirmed on hardware, see TODO.md). The old reset stopped
  the DMP for 50 ms and lost any rotation during that time.
- **Stall recovery:** if `loop()` sees no new sensor data for 100 ms (an I2C
  error or a lost edge), it calls `I2C::kick()` to restart the chain.

The chain uses about 50 TWI interrupts per 10 ms, roughly 5 % of the CPU,
spread out in ~10 µs pieces.

## Main loop timing and the beeper

### Why loop timing matters

The tone *pitch* is generated by Timer1 in hardware and is never disturbed by
the CPU. Its *rhythm* is not: `beeper.update()` runs once per `loop()` pass
and is the only place where a beep is switched on or off or changes pitch. A
slow pass therefore delays the next beep edge by the length of that pass.

The climbing beep pattern advances with climbed altitude, not time:

```
beep period = 1.0 / (0.51 · v + 1.62) seconds, half on, half off
```

| Climb rate | Period | Beep on |
|---|---|---|
| 0.2 m/s | 580 ms | 290 ms |
| 1 m/s | 470 ms | 235 ms |
| 3 m/s | 320 ms | 160 ms |
| 5 m/s | 240 ms | 120 ms |

Pattern positions are computed from `millis()`, so delays don't accumulate:
a slow pass makes one edge late, the next one is on time again. A delay of
1 ms is inaudible. Delays around 10 ms and more are audible at strong climbs
as an uneven rhythm or a clipped beep. The sink tone is continuous and only
changes pitch, so it is much less sensitive.

The second constraint is the sensor rate: the five processing steps of one
sample have to fit into the 10 ms between DMP interrupts (see below).

**Design rule: every `loop()` pass should stay around 1 ms. Anything longer
has to be split into steps.**

### The step machine

`I2C::newData` is set to 1 by the interrupt chain when a new sample is ready.
`loop()` does one step per pass and moves `newData` on:

| `newData` | Step | Estimated cost |
|---|---|---|
| 1 | `ms.update()`: MS5611 compensation (2nd order below 20 °C) | 0.25–0.4 ms |
| 2 | `ms.getAltitude()`: ISA formula with `pow()` | ~0.6 ms |
| 3 | `mpu.getVertaccel()`: gravity direction from the quaternion, vertical acceleration | ~0.5 ms |
| 4 | `kalmanvert.update1()`: predict | ~0.4 ms |
| 5 | `kalmanvert.update2()`: correct, then `beeper.setVelocity()` | ~0.4 ms |
| 0 | idle: `igcHmac.step()`, 4 SHA-256 rounds if a hash block is pending | ~0.5 ms or nothing |

Every pass also runs, regardless of the step:

| Work | Estimated cost |
|---|---|
| `varioPower.update()`: watchdog reset, button, battery check every 2 min | < 0.05 ms |
| `beeper.update()`: pattern position, tone on/off | ~0.15 ms, ~0.3 ms when the tone changes |
| stall check, GPS sentence check | < 0.02 ms |

The costs are estimates from the floating-point operation counts (soft float
on AVR: roughly 100 cycles per add/multiply, 500 per divide, 4000 per
`pow()`), not measurements. See [Measuring](#measuring-on-hardware).

So one sample needs about 3–4 ms of the 10 ms period, and the rest is spent
in idle passes of ~0.2–0.5 ms each. If a pass takes longer than the rest of
the period, the interrupt sets `newData = 1` again and the step sequence
restarts with the newer sample. Nothing breaks, but a sample is skipped.
`kalmanvert.update1()` measures `dt` from the timestamps and caps it at
200 ms, so skipped samples don't disturb the filter.

`loop()` reads `millis()` once per pass (`now`) and uses it for every timing
decision in that pass.

### Long passes that remain

| Event | How often | Estimated length |
|---|---|---|
| GPS sentence parse (RMC, GGA, B record build, 37 bytes to SD buffer + HMAC) | 2× per second | 1–2 ms |
| SD block boundary: write data block, read + write directory entry | every 512 bytes ≈ 14 s | ~5 ms + 2 card busy waits (typ. 1–5 ms each, up to 600 ms timeout) |
| FAT cluster boundary: FAT scan + 4 FAT updates | every cluster (minutes) | ~10–20 ms + busy waits |
| IGC file creation: SD re-init, directory and FAT scan, header, ~7 SHA-256 blocks | once, at the first good GPS fix | ~0.1 s to over 1 s, depending on the card and how full it is |
| Shutdown: G record, final sync | once | not relevant |

The SD block write is now the most likely source of an audible hiccup in
flight (see TODO.md).

### SHA-256 split into steps

The IGC signature is an HMAC-SHA256 over every byte written to the file. A
SHA-256 block (64 bytes, 64 rounds) costs an estimated 6 ms. Done in one go it
would stall a pass every ~1.7 s, which used to happen (and took ~3× longer
before the rotation change below). Now:

- When 64 bytes are collected, `Sha256::write()` copies them into the message
  schedule `W[16]` and the working state `a…h` and marks the transform
  pending. The input buffer is free again immediately.
- `loop()` calls `igcHmac.step()` only in idle passes (`newData == 0`). Each
  call computes `IGC_SHA256_ROUNDS_PER_STEP` (4) rounds, about 0.5 ms. A block
  finishes within one or two sensor periods.
- `W` and `a…h` are static and shared, because only one transform can be
  pending. Starting another transform, `finalize()` or `digest()` first
  finishes the pending one, so the signature never depends on how often
  `step()` was called.
- The 32-bit rotations are written as whole-byte moves plus at most 4
  single-bit rotations. AVR has no barrel shifter, and gcc otherwise compiles
  `x >> 13` as a loop of 13 shifts. An empty `asm` keeps gcc from folding the
  pieces back together. This removed all shift loops from the rounds
  (estimated ~3× faster) and made the code smaller.

## Kalman filter: `kalmanvert`

Two states: altitude `p` and vertical speed `v`. The measured vertical
acceleration drives the prediction and the barometric altitude corrects it
(σp = 0.1 m, σa = 0.3 m/s²).

- `update1(a, t)`: `p += v·dt + a·dt²/2`, `v += a·dt`, covariance prediction.
  `dt` is capped at 200 ms.
- `update2(z)`: Kalman gain from `p11` and the measurement variance,
  correct `p` and `v`, update covariance.
- `calibratePosition(gpsAlt)` only stores an offset. `getPosition()` stays pure
  ISA (used for IGC pressure altitude), `getCalibratedPosition()` adds the
  GPS offset.

## Audio: `beeper` and `toneAC`

| Zone | Vertical speed | Sound |
|---|---|---|
| Sink | < −2.5 m/s | continuous tone, 386 Hz + 33 Hz per m/s (lower when sinking faster) |
| Silent | −2.5 to −0.3 m/s | nothing |
| Near climb | −0.3 to +0.2 m/s | short beeps (disabled by default) |
| Climb | > +0.2 m/s | on/off pattern, 386 Hz + 141 Hz per m/s |

Zone changes have a 0.1 m/s hysteresis. In the climb zone a new pitch is only
applied at the start of the next beep, so a beep never changes pitch halfway.
`toneAC` drives the piezo push-pull on OC1A/OC1B with phase-correct PWM. The
volume is set by the duty cycle.

## GPS path

1. **`SerialNmea` (RX interrupt)** stores bytes in a 128-byte ring buffer
   while checking the tag and checksum on the fly. Only valid `$GPRMC` and
   `$GPGGA` sentences are kept. Without Bluetooth the bytes are dropped after
   use instead of being forwarded to TX.
2. **`loop()`** locks the newest RMC or GGA (`lockRMC()`/`lockGGA()`) and feeds
   it byte by byte to `NmeaParser` and, for GGA, to `IGCSentence`.
3. **`NmeaParser`** extracts speed and date (RMC), time, satellites, HDOP and
   altitude (GGA) as fixed-point integers.

Variometer states:

| State | Next when |
|---|---|
| `INITIAL` | RMC has a date |
| `DATE_RECORDED` | GGA altitude with HDOP < 3.5 → baro offset calibrated, IGC file created |
| `CALIBRATED` | >15 s since power-on, \|v\| > 0.5 m/s and speed > 10 km/h |
| `FLIGHT_STARTED` | (near-climb sounds enabled if configured) |

## IGC logging

### File

`createSDCardTrackFile()` creates `YYMMDDNN.IGC` on the FAT16 card.
`NN` is the first free number from 01, and is also the flight number
in `HFDTEDATE`. The writer keeps a single 512-byte block buffer.

### Header

The header is one PROGMEM string built from `VarioSettings.h`. The date
placeholder is replaced with the GPS date and the flight number is added:

```
AXYYHASKO1
HFDTEDATE:060626,01
HFPLTPILOTINCHARGE:David Hasko
HFCM2CREW2:NIL
HFGTYGLIDERTYPE:OZONE Zeolite 2
HFGIDGLIDERID:D-HASK
HFDTMGPSDATUM:WGS84
HFRFWFIRMWAREVERSION:1.3
HFRHWHARDWAREVERSION:0.2
HFFTYFRTYPE:Hasko,HaskoVarioGPS
HFGPSRECEIVER:Quectel,L80,22,9999
HFPRSPRESSALTSENSOR:TE,MS5611,9999
HFFRSSECURITYOK
HFALGALTGPS:GEO
HFALPALTPRESSURE:ISA
```

`XYY` is the IGC code for recorders without IGC approval. The CIVL
`HFALG`/`HFALP` records follow the required IGC records.

### B records

`IGCSentence` builds each record directly into a 37-byte buffer in `loop()`,
parsing GGA fields into fixed positions:

```
B HHMMSS DDMMmmmN DDDMMmmmE V PPPPP GGGGG CRLF
```

- Written only if time, latitude, longitude, both hemispheres and a real fix
  (GGA quality 1–5) are present. Dead reckoning (6) and no fix (0) are
  dropped.
- `V` and GNSS altitude `00000` when GGA has no altitude (2D fix), otherwise
  `A`.
- Pressure altitude from `kalmanvert.getPosition()` (ISA), rounded.
- Negative altitudes as `-0012`. Lat/long with fewer than 3 minute decimals
  are padded with zeros.

### Signature (G record)

`writeFeed()` sends every header and B record byte both to the file and to
the HMAC. On shutdown (long press or < 3.3 V) `loop()` calls
`finalizeIGCFile()` before going to sleep. It finishes the HMAC, writes two
lines `G` + 32 hex characters and syncs the file. `tools/igc_verify.py`
recomputes the HMAC over all bytes before the first G line.

The file is only signed on a clean shutdown. A watchdog reset or sudden power
loss leaves it without a G record and loses the last partial block.

## Power management: `varioPower`

- **Power on:** button held through the 1 s start delay, regulator on (PD5).
- **Power off:** button held > 1 s, or battery < 3.3 V, sets
  `shutdownPending`. `loop()` finalises the IGC file, then `completeShutdown()`
  plays the shutdown sound, disables TWI, UART, SPI and ADC, drives all used
  pins low, switches the regulator off and enters power-down sleep. A LOW level
  on the button wakes it, and the watchdog resets the MCU.
- **Battery:** checked every 2 minutes on A1. Below 3.5 V two short warning
  beeps (vario beeps muted meanwhile), below 3.3 V shutdown.
- **Watchdog:** 4 s, reset in every `loop()` pass.
- **Firmware update:** button still held after the boot melody → jump to the
  optiboot-sdcard bootloader at `0x7800`, which flashes `FIRM.HEX`.

## Resource budget

| Resource | Used | Available |
|---|---|---|
| Flash | 27794 bytes | 30720 (below the bootloader) |
| RAM, static | 1457 bytes | 2048 |
| Stack, deepest path (IGC file creation + nested TWI interrupt) | ~360 bytes | 591 |

Large RAM users: SD block buffer (512), serial ring buffer (128 + state),
HMAC state (2 × ~137), SHA-256 work area (99).

## Testing

- `python3 tools/crypto_test/run_test.py` builds `igcrypto` for the PC with
  sanitizers and checks 967 digests against Python's `hashlib`/`hmac`, with
  1, 4, 7 and 64 rounds per step, plus a simulated 1-hour IGC file verified
  with `igc_verify.py`.
- `python3 tools/igc_verify.py FILE.IGC` checks the G record of a real flight.

## Measuring on hardware

The timing figures above are estimates. To measure them, toggle a free pin
around the code of interest and look at it with an oscilloscope or logic
analyser, e.g. set the pin high at the start of `loop()` and low at the end
to see every pass length. Don't use PC2 for this: it is wired to the GNSS
receiver's RX input.
