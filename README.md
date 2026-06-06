# HaskoVario

**Open-source Arduino variometer for paragliding/hang-gliding.**

This is a fork of [GNUVario](https://github.com/prunkdump/GNUVario) by
Baptiste PELLEGRIN, modified to run on a **custom PCB** that omits the
screen and Bluetooth module. The focus is on a minimal, reliable
bare-bones variometer with audio feedback, GPS, and SD card flight
logging.

## Features

- High-precision barometric altitude (MS5611) fused with IMU vertical
  acceleration (MPU6050) via a Kalman filter
- Real-time climb/sink audio feedback (buzzer via toneAC)
- GPS ground speed, glide ratio, and barometric altitude calibration
- IGC flight logging to SD card (FAT16)
- **IGC security G-record signing** (HMAC-SHA256, CIVL spec 3.1.4.3)
- SD card-based firmware updates (no USB cable needed in the field)
- Low-battery monitoring and automatic shutdown
- Long-press power on/off with watchdog-based reset

## Custom PCB

This fork is designed for a PCB that strips out the optional Nokia 5110
screen and Bluetooth module from the original GNUVario design — just the
core sensors, GPS, buzzer, SD card, and battery management.

## Custom Bootloader

The variometer uses a custom Optiboot variant (forked from the original
[GNUVario bootloader](https://github.com/prunkdump/GNUVario)) that adds
SD card firmware update capability:

**[optiboot-sdcard](https://github.com/Hasi123/optiboot-sdcard)** — A
modified Optiboot bootloader that reads `FIRM.HEX` from a FAT16 SD card
and flashes it to the MCU on startup. This allows updating the firmware
in the field without a USB programmer.

## Firmware Update Instructions

1. Remove the SD card from the vario and connect it to your PC.
2. Format the SD card as **FAT** (not FAT32). Note: FAT only supports
   partitions up to 1 GB.
3. Download the latest `FIRM.HEX` from the
   [Releases page](https://github.com/Hasi123/HaskoVario/releases).
4. Copy `FIRM.HEX` to the root of the SD card.
5. Safely eject the SD card and insert it into the vario.
6. Power on the vario and **hold the power button** until the boot-up
   sound finishes.
7. (Optional) Delete `FIRM.HEX` from the SD card after the update to
   avoid re-flashing on every boot.

## IGC Security (G Record Signing)

The variometer can sign IGC flight logs with an HMAC-SHA256 G record
conforming to the
[CIVL IGC specification 3.1.4.3](https://www.fai.org/sites/default/files/documents/igc_specification_with_alticlass_and_fr_annexes_with_gnss_2022-1.pdf).

When enabled, every IGC file contains a `G` record with a 28-character
Base64 signature that authenticates the entire file content (header + all
B records). Anyone in possession of the shared key can verify that the
flight log has not been tampered with.

### Enabling

Uncomment the line in `libraries/VarioSettings/VarioSettings.h`:

```cpp
#define HAVE_IGC_SECURITY
```

### Building

```bash
arduino-cli compile --fqbn HaskoVario:avr:HaskoVario \
  --output-dir build/ variometer/
```

The crypto build uses ~28 KB flash / ~1.5 KB RAM.

### Verifying G Records

Use the included Python tool:

```bash
python3 tools/igc_verify.py path/to/log.igc
```

The script recomputes the HMAC-SHA256 digest from the file data and
compares it to the embedded G record. Sample signed IGC files (from
actual firmware runs) are in the `IGCs/` directory:

```bash
python3 tools/igc_verify.py IGCs/*.IGC
#   ✓ 26060600.IGC: G record VALID
#   ✓ 26060601.IGC: G record VALID
#   ✓ 26060602.IGC: G record VALID
```

### Key

The shared HMAC key is defined in `VarioSettings.h`:

```cpp
#define VARIOMETER_HMAC_KEY { ... }
```

In production, change this key to a secret value known only to
authorized verifiers.

### Implementation

The crypto components live in `libraries/igcrypto/`:

| File | Purpose |
|------|---------|
| `sha256.h/.cpp` | Standalone SHA-256 (rolling window `W[16]`, 64 B stack) |
| `hmac_sha256.h/.cpp` | HMAC-SHA256 wrapper |
| `igc_key.h` | Key accessor |

The IGC header (pilot name, glider type, model, etc.) is stored as a
single PROGMEM string assembled at compile time from the values in
`VarioSettings.h`. At file creation, the header is written byte-by-byte
to the IGC file via `pgm_read_byte()`, with the placeholder date
overwritten by the actual GPS date.

The integration in `variometer/variometer.ino` feeds the HMAC engine
as bytes are written to the IGC file: header bytes at file creation, and
B-record bytes as each GPS sentence is parsed. On power-off, the final
digest is computed and the G record is appended before `file.sync()`.

A **two-phase shutdown** (`varioPower::shutdownPending` /
`isShutdownPending()` / `completeShutdown()`) ensures the G record is
written and flushed before power is cut.

### Configuration

Change the pilot name, glider type, or model in
`libraries/VarioSettings/VarioSettings.h`:

```cpp
#define VARIOMETER_MODEL "HaskoVarioGPS"
#define VARIOMETER_PILOT_NAME "David Hasko"
#define VARIOMETER_GLIDER_NAME "OZONE Zeolite 2"
```

After changing these values, recompile and flash. The IGC header in
PROGMEM will be rebuilt automatically.

## Where to Start

- Read the code documentation: [HOW_IT_WORKS.md](HOW_IT_WORKS.md)
- Design and crypto analysis: [CRYPTO_ANALYSIS.md](CRYPTO_ANALYSIS.md)
- HMAC implementation plan: [HMAC_IMPL_PLAN.md](HMAC_IMPL_PLAN.md)
- View the hardware schematic: [schematic.pdf](schematic.pdf)
- IGC header and crypto are in `libraries/GpsSentences/IGCSentence.cpp`
  and `libraries/igcrypto/`; configure in `VarioSettings.h`
- Flash the custom bootloader first, then use the SD card for firmware
  updates

## Credits

- **Baptiste PELLEGRIN** — Original
  [GNUVario](https://github.com/prunkdump/GNUVario) project and
  bootloader. All core sensor, audio, GPS, Kalman filter, and bootloader
  code is his work.
- **David Hasko** — Fork maintainer, custom PCB design, bootloader
  integration.
