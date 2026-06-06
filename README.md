# HaskoVario

**Open-source zero-latency audio variometer for paragliding/hang-gliding
with GPS flight logging and IGC security — designed for low power on a
custom PCB.**

Built on the sensor fusion and Kalman filter core of
[GNUVario](https://github.com/prunkdump/GNUVario) by Baptiste PELLEGRIN,
this fork strips everything non-essential to deliver a focused, reliable
flight instrument.

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

## Building

The firmware uses [MiniCore](https://github.com/MCUdude/MiniCore) for the
ATmega328P. Install it first:

```bash
arduino-cli core update-index
arduino-cli core install MiniCore:avr
```

Compile with:

```bash
arduino-cli compile --fqbn MiniCore:avr:328:clock=8MHz_external,LTO=Os_flto,variant=modelP,bootloader=no_bootloader \
  --output-dir variometer/build/ variometer/
```

Output: `variometer/build/variometer.ino.hex` — rename to `FIRM.HEX` for
SD card updates.

## Custom Bootloader

The variometer uses a custom Optiboot variant (forked from the original
[GNUVario bootloader](https://github.com/prunkdump/GNUVario)) that adds
SD card firmware update capability:

**[optiboot-sdcard](https://github.com/Hasi123/optiboot-sdcard)** — A
modified Optiboot bootloader that reads `FIRM.HEX` from a FAT16 SD card
and flashes it to the MCU on startup. This allows updating the firmware
in the field without a USB programmer.

## Firmware Update — SD Card (field, no bootloader reflash)

1. Remove the SD card from the vario and connect it to your PC.
2. Format the SD card as **FAT** (not FAT32). Note: FAT only supports
   partitions up to 1 GB.
3. Copy `FIRM.HEX` to the root of the SD card.
4. Safely eject the SD card and insert it into the vario.
5. Power on the vario and **hold the power button** until the boot-up
   sound finishes.
6. (Optional) Delete `FIRM.HEX` from the SD card after the update to
   avoid re-flashing on every boot.

## Firmware Update — ISP Flash (first time / with bootloader)

Flash the bootloader and compiled firmware together via an ISP programmer
(e.g. USBasp). Set the fuses for 8 MHz external crystal:

```bash
avrdude -c usbasp -p m328p \
  -U lfuse:w:0xFF:m \
  -U hfuse:w:0xD2:m \
  -U efuse:w:0xFD:m \
  -U flash:w:path/to/variometer.ino.with_bootloader.hex
```

The bootloader hex is available from the
[optiboot-sdcard](https://github.com/Hasi123/optiboot-sdcard) repository.

## IGC Security (G Record Signing)

The variometer signs IGC flight logs with an HMAC-SHA256 G record
conforming to the
[CIVL IGC specification 3.1.4.3](https://www.fai.org/sites/default/files/documents/igc_specification_with_alticlass_and_fr_annexes_with_gnss_2022-1.pdf).

The G record contains the full 256-bit HMAC-SHA256 digest split across
two `G<64 hex chars>` lines, authenticating all header and B-record data.

### Enabling

`HAVE_IGC_SECURITY` is enabled by default in
`libraries/VarioSettings/VarioSettings.h`. To disable, comment out the
`#define`.

### Verifying

```bash
python3 tools/igc_verify.py path/to/log.igc
```

Sample signed files are in the `IGCs/` directory:
```bash
python3 tools/igc_verify.py IGCs/*.IGC
```

## Configuration: `VarioSettings.h`

All compile-time settings live in `libraries/VarioSettings/VarioSettings.h`.
Key categories:

| Category | Examples |
|----------|----------|
| **Feature flags** | `HAVE_SPEAKER`, `HAVE_ACCELEROMETER`, `HAVE_GPS`, `HAVE_SDCARD`, `HAVE_IGC_SECURITY` |
| **IGC header fields** | Pilot name, glider type/ID, model, firmware/hardware version |
| **IGC security** | `HAVE_IGC_SECURITY`, `VARIOMETER_HMAC_KEY` (32-byte HMAC key) |
| **Beep thresholds** | Sink/climb thresholds, near-climbing sensitivity |
| **Flight detection** | Min time/speed thresholds for start-of-flight logic |
| **GPS/baro calibration** | HDOP threshold for GPS altitude calibration |
| **Baud rates** | GPS/Bluetooth serial speed |
| **Pin assignments** | CS pins, interrupt pins, buzzer pins |
| **Climb/glide settings** | Integration time windows for climb rate and glide ratio |

## Where to Start

- Read the code documentation: [HOW_IT_WORKS.md](HOW_IT_WORKS.md)
- View the hardware schematic: [schematic.pdf](schematic.pdf)
- IGC header and crypto are in `libraries/GpsSentences/IGCSentence.cpp`
  and `libraries/igcrypto/`; configure in `VarioSettings.h`

## Credits

- **Baptiste PELLEGRIN** — Original
  [GNUVario](https://github.com/prunkdump/GNUVario) project and
  bootloader. All core sensor, audio, GPS, Kalman filter, and bootloader
  code is his work.
- **David Hasko** — Fork maintainer, custom PCB design, bootloader
  integration.
