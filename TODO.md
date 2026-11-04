# TODO

Open items, roughly by priority. Spec references: IGC Technical Specification
(Jan 2026, AL10) and CIVL Flight Recorder Specification (FAI S7H, 2024), both in
`igc specifications/`.

## Needs hardware testing

- [ ] **MPU FIFO reset with the DMP running.** The sensor chain now resets the
  FIFO from the I2C interrupt chain (`USER_CTRL = DMP_EN|FIFO_RST`, then
  `DMP_EN|FIFO_EN`) without stopping the DMP. Check that the vario reacts
  normally after power-on and over long sessions. If resets leave partial
  packets, the `fifo_count % 32` check resets again; watch for that looping.
- [ ] **Long session beep check**, especially right after GPS fix, when the IGC
  file is created (SD re-init, scans and header hashing, 0.1 s to over 1 s
  in one loop pass).
- [ ] **IGC file from a real flight** passes `tools/igc_verify.py` and opens in
  common analysis software. Balcony test 2026-11-04 (`26110401.IGC`) passed:
  valid G record, 329 well-formed B records at 1 s, correct header.

## GNSS receiver (Quectel L80)

Sources: `Datasheets/L80_Hardware_Design_V1.1.pdf` (Table 5),
`Quectel_L80_GPS_Protocol_Specification_V1.4.pdf` (ch. 2, 3.18, 3.27, 3.41,
ch. 4), `Quectel_GNSS_SDK_Commands_Manual_V1.4.pdf`,
`Quectel_GNSS_Low_Power_Mode_Application_Note_V2.0.pdf`.

Factory defaults: 9600 baud, 1 Hz, WGS84, RMC+VTG+GGA+GSA+GSV+GLL+GPTXT
output, SBAS on, AIC on, EASY on, LOCUS off, PPS 100 ms pulse. Static
navigation, navigation mode (PMTK886) and FLP default are not documented.

- [ ] **Connect the L80 RX line to the MCU, open-drain with one pull-up.**
  It is not connected yet, so the `HAVE_GNSS_CONFIG` commands sent at boot
  never reach the receiver and it runs on factory defaults. The ATmega runs
  from the Li-ion cell (up to 4.2 V) but L80 RXD1 allows max 3.6 V
  (VIH 2.0–3.6 V, VIL ≤ 0.8 V). The datasheet documents no internal pull or
  clamp on RXD1, so a series resistor alone would rely on undocumented
  clamp diodes and back-power the L80 in sleep. Wiring:
  ```
  L80 3.3 V ──[ 4.7k–10k ]──┬── L80 RXD1 (module pin 1)
                            │
  ATmega PC2 / A2 (pin 25) ─┘   direct
  ```
  Rise time ~0.2 µs with 10 k / 20 pF, fine for 9600 baud (104 µs/bit).
- [ ] **`WSerial` open-drain mode** (needed for the wiring above). Send 0 =
  output low; send 1 = input with `PORT` bit 0. Idle and constructor as
  input, never `PORT` high: the internal pull-up goes to battery voltage.
  With the pull-up powered from the L80 rail, the line drops to 0 V when the
  regulator is off.
- [ ] **PC2 in `VarioPower::sleep()`**: add it to the pins set low (or leave
  it as input without pull-up) so it can't feed the unpowered L80.
- [ ] **L80 TX → ATmega RX margin.** ATmega VIH = 0.6 × VCC = 2.52 V at
  4.2 V; L80 TXD1 VOH min is 2.4 V (typ. ~3.1 V). Slightly out of spec at
  full charge, works in practice (balcony test). Keep in mind for the next
  PCB revision.
- [ ] **Drop FLP mode from the boot commands.** `$PQFLP,W,1,1` enables
  "fitness low power" mode, made for wearables while walking/running:
  duty-cycled tracking at ~40 % current with a "tiny accuracy trade-off",
  "high dynamic performance will have a little impact", SBAS affected, EASY
  and PPS disabled. It also needs FLP-capable L80 firmware, takes 5 min to
  fully enter, and Quectel recommends 115200 baud for it. Not a good fit for
  a flight recorder (CIVL 3.3.5 asks for a mode suitable for HPG). The
  `,1` saves it in the L80's flash, so removing the line is not enough
  once it has been sent: send `$PQFLP,W,0,1*21` instead (normal mode,
  saved). Same for `$PQTXT` (saved) and `$PQ1PPS` (always saved).
- [ ] **Static navigation (frozen position).** In the balcony test the
  position was identical to the last digit for 294 s and the GPS altitude
  for 298 s with factory defaults, so static navigation (PMTK386: "if the
  actual speed is below the threshold, output position will keep the same
  and output speed will be zero", 0–2 m/s, 0 = off) appears to be on by
  default. Its default value is not documented and the L80 has no query
  for it. Add `$PMTK386,0*23` to `gnssCfgCmds` and check that a stationary
  position wanders by a few metres. Not saved, so it has to be sent at
  every boot (it is).
- [ ] **Navigation mode (PMTK886).** 0 = normal, 1 = fitness (<5 m/s), 2 =
  aviation (high dynamics, large accelerations), 3 = balloon (vertical
  movement). Default not documented, probably normal. Compare normal and
  aviation on a real flight (`$PMTK886,0*28` / `$PMTK886,2*2A`).
- [x] **GGA altitude datum: above the geoid, `HFALGALTGPS:GEO` is correct.**
  The L80 spec calls the GGA altitude "according to WGS84 ellipsoid", which
  conflicts with standard NMEA. Checked with the balcony test (2026-11-04):
  GPS altitude 593 m; terrain 584 m (EU-DEM 25 m) / 587 m (Copernicus 90 m)
  plus ~5 m balcony = 589–592 m above sea level. The geoid there is 47–48 m
  above the ellipsoid (EGM96/EGM2008), so an ellipsoid altitude would have
  read ~637–640 m. The 1–4 m difference to sea level is within GPS vertical
  accuracy.
- [ ] **RMC speed is parsed 10× too small.** L80 RMC speed has 2 decimals
  (`0.00`), `NMEA_RMC_SPEED_PRECISION` is `1000.0` (3 decimals). Flight start
  needs 10 km/h, which currently means ~100 km/h real ground speed, so
  `FLIGHT_START` practically never triggers. Set it to `100.0` (check a raw
  RMC first). `NMEA_GGA_TIME_PRECISION` (100) doesn't match the L80's 3
  decimals either, but `nmeaParser.time` is unused.

## IGC / CIVL compliance

- [ ] **GNSS drop-out recording** (CIVL 3.3.4, IGC A4.1.3). B records currently
  stop when there is no valid position. The spec wants them to continue at the
  fix interval with `V`, the last known lat/long, the pressure altitude and
  GNSS altitude `00000`. Needs a 1 s timer when GGA has no fix, the last
  position kept, and time from `millis()` since the last GGA time.
- [ ] **Signature only on clean shutdown** (CIVL 3.1.3). A watchdog reset,
  brown-out or pulled battery leaves the file without a G record. Option:
  rewrite the G record after each B record (finalize a copy of the HMAC
  state, overwrite the G lines in place, sync).
- [ ] **Pressure altitude source** (CIVL 3.3.5). B records use
  `kalmanvert.getPosition()` (baro fused with the accelerometer). The spec
  limits processing to the minimum required by the sensor; `ms.getAltitude()`
  (raw ISA) would comply. Never use the GNSS-calibrated altitude here.
- [ ] **Max altitudes in HFGPS / HFPRS** are placeholders (`9999`). Use the real
  limits: L80 18000 m (hardware design, "Maximum Altitude 18,000m"), and
  the MS5611 range through the firmware's altitude formula (~25900 m at
  10 mbar).
- [ ] **HMAC key protection** (CIVL 3.1.4.4, 3.1.5). The key is in the public
  repo, in `igc_verify.py` and readable from the MCU flash. Fine for personal
  logs, blocks any CIVL approval. Would need a private key outside the repo,
  lock bits, and a validator that doesn't contain the key.
- [ ] **Pilot/glider data entry** (CIVL 3.1.2). Only compile-time settings now.
- [ ] **File name.** `YYMMDDNN.IGC` is not the IGC long or short style; tolerated
  for non-IGC recorders. The FAT16 driver only supports 8.3 names, so the
  short style `YMDCXXXF.IGC` would be the reachable option.

## Code

- [ ] **Regression test for B records**: add the PC simulation used for the
  compliance check (real `IGCHeader`, `NmeaParser`, `IGCSentence`, HMAC, edge
  case GGA sentences, strict format regex) to `tools/`.
- [ ] **Remove dead code** `MPU6050::resetFIFO()` and `MPU6050::getFIFO()`
  (blocking I2C, no longer called; must never be called while INT0 is active).
- [ ] **Low-battery warning restores volume to 10** instead of
  `VARIOMETER_BEEP_VOLUME` (`varioPower.cpp`, case 3).
- [ ] **`ms5611::getAltitude()` uses `pow()`**, several thousand cycles per
  call; a polynomial approximation would be faster and save ~1 KB flash.
- [ ] **Instrumentation** to confirm the sensor-chain fix in the field: count
  FIFO resets and `I2C::kick()` stalls. No Bluetooth module is fitted, so it
  would need another output (e.g. an L record or the SD card).
- [ ] **`newData` step machine**: the ISR can set `newData = 1` while `loop()`
  does `newData++`, restarting a step sequence. Harmless today (only a
  skipped sample); keep in mind if steps get longer.

## Loop timing (beeper rhythm)

See "Main loop timing and the beeper" in HOW_IT_WORKS.md.

- [ ] **Measure pass lengths on hardware** (pin toggle + scope/logic analyser)
  to replace the estimates in HOW_IT_WORKS.md.
- [ ] **SD block writes stall one pass** for ~5 ms plus two card busy waits
  (typ. 1–5 ms each, up to 600 ms timeout) every ~14 s, and longer at
  cluster boundaries. Now the most likely audible hiccup. Options: split
  the block switch (write data block / read entry / write entry) into
  separate idle passes like the SHA steps, or update the directory entry
  size only on sync instead of every block.
- [ ] **IGC file creation** stalls once for 0.1 s to over 1 s (SD re-init,
  directory and FAT scans). Usually before takeoff, but could happen in the
  air if the GPS fix comes late.

## Resource budget (Nov 2026)

Flash 27794 / 30720 bytes usable below the bootloader (0x7800), RAM 1457 /
2048 bytes static, deepest stack path ~360 bytes of the remaining 591.
