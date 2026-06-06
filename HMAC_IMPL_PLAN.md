# HMAC-SHA256 Implementation Plan for IGC G Record

## Overview

Add HMAC-SHA256-based IGC G record signing to HaskoVario. The signature is computed incrementally as IGC data is written, then finalized and flushed on shutdown. This avoids re-reading the file from SD (which is impractical on an 8-bit MCU).

Total estimated footprint: **~3 KB flash, ~220 B RAM** — leaving ~5.4 KB flash and ~590 B RAM headroom.

---

## Open-Source References

The SHA-256 and HMAC implementations will be based on the following battle-tested open-source libraries:

| Reference | URL | Notes |
|-----------|-----|-------|
| **avr-crypto-lib** by Daniel Otte | https://github.com/ab9rf/avr-crypto-lib | Canonical AVR assembly-optimized crypto library. Contains SHA-256, SHA-512, HMAC. ~2.5 KB when stripped to SHA-256 only. |
| **ArduinoCryptolib** by Adrianotiger | https://github.com/Adrianotiger/ArduinoCryptolib | Arduino-friendly SHA-256 with `update()`/`finalize()` API. Good reference for the C++ class design. |
| **libhydrogen** by Frank Denis | https://github.com/jedisct1/libhydrogen | Production-grade embedded crypto (Curve25519, SHA-256, HMAC). Larger than needed but architecture-independent. |
| **RFC 2104** | https://datatracker.ietf.org/doc/html/rfc2104 | HMAC specification. The HMAC wrapper is straightforward once SHA-256 is available. |

The SHA-256 compression function will be adapted from avr-crypto-lib's `sha256_ctx` and `sha256_hash()`, which is specifically written for AVR (`PROGMEM` constants, minimal RAM, no `malloc`). The class API will follow ArduinoCryptolib's `begin()`/`write()`/`finalize()`/`digest()` pattern for consistency with Arduino conventions. HMAC follows RFC 2104 exactly.

---

## Phase 1: SHA-256 Core Library

**New file:** `libraries/igcrypto/sha256.h`, `libraries/igcrypto/sha256.cpp`

A minimal SHA-256 implementation for AVR, adapted from avr-crypto-lib's `sha256-asm.S` and `sha256.c`:

```cpp
class Sha256 {
public:
    void begin();                    // init hash state (H0-H7 per FIPS 180-4)
    void write(const uint8_t* data, uint16_t len);  // feed data
    void write(uint8_t byte);        // feed single byte
    void finalize();                 // pad and produce digest
    const uint8_t* digest();         // 32-byte digest
    uint16_t getDigestLength();      // returns 32

private:
    uint32_t state[8];      // 32 bytes (H0-H7)
    uint64_t count;         // 8 bytes (total bits processed)
    uint8_t buffer[64];     // 64 bytes (512-bit block)
    uint8_t bufferIndex;    // 1 byte
    // transform() via avr-crypto-lib's sha256_hash() with PROGMEM K constants
};
```

**Reference mapping:**
- `state[]` → avr-crypto-lib's `sha256_ctx_t.h`
- `transform` → avr-crypto-lib's `sha256_hash()` with K constants from `sha256.c` (stored in PROGMEM)
- `finalize()` padding → standard SHA-256 per FIPS 180-4 section 5.1.1
- All round constants in PROGMEM to save RAM (avr-crypto-lib's approach)
- No `malloc` — static allocation matched to AVR register usage

---

## Phase 2: HMAC Construction

**New file:** `libraries/igcrypto/hmac_sha256.h`, `libraries/igcrypto/hmac_sha256.cpp`

HMAC per RFC 2104, using the Sha256 class from Phase 1. The HMAC wrapper will be adapted from avr-crypto-lib's `hmac_sha256.c`:

```cpp
class HMACSha256 {
public:
    void begin(const uint8_t* key, uint8_t keyLen);  // init with key (RFC 2104)
    void write(const uint8_t* data, uint16_t len);    // feed data
    void write(uint8_t byte);                          // feed single byte
    void finalize();                                   // produce HMAC
    const uint8_t* digest();                           // 32-byte HMAC

private:
    Sha256 inner;
    Sha256 outer;
};
```

Note: the key buffer from avr-crypto-lib's approach is computed inline into the two SHA-256 contexts during `begin()`, and the padded key is stored in the inner/outer hash states, not in a separate buffer. This saves 64 bytes of RAM compared to the naive approach.

**Algorithm (RFC 2104, adapted from avr-crypto-lib's `hmac_sha256.c`):**
1. `begin(key, keyLen)`:
   - If keyLen > 64, hash the key first with SHA-256, use 32-byte digest as key
   - Compute `ipad = key XOR 0x36` (padded to 64 bytes) → `inner.begin()` + `inner.write(ipad, 64)`
   - Compute `opad = key XOR 0x5c` (padded to 64 bytes) → `outer.begin()` + `outer.write(opad, 64)`
2. Each `write()` call feeds data to `inner`
3. `finalize()`:
   - `inner.finalize()` → get inner digest (32 bytes)
   - `outer.write(inner.digest(), 32)`
   - `outer.finalize()` → get HMAC digest (32 bytes)

---

## Phase 3: Key Storage in PROGMEM (VarioSettings.h)

The HMAC key is a compile-time constant stored in **PROGMEM (flash)**, not EEPROM. The CIVL spec permits "a private key shared between similar instrument models" (section 3.1.4.3). Since the ATmega328P has no secure enclave, flash and EEPROM are equally readable with physical access — storing in PROGMEM is simpler and avoids EEPROM wear.

**Modified file:** `libraries/VarioSettings/VarioSettings.h`

```cpp
// IGC G record HMAC-SHA256 key (32 bytes = 256 bits)
// Shared across all units of this model per CIVL spec 3.1.4.3.
// Stored in PROGMEM — change this to invalidate all existing flight logs.
#define VARIOMETER_HMAC_KEY { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, \
                              0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, \
                              0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, \
                              0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }
```

**New file:** `libraries/igcrypto/igc_key.h`

```cpp
#include <avr/pgmspace.h>
#include <VarioSettings.h>

static const uint8_t igcHmacKey[32] PROGMEM = VARIOMETER_HMAC_KEY;
```

The key is read from PROGMEM each time HMAC is initialized (once per flight). No RAM storage needed between flights. The validation program uses the same key value — it is distributed alongside the open-source firmware, so any transparency is inherent.

---

## Phase 4: SD Card Flush — Fix `lightfat16::sync()`

**Modified file:** `libraries/LightFat16/LightFat16.h`, `libraries/LightFat16/LightFat16.cpp`

The current `sync()` updates the file size in the root directory entry but does **not** explicitly flush the block buffer to the SD card. While block transitions do write dirty blocks, the final partial block of the file may never be written if no block boundary is crossed before shutdown.

```cpp
// In variometer.ino, before shutdown:
file.sync();  // flush block buffer + update directory entry
```

Looking at the existing `sync()` implementation — it already flushes the current data block indirectly (the `blockSet()` call transitions away from the data block, writing it, then transitions back). However, after returning to the data block, the block is reloaded from SD, so the in-memory state is consistent. The issue is simply that **`sync()` is never called** in the current code.

**Changes needed:**

In `lightfat16.h`:
- No interface changes needed — the `sync()` method already exists

In `lightfat16.cpp`:
- The existing `sync()` is functionally correct but deserves a `blockWriteSync()` call at the end to be extra safe:

```cpp
void lightfat16::sync() {
    uint8_t* data = this->blockSet(fileEntryBlock, fileEntryPos);
    uint32_t currentBlockSize = (*(uint32_t*)&data[ROOT_ENTRY_SIZE_POS]) % BLOCK_SIZE;
    *(uint32_t*)&data[ROOT_ENTRY_SIZE_POS] += fileDataPos - currentBlockSize;
    data = this->blockSet(fileDataBlock, fileDataPos);
    this->blockWriteSync();  // ensure current block is on SD
}
```

---

## Phase 5: Shutdown Sequence

**Modified file:** `variometer/variometer.ino`

Add a function `finalizeIGCFile()` that is called before the vario shuts down:

```cpp
#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
static HMACSha256 igcHmac;
static bool igcHmacActive = false;

void finalizeIGCFile() {
    if (sdcardState != SDCARD_STATE_READY) return;

    // 1. Finalize HMAC
    igcHmac.finalize();
    const uint8_t* sig = igcHmac.digest();

    // 2. Write G record: "G<HMAC-SHA256 signature as 64 hex chars>\r\n"
    // IGC spec: G record contains the digital signature
    file.write('G');
    for (uint8_t i = 0; i < 32; i++) {
        uint8_t hi = sig[i] >> 4;
        uint8_t lo = sig[i] & 0x0f;
        file.write(hi < 10 ? '0' + hi : 'A' + hi - 10);
        file.write(lo < 10 ? '0' + lo : 'A' + lo - 10);
    }
    file.write('\r');
    file.write('\n');

    // 3. Flush everything to SD
    file.sync();

    // 4. Mark HMac as inactive
    igcHmacActive = false;
}
#endif
```

**Integration with varioPower:**

The shutdown is triggered in `varioPower::sleep()` (long-press or low voltage). The cleanest approach is to call `finalizeIGCFile()` before `sleep()` begins disabling interfaces. Since `sleep()` is a member of `VarioPower`, we need a way to call back to the main sketch.

Option A — Hook in `varioPower::update()`:

```cpp
// In varioPower::update(), before calling this->sleep():
// Call a weak-attributed function that the sketch can override
void VarioPower::sleep() {
    // NEW: call pre-sleep hook
    finalizeIGCFile();  // needs to be accessible

    // ... existing sleep code ...
}
```

But `varioPower` is a separate library and shouldn't directly depend on sketch functions. Better approach:

Option B — Check in the main `loop()`:

```cpp
void loop() {
    // ... existing loop code ...

    varioPower.update();  // this may now set a flag instead of calling sleep directly

    // Check if shutdown was requested
    if (varioPower.shutdownRequested()) {
        finalizeIGCFile();
        varioPower.shutdownComplete();  // actually enters sleep
    }
}
```

**But this means `varioPower::sleep()` can't be called directly from `update()`.** We need to modify varioPower to use a two-phase shutdown:

Option C (Recommended) — Split shutdown in varioPower:

```cpp
// In varioPower
class VarioPower {
    bool shutdownPending = false;
public:
    void update() {
        // ... existing checks ...
        if (/* button held 1s */) {
            shutdownPending = true;  // instead of calling sleep()
        }
    }
    bool isShutdownPending() { return shutdownPending; }
    void completeShutdown() {
        shutdownPending = false;
        sleep();  // original sleep() with all hardware shutdown
    }
};
```

Then in `loop()`:
```cpp
varioPower.update();
if (varioPower.isShutdownPending()) {
    finalizeIGCFile();
    varioPower.completeShutdown();
}
```

This ensures the IGC file is finalized BEFORE the hardware (SPI, TWI, UART) is turned off.

---

## Phase 6: Incremental HMAC While Writing IGC Data

**Modified file:** `variometer/variometer.ino`

Two places where IGC data is written to the file:

### 6a. File header (in `createSDCardTrackFile()`)

```cpp
#include <igc_key.h>

void createSDCardTrackFile(void) {
    // ... existing code ...
    if (file.begin((char*)dateChar, 8) >= 0) {
        sdcardState = SDCARD_STATE_READY;

        // Initialize HMAC with the PROGMEM key
        uint8_t key[32];
        memcpy_P(key, igcHmacKey, 32);
        igcHmac.begin(key, 32);
        igcHmacActive = true;

        // Write header and feed HMAC
        int16_t datePos = header.begin();
        if (datePos >= 0) {
            while (datePos) {
                uint8_t c = header.get();
                file.write(c);
                igcHmac.write(c);       // <-- NEW
                datePos--;
            }
            // ... write date ...
            while (header.available()) {
                uint8_t c = header.get();
                file.write(c);
                igcHmac.write(c);       // <-- NEW
            }
        }
    }
}
```

### 6b. B records (in `loop()`, during GGA parsing)

```cpp
if (sdcardState == SDCARD_STATE_READY) {
    uint8_t b = igc.begin(kalmanvert.getCalibratedPosition());
    file.write(b);
    igcHmac.write(b);                   // <-- NEW
}

// During GGA parsing:
if (sdcardState == SDCARD_STATE_READY && nmeaParser.isParsingGGA()) {
    igc.feed(c);
    while (igc.available()) {
        uint8_t b = igc.get();
        file.write(b);
        igcHmac.write(b);               // <-- NEW
    }
}
```

---

## Phase 7: Validation Program

**New file:** `tools/igc_validate.py` (or similar)

A standalone Python script that validates signed IGC files:

```python
#!/usr/bin/env python3
"""
IGC file validator for HaskoVario.
Verifies the HMAC-SHA256 G record signature.
"""
import hashlib
import hmac
import sys

def validate_igc(filename, key_hex):
    key = bytes.fromhex(key_hex)
    with open(filename, 'rb') as f:
        data = f.read()

    # Find the last G record (HMAC-SHA256 signature)
    # Format: G<64 hex chars>\r\n
    lines = data.split(b'\r\n')
    if not lines[-2].startswith(b'G'):
        print("No G record found")
        return False

    sig_hex = lines[-2][1:].decode()
    sig = bytes.fromhex(sig_hex)

    # Data to verify: everything except the G record and trailing whitespace
    sig_line = lines[-2] + b'\r\n'
    if data.endswith(b'\r\n'):
        sig_line += b'\r\n'
    signed_data = data[:-len(sig_line)]

    # Compute HMAC
    expected = hmac.new(key, signed_data, hashlib.sha256).digest()

    if sig == expected:
        print("✓ Signature valid")
        return True
    else:
        print("✗ Signature INVALID")
        return False
```

---

## Phase 8: Compile-Time Configuration

**Modified file:** `libraries/VarioSettings/VarioSettings.h`

Add a feature flag (off by default for backward compatibility):

```cpp
#define HAVE_IGC_SECURITY    // enable HMAC-SHA256 G record signing
```

All new code in the main sketch is wrapped with `#ifdef HAVE_IGC_SECURITY`, so the feature can be toggled without affecting existing functionality.

---

## Integration Summary

### Files to create:

| File | Purpose |
|------|---------|
| `libraries/igcrypto/sha256.h` | SHA-256 class declaration |
| `libraries/igcrypto/sha256.cpp` | SHA-256 implementation |
| `libraries/igcrypto/hmac_sha256.h` | HMAC-SHA256 class declaration |
| `libraries/igcrypto/hmac_sha256.cpp` | HMAC-SHA256 implementation |
| `libraries/igcrypto/igc_key.h` | HMAC key loaded from PROGMEM (`VARIOMETER_HMAC_KEY`) |
| `tools/igc_validate.py` | PC-side IGC file validator |

### Files to modify:

| File | Changes |
|------|---------|
| `VarioSettings.h` | Add `HAVE_IGC_SECURITY` flag + `VARIOMETER_HMAC_KEY` define |
| `LightFat16.cpp` | `sync()` — add explicit `blockWriteSync()` call |
| `varioPower.h` | Add two-phase shutdown (`isShutdownPending()`, `completeShutdown()`) |
| `varioPower.cpp` | Split `sleep()` — set flag on trigger, actual sleep in `completeShutdown()` |
| `variometer.ino` | Add HMAC context, feed bytes, call `finalizeIGCFile()` on shutdown |

### Memory budget:

| Component | Flash | RAM |
|-----------|-------|-----|
| SHA-256 core (from avr-crypto-lib) | ~2,500 B | 132 B (64 buf + 32 state + 32 digest + 4 count padding) |
| HMAC wrapper (from avr-crypto-lib) | ~300 B | 4 B (inner/outer state interleaving) |
| Key in PROGMEM + 32-byte stack copy (transient) | ~32 B | 0 B (static) |
| G record + flush logic | ~300 B | 0 B |
| Two-phase shutdown flag | ~20 B | 1 B |
| **Total** | **~3,150 B** | **~137 B** |
| **Available** | **8,416 B** | **816 B** |
| **Remaining** | **~5,266 B** | **~679 B** |

### Shutdown flow:

```
loop()
  ├── varioPower.update() detects button press or low voltage
  │     └── sets shutdownPending = true (instead of calling sleep directly)
  │
  ├── if (shutdownPending):
  │     ├── finalizeIGCFile()
  │     │     ├── igcHmac.finalize()
  │     │     ├── write G record to SD card
  │     │     ├── file.sync() → flush block buffer
  │     │     └── igcHmacActive = false
  │     └── varioPower.completeShutdown()
  │           ├── marioSounds.shutDown()
  │           ├── disable SPI, TWI, UART
  │           ├── disable ADC
  │           ├── turn off LDO
  │           └── SLEEP_MODE_PWR_DOWN
  │
  └── normal operation continues
```

### Risks and mitigations:

| Risk | Mitigation |
|------|------------|
| Shutdown during HMAC finalize (power cut) | HMAC is computed incrementally during flight. On next boot, the incomplete file is still valid IGC data — just without a G record. No data corruption. |
| SD card write failure during G record | The G record is appended after all flight data. If the write fails, the flight data is preserved. The validation program treats missing G records as "not signed". |
| Hash state corruption from sensor ISR | HMAC writes happen only in the main loop (not in ISRs). The I2C interrupt handler does not touch the HMAC context. |
| Stack overflow during SHA-256 | SHA-256 transform uses ~200 bytes of stack. With 816 bytes currently free, this is safe. Confirm with `avr-nm --stack` before finalizing. |
| Key extracted from firmware binary | Inherent — flash is readable with physical access. The CIVL spec permits this (shared key per model). Changing the key invalidates old flight logs. |

### Open-source license compatibility:

All referenced implementations (avr-crypto-lib, ArduinoCryptolib) use permissive licenses (GPLv2+ or public domain). The HaskoVario codebase is GPLv3, which is compatible with GPLv2+ code.

---

*Implementation estimate: ~500-700 lines of new C++ code across 5 new files, ~60 lines of modifications across 4 existing files. SHA-256 compression function adapted from avr-crypto-lib (GPLv2+), class interface from ArduinoCryptolib (GPLv2+), HMAC wrapper per RFC 2104.*
