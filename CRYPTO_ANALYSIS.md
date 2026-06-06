# Cryptography Analysis: IGC/CIVL Flight Recorder G Record

## Specification Requirements

From the [FAI/CIVL Flight Recorder Specification 2024](https://www.fai.org/sites/default/files/civl/documents/sporting_code_s7_h_-_civl_flight_recorder_specification_2024.pdf), section 3.1.4:

- The G record is a **digital security signature** appended to the IGC file to verify data integrity and authenticity.
- Data protected: all records except H records with O source and L records without manufacturer identifier (so other parties can annotate without breaking the signature).
- **IGC specification** mandates **asymmetric cryptography** with private keys unique to each instrument.
- **CIVL specification** (this document) allows two approaches:
  1. HMAC (symmetric) — minimum **HMAC-SHA256** with a 256-bit key.
  2. A private key **shared between similar instrument models**.
- Under **Acquired Rights** (3.5.2): other industry-standard security algorithms may be acceptable if they provide strong security.
- The manufacturer must supply a **validation program** that recomputes the signature and compares it to the G record.
- Private/secret keys must be protected from read access on the device, from firmware update programs, and from the validation program.

## Current Status

The HaskoVario codebase **has no cryptography whatsoever**:

- No G record generation
- No hash functions (SHA-256 or otherwise)
- No key storage or management
- IGC file creation writes A, H, and B records only — the G record is never appended

All existing checksums are simple NMEA XOR parity (not cryptographic).

## Platform Constraints & Current Usage

The variometer runs on an **8-bit AVR ATmega328P** with the custom HaskoVario board definition:

| Resource | Limit | Current Usage | Available |
|----------|-------|---------------|-----------|
| Flash | 32,256 bytes | **23,840 bytes (73%)** | 8,416 bytes |
| RAM | 2,048 bytes | **1,232 bytes (60%)** | 816 bytes |
| Clock | 8 MHz (external) | — | — |
| Crypto HW | None | — | — |

Compiled with `HaskoVario:avr:HaskoVario` FQBN, LTO enabled, `-Os` optimization.  
Current feature set: `HAVE_SPEAKER`, `HAVE_ACCELEROMETER`, `HAVE_GPS`, `HAVE_SDCARD`, `HAVE_BLUETOOTH` (screen and voltage divisor disabled).

This leaves **~8 KB flash** and **~800 bytes RAM** for a crypto implementation.

## Public-Key Algorithm Evaluation

### 1. Ed25519 (EdDSA with Curve25519) — **Recommended**

| Property | Value |
|----------|-------|
| Public key size | 32 bytes |
| Private key size | 32 bytes |
| Signature size | 64 bytes |
| Code footprint | ~4-6 KB (libhydrogen or similar) |
| RAM usage | ~500 bytes |
| Signing speed | ~50 ms on 16 MHz AVR |

**Strengths:**
- Small keys and signatures — signatures fit easily in a single G record line (IGC lines are limited to ~80 chars)
- High security level (~128-bit)
- Constant-time implementation avoids timing side-channels
- Deterministic (same input always produces same signature — no RNG failures)
- Compact implementations exist for AVR: [libhydrogen](https://github.com/jedisct1/libhydrogen), [AVR-Crypto-Lib](https://github.com/cantora/avr-crypto-lib)
- The public key can be embedded in the IGC header as an H record (e.g., `HFGPGKEY:<base64 key>`) or in the validation program

**Weaknesses:**
- Larger code footprint than HMAC-SHA256 (~2 KB for a minimal implementation)
- Requires a random number generator for key generation (but not for signing, since Ed25519 is deterministic)

**G record format compatibility:** Ed25519 signatures are 64 bytes = 128 hex characters, which fits across two G records:
```
G<base64 or hex encoded signature part 1>
G<hex encoded signature part 2>
```
Or a single line if using base64 (~88 chars).

### 2. ECDSA with P-256 (secp256r1)

| Property | Value |
|----------|-------|
| Public key size | 64 bytes (uncompressed) / 33 bytes (compressed) |
| Private key size | 32 bytes |
| Signature size | 64 bytes (two 32-byte integers) |
| Code footprint | ~8-12 KB |
| RAM usage | ~1 KB |

**Strengths:**
- NIST standard, very widely supported
- Many existing validation tools

**Weaknesses:**
- Significantly larger code and RAM footprint than Ed25519
- Signing requires a **cryptographically secure random number generator** per-signature (nonce reuse is catastrophic)
- More complex implementation on constrained MCUs
- Uncompressed public keys are large; compressed keys require point decompression math

**Verdict:** Technically possible but impractical on ATmega328P. The per-signature RNG requirement is a serious risk.

### 3. ECDSA with P-384 (secp384r1)

| Property | Value |
|----------|-------|
| Public key size | 96 bytes (uncompressed) / 49 bytes (compressed) |
| Private key size | 48 bytes |
| Signature size | 96 bytes (two 48-byte integers) |
| Code footprint | ~15-20 KB |
| RAM usage | ~2+ KB |

**Verdict:** Exceeds available flash and RAM on ATmega328P. Not feasible.

### 4. RSA-2048

| Property | Value |
|----------|-------|
| Public key size | 256 bytes |
| Private key size | ~1 KB (CRT form) |
| Signature size | 256 bytes |
| Code footprint | ~6-10 KB |
| RAM usage | ~256+ bytes (modular exponentiation) |

**Verdict:** 256-byte signatures would span ~4 G record lines. The large RAM and flash usage and very slow signing (+ verification) on AVR make this impractical for an 8-bit MCU.

### 5. RSA-4096

**Verdict:** Completely infeasible. 512-byte signatures, massive memory and CPU requirements.

## Recommended Approach

### Option A: Ed25519 (asymmetric, true public-key crypto)

Satisfies the IGC-level requirement of asymmetric cryptography. Best fit for constrained embedded hardware.

**Implementation plan:**

1. **Integrate a lightweight Ed25519 library** — [libhydrogen](https://github.com/jedisct1/libhydrogen) is purpose-built for embedded systems and is about 4-6 KB of flash. Alternatively, a stripped-down [ed25519-donna](https://github.com/floodyberry/ed25519-donna) variant.

2. **Key generation** — Generate a unique key pair per instrument during manufacturing or first boot. Store the private key in EEPROM (marked read-protected if the MCU supports it). The public key is embedded in the validation program.

3. **IGC file signing** — After flight (or incrementally during flight), compute the SHA-512 hash of all protected records, then sign with Ed25519. Append the signature as G records.

4. **Validation program** — A PC-side tool (Python/Go) that reads the IGC file, extracts the protected records, reproduces the hash, and verifies the Ed25519 signature against the manufacturer's public key.

5. **Key protection** — The private key must never be readable from the device. No firmware update path should expose it. The public key in the validation program is, by design, public.

**Hardware security note:** ATmega328P has **no secure key storage**. Software-only protection is limited. The specification accepts this (3.1.1: "take all practicable measures").

### Option B: HMAC-SHA256 (symmetric, CIVL-permitted)

The spec explicitly allows this as an alternative. Simpler, smaller code, but requires keeping the secret key in both the device and the validation program.

**Implementation plan:**

1. Add a SHA-256 implementation (~2 KB flash).
2. Build HMAC-SHA256 on top.
3. Store a 256-bit secret key in EEPROM (shared across all units or per batch).
4. After flight, compute HMAC over all protected records and append as G records.
5. The validation program recomputes the HMAC with the same secret key and compares.

**Downside:** The secret key must be embedded in the validation program, which is distributed publicly. Anyone with the validation program can generate valid signatures — though the spec permits this (shared key between similar models). The key could be obfuscated in the validation binary to make extraction slightly harder.

### Option C: Hybrid — Ed25519 for signing, SHA-256 for hashing

Use SHA-256 (lighter than SHA-512) to hash the file content, then sign the 32-byte hash with Ed25519. This gives asymmetric security with minimal per-file computation cost.

## Feasibility Assessment

### HMAC-SHA256 — **✅ Feasible with comfortable margins**

| Metric | Current | +HMAC-SHA256 | Margin |
|--------|---------|-------------|--------|
| Flash | 23,840 B | ~26,800 B (~83%) | ~5,400 B free |
| RAM (static) | 1,232 B | ~1,450 B (~71%) | ~600 B free |

SHA-256 on AVR is well-known (avr-crypto-lib, ArduinoCryptolib). The core compression function is ~2-2.5 KB of flash with ~200 B RAM for state. HMAC wrapper adds minimal overhead. LTO will inline aggressively across the small codebase. The signature is 32 bytes (64 hex chars) — fits in one or two G records. Signing time at 8 MHz: ~50 ms per 1 KB of data.

### Ed25519 — **⚠️ Feasible but very tight**

| Metric | Current | +Ed25519 | Margin |
|--------|---------|----------|--------|
| Flash | 23,840 B | ~29,000 B (~90%) | ~3,200 B free |
| RAM (static) | 1,232 B | ~1,750 B (~85%) | ~300 B free |

Ed25519 needs SHA-512 (larger than SHA-256) plus modular arithmetic (256-bit field ops). On AVR:

- **libhydrogen** (~5-6 KB) — includes the full set: SHA-256, SHA-512, Curve25519, Ed25519, key exchange. Full-featured but heavy.
- **Minimal Ed25519** (~4 KB) — extracted sign-only path with hand-optimized AVR assembly for field arithmetic. Possible but significant effort.
- **Hybrid: SHA-256 hash + Ed25519 sign** (~4.5 KB) — sign a 32-byte SHA-256 digest instead of hashing the whole file with SHA-512. Saves ~1 KB over full SHA-512.

Signatures are 64 bytes (128 hex chars) — split across two G record lines. Signing at 8 MHz: ~2-3 seconds (dominated by the modular exponentiation).

**The 300-byte RAM margin is the real concern.** The stack shares RAM with `.bss`/`.data`. A deep call chain during signing could overflow. A custom stack usage analysis would be needed before committing.

### Comparison

| Criterion | HMAC-SHA256 | Ed25519 | ECDSA P-256 |
|-----------|-------------|---------|-------------|
| Asymmetric (IGC level) | ❌ | ✅ | ✅ |
| CIVL spec compliance | ✅ | ✅ | ✅ (Acquired Rights 3.5.2) |
| Flash used | ~3 KB | ~5 KB | ~10 KB |
| RAM used | ~220 B | ~520 B | ~1 KB |
| No per-signature RNG | N/A | ✅ | ❌ (catastrophic on failure) |
| **Feasible on ATmega328P at 8 MHz** | ✅ **Comfortable** | ⚠️ **Tight (RAM)** | ❌ **No** |

### Conclusion

- **HMAC-SHA256** is the most practical option. It has comfortable flash and RAM margins, simple implementation using existing avr-crypto-lib, and is explicitly permitted by the CIVL specification (section 3.1.4.3). The 32-byte G record is clean and efficient.
- **Ed25519** is technically feasible but leaves razor-thin RAM headroom (~300 bytes). It would require careful stack analysis and possibly disabling other features (e.g., Bluetooth or GPS) to free resources.
- **ECDSA P-256** is infeasible — it alone would consume ~10 KB flash and ~1 KB RAM, exceeding available resources on this MCU.
- If the key management concern of HMAC (shared secret in the validation program) is acceptable, it is the clear winner. If true asymmetric cryptography is needed per IGC requirements, Ed25519 is the only viable option, but the RAM budget demands caution.

## Implementation Outline

### Files to create/modify:

| File | Purpose |
|------|---------|
| `libraries/igcrypto/` | New library: SHA-256 + Ed25519 or HMAC-SHA256 |
| `libraries/GpsSentences/IGCSentence.h/.cpp` | Add G record generation |
| `variometer/variometer.ino` | Call G record signing after flight |
| Validation tool | PC-side `igc-validate.py` or similar |

### Example G record format (Ed25519, 64-byte signature as hex):

```
GED25519SIG<64 hex chars first half>
G<64 hex chars second half>
```

IGC files limit lines to ~80 characters, so splitting across two G records is typical.

### Key storage in EEPROM:

- Reserve a block in EEPROM (e.g., 32 bytes for Ed25519 private key or 32 bytes for HMAC key)
- Generate once at first boot (or at manufacturing time)
- Mark the EEPROM block with a validation tag (similar to how `IGCHeader` uses `IGC_SENTENCE_EEPROM_TAG`)
- The `SetVarioParameters` sketch could be extended to inject a device-unique key

---

*Analysis based on FAI/CIVL Sporting Code Section 7H (2024 Edition) and the HaskoVario codebase.*
