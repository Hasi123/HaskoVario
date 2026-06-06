/* hmac_sha256 -- HMAC-SHA256 for AVR
 *
 * Per RFC 2104.
 * Reference: avr-crypto-lib (Daniel Otte)
 *
 * This file is part of HaskoVario.
 *
 * HaskoVario is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "hmac_sha256.h"

HMACSha256::HMACSha256() {
}

void HMACSha256::begin(const uint8_t* key, uint8_t keyLen) {
  uint8_t pad[64];
  uint8_t hashedKey[32];

  if (keyLen > 64) {
    Sha256 hasher;
    hasher.begin();
    hasher.write(key, keyLen);
    hasher.finalize();
    const uint8_t* h = hasher.digest();
    for (uint8_t i = 0; i < 32; i++) {
      hashedKey[i] = h[i];
    }
    key = hashedKey;
    keyLen = 32;
  }

  for (uint8_t i = 0; i < 64; i++) {
    if (i < keyLen) {
      pad[i] = key[i] ^ 0x36;
    } else {
      pad[i] = 0x36;
    }
  }
  inner.begin();
  inner.write(pad, 64);

  for (uint8_t i = 0; i < 64; i++) {
    if (i < keyLen) {
      pad[i] = key[i] ^ 0x5c;
    } else {
      pad[i] = 0x5c;
    }
  }
  outer.begin();
  outer.write(pad, 64);
}

void HMACSha256::write(uint8_t byte) {
  inner.write(byte);
}

void HMACSha256::write(const uint8_t* data, uint16_t len) {
  inner.write(data, len);
}

void HMACSha256::finalize() {
  inner.finalize();
  const uint8_t* innerDigest = inner.digest();
  outer.write(innerDigest, 32);
  outer.finalize();
}

const uint8_t* HMACSha256::digest() {
  return outer.digest();
}
