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

#ifndef HMAC_SHA256_H
#define HMAC_SHA256_H

#include <Arduino.h>
#include "sha256.h"

class HMACSha256 {
public:
  HMACSha256();
  void begin(const uint8_t* key, uint8_t keyLen);
  void write(uint8_t byte);
  void write(const uint8_t* data, uint16_t len);
  void finalize();
  const uint8_t* digest();
  uint16_t getDigestLength() { return 32; }

private:
  Sha256 inner;
  Sha256 outer;
};

#endif
