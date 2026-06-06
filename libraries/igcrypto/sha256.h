/* sha256 -- Minimal SHA-256 for AVR
 *
 * Based on FIPS 180-4.
 * Reference: avr-crypto-lib (Daniel Otte), ArduinoCryptolib (Adrianotiger)
 *
 * This file is part of HaskoVario.
 *
 * HaskoVario is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef SHA256_H
#define SHA256_H

#include <Arduino.h>
#include <avr/pgmspace.h>

class Sha256 {
public:
  Sha256();
  void begin();
  void write(uint8_t byte);
  void write(const uint8_t* data, uint16_t len);
  void finalize();
  const uint8_t* digest();
  uint16_t getDigestLength() { return 32; }

private:
  uint32_t state[8];
  uint64_t count;
  uint8_t buffer[64];
  uint8_t bufferIndex;
  uint8_t digestBuffer[32];

  void transform(const uint8_t block[64]);
};

#endif
