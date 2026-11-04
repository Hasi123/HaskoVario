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
#include <VarioSettings.h>

/* rounds of the pending transform computed by each step() call */
#ifndef IGC_SHA256_ROUNDS_PER_STEP
#define IGC_SHA256_ROUNDS_PER_STEP 4
#endif

/*****************************************************************/
/* Split transform :                                             */
/* When a 64 byte block is full, the message is copied to W and  */
/* the 64 rounds are left pending. step() computes some of them, */
/* call it from loop() when there is spare time. The input       */
/* buffer is free again immediately.                             */
/* Only one transform can be pending (shared work area) : any    */
/* new transform, finalize() or digest() first completes it, so  */
/* the result never depends on how often step() is called.       */
/*****************************************************************/
class Sha256 {
public:
  Sha256();
  void begin();
  void write(uint8_t byte);
  void write(const uint8_t* data, uint16_t len);
  void finalize();
  const uint8_t* digest();
  uint16_t getDigestLength() { return 32; }

  /* compute some rounds, return true while work remains */
  static bool step(void);

private:
  uint32_t state[8];
  uint64_t count;
  uint8_t buffer[64];
  uint8_t bufferIndex;
  uint8_t digestBuffer[32];

  void startTransform(void);
  void runRounds(uint8_t rounds);
  static void finishPending(void);

  /* work area of the pending transform */
  static Sha256* pending;
  static uint32_t W[16];
  static uint32_t work[8];  //a..h
  static uint8_t round;
};

#endif
