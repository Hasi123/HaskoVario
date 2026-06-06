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

#include "sha256.h"

static const uint32_t K[64] PROGMEM = {
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
  0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
  0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
  0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
  0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
  0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
  0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
  0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
  0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

#define ROTR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))
#define CH(x, y, z) (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define SIG0(x) (ROTR((x), 2) ^ ROTR((x), 13) ^ ROTR((x), 22))
#define SIG1(x) (ROTR((x), 6) ^ ROTR((x), 11) ^ ROTR((x), 25))
#define SIG2(x) (ROTR((x), 7) ^ ROTR((x), 18) ^ ((x) >> 3))
#define SIG3(x) (ROTR((x), 17) ^ ROTR((x), 19) ^ ((x) >> 10))

Sha256::Sha256() {
  begin();
}

void Sha256::begin() {
  state[0] = 0x6a09e667;
  state[1] = 0xbb67ae85;
  state[2] = 0x3c6ef372;
  state[3] = 0xa54ff53a;
  state[4] = 0x510e527f;
  state[5] = 0x9b05688c;
  state[6] = 0x1f83d9ab;
  state[7] = 0x5be0cd19;
  count = 0;
  bufferIndex = 0;
}

void Sha256::write(uint8_t byte) {
  buffer[bufferIndex++] = byte;
  count += 8;
  if (bufferIndex == 64) {
    transform(buffer);
    bufferIndex = 0;
  }
}

void Sha256::write(const uint8_t* data, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    write(data[i]);
  }
}

void Sha256::finalize() {
  uint64_t bits = count;
  write(0x80);
  while (bufferIndex != 56) {
    write(0x00);
  }
  for (uint8_t i = 0; i < 8; i++) {
    write((uint8_t)(bits >> (56 - i * 8)));
  }
}

void Sha256::transform(const uint8_t block[64]) {
  uint32_t W[16];
  uint32_t a, b, c, d, e, f, g, h;
  uint32_t T1, T2;

  for (uint8_t t = 0; t < 16; t++) {
    W[t] = ((uint32_t)block[t * 4]) << 24;
    W[t] |= ((uint32_t)block[t * 4 + 1]) << 16;
    W[t] |= ((uint32_t)block[t * 4 + 2]) << 8;
    W[t] |= ((uint32_t)block[t * 4 + 3]);
  }

  a = state[0]; b = state[1]; c = state[2]; d = state[3];
  e = state[4]; f = state[5]; g = state[6]; h = state[7];

  for (uint8_t t = 0; t < 64; t++) {
    if (t >= 16) {
      W[t & 0xF] = SIG3(W[(t - 2) & 0xF]) + W[(t - 7) & 0xF]
                 + SIG2(W[(t - 15) & 0xF]) + W[(t - 16) & 0xF];
    }
    T1 = h + SIG1(e) + CH(e, f, g) + pgm_read_dword(&K[t]) + W[t & 0xF];
    T2 = SIG0(a) + MAJ(a, b, c);
    h = g;
    g = f;
    f = e;
    e = d + T1;
    d = c;
    c = b;
    b = a;
    a = T1 + T2;
  }

  state[0] += a; state[1] += b; state[2] += c; state[3] += d;
  state[4] += e; state[5] += f; state[6] += g; state[7] += h;
}

const uint8_t* Sha256::digest() {
  for (uint8_t i = 0; i < 8; i++) {
    digestBuffer[i * 4]     = (uint8_t)(state[i] >> 24);
    digestBuffer[i * 4 + 1] = (uint8_t)(state[i] >> 16);
    digestBuffer[i * 4 + 2] = (uint8_t)(state[i] >> 8);
    digestBuffer[i * 4 + 3] = (uint8_t)(state[i]);
  }
  return digestBuffer;
}
