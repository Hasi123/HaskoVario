/* PC test of libraries/igcrypto (split SHA-256 transform)
 *
 * Built and checked by run_test.py, which compares every line against
 * Python's hashlib/hmac. Output lines :
 *   SHA  <msg hex> <digest hex>
 *   HMAC <key hex> <msg hex> <digest hex>
 *   SHA_A1M <digest hex>            (one million 'a')
 * and writes test.IGC, built like the firmware does, for igc_verify.py.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <vector>

#include "sha256.h"
#include "hmac_sha256.h"
#include "IGCSentence.h"

typedef std::vector<uint8_t> Bytes;

/* deterministic PRNG */
static uint32_t rngState = 0x12345678;
static uint32_t rnd(void) {
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return rngState;
}

static void printHex(const uint8_t* d, size_t n) {
  for (size_t i = 0; i < n; i++) printf("%02x", d[i]);
}

static Bytes randomBytes(size_t n) {
  Bytes b(n);
  for (size_t i = 0; i < n; i++) b[i] = (uint8_t)rnd();
  return b;
}

/* how step() is called between writes */
enum StepMode { STEP_NONE, STEP_EACH_BYTE, STEP_RANDOM, STEP_DRAIN };

static void doSteps(StepMode mode) {
  switch (mode) {
    case STEP_NONE: break;
    case STEP_EACH_BYTE: Sha256::step(); break;
    case STEP_RANDOM: for (uint32_t i = rnd() % 6; i; i--) Sha256::step(); break;
    case STEP_DRAIN: while (Sha256::step()) { } break;
  }
}

/* write in random sized chunks, stepping in between */
static void feedSha(Sha256& h, const Bytes& m, StepMode mode) {
  size_t i = 0;
  while (i < m.size()) {
    size_t chunk = 1 + rnd() % 80;
    if (chunk > m.size() - i) chunk = m.size() - i;
    if (mode == STEP_EACH_BYTE) {
      for (size_t k = 0; k < chunk; k++) { h.write(m[i + k]); doSteps(mode); }
    } else {
      h.write(&m[i], (uint16_t)chunk);
      doSteps(mode);
    }
    i += chunk;
  }
}

static void feedHmac(HMACSha256& h, const Bytes& m, StepMode mode) {
  size_t i = 0;
  while (i < m.size()) {
    size_t chunk = 1 + rnd() % 80;
    if (chunk > m.size() - i) chunk = m.size() - i;
    for (size_t k = 0; k < chunk; k++) {
      h.write(m[i + k]);
      if (mode == STEP_EACH_BYTE) doSteps(mode);
    }
    if (mode != STEP_EACH_BYTE) doSteps(mode);
    i += chunk;
  }
}

static void shaCase(const Bytes& m, StepMode mode) {
  Sha256 h;
  feedSha(h, m, mode);
  h.finalize();
  printf("SHA ");
  printHex(m.data(), m.size());
  printf(" ");
  printHex(h.digest(), 32);
  printf("\n");
}

static void hmacCase(const Bytes& key, const Bytes& m, StepMode mode) {
  HMACSha256 h;
  h.begin(key.data(), (uint8_t)key.size());
  doSteps(mode);
  feedHmac(h, m, mode);
  h.finalize();
  printf("HMAC ");
  printHex(key.data(), key.size());
  printf(" ");
  printHex(m.data(), m.size());
  printf(" ");
  printHex(h.digest(), 32);
  printf("\n");
}

/* two hashes alternate : pending transform must change owner correctly */
static void interleavedCase(void) {
  Bytes m1 = randomBytes(1000 + rnd() % 500);
  Bytes m2 = randomBytes(1000 + rnd() % 500);
  Sha256 h1, h2;
  size_t i1 = 0, i2 = 0;
  while (i1 < m1.size() || i2 < m2.size()) {
    for (uint32_t n = rnd() % 70; n && i1 < m1.size(); n--) h1.write(m1[i1++]);
    doSteps(STEP_RANDOM);
    for (uint32_t n = rnd() % 70; n && i2 < m2.size(); n--) h2.write(m2[i2++]);
    doSteps(STEP_RANDOM);
  }
  h1.finalize();
  h2.finalize();
  printf("SHA "); printHex(m1.data(), m1.size()); printf(" "); printHex(h1.digest(), 32); printf("\n");
  printf("SHA "); printHex(m2.data(), m2.size()); printf(" "); printHex(h2.digest(), 32); printf("\n");
}

/* begin() while our transform is pending must drop it */
static void restartCase(void) {
  Sha256 h;
  Bytes junk = randomBytes(100);  //leaves a pending transform
  h.write(junk.data(), (uint16_t)junk.size());
  h.begin();
  Bytes m = randomBytes(200);
  feedSha(h, m, STEP_RANDOM);
  h.finalize();
  printf("SHA "); printHex(m.data(), m.size()); printf(" "); printHex(h.digest(), 32); printf("\n");
}

/* IGC file like createSDCardTrackFile() + loop() + finalizeIGCFile() */
static void igcFileCase(void) {
  static const uint8_t key[32] = VARIOMETER_HMAC_KEY;
  HMACSha256 hmac;
  IGCHeader header;
  Bytes file;

  hmac.begin(key, 32);

  /* header, date written over the placeholder, then flight number */
  int16_t datePos = header.begin();
  while (datePos) { uint8_t b = header.get(); file.push_back(b); hmac.write(b); datePos--; }
  const char* date = "060626";
  for (int i = 0; i < 6; i++) { file.push_back(date[i]); hmac.write(date[i]); header.get(); }
  const char* flight = ",01\r\n";
  for (int i = 0; i < 5; i++) { file.push_back(flight[i]); hmac.write(flight[i]); }
  while (header.available()) { uint8_t b = header.get(); file.push_back(b); hmac.write(b); }

  /* B records, ~1 per second, loop() idle passes between them */
  for (int s = 0; s < 3600; s++) {
    char rec[64];
    int n = snprintf(rec, sizeof(rec), "B%02d%02d%02d4712%03uN01123%03uEA%05u%05u\r\n",
                     (s / 3600 + 10) % 24, (s / 60) % 60, s % 60,
                     (unsigned)(rnd() % 1000), (unsigned)(rnd() % 1000),
                     (unsigned)(1000 + rnd() % 2000), (unsigned)(1000 + rnd() % 2000));
    for (int i = 0; i < n; i++) { file.push_back(rec[i]); hmac.write(rec[i]); }
    for (uint32_t k = rnd() % 25; k; k--) HMACSha256::step();
  }

  /* G record : 2 lines of 32 hex chars */
  hmac.finalize();
  const uint8_t* d = hmac.digest();
  for (int line = 0; line < 2; line++) {
    file.push_back('G');
    for (int i = 0; i < 16; i++) {
      char hx[3];
      snprintf(hx, sizeof(hx), "%02X", d[line * 16 + i]);
      file.push_back(hx[0]);
      file.push_back(hx[1]);
    }
    file.push_back('\r');
    file.push_back('\n');
  }

  FILE* f = fopen("test.IGC", "wb");
  fwrite(file.data(), 1, file.size(), f);
  fclose(f);
}

int main(void) {
  static const StepMode modes[] = { STEP_NONE, STEP_EACH_BYTE, STEP_RANDOM, STEP_DRAIN };

  /* FIPS 180-4 examples */
  const char* abc = "abc";
  const char* two = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
  for (StepMode mode : modes) {
    shaCase(Bytes(), mode);
    shaCase(Bytes(abc, abc + strlen(abc)), mode);
    shaCase(Bytes(two, two + strlen(two)), mode);
  }

  /* every length around the block and padding boundaries */
  for (size_t len = 0; len <= 200; len++) {
    for (StepMode mode : modes) shaCase(randomBytes(len), mode);
  }

  /* long random messages */
  for (int i = 0; i < 40; i++) shaCase(randomBytes(rnd() % 5000), modes[i % 4]);

  /* one million 'a', stepping */
  {
    Sha256 h;
    for (int i = 0; i < 1000000; i++) { h.write('a'); if (!(i % 37)) Sha256::step(); }
    h.finalize();
    printf("SHA_A1M "); printHex(h.digest(), 32); printf("\n");
  }

  /* HMAC : real key, random keys up to one block (longer keys unsupported) */
  static const uint8_t igcKey[32] = VARIOMETER_HMAC_KEY;
  for (int i = 0; i < 60; i++) {
    Bytes key = (i % 3 == 0) ? Bytes(igcKey, igcKey + 32) : randomBytes((i % 3 == 1) ? 1 + rnd() % 64 : 64);
    hmacCase(key, randomBytes(rnd() % 3000), modes[i % 4]);
  }

  for (int i = 0; i < 20; i++) interleavedCase();
  for (int i = 0; i < 10; i++) restartCase();

  igcFileCase();
  return 0;
}
