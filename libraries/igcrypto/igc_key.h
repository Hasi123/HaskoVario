/* igc_key -- HMAC-SHA256 key for IGC G record
 *
 * The key is defined in VarioSettings.h and stored in PROGMEM.
 * Shared across all units of this model per CIVL spec 3.1.4.3.
 *
 * This file is part of HaskoVario.
 *
 * HaskoVario is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef IGC_KEY_H
#define IGC_KEY_H

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <VarioSettings.h>

#ifdef HAVE_IGC_SECURITY
static const uint8_t igcHmacKey[32] PROGMEM = VARIOMETER_HMAC_KEY;
#endif

#endif
