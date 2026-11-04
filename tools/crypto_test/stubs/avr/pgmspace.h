/* PC stand-in for avr/pgmspace.h : flash is ordinary memory */
#ifndef PGMSPACE_STUB_H
#define PGMSPACE_STUB_H

#include <stdint.h>
#include <string.h>

#define PROGMEM
#define pgm_read_byte(p) (*(const uint8_t*)(p))
#define pgm_read_byte_near(p) (*(const uint8_t*)(p))
#define pgm_read_dword(p) (*(const uint32_t*)(p))
#define memcpy_P memcpy

#endif
