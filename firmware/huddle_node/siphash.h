/* SipHash-2-4 (Aumasson and Bernstein, 2012), 128-bit key, 64-bit output. Portable C99. */
#ifndef HUDDLE_SIPHASH_H
#define HUDDLE_SIPHASH_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIPHASH_KEY_LEN 16

uint64_t siphash24(const uint8_t key[SIPHASH_KEY_LEN], const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
