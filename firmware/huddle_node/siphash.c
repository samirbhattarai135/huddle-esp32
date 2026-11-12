#include "siphash.h"

static uint64_t rotl(uint64_t x, int b) { return (x << b) | (x >> (64 - b)); }

static uint64_t load64(const uint8_t *p, size_t n) {
    uint64_t v = 0;
    for (size_t i = 0; i < n; i++) v |= (uint64_t)p[i] << (8 * i);
    return v;
}

static void sipround(uint64_t v[4]) {
    v[0] += v[1]; v[1] = rotl(v[1], 13); v[1] ^= v[0]; v[0] = rotl(v[0], 32);
    v[2] += v[3]; v[3] = rotl(v[3], 16); v[3] ^= v[2];
    v[0] += v[3]; v[3] = rotl(v[3], 21); v[3] ^= v[0];
    v[2] += v[1]; v[1] = rotl(v[1], 17); v[1] ^= v[2]; v[2] = rotl(v[2], 32);
}

uint64_t siphash24(const uint8_t key[SIPHASH_KEY_LEN], const uint8_t *data, size_t len) {
    uint64_t k0 = load64(key, 8), k1 = load64(key + 8, 8);
    uint64_t v[4] = {k0 ^ 0x736f6d6570736575ULL, k1 ^ 0x646f72616e646f6dULL,
                     k0 ^ 0x6c7967656e657261ULL, k1 ^ 0x7465646279746573ULL};
    size_t whole = len & ~(size_t)7;
    for (size_t i = 0; i < whole; i += 8) {
        uint64_t m = load64(data + i, 8);
        v[3] ^= m;
        sipround(v);
        sipround(v);
        v[0] ^= m;
    }
    uint64_t last = load64(data + whole, len - whole) | (uint64_t)len << 56;
    v[3] ^= last;
    sipround(v);
    sipround(v);
    v[0] ^= last;
    v[2] ^= 0xff;
    for (int i = 0; i < 4; i++) sipround(v);
    return v[0] ^ v[1] ^ v[2] ^ v[3];
}
