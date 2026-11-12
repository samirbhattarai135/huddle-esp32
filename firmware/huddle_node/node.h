/*
 * One in-assembly robot: the arena command protocol, the neighbor link and the
 * Huddle state. Transport-free so the same code runs on the ESP32 (ESP-NOW)
 * and on the host (tools/fake_node.c, driven by tools/arena.py --fake).
 */
#ifndef HUDDLE_NODE_H
#define HUDDLE_NODE_H

#include <stdbool.h>
#include <stdint.h>

#include "huddle.h"
#include "siphash.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NODE_MAC_LEN 6
#define NODE_PAYLOAD_LEN 6 /* uint16 status word + uint32 pose word, little-endian */
#define NODE_SEQ_LEN 4     /* uint32 per-sender frame counter, little-endian */
#define NODE_TAG_LEN 8     /* SipHash-2-4 output, little-endian */
#define NODE_FRAME_LEN (NODE_PAYLOAD_LEN + NODE_SEQ_LEN + NODE_TAG_LEN) /* 18 */
#define NODE_WINDOW 32     /* the shape must fit in p, q in [-16, 15] */

typedef struct {
    void (*send)(void *ctx, const uint8_t mac[NODE_MAC_LEN], const uint8_t frame[NODE_FRAME_LEN]);
    void (*log)(void *ctx, const char *line);
    void *ctx;
} node_io;

typedef struct {
    uint8_t self_mac[NODE_MAC_LEN];
    uint8_t shape_bits[NODE_WINDOW * NODE_WINDOW / 8];
    huddle_shape shape;
    int shape_cells;
    bool initialized;
    huddle_robot robot;
    uint8_t attached;                       /* bit w: a neighbor is attached on wall w */
    uint8_t peer[HUDDLE_WALLS][NODE_MAC_LEN];
    uint16_t view[HUDDLE_WALLS];            /* monotone merge of the status words heard on each wall */
    uint8_t heard;                          /* bit w: view[w] holds an authenticated word */
    uint8_t key[SIPHASH_KEY_LEN];           /* session key, RAM only; set by "KEY <hex>" */
    bool has_key;
    uint32_t tx_seq;                        /* last sequence number sent */
    uint32_t rx_seq[HUDDLE_WALLS];          /* last sequence number accepted per wall */
    uint32_t rejected;                      /* frames dropped (bad length, tag, binding or sequence) */
    uint8_t signals;
} node;

/* Clears everything except self_mac. */
void node_reset(node *n);

/* One line from the arena, e.g. "KEY <32 hex>", "CELL 1 -2", "ROOT",
 * "ATTACH 5 aa:bb:cc:dd:ee:ff". */
void node_command(node *n, const char *line, const node_io *io);

/*
 * One ESP-NOW frame from a neighbor. Dropped unless it is NODE_FRAME_LEN bytes
 * from the MAC bound to the wall it claims, carries a valid tag under the session
 * key, and a sequence number above the last one accepted on that wall.
 */
void node_receive(node *n, const uint8_t mac[NODE_MAC_LEN], const uint8_t *data, int len,
                  const node_io *io);

/* One Signal Activation step: send an authenticated frame to every attached
 * neighbor (nothing is sent before a key is set), then recompute the signaled walls. Returns the signal mask. */
uint8_t node_tick(node *n, const node_io *io);

#ifdef __cplusplus
}
#endif

#endif
