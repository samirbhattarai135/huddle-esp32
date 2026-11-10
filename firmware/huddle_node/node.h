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

#ifdef __cplusplus
extern "C" {
#endif

#define NODE_MAC_LEN 6
#define NODE_PAYLOAD_LEN 6 /* uint16 status word + uint32 pose word, little-endian */
#define NODE_WINDOW 32     /* the shape must fit in p, q in [-16, 15] */

typedef struct {
    void (*send)(void *ctx, const uint8_t mac[NODE_MAC_LEN], const uint8_t payload[NODE_PAYLOAD_LEN]);
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
    uint16_t view[HUDDLE_WALLS];            /* newest status word from each neighbor */
    uint8_t signals;
} node;

/* Clears everything except self_mac. */
void node_reset(node *n);

/* One line from the arena, e.g. "CELL 1 -2", "ROOT", "ATTACH 5 aa:bb:cc:dd:ee:ff". */
void node_command(node *n, const char *line, const node_io *io);

/* One ESP-NOW frame from a neighbor. */
void node_receive(node *n, const uint8_t mac[NODE_MAC_LEN], const uint8_t *data, int len,
                  const node_io *io);

/* One Signal Activation step: send both integers to every attached neighbor,
 * then recompute the signaled walls. Returns the signal mask. */
uint8_t node_tick(node *n, const node_io *io);

#ifdef __cplusplus
}
#endif

#endif
