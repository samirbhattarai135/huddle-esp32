/*
 * Huddle: per-robot decision logic from Chin et al., "Huddle: Parallel Shape
 * Assembly using Decentralized, Minimalistic Robots" (DARS 2026), Algorithms 1-6.
 *
 * Portable C99. No heap use, no global mutable state. Every function is a pure
 * function of its arguments, so the same file builds for the host simulator and
 * for the ESP32 firmware.
 */
#ifndef HUDDLE_H
#define HUDDLE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HUDDLE_WALLS 6

/* Wall indices, counter-clockwise, as in Fig. 1c of the paper. */
enum {
    HUDDLE_F = 0,  /* front,      u = ( 0,  1) */
    HUDDLE_FL = 1, /* front-left, u = (-1,  1) */
    HUDDLE_RL = 2, /* rear-left,  u = (-1,  0) */
    HUDDLE_R = 3,  /* rear,       u = ( 0, -1) */
    HUDDLE_RR = 4, /* rear-right, u = ( 1, -1) */
    HUDDLE_FR = 5  /* front-right,u = ( 1,  0) */
};

typedef enum {
    HUDDLE_NULL = 0,     /* wall not intended for attachment */
    HUDDLE_FREE = 1,     /* connection wall, nobody attached yet */
    HUDDLE_OCCUPIED = 2  /* connection wall with an attached neighbor */
} huddle_status;

typedef struct {
    int16_t p; /* column */
    int16_t q; /* row */
} huddle_cell;

extern const huddle_cell HUDDLE_UNIT[HUDDLE_WALLS];

/*
 * Target shape S as a bitmap over a bounding box, in the root robot's frame
 * (the root sits at (0,0)). Bit index = (q - q_min) * width + (p - p_min).
 * The caller owns the storage; the library only reads it.
 */
typedef struct {
    int16_t p_min;
    int16_t q_min;
    uint16_t width;
    uint16_t height;
    const uint8_t *bits;
} huddle_shape;

bool huddle_shape_contains(const huddle_shape *shape, int p, int q);

typedef struct {
    huddle_cell position;
    uint8_t status[HUDDLE_WALLS]; /* huddle_status per wall */
    bool nucleus_left;            /* alpha_left from Algorithm 1 */
    bool nucleus_right;           /* alpha_right from Algorithm 1 */
    int8_t growth;                /* g_i from Algorithm 4: -1, 0, +1 */
} huddle_robot;

/*
 * Initialization phase. occupied_mask has bit w set when wall w senses an
 * attached neighbor at the instant of docking.
 */
void huddle_init_root(huddle_robot *robot, const huddle_shape *shape,
                      uint8_t occupied_mask);

/*
 * Initialization from the 32-bit pose word sent by the neighbor that admitted
 * this robot. Returns false if the word is malformed.
 */
bool huddle_init_from_neighbor(huddle_robot *robot, const huddle_shape *shape,
                               uint32_t pose_word, uint8_t occupied_mask);

/* Called when a wall senses a newly attached neighbor. */
void huddle_mark_occupied(huddle_robot *robot, int wall);

/* Algorithm 5: nominal walls W_i as a bitmask (bit w = wall w). */
uint8_t huddle_nominal_walls(const huddle_robot *robot);

/*
 * Algorithm 6: walls whose signal is delayed, W~_i, as a bitmask.
 * neighbor_status[w] is the 16-bit status word last received from the
 * neighbor on wall w (ignored for walls that are not occupied).
 */
uint8_t huddle_delayed_walls(const huddle_robot *robot,
                             const uint16_t neighbor_status[HUDDLE_WALLS]);

/* Walls that signal this step: W_i \ W~_i. */
uint8_t huddle_signal_walls(const huddle_robot *robot,
                            const uint16_t neighbor_status[HUDDLE_WALLS]);

/* ---- Wire format: the paper's two integers ---------------------------- */

/* 16-bit wall-status word: 2 bits per wall, wall w at bits [2w+1:2w]. */
uint16_t huddle_status_word(const huddle_robot *robot);
huddle_status huddle_status_in_word(uint16_t word, int wall);

/*
 * 32-bit pose word, two's complement fields:
 *   bits  0-11  p   (int12)
 *   bits 12-23  q   (int12)
 *   bits 24-25  du  (int2)   unit vector of the sender's wall that faces
 *   bits 26-27  dv  (int2)   the receiver
 *   bits 28-31  zero
 * Returns false if the position does not fit in int12.
 */
bool huddle_pose_word(huddle_cell position, int wall, uint32_t *word);

/*
 * Decodes a pose word: sender position, and the index of the sender's wall.
 * Returns false if the unit vector is not one of the six walls.
 */
bool huddle_decode_pose(uint32_t word, huddle_cell *sender, int *sender_wall);

#ifdef __cplusplus
}
#endif

#endif
