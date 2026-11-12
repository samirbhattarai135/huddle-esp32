#include "huddle.h"

const huddle_cell HUDDLE_UNIT[HUDDLE_WALLS] = {
    {0, 1}, {-1, 1}, {-1, 0}, {0, -1}, {1, -1}, {1, 0}};

typedef enum { FLANK_FALSE, FLANK_TRUE, FLANK_UNDEFINED } flank_flag;

typedef struct {
    int lo;
    int hi;
} segment;

static const int LEFT_FLANK[2] = {HUDDLE_FL, HUDDLE_RL};
static const int RIGHT_FLANK[2] = {HUDDLE_RR, HUDDLE_FR};

static int prev_wall(int wall) { return (wall + HUDDLE_WALLS - 1) % HUDDLE_WALLS; }
static int next_wall(int wall) { return (wall + 1) % HUDDLE_WALLS; }
static bool is_fore_aft(int wall) { return wall == HUDDLE_F || wall == HUDDLE_R; }

/* C99 '/' truncates toward zero; the paper's midpoint is rounded down. */
static int floor_half(int value) { return value >= 0 ? value / 2 : -((-value + 1) / 2); }

bool huddle_shape_contains(const huddle_shape *shape, int p, int q) {
    int col = p - shape->p_min;
    int row = q - shape->q_min;
    if (col < 0 || row < 0 || col >= shape->width || row >= shape->height) {
        return false;
    }
    unsigned bit = (unsigned)row * shape->width + (unsigned)col;
    return (shape->bits[bit >> 3] >> (bit & 7)) & 1u;
}

/* Contiguous run of target cells in column p that contains row q. */
static segment segment_at(const huddle_shape *shape, int p, int q) {
    segment run = {q, q};
    while (huddle_shape_contains(shape, p, run.lo - 1)) run.lo--;
    while (huddle_shape_contains(shape, p, run.hi + 1)) run.hi++;
    return run;
}

/* ---- Algorithm 2: role from self-wall status ---------------------------- */

static flank_flag role_from_walls(const huddle_robot *robot, const int flank[2]) {
    bool has_free = false;
    for (int i = 0; i < 2; i++) {
        int wall = flank[i];
        if (robot->status[wall] != HUDDLE_FREE) continue;
        has_free = true;
        /* "Between 2 null walls": both hexagon-adjacent walls are null. */
        if (robot->status[prev_wall(wall)] == HUDDLE_NULL &&
            robot->status[next_wall(wall)] == HUDDLE_NULL) {
            return FLANK_TRUE;
        }
    }
    return has_free ? FLANK_UNDEFINED : FLANK_FALSE;
}

/* ---- Algorithm 3: role from the adjacent target column ------------------ */

/* Experiment switches; the defaults follow the paper. Only the simulator's
 * ablation builds set these. */
#ifndef HUDDLE_MIDPOINT_ROUND_UP
#define HUDDLE_MIDPOINT_ROUND_UP 0 /* Fig. 2 caption: midpoint is rounded down */
#endif
#ifndef HUDDLE_SPECIAL_FLANK_SIGNAL
#define HUDDLE_SPECIAL_FLANK_SIGNAL 1 /* Remark 1: lines 11-13 of Algorithm 5 */
#endif

static bool is_nucleus_for(int q_self, segment self, segment adjacent) {
    int q_mid = floor_half(adjacent.lo + adjacent.hi + HUDDLE_MIDPOINT_ROUND_UP);
    if (q_self == q_mid) return true;
    if (q_mid >= self.lo && q_mid <= self.hi) return false; /* another robot's row */
    /* m* = row of the adjacent segment nearest to m whose same-row cell is in
     * our segment. The overlap is an interval and m lies outside it, so the
     * nearest row is unique. */
    int overlap_lo = adjacent.lo > self.lo ? adjacent.lo : self.lo;
    int overlap_hi = adjacent.hi < self.hi ? adjacent.hi : self.hi;
    if (overlap_lo > overlap_hi) return false; /* diagonal-only contact: Algorithm 2's job */
    int q_star = q_mid < overlap_lo ? overlap_lo : overlap_hi;
    return q_self == q_star;
}

static bool role_from_column(const huddle_robot *robot, const huddle_shape *shape,
                             int direction) {
    int p = robot->position.p;
    int q = robot->position.q;
    int p_adj = p + direction;
    segment self = segment_at(shape, p, q);
    /* Rows of column p+a that touch our segment: left neighbors of (p,q) are
     * rows q and q+1, right neighbors are rows q-1 and q. */
    int scan_lo = direction < 0 ? self.lo : self.lo - 1;
    int scan_hi = direction < 0 ? self.hi + 1 : self.hi;
    for (int row = scan_lo; row <= scan_hi; row++) {
        if (!huddle_shape_contains(shape, p_adj, row)) continue;
        segment adjacent = segment_at(shape, p_adj, row);
        if (is_nucleus_for(q, self, adjacent)) return true;
        row = adjacent.hi; /* skip the rest of this segment */
    }
    return false;
}

/* ---- Algorithm 1 + 4: role and growth direction ------------------------- */

static void determine_role(huddle_robot *robot, const huddle_shape *shape) {
    flank_flag left = role_from_walls(robot, LEFT_FLANK);
    flank_flag right = role_from_walls(robot, RIGHT_FLANK);
    robot->nucleus_left = left == FLANK_UNDEFINED ? role_from_column(robot, shape, -1)
                                                  : left == FLANK_TRUE;
    robot->nucleus_right = right == FLANK_UNDEFINED ? role_from_column(robot, shape, +1)
                                                    : right == FLANK_TRUE;
}

static int8_t determine_growth(const huddle_robot *robot) {
    const uint8_t *s = robot->status;
    if (robot->nucleus_left && robot->nucleus_right) return 0;
    if (s[HUDDLE_FL] == HUDDLE_OCCUPIED || s[HUDDLE_RL] == HUDDLE_OCCUPIED ||
        robot->nucleus_right) {
        return 1;
    }
    if (s[HUDDLE_RR] == HUDDLE_OCCUPIED || s[HUDDLE_FR] == HUDDLE_OCCUPIED ||
        robot->nucleus_left) {
        return -1;
    }
    return 0;
}

static void initialize(huddle_robot *robot, const huddle_shape *shape, huddle_cell position,
                       uint8_t occupied_mask) {
    robot->position = position;
    for (int w = 0; w < HUDDLE_WALLS; w++) {
        bool is_target = huddle_shape_contains(shape, position.p + HUDDLE_UNIT[w].p,
                                               position.q + HUDDLE_UNIT[w].q);
        if (occupied_mask & (1u << w)) {
            robot->status[w] = HUDDLE_OCCUPIED;
        } else {
            robot->status[w] = is_target ? HUDDLE_FREE : HUDDLE_NULL;
        }
    }
    determine_role(robot, shape);
    robot->growth = determine_growth(robot);
}

void huddle_init_root(huddle_robot *robot, const huddle_shape *shape, uint8_t occupied_mask) {
    huddle_cell origin = {0, 0};
    initialize(robot, shape, origin, occupied_mask);
}

bool huddle_init_from_neighbor(huddle_robot *robot, const huddle_shape *shape,
                               uint32_t pose_word, uint8_t occupied_mask) {
    huddle_cell sender;
    int sender_wall;
    if (!huddle_decode_pose(pose_word, &sender, &sender_wall)) return false;
    huddle_cell position = {(int16_t)(sender.p + HUDDLE_UNIT[sender_wall].p),
                            (int16_t)(sender.q + HUDDLE_UNIT[sender_wall].q)};
    initialize(robot, shape, position, occupied_mask | (uint8_t)(1u << ((sender_wall + 3) % 6)));
    return true;
}

void huddle_mark_occupied(huddle_robot *robot, int wall) {
    robot->status[wall] = HUDDLE_OCCUPIED;
}

/* ---- Algorithm 5: nominal signal walls ---------------------------------- */

static uint8_t free_walls(const huddle_robot *robot, const int flank[2]) {
    uint8_t mask = 0;
    for (int i = 0; i < 2; i++) {
        if (robot->status[flank[i]] == HUDDLE_FREE) mask |= (uint8_t)(1u << flank[i]);
    }
    return mask;
}

static bool is_null_fore_aft(const huddle_robot *robot, int wall) {
    return is_fore_aft(wall) && robot->status[wall] == HUDDLE_NULL;
}

/* Remark 1 special condition: w sits between 2 occupied walls, or between a
 * null fore-aft wall and an occupied wall. */
static bool meets_special_flank_condition(const huddle_robot *robot, int wall) {
    int a = prev_wall(wall);
    int b = next_wall(wall);
    bool a_occupied = robot->status[a] == HUDDLE_OCCUPIED;
    bool b_occupied = robot->status[b] == HUDDLE_OCCUPIED;
    return (a_occupied && b_occupied) || (is_null_fore_aft(robot, a) && b_occupied) ||
           (is_null_fore_aft(robot, b) && a_occupied);
}

uint8_t huddle_nominal_walls(const huddle_robot *robot) {
    const uint8_t *s = robot->status;
    if (s[HUDDLE_F] == HUDDLE_FREE || s[HUDDLE_R] == HUDDLE_FREE) {
        uint8_t walls = 0;
        if (s[HUDDLE_F] == HUDDLE_FREE) walls |= 1u << HUDDLE_F;
        if (s[HUDDLE_R] == HUDDLE_FREE) walls |= 1u << HUDDLE_R;
        return walls;
    }
    static const int FLANK_WALLS[4] = {HUDDLE_FL, HUDDLE_RL, HUDDLE_RR, HUDDLE_FR};
    uint8_t walls = 0;
    for (int i = 0; i < 4; i++) {
        int w = FLANK_WALLS[i];
        if (HUDDLE_SPECIAL_FLANK_SIGNAL && s[w] == HUDDLE_FREE &&
            meets_special_flank_condition(robot, w)) {
            walls |= (uint8_t)(1u << w);
        }
    }
    if (robot->nucleus_left) walls |= free_walls(robot, LEFT_FLANK);
    if (robot->nucleus_right) walls |= free_walls(robot, RIGHT_FLANK);
    return walls;
}

/* ---- Algorithm 6: delayed walls ----------------------------------------- */

static bool neighbor_wall_is_free(const huddle_robot *robot,
                                  const uint16_t neighbor_status[HUDDLE_WALLS], int via,
                                  int their_wall) {
    return robot->status[via] == HUDDLE_OCCUPIED &&
           huddle_status_in_word(neighbor_status[via], their_wall) == HUDDLE_FREE;
}

uint8_t huddle_delayed_walls(const huddle_robot *robot,
                             const uint16_t neighbor_status[HUDDLE_WALLS]) {
    int front_side;
    int rear_side;
    if (robot->growth == -1) {
        front_side = HUDDLE_FR;
        rear_side = HUDDLE_RR;
    } else if (robot->growth == 1) {
        front_side = HUDDLE_FL;
        rear_side = HUDDLE_RL;
    } else {
        return 0;
    }
    uint8_t delayed = 0;
    if (neighbor_wall_is_free(robot, neighbor_status, front_side, HUDDLE_F)) {
        delayed |= 1u << HUDDLE_F;
    }
    if (neighbor_wall_is_free(robot, neighbor_status, rear_side, HUDDLE_R)) {
        delayed |= 1u << HUDDLE_R;
    }
    return delayed;
}

uint8_t huddle_signal_walls(const huddle_robot *robot,
                            const uint16_t neighbor_status[HUDDLE_WALLS]) {
    return (uint8_t)(huddle_nominal_walls(robot) & ~huddle_delayed_walls(robot, neighbor_status));
}

/* ---- Wire format -------------------------------------------------------- */

uint16_t huddle_status_word(const huddle_robot *robot) {
    uint16_t word = 0;
    for (int w = 0; w < HUDDLE_WALLS; w++) {
        word |= (uint16_t)((robot->status[w] & 3u) << (2 * w));
    }
    return word;
}

huddle_status huddle_status_in_word(uint16_t word, int wall) {
    return (huddle_status)((word >> (2 * wall)) & 3u);
}

uint16_t huddle_merge_status_word(uint16_t known, uint16_t received) {
    uint16_t merged = known;
    for (int w = 0; w < HUDDLE_WALLS; w++) {
        if (huddle_status_in_word(known, w) == HUDDLE_FREE &&
            huddle_status_in_word(received, w) == HUDDLE_OCCUPIED) {
            merged = (uint16_t)(merged ^ (3u << (2 * w))); /* 01 -> 10 */
        }
    }
    return merged;
}

/* Sign-extends the low `bits` bits of value. */
static int sign_extend(uint32_t value, int bits) {
    uint32_t mask = (1u << bits) - 1u;
    uint32_t sign = 1u << (bits - 1);
    value &= mask;
    return (value & sign) ? (int)value - (int)(mask + 1u) : (int)value;
}

static bool fits_int12(int value) { return value >= -2048 && value <= 2047; }

bool huddle_pose_word(huddle_cell position, int wall, uint32_t *word) {
    if (!fits_int12(position.p) || !fits_int12(position.q) || wall < 0 ||
        wall >= HUDDLE_WALLS) {
        return false;
    }
    *word = ((uint32_t)position.p & 0xFFFu) | ((uint32_t)position.q & 0xFFFu) << 12 |
            ((uint32_t)HUDDLE_UNIT[wall].p & 3u) << 24 |
            ((uint32_t)HUDDLE_UNIT[wall].q & 3u) << 26;
    return true;
}

bool huddle_decode_pose(uint32_t word, huddle_cell *sender, int *sender_wall) {
    if (word >> 28) return false;
    int du = sign_extend(word >> 24, 2);
    int dv = sign_extend(word >> 26, 2);
    for (int w = 0; w < HUDDLE_WALLS; w++) {
        if (HUDDLE_UNIT[w].p == du && HUDDLE_UNIT[w].q == dv) {
            sender->p = (int16_t)sign_extend(word, 12);
            sender->q = (int16_t)sign_extend(word >> 12, 12);
            *sender_wall = w;
            return true;
        }
    }
    return false;
}
