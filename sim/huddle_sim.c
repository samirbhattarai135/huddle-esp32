/*
 * Monte Carlo harness for the Huddle core (core/huddle.c).
 *
 * Generates random hole-free hexagonal shapes, assembles each one with k
 * simultaneous attachments per step (k = 1..4), and checks Definition 1
 * (reachability) and Definition 2 (hole-free) after every step.
 *
 * Link models:
 *   sync     neighbor status words are exchanged every step (the paper's model)
 *   lossy    each status message is dropped with probability --drop and
 *            delivered after 0..--delay steps; robots act on the newest word
 *            they have. --unknown picks what a robot assumes before the first
 *            word from a new neighbor arrives.
 *   latency  a passing-by robot commits to a signaled opening at step t and
 *            docks at step t + --latency; --recheck makes it abort if the
 *            opening is no longer signaled when it arrives.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "huddle.h"

#define GRID 512
#define GRID_OFFSET 256
#define SHAPE_RADIUS 120
#define MAX_CELLS 1024
#define MAX_DELAY 8
#define MAX_K 4

typedef struct {
    bool lossy;
    double drop;
    int delay;
    bool unknown_is_free;
    int latency;
    bool recheck;
    bool ablate_delay; /* negative control: Algorithm 6 sees no neighbor state */
    bool blind_simultaneous; /* same-step dockers do not see each other at init */
} link_config;

typedef struct {
    uint64_t seed;
    int shapes;
    int shape_offset;
    int min_size;
    int max_size;
    int k_min;
    int k_max;
    bool verify_holes;
    bool render;
    link_config link;
} options;

/* ---- RNG (splitmix64) ---------------------------------------------------- */

typedef struct {
    uint64_t state;
} rng;

static uint64_t rng_next(rng *r) {
    uint64_t z = (r->state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

static int rng_below(rng *r, int n) { return (int)(rng_next(r) % (uint64_t)n); }
static double rng_unit(rng *r) { return (double)(rng_next(r) >> 11) / 9007199254740992.0; }

static rng rng_for(uint64_t seed, uint64_t a, uint64_t b) {
    rng r = {seed ^ (a * 0xD1B54A32D192ED03ull) ^ (b * 0xABC98388FB8FAC03ull)};
    rng_next(&r);
    return r;
}

/* ---- Grid helpers -------------------------------------------------------- */

static int cell_index(int p, int q) { return (p + GRID_OFFSET) * GRID + (q + GRID_OFFSET); }

static huddle_cell neighbor_of(huddle_cell c, int wall) {
    huddle_cell n = {(int16_t)(c.p + HUDDLE_UNIT[wall].p), (int16_t)(c.q + HUDDLE_UNIT[wall].q)};
    return n;
}

/*
 * Euler characteristic of a cell set on the hex lattice, viewed as the
 * triangulated complex (cells, adjacencies, triangles of 3 mutually adjacent
 * cells): chi = V - E + T = components - holes. Adding cell c changes it by
 * 1 - (occupied neighbors) + (adjacent occupied neighbor pairs).
 */
static int euler_delta(const uint8_t *occupied, huddle_cell c) {
    int edges = 0;
    int triangles = 0;
    for (int w = 0; w < HUDDLE_WALLS; w++) {
        huddle_cell a = neighbor_of(c, w);
        huddle_cell b = neighbor_of(c, (w + 1) % HUDDLE_WALLS);
        bool has_a = occupied[cell_index(a.p, a.q)];
        edges += has_a;
        triangles += has_a && occupied[cell_index(b.p, b.q)];
    }
    return 1 - edges + triangles;
}

/* ---- Shape generation ---------------------------------------------------- */

typedef struct {
    huddle_cell cells[MAX_CELLS];
    int count;
    huddle_shape view; /* root-frame bitmap */
    uint8_t bits[(4 * SHAPE_RADIUS + 3) * (4 * SHAPE_RADIUS + 3) / 8 + 1];
} target_shape;

static uint8_t g_scratch[GRID * GRID];

static int shape_neighbors(huddle_cell c) {
    int count = 0;
    for (int w = 0; w < HUDDLE_WALLS; w++) {
        huddle_cell n = neighbor_of(c, w);
        count += g_scratch[cell_index(n.p, n.q)];
    }
    return count;
}

/*
 * Random growth from (0,0). Style 0 picks uniformly among candidates, style 1
 * prefers candidates with few shape neighbors (thin, branchy shapes), style 2
 * prefers many (compact blobs). A candidate that would enclose a hole
 * (Euler characteristic below 1) is rejected.
 */
static void generate_shape(rng *r, int size, target_shape *shape) {
    int style = rng_below(r, 3);
    shape->cells[0] = (huddle_cell){0, 0};
    shape->count = 1;
    g_scratch[cell_index(0, 0)] = 1;
    int attempts = 0;
    while (shape->count < size && attempts < 200 * size) {
        attempts++;
        huddle_cell best = {0, 0};
        int best_score = -1;
        for (int draw = 0; draw < (style == 0 ? 1 : 3); draw++) {
            huddle_cell from = shape->cells[rng_below(r, shape->count)];
            huddle_cell c = neighbor_of(from, rng_below(r, HUDDLE_WALLS));
            if (abs(c.p) > SHAPE_RADIUS || abs(c.q) > SHAPE_RADIUS) continue;
            if (g_scratch[cell_index(c.p, c.q)]) continue;
            int n = shape_neighbors(c);
            int score = style == 1 ? 6 - n : n;
            if (score > best_score) {
                best_score = score;
                best = c;
            }
        }
        if (best_score < 0) continue;
        if (euler_delta(g_scratch, best) < 0) continue; /* would close a hole */
        g_scratch[cell_index(best.p, best.q)] = 1;
        shape->cells[shape->count++] = best;
    }
    for (int i = 0; i < shape->count; i++) {
        g_scratch[cell_index(shape->cells[i].p, shape->cells[i].q)] = 0;
    }

    /* Move a random cell to the origin: that is where the root robot sits. */
    huddle_cell root = shape->cells[rng_below(r, shape->count)];
    int p_min = 1 << 20, q_min = 1 << 20, p_max = -(1 << 20), q_max = -(1 << 20);
    for (int i = 0; i < shape->count; i++) {
        huddle_cell *c = &shape->cells[i];
        c->p = (int16_t)(c->p - root.p);
        c->q = (int16_t)(c->q - root.q);
        if (c->p < p_min) p_min = c->p;
        if (c->q < q_min) q_min = c->q;
        if (c->p > p_max) p_max = c->p;
        if (c->q > q_max) q_max = c->q;
    }
    shape->view.p_min = (int16_t)p_min;
    shape->view.q_min = (int16_t)q_min;
    shape->view.width = (uint16_t)(p_max - p_min + 1);
    shape->view.height = (uint16_t)(q_max - q_min + 1);
    size_t bytes = ((size_t)shape->view.width * shape->view.height + 7) / 8;
    memset(shape->bits, 0, bytes);
    for (int i = 0; i < shape->count; i++) {
        unsigned bit = (unsigned)(shape->cells[i].q - q_min) * shape->view.width +
                       (unsigned)(shape->cells[i].p - p_min);
        shape->bits[bit >> 3] |= (uint8_t)(1u << (bit & 7));
    }
    shape->view.bits = shape->bits;
}

/* ---- Assembly world ------------------------------------------------------ */

typedef struct {
    int32_t deliver_at;
    int32_t sent_at;
    uint16_t word;
    bool in_flight;
} message;

typedef struct {
    huddle_robot core;
    int16_t neighbor[HUDDLE_WALLS];
    uint16_t view[HUDDLE_WALLS];
    int32_t view_sent_at[HUDDLE_WALLS];
    message inbox[HUDDLE_WALLS][MAX_DELAY + 1];
    uint8_t last_signals;
} sim_robot;

typedef struct {
    huddle_cell cell;
    int16_t sender;
    int8_t sender_wall;
    int32_t land_at;
} arrival;

typedef struct {
    long raised;     /* wall-steps with a signal on */
    long withdrawn;  /* signal on at t-1, off at t, wall still free */
    long suppressed; /* off only because the robot's neighbor view is stale */
} signal_stats;

typedef enum {
    OUTCOME_COMPLETE,
    OUTCOME_UNREACHABLE,
    OUTCOME_HOLE,
    OUTCOME_DEADLOCK,
    OUTCOME_MISPOSITION,
    OUTCOME_OFF_TARGET,
    OUTCOME_STEP_LIMIT,
    OUTCOME_COUNT
} outcome;

static const char *OUTCOME_NAME[OUTCOME_COUNT] = {
    "complete", "unreachable", "hole", "deadlock", "misposition", "off_target", "step_limit"};

typedef struct {
    outcome result;
    int steps;
    int aborted_dockings;
    int robots;
    signal_stats signals;
} trial_result;

static int16_t g_robot_at[GRID * GRID];
static uint8_t g_occupied[GRID * GRID];
static uint8_t g_target[GRID * GRID];
static uint8_t g_degree[GRID * GRID];
static sim_robot g_robots[MAX_CELLS];
static arrival g_arrivals[MAX_CELLS];

#define ALL_FREE_WORD 0x0555u /* every wall reported free */

static uint16_t neighbor_word(const sim_robot *robot, int wall, const link_config *link) {
    if (link->ablate_delay) return 0;
    if (!link->lossy) return robot->view[wall];
    if (robot->view_sent_at[wall] < 0) return link->unknown_is_free ? ALL_FREE_WORD : 0;
    return robot->view[wall];
}

static void exchange_status(int robot_count, int now, const link_config *link, rng *r) {
    for (int i = 0; i < robot_count; i++) {
        sim_robot *robot = &g_robots[i];
        for (int w = 0; w < HUDDLE_WALLS; w++) {
            int n = robot->neighbor[w];
            if (n < 0) continue;
            uint16_t word = huddle_status_word(&g_robots[n].core);
            if (!link->lossy) {
                robot->view[w] = word;
                continue;
            }
            if (rng_unit(r) >= link->drop) {
                message *slot = &robot->inbox[w][now % (MAX_DELAY + 1)];
                slot->deliver_at = now + (link->delay ? rng_below(r, link->delay + 1) : 0);
                slot->sent_at = now;
                slot->word = word;
                slot->in_flight = true;
            }
            for (int s = 0; s <= MAX_DELAY; s++) {
                message *m = &robot->inbox[w][s];
                if (!m->in_flight || m->deliver_at > now) continue;
                m->in_flight = false;
                if (m->sent_at > robot->view_sent_at[w]) {
                    robot->view[w] = m->word;
                    robot->view_sent_at[w] = m->sent_at;
                }
            }
        }
    }
}

static bool is_target_cell(huddle_cell c) { return g_target[cell_index(c.p, c.q)]; }
static bool is_occupied_cell(huddle_cell c) { return g_occupied[cell_index(c.p, c.q)]; }

/* Flood fill over empty cells from outside the shape's bounding box. */
static bool has_hole_by_flood(const target_shape *shape) {
    static int32_t stack[GRID * GRID];
    static uint8_t seen[GRID * GRID];
    int p0 = shape->view.p_min - 1, q0 = shape->view.q_min - 1;
    int p1 = shape->view.p_min + shape->view.width, q1 = shape->view.q_min + shape->view.height;
    for (int p = p0; p <= p1; p++)
        for (int q = q0; q <= q1; q++) seen[cell_index(p, q)] = 0;
    int top = 0;
    stack[top++] = cell_index(p0, q0);
    seen[cell_index(p0, q0)] = 1;
    while (top) {
        int idx = stack[--top];
        huddle_cell c = {(int16_t)(idx / GRID - GRID_OFFSET), (int16_t)(idx % GRID - GRID_OFFSET)};
        for (int w = 0; w < HUDDLE_WALLS; w++) {
            huddle_cell n = neighbor_of(c, w);
            if (n.p < p0 || n.p > p1 || n.q < q0 || n.q > q1) continue;
            int ni = cell_index(n.p, n.q);
            if (seen[ni] || g_occupied[ni]) continue;
            seen[ni] = 1;
            stack[top++] = ni;
        }
    }
    for (int i = 0; i < shape->count; i++) {
        int idx = cell_index(shape->cells[i].p, shape->cells[i].q);
        if (!g_occupied[idx] && !seen[idx]) return true;
    }
    return false;
}

typedef struct {
    int robot_count;
    int euler;
    int new_count;
    int16_t new_ids[MAX_CELLS];
} world;

static void place_robot(world *wd, huddle_cell cell) {
    int id = wd->robot_count++;
    sim_robot *robot = &g_robots[id];
    memset(robot, 0, sizeof *robot);
    robot->core.position = cell;
    for (int w = 0; w < HUDDLE_WALLS; w++) {
        robot->neighbor[w] = -1;
        robot->view_sent_at[w] = -1;
    }
    int idx = cell_index(cell.p, cell.q);
    wd->euler += euler_delta(g_occupied, cell);
    g_occupied[idx] = 1;
    g_robot_at[idx] = (int16_t)id;
    for (int w = 0; w < HUDDLE_WALLS; w++) {
        huddle_cell n = neighbor_of(cell, w);
        g_degree[cell_index(n.p, n.q)]++;
    }
    wd->new_ids[wd->new_count++] = (int16_t)id;
}

/* Wires up attachments for this step's new robots and runs their Initialization. */
static bool initialize_new_robots(world *wd, const target_shape *shape, const int16_t *senders,
                                  const int8_t *sender_walls, bool blind_simultaneous) {
    int first_new = wd->new_count ? wd->new_ids[0] : wd->robot_count;
    for (int i = 0; i < wd->new_count; i++) {
        int id = wd->new_ids[i];
        sim_robot *robot = &g_robots[id];
        uint8_t occupied = 0;
        for (int w = 0; w < HUDDLE_WALLS; w++) {
            huddle_cell n = neighbor_of(robot->core.position, w);
            int16_t other = g_robot_at[cell_index(n.p, n.q)];
            if (!g_occupied[cell_index(n.p, n.q)]) continue;
            if (!blind_simultaneous || other < first_new) occupied |= (uint8_t)(1u << w);
            robot->neighbor[w] = other;
            int facing = (w + 3) % HUDDLE_WALLS;
            g_robots[other].neighbor[facing] = (int16_t)id;
            huddle_mark_occupied(&g_robots[other].core, facing);
        }
        huddle_cell actual = robot->core.position;
        if (senders[i] < 0) {
            huddle_init_root(&robot->core, &shape->view, occupied);
        } else {
            uint32_t pose;
            if (!huddle_pose_word(g_robots[senders[i]].core.position, sender_walls[i], &pose) ||
                !huddle_init_from_neighbor(&robot->core, &shape->view, pose, occupied)) {
                return false;
            }
        }
        if (robot->core.position.p != actual.p || robot->core.position.q != actual.q) {
            return false;
        }
    }
    /* Robots that docked in the same step see each other at init. Mark any
     * wall that became occupied after its owner initialized. */
    for (int i = 0; i < wd->new_count; i++) {
        sim_robot *robot = &g_robots[wd->new_ids[i]];
        for (int w = 0; w < HUDDLE_WALLS; w++) {
            if (robot->neighbor[w] >= 0) huddle_mark_occupied(&robot->core, w);
        }
    }
    return true;
}

/* Every robot's wall status must match the ground truth around it. */
static int first_stale_status(int robot_count) {
    for (int i = 0; i < robot_count; i++) {
        const huddle_robot *robot = &g_robots[i].core;
        for (int w = 0; w < HUDDLE_WALLS; w++) {
            huddle_cell n = neighbor_of(robot->position, w);
            int idx = cell_index(n.p, n.q);
            uint8_t truth = g_occupied[idx] ? HUDDLE_OCCUPIED : g_target[idx] ? HUDDLE_FREE : HUDDLE_NULL;
            if (robot->status[w] != truth) return i;
        }
    }
    return -1;
}

static bool any_unreachable(const world *wd) {
    for (int i = 0; i < wd->new_count; i++) {
        huddle_cell c = g_robots[wd->new_ids[i]].core.position;
        for (int w = 0; w < HUDDLE_WALLS; w++) {
            huddle_cell n = neighbor_of(c, w);
            int idx = cell_index(n.p, n.q);
            if (g_target[idx] && !g_occupied[idx] && g_degree[idx] > 3) return true;
        }
    }
    return false;
}

static void reset_world(const target_shape *shape, int robot_count) {
    for (int i = 0; i < robot_count; i++) {
        huddle_cell c = g_robots[i].core.position;
        g_occupied[cell_index(c.p, c.q)] = 0;
        g_robot_at[cell_index(c.p, c.q)] = -1;
        for (int w = 0; w < HUDDLE_WALLS; w++) {
            huddle_cell n = neighbor_of(c, w);
            g_degree[cell_index(n.p, n.q)] = 0;
        }
    }
    for (int i = 0; i < shape->count; i++) {
        g_target[cell_index(shape->cells[i].p, shape->cells[i].q)] = 0;
    }
}

typedef struct {
    huddle_cell cell;
    int16_t sender;
    int8_t sender_wall;
} opening;

static void tally_signals(sim_robot *robot, uint8_t signals, const link_config *link,
                          signal_stats *stats) {
    uint8_t still_free = 0;
    for (int w = 0; w < HUDDLE_WALLS; w++) {
        if (robot->core.status[w] == HUDDLE_FREE) still_free |= (uint8_t)(1u << w);
    }
    for (uint8_t bits = signals; bits; bits &= (uint8_t)(bits - 1)) stats->raised++;
    for (uint8_t bits = robot->last_signals & ~signals & still_free; bits; bits &= (uint8_t)(bits - 1)) {
        stats->withdrawn++;
    }
    robot->last_signals = signals;
    if (!link->lossy) return;
    uint16_t fresh[HUDDLE_WALLS];
    for (int w = 0; w < HUDDLE_WALLS; w++) {
        int n = robot->neighbor[w];
        fresh[w] = n < 0 ? 0 : huddle_status_word(&g_robots[n].core);
    }
    uint8_t ideal = huddle_signal_walls(&robot->core, fresh);
    for (uint8_t bits = ideal & ~signals; bits; bits &= (uint8_t)(bits - 1)) stats->suppressed++;
}

/* Signaled openings, one entry per distinct cell. Returns -1 if a robot
 * signals toward a cell outside the target shape. stats may be NULL. */
static int collect_openings(int robot_count, const link_config *link, opening *out,
                            signal_stats *stats) {
    static int32_t listed_at[GRID * GRID];
    static int32_t epoch;
    epoch++;
    int count = 0;
    for (int i = 0; i < robot_count; i++) {
        sim_robot *robot = &g_robots[i];
        uint16_t words[HUDDLE_WALLS];
        for (int w = 0; w < HUDDLE_WALLS; w++) words[w] = neighbor_word(robot, w, link);
        uint8_t signals = huddle_signal_walls(&robot->core, words);
        if (stats) tally_signals(robot, signals, link, stats);
        for (int w = 0; w < HUDDLE_WALLS; w++) {
            if (!(signals & (1u << w))) continue;
            huddle_cell c = neighbor_of(robot->core.position, w);
            if (!is_target_cell(c) || is_occupied_cell(c)) return -1;
            int idx = cell_index(c.p, c.q);
            if (listed_at[idx] == epoch) continue;
            listed_at[idx] = epoch;
            out[count++] = (opening){c, (int16_t)i, (int8_t)w};
        }
    }
    return count;
}

static bool opening_listed(const opening *list, int count, huddle_cell c, int *at) {
    for (int i = 0; i < count; i++) {
        if (list[i].cell.p == c.p && list[i].cell.q == c.q) {
            *at = i;
            return true;
        }
    }
    return false;
}

static trial_result run_trial(const target_shape *shape, int k, const link_config *link, rng *r,
                              bool verify_holes) {
    static opening openings[MAX_CELLS];
    static int16_t senders[MAX_CELLS];
    static int8_t sender_walls[MAX_CELLS];
    trial_result result = {OUTCOME_COMPLETE, 0, 0, 0, {0, 0, 0}};
    for (int i = 0; i < shape->count; i++) {
        g_target[cell_index(shape->cells[i].p, shape->cells[i].q)] = 1;
    }
    world wd = {0, 0, 0, {0}};
    int pending = 0;
    int step_limit = 20 * shape->count + 200;

    place_robot(&wd, (huddle_cell){0, 0});
    senders[0] = -1;
    sender_walls[0] = -1;
    if (!initialize_new_robots(&wd, shape, senders, sender_walls, link->blind_simultaneous)) {
        result.result = OUTCOME_MISPOSITION;
        goto done;
    }

    for (int now = 1;; now++) {
        if (wd.robot_count == shape->count) break;
        if (now > step_limit) {
            result.result = OUTCOME_STEP_LIMIT;
            break;
        }
        result.steps = now;
        exchange_status(wd.robot_count, now, link, r);
        int open = collect_openings(wd.robot_count, link, openings, &result.signals);
        if (open < 0) {
            result.result = OUTCOME_OFF_TARGET;
            break;
        }

        /* Openings that already have a robot on the way are not chosen again. */
        for (int a = 0; a < pending; a++) {
            int at;
            if (opening_listed(openings, open, g_arrivals[a].cell, &at)) openings[at] = openings[--open];
        }
        int pick = open < k ? open : k;
        for (int i = 0; i < pick; i++) {
            int j = i + rng_below(r, open - i);
            opening chosen = openings[j];
            openings[j] = openings[i];
            openings[i] = chosen;
            int land_at = now + link->latency;
            g_arrivals[pending++] = (arrival){chosen.cell, chosen.sender, chosen.sender_wall, land_at};
        }
        if (pending == 0) {
            result.result = OUTCOME_DEADLOCK;
            break;
        }

        /* Dock every robot due this step, simultaneously. */
        int signaled_now = 0;
        if (link->latency > 0 && link->recheck) {
            signaled_now = collect_openings(wd.robot_count, link, openings, NULL);
        }
        wd.new_count = 0;
        int kept = 0;
        for (int a = 0; a < pending; a++) {
            arrival due = g_arrivals[a];
            if (due.land_at > now) {
                g_arrivals[kept++] = due;
                continue;
            }
            int at;
            if (link->latency > 0 && link->recheck &&
                !opening_listed(openings, signaled_now, due.cell, &at)) {
                result.aborted_dockings++;
                continue;
            }
            senders[wd.new_count] = due.sender;
            sender_walls[wd.new_count] = due.sender_wall;
            place_robot(&wd, due.cell);
        }
        pending = kept;
        if (!initialize_new_robots(&wd, shape, senders, sender_walls, link->blind_simultaneous)) {
            result.result = OUTCOME_MISPOSITION;
            break;
        }
        if (any_unreachable(&wd)) {
            result.result = OUTCOME_UNREACHABLE;
            break;
        }
        if (verify_holes && first_stale_status(wd.robot_count) >= 0) {
            int i = first_stale_status(wd.robot_count);
            for (int w = 0; w < 6; w++) fprintf(stderr, "%d", g_robots[i].core.status[w]);
            fprintf(stderr, " internal: robot (%d,%d) wall status disagrees with the world at step %d\n",
                    g_robots[i].core.position.p, g_robots[i].core.position.q, now);
            exit(2);
        }
        bool hole = wd.euler != 1;
        if (verify_holes && hole != has_hole_by_flood(shape)) {
            fprintf(stderr, "internal: Euler and flood-fill hole checks disagree\n");
            exit(2);
        }
        if (hole) {
            result.result = OUTCOME_HOLE;
            break;
        }
    }
done:
    result.robots = wd.robot_count;
    reset_world(shape, wd.robot_count);
    return result;
}

/* ---- ASCII rendering ----------------------------------------------------- */

/* Column p is drawn at x = 3p; a row step moves 2 lines, a column step 1 line
 * (axial coordinates on a flat-topped hex layout). */
static void render(const target_shape *shape, int robot_count) {
    int x_min = 1 << 20, x_max = -(1 << 20), y_min = 1 << 20, y_max = -(1 << 20);
    for (int i = 0; i < shape->count; i++) {
        int x = 3 * shape->cells[i].p, y = 2 * shape->cells[i].q + shape->cells[i].p;
        if (x < x_min) x_min = x;
        if (x > x_max) x_max = x;
        if (y < y_min) y_min = y;
        if (y > y_max) y_max = y;
    }
    int width = x_max - x_min + 2, height = y_max - y_min + 1;
    if (width > 200 || height > 120) {
        printf("(shape too large to render: %dx%d)\n", width, height);
        return;
    }
    static char canvas[120][202];
    for (int y = 0; y < height; y++) {
        memset(canvas[y], ' ', (size_t)width);
        canvas[y][width] = '\0';
    }
    for (int i = 0; i < shape->count; i++) {
        int x = 3 * shape->cells[i].p - x_min, y = y_max - (2 * shape->cells[i].q + shape->cells[i].p);
        canvas[y][x] = '.';
    }
    for (int i = 0; i < robot_count; i++) {
        const huddle_robot *robot = &g_robots[i].core;
        int x = 3 * robot->position.p - x_min, y = y_max - (2 * robot->position.q + robot->position.p);
        char mark = 'o';
        if (robot->nucleus_left && robot->nucleus_right) mark = 'B';
        else if (robot->nucleus_left) mark = 'L';
        else if (robot->nucleus_right) mark = 'R';
        canvas[y][x] = i == 0 ? '@' : mark;
    }
    for (int y = 0; y < height; y++) printf("    %s\n", canvas[y]);
    if (getenv("HUDDLE_DUMP")) {
        for (int i = 0; i < robot_count; i++) {
            const huddle_robot *robot = &g_robots[i].core;
            printf("    robot (%d,%d) left=%d right=%d g=%d status=", robot->position.p,
                   robot->position.q, robot->nucleus_left, robot->nucleus_right, robot->growth);
            for (int w = 0; w < HUDDLE_WALLS; w++) printf("%c", "NFO"[robot->status[w]]);
            printf("\n");
        }
        for (int i = 0; i < shape->count; i++) printf("    target (%d,%d)\n", shape->cells[i].p, shape->cells[i].q);
    }
    printf("    @ root, L/R/B left/right/both-flank nucleus, o regular, . unfilled target\n");
}

/* ---- Driver -------------------------------------------------------------- */

static const char *model_label(const link_config *link, char *buf, size_t len) {
    int n = snprintf(buf, len, "%s", link->lossy ? "lossy" : "sync");
    if (link->lossy) {
        n += snprintf(buf + n, len - (size_t)n, " drop=%.2f delay<=%d unknown=%s", link->drop,
                      link->delay, link->unknown_is_free ? "free" : "null");
    }
    if (link->latency > 0) {
        n += snprintf(buf + n, len - (size_t)n, " latency=%d recheck=%s", link->latency,
                      link->recheck ? "on" : "off");
    }
    if (link->ablate_delay) n += snprintf(buf + n, len - (size_t)n, " Algorithm-6-disabled");
    if (link->blind_simultaneous) snprintf(buf + n, len - (size_t)n, " blind-simultaneous");
    return buf;
}

static void usage(void) {
    fprintf(stderr,
            "usage: huddle_sim [--shapes N] [--seed S] [--offset I] [--min-size A] [--max-size B]\n"
            "                  [--k K | --k-range A B] [--verify-holes] [--render]\n"
            "                  [--lossy DROP DELAY free|null] [--latency L on|off] [--no-delay]\n");
    exit(2);
}

static options parse_options(int argc, char **argv) {
    options o = {1, 1000, 0, 10, 300, 1, MAX_K, false, false, {false, 0, 0, true, 0, false, false, false}};
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        bool more = i + 1 < argc;
        if (!strcmp(a, "--shapes") && more) o.shapes = atoi(argv[++i]);
        else if (!strcmp(a, "--seed") && more) o.seed = strtoull(argv[++i], NULL, 10);
        else if (!strcmp(a, "--offset") && more) o.shape_offset = atoi(argv[++i]);
        else if (!strcmp(a, "--min-size") && more) o.min_size = atoi(argv[++i]);
        else if (!strcmp(a, "--max-size") && more) o.max_size = atoi(argv[++i]);
        else if (!strcmp(a, "--k") && more) o.k_min = o.k_max = atoi(argv[++i]);
        else if (!strcmp(a, "--k-range") && i + 2 < argc) {
            o.k_min = atoi(argv[++i]);
            o.k_max = atoi(argv[++i]);
        } else if (!strcmp(a, "--verify-holes")) o.verify_holes = true;
        else if (!strcmp(a, "--render")) o.render = true;
        else if (!strcmp(a, "--no-delay")) o.link.ablate_delay = true;
        else if (!strcmp(a, "--blind-simultaneous")) o.link.blind_simultaneous = true;
        else if (!strcmp(a, "--lossy") && i + 3 < argc) {
            o.link.lossy = true;
            o.link.drop = atof(argv[++i]);
            o.link.delay = atoi(argv[++i]);
            o.link.unknown_is_free = !strcmp(argv[++i], "free");
        } else if (!strcmp(a, "--latency") && i + 2 < argc) {
            o.link.latency = atoi(argv[++i]);
            o.link.recheck = !strcmp(argv[++i], "on");
        } else usage();
    }
    if (o.min_size < 1 || o.max_size >= MAX_CELLS || o.min_size > o.max_size || o.k_min < 1 ||
        o.k_max > MAX_K || o.k_min > o.k_max || o.link.delay < 0 || o.link.delay > MAX_DELAY ||
        o.link.drop < 0 || o.link.drop >= 1 || o.link.latency < 0) {
        usage();
    }
    return o;
}

int main(int argc, char **argv) {
    options o = parse_options(argc, argv);
    static target_shape shape;
    memset(g_robot_at, 0xFF, sizeof g_robot_at);

    long trials[MAX_K + 1] = {0}, outcomes[MAX_K + 1][OUTCOME_COUNT] = {{0}};
    long steps[MAX_K + 1] = {0}, aborted[MAX_K + 1] = {0};
    long raised[MAX_K + 1] = {0}, withdrawn[MAX_K + 1] = {0}, suppressed[MAX_K + 1] = {0};
    long size_sum = 0;
    int size_max = 0;
    bool rendered = false;
    int failures_drawn = 0;

    for (int s = o.shape_offset; s < o.shape_offset + o.shapes; s++) {
        rng shape_rng = rng_for(o.seed, (uint64_t)s, 0);
        int size = o.min_size + rng_below(&shape_rng, o.max_size - o.min_size + 1);
        generate_shape(&shape_rng, size, &shape);
        size_sum += shape.count;
        if (shape.count > size_max) size_max = shape.count;
        for (int k = o.k_min; k <= o.k_max; k++) {
            rng trial_rng = rng_for(o.seed, (uint64_t)s, (uint64_t)k);
            trial_result t = run_trial(&shape, k, &o.link, &trial_rng, o.verify_holes);
            trials[k]++;
            outcomes[k][t.result]++;
            aborted[k] += t.aborted_dockings;
            raised[k] += t.signals.raised;
            withdrawn[k] += t.signals.withdrawn;
            suppressed[k] += t.signals.suppressed;
            if (t.result == OUTCOME_COMPLETE) steps[k] += t.steps;
            else {
                printf("FAIL shape=%d seed=%" PRIu64 " k=%d size=%d outcome=%s step=%d\n", s, o.seed,
                       k, shape.count, OUTCOME_NAME[t.result], t.steps);
                if (o.render && failures_drawn++ < 1) render(&shape, t.robots);
            }
            if (o.render && !rendered && t.result == OUTCOME_COMPLETE && shape.count >= 40 &&
                shape.count <= 120) {
                /* g_robots still holds the trial that just finished. */
                printf("Final assembly, shape %d (%d robots, k=%d):\n", s, shape.count, k);
                render(&shape, shape.count);
                rendered = true;
            }
        }
    }

    char label[96];
    model_label(&o.link, label, sizeof label);
    for (int k = o.k_min; k <= o.k_max; k++) {
        printf("RESULT model=\"%s\" k=%d trials=%ld", label, k, trials[k]);
        for (int out = 0; out < OUTCOME_COUNT; out++) printf(" %s=%ld", OUTCOME_NAME[out], outcomes[k][out]);
        printf(" steps=%ld aborted=%ld raised=%ld withdrawn=%ld suppressed=%ld shapes=%d size_sum=%ld "
               "size_max=%d\n",
               steps[k], aborted[k], raised[k], withdrawn[k], suppressed[k], o.shapes, size_sum, size_max);
    }
    long failures = 0;
    for (int k = o.k_min; k <= o.k_max; k++) failures += trials[k] - outcomes[k][OUTCOME_COMPLETE];
    return failures ? 1 : 0;
}
