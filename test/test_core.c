/* Hand-checked cases from the paper's figures plus the wire format. */
#include <stdio.h>
#include <string.h>

#include "huddle.h"

static int failures;

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) {                                                 \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            failures++;                                                \
        }                                                              \
    } while (0)

#define BIT(w) (1u << (w))

typedef struct {
    huddle_shape view;
    uint8_t bits[64];
} small_shape;

/* Cells must lie in p, q in [-4, 3]. */
static void make_shape(small_shape *s, const int (*cells)[2], int count) {
    memset(s, 0, sizeof *s);
    s->view = (huddle_shape){-4, -4, 8, 8, s->bits};
    for (int i = 0; i < count; i++) {
        unsigned bit = (unsigned)(cells[i][1] + 4) * 8 + (unsigned)(cells[i][0] + 4);
        s->bits[bit >> 3] |= (uint8_t)(1u << (bit & 7));
    }
}

static huddle_robot dock(const small_shape *s, huddle_cell from, int wall, uint8_t occupied) {
    huddle_robot robot;
    uint32_t pose;
    CHECK(huddle_pose_word(from, wall, &pose));
    CHECK(huddle_init_from_neighbor(&robot, &s->view, pose, occupied));
    return robot;
}

/* Fig. 2: five-robot assembly, root at (0,0). */
static void test_figure_2(void) {
    static const int cells[][2] = {{0, 0}, {0, 1}, {1, 0}, {-1, 1}, {-1, 2}};
    small_shape s;
    make_shape(&s, cells, 5);
    uint16_t none[HUDDLE_WALLS] = {0};

    huddle_robot root;
    huddle_init_root(&root, &s.view, 0);
    CHECK(root.nucleus_right && !root.nucleus_left); /* "a right nucleus" */
    CHECK(huddle_signal_walls(&root, none) == BIT(HUDDLE_F)); /* (a) */

    huddle_robot second = dock(&s, root.position, HUDDLE_F, 0);
    huddle_mark_occupied(&root, HUDDLE_F);
    CHECK(second.position.p == 0 && second.position.q == 1); /* (c) */
    CHECK(huddle_signal_walls(&root, none) == BIT(HUDDLE_FR)); /* (b) */
    CHECK(second.nucleus_left && !second.nucleus_right); /* (d) */
    CHECK(second.growth == -1);
    CHECK(huddle_signal_walls(&second, none) == (BIT(HUDDLE_FL) | BIT(HUDDLE_RL))); /* (e) */
}

/* Fig. 3a(iii): a free FL wall between two null walls makes the robot the
 * only possible left nucleus (Algorithm 2). */
static void test_flank_between_null_walls(void) {
    static const int cells[][2] = {{0, 0}, {0, -1}, {-1, 1}, {-1, 2}};
    small_shape s;
    make_shape(&s, cells, 4);
    huddle_robot robot = dock(&s, (huddle_cell){0, -1}, HUDDLE_F, 0);
    CHECK(robot.status[HUDDLE_F] == HUDDLE_NULL && robot.status[HUDDLE_RL] == HUDDLE_NULL);
    CHECK(robot.nucleus_left);
}

/* Fig. 3a(ii): the midpoint row has no cell in our column, so the nucleus
 * moves to the nearest row that has one (m*). Midpoint of rows 1..4 rounds
 * down to 2; our column holds rows 0..1, so m* = row 1. */
static void test_nearest_connection(void) {
    static const int cells[][2] = {{0, 0}, {0, 1}, {-1, 1}, {-1, 2}, {-1, 3}, {-1, 4}};
    small_shape s;
    make_shape(&s, cells, 6);
    huddle_robot bottom = dock(&s, (huddle_cell){0, 1}, HUDDLE_R, 0);
    huddle_robot top = dock(&s, (huddle_cell){0, 0}, HUDDLE_F, 0);
    CHECK(!bottom.nucleus_left);
    CHECK(top.nucleus_left);
}

/* Midpoint of rows -3..-2 rounds down to -3, not toward zero. */
static void test_negative_midpoint(void) {
    static const int cells[][2] = {{0, 0}, {0, -1}, {0, -2}, {0, -3}, {1, -3}, {1, -2}};
    small_shape s;
    make_shape(&s, cells, 6);
    huddle_robot at_minus3 = dock(&s, (huddle_cell){0, -2}, HUDDLE_R, 0);
    huddle_robot at_minus2 = dock(&s, (huddle_cell){0, -1}, HUDDLE_R, 0);
    CHECK(at_minus3.position.q == -3 && at_minus2.position.q == -2);
    /* Right column rows -3..-2: floor(-2.5) = -3, so the robot at -3
     * nucleates right. Truncation toward zero would pick -2 instead. */
    CHECK(at_minus3.nucleus_right);
    CHECK(!at_minus2.nucleus_right);
}

/* Fig. 3b / Algorithm 6: with g = -1, F waits while the FR neighbor's F is free. */
static void test_delay(void) {
    static const int cells[][2] = {{0, 0}, {0, 1}, {1, 0}, {1, 1}, {-1, 0}};
    small_shape s;
    make_shape(&s, cells, 5);
    huddle_robot robot = dock(&s, (huddle_cell){1, 0}, HUDDLE_RL, 0);
    CHECK(robot.growth == -1);
    huddle_robot neighbor = dock(&s, (huddle_cell){0, 0}, HUDDLE_FR, 0);
    uint16_t words[HUDDLE_WALLS] = {0};
    words[HUDDLE_FR] = huddle_status_word(&neighbor);
    CHECK(huddle_status_in_word(words[HUDDLE_FR], HUDDLE_F) == HUDDLE_FREE);
    CHECK(huddle_nominal_walls(&robot) & BIT(HUDDLE_F));
    CHECK(huddle_delayed_walls(&robot, words) == BIT(HUDDLE_F));
    CHECK(!(huddle_signal_walls(&robot, words) & BIT(HUDDLE_F)));
}

static void test_wire_format(void) {
    for (int p = -2048; p <= 2047; p += 97) {
        for (int w = 0; w < HUDDLE_WALLS; w++) {
            huddle_cell in = {(int16_t)p, (int16_t)(-p - 1)}, out;
            uint32_t word;
            int wall;
            CHECK(huddle_pose_word(in, w, &word));
            CHECK(huddle_decode_pose(word, &out, &wall));
            CHECK(out.p == in.p && out.q == in.q && wall == w);
        }
    }
    uint32_t word;
    CHECK(!huddle_pose_word((huddle_cell){2048, 0}, 0, &word));
    huddle_cell out;
    int wall;
    CHECK(!huddle_decode_pose(0x0F000000u, &out, &wall)); /* (du,dv) = (-1,-1) is not a wall */
    CHECK(!huddle_decode_pose(0x10000000u, &out, &wall)); /* reserved bits set */
}

int main(void) {
    test_figure_2();
    test_flank_between_null_walls();
    test_nearest_connection();
    test_negative_midpoint();
    test_delay();
    test_wire_format();
    printf(failures ? "core tests: %d failed\n" : "core tests: all passed\n", failures);
    return failures != 0;
}
