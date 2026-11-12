/*
 * Neighbor link tests for firmware/huddle_node/node.c and siphash.c: SipHash
 * vectors, the 18-byte frame layout, and rejection of tampered, replayed,
 * misdirected, wrongly keyed and keyless frames.
 */
#include <stdio.h>
#include <string.h>

#include "node.h"

static int failures;
#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
            failures++;                                                      \
        }                                                                    \
    } while (0)

static const uint8_t KEY_A[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
static const char KEY_A_HEX[] = "000102030405060708090a0b0c0d0e0f";
static const char KEY_B_HEX[] = "ffeeddccbbaa99887766554433221100";
static const uint8_t MAC_A[6] = {2, 0, 0, 0, 0, 0xA};
static const uint8_t MAC_B[6] = {2, 0, 0, 0, 0, 0xB};
static const uint8_t MAC_C[6] = {2, 0, 0, 0, 0, 0xC};
static const char TEXT_A[] = "02:00:00:00:00:0a";
static const char TEXT_B[] = "02:00:00:00:00:0b";
static const char TEXT_C[] = "02:00:00:00:00:0c";

/* ---- SipHash-2-4 ------------------------------------------------------- */

/* Key 00..0f, message 00,01,...,len-1. Values from the SipHash paper's reference
 * vectors (lengths 0 and 1), and for the other lengths from OpenSSL 4.1 SIPHASH
 * with 8-byte output, which agrees with the paper on lengths 0 and 1. */
static void test_siphash_vectors(void) {
    static const struct { int len; uint64_t tag; } vectors[] = {
        {0, 0x726fdb47dd0e0e31ULL},  {1, 0x74f839c593dc67fdULL},  {7, 0xab0200f58b01d137ULL},
        {8, 0x93f5f5799a932462ULL},
        {15, 0xa129ca6149be45e5ULL}, {22, 0x93536795e3a33e88ULL},
    };
    uint8_t message[22];
    for (int i = 0; i < 22; i++) message[i] = (uint8_t)i;
    for (size_t i = 0; i < sizeof vectors / sizeof vectors[0]; i++) {
        CHECK(siphash24(KEY_A, message, (size_t)vectors[i].len) == vectors[i].tag);
    }
}

/* ---- Harness: two or three nodes and a frame recorder ------------------- */

typedef struct {
    uint8_t to[6];
    uint8_t frame[NODE_FRAME_LEN];
} sent_frame;

typedef struct {
    sent_frame frames[16];
    int count;
    char last_log[96];
    char all_log[512];
} recorder;

static void record_send(void *ctx, const uint8_t mac[NODE_MAC_LEN], const uint8_t frame[NODE_FRAME_LEN]) {
    recorder *r = ctx;
    if (r->count < 16) {
        memcpy(r->frames[r->count].to, mac, 6);
        memcpy(r->frames[r->count].frame, frame, NODE_FRAME_LEN);
        r->count++;
    }
}

static void record_log(void *ctx, const char *line) {
    recorder *r = ctx;
    snprintf(r->last_log, sizeof r->last_log, "%s", line);
    strncat(r->all_log, line, sizeof r->all_log - strlen(r->all_log) - 1);
}

/* Shape: (0,0) root, (0,1) above it, (-1,1) up-left of it. */
static void boot(node *n, const uint8_t mac[6], recorder *r, const node_io *io, const char *key_hex) {
    (void)r;
    memcpy(n->self_mac, mac, 6);
    node_reset(n);
    if (key_hex) {
        char line[64];
        snprintf(line, sizeof line, "KEY %s", key_hex);
        node_command(n, line, io);
    }
    node_command(n, "CELL 0 0", io);
    node_command(n, "CELL 0 1", io);
    node_command(n, "CELL -1 1", io);
}

static void attach(node *n, int wall, const char *mac_text, const node_io *io) {
    char line[64];
    snprintf(line, sizeof line, "ATTACH %d %s", wall, mac_text);
    node_command(n, line, io);
}

static void write_u32(uint8_t *b, uint32_t v) {
    for (int i = 0; i < 4; i++) b[i] = (uint8_t)(v >> (8 * i));
}

/* Builds a frame from the documented layout, independently of node.c:
 * status u16 | pose u32 | seq u32 | tag u64, all little-endian;
 * tag = SipHash(key, sender_mac || receiver_mac || seq || status || pose). */
static void make_frame(const uint8_t key[16], const uint8_t sender[6], const uint8_t receiver[6],
                       uint16_t status, uint32_t pose, uint32_t seq, uint8_t frame[NODE_FRAME_LEN]) {
    uint8_t msg[22];
    frame[0] = (uint8_t)status;
    frame[1] = (uint8_t)(status >> 8);
    write_u32(frame + 2, pose);
    write_u32(frame + 6, seq);
    memcpy(msg, sender, 6);
    memcpy(msg + 6, receiver, 6);
    memcpy(msg + 12, frame + 6, 4);
    memcpy(msg + 16, frame, 6);
    uint64_t tag = siphash24(key, msg, sizeof msg);
    for (int i = 0; i < 8; i++) frame[10 + i] = (uint8_t)(tag >> (8 * i));
}

/* A and B, A is the root at (0,0) and B docked on its front wall F (B's wall R). */
typedef struct {
    node a, b;
    recorder ra, rb;
    node_io ioa, iob;
} pair;

static void make_pair(pair *p, const char *key_a_hex, const char *key_b_hex) {
    memset(p, 0, sizeof *p);
    p->ioa = (node_io){record_send, record_log, &p->ra};
    p->iob = (node_io){record_send, record_log, &p->rb};
    boot(&p->a, MAC_A, &p->ra, &p->ioa, key_a_hex);
    boot(&p->b, MAC_B, &p->rb, &p->iob, key_b_hex);
    attach(&p->a, HUDDLE_F, TEXT_B, &p->ioa);
    attach(&p->b, HUDDLE_R, TEXT_A, &p->iob);
    node_command(&p->a, "ROOT", &p->ioa);
}

static void test_layout_and_accept(void) {
    pair p;
    make_pair(&p, KEY_A_HEX, KEY_A_HEX);
    node_tick(&p.a, &p.ioa);
    CHECK(p.ra.count == 1 && !memcmp(p.ra.frames[0].to, MAC_B, 6));
    CHECK(NODE_FRAME_LEN == 18);
    uint8_t expected[NODE_FRAME_LEN];
    uint32_t pose;
    CHECK(huddle_pose_word((huddle_cell){0, 0}, HUDDLE_F, &pose));
    make_frame(KEY_A, MAC_A, MAC_B, huddle_status_word(&p.a.robot), pose, 1, expected);
    CHECK(!memcmp(p.ra.frames[0].frame, expected, NODE_FRAME_LEN));
    node_receive(&p.b, MAC_A, p.ra.frames[0].frame, NODE_FRAME_LEN, &p.iob);
    CHECK(p.b.initialized && p.b.robot.position.p == 0 && p.b.robot.position.q == 1);
    CHECK(p.b.rejected == 0);
}

static void test_tampered_frame(void) {
    for (int byte = 0; byte < NODE_FRAME_LEN; byte++) {
        for (int bit = 0; bit < 8; bit += 7) {
            pair p;
            make_pair(&p, KEY_A_HEX, KEY_A_HEX);
            node_tick(&p.a, &p.ioa);
            uint8_t frame[NODE_FRAME_LEN];
            memcpy(frame, p.ra.frames[0].frame, sizeof frame);
            frame[byte] ^= (uint8_t)(1u << bit);
            node_receive(&p.b, MAC_A, frame, NODE_FRAME_LEN, &p.iob);
            CHECK(!p.b.initialized && p.b.rejected == 1);
        }
    }
    pair p;
    make_pair(&p, KEY_A_HEX, KEY_A_HEX);
    node_tick(&p.a, &p.ioa);
    node_receive(&p.b, MAC_A, p.ra.frames[0].frame, NODE_FRAME_LEN - 1, &p.iob); /* truncated */
    CHECK(!p.b.initialized && p.b.rejected == 1);
}

static void test_replayed_frame(void) {
    pair p;
    make_pair(&p, KEY_A_HEX, KEY_A_HEX);
    node_tick(&p.a, &p.ioa);
    node_tick(&p.a, &p.ioa);
    node_receive(&p.b, MAC_A, p.ra.frames[0].frame, NODE_FRAME_LEN, &p.iob);
    CHECK(p.b.initialized && p.b.rejected == 0);
    node_receive(&p.b, MAC_A, p.ra.frames[0].frame, NODE_FRAME_LEN, &p.iob); /* same frame again */
    CHECK(p.b.rejected == 1);
    node_receive(&p.b, MAC_A, p.ra.frames[1].frame, NODE_FRAME_LEN, &p.iob); /* newer one is fine */
    CHECK(p.b.rejected == 1 && p.b.rx_seq[HUDDLE_R] == 2);
    node_receive(&p.b, MAC_A, p.ra.frames[0].frame, NODE_FRAME_LEN, &p.iob); /* old one after newer */
    CHECK(p.b.rejected == 2);
}

/* Even a correctly tagged older word (say, a sender that is behind) cannot
 * lower a wall: occupied stays occupied, null stays null. */
static void test_older_word_cannot_lower(void) {
    pair p;
    make_pair(&p, KEY_A_HEX, KEY_A_HEX);
    uint32_t pose;
    CHECK(huddle_pose_word((huddle_cell){0, 0}, HUDDLE_F, &pose));
    uint16_t newer = (uint16_t)(HUDDLE_FREE | HUDDLE_OCCUPIED << 2 | HUDDLE_NULL << 4);
    uint16_t older = (uint16_t)(HUDDLE_FREE | HUDDLE_FREE << 2 | HUDDLE_NULL << 4);
    uint8_t frame[NODE_FRAME_LEN];
    make_frame(KEY_A, MAC_A, MAC_B, older, pose, 1, frame);
    node_receive(&p.b, MAC_A, frame, NODE_FRAME_LEN, &p.iob);
    make_frame(KEY_A, MAC_A, MAC_B, newer, pose, 2, frame);
    node_receive(&p.b, MAC_A, frame, NODE_FRAME_LEN, &p.iob);
    CHECK(p.b.view[HUDDLE_R] == newer);
    make_frame(KEY_A, MAC_A, MAC_B, older, pose, 3, frame);
    node_receive(&p.b, MAC_A, frame, NODE_FRAME_LEN, &p.iob);
    CHECK(p.b.rejected == 0 && p.b.view[HUDDLE_R] == newer);
}

static void test_wrong_and_missing_key(void) {
    pair p;
    make_pair(&p, KEY_A_HEX, KEY_B_HEX);
    node_tick(&p.a, &p.ioa);
    node_receive(&p.b, MAC_A, p.ra.frames[0].frame, NODE_FRAME_LEN, &p.iob);
    CHECK(!p.b.initialized && p.b.rejected == 1);

    make_pair(&p, KEY_A_HEX, NULL); /* receiver has no key yet */
    node_tick(&p.a, &p.ioa);
    node_receive(&p.b, MAC_A, p.ra.frames[0].frame, NODE_FRAME_LEN, &p.iob);
    CHECK(!p.b.initialized && p.b.rejected == 1);

    make_pair(&p, NULL, KEY_A_HEX); /* sender has no key: sends nothing */
    node_tick(&p.a, &p.ioa);
    CHECK(p.ra.count == 0);

    make_pair(&p, KEY_A_HEX, KEY_A_HEX);
    node_command(&p.b, "RESET", &p.iob); /* a reset forgets the key */
    CHECK(!p.b.has_key);
}

static void test_wrong_source_and_destination(void) {
    pair p;
    make_pair(&p, KEY_A_HEX, KEY_A_HEX);
    node_tick(&p.a, &p.ioa);
    node_receive(&p.b, MAC_C, p.ra.frames[0].frame, NODE_FRAME_LEN, &p.iob); /* spoofed source */
    CHECK(!p.b.initialized && p.b.rejected == 1);

    /* The attacker has the key but not the binding: a frame tagged as coming from C. */
    uint32_t pose;
    uint8_t frame[NODE_FRAME_LEN];
    CHECK(huddle_pose_word((huddle_cell){0, 0}, HUDDLE_F, &pose));
    make_frame(KEY_A, MAC_C, MAC_B, 0x0555, pose, 1, frame);
    node_receive(&p.b, MAC_C, frame, NODE_FRAME_LEN, &p.iob);
    CHECK(!p.b.initialized && p.b.rejected == 2);

    /* A frame addressed to B, delivered to a third board that A is also bound to. */
    node c;
    recorder rc = {0};
    node_io ioc = {record_send, record_log, &rc};
    boot(&c, MAC_C, &rc, &ioc, KEY_A_HEX);
    attach(&c, HUDDLE_R, TEXT_A, &ioc);
    node_receive(&c, MAC_A, p.ra.frames[0].frame, NODE_FRAME_LEN, &ioc);
    CHECK(!c.initialized && c.rejected == 1);
}

static void test_rebinding_resets_sequence(void) {
    pair p;
    make_pair(&p, KEY_A_HEX, KEY_A_HEX);
    uint32_t pose;
    uint8_t frame[NODE_FRAME_LEN];
    CHECK(huddle_pose_word((huddle_cell){0, 0}, HUDDLE_F, &pose));
    make_frame(KEY_A, MAC_A, MAC_B, 0x0555, pose, 100, frame);
    node_receive(&p.b, MAC_A, frame, NODE_FRAME_LEN, &p.iob);
    CHECK(p.b.rx_seq[HUDDLE_R] == 100);
    attach(&p.b, HUDDLE_R, TEXT_A, &p.iob); /* same peer again: counter kept */
    CHECK(p.b.rx_seq[HUDDLE_R] == 100);
    attach(&p.b, HUDDLE_R, TEXT_C, &p.iob); /* a different board: counter starts over */
    CHECK(p.b.rx_seq[HUDDLE_R] == 0);
    make_frame(KEY_A, MAC_C, MAC_B, 0x0555, pose, 1, frame);
    node_receive(&p.b, MAC_C, frame, NODE_FRAME_LEN, &p.iob);
    CHECK(p.b.rx_seq[HUDDLE_R] == 1);
    make_frame(KEY_A, MAC_C, MAC_B, 0x0555, pose, 0, frame); /* sequence 0 is never valid */
    node_receive(&p.b, MAC_C, frame, NODE_FRAME_LEN, &p.iob);
    CHECK(p.b.rx_seq[HUDDLE_R] == 1);
}

static void test_key_command(void) {
    node n;
    recorder r = {0};
    node_io io = {record_send, record_log, &r};
    boot(&n, MAC_A, &r, &io, NULL);
    node_command(&n, "KEY 0011", &io);
    CHECK(!n.has_key && !strcmp(r.last_log, "ERR bad key"));
    node_command(&n, "KEY 0011223344556677889900aabbccddeeZZ", &io);
    CHECK(!n.has_key);
    node_command(&n, "KEY 0011223344556677889900aabbccddee", &io);
    CHECK(n.has_key && !strcmp(r.last_log, "OK KEY"));
    CHECK(strstr(r.all_log, "0011223344") == NULL); /* the key is never echoed */
}

int main(void) {
    test_siphash_vectors();
    test_layout_and_accept();
    test_tampered_frame();
    test_replayed_frame();
    test_older_word_cannot_lower();
    test_wrong_and_missing_key();
    test_wrong_source_and_destination();
    test_rebinding_resets_sequence();
    test_key_command();
    printf(failures ? "node tests: %d failed\n" : "node tests: all passed\n", failures);
    return failures != 0;
}
