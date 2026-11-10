#include "node.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Before a neighbor's first status word arrives, assume all its walls are
 * free. Algorithm 6 then delays rather than signals; assuming "null" instead
 * breaks reachability in the lossy-link simulation (see README). */
#define UNKNOWN_NEIGHBOR_WORD 0x0555u

static void say(const node_io *io, const char *line) { io->log(io->ctx, line); }

static void sayf(const node_io *io, const char *fmt, ...) {
    char line[96];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof line, fmt, args);
    va_end(args);
    say(io, line);
}

void node_reset(node *n) {
    uint8_t mac[NODE_MAC_LEN];
    memcpy(mac, n->self_mac, sizeof mac);
    memset(n, 0, sizeof *n);
    memcpy(n->self_mac, mac, sizeof mac);
    n->shape = (huddle_shape){-NODE_WINDOW / 2, -NODE_WINDOW / 2, NODE_WINDOW, NODE_WINDOW,
                              n->shape_bits};
}

static bool parse_mac(const char *text, uint8_t mac[NODE_MAC_LEN]) {
    for (int i = 0; i < NODE_MAC_LEN; i++) {
        char *end;
        unsigned long byte = strtoul(text, &end, 16);
        if (end == text || byte > 0xFF || (i < NODE_MAC_LEN - 1 && *end != ':')) return false;
        mac[i] = (uint8_t)byte;
        text = end + 1;
    }
    return true;
}

static void format_mac(const uint8_t mac[NODE_MAC_LEN], char out[18]) {
    snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4],
             mac[5]);
}

static void announce_init(node *n, const node_io *io) {
    const huddle_robot *r = &n->robot;
    sayf(io, "INIT %d %d left=%d right=%d growth=%d", r->position.p, r->position.q,
         r->nucleus_left, r->nucleus_right, r->growth);
}

static void add_cell(node *n, int p, int q, const node_io *io) {
    int col = p - n->shape.p_min, row = q - n->shape.q_min;
    if (col < 0 || row < 0 || col >= NODE_WINDOW || row >= NODE_WINDOW) {
        sayf(io, "ERR cell %d %d outside the %dx%d window", p, q, NODE_WINDOW, NODE_WINDOW);
        return;
    }
    unsigned bit = (unsigned)row * NODE_WINDOW + (unsigned)col;
    if (!(n->shape_bits[bit >> 3] & (1u << (bit & 7)))) n->shape_cells++;
    n->shape_bits[bit >> 3] |= (uint8_t)(1u << (bit & 7));
}

static void attach(node *n, int wall, const uint8_t mac[NODE_MAC_LEN]) {
    if (!(n->attached & (1u << wall))) n->view[wall] = UNKNOWN_NEIGHBOR_WORD;
    n->attached |= (uint8_t)(1u << wall);
    memcpy(n->peer[wall], mac, NODE_MAC_LEN);
    if (n->initialized) huddle_mark_occupied(&n->robot, wall);
}

void node_command(node *n, const char *line, const node_io *io) {
    int p, q, wall;
    char mac_text[24];
    uint8_t mac[NODE_MAC_LEN];
    if (!strcmp(line, "WHO")) {
        char text[18];
        format_mac(n->self_mac, text);
        sayf(io, "MAC %s", text);
    } else if (!strcmp(line, "RESET")) {
        node_reset(n);
        say(io, "OK RESET");
    } else if (sscanf(line, "CELL %d %d", &p, &q) == 2) {
        add_cell(n, p, q, io);
    } else if (!strcmp(line, "SHAPE_DONE")) {
        sayf(io, "OK SHAPE %d", n->shape_cells);
    } else if (!strcmp(line, "ROOT")) {
        huddle_init_root(&n->robot, &n->shape, n->attached);
        n->initialized = true;
        announce_init(n, io);
    } else if (sscanf(line, "ATTACH %d %23s", &wall, mac_text) == 2 && wall >= 0 &&
               wall < HUDDLE_WALLS && parse_mac(mac_text, mac)) {
        attach(n, wall, mac);
        sayf(io, "OK ATTACH %d", wall);
    } else if (!strcmp(line, "STATE")) {
        sayf(io, "STATE init=%d p=%d q=%d attached=%d signals=%d", n->initialized,
             n->robot.position.p, n->robot.position.q, n->attached, n->signals);
    } else {
        say(io, "ERR unknown command");
    }
}

static uint16_t read_u16(const uint8_t *b) { return (uint16_t)(b[0] | b[1] << 8); }
static uint32_t read_u32(const uint8_t *b) {
    return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;
}

void node_receive(node *n, const uint8_t mac[NODE_MAC_LEN], const uint8_t *data, int len,
                  const node_io *io) {
    if (len != NODE_PAYLOAD_LEN) return;
    uint16_t status = read_u16(data);
    uint32_t pose = read_u32(data + 2);
    huddle_cell sender;
    int sender_wall;
    if (!huddle_decode_pose(pose, &sender, &sender_wall)) {
        say(io, "ERR malformed pose word");
        return;
    }
    int wall = (sender_wall + 3) % HUDDLE_WALLS; /* our wall facing the sender */
    if (!n->initialized) {
        attach(n, wall, mac);
        if (!huddle_init_from_neighbor(&n->robot, &n->shape, pose, n->attached)) return;
        n->initialized = true;
        announce_init(n, io);
    } else if (sender.p + HUDDLE_UNIT[sender_wall].p != n->robot.position.p ||
               sender.q + HUDDLE_UNIT[sender_wall].q != n->robot.position.q) {
        sayf(io, "ERR neighbor at %d %d via wall %d does not border us", sender.p, sender.q,
             sender_wall);
        return;
    } else if (!(n->attached & (1u << wall))) {
        attach(n, wall, mac); /* its frame beat the arena's ATTACH line */
    }
    n->view[wall] = status;
}

uint8_t node_tick(node *n, const node_io *io) {
    if (!n->initialized) return 0;
    uint16_t status = huddle_status_word(&n->robot);
    for (int w = 0; w < HUDDLE_WALLS; w++) {
        uint32_t pose;
        if (!(n->attached & (1u << w)) || !huddle_pose_word(n->robot.position, w, &pose)) continue;
        uint8_t payload[NODE_PAYLOAD_LEN] = {(uint8_t)status, (uint8_t)(status >> 8),
                                             (uint8_t)pose, (uint8_t)(pose >> 8),
                                             (uint8_t)(pose >> 16), (uint8_t)(pose >> 24)};
        io->send(io->ctx, n->peer[w], payload);
    }
    uint8_t signals = huddle_signal_walls(&n->robot, n->view);
    if (signals != n->signals) {
        n->signals = signals;
        sayf(io, "SIG %d %d %d", signals, n->robot.position.p, n->robot.position.q);
    }
    return signals;
}
