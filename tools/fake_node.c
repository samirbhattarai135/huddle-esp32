/*
 * Host stand-in for one ESP32 board, for tools/arena.py --fake. Runs the same
 * firmware/huddle_node/node.c. Serial lines come in on stdin; radio frames
 * come in as "RX <mac> <12 hex digits>" and go out as "TX <mac> <hex>".
 * "TICK" runs one Signal Activation step and answers "TICKED".
 */
#include <stdio.h>
#include <string.h>

#include "node.h"

static void print_mac(const uint8_t mac[NODE_MAC_LEN]) {
    printf("%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void send_frame(void *ctx, const uint8_t mac[NODE_MAC_LEN], const uint8_t payload[NODE_PAYLOAD_LEN]) {
    (void)ctx;
    printf("TX ");
    print_mac(mac);
    printf(" ");
    for (int i = 0; i < NODE_PAYLOAD_LEN; i++) printf("%02x", payload[i]);
    printf("\n");
}

static void log_line(void *ctx, const char *line) {
    (void)ctx;
    printf("%s\n", line);
}

static int parse_hex_bytes(const char *text, char separator, uint8_t *out, int count) {
    for (int i = 0; i < count; i++) {
        unsigned byte;
        if (sscanf(text, "%2x", &byte) != 1) return 0;
        out[i] = (uint8_t)byte;
        text += 2;
        if (separator && i < count - 1 && *text++ != separator) return 0;
    }
    return 1;
}

int main(int argc, char **argv) {
    static node robot;
    if (argc != 2 || !parse_hex_bytes(argv[1], ':', robot.self_mac, NODE_MAC_LEN)) {
        fprintf(stderr, "usage: fake_node aa:bb:cc:dd:ee:ff\n");
        return 2;
    }
    node_reset(&robot);
    const node_io io = {send_frame, log_line, NULL};
    char line[128];
    while (fgets(line, sizeof line, stdin)) {
        line[strcspn(line, "\r\n")] = '\0';
        uint8_t mac[NODE_MAC_LEN], payload[NODE_PAYLOAD_LEN];
        if (!strncmp(line, "RX ", 3)) {
            if (strlen(line) == 3 + 17 + 1 + 2 * NODE_PAYLOAD_LEN &&
                parse_hex_bytes(line + 3, ':', mac, NODE_MAC_LEN) &&
                parse_hex_bytes(line + 21, 0, payload, NODE_PAYLOAD_LEN)) {
                node_receive(&robot, mac, payload, NODE_PAYLOAD_LEN, &io);
            } else {
                printf("ERR bad RX line\n");
            }
        } else if (!strcmp(line, "TICK")) {
            node_tick(&robot, &io);
            printf("TICKED\n");
        } else {
            node_command(&robot, line, &io);
        }
        fflush(stdout);
    }
    return 0;
}
