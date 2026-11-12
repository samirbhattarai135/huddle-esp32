// One ESP32 = one in-assembly Huddle robot.
//
// Neighbor link: ESP-NOW unicast, one 18-byte frame per attached neighbor per
// step: the paper's two integers (16-bit wall-status word, 32-bit position +
// wall unit vector), a 32-bit sequence number and a 64-bit SipHash-2-4 tag
// (layout in node.h and the README). The 128-bit tag key is a session key that
// tools/arena.py sends as "KEY <hex>" over USB serial; it lives in RAM only and
// is never written to NVS or flash. Frames are neither sent nor accepted until
// it arrives. ESP-NOW's own encryption is not used.
//
// Physical docking is emulated: tools/arena.py tells the board over USB serial
// which wall a neighbor attached to and that neighbor's MAC address. Signals
// are reported as "SIG <mask> <p> <q>" lines and on the LED.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "node.h"

// Hardware knobs. Most ESP32 DevKit v1 boards have a blue LED on GPIO 2.
static const int STATUS_LED_PIN = 2;
static const uint32_t STEP_MS = 100;   // Signal Activation period
static const uint8_t WIFI_CHANNEL = 1; // every board must share it

struct RadioFrame {
    uint8_t mac[NODE_MAC_LEN];
    uint8_t data[NODE_FRAME_LEN];
    int len;
};

static node robot;
static QueueHandle_t rx_queue;
static char line[96];
static size_t line_len;
static uint32_t last_step_ms;

static void log_line(void *, const char *text) { Serial.println(text); }

static void send_frame(void *, const uint8_t mac[NODE_MAC_LEN], const uint8_t frame[NODE_FRAME_LEN]) {
    if (!esp_now_is_peer_exist(mac)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, mac, NODE_MAC_LEN);
        peer.channel = 0; // current channel
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) != ESP_OK) {
            Serial.println("ERR esp_now_add_peer failed");
            return;
        }
    }
    esp_now_send(mac, frame, NODE_FRAME_LEN);
}

static const node_io io = {send_frame, log_line, nullptr};

// Runs in the Wi-Fi task: hand the frame to loop() instead of touching state here.
static void on_receive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    RadioFrame frame = {};
    memcpy(frame.mac, info->src_addr, NODE_MAC_LEN);
    frame.len = len;
    memcpy(frame.data, data, len < NODE_FRAME_LEN ? len : NODE_FRAME_LEN);
    xQueueSend(rx_queue, &frame, 0); // drop the frame if the queue is full
}

static void update_led(uint32_t now_ms) {
    // Off: not in the assembly. Solid: signaling on at least one wall.
    // Slow blink: in the assembly, nothing to signal this step.
    bool on = robot.initialized && (robot.signals || (now_ms / 500) % 2 == 0);
    digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
}

void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED_PIN, OUTPUT);
    rx_queue = xQueueCreate(32, sizeof(RadioFrame));

    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ERR esp_now_init failed");
    }
    esp_now_register_recv_cb(on_receive);

    esp_wifi_get_mac(WIFI_IF_STA, robot.self_mac);
    node_reset(&robot);
    node_command(&robot, "WHO", &io);
}

void loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c != '\n') {
            if (line_len < sizeof line - 1) line[line_len++] = c;
            continue;
        }
        line[line_len] = '\0';
        if (line_len) node_command(&robot, line, &io);
        line_len = 0;
    }

    RadioFrame frame;
    while (xQueueReceive(rx_queue, &frame, 0) == pdTRUE) {
        node_receive(&robot, frame.mac, frame.data, frame.len, &io);
    }

    uint32_t now_ms = millis();
    if (now_ms - last_step_ms >= STEP_MS) {
        last_step_ms = now_ms;
        node_tick(&robot, &io);
    }
    update_led(now_ms);
}
