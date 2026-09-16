/* Isolated host-side checks of functions extracted verbatim from the upload.
 * This is NOT ESP32 firmware and does not exercise NimBLE or real hardware.
 * Build: gcc -std=c11 -Wall -Wextra -fsanitize=address,undefined \
 *          sr900_review_tests.c -o sr900_review_tests
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define HEAT_SET 1
#define FAN_SET 2
#define STOP_ROAST 25
#define COOL_DN 24
#define FRAME_LEN 34
#define STX 0x20
#define ETX_LO 0x30
#define ETX_HI 0x03
#define CHECKSUM_POS 31
static char action[160];
static void send_value_command(uint8_t type, uint8_t value) {
    snprintf(action, sizeof(action), "%s(%u)", type == HEAT_SET ? "HEAT" : "FAN", value);
}
static void send_start_roast(uint8_t rt, uint8_t ct, uint8_t heat, uint8_t fan) {
    snprintf(action, sizeof(action), "START(roast=%u,cool=%u,heat=%u,fan=%u)", rt,ct,heat,fan);
}
static void send_simple_command(uint8_t type) {
    snprintf(action, sizeof(action), "%s", type == STOP_ROAST ? "STOP" : "COOL");
}
static void compute_command_token(const uint8_t *mac, const uint8_t *rnd, uint8_t *out)
{
    /* out[i] = (factor(mac[5-i]) * rnd[i]) & 0xFF
     * factor(x) = x+1 if x is 0 or a power of two, else x */
    const uint8_t mac_idx[4] = {5, 4, 3, 2};
    for (int i = 0; i < 4; i++) {
        uint8_t mb = mac[mac_idx[i]];
        bool is_bump = (mb == 0) || ((mb & (mb - 1)) == 0); /* 0 or power of two */
        uint8_t factor = is_bump ? (uint8_t)(mb + 1) : mb;
        out[i] = (uint8_t)((uint32_t)factor * rnd[i] & 0xFF);
    }
}

static void new_frame(uint8_t *b)
{
    memset(b, 0, FRAME_LEN);
    b[0] = STX;
    b[32] = ETX_LO;
    b[33] = ETX_HI;
}

static void finalize_frame(uint8_t *b)
{
    uint32_t sum = 0;
    for (int i = 1; i < 31; i++) sum += b[i];
    b[CHECKSUM_POS] = (uint8_t)(sum & 0xFF);
}

static void handle_hibean_command(const char *cmd, uint16_t len)
{
    char buf[64];
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    memcpy(buf, cmd, len);
    buf[len] = '\0';
    ESP_LOGI(TAG, "HiBean command: %s", buf);

    if (strncmp(buf, "HEAT;", 5) == 0) {
        int v = atoi(buf + 5);
       if (v < 0) { v = 0; }
if (v > 9) { v = 9; }
        send_value_command(HEAT_SET, (uint8_t)v);
    } else if (strncmp(buf, "FAN;", 4) == 0) {
        int v = atoi(buf + 4);
        if (v < 0) { v = 0; }
    if (v > 9) { v = 9; }
        send_value_command(FAN_SET, (uint8_t)v);
    } else if (strncmp(buf, "START", 5) == 0) {
        /* defaults: 10 min roast, 4 min cool, heat 5, fan 5 — adjust as needed */
        send_start_roast(10, 4, 5, 5);
    } else if (strncmp(buf, "STOP", 4) == 0) {
        send_simple_command(STOP_ROAST);
    } else if (strncmp(buf, "COOL", 4) == 0) {
        send_simple_command(COOL_DN);
    } else {
        ESP_LOGW(TAG, "Unknown command: %s", buf);
    }
}
static uint8_t reference_factor(uint8_t mb) {
    /* Literal bump set from the referenced artisan sr900-support driver,
     * retrieved September 15, 2026: {0,2,4,8,16,32,64,128}; excludes 1. */
    switch (mb) {
        case 0: case 2: case 4: case 8: case 16: case 32: case 64: case 128:
            return (uint8_t)(mb + 1);
        default: return mb;
    }
}
int main(void) {
    const struct { const char *input; const char *observed; } cases[] = {
        {"HEAT;5", "HEAT(5)"},
        {"HEAT;abc", "HEAT(0)"},
        {"HEAT;", "HEAT(0)"},
        {"HEAT;3xyz", "HEAT(3)"},
        {"HEAT;100", "HEAT(9)"},
        {"FAN;abc", "FAN(0)"},
        {"FAN;5\nHEAT;9", "FAN(5)"},
        {"STARTjunk", "START(roast=10,cool=4,heat=5,fan=5)"},
        {"START\nSTOP", "START(roast=10,cool=4,heat=5,fan=5)"},
        {"STOPPING", "STOP"},
        {"COOLING", "COOL"},
        {"UNKNOWN", "NO COMMAND"},
    };
    puts("Command parser: actual behavior of unmodified uploaded function");
    for (size_t i = 0; i < sizeof(cases)/sizeof(cases[0]); ++i) {
        strcpy(action, "NO COMMAND");
        handle_hibean_command(cases[i].input, (uint16_t)strlen(cases[i].input));
        printf("case %2zu: %s\n", i + 1, action);
        assert(strcmp(action, cases[i].observed) == 0);
    }
    char longcmd[100];
    memset(longcmd, 'x', sizeof(longcmd));
    memcpy(longcmd, "START", 5);
    handle_hibean_command(longcmd, sizeof(longcmd));
    assert(strcmp(action, "START(roast=10,cool=4,heat=5,fan=5)") == 0);
    puts("Oversized 100-byte START-prefixed input is also accepted as START.");
    unsigned differences = 0;
    for (unsigned mb = 0; mb <= 255; ++mb) {
        for (unsigned r = 0; r <= 255; ++r) {
            uint8_t mac[6], rnd[4], token[4];
            memset(mac, mb, sizeof(mac));
            memset(rnd, r, sizeof(rnd));
            compute_command_token(mac, rnd, token);
            uint8_t expected = (uint8_t)(reference_factor((uint8_t)mb) * r);
            if (token[0] != expected) {
                assert(mb == 1 && r != 0);
                ++differences;
            }
        }
    }
    assert(differences == 255);
    printf("Token comparison: %u mismatches in 65536 byte/random combinations;\n", differences);
    puts("all mismatches are MAC byte 0x01 with nonzero random byte.");
    uint8_t mac[6] = {0,0,0,0,0,1}, rnd[4] = {7,0,0,0}, token[4];
    compute_command_token(mac, rnd, token);
    printf("Example: MAC[5]=1,RND[0]=7: upload=%u, reference=%u\n", token[0], 7u);
    uint8_t frame[FRAME_LEN];
    new_frame(frame);
    assert(frame[0] == STX && frame[32] == ETX_LO && frame[33] == ETX_HI);
    for (int i = 1; i <= 30; ++i) frame[i] = (uint8_t)i;
    finalize_frame(frame);
    assert(frame[31] == 209);
    puts("Frame helper check: sum(bytes 1..30) modulo 256 = 209, correct.");
    puts("PASS: all assertions about current behavior; no sanitizer errors.");
    return 0;
}
