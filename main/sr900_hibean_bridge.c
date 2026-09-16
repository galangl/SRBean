/* SPDX-License-Identifier: AGPL-3.0-or-later
 * ESP32-S3 SR900 <-> HiBean Custom TC4 bridge
 * Revision: feedback-sync-3
 *
 * SR900 protocol based on brianschmitt/artisan, sr900-support,
 * src/artisanlib/sr900.py.
 * Protocol reference copyright (C) 2010-2026 The Artisan team represented by
 * Marko Luther and all contributors; AGPL-3.0-or-later.
 * Distributed without warranty; see https://www.gnu.org/licenses/agpl-3.0.html.
 * This bridge revision has not been tested on physical hardware.
 *
 * ESP-IDF NimBLE: central + peripheral, observer + broadcaster enabled;
 * CONFIG_BT_NIMBLE_MAX_CONNECTIONS >= 2; configure preferred ATT MTU >= 64.
 * The negotiated SR900 MTU must be >= 37 for an unfragmented 34-byte write.
 * Uses bt, nvs_flash and esp_timer components.
 *
 * HiBean device: ESP32-SR900-Bridge
 * Service: 7a9e0001-1234-4a5e-8b3d-9f1e2c3a4b50
 * Write:   7a9e0003-1234-4a5e-8b3d-9f1e2c3a4b50
 * Notify:  7a9e0002-1234-4a5e-8b3d-9f1e2c3a4b50
 * Commands: READ, HEAT;0..9, FAN;0..9, START, STOP, COOL.
 * One complete ASCII command per BLE write; optional LF, CR or CRLF.
 * READ response: BT,ET,FAN,HEATER\n (integer temperatures in Fahrenheit).
 * Poll with READ every 2000 ms. Status notifications are request-driven.
 *
 * A read following HEAT/FAN waits for both ATT write acknowledgment and
 * subsequent matching SR900 telemetry. No fabricated feedback. A 5-second
 * timeout fails the phone session; it does NOT stop or cool the roaster.
 * START/STOP/COOL reads wait for acknowledgment + subsequent telemetry;
 * that telemetry is NOT proof the requested operating state was reached.
 * Normal commands are serialized: busy requests return an ATT error.
 * HEAT;0 / STOP / COOL may supersede a pending request; an already-submitted
 * write cannot be recalled. Nothing is replayed after a disconnection.
 *
 * START remains explicit: roast 10 min, cool 4 min, heat 5, fan 5.
 * No automatic START, arming, PID, or disconnect-triggered cooling.
 * Phone loss cannot guarantee a safe machine state; supervise the roaster.
 * Keep controls disabled until hardware validation of reads and restores.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#ifdef SR900_BRIDGE_HOST_TEST
#include "bridge_test_stubs.h"
#else
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nimble/nimble_npl.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_att.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#endif

static const char *TAG = "sr900_bridge";
#define ROASTER_DEVICE_NAME "SR900-FDB666"

static const ble_uuid16_t sr900_service_uuid = BLE_UUID16_INIT(0xDF00);
static const ble_uuid16_t sr900_notify_char_uuid = BLE_UUID16_INIT(0xDF01);
static const ble_uuid16_t sr900_write_char_uuid  = BLE_UUID16_INIT(0xDF02);

#define FRAME_LEN 34
#define STX 0x20
#define ETX_LO 0x30
#define ETX_HI 0x03
#define CHECKSUM_POS 31

#define HEAT_SET        1
#define FAN_SET         2
#define START_ROAST     21
#define COOL_DN         24
#define STOP_ROAST      25
#define MAC_ADDRESS_REQ 38
#define SETTINGS        43

#define RES_SETTINGS_ACK      28
#define RES_ROASTER_STATUS    33
#define RES_ROASTER_STARTED   34
#define RES_COOLER_STARTED    35
#define RES_ROASTER_FINISHED  36
#define RES_MAC_ADDRESS       39

static const uint8_t RESERVED[4] = {0x53, 0x45, 0x51, 0x4F}; /* "SEQO" */

/* -------------------------------------------------------------------- */
/* Our own simple GATT service for HiBean (Custom TC4)                  */
/* -------------------------------------------------------------------- */
static const ble_uuid128_t hb_service_uuid =
    BLE_UUID128_INIT(0x50,0x4b,0x3a,0x2c,0x1e,0x9f,0x3d,0x8b,
                      0x5e,0x4a,0x34,0x12,0x01,0x00,0x9e,0x7a);
static const ble_uuid128_t hb_status_char_uuid =
    BLE_UUID128_INIT(0x50,0x4b,0x3a,0x2c,0x1e,0x9f,0x3d,0x8b,
                      0x5e,0x4a,0x34,0x12,0x02,0x00,0x9e,0x7a);
static const ble_uuid128_t hb_command_char_uuid =
    BLE_UUID128_INIT(0x50,0x4b,0x3a,0x2c,0x1e,0x9f,0x3d,0x8b,
                      0x5e,0x4a,0x34,0x12,0x03,0x00,0x9e,0x7a);


#define FEEDBACK_TIMEOUT_MS 5000
#define READ_TIMEOUT_MS 5000
#define STATUS_MAX_AGE_MS 5000
#define SETUP_TIMEOUT_MS 20000
#define TX_QUEUE_CAP 4
#define ATT_UNAVAILABLE BLE_ATT_ERR_UNLIKELY

enum link_phase {
    LINK_DOWN, LINK_CONNECTING, LINK_MTU, LINK_DISCOVERY,
    LINK_SUBSCRIBE, LINK_MAC, LINK_SETTINGS, LINK_READY, LINK_FAULT
};
static enum link_phase phase = LINK_DOWN;
static uint16_t client_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t hibean_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t hb_status_val_handle;
static uint16_t roaster_write_val_handle, roaster_notify_val_handle;
static uint16_t roaster_notify_cccd_handle, svc_start, svc_end, notify_end;
static uint8_t write_properties, notify_properties, own_addr_type;
static uint8_t roaster_mac[6], rnd_bytes[4], command_token[4];
static bool have_token, settings_acked, settings_write_acked;
static bool hb_subscribed, hb_closing, read_waiting, read_fault, synced;
static bool have_status, mtu_wait_existing;
static int last_bt = -1, last_et = -1, last_fan = -1, last_heater = -1;
static int64_t last_status_ms, read_deadline, setup_deadline, retry_at;
static uint32_t status_seq, session_epoch, transaction_seq, command_seq;
static unsigned retry_count;
static struct ble_npl_callout maintenance_callout;

struct tx_item {
    uint8_t frame[FRAME_LEN];
    uint32_t command_id; /* 0 for handshake frames */
};
static struct tx_item tx_queue[TX_QUEUE_CAP], tx_active;
static unsigned tx_count;
static bool tx_busy;
static uint32_t tx_serial;
static int64_t tx_deadline;

struct pending_control {
    bool active, sent, acknowledged;
    uint8_t type, value;
    uint32_t id, sent_status_seq;
    int64_t admitted_ms, sent_ms, deadline;
};
static struct pending_control pending;

static void ble_app_scan(void);
static void ble_app_advertise(void);
static void pump_tx(void);
static void fail_link(const char *why, int rc);
static void fail_phone(const char *why);
static void try_status_response(void);
static void check_control_feedback(void);
static void send_settings(void);
static void process_roaster_frame(const uint8_t *d, uint16_t len);
static int gap_client_event_cb(struct ble_gap_event *event, void *arg);

static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

static bool live_callback(uint16_t conn, void *arg)
{
    return conn == client_conn_handle &&
           (uint32_t)(uintptr_t)arg == session_epoch && phase != LINK_FAULT;
}

static void set_phase(enum link_phase next)
{
    phase = next;
    setup_deadline = now_ms() + SETUP_TIMEOUT_MS;
}

static bool status_fresh(void)
{
    return have_status && phase == LINK_READY &&
           now_ms() - last_status_ms <= STATUS_MAX_AGE_MS;
}

static void new_frame(uint8_t *b)
{
    memset(b, 0, FRAME_LEN);
    b[0] = STX; b[32] = ETX_LO; b[33] = ETX_HI;
}

static void fill_random(uint8_t *b, int start, int end)
{
    for (int i = start; i <= end; ++i) b[i] = (uint8_t)esp_random();
}

static void finalize_frame(uint8_t *b)
{
    unsigned sum = 0;
    for (int i = 1; i < CHECKSUM_POS; ++i) sum += b[i];
    b[CHECKSUM_POS] = (uint8_t)sum;
}

static void compute_command_token(const uint8_t *mac, const uint8_t *rnd,
                                  uint8_t *out)
{
    for (int i = 0; i < 4; ++i) {
        uint8_t mb = mac[5-i];
        bool bump = mb == 0 || (mb >= 2 && (mb & (mb-1)) == 0);
        out[i] = (uint8_t)((bump ? mb+1 : mb) * rnd[i]);
    }
}

static void clear_roaster_state(void)
{
    ++session_epoch; ++transaction_seq;
    mtu_wait_existing = false;
    tx_busy = false; tx_count = 0;
    memset(&pending, 0, sizeof(pending));
    have_token = settings_acked = settings_write_acked = false;
    have_status = false; status_seq = 0;
    last_bt = last_et = last_fan = last_heater = -1;
    roaster_write_val_handle = roaster_notify_val_handle = 0;
    roaster_notify_cccd_handle = svc_start = svc_end = notify_end = 0;
    write_properties = notify_properties = 0;
}

static void schedule_scan(void)
{
    phase = LINK_DOWN;
    unsigned shift = retry_count < 4 ? retry_count : 4;
    int delay = 500 * (1 << shift);
    ++retry_count;
    retry_at = now_ms() + delay;
    ESP_LOGI(TAG, "SR900 retry in %d ms", delay);
}

static void fail_phone(const char *why)
{
    read_fault = true;
    read_waiting = false;
    /* Do this immediately, not only when the asynchronous GAP event arrives. */
    if (phase == LINK_READY) {
        tx_count = 0;
        if (pending.active && !pending.sent) pending.active = false;
    }
    ESP_LOGE(TAG, "HiBean session failed: %s; machine state is not guaranteed", why);
    if (hibean_conn_handle != BLE_HS_CONN_HANDLE_NONE && !hb_closing) {
        hb_closing = true;
        int rc = ble_gap_terminate(hibean_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        if (rc) ESP_LOGE(TAG, "HiBean terminate rc=%d", rc);
    }
}

static void fail_link(const char *why, int rc)
{
    ESP_LOGE(TAG, "SR900 failure: %s rc=%d", why, rc);
    uint16_t conn = client_conn_handle;
    clear_roaster_state();
    phase = LINK_FAULT;
    fail_phone(why);
    if (conn != BLE_HS_CONN_HANDLE_NONE) {
        int err = ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        if (err == BLE_HS_ENOTCONN) {
            client_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            schedule_scan();
        } else if (err) {
            ESP_LOGE(TAG, "SR900 terminate rc=%d; awaiting disconnect", err);
        }
    } else {
        schedule_scan();
    }
}

static void maybe_ready(void)
{
    if (phase == LINK_SETTINGS && settings_acked && settings_write_acked) {
        set_phase(LINK_READY);
        retry_count = 0;
        ESP_LOGI(TAG, "SR900 READY: settings and ATT write acknowledged");
        try_status_response();
    }
}

static int tx_done(uint16_t conn, const struct ble_gatt_error *error,
                   struct ble_gatt_attr *attr, void *arg)
{
    (void)attr;
    if (!tx_busy || conn != client_conn_handle ||
        (uint32_t)(uintptr_t)arg != tx_serial) return 0;
    uint8_t type = tx_active.frame[6];
    uint32_t id = tx_active.command_id;
    tx_busy = false;
    ESP_LOGI(TAG, "TX complete type=%u id=%" PRIu32 " ATT status=%d",
             type, id, error->status);
    if (error->status) {
        fail_link("acknowledged write failed", error->status);
        return 0;
    }
    if (type == SETTINGS) {
        settings_write_acked = true;
        maybe_ready();
    }
    if (pending.active && pending.id == id) {
        pending.acknowledged = true;
        check_control_feedback();
    }
    pump_tx();
    return 0;
}

static void pump_tx(void)
{
    if (tx_busy || !tx_count || phase == LINK_FAULT) return;
    if (client_conn_handle == BLE_HS_CONN_HANDLE_NONE ||
        !roaster_write_val_handle || ble_att_mtu(client_conn_handle) < FRAME_LEN+3) {
        fail_link("TX requires connected SR900 and MTU >= 37", BLE_HS_EINVAL);
        return;
    }
    tx_active = tx_queue[0];
    --tx_count;
    memmove(tx_queue, tx_queue+1, tx_count * sizeof(tx_queue[0]));
    tx_busy = true;
    tx_serial = ++transaction_seq;
    tx_deadline = now_ms() + FEEDBACK_TIMEOUT_MS;
    if (pending.active && pending.id == tx_active.command_id) {
        pending.sent = true;
        pending.sent_ms = now_ms();
        pending.sent_status_seq = status_seq;
    }
    ESP_LOGI(TAG, "TX acknowledged write type=%u id=%" PRIu32 " len=%d MTU=%u",
             tx_active.frame[6], tx_active.command_id, FRAME_LEN,
             ble_att_mtu(client_conn_handle));
    int rc = ble_gattc_write_flat(client_conn_handle, roaster_write_val_handle,
                                 tx_active.frame, FRAME_LEN, tx_done,
                                 (void *)(uintptr_t)tx_serial);
    if (rc) {
        tx_busy = false;
        fail_link("write initiation failed", rc);
    }
}

static int enqueue_frame(const uint8_t *frame, uint32_t id)
{
    if (tx_count >= TX_QUEUE_CAP || phase == LINK_FAULT) return BLE_HS_EBUSY;
    memcpy(tx_queue[tx_count].frame, frame, FRAME_LEN);
    tx_queue[tx_count++].command_id = id;
    pump_tx();
    return phase == LINK_FAULT || phase == LINK_DOWN ? BLE_HS_ENOTCONN : 0;
}

static void send_mac_request(void)
{
    uint8_t frame[FRAME_LEN];
    new_frame(frame);
    memcpy(frame+1, RESERVED, 4);
    frame[6] = MAC_ADDRESS_REQ;
    fill_random(frame, 7, 30);
    memcpy(rnd_bytes, frame+7, 4);
    finalize_frame(frame);
    set_phase(LINK_MAC);
    int rc = enqueue_frame(frame, 0);
    if (rc && phase != LINK_FAULT && phase != LINK_DOWN) fail_link("MAC queue", rc);
}

static void send_settings(void)
{
    uint8_t frame[FRAME_LEN];
    new_frame(frame);
    memcpy(frame+1, command_token, 4);
    frame[5] = 64+2; /* External probe, original 113-118 V setting. */
    frame[6] = SETTINGS;
    memcpy(frame+7, roaster_mac, 6);
    fill_random(frame, 13, 30);
    finalize_frame(frame);
    set_phase(LINK_SETTINGS);
    int rc = enqueue_frame(frame, 0);
    if (rc && phase != LINK_FAULT && phase != LINK_DOWN) fail_link("settings queue", rc);
}

/* Pure builder: used by host regression tests as well as the live sender. */
static void build_control_frame(uint8_t *frame, uint8_t type, uint8_t value)
{
    new_frame(frame);
    memcpy(frame+1, command_token, 4);
    frame[6] = type;
    memcpy(frame+7, roaster_mac, 6);
    if (type == HEAT_SET || type == FAN_SET) {
        frame[13] = value;
        if (type == HEAT_SET) {
            frame[14] = 0; /* agenticRoast: manual control */
            fill_random(frame, 15, 30);
        } else fill_random(frame, 14, 30);
    } else if (type == START_ROAST) {
        frame[13] = 10; frame[14] = 4; frame[15] = 5; frame[16] = 5;
        frame[17] = 0; frame[18] = 3;
        fill_random(frame, 19, 30);
    } else fill_random(frame, 13, 30);
    finalize_frame(frame);
}

static int submit_control(uint8_t type, uint8_t value)
{
    bool recovery = type == STOP_ROAST || type == COOL_DN ||
                    (type == HEAT_SET && value == 0);
    if (phase != LINK_READY || !have_token || hb_closing ||
        (!recovery && (read_fault || !status_fresh()))) {
        ESP_LOGW(TAG, "Control rejected: link not ready or telemetry unavailable");
        return ATT_UNAVAILABLE;
    }
    if ((pending.active || tx_busy || tx_count) && !recovery) {
        ESP_LOGW(TAG, "Control rejected: previous transaction still pending");
        return ATT_UNAVAILABLE;
    }
    if (pending.active || tx_count) {
        ESP_LOGW(TAG, "Recovery command supersedes pending control; in-flight write cannot be recalled");
        tx_count = 0;
    }
    pending = (struct pending_control) {
        .active = true, .type = type, .value = value,
        .id = ++command_seq, .admitted_ms = now_ms(),
        .deadline = now_ms() + FEEDBACK_TIMEOUT_MS
    };
    uint8_t frame[FRAME_LEN];
    build_control_frame(frame, type, value);
    ESP_LOGI(TAG, "CONTROL admitted type=%u value=%u id=%" PRIu32,
             type, value, pending.id);
    if (enqueue_frame(frame, pending.id)) {
        fail_phone("control was not queued");
        pending.active = false;
        return ATT_UNAVAILABLE;
    }
    return 0; /* Accepted by bridge; execution requires telemetry confirmation. */
}

static void check_control_feedback(void)
{
    if (!pending.active || !pending.sent || !pending.acknowledged ||
        !have_status || status_seq == pending.sent_status_seq) return;
    /* Deadline must be checked even if a late status/callback beats the timer. */
    if (now_ms() >= pending.deadline) return;
    bool match = pending.type == HEAT_SET ? last_heater == pending.value :
                 pending.type == FAN_SET ? last_fan == pending.value : true;
    if (!match) {
        ESP_LOGI(TAG, "WAIT feedback id=%" PRIu32 " wanted=%u fan=%d heater=%d",
                 pending.id, pending.value, last_fan, last_heater);
        return;
    }
    ESP_LOGI(TAG, "%s id=%" PRIu32 " elapsed=%" PRId64 "ms fan=%d heater=%d",
             pending.type == HEAT_SET || pending.type == FAN_SET ?
             "CONTROL confirmed" : "Post-command status received (not state confirmation)",
             pending.id, now_ms()-pending.admitted_ms, last_fan, last_heater);
    pending.active = false;
    try_status_response();
}

static void try_status_response(void)
{
    if (!read_waiting || pending.active || read_fault || hb_closing ||
        !hb_subscribed || hibean_conn_handle == BLE_HS_CONN_HANDLE_NONE ||
        !status_fresh()) return;
    if (now_ms() >= read_deadline) {
        fail_phone("READ deadline expired");
        return;
    }
    char buf[48];
    int len = snprintf(buf, sizeof(buf), "%d,%d,%d,%d\n",
                       last_bt, last_et, last_fan, last_heater);
    if (len <= 0 || len >= (int)sizeof(buf) ||
        ble_att_mtu(hibean_conn_handle) < len+3) {
        fail_phone("status frame exceeds notification MTU");
        return;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
    if (!om) { fail_phone("status allocation failed"); return; }
    int rc = ble_gatts_notify_custom(hibean_conn_handle, hb_status_val_handle, om);
    if (rc) { fail_phone("status notification submission failed"); return; }
    read_waiting = false;
    ESP_LOGI(TAG, "READ response queued: %.*s sample_age=%" PRId64 "ms",
             len-1, buf, now_ms()-last_status_ms);
}

/* Strict, pure parser. Reject prefixes, embedded NUL, multiline, decimals, and
 * oversized input instead of silently turning malformed input into commands. */
enum command_kind { CMD_INVALID, CMD_READ, CMD_HEAT, CMD_FAN,
                    CMD_START, CMD_STOP, CMD_COOL };
static enum command_kind parse_command(const char *data, size_t len, uint8_t *value)
{
    if (!len || len >= 64 || memchr(data, 0, len)) return CMD_INVALID;
    if (len && data[len-1] == '\n') --len;
    if (len && data[len-1] == '\r') --len;
    if (!len || memchr(data, '\n', len) || memchr(data, '\r', len)) return CMD_INVALID;
    if (len == 4 && !memcmp(data, "READ", 4)) return CMD_READ;
    if (len == 5 && !memcmp(data, "START", 5)) return CMD_START;
    if (len == 4 && !memcmp(data, "STOP", 4)) return CMD_STOP;
    if (len == 4 && !memcmp(data, "COOL", 4)) return CMD_COOL;
    size_t prefix = len == 6 && !memcmp(data, "HEAT;", 5) ? 5 :
                    len == 5 && !memcmp(data, "FAN;", 4) ? 4 : 0;
    if (!prefix || data[prefix] < '0' || data[prefix] > '9') return CMD_INVALID;
    *value = (uint8_t)(data[prefix]-'0');
    return prefix == 5 ? CMD_HEAT : CMD_FAN;
}

static int handle_hibean_command(const char *data, uint16_t len)
{
    uint8_t value = 0;
    enum command_kind cmd = parse_command(data, len, &value);
    if (cmd == CMD_INVALID) {
        ESP_LOGW(TAG, "Rejected malformed HiBean command len=%u", len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    ESP_LOGI(TAG, "HiBean command: %.*s", (int)len, data);
    if (cmd == CMD_READ) {
        if (read_fault || hb_closing || !hb_subscribed || phase != LINK_READY) {
            ESP_LOGW(TAG, "READ rejected: ready=%d subscribed=%d fault=%d",
                     phase == LINK_READY, hb_subscribed, read_fault);
            return ATT_UNAVAILABLE;
        }
        if (!read_waiting) {
            read_waiting = true;
            read_deadline = now_ms()+READ_TIMEOUT_MS;
        }
        if (pending.active)
            ESP_LOGI(TAG, "READ deferred for control id=%" PRIu32, pending.id);
        try_status_response();
        return 0;
    }
    uint8_t type = cmd == CMD_HEAT ? HEAT_SET : cmd == CMD_FAN ? FAN_SET :
                   cmd == CMD_START ? START_ROAST : cmd == CMD_STOP ? STOP_ROAST : COOL_DN;
    return submit_control(type, value);
}

static int hb_gatt_access_cb(uint16_t conn, uint16_t attr,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)attr; (void)arg;
    if (conn != hibean_conn_handle || hb_closing) return ATT_UNAVAILABLE;
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        char buf[64];
        if (!len || len >= sizeof(buf)) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        if (ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL)) return ATT_UNAVAILABLE;
        return handle_hibean_command(buf, len);
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        /* ATT reads cannot be held by this callback; use text READ + Notify. */
        if (pending.active || read_fault || !status_fresh()) return ATT_UNAVAILABLE;
        char buf[48];
        int len = snprintf(buf, sizeof(buf), "%d,%d,%d,%d\n",
                           last_bt, last_et, last_fan, last_heater);
        if (len <= 0 || len >= (int)sizeof(buf)) return ATT_UNAVAILABLE;
        return os_mbuf_append(ctxt->om, buf, len) ? BLE_ATT_ERR_INSUFFICIENT_RES : 0;
    }
    return ATT_UNAVAILABLE;
}

static const struct ble_gatt_svc_def hb_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (const ble_uuid_t *)&hb_service_uuid,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = (const ble_uuid_t *)&hb_status_char_uuid,
                .access_cb = hb_gatt_access_cb,
                .val_handle = &hb_status_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = (const ble_uuid_t *)&hb_command_char_uuid,
                .access_cb = hb_gatt_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 }
        },
    },
    { 0 }
};


static int hb_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            hibean_conn_handle = event->connect.conn_handle;
            hb_subscribed = hb_closing = read_waiting = read_fault = false;
            ESP_LOGI(TAG, "HiBean connected handle=%u", hibean_conn_handle);
        } else {
            ESP_LOGW(TAG, "HiBean connection failed status=%d", event->connect.status);
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        if (event->disconnect.conn.conn_handle != hibean_conn_handle) break;
        ESP_LOGW(TAG, "HiBean disconnected reason=%d; no automatic machine stop",
                 event->disconnect.reason);
        hibean_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        hb_subscribed = hb_closing = read_waiting = false;
        /* Discard unsent controls; never dispatch a stale START after phone loss. */
        if (phase == LINK_READY) {
            tx_count = 0;
            if (pending.active && !pending.sent) pending.active = false;
        }
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.conn_handle == hibean_conn_handle &&
            event->subscribe.attr_handle == hb_status_val_handle) {
            hb_subscribed = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "HiBean notify subscription=%d", hb_subscribed);
            if (!hb_subscribed && read_waiting) fail_phone("notification subscription removed");
        }
        break;
    case BLE_GAP_EVENT_NOTIFY_TX:
        if (event->notify_tx.conn_handle == hibean_conn_handle &&
            event->notify_tx.attr_handle == hb_status_val_handle &&
            event->notify_tx.status)
            fail_phone("notification transmission failed");
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_app_advertise();
        break;
    default: break;
    }
    return 0;
}

static void ble_app_advertise(void)
{
    if (!synced || hibean_conn_handle != BLE_HS_CONN_HANDLE_NONE ||
        ble_gap_adv_active()) return;
    struct ble_hs_adv_fields fields = {0};
    struct ble_gap_adv_params params = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&hb_service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc) { ESP_LOGE(TAG, "advertising fields rc=%d", rc); return; }
    memset(&fields, 0, sizeof(fields));
    const char *name = "ESP32-SR900-Bridge";
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&fields);
    if (rc) { ESP_LOGE(TAG, "scan response fields rc=%d", rc); return; }
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &params, hb_gap_event_cb, NULL);
    if (rc) ESP_LOGE(TAG, "advertise start rc=%d", rc);
    else ESP_LOGI(TAG, "Advertising ESP32-SR900-Bridge");
}

static int subscribe_cb(uint16_t conn, const struct ble_gatt_error *error,
                        struct ble_gatt_attr *attr, void *arg)
{
    (void)attr;
    if (!live_callback(conn, arg)) return 0;
    if (error->status) fail_link("CCCD subscription", error->status);
    else {
        ESP_LOGI(TAG, "Subscribed to SR900 notifications");
        send_mac_request();
    }
    return 0;
}

static int disc_dsc_cb(uint16_t conn, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                       void *arg)
{
    (void)chr_val_handle;
    if (!live_callback(conn, arg)) return 0;
    if (!error->status && dsc) {
        if (!ble_uuid_cmp(&dsc->uuid.u, BLE_UUID16_DECLARE(0x2902)))
            roaster_notify_cccd_handle = dsc->handle;
    } else if (error->status == BLE_HS_EDONE) {
        if (!roaster_notify_cccd_handle) { fail_link("CCCD missing", BLE_HS_ENOENT); return 0; }
        set_phase(LINK_SUBSCRIBE);
        uint8_t value[2] = {1,0};
        int rc = ble_gattc_write_flat(conn, roaster_notify_cccd_handle, value, 2,
                                      subscribe_cb, arg);
        if (rc) fail_link("CCCD write start", rc);
    } else if (error->status) fail_link("descriptor discovery", error->status);
    return 0;
}

static int disc_chr_cb(uint16_t conn, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    if (!live_callback(conn, arg)) return 0;
    if (!error->status && chr) {
        /* Characteristics are enumerated in handle order. Derive DF01's real
         * descriptor range from the next characteristic declaration. */
        if (roaster_notify_val_handle && chr->def_handle > roaster_notify_val_handle &&
            chr->def_handle-1 < notify_end) notify_end = chr->def_handle-1;
        if (!ble_uuid_cmp(&chr->uuid.u, &sr900_notify_char_uuid.u)) {
            roaster_notify_val_handle = chr->val_handle;
            notify_properties = chr->properties;
        } else if (!ble_uuid_cmp(&chr->uuid.u, &sr900_write_char_uuid.u)) {
            roaster_write_val_handle = chr->val_handle;
            write_properties = chr->properties;
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (!roaster_write_val_handle || !roaster_notify_val_handle ||
            !(write_properties & BLE_GATT_CHR_PROP_WRITE) ||
            !(notify_properties & BLE_GATT_CHR_PROP_NOTIFY) ||
            notify_end <= roaster_notify_val_handle) {
            fail_link("required characteristics/properties missing", BLE_HS_ENOENT);
            return 0;
        }
        set_phase(LINK_DISCOVERY);
        int rc = ble_gattc_disc_all_dscs(conn, roaster_notify_val_handle, notify_end,
                                        disc_dsc_cb, arg);
        if (rc) fail_link("descriptor discovery start", rc);
    } else if (error->status) fail_link("characteristic discovery", error->status);
    return 0;
}

static int disc_svc_cb(uint16_t conn, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg)
{
    if (!live_callback(conn, arg)) return 0;
    if (!error->status && svc) {
        svc_start = svc->start_handle; svc_end = svc->end_handle;
    } else if (error->status == BLE_HS_EDONE) {
        if (!svc_start) { fail_link("DF00 service missing", BLE_HS_ENOENT); return 0; }
        notify_end = svc_end;
        set_phase(LINK_DISCOVERY);
        int rc = ble_gattc_disc_all_chrs(conn, svc_start, svc_end, disc_chr_cb, arg);
        if (rc) fail_link("characteristic discovery start", rc);
    } else if (error->status) fail_link("service discovery", error->status);
    return 0;
}

/* One transition per connection, using the negotiated ATT MTU, not our
 * preferred size. Also used when NimBLE already owns the MTU procedure. */
static void advance_after_mtu(void)
{
    if (phase != LINK_MTU || client_conn_handle == BLE_HS_CONN_HANDLE_NONE ||
        ble_att_mtu(client_conn_handle) < FRAME_LEN+3) return;
    mtu_wait_existing = false;
    ESP_LOGI(TAG, "SR900 MTU ready: %u; starting service discovery",
             ble_att_mtu(client_conn_handle));
    set_phase(LINK_DISCOVERY);
    int rc = ble_gattc_disc_svc_by_uuid(client_conn_handle, &sr900_service_uuid.u,
                                       disc_svc_cb, (void *)(uintptr_t)session_epoch);
    if (rc) fail_link("service discovery start", rc);
}

static int mtu_cb(uint16_t conn, const struct ble_gatt_error *error,
                  uint16_t mtu, void *arg)
{
    if (!live_callback(conn, arg) || phase != LINK_MTU) return 0;
    ESP_LOGI(TAG, "SR900 MTU callback status=%d MTU=%u actual=%u",
             error->status, mtu, ble_att_mtu(conn));
    if (error->status || ble_att_mtu(conn) < FRAME_LEN+3) {
        fail_link("MTU exchange failed or negotiated MTU below 37",
                  error->status ? error->status : BLE_HS_EINVAL);
        return 0;
    }
    advance_after_mtu();
    return 0;
}

static void process_roaster_frame(const uint8_t *d, uint16_t len)
{
    if (phase == LINK_FAULT || len != FRAME_LEN || d[0] != STX ||
        d[32] != ETX_LO || d[33] != ETX_HI) return;
    unsigned sum = 0;
    for (int i = 1; i < CHECKSUM_POS; ++i) sum += d[i];
    if ((uint8_t)sum != d[CHECKSUM_POS]) {
        ESP_LOGW(TAG, "Invalid SR900 checksum");
        return;
    }
    uint8_t type = d[6];
    if (type == RES_MAC_ADDRESS && phase == LINK_MAC) {
        memcpy(roaster_mac, d+7, 6);
        compute_command_token(roaster_mac, rnd_bytes, command_token);
        have_token = true;
        ESP_LOGI(TAG, "MAC received; scheduling settings");
        send_settings();
    } else if (type == RES_SETTINGS_ACK && phase == LINK_SETTINGS) {
        settings_acked = true;
        ESP_LOGI(TAG, "SETTINGS ACKED");
        maybe_ready();
    } else if (type == RES_ROASTER_STATUS) {
        last_fan = d[13]; last_heater = d[14];
        last_bt = d[17]*256 + d[18];
        last_et = d[19]*256 + d[20];
        last_status_ms = now_ms();
        have_status = true;
        ++status_seq;
        ESP_LOGI(TAG, "STATUS seq=%" PRIu32 " fan=%d heater=%d BT=%dF ET=%dF",
                 status_seq, last_fan, last_heater, last_bt, last_et);
        check_control_feedback();
        try_status_response();
    } else if (type == RES_ROASTER_STARTED) {
        ESP_LOGI(TAG, "SR900 reports ROASTER STARTED");
    } else if (type == RES_COOLER_STARTED) {
        ESP_LOGI(TAG, "SR900 reports COOLER STARTED");
    } else if (type == RES_ROASTER_FINISHED) {
        ESP_LOGI(TAG, "SR900 reports ROASTER FINISHED");
    }
}

static int gap_client_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        if (phase != LINK_DOWN || client_conn_handle != BLE_HS_CONN_HANDLE_NONE) break;
        struct ble_hs_adv_fields fields = {0};
        if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data)) break;
        if (fields.name_len != strlen(ROASTER_DEVICE_NAME) || !fields.name ||
            memcmp(fields.name, ROASTER_DEVICE_NAME, fields.name_len)) break;
        set_phase(LINK_CONNECTING); /* Set before cancel: suppress scan-complete restart. */
        int rc = ble_gap_disc_cancel();
        if (rc) { ESP_LOGW(TAG, "scan cancel rc=%d", rc); schedule_scan(); break; }
        struct ble_gap_conn_params params = {0};
        params.scan_itvl = 16; params.scan_window = 16;
        params.itvl_min = 80; params.itvl_max = 100;
        params.supervision_timeout = 400;
        setup_deadline = now_ms()+35000;
        rc = ble_gap_connect(own_addr_type, &event->disc.addr, 30000,
                              &params, gap_client_event_cb, NULL);
        if (rc) { ESP_LOGW(TAG, "connect initiation rc=%d", rc); schedule_scan(); }
        break;
    }
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status) {
            ESP_LOGW(TAG, "SR900 connect failed status=%d", event->connect.status);
            schedule_scan();
        } else {
            clear_roaster_state();
            client_conn_handle = event->connect.conn_handle;
            set_phase(LINK_MTU);
            ESP_LOGI(TAG, "SR900 connected handle=%u; exchanging MTU", client_conn_handle);
            int rc = ble_gattc_exchange_mtu(client_conn_handle, mtu_cb,
                                           (void *)(uintptr_t)session_epoch);
            if (rc == BLE_HS_EALREADY) {
                /* ESP-IDF can start this procedure before the connect callback.
                 * Its completion will not invoke our rejected callback. Poll
                 * the actual MTU on the host queue, within setup_deadline. */
                mtu_wait_existing = true;
                ESP_LOGI(TAG, "MTU exchange already active/completed; waiting for MTU >= 37 (actual=%u)",
                         ble_att_mtu(client_conn_handle));
            } else if (rc) fail_link("MTU exchange initiation", rc);
        }
        break;
    case BLE_GAP_EVENT_NOTIFY_RX: {
        if (event->notify_rx.conn_handle != client_conn_handle ||
            event->notify_rx.attr_handle != roaster_notify_val_handle ||
            phase == LINK_FAULT) break;
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (len != FRAME_LEN) {
            ESP_LOGW(TAG, "Unexpected SR900 notification length=%u", len);
            break;
        }
        uint8_t frame[FRAME_LEN];
        if (!ble_hs_mbuf_to_flat(event->notify_rx.om, frame, sizeof(frame), NULL))
            process_roaster_frame(frame, sizeof(frame));
        break;
    }
    case BLE_GAP_EVENT_DISCONNECT:
        if (event->disconnect.conn.conn_handle != client_conn_handle) break;
        ESP_LOGW(TAG, "SR900 disconnected reason=%d", event->disconnect.reason);
        client_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        clear_roaster_state();
        fail_phone("SR900 disconnected; cannot confirm machine state");
        schedule_scan();
        break;
    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (phase == LINK_DOWN) retry_at = now_ms()+500;
        break;
    default: break;
    }
    return 0;
}

static void ble_app_scan(void)
{
    if (!synced || phase != LINK_DOWN || ble_gap_disc_active() ||
        client_conn_handle != BLE_HS_CONN_HANDLE_NONE) return;
    struct ble_gap_disc_params params = {
        .filter_duplicates = 1, .passive = 0, .itvl = 0x10, .window = 0x10
    };
    int rc = ble_gap_disc(own_addr_type, 10000, &params, gap_client_event_cb, NULL);
    if (rc) { ESP_LOGW(TAG, "scan start rc=%d", rc); schedule_scan(); }
    else ESP_LOGI(TAG, "Scanning for %s", ROASTER_DEVICE_NAME);
}

/* Always runs on NimBLE's host event queue; no sleeping in a GATT callback. */
static void maintenance(struct ble_npl_event *event)
{
    (void)event;
    if (!synced) return;
    int64_t now = now_ms();
    if (tx_busy && now >= tx_deadline) {
        fail_link("ATT write timeout", BLE_HS_ETIMEOUT);
    } else if (pending.active && now >= pending.deadline) {
        ESP_LOGE(TAG, "CONTROL timeout id=%" PRIu32 " wanted=%u fan=%d heater=%d",
                 pending.id, pending.value, last_fan, last_heater);
        pending.active = false;
        tx_count = 0;
        fail_phone("control feedback timeout");
    } else if (read_waiting && now >= read_deadline) {
        fail_phone("READ timeout");
    }
    if (phase == LINK_MTU && mtu_wait_existing && now < setup_deadline)
        advance_after_mtu();
    if (phase != LINK_DOWN && phase != LINK_READY && phase != LINK_FAULT &&
        now >= setup_deadline) {
        if (phase == LINK_CONNECTING) {
            int rc = ble_gap_conn_cancel();
            ESP_LOGW(TAG, "connection attempt timeout; cancel rc=%d", rc);
            /* Failed connect event normally schedules the next scan. */
            if (rc) schedule_scan();
            else setup_deadline = now+5000;
        } else if (phase == LINK_MTU) {
            ESP_LOGE(TAG, "MTU wait expired: actual=%u required=%u",
                     ble_att_mtu(client_conn_handle), FRAME_LEN+3);
            fail_link("MTU negotiation timeout", BLE_HS_ETIMEOUT);
        } else fail_link("discovery/handshake timeout", BLE_HS_ETIMEOUT);
    }
    if (phase == LINK_DOWN && now >= retry_at) ble_app_scan();
    try_status_response();
    ble_app_advertise();
    int rc = ble_npl_callout_reset(&maintenance_callout, ble_npl_time_ms_to_ticks32(100));
    if (rc) ESP_LOGE(TAG, "maintenance timer reset rc=%d", rc);
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE host reset reason=%d", reason);
    synced = false;
    ble_npl_callout_stop(&maintenance_callout);
    client_conn_handle = hibean_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    hb_subscribed = hb_closing = read_waiting = false;
    read_fault = true;
    clear_roaster_state();
    phase = LINK_DOWN;
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (!rc) rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc) { ESP_LOGE(TAG, "BLE address initialization rc=%d", rc); return; }
    synced = true;
    retry_count = 0;
    retry_at = now_ms();
    ESP_LOGI(TAG, "NimBLE synced; HiBean status handle=%u", hb_status_val_handle);
    if (!hb_status_val_handle) { synced = false; ESP_LOGE(TAG, "GATT service not registered"); return; }
    ble_app_scan();
    ble_app_advertise();
    rc = ble_npl_callout_reset(&maintenance_callout, ble_npl_time_ms_to_ticks32(100));
    if (rc) ESP_LOGE(TAG, "maintenance timer start rc=%d", rc);
}

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    ESP_LOGI(TAG, "Bridge revision: feedback-sync-3");
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        /* Do not erase a user's NVS partition automatically. */
        ESP_LOGE(TAG, "nvs_flash_init failed: %s; NVS was not erased", esp_err_to_name(err));
        return;
    }
    int rc = nimble_port_init();
    if (rc) { ESP_LOGE(TAG, "nimble_port_init rc=%d", rc); return; }
    ble_svc_gap_init();
    ble_svc_gatt_init();
    rc = ble_svc_gap_device_name_set("ESP32-SR900-Bridge");
    if (rc) { ESP_LOGE(TAG, "device name rc=%d", rc); return; }
    rc = ble_att_set_preferred_mtu(64);
    if (rc) { ESP_LOGE(TAG, "preferred MTU rc=%d; configure NimBLE ATT MTU >= 64", rc); return; }
    rc = ble_gatts_count_cfg(hb_gatt_svcs);
    if (!rc) rc = ble_gatts_add_svcs(hb_gatt_svcs);
    if (rc) { ESP_LOGE(TAG, "GATT registration rc=%d", rc); return; }
    ble_npl_callout_init(&maintenance_callout, nimble_port_get_dflt_eventq(),
                         maintenance, NULL);
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    nimble_port_freertos_init(ble_host_task);
}
