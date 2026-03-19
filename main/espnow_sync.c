#include "espnow_sync.h"
#include "sync_config.h"
#include "sd_storage.h"
#include "data_merge.h"

#include <string.h>
#include <stdlib.h>
#include <strings.h>
#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_timer.h"

static const char *TAG = "espnow_sync";

// ─── Frame types ──────────────────────────────────────────────────────────────
#define MSG_ANNOUNCE  0x01
#define MSG_SYNC_REQ  0x02
#define MSG_DATA      0x03
#define MSG_DONE      0x04

// ─── Wire frame layout ────────────────────────────────────────────────────────
// Maximum ESP-NOW payload is 250 bytes.
// We reserve 4 bytes for the header → 246 bytes of CSV per DATA frame.
#define ESPNOW_MTU          250
#define ESPNOW_DATA_PAYLOAD (ESPNOW_MTU - 4)

#pragma pack(push, 1)
typedef struct {
    uint8_t  type;     // MSG_*
    uint8_t  seq;      // sequence number (wraps at 255)
    uint16_t length;   // payload length in this frame
    uint8_t  payload[ESPNOW_DATA_PAYLOAD];
} espnow_frame_t;
#pragma pack(pop)

// ─── Internal receive queue ───────────────────────────────────────────────────
typedef struct {
    uint8_t        src_mac[6];
    espnow_frame_t frame;
} rx_item_t;

#define RX_QUEUE_DEPTH  32
static QueueHandle_t  s_rx_queue  = NULL;
static SemaphoreHandle_t s_tx_done = NULL; // signalled by the TX callback
static uint8_t        s_own_mac[6];

// ─── Per-peer delta-sync state ────────────────────────────────────────────────
// Tracks the timestamp watermark and merged.csv row offset for each known peer.
// Only new GPS rows (timestamp > last_sent_timestamp, fix==1) from sync_data.csv
// and new merged.csv rows (beyond last_merged_row) are transmitted each
// sync round, keeping bandwidth proportional to new records only.
// Call espnow_sync_reset_peer_states() whenever the data files are cleared so
// the watermarks restart from zero on the fresh files.
#define MAX_SYNC_PEERS 8
typedef struct {
    uint8_t mac[6];
    int64_t last_sent_timestamp; // Unix epoch of last own GPS row sent to this peer
    int     last_merged_row;     // rows of merged.csv already sent to this peer
} PeerSyncState;
static PeerSyncState s_peer_states[MAX_SYNC_PEERS];
static int           s_peer_state_count = 0;

static PeerSyncState *get_peer_state(const uint8_t *mac)
{
    for (int i = 0; i < s_peer_state_count; i++)
        if (memcmp(s_peer_states[i].mac, mac, 6) == 0)
            return &s_peer_states[i];
    if (s_peer_state_count < MAX_SYNC_PEERS) {
        PeerSyncState *p = &s_peer_states[s_peer_state_count++];
        memcpy(p->mac, mac, 6);
        p->last_sent_timestamp = 0;
        p->last_merged_row     = 0;
        return p;
    }
    return NULL; // table full – caller falls back to defaults
}

// ─── ESP-NOW TX-done callback (called from WiFi task after each send) ────────
static void on_send(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    // Unblock send_frame regardless of success/failure – caller checks status
    BaseType_t higher_prio = pdFALSE;
    xSemaphoreGiveFromISR(s_tx_done, &higher_prio);
    portYIELD_FROM_ISR(higher_prio);
}

// ─── ESP-NOW receive callback (ISR-safe) ─────────────────────────────────────
static void on_recv(const esp_now_recv_info_t *info,
                    const uint8_t *data, int data_len)
{
    if (data_len < 4 || data_len > (int)sizeof(espnow_frame_t)) return;

    rx_item_t item;
    memcpy(item.src_mac, info->src_addr, 6);
    memcpy(&item.frame, data, data_len);

    // Non-blocking enqueue; drop if the queue is full (back-pressure)
    xQueueSendFromISR(s_rx_queue, &item, NULL);
}

// ─── Utility: send a frame to a peer (with retries) ──────────────────────────
static bool send_frame(const uint8_t *peer_mac, espnow_frame_t *frame,
                       size_t payload_len)
{
    frame->length = (uint16_t)payload_len;
    size_t total  = 4 + payload_len;

    // Register peer if not already known
    if (!esp_now_is_peer_exist(peer_mac)) {
        esp_now_peer_info_t peer_info = { 0 };
        memcpy(peer_info.peer_addr, peer_mac, 6);
        peer_info.channel = 0; // 0 = follow current WiFi home channel
        peer_info.encrypt = false;
        esp_err_t err = esp_now_add_peer(&peer_info);
        if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
            ESP_LOGE(TAG, "add_peer failed: %s", esp_err_to_name(err));
            return false;
        }
    }

    // Consume any stale semaphore token before sending
    xSemaphoreTake(s_tx_done, 0);

    esp_err_t err = esp_now_send(peer_mac, (uint8_t *)frame, total);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_send failed: %s", esp_err_to_name(err));
        return false;
    }
    // Wait for the TX-done callback (max 200 ms) before returning
    if (xSemaphoreTake(s_tx_done, pdMS_TO_TICKS(200)) != pdTRUE) {
        ESP_LOGW(TAG, "TX-done timeout for frame to " MACSTR, MAC2STR(peer_mac));
        return false;
    }
    return true;
}

// ─── Stream CSV delta to a peer ───────────────────────────────────────────────
// Sends two deltas in a single stream (shared sequence counter, single DONE):
//   1. Rows from sync_data.csv where timestamp > last_sent_timestamp
//   2. Rows from merged.csv not yet sent to this peer (forwarded peer records).
static int stream_csv_to_peer(const uint8_t *peer_mac)
{
    PeerSyncState *state        = get_peer_state(peer_mac);
    int64_t send_from_ts        = state ? state->last_sent_timestamp : 0;
    int     merged_start        = state ? state->last_merged_row      : 0;

    uint8_t seq              = 0;
    size_t  total_sent       = 0;
    int     gps_rows_sent    = 0;
    int     merged_rows_sent = 0;
    int64_t max_ts_sent      = send_from_ts;

    espnow_frame_t frame;
    size_t fill = 0;
    char   line[256];

    // ── 1. Stream own GPS rows from sync_data.csv ─────────────────────────
    {
        FILE *f = fopen(CSV_DATA_FILE, "r");
        if (f) {
            while (fgets(line, sizeof(line), f)) {
                if (!strchr(line, ',')) continue;
                int64_t ts = (int64_t)strtoll(line, NULL, 10);
                if (ts <= send_from_ts) continue; // already sent in a prior round

                size_t ll = strlen(line);
                if (fill > 0 && fill + ll > ESPNOW_DATA_PAYLOAD) {
                    frame.type = MSG_DATA; frame.seq = seq++;
                    if (!send_frame(peer_mac, &frame, fill)) { fclose(f); return -1; }
                    total_sent += fill; fill = 0;
                }
                if (ll > ESPNOW_DATA_PAYLOAD) ll = ESPNOW_DATA_PAYLOAD;
                memcpy(frame.payload + fill, line, ll);
                fill += ll;
                gps_rows_sent++;
                if (ts > max_ts_sent) max_ts_sent = ts;
            }
            fclose(f);
        }
    }

    // ── 2. Stream merged.csv delta ────────────────────────────────────────
    FILE *f = fopen(CSV_MERGED_FILE, "r");
    if (f) {
        int skipped = 0;
        while (fgets(line, sizeof(line), f)) {
            if (skipped < merged_start) { skipped++; continue; }

            size_t ll = strlen(line);
            if (fill > 0 && fill + ll > ESPNOW_DATA_PAYLOAD) {
                frame.type = MSG_DATA; frame.seq = seq++;
                if (!send_frame(peer_mac, &frame, fill)) { fclose(f); return -1; }
                total_sent += fill; fill = 0;
            }
            if (ll > ESPNOW_DATA_PAYLOAD) ll = ESPNOW_DATA_PAYLOAD;
            memcpy(frame.payload + fill, line, ll);
            fill += ll;
            merged_rows_sent++;
        }
        fclose(f);
    }

    // ── Flush remainder ───────────────────────────────────────────────────
    if (fill > 0) {
        frame.type = MSG_DATA; frame.seq = seq++;
        if (!send_frame(peer_mac, &frame, fill)) return -1;
        total_sent += fill;
    }

    // ── Send DONE; advance offsets only on success ────────────────────────
    espnow_frame_t done = { .type = MSG_DONE, .seq = seq, .length = 0 };
    if (!send_frame(peer_mac, &done, 0)) return -1;

    if (state) {
        state->last_sent_timestamp  = max_ts_sent;
        state->last_merged_row     += merged_rows_sent;
    }

    ESP_LOGI(TAG, "Sent %d GPS + %d forwarded rows (%zu B) to " MACSTR
             " [ts_hwm:%" PRId64 ", merged:%d-%d]",
             gps_rows_sent, merged_rows_sent, total_sent, MAC2STR(peer_mac),
             max_ts_sent, merged_start, merged_start + merged_rows_sent - 1);
    return gps_rows_sent + merged_rows_sent;
}

// ─── Receive CSV data from a peer ─────────────────────────────────────────────
// Writes received DATA frames directly to CSV_MERGE_TMP on the SD card,
// then calls data_merge_incoming to fold them into the main data file.
// Returns number of records merged, or -1 on error/timeout.
static int receive_and_merge_from_peer(const uint8_t *peer_mac)
{
    // Open the temp file for writing (overwrite any previous run).
    // No need to hold sd_lock for the whole receive window — the data
    // generator writes to data.csv while we write to merge_tmp.csv,
    // so there is no file-level contention. The sd_lock is taken briefly
    // per-operation inside the sd_storage helpers.
    sd_lock();
    FILE *tmp = fopen(CSV_MERGE_TMP, "w");
    sd_unlock();
    if (!tmp) {
        ESP_LOGE(TAG, "Cannot open merge temp file %s", CSV_MERGE_TMP);
        return -1;
    }

    int64_t deadline = esp_timer_get_time() + (int64_t)ESPNOW_RECV_TIMEOUT_MS * 1000;
    size_t  bytes_written = 0;
    bool    got_done = false;

    while (esp_timer_get_time() < deadline) {
        rx_item_t item;
        if (xQueueReceive(s_rx_queue, &item, pdMS_TO_TICKS(200)) != pdTRUE) continue;

        // Only process frames from the expected peer
        if (memcmp(item.src_mac, peer_mac, 6) != 0) continue;

        if (item.frame.type == MSG_DONE) {
            ESP_LOGI(TAG, "Received DONE from " MACSTR, MAC2STR(peer_mac));
            got_done = true;
            break;
        }

        if (item.frame.type == MSG_DATA) {
            uint16_t chunk = item.frame.length;
            if (chunk > ESPNOW_DATA_PAYLOAD) chunk = ESPNOW_DATA_PAYLOAD;
            if (fwrite(item.frame.payload, 1, chunk, tmp) != chunk) {
                ESP_LOGE(TAG, "SD write error during receive");
                fclose(tmp);
                remove(CSV_MERGE_TMP);
                return -1;
            }
            bytes_written += chunk;
        }
    }

    fclose(tmp);

    if (!got_done && bytes_written == 0) {
        ESP_LOGW(TAG, "No data received from " MACSTR " (timeout)", MAC2STR(peer_mac));
        remove(CSV_MERGE_TMP);
        return 0;
    }

    ESP_LOGI(TAG, "Received %zu bytes from " MACSTR ", merging...",
             bytes_written, MAC2STR(peer_mac));

    // Format own device MAC as 4-char hex string to filter echo-back rows
    char own_mac_str[5];
    snprintf(own_mac_str, sizeof(own_mac_str), "%02X%02X", s_own_mac[4], s_own_mac[5]);

    int merged = data_merge_from_file(CSV_MERGE_TMP, own_mac_str);
    remove(CSV_MERGE_TMP);
    return merged;
}

// ─── Public API ───────────────────────────────────────────────────────────────

bool espnow_init(void)
{
    ESP_ERROR_CHECK(esp_read_mac(s_own_mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG, "Own MAC: " MACSTR, MAC2STR(s_own_mac));

    s_rx_queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(rx_item_t));
    if (!s_rx_queue) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return false;
    }

    s_tx_done = xSemaphoreCreateBinary();
    if (!s_tx_done) {
        ESP_LOGE(TAG, "Failed to create TX semaphore");
        return false;
    }

    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed");
        return false;
    }

    ESP_ERROR_CHECK(esp_now_register_send_cb(on_send));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));

    // Register broadcast address as a peer so we can send to it
    uint8_t broadcast[6] = ESPNOW_BROADCAST_MAC;
    if (!esp_now_is_peer_exist(broadcast)) {
        esp_now_peer_info_t bcast_peer = { 0 };
        memcpy(bcast_peer.peer_addr, broadcast, 6);
        bcast_peer.channel = 0; // 0 = follow current WiFi home channel
        bcast_peer.encrypt = false;
        ESP_ERROR_CHECK(esp_now_add_peer(&bcast_peer));
    }

    ESP_LOGI(TAG, "ESP-NOW initialised");
    return true;
}

void espnow_deinit(void)
{
    esp_now_unregister_send_cb();
    esp_now_unregister_recv_cb();
    esp_now_deinit();
    if (s_rx_queue) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
    }
    if (s_tx_done) {
        vSemaphoreDelete(s_tx_done);
        s_tx_done = NULL;
    }
}

void espnow_sync_reset_peer_states(void)
{
    s_peer_state_count = 0;
    ESP_LOGI(TAG, "Per-peer sync state cleared");
}

int espnow_sync_round(void)
{
    int peers_synced = 0;
    uint8_t broadcast[6] = ESPNOW_BROADCAST_MAC;

    // ── 1. Broadcast our ANNOUNCE ──────────────────────────────────────────
    espnow_frame_t ann = { .type = MSG_ANNOUNCE, .seq = 0 };
    memcpy(ann.payload, s_own_mac, 6);
    send_frame(broadcast, &ann, 6);
    ESP_LOGI(TAG, "ANNOUNCE broadcast sent");

    // ── 2. Listen for ANNOUNCE or SYNC_REQ ────────────────────────────────
    int64_t discover_end = esp_timer_get_time()
                           + (int64_t)ESPNOW_DISCOVER_MS * 1000;

    // Track peers we've already synced this round to avoid double-sync
    uint8_t synced_peers[20][6];
    int     synced_count = 0;

    while (esp_timer_get_time() < discover_end) {
        rx_item_t item;
        if (xQueueReceive(s_rx_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        // ── Ignore our own broadcast reflections ─────────────────────────
        if (memcmp(item.src_mac, s_own_mac, 6) == 0) continue;

        // ── Check already synced this round ──────────────────────────────
        bool already_done = false;
        for (int i = 0; i < synced_count; i++) {
            if (memcmp(synced_peers[i], item.src_mac, 6) == 0) {
                already_done = true; break;
            }
        }
        if (already_done) continue;

        if (item.frame.type == MSG_ANNOUNCE) {
            ESP_LOGI(TAG, "Peer ANNOUNCE from " MACSTR, MAC2STR(item.src_mac));

            // Send SYNC_REQ to that peer
            espnow_frame_t req = { .type = MSG_SYNC_REQ, .seq = 0 };
            memcpy(req.payload, s_own_mac, 6);
            if (!send_frame(item.src_mac, &req, 6)) continue;

            // Receive their CSV
            int records_rx = receive_and_merge_from_peer(item.src_mac);
            if (records_rx >= 0) {
                ESP_LOGI(TAG, "Merged %d new records from " MACSTR,
                         records_rx, MAC2STR(item.src_mac));
            }

            // Send our CSV back
            int records_tx = stream_csv_to_peer(item.src_mac);

            sd_log_connection(item.src_mac,
                              records_rx  >= 0 ? records_rx  : 0,
                              records_tx  >= 0 ? records_tx  : 0);

            if (synced_count < 20) {
                memcpy(synced_peers[synced_count++], item.src_mac, 6);
            }
            peers_synced++;

        } else if (item.frame.type == MSG_SYNC_REQ) {
            ESP_LOGI(TAG, "Peer SYNC_REQ from " MACSTR, MAC2STR(item.src_mac));

            // They requested our data first – stream ours, then receive theirs
            int records_tx = stream_csv_to_peer(item.src_mac);

            int records_rx = receive_and_merge_from_peer(item.src_mac);
            if (records_rx >= 0) {
                ESP_LOGI(TAG, "Merged %d new records from " MACSTR,
                         records_rx, MAC2STR(item.src_mac));
            }

            sd_log_connection(item.src_mac,
                              records_rx >= 0 ? records_rx : 0,
                              records_tx >= 0 ? records_tx : 0);

            if (synced_count < 20) {
                memcpy(synced_peers[synced_count++], item.src_mac, 6);
            }
            peers_synced++;
        }
    }

    ESP_LOGI(TAG, "Sync round complete, %d peer(s) synced", peers_synced);
    return peers_synced;
}
