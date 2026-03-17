#include "data_merge.h"
#include "sync_config.h"
#include "sd_storage.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"

static const char *TAG = "data_merge";

// ─── Per-MAC high-water mark dedup ────────────────────────────────────────────
// Tracks the highest timestamp already stored in merged.csv for each unique
// device MAC.  Only records with ts > max_ts_for_that_MAC are appended.
//
// This is needed because a row can arrive from multiple paths:
//   - directly from device A
//   - forwarded by device C (which got it from device A earlier)
// The sender-side delta only prevents re-sending rows *this device* sent before;
// it cannot know about rows the receiver already obtained via a third device.
//
// Fixed stack usage: 32 * ~26 bytes = ~830 bytes regardless of record count.

#define MAX_TRACKED_MACS 32

typedef struct {
    char    mac[5];    // "AABB\0" – last 4 hex digits
    int64_t max_ts;
} MacTracker;

// Parse the nth (0-based) comma-separated field of a CSV line into dst.
static bool csv_field(const char *line, int n, char *dst, size_t dst_sz)
{
    const char *p = line;
    for (int i = 0; i < n; i++) {
        p = strchr(p, ',');
        if (!p) return false;
        p++;
    }
    size_t len = strcspn(p, ",\r\n");
    if (len == 0 || len >= dst_sz) return false;
    memcpy(dst, p, len);
    dst[len] = '\0';
    return true;
}

// One-pass scan of a CSV file; fills trackers[] with the max timestamp per MAC.
static void load_max_timestamps(const char *path,
                                MacTracker *t, int *count, int max)
{
    FILE *f = fopen(path, "r");
    if (!f) return;
    char line[256];
    bool first = true;
    while (fgets(line, sizeof(line), f)) {
        if (first) { first = false; continue; } // skip header
        line[strcspn(line, "\r\n")] = '\0';
        if (line[0] == '\0') continue;

        char ts_str[24], mac[5];
        if (!csv_field(line, 0, ts_str, sizeof(ts_str))) continue;
        if (!csv_field(line, 6, mac,    sizeof(mac)))    continue;
        int64_t ts = (int64_t)strtoll(ts_str, NULL, 10);

        int idx = -1;
        for (int i = 0; i < *count; i++)
            if (strcmp(t[i].mac, mac) == 0) { idx = i; break; }

        if (idx < 0) {
            if (*count >= max) continue; // table full, drop least-common MACs
            idx = (*count)++;
            memcpy(t[idx].mac, mac, sizeof(t[idx].mac));
            t[idx].max_ts = ts;
        } else if (ts > t[idx].max_ts) {
            t[idx].max_ts = ts;
        }
    }
    fclose(f);
}

// ─── Public API ───────────────────────────────────────────────────────────────

int data_merge_from_file(const char *incoming_path)
{
    sd_lock();

    // Load the highest timestamp already stored per device MAC.
    MacTracker trackers[MAX_TRACKED_MACS];
    int        tracker_count = 0;
    load_max_timestamps(CSV_MERGED_FILE, trackers, &tracker_count, MAX_TRACKED_MACS);
    ESP_LOGI(TAG, "Loaded HWM for %d device MAC(s)", tracker_count);

    FILE *in = fopen(incoming_path, "r");
    if (!in) {
        ESP_LOGE(TAG, "Cannot open %s", incoming_path);
        sd_unlock();
        return -1;
    }
    FILE *out = fopen(CSV_MERGED_FILE, "a");
    if (!out) {
        ESP_LOGE(TAG, "Cannot open %s for append", CSV_MERGED_FILE);
        fclose(in);
        sd_unlock();
        return -1;
    }

    int  appended = 0, skipped = 0;
    char line[256];

    while (fgets(line, sizeof(line), in)) {
        // Skip stray header
        if (strncmp(line, "timestamp", 9) == 0) continue;
        if (line[0] == '\n' || line[0] == '\r' || line[0] == '\0') continue;

        // Ensure trailing newline
        size_t ll = strlen(line);
        if (ll > 0 && line[ll - 1] != '\n') { line[ll++] = '\n'; line[ll] = '\0'; }

        char ts_str[24], mac[5];
        if (!csv_field(line, 0, ts_str, sizeof(ts_str)) ||
            !csv_field(line, 6, mac,    sizeof(mac))) {
            ESP_LOGW(TAG, "Skipping malformed line: %.60s", line);
            continue;
        }
        int64_t ts = (int64_t)strtoll(ts_str, NULL, 10);

        // Find or create tracker for this MAC
        int idx = -1;
        for (int i = 0; i < tracker_count; i++)
            if (strcmp(trackers[i].mac, mac) == 0) { idx = i; break; }

        // Skip if we've already stored this or a newer record for this MAC
        if (idx >= 0 && ts <= trackers[idx].max_ts) {
            skipped++;
            continue;
        }

        if (fputs(line, out) >= 0) {
            // Advance the high-water mark so later rows in this batch are also checked
            if (idx < 0 && tracker_count < MAX_TRACKED_MACS) {
                idx = tracker_count++;
                memcpy(trackers[idx].mac, mac, sizeof(trackers[idx].mac));
                trackers[idx].max_ts = ts;
            } else if (idx >= 0 && ts > trackers[idx].max_ts) {
                trackers[idx].max_ts = ts;
            }
            appended++;
        } else {
            ESP_LOGE(TAG, "Write error on %s", CSV_MERGED_FILE);
            break;
        }
    }

    fclose(out);
    fclose(in);
    sd_unlock();

    ESP_LOGI(TAG, "Merged %d new record(s), skipped %d duplicate(s)", appended, skipped);
    return appended;
}
