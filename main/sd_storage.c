#include "sd_storage.h"
#include "config.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "sd_storage";
static sdmmc_card_t *s_card = NULL;

// Recursive mutex so the same task can re-enter (e.g. data_merge holds lock
// and also calls sd_append_record internally if needed in future).
static SemaphoreHandle_t s_sd_mutex = NULL;

void sd_lock(void)   { xSemaphoreTakeRecursive(s_sd_mutex, portMAX_DELAY); }
void sd_unlock(void) { xSemaphoreGiveRecursive(s_sd_mutex); }

// ─── Mount ────────────────────────────────────────────────────────────────────

bool sd_init(void)
{
    if (s_card != NULL) {
        return true; // already mounted
    }

    if (s_sd_mutex == NULL) {
        s_sd_mutex = xSemaphoreCreateRecursiveMutex();
        configASSERT(s_sd_mutex);
    }

    // In ESP-IDF v5.x, esp_vfs_fat_sdmmc_mount re-runs SDMMC card init (CMD0
    // sequence) BEFORE checking whether the VFS path is already registered.
    // If the SD card was already mounted by main, that re-init
    // disturbs the card and returns an error — ESP_ERR_INVALID_STATE is never
    // reached. Guard against this by testing the mount point directly.
    struct stat mnt_st;
    if (stat(SD_MOUNT_POINT, &mnt_st) == 0) {
        s_card = (sdmmc_card_t *)1; // sentinel: mounted externally
        ESP_LOGI(TAG, "SD already mounted at %s — reusing", SD_MOUNT_POINT);
        return true;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 5,   // data.csv(r) + data.csv(a) + merge_tmp + headroom
        .allocation_unit_size   = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT; // 1-bit mode

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk   = SDMMC_CLK_GPIO;
    slot.cmd   = SDMMC_CMD_GPIO;
    slot.d0    = SDMMC_D0_GPIO;
    slot.width = 1;
    // Enable internal pull-ups on the data/cmd lines
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot,
                                             &mount_cfg, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Mount failed: %s", esp_err_to_name(ret));
        s_card = NULL;
        return false;
    }

    ESP_LOGI(TAG, "SD card mounted at %s  (%.1f MB)",
             SD_MOUNT_POINT,
             (float)((uint64_t)s_card->csd.capacity * s_card->csd.sector_size)
             / (1024.0f * 1024.0f));
    return true;
}

void sd_deinit(void)
{
    if (s_card == NULL) return;
    if (s_card == (sdmmc_card_t *)1) {
        // Card was mounted externally — do not unmount.
        s_card = NULL;
        return;
    }
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
    s_card = NULL;
    ESP_LOGI(TAG, "SD card unmounted");
}

// ─── CSV file helpers ─────────────────────────────────────────────────────────

static bool sd_ensure_file(const char *path)
{
    sd_lock();
    struct stat st;
    if (stat(path, &st) == 0) { sd_unlock(); return true; }
    FILE *f = fopen(path, "w");
    if (!f) { ESP_LOGE(TAG, "Cannot create %s", path); sd_unlock(); return false; }
    fclose(f);
    sd_unlock();
    ESP_LOGI(TAG, "Created %s", path);
    return true;
}

static bool sd_append_line(const char *path, const char *line)
{
    if (!line) return false;
    sd_lock();
    FILE *f = fopen(path, "a");
    if (!f) { ESP_LOGE(TAG, "Cannot open %s for append", path); sd_unlock(); return false; }
    bool ok = (fputs(line, f) >= 0);
    fclose(f);
    sd_unlock();
    return ok;
}

static bool sd_delete_file(const char *path)
{
    sd_lock();
    bool ok = (remove(path) == 0);
    sd_unlock();
    if (ok) ESP_LOGI(TAG, "Deleted %s", path);
    else    ESP_LOGE(TAG, "Cannot delete %s", path);
    return ok;
}

bool sd_ensure_data_file(void)          { return sd_ensure_file(CSV_DATA_FILE); }

bool sd_append_record(const char *line)   { return sd_append_line(CSV_DATA_FILE, line); }

int sd_record_count(void)
{
    sd_lock();
    FILE *f = fopen(CSV_DATA_FILE, "r");
    if (!f) { sd_unlock(); return -1; }

    int count = 0;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] != '\n' && line[0] != '\r' && line[0] != '\0') {
            count++;
        }
    }
    fclose(f);
    sd_unlock();
    return count;
}

bool sd_delete_data_file(void)            { return sd_delete_file(CSV_DATA_FILE); }

int sd_next_bak_index(const char *fmt, char *out_path, size_t out_size)
{
    struct stat st;
    for (int i = 1; i <= 999; i++) {
        snprintf(out_path, out_size, fmt, i);
        if (stat(out_path, &st) != 0) return i;
    }
    return -1;
}

static bool sd_backup_file(const char *path, const char *bak_fmt)
{
    char bak_path[48];
    sd_lock();
    int idx = sd_next_bak_index(bak_fmt, bak_path, sizeof(bak_path));
    if (idx < 0) { ESP_LOGE(TAG, "No free backup slot for %s", path); sd_unlock(); return false; }
    if (rename(path, bak_path) != 0) {
        ESP_LOGE(TAG, "Cannot rename %s -> %s", path, bak_path);
        sd_unlock();
        return false;
    }
    ESP_LOGI(TAG, "Backed up %s -> %s", path, bak_path);
    FILE *f = fopen(path, "w");
    if (f) fclose(f);
    ESP_LOGI(TAG, "Created fresh %s", path);
    sd_unlock();
    return true;
}

bool sd_backup_data_file(void)            { return sd_backup_file(CSV_DATA_FILE, CSV_DATA_BAK_FMT); }

// ─── Peer-data (merged) file ──────────────────────────────────────────────────

bool sd_ensure_merged_file(void)          { return sd_ensure_file(CSV_MERGED_FILE); }

bool sd_append_merged_record(const char *line) { return sd_append_line(CSV_MERGED_FILE, line); }

int sd_read_merged(char *buf, size_t buf_size)
{
    if (!buf || buf_size == 0) return -1;
    sd_lock();
    FILE *f = fopen(CSV_MERGED_FILE, "r");
    if (!f) {
        sd_unlock();
        return -1; // not an error — file may not exist yet
    }
    size_t n = fread(buf, 1, buf_size - 1, f);
    buf[n] = '\0';
    fclose(f);
    sd_unlock();
    return (int)n;
}

bool sd_delete_merged_file(void)          { return sd_delete_file(CSV_MERGED_FILE); }

bool sd_backup_merged_file(void)          { return sd_backup_file(CSV_MERGED_FILE, CSV_MERGED_BAK_FMT); }

int sd_read_all(char *buf, size_t buf_size)
{
    if (!buf || buf_size == 0) return -1;
    sd_lock();
    FILE *f = fopen(CSV_DATA_FILE, "r");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open %s for read", CSV_DATA_FILE);
        sd_unlock();
        return -1;
    }
    size_t n = fread(buf, 1, buf_size - 1, f);
    buf[n] = '\0';
    fclose(f);
    sd_unlock();
    return (int)n;
}

// ─── Connection log ───────────────────────────────────────────────────────────

bool sd_log_connection(const uint8_t *peer_mac, int records_rx, int records_tx)
{
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);

    char line[128];
    snprintf(line, sizeof(line),
             "%04d-%02d-%02dT%02d:%02d:%02d,%02X:%02X:%02X:%02X:%02X:%02X,%d,%d\n",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec,
             peer_mac[0], peer_mac[1], peer_mac[2],
             peer_mac[3], peer_mac[4], peer_mac[5],
             records_rx, records_tx);

    sd_lock();
    struct stat st;
    bool needs_header = (stat(CONNECTIONS_LOG_FILE, &st) != 0);
    FILE *f = fopen(CONNECTIONS_LOG_FILE, "a");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open %s for append", CONNECTIONS_LOG_FILE);
        sd_unlock();
        return false;
    }
    if (needs_header) {
        fputs("timestamp,peer_mac,records_rx,records_tx\n", f);
    }
    bool ok = (fputs(line, f) >= 0);
    fclose(f);
    sd_unlock();
    return ok;
}

// ─── Upload staging helpers ───────────────────────────────────────────────────

// Internal: append all bytes from src_path to dst_path, then delete src_path.
// Called under sd_lock().
static void append_and_remove(const char *src_path, const char *dst_path)
{
    FILE *src = fopen(src_path, "r");
    FILE *dst = fopen(dst_path, "a");
    if (src && dst) {
        char buf[256];
        size_t n;
        while ((n = fread(buf, 1, sizeof(buf), src)) > 0) {
            fwrite(buf, 1, n, dst);
        }
    }
    if (src) fclose(src);
    if (dst) fclose(dst);
    remove(src_path);
}

static bool stage_file_for_upload(const char *src, const char *stage)
{
    sd_lock();

    struct stat st;
    bool stage_exists = (stat(stage, &st) == 0 && st.st_size > 0);
    bool src_exists   = (stat(src,   &st) == 0 && st.st_size > 0);

    if (!stage_exists && !src_exists) {
        // Nothing to upload
        sd_unlock();
        return false;
    }

    if (!stage_exists) {
        // Normal happy path: atomically move the live file to the staging slot
        bool ok = (rename(src, stage) == 0);
        sd_unlock();
        if (!ok) ESP_LOGE(TAG, "stage rename %s -> %s failed", src, stage);
        return ok;
    }

    // A staging file already exists from a previous failed upload.
    // Append any new data from the live file into it so it is not lost,
    // then remove the live file so the GPS task gets a clean slate.
    if (src_exists) {
        append_and_remove(src, stage);
    }

    sd_unlock();
    return true;
}

bool sd_stage_data_for_upload(void)
{
    return stage_file_for_upload(CSV_DATA_FILE, CSV_UPLOAD_STAGE);
}

bool sd_stage_merged_for_upload(void)
{
    return stage_file_for_upload(CSV_MERGED_FILE, CSV_MERGE_STAGE);
}

bool sd_delete_upload_stage(void)
{
    sd_lock();
    bool ok = (remove(CSV_UPLOAD_STAGE) == 0);
    sd_unlock();
    if (ok) ESP_LOGI(TAG, "Deleted %s", CSV_UPLOAD_STAGE);
    else    ESP_LOGW(TAG, "Cannot delete %s (may not exist)", CSV_UPLOAD_STAGE);
    return ok;
}

bool sd_delete_merge_stage(void)
{
    sd_lock();
    bool ok = (remove(CSV_MERGE_STAGE) == 0);
    sd_unlock();
    if (ok) ESP_LOGI(TAG, "Deleted %s", CSV_MERGE_STAGE);
    else    ESP_LOGW(TAG, "Cannot delete %s (may not exist)", CSV_MERGE_STAGE);
    return ok;
}

int sd_read_chunk(const char *path, char *buf, size_t buf_size, long *offset)
{
    if (!buf || buf_size < 2 || !offset || !path) return -1;

    sd_lock();
    FILE *f = fopen(path, "r");
    if (!f) { sd_unlock(); return 0; } // file absent = EOF

    if (fseek(f, *offset, SEEK_SET) != 0) {
        fclose(f);
        sd_unlock();
        return 0;
    }

    size_t n = fread(buf, 1, buf_size - 1, f);
    bool is_eof = feof(f);
    fclose(f);
    sd_unlock();

    if (n == 0) return 0; // nothing left to read

    // If we did not reach EOF there is more data in the file.
    // Truncate our buffer to the last complete line so the CSV parser never
    // receives a half-written record.
    if (!is_eof) {
        size_t last_nl = n;
        while (last_nl > 0 && buf[last_nl - 1] != '\n') last_nl--;
        if (last_nl == 0) {
            // A single record is longer than our buffer — extremely unlikely
            // with ~80-byte GPS records but handled safely: skip it.
            ESP_LOGW(TAG, "sd_read_chunk: record exceeds buffer (%zu bytes) at offset %ld — skipping",
                     buf_size, *offset);
            *offset += (long)n;
            buf[0] = '\0';
            return 0;
        }
        n = last_nl;
    }

    buf[n] = '\0';
    *offset += (long)n;
    return (int)n;
}
