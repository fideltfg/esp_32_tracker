#pragma once

// upload.h — HTTP CSV upload + OTA firmware updates.
//
// The upload functions use wifi_manager for connectivity (no direct WiFi).
// The CSV→JSON conversion and chunked POST logic is retained from v1.

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef enum {
    UPLOAD_FAILED = 0,
    UPLOAD_NO_DATA,
    UPLOAD_OK,
} upload_result_t;

typedef struct {
    upload_result_t own_data;
    upload_result_t merged_data;
} upload_report_t;

/**
 * @brief Upload own GPS data (CSV_DATA_FILE → server).
 */
upload_result_t upload_own_csv(void);

/**
 * @brief Upload merged peer data (CSV_MERGED_FILE → server).
 */
upload_result_t upload_merged_csv(void);

/**
 * @brief Upload both own and merged data.
 *        Returns true if neither upload failed.
 */
bool upload_all_csv(upload_report_t *report);

/**
 * @brief Re-upload all .bak archive files.
 */
bool upload_reupload_bak(int *out_count);

/**
 * @brief Flash firmware from in-memory buffer (web UI upload).
 * @return true on success (device should reboot after).
 */
bool upload_ota_from_buffer(const uint8_t *data, size_t len);
