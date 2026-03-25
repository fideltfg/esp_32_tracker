#pragma once
#include <stdbool.h>

/**
 * @file wifi_upload.h
 * @brief WiFi station connect/disconnect and HTTP CSV upload.
 */

typedef enum {
	WIFI_UPLOAD_FAILED = 0,
	WIFI_UPLOAD_NO_DATA,
	WIFI_UPLOAD_UPLOADED,
} wifi_upload_result_t;

typedef struct {
	wifi_upload_result_t own_data;
	wifi_upload_result_t merged_data;
} wifi_upload_report_t;

/**
 * @brief Initialise the WiFi stack and event loop (call once at boot).
 */
void wifi_stack_init(void);

/**
 * @brief Attempt to connect to the configured AP (WIFI_SSID / WIFI_PASSWORD).
 * @return true if IP was obtained within the retry limit.
 */
bool wifi_connect(void);

/**
 * @brief Disconnect from the current AP and stop the station.
 */
void wifi_disconnect(void);

/**
 * @brief Returns true if the station currently has an IP address.
 */
bool wifi_is_connected(void);

/**
 * @brief POST the contents of CSV_DATA_FILE to UPLOAD_URL.
 * @return Result indicating failure, no data, or successful upload.
 */
wifi_upload_result_t wifi_upload_csv(void);

/**
 * @brief Upload own data first, then peer data (if any).
 *        Calls wifi_upload_csv() then wifi_upload_merged_csv() sequentially.
 *        Returns true only when neither upload failed. The report contains the
 *        per-file outcome so the caller can clear only what was uploaded.
 */
bool wifi_upload_all_csv(wifi_upload_report_t *report);

/**
 * @brief POST the contents of CSV_MERGED_FILE to UPLOAD_URL.
 * @return Result indicating failure, no data, or successful upload.
 */
wifi_upload_result_t wifi_upload_merged_csv(void);

/**
 * @brief Re-upload all .bak archive files from the SD card to the server.
 *        Useful for recovering data after a server-side failure.
 *        Does NOT delete the .bak files after a successful upload.
 * @param out_uploaded Optional — receives the count of files successfully uploaded.
 * @return true if at least one file was uploaded and none failed.
 */
bool wifi_reupload_bak_files(int *out_uploaded);
