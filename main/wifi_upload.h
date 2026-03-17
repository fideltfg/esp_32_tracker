#pragma once
#include <stdbool.h>

/**
 * @file wifi_upload.h
 * @brief WiFi station connect/disconnect and HTTP CSV upload.
 */

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
 *        On HTTP 200/201 the function returns true; the caller should then
 *        delete the file via sd_delete_data_file().
 * @return true on successful upload (server accepted the data).
 */
bool wifi_upload_csv(void);

/**
 * @brief Upload own data first, then peer data (if any).
 *        Calls wifi_upload_csv() then wifi_upload_merged_csv() sequentially.
 *        Returns true only if the own-data upload succeeded (peer upload
 *        failure is logged but does not affect the return value).
 */
bool wifi_upload_all_csv(void);

/**
 * @brief POST the contents of CSV_MERGED_FILE to UPLOAD_URL.
 *        Silently succeeds (returns true) if the file does not exist yet.
 * @return true on successful upload or if there was nothing to upload.
 */
bool wifi_upload_merged_csv(void);
