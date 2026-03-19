#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @file sd_storage.h
 * @brief SDMMC initialisation, CSV helpers, and file lifecycle management.
 *
 * All paths assume the SD card is mounted at SD_MOUNT_POINT (see config.h).
 * The CSV schema is:
 *   timestamp,lat,lon,alt,speed,accel_x,accel_y,accel_z,device_mac
 */

/**
 * @brief Acquire / release the SD card mutex.
 *
 * Any task that calls fopen/fclose on SD files directly (e.g. data_merge)
 * must hold this lock for the entire operation to prevent concurrent access
 * from the data-generator task or any other task using sd_storage helpers.
 *
 * sd_lock() blocks indefinitely; sd_unlock() must always be paired with it.
 */
void sd_lock(void);
void sd_unlock(void);

/**
 * @brief Mount the SD card via SDMMC in 1-bit mode.
 * @return true on success, false on any error.
 */
bool sd_init(void);

/**
 * @brief Unmount and release the SD card.
 */
void sd_deinit(void);

/**
 * @brief Ensure CSV_DATA_FILE exists with the correct header row.
 *        Creates the file if absent; does nothing if already present.
 * @return true on success.
 */
bool sd_ensure_data_file(void);

/**
 * @brief Append a single pre-formatted CSV line (including trailing '\n') to
 *        CSV_DATA_FILE.  Caller is responsible for correct column order.
 * @param line  Null-terminated string, e.g. "1710000000,1.23,4.56,...,AA:BB:..\n"
 * @return true on success.
 */
bool sd_append_record(const char *line);

/**
 * @brief Count the number of data rows in CSV_DATA_FILE (header excluded).
 * @return Row count, or -1 on error.
 */
int  sd_record_count(void);

/**
 * @brief Delete CSV_DATA_FILE (called after a successful upload).
 * @return true on success.
 */
bool sd_delete_data_file(void);

// ─── Peer-data (merged) file ──────────────────────────────────────────────────

/**
 * @brief Ensure CSV_MERGED_FILE exists with the correct header row.
 */
bool sd_ensure_merged_file(void);

/**
 * @brief Append a pre-formatted CSV line to CSV_MERGED_FILE.
 */
bool sd_append_merged_record(const char *line);

/**
 * @brief Read the entire CSV_MERGED_FILE into a caller-allocated buffer.
 * @return Bytes read, or -1 on error.
 */
int  sd_read_merged(char *buf, size_t buf_size);

/**
 * @brief Delete CSV_MERGED_FILE (called after a successful upload).
 * @return true on success.
 */
bool sd_delete_merged_file(void);

/**
 * @brief Read the entire CSV_DATA_FILE into a caller-allocated buffer.
 *        The buffer is null-terminated.  If the file is larger than buf_size-1
 *        the content is truncated.
 * @param buf       Destination buffer.
 * @param buf_size  Size of buffer in bytes.
 * @return Number of bytes read, or -1 on error.
 */
int  sd_read_all(char *buf, size_t buf_size);

/**
 * @brief Append one row to CONNECTIONS_LOG_FILE.
 *        Creates the file with a header row on first call.
 * @param peer_mac    6-byte MAC address of the peer device.
 * @param records_rx  Number of records received from the peer this sync.
 * @param records_tx  Number of records sent to the peer this sync.
 * @return true on success.
 */
bool sd_log_connection(const uint8_t *peer_mac, int records_rx, int records_tx);
