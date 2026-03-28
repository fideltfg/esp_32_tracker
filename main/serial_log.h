#pragma once

// serial_log.h — Mirror all ESP_LOG output to a rolling log file on the SD card.
//
// Call serial_log_init() once after the SD card has been mounted.
// All subsequent ESP_LOG messages are written to DEBUG_LOG_FILE (/sdcard/debug.log)
// in addition to the normal UART output.  When the file reaches DEBUG_LOG_MAX_BYTES
// (10 MB) it is rotated: the current file is renamed to DEBUG_LOG_OLD_FILE and a
// fresh log file is started.  Both files are available for download via the web UI.

/**
 * @brief Install the log hook and start the writer task.
 *        Safe to call multiple times (no-op after first call).
 *        Must be called after sd_init() succeeds.
 */
void serial_log_init(void);

/**
 * @brief Remove the log hook and restore the original vprintf handler.
 *        Pending messages already in the queue are flushed before returning.
 */
void serial_log_stop(void);
