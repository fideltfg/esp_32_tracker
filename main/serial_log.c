// serial_log.c — Mirror all ESP_LOG output to a rolling file on the SD card.
//
// Design: write directly from the vprintf hook — no background task, no ring
// buffer, no inter-task notifications.  Confirmed safe because the ESP-IDF
// v5.5 text-format log path (LOG_VERSION=1) does NOT hold the log mutex when
// it calls the vprintf hook, so we can safely block on FATFS here.
//
// Flushing: we call fflush() + fsync() on every write.  fflush() drains the
// newlib stdio buffer; fsync() calls FATFS f_sync() which commits the FAT
// directory entry to the SD card.  Without fsync(), stat() returns the old
// (stale) file size and downloads return truncated data.
//
// Concurrency: a non-blocking trylock (0-tick wait) on a FreeRTOS mutex guards
// the shared FILE*.  If a second core is already mid-write, the new message is
// SKIPPED rather than blocking — acceptable for a debug log.
// A per-core re-entrancy depth prevents recursion if FATFS/SDMMC internally
// emits an ESP_LOG message while fsync is in progress.

#include "serial_log.h"
#include "config.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>     // fsync(), fileno()

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static FILE             *s_logfile      = NULL;
static long              s_file_bytes   = 0;
static SemaphoreHandle_t s_mutex        = NULL;
static vprintf_like_t    s_orig_vprintf = NULL;
static volatile bool     s_active       = false;

// Per-core re-entrancy depth: prevents recursive calls when FATFS/SDMMC
// emits its own ESP_LOG messages during fflush/fsync.
static volatile int s_depth[2] = {0, 0};

// ── vprintf hook ──────────────────────────────────────────────────────────────

static int s_log_vprintf(const char *fmt, va_list args)
{
    va_list copy;
    va_copy(copy, args);

    // 1) Always forward to original handler (UART output preserved).
    int ret = s_orig_vprintf(fmt, args);

    // 2) Capture to SD file.
    if (s_active) {
        int core = xPortGetCoreID();

        // Re-entrancy guard: skip if this core is already inside a write.
        if (s_depth[core] == 0) {
            // Non-blocking trylock: skip message if other core holds the mutex.
            if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
                s_depth[core] = 1;

                // ── Rotation ─────────────────────────────────────────────────
                if (s_file_bytes > DEBUG_LOG_MAX_BYTES) {
                    if (s_logfile) { fclose(s_logfile); s_logfile = NULL; }
                    remove(DEBUG_LOG_OLD_FILE);
                    rename(DEBUG_LOG_FILE, DEBUG_LOG_OLD_FILE);
                    s_file_bytes = 0;
                }

                // Re-open after rotation, or recover from earlier open failure.
                if (!s_logfile) {
                    s_logfile = fopen(DEBUG_LOG_FILE, "a");
                }

                if (s_logfile) {
                    int n = vfprintf(s_logfile, fmt, copy);
                    if (n > 0) {
                        s_file_bytes += n;
                        // Flush newlib stdio buffer then sync FATFS directory
                        // entry to SD card so stat() returns the correct size.
                        fflush(s_logfile);
                        fsync(fileno(s_logfile));
                    }
                }

                s_depth[core] = 0;
                xSemaphoreGive(s_mutex);
            }
        }
    }

    va_end(copy);
    return ret;
}

// ── Public API ────────────────────────────────────────────────────────────────

void serial_log_init(void)
{
    if (s_active) return;

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        printf("serial_log: mutex create failed\n");
        return;
    }

    // Discover existing file size for rotation tracking.
    struct stat st;
    if (stat(DEBUG_LOG_FILE, &st) == 0) {
        s_file_bytes = (long)st.st_size;
    }

    s_logfile = fopen(DEBUG_LOG_FILE, "a");
    if (s_logfile) {
        const char *hdr = "=== serial_log started ===\n";
        fwrite(hdr, 1, strlen(hdr), s_logfile);
        fflush(s_logfile);
        fsync(fileno(s_logfile));
        s_file_bytes += (long)strlen(hdr);
        printf("serial_log: file open OK -> %s (%ld B)\n",
               DEBUG_LOG_FILE, s_file_bytes);
    } else {
        printf("serial_log: fopen FAILED for %s\n", DEBUG_LOG_FILE);
    }

    s_orig_vprintf = esp_log_set_vprintf(s_log_vprintf);
    s_active = true;
}

void serial_log_stop(void)
{
    if (!s_active) return;
    s_active = false;
    if (s_orig_vprintf) {
        esp_log_set_vprintf(s_orig_vprintf);
        s_orig_vprintf = NULL;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    if (s_logfile) {
        fflush(s_logfile);
        fsync(fileno(s_logfile));
        fclose(s_logfile);
        s_logfile = NULL;
    }
}
