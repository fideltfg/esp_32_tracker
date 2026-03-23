#pragma once
#include <stdbool.h>
#include <stdint.h>

/**
 * @file espnow_sync.h
 * @brief ESP-NOW peer discovery and bidirectional CSV data exchange.
 *
 * Protocol summary (all frames sent over ESP-NOW):
 *
 *   ANNOUNCE  – broadcast by every device periodically.
 *               Payload: own device MAC (6 bytes).
 *
 *   SYNC_REQ  – unicast sent to the announcer when we want to exchange data.
 *               Payload: our device MAC (6 bytes).
 *
 *   DATA      – carries a chunk of CSV text (up to ESPNOW_MTU bytes).
 *               Sequence number allows re-assembly.
 *
 *   DONE      – signals end of DATA stream from one side.
 *
 * After a SYNC_REQ/DONE handshake BOTH devices end up with the merged dataset.
 */

/**
 * @brief Initialise ESP-NOW (must be called after WiFi is started in STA mode,
 *        or after esp_wifi_set_mode(WIFI_MODE_STA) if WiFi is not used).
 * @return true on success.
 */
bool espnow_init(void);

/**
 * @brief De-initialise ESP-NOW.
 */
void espnow_deinit(void);

/**
 * @brief Run one peer-discovery + data-exchange round.
 *
 * The function:
 *  1. Broadcasts ANNOUNCE for ESPNOW_DISCOVER_MS milliseconds.
 *  2. Responds to any incoming SYNC_REQ by streaming our CSV.
 *  3. If we receive an ANNOUNCE we send a SYNC_REQ and await the peer's CSV.
 *  4. Merges received data into our local CSV_DATA_FILE.
 *
 * Designed to be called repeatedly from the main state-machine loop.
 *
 * @return Number of peers successfully synced with this round (0 is normal
 *         when no peers are nearby).
 */
int espnow_sync_round(void);

/**
 * @brief Reset all per-peer "last sent row" offsets to zero.
 *
 * Must be called whenever data.csv is deleted and recreated, so that
 * the next sync with each peer starts from row 0 of the fresh file.
 */
void espnow_sync_reset_peer_states(void);

/**
 * @brief Reset only the per-peer own-data timestamp watermarks.
 *
 * Call this when sync_data.csv is deleted and recreated but merged data is
 * retained, so peers do not miss new rows from the fresh file.
 */
void espnow_sync_reset_own_state(void);

/**
 * @brief Reset only the per-peer merged-data row offsets.
 *
 * Call this when sync_merged.csv is deleted and recreated but own data is
 * retained, so peers do not skip rows in the fresh merged file.
 */
void espnow_sync_reset_merged_state(void);
