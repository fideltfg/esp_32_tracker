#pragma once

// wifi_manager.h — Single WiFi owner for the entire application.
//
// Owns the WiFi event handler, STA mode, scan, multi-AP connect, and
// auto-reconnect.  No other module should register WiFi event handlers.

#include <stdbool.h>

/**
 * @brief Initialise WiFi subsystem: netif, event loop, STA driver, start.
 *        Call once at boot, after nvs_flash_init() and config_init().
 */
void wifi_mgr_init(void);

/**
 * @brief Scan for available APs, match against the config AP list, and
 *        connect to the one with the strongest RSSI.
 *        Falls back to iterating the config list in order if scan fails.
 * @return true if an IP address was obtained.
 */
bool wifi_mgr_connect(void);

/**
 * @brief Disconnect from the current AP.
 */
void wifi_mgr_disconnect(void);

/**
 * @brief Returns true if the station currently has an IP address.
 *        Queries the driver directly (no event-group race).
 */
bool wifi_mgr_is_connected(void);

/**
 * @brief Copy the current IP address into buf (e.g. "192.168.1.42").
 *        Returns "0.0.0.0" if not connected.
 */
void wifi_mgr_get_ip_str(char *buf, int buf_len);

/**
 * @brief Returns true if no valid WiFi credentials are configured (factory
 *        reset state or first boot).  An SSID is considered invalid when the
 *        AP list is empty or the first entry is blank / a placeholder
 *        (starts with '<').
 */
bool wifi_mgr_needs_provisioning(void);

/**
 * @brief Start a SoftAP (APSTA mode) with the given open SSID so the user
 *        can connect and reach the /provision page at 192.168.4.1.
 *        Must be called after wifi_mgr_init().
 */
void wifi_mgr_start_provisioning_ap(const char *ap_ssid);
