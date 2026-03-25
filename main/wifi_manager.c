// wifi_manager.c — Single WiFi owner for the entire application.
//
// Scan-based AP selection picks the strongest RSSI match from the config AP
// list.  All WiFi event handling is centralised here.

#include "wifi_manager.h"
#include "config.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

static const char *TAG = "WIFI_MGR";

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define SCAN_MAX_APS        16

static EventGroupHandle_t s_event_group = NULL;
static int                s_retry_count  = 0;
static int                s_max_retries  = 3;
static bool               s_started      = false;

// ── Event handler (the ONLY WiFi event handler in the app) ───────────────────

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    if (base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            if (s_retry_count < s_max_retries) {
                esp_wifi_connect();
                s_retry_count++;
                ESP_LOGW(TAG, "Disconnected — retry %d/%d", s_retry_count, s_max_retries);
            } else {
                xEventGroupSetBits(s_event_group, WIFI_FAIL_BIT);
            }
        }
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&evt->ip_info.ip));
        s_retry_count = 0;
        xEventGroupSetBits(s_event_group, WIFI_CONNECTED_BIT);
    }
}

// ── Init ─────────────────────────────────────────────────────────────────────

void wifi_mgr_init(void)
{
    if (s_started) return;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    s_event_group = xEventGroupCreate();
    configASSERT(s_event_group);

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Apply power-save mode
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

    s_max_retries = config_get()->wifi_max_retry;
    s_started = true;
    ESP_LOGI(TAG, "WiFi STA initialised");
}

// ── Scan + connect to strongest matching AP ──────────────────────────────────

// Try connecting to a specific SSID/password
static bool try_connect(const char *ssid, const char *password)
{
    xEventGroupClearBits(s_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    s_retry_count = 0;

    wifi_config_t wifi_cfg = { 0 };
    strncpy((char *)wifi_cfg.sta.ssid, ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, password, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(s_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(15000));
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

bool wifi_mgr_connect(void)
{
    const tracker_config_t *cfg = config_get();
    if (cfg->wifi_ap_count == 0) {
        ESP_LOGW(TAG, "No WiFi APs configured");
        return false;
    }

    // ── Phase 1: Scan and pick strongest matching AP ─────────────────────────
    wifi_scan_config_t scan_cfg = {
        .show_hidden = true,
        .scan_type   = WIFI_SCAN_TYPE_ACTIVE,
    };

    esp_err_t scan_err = esp_wifi_scan_start(&scan_cfg, true);
    if (scan_err == ESP_OK) {
        uint16_t ap_num = 0;
        esp_wifi_scan_get_ap_num(&ap_num);

        if (ap_num > SCAN_MAX_APS) ap_num = SCAN_MAX_APS;

        wifi_ap_record_t ap_records[SCAN_MAX_APS];
        esp_wifi_scan_get_ap_records(&ap_num, ap_records);

        ESP_LOGI(TAG, "Scan found %d AP(s)", ap_num);

        // Find the configured AP with the strongest RSSI
        int best_cfg_idx = -1;
        int8_t best_rssi = -127;

        for (int s = 0; s < ap_num; s++) {
            for (int c = 0; c < cfg->wifi_ap_count; c++) {
                if (strcmp((char *)ap_records[s].ssid, cfg->wifi_aps[c].ssid) == 0) {
                    if (ap_records[s].rssi > best_rssi) {
                        best_rssi    = ap_records[s].rssi;
                        best_cfg_idx = c;
                    }
                }
            }
        }

        if (best_cfg_idx >= 0) {
            ESP_LOGI(TAG, "Best match: \"%s\" (RSSI %d dBm)",
                     cfg->wifi_aps[best_cfg_idx].ssid, best_rssi);
            if (try_connect(cfg->wifi_aps[best_cfg_idx].ssid,
                            cfg->wifi_aps[best_cfg_idx].password)) {
                return true;
            }
            ESP_LOGW(TAG, "Best-RSSI AP failed, falling through to ordered list");
        } else {
            ESP_LOGW(TAG, "No configured AP found in scan results");
        }
    } else {
        ESP_LOGW(TAG, "WiFi scan failed: %s — falling back to ordered list",
                 esp_err_to_name(scan_err));
    }

    // ── Phase 2: Fallback — iterate config list in order ─────────────────────
    for (int i = 0; i < cfg->wifi_ap_count; i++) {
        ESP_LOGI(TAG, "Trying AP %d/%d: \"%s\"", i + 1, cfg->wifi_ap_count,
                 cfg->wifi_aps[i].ssid);
        if (try_connect(cfg->wifi_aps[i].ssid, cfg->wifi_aps[i].password)) {
            return true;
        }
    }

    ESP_LOGE(TAG, "All %d AP(s) failed", cfg->wifi_ap_count);
    return false;
}

// ── Disconnect ───────────────────────────────────────────────────────────────

void wifi_mgr_disconnect(void)
{
    esp_wifi_disconnect();
}

// ── Status ───────────────────────────────────────────────────────────────────

bool wifi_mgr_is_connected(void)
{
    wifi_ap_record_t ap_info;
    return (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);
}

void wifi_mgr_get_ip_str(char *buf, int buf_len)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip_info;
    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK &&
        ip_info.ip.addr != 0) {
        snprintf(buf, buf_len, IPSTR, IP2STR(&ip_info.ip));
    } else {
        strncpy(buf, "0.0.0.0", buf_len);
    }
}
