idf_component_register(SRCS "main.c" "webserver.c" "upload.c" "power.c"
                            "lcd.c" "rtc.c" "imu.c" "gps.c" "wifi_manager.c"
                            "config.c" "sd_storage.c" "data_merge.c" "espnow_sync.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver nvs_flash esp_wifi esp_http_server fatfs sdmmc
                                esp_http_client esp_netif esp_event esp_timer log
                                app_update esp_https_ota)



