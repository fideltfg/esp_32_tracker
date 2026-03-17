idf_component_register(SRCS "sd_storage.c" "wifi_upload.c" "data_merge.c" "espnow_sync.c" "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver nvs_flash esp_wifi esp_http_server fatfs sdmmc
                                esp_http_client esp_netif esp_event esp_timer log)

