#pragma once

// webserver.h — HTTP server: dashboard, live GPS, config, OTA, map.

#include "esp_http_server.h"

/**
 * @brief Start the HTTP server and register all URI handlers.
 * @return Server handle, or NULL on failure.
 */
httpd_handle_t webserver_start(void);

/**
 * @brief Stop the HTTP server.
 */
void webserver_stop(httpd_handle_t server);
