#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_http_server.h"
#include "esp_event.h"
#include "esp_err.h"

// ---------------------------------------------------------------------------
// WiFi credentials
//
// IMPORTANT: hardcoding credentials in a header bakes them into the firmware
// binary. For anything you intend to share or commit to git, move these to
// menuconfig (Component config -> ESP WIFI -> ...) or NVS storage.
// ---------------------------------------------------------------------------
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

#define WIFI_SUCESS (1 << 0)
#define WIFI_FAILURE (1 << 1)
#define MAX_FAILURES 10

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------
esp_err_t connect_wifi();
httpd_handle_t start_webserver();

// ---------------------------------------------------------------------------
// Event handlers (registered by connect_wifi)
// ---------------------------------------------------------------------------
void wifi_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data);
void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data);