#include "wifi.h"
#include "esp_log.h"

static const char *TAG = "WIFI_COMM";

esp_err_t wifi_init_sta(void) {
    ESP_LOGI(TAG, "Wi-Fi Telemetry module ready for connection if needed");
    return ESP_OK;
}

