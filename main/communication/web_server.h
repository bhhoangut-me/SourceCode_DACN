#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"
#include "drivers/tctr5000.h"

#ifdef __cplusplus
extern "C" {
#endif

// Cấu hình WiFi Access Point
#define WIFI_AP_SSID        "ROBOT_DACN"
#define WIFI_AP_PASS        "12345678"
#define WIFI_AP_CHANNEL     6
#define WIFI_AP_MAX_CONN    4

// Khởi tạo WiFi AP + HTTP Server + WebSocket + OTA
esp_err_t web_server_init(void);

// Gửi dữ liệu telemetry qua WebSocket (gọi từ control task)
void web_server_send_telemetry(const line_sensor_data_t *line_data);

#ifdef __cplusplus
}
#endif

#endif // WEB_SERVER_H

