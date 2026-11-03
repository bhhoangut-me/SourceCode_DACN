#ifndef WIFI_COMM_H
#define WIFI_COMM_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Khởi tạo Wi-Fi (nếu cần truyền nhận dữ liệu telemetry / debug)
esp_err_t wifi_init_sta(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_COMM_H

