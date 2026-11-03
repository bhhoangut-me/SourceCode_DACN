#ifndef MG90S_H
#define MG90S_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Khởi tạo PWM điều khiển Servo MG90S
esp_err_t mg90s_init(void);

// Đặt góc lái theo độ (-90 độ đến +90 độ, 0 độ là chính giữa)
esp_err_t mg90s_set_angle_deg(float angle_deg);

// Đặt góc lái theo radian (-0.61 rad đến +0.61 rad theo giới hạn thiết kế)
esp_err_t mg90s_set_angle_rad(float angle_rad);

// Ngắt xung điều khiển servo
void mg90s_stop(void);

#ifdef __cplusplus
}
#endif

#endif // MG90S_H

