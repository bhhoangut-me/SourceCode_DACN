#ifndef MG90S_H
#define MG90S_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Góc Servo chuẩn khi bánh lái thẳng
#define SERVO_CENTER_DEG            90.0f

// Khởi tạo PWM điều khiển Servo MG90S (mặc định về 90 độ - bánh lái thẳng)
esp_err_t mg90s_init(void);

// Đặt trực tiếp góc Servo từ 0 đến 180 độ (Thẳng = 90 độ)
esp_err_t mg90s_set_servo_angle(float servo_angle_deg);

// Đặt góc bẻ lái bánh xe delta (-45 đến +45 độ) -> Servo = 90 + delta
esp_err_t mg90s_set_angle_deg(float steer_angle_deg);

// Đặt góc bẻ lái bánh xe delta (radian) -> Servo = 90 + (delta * RAD_TO_DEG)
esp_err_t mg90s_set_angle_rad(float steer_angle_rad);

// Lấy góc Servo thực tế hiện tại (0 - 180 độ, thẳng là 90 độ)
float mg90s_get_servo_angle(void);

// Lấy góc bẻ lái bánh xe hiện tại (-45 đến +45 độ, thẳng là 0 độ)
float mg90s_get_steering_angle_deg(void);

// Căn chỉnh góc lệch tâm (Trim offset) cơ khí (mặc định 0.0 độ)
void mg90s_set_trim_offset(float offset_deg);
float mg90s_get_trim_offset(void);

// Đảo chiều quay servo nếu cơ cấu đòn bẩy bị ngược
void mg90s_set_invert(bool invert);
bool mg90s_get_invert(void);

// Ngắt xung điều khiển servo
void mg90s_stop(void);

#ifdef __cplusplus
}
#endif

#endif // MG90S_H
