#ifndef JGB37_MOTOR_H
#define JGB37_MOTOR_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Khởi tạo PWM điều khiển driver động cơ cầu H (DRV8871 / TB67H450)
esp_err_t jgb37_motor_init(void);

// Đặt giá trị PWM [-255, 255]: Dương là tiến, Âm là lùi, 0 là dừng thả nổi
esp_err_t jgb37_motor_set_pwm(float pwm);

// Thắng động cơ (chập 2 cực cuộn dây xuống GND hoặc kéo lên VCC)
esp_err_t jgb37_motor_brake(void);

// Dừng tự do
esp_err_t jgb37_motor_stop(void);

#ifdef __cplusplus
}
#endif

#endif // JGB37_MOTOR_H

