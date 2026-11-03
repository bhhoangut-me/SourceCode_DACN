#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include "pid.h"
#include "config/robot_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pid_controller_t pi;
    float target_velocity_mps;  // Vận tốc dài đặt v (m/s)
    float target_rpm;           // Vận tốc góc động cơ đặt (RPM)
    float actual_rpm;           // Vận tốc góc thực tế từ encoder (RPM)
    float actual_velocity_mps;  // Vận tốc dài thực tế xe (m/s)
    float output_pwm;           // Giá trị PWM (0 - 255)
} speed_controller_t;

void speed_control_init(speed_controller_t *ctrl);
float speed_control_linear_to_rpm(float velocity_mps);
float speed_control_rpm_to_linear(float rpm);
float speed_control_update(speed_controller_t *ctrl, float target_velocity_mps, float current_rpm);
void speed_control_reset(speed_controller_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif // SPEED_CONTROL_H

