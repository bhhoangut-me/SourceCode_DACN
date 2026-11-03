#include "speed_control.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Quy đổi từ vận tốc dài v (m/s) sang tốc độ trục động cơ n_motor (RPM)
// n_motor = (v / (pi * D)) * 60 * u_dg (Công thức 3.7 và 5.44 trong thuyết minh)
float speed_control_linear_to_rpm(float velocity_mps) {
    if (velocity_mps <= 0.001f) return 0.0f;
    return (velocity_mps / ((float)M_PI * WHEEL_DIAMETER_M)) * 60.0f * DIFF_GEAR_RATIO;
}

// Quy đổi ngược lại từ RPM trục động cơ sang vận tốc dài của xe v (m/s)
float speed_control_rpm_to_linear(float rpm) {
    if (rpm <= 0.001f) return 0.0f;
    return (rpm / (60.0f * DIFF_GEAR_RATIO)) * ((float)M_PI * WHEEL_DIAMETER_M);
}

void speed_control_init(speed_controller_t *ctrl) {
    if (!ctrl) return;
    
    // Khởi tạo bộ PI theo thông số đã nhận dạng trong thuyết minh (Chương 5.6)
    // Kp = 0.3975, Ki = 2.3751, Kd = 0.0, Ts = 0.01s (10ms)
    // Giới hạn PWM [0, 255], bù deadband = 15
    pid_init(&ctrl->pi, 
             MOTOR_PI_KP, 
             MOTOR_PI_KI, 
             0.0f, 
             (float)MOTOR_CONTROL_PERIOD_MS / 1000.0f, 
             0.0f, 
             255.0f, 
             (float)MOTOR_DEADBAND_PWM);
             
    ctrl->target_velocity_mps = 0.0f;
    ctrl->target_rpm = 0.0f;
    ctrl->actual_rpm = 0.0f;
    ctrl->actual_velocity_mps = 0.0f;
    ctrl->output_pwm = 0.0f;
}

float speed_control_update(speed_controller_t *ctrl, float target_velocity_mps, float current_rpm) {
    if (!ctrl) return 0.0f;

    ctrl->target_velocity_mps = target_velocity_mps;
    ctrl->target_rpm = speed_control_linear_to_rpm(target_velocity_mps);
    ctrl->actual_rpm = current_rpm;
    ctrl->actual_velocity_mps = speed_control_rpm_to_linear(current_rpm);

    if (ctrl->target_rpm <= 1.0f) {
        speed_control_reset(ctrl);
        ctrl->output_pwm = 0.0f;
        return 0.0f;
    }

    ctrl->output_pwm = pid_compute(&ctrl->pi, ctrl->target_rpm, ctrl->actual_rpm);
    return ctrl->output_pwm;
}

void speed_control_reset(speed_controller_t *ctrl) {
    if (!ctrl) return;
    pid_reset(&ctrl->pi);
    ctrl->target_velocity_mps = 0.0f;
    ctrl->target_rpm = 0.0f;
    ctrl->output_pwm = 0.0f;
}

