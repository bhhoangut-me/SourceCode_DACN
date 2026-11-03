#include "pid.h"
#include <math.h>

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float ts, float out_min, float out_max, float deadband) {
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->ts = (ts > 0.0f) ? ts : 0.01f;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->deadband = deadband;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_compute(pid_controller_t *pid, float setpoint, float feedback) {
    if (!pid) return 0.0f;

    float error = setpoint - feedback;

    // Thành phần tỷ lệ (P)
    float p_term = pid->kp * error;

    // Tích phân thử nghiệm (I)
    float i_term = pid->integral + pid->ki * error * pid->ts;

    // Thành phần vi phân (D)
    float d_term = 0.0f;
    if (pid->ts > 0.0f) {
        d_term = pid->kd * (error - pid->prev_error) / pid->ts;
    }

    float output = p_term + i_term + d_term;

    // Anti-windup (Clamping tích phân khi chạm ngưỡng biên)
    if (output > pid->out_max) {
        output = pid->out_max;
        // Chỉ cho phép tích phân nếu lỗi kéo output về phía giảm
        if (error < 0.0f) {
            pid->integral = i_term;
        }
    } else if (output < pid->out_min) {
        output = pid->out_min;
        // Chỉ cho phép tích phân nếu lỗi kéo output về phía tăng
        if (error > 0.0f) {
            pid->integral = i_term;
        }
    } else {
        pid->integral = i_term;
    }

    // Bù deadband cho động cơ
    if (output > 0.0f && pid->deadband > 0.0f) {
        output += pid->deadband;
        if (output > pid->out_max) output = pid->out_max;
    } else if (output < 0.0f && pid->deadband > 0.0f) {
        output -= pid->deadband;
        if (output < pid->out_min) output = pid->out_min;
    }

    pid->prev_error = error;
    return output;
}

void pid_reset(pid_controller_t *pid) {
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

