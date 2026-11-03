#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float ts;             // Thời gian lấy mẫu (giây)
    
    float out_min;        // Giới hạn ngõ ra dưới
    float out_max;        // Giới hạn ngõ ra trên
    float deadband;       // Bù vùng chết
    
    float integral;       // Thành phần tích phân
    float prev_error;     // Sai số chu kỳ trước
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float ts, float out_min, float out_max, float deadband);
float pid_compute(pid_controller_t *pid, float setpoint, float feedback);
void pid_reset(pid_controller_t *pid);

#ifdef __cplusplus
}
#endif

#endif // PID_H

