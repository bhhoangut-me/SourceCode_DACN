#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "control/speed_control.h"
#include "drivers/tctr5000.h"

#ifdef __cplusplus
extern "C" {
#endif

// Khởi tạo toàn bộ driver điều khiển và cảm biến phục vụ control loop
void control_system_init(void);

// Đặt vận tốc dài mục tiêu v_d (m/s)
void control_set_target_speed(float speed_mps);

// Lấy thông số telemetry giám sát
float control_get_current_speed(void);
float control_get_lateral_error_mm(void);
float control_get_steer_angle_deg(void);

// FreeRTOS Task điều khiển thời gian thực (10ms chu kỳ - Core 1)
void Control_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_TASK_H
