#ifndef STATE_TASK_H
#define STATE_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "drivers/vl53l0x.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ROBOT_STATE_INIT = 0,
    ROBOT_STATE_WAIT_START,
    ROBOT_STATE_DETECT_OBSTACLE,
    ROBOT_STATE_LINE_TRACKING,
    ROBOT_STATE_INTERSECTION,
    ROBOT_STATE_FINISH,
    ROBOT_STATE_STOPPED
} robot_state_t;

// Khởi tạo máy trạng thái
void state_manager_init(void);

// Lấy trạng thái hiện tại
robot_state_t state_get_current(void);

// Đặt trạng thái mới
void state_set(robot_state_t new_state);

// Lấy nhánh đường đã chọn
route_selection_t state_get_selected_route(void);

// Đặt nhánh đường đã chọn
void state_set_selected_route(route_selection_t route, float distance_mm);

// Kiểm tra tín hiệu bắt đầu chạy (nút BOOT GPIO 0)
bool state_is_start_button_pressed(void);

// FreeRTOS Task quản lý trạng thái hệ thống
void State_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // STATE_TASK_H
