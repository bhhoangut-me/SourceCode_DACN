#ifndef OBSTACLE_TASK_H
#define OBSTACLE_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

// Khởi tạo phần cứng cảm biến vật cản
void obstacle_task_init(void);

// FreeRTOS Task đo khoảng cách và quyết định tuyến đường
void Obstacle_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // OBSTACLE_TASK_H
