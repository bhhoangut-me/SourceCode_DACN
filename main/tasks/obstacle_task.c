#include "obstacle_task.h"
#include "drivers/vl53l0x.h"
#include "tasks/state_task.h"
#include "esp_log.h"

static const char *TAG = "OBSTACLE_TASK";

void obstacle_task_init(void) {
    esp_err_t err = vl53l0x_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Khoi tao VL53L0X that bai: %d", err);
    } else {
        ESP_LOGI(TAG, "Khoi tao VL53L0X thanh cong");
    }
}

void Obstacle_Task(void *pvParameters) {
    ESP_LOGI(TAG, "Obstacle_Task khoi dong tren Core %d", xPortGetCoreID());

    while (1) {
        robot_state_t state = state_get_current();

        if (state == ROBOT_STATE_DETECT_OBSTACLE) {
            ESP_LOGI(TAG, "Bat dau tien trinh do tim vat can tai vung phan nhanh...");
            
            float measured_dist = 0.0f;
            route_selection_t route = vl53l0x_decide_route(&measured_dist);

            state_set_selected_route(route, measured_dist);

            // Nghỉ ngắn để chuẩn bị chạy
            vTaskDelay(pdMS_TO_TICKS(500));

            // Kích hoạt bám line
            state_set(ROBOT_STATE_LINE_TRACKING);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
