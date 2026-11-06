#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>

#include "communication/web_server.h"
#include "tasks/control_task.h"
#include "tasks/obstacle_task.h"
#include "tasks/state_task.h"

static const char *TAG = "MAIN_APP";

void app_main(void) {
  ESP_LOGI(
      TAG,
      "=== ROBOT DO LINE TRÁNH VẬT CẢN (MECHATRONICS PROJECT - HCMUT) ===");

  // 1. Khởi tạo NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // 2. Khởi tạo hệ thống quản lý trạng thái & LED/Nút nhấn
  state_manager_init();

  // 3. Khởi tạo driver ngoại vi điều khiển (TCRT5000, MG90S, DRV8871, Encoder)
  control_system_init();

  // 4. Khởi tạo cảm biến khoảng cách VL53L0X
  obstacle_task_init();

  // 5. Khởi tạo Wi-Fi SoftAP, Web Dashboard, WebSocket & OTA Server
  web_server_init();

  ESP_LOGI(
      TAG,
      "Khoi tao toan bo phan cung hoan tat. Khoi chay cac FreeRTOS Tasks...");

  // 5. Tạo các FreeRTOS Tasks:
  // - Control_Task: Vòng lặp điều khiển thời gian thực (10ms / 100Hz), ưu tiên
  // cao nhất, chạy trên Core 1
  xTaskCreatePinnedToCore(Control_Task, "Control_Task", 4096, NULL,
                          configMAX_PRIORITIES - 2, // Mức ưu tiên cao
                          NULL,
                          1 // Ghim Core 1
  );

  // - Obstacle_Task: Quét ToF xác định nhánh rẽ trước khi chạy, chạy trên Core
  // 0
  xTaskCreatePinnedToCore(Obstacle_Task, "Obstacle_Task", 4096, NULL, 3, NULL,
                          0 // Ghim Core 0
  );

  // - State_Task: Máy trạng thái, xử lý nút bấm, đèn LED và giám sát, chạy trên
  // Core 0
  xTaskCreatePinnedToCore(State_Task, "State_Task", 4096, NULL, 2, NULL,
                          0 // Ghim Core 0
  );

  ESP_LOGI(
      TAG,
      "San sang! Nhan nut BOOT (GPIO 0) tren mach de bat dau chuong trinh.");
}
