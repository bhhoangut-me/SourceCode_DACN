#include "state_task.h"
#include "config/pin_config.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "STATE_TASK";

static robot_state_t s_current_state = ROBOT_STATE_INIT;
static route_selection_t s_selected_route = ROUTE_UNKNOWN;
static float s_obstacle_distance_mm = 0.0f;
static SemaphoreHandle_t s_state_mutex = NULL;

void state_manager_init(void) {
    if (!s_state_mutex) {
        s_state_mutex = xSemaphoreCreateMutex();
    }

    // Cấu hình nút nhấn BOOT (GPIO 0)
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BOOT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);

    // Cấu hình LED trạng thái (LED Đỏ 17, LED Xanh 15)
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_RED_GPIO) | (1ULL << LED_GREEN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_conf);

    gpio_set_level(LED_RED_GPIO, 0);
    gpio_set_level(LED_GREEN_GPIO, 0);
}

robot_state_t state_get_current(void) {
    robot_state_t st = ROBOT_STATE_STOPPED;
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        st = s_current_state;
        xSemaphoreGive(s_state_mutex);
    }
    return st;
}

void state_set(robot_state_t new_state) {
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_current_state != new_state) {
            ESP_LOGI(TAG, "Chuyen trang thai: %d -> %d", s_current_state, new_state);
            s_current_state = new_state;
        }
        xSemaphoreGive(s_state_mutex);
    }
}

route_selection_t state_get_selected_route(void) {
    route_selection_t r = ROUTE_UNKNOWN;
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        r = s_selected_route;
        xSemaphoreGive(s_state_mutex);
    }
    return r;
}

void state_set_selected_route(route_selection_t route, float distance_mm) {
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_selected_route = route;
        s_obstacle_distance_mm = distance_mm;
        ESP_LOGI(TAG, "Da xac dinh tuyen duong: %s (Khoang cach vat can: %.1f mm)",
                 (route == ROUTE_UPPER) ? "NHANH TREN" : "NHANH DUOI", distance_mm);
        xSemaphoreGive(s_state_mutex);
    }
}

bool state_is_start_button_pressed(void) {
    return (gpio_get_level(BOOT_GPIO) == 0);
}

void State_Task(void *pvParameters) {
    ESP_LOGI(TAG, "State_Task khoi dong tren Core %d", xPortGetCoreID());

    // Chuyển sang chờ tín hiệu Start
    state_set(ROBOT_STATE_WAIT_START);

    while (1) {
        robot_state_t state = state_get_current();

        switch (state) {
            case ROBOT_STATE_WAIT_START:
                // Nhấp nháy LED xanh báo sẵn sàng
                gpio_set_level(LED_GREEN_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level(LED_GREEN_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(200));

                // Bấm nút BOOT (GPIO 0) để khởi chạy
                if (state_is_start_button_pressed()) {
                    ESP_LOGI(TAG, "Nut START duoc nhan! Bat dau do vat can...");
                    state_set(ROBOT_STATE_DETECT_OBSTACLE);
                }
                break;

            case ROBOT_STATE_DETECT_OBSTACLE:
                // LED đỏ sáng trong lúc quét ToF
                gpio_set_level(LED_RED_GPIO, 1);
                gpio_set_level(LED_GREEN_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case ROBOT_STATE_LINE_TRACKING:
                // LED xanh sáng cố định khi đang bám line
                gpio_set_level(LED_RED_GPIO, 0);
                gpio_set_level(LED_GREEN_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case ROBOT_STATE_INTERSECTION:
                // LED xanh chớp nhanh khi đang rẽ nhánh ngã ba
                gpio_set_level(LED_GREEN_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(LED_GREEN_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(50));
                break;

            case ROBOT_STATE_FINISH:
                // Kết thúc hành trình: LED xanh sáng, LED đỏ tắt
                gpio_set_level(LED_GREEN_GPIO, 1);
                gpio_set_level(LED_RED_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;

            case ROBOT_STATE_STOPPED:
            default:
                gpio_set_level(LED_GREEN_GPIO, 0);
                gpio_set_level(LED_RED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
        }
    }
}
