#include "jgb37_encoder.h"
#include "config/pin_config.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include <math.h>

static volatile int64_t s_pulse_count = 0;
static int64_t s_last_pulse_count = 0;
static float s_current_rpm = 0.0f;

static void IRAM_ATTR encoder_isr_handler(void* arg) {
    int level_a = gpio_get_level(JGB37_ENC_A_GPIO);
    int level_b = gpio_get_level(JGB37_ENC_B_GPIO);

    // Giải mã kênh cầu phương (Quadrature decoding)
    if (level_a == level_b) {
        s_pulse_count++;
    } else {
        s_pulse_count--;
    }
}

esp_err_t jgb37_encoder_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << JGB37_ENC_A_GPIO) | (1ULL << JGB37_ENC_B_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,    // GPIO 34, 35 là input-only, không có pull-up nội
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE        // Bắt cạnh lên và xuống để tăng độ phân giải
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) return err;

    // Cài đặt ISR service nếu chưa có
    gpio_install_isr_service(0);

    // Đăng ký ngắt cho kênh A
    gpio_isr_handler_add(JGB37_ENC_A_GPIO, encoder_isr_handler, (void*)JGB37_ENC_A_GPIO);

    s_pulse_count = 0;
    s_last_pulse_count = 0;
    s_current_rpm = 0.0f;

    return ESP_OK;
}

int64_t jgb37_encoder_get_count(void) {
    return s_pulse_count;
}

void jgb37_encoder_reset_count(void) {
    s_pulse_count = 0;
    s_last_pulse_count = 0;
    s_current_rpm = 0.0f;
}

float jgb37_encoder_update_rpm(float delta_time_s) {
    if (delta_time_s <= 0.0001f) return s_current_rpm;

    int64_t current_count = s_pulse_count;
    int64_t delta_pulses = current_count - s_last_pulse_count;
    s_last_pulse_count = current_count;

    // RPM = (số xung / số xung trên 1 vòng) / (delta_time_s / 60)
    float revs = (float)delta_pulses / ENCODER_PPR;
    s_current_rpm = (revs / delta_time_s) * 60.0f;

    return s_current_rpm;
}

float jgb37_encoder_get_rpm(void) {
    return s_current_rpm;
}
