#include "mg90s.h"
#include "config/pin_config.h"
#include "config/robot_config.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MG90S";

#define SERVO_LEDC_TIMER            LEDC_TIMER_0
#define SERVO_LEDC_MODE             LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL          LEDC_CHANNEL_0
#define SERVO_LEDC_DUTY_RES         LEDC_TIMER_14_BIT // 14 bit (0 - 16383)
#define SERVO_LEDC_FREQ_HZ          50                 // Tần số RC servo 50 Hz (chu kỳ 20 ms)

// Độ rộng xung MG90S chuẩn: 0 độ = 500 us, 90 độ (thẳng) = 1500 us, 180 độ = 2500 us
#define SERVO_MIN_PULSE_US          500   // 0 độ
#define SERVO_MID_PULSE_US          1500  // 90 độ (thẳng bánh)
#define SERVO_MAX_PULSE_US          2500  // 180 độ
#define SERVO_PERIOD_US             20000 // 20 ms = 20,000 us

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
#endif

static float s_current_servo_angle = SERVO_CENTER_DEG; // Mặc định 90 độ
static float s_current_steer_deg   = 0.0f;             // Mặc định 0 độ bánh thẳng
static float s_trim_offset_deg     = 0.0f;             // Bù lệch cơ khí
static bool  s_invert_direction    = true;             // Đã đảo chiều: lệch trái bẻ lái sang trái để bám line

static uint32_t pulse_to_duty(uint32_t pulse_us) {
    return (uint32_t)(((uint64_t)pulse_us * ((1 << 14) - 1)) / SERVO_PERIOD_US);
}

esp_err_t mg90s_init(void) {
    ledc_timer_config_t timer_conf = {
        .speed_mode       = SERVO_LEDC_MODE,
        .duty_resolution  = SERVO_LEDC_DUTY_RES,
        .timer_num        = SERVO_LEDC_TIMER,
        .freq_hz          = SERVO_LEDC_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) return err;

    ledc_channel_config_t ch_conf = {
        .gpio_num       = MG90S_PWM_GPIO,
        .speed_mode     = SERVO_LEDC_MODE,
        .channel        = SERVO_LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = SERVO_LEDC_TIMER,
        .duty           = pulse_to_duty(SERVO_MID_PULSE_US), // Khởi động ngay tại 1500us (90 độ)
        .hpoint         = 0
    };
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) return err;

    // Đưa bánh xe về thẳng tắp (Servo = 90 độ)
    ESP_LOGI(TAG, "Khoi tao Servo MG90S thanh cong: Chan GPIO %d, Vi tri mac dinh: 90 do (1500 us)", MG90S_PWM_GPIO);
    return mg90s_set_servo_angle(SERVO_CENTER_DEG);
}

esp_err_t mg90s_set_servo_angle(float servo_angle_deg) {
    // Giới hạn an toàn toàn dải của servo [0 độ, 180 độ]
    if (servo_angle_deg < 0.0f)   servo_angle_deg = 0.0f;
    if (servo_angle_deg > 180.0f) servo_angle_deg = 180.0f;

    s_current_servo_angle = servo_angle_deg;

    // Quy đổi góc [0, 180] tương ứng pulse [500, 2500] us:
    // pulse = 500 + (angle / 180) * 2000 us
    // Tại 90 độ -> pulse = 500 + (90/180)*2000 = 1500 us (thẳng tắp)
    float pulse_us = SERVO_MIN_PULSE_US + (servo_angle_deg / 180.0f) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);

    uint32_t duty = pulse_to_duty((uint32_t)pulse_us);
    esp_err_t err = ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    if (err == ESP_OK) {
        err = ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
    }
    return err;
}

esp_err_t mg90s_set_angle_deg(float steer_angle_deg) {
    // 1. Giới hạn an toàn góc bẻ lái theo cấu hình ROBOT_MAX_STEER_ANGLE_RAD (~65 độ)
    float max_steer = ROBOT_MAX_STEER_ANGLE_RAD * RAD_TO_DEG;
    if (steer_angle_deg > max_steer)  steer_angle_deg = max_steer;
    if (steer_angle_deg < -max_steer) steer_angle_deg = -max_steer;

    s_current_steer_deg = steer_angle_deg;

    // 2. Tính góc servo tương ứng:
    // Bánh thẳng = 90 độ
    // Rẽ trái/phải = 90 độ +/- góc bẻ lái + góc bù lệch cơ khí
    float dir = s_invert_direction ? -1.0f : 1.0f;
    float target_servo_angle = SERVO_CENTER_DEG + s_trim_offset_deg + (dir * steer_angle_deg);

    return mg90s_set_servo_angle(target_servo_angle);
}

esp_err_t mg90s_set_angle_rad(float steer_angle_rad) {
    return mg90s_set_angle_deg(steer_angle_rad * RAD_TO_DEG);
}

float mg90s_get_servo_angle(void) {
    return s_current_servo_angle;
}

float mg90s_get_steering_angle_deg(void) {
    return s_current_steer_deg;
}

void mg90s_set_trim_offset(float offset_deg) {
    s_trim_offset_deg = offset_deg;
    mg90s_set_angle_deg(s_current_steer_deg);
}

float mg90s_get_trim_offset(void) {
    return s_trim_offset_deg;
}

void mg90s_set_invert(bool invert) {
    s_invert_direction = invert;
}

bool mg90s_get_invert(void) {
    return s_invert_direction;
}

void mg90s_stop(void) {
    ledc_stop(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, 0);
}
