#include "mg90s.h"
#include "config/pin_config.h"
#include "config/robot_config.h"
#include "driver/ledc.h"
#include <math.h>

#define SERVO_LEDC_TIMER            LEDC_TIMER_0
#define SERVO_LEDC_MODE             LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL          LEDC_CHANNEL_0
#define SERVO_LEDC_DUTY_RES         LEDC_TIMER_14_BIT // Độ phân giải 14 bit (0 - 16383)
#define SERVO_LEDC_FREQ_HZ          50                 // Tần số RC servo 50 Hz (chu kỳ 20 ms)

#define SERVO_MIN_PULSE_US          500   // -90 độ
#define SERVO_MID_PULSE_US          1500  // 0 độ (trung tâm)
#define SERVO_MAX_PULSE_US          2500  // +90 độ
#define SERVO_PERIOD_US             20000 // 20 ms = 20,000 us

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
#endif

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
        .duty           = pulse_to_duty(SERVO_MID_PULSE_US),
        .hpoint         = 0
    };
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) return err;

    // Đặt ban đầu về vị trí thẳng (0 độ)
    return mg90s_set_angle_deg(0.0f);
}

esp_err_t mg90s_set_angle_deg(float angle_deg) {
    // Giới hạn an toàn góc bẻ lái cơ học của robot (~35 độ)
    float max_deg = ROBOT_MAX_STEER_ANGLE_RAD * RAD_TO_DEG;
    if (angle_deg > max_deg) angle_deg = max_deg;
    if (angle_deg < -max_deg) angle_deg = -max_deg;

    // Quy đổi góc [-90, +90] tương ứng pulse [500, 2500] us
    float pulse_us = SERVO_MID_PULSE_US + (angle_deg / 90.0f) * (SERVO_MAX_PULSE_US - SERVO_MID_PULSE_US);
    
    if (pulse_us < SERVO_MIN_PULSE_US) pulse_us = SERVO_MIN_PULSE_US;
    if (pulse_us > SERVO_MAX_PULSE_US) pulse_us = SERVO_MAX_PULSE_US;

    uint32_t duty = pulse_to_duty((uint32_t)pulse_us);
    esp_err_t err = ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    if (err == ESP_OK) {
        err = ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
    }
    return err;
}

esp_err_t mg90s_set_angle_rad(float angle_rad) {
    return mg90s_set_angle_deg(angle_rad * RAD_TO_DEG);
}

void mg90s_stop(void) {
    ledc_stop(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, 0);
}

