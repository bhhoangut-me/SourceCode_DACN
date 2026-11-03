#include "jgb37_motor.h"
#include "config/pin_config.h"
#include "driver/ledc.h"
#include <math.h>

#define MOTOR_LEDC_TIMER            LEDC_TIMER_1
#define MOTOR_LEDC_MODE             LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_CH_IN1           LEDC_CHANNEL_1
#define MOTOR_LEDC_CH_IN2           LEDC_CHANNEL_2
#define MOTOR_LEDC_DUTY_RES         LEDC_TIMER_8_BIT   // Độ phân giải 8 bit (0 - 255)
#define MOTOR_LEDC_FREQ_HZ          20000              // 20 kHz (êm, không rít)

esp_err_t jgb37_motor_init(void) {
    ledc_timer_config_t timer_conf = {
        .speed_mode       = MOTOR_LEDC_MODE,
        .duty_resolution  = MOTOR_LEDC_DUTY_RES,
        .timer_num        = MOTOR_LEDC_TIMER,
        .freq_hz          = MOTOR_LEDC_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) return err;

    ledc_channel_config_t ch_in1 = {
        .gpio_num       = TB67_IN1_GPIO,
        .speed_mode     = MOTOR_LEDC_MODE,
        .channel        = MOTOR_LEDC_CH_IN1,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = MOTOR_LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    err = ledc_channel_config(&ch_in1);
    if (err != ESP_OK) return err;

    ledc_channel_config_t ch_in2 = {
        .gpio_num       = TB67_IN2_GPIO,
        .speed_mode     = MOTOR_LEDC_MODE,
        .channel        = MOTOR_LEDC_CH_IN2,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = MOTOR_LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    err = ledc_channel_config(&ch_in2);
    if (err != ESP_OK) return err;

    return jgb37_motor_stop();
}

esp_err_t jgb37_motor_set_pwm(float pwm) {
    if (pwm > 255.0f) pwm = 255.0f;
    if (pwm < -255.0f) pwm = -255.0f;

    uint32_t duty = (uint32_t)fabsf(pwm);

    if (pwm > 0.0f) {
        // Chạy tới: IN1 phát PWM, IN2 nối đất (0)
        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1, duty);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1);

        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2, 0);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2);
    } else if (pwm < 0.0f) {
        // Chạy lùi: IN1 = 0, IN2 phát PWM
        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1, 0);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1);

        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2, duty);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2);
    } else {
        // Dừng thả nổi
        return jgb37_motor_stop();
    }
    return ESP_OK;
}

esp_err_t jgb37_motor_brake(void) {
    // Kéo cả 2 chân IN1, IN2 lên HIGH (duty 255) để hãm điện động
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1, 255);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1);

    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2, 255);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2);
    return ESP_OK;
}

esp_err_t jgb37_motor_stop(void) {
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1, 0);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1);

    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2, 0);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2);
    return ESP_OK;
}

