#include "tctr5000.h"
#include "config/pin_config.h"
#include "config/robot_config.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"
#include <string.h>

#define TCRT_ADC_CHANNEL            ADC_CHANNEL_4 // GPIO 32 là ADC1_CHANNEL_4
#define SETTLE_DELAY_US             60             // Thời gian trễ ổn định sau khi chuyển kênh MUX

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;

// Giá trị mặc định đo đạc thực nghiệm từ Bảng 4.2 Thuyết minh:
// White ~ 420, Black ~ 2859
static uint16_t s_default_white[LINE_SENSOR_COUNT] = {420, 420, 420, 420, 420, 420, 420};
static uint16_t s_default_black[LINE_SENSOR_COUNT] = {2859, 2859, 2859, 2859, 2859, 2859, 2859};

esp_err_t line_sensor_init(void) {
    // 1. Cấu hình các chân điều khiển MUX HEF4051B (ss1, ss2, ss3) làm ngõ ra
    gpio_config_t mux_gpio_conf = {
        .pin_bit_mask = (1ULL << TCRT_SEL1_GPIO) | 
                        (1ULL << TCRT_SEL2_GPIO) | 
                        (1ULL << TCRT_SEL3_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t err = gpio_config(&mux_gpio_conf);
    if (err != ESP_OK) return err;

    gpio_set_level(TCRT_SEL1_GPIO, 0);
    gpio_set_level(TCRT_SEL2_GPIO, 0);
    gpio_set_level(TCRT_SEL3_GPIO, 0);

    // 2. Cấu hình ADC Oneshot đọc chân GPIO 32 (ADC1 Channel 4)
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    err = adc_oneshot_new_unit(&init_config1, &s_adc1_handle);
    if (err != ESP_OK) return err;

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12, // Dải đo lên đến ~3.3V
    };
    return adc_oneshot_config_channel(s_adc1_handle, TCRT_ADC_CHANNEL, &chan_config);
}

void line_sensor_set_calibration(const uint16_t white_vals[LINE_SENSOR_COUNT], const uint16_t black_vals[LINE_SENSOR_COUNT]) {
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        if (white_vals) s_default_white[i] = white_vals[i];
        if (black_vals) s_default_black[i] = black_vals[i];
    }
}

static void mux_select_channel(uint8_t ch) {
    gpio_set_level(TCRT_SEL1_GPIO, (ch & 0x01) ? 1 : 0);
    gpio_set_level(TCRT_SEL2_GPIO, (ch & 0x02) ? 1 : 0);
    gpio_set_level(TCRT_SEL3_GPIO, (ch & 0x04) ? 1 : 0);
    esp_rom_delay_us(SETTLE_DELAY_US);
}

esp_err_t line_sensor_read(line_sensor_data_t *data) {
    if (!data || !s_adc1_handle) return ESP_ERR_INVALID_STATE;

    float weighted_sum = 0.0f;
    float sum_weights = 0.0f;
    uint8_t active_count = 0;

    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        // Chuyển kênh MUX HEF4051B từ Y0 đến Y6
        mux_select_channel(i);

        int raw_val = 0;
        esp_err_t err = adc_oneshot_read(s_adc1_handle, TCRT_ADC_CHANNEL, &raw_val);
        if (err != ESP_OK) {
            raw_val = 0;
        }
        data->raw_adc[i] = (uint16_t)raw_val;

        // Chuẩn hóa tín hiệu về thang đo [0, 1000]
        // 0: Hoàn toàn là nền trắng; 1000: Hoàn toàn là vạch đen
        uint16_t w_min = s_default_white[i];
        uint16_t b_max = s_default_black[i];
        int32_t calib = 0;

        if (b_max > w_min) {
            calib = (int32_t)(raw_val - w_min) * 1000 / (int32_t)(b_max - w_min);
        }
        if (calib < 0) calib = 0;
        if (calib > 1000) calib = 1000;

        data->calibrated[i] = (uint16_t)calib;

        // Nếu giá trị > 400 coi như cảm biến này đang thấy line
        if (calib > 400) {
            active_count++;
        }

        // Tích lũy trọng số theo vị trí tọa độ của mắt cảm biến LINE_SENSOR_POS_MM[i]
        weighted_sum += (float)calib * LINE_SENSOR_POS_MM[i];
        sum_weights += (float)calib;
    }

    data->active_sensor_count = active_count;

    // Kiểm tra mất line: không cảm biến nào thấy line đen đủ lớn
    if (sum_weights < 300.0f) {
        data->line_lost = true;
        data->intersection_detected = false;
        // Giữ nguyên vị trí cũ hoặc không thay đổi
    } else {
        data->line_lost = false;
        // Công thức trung bình trọng số (Chương 4.2.8):
        // e2 = sum(x_i * w_i) / sum(w_i)
        data->position_mm = weighted_sum / sum_weights;
        data->position_m = data->position_mm / 1000.0f;

        // Phát hiện ngã rẽ nhánh / vạch giao cắt:
        // Khi gặp giao điểm hoặc vạch ngang, phần lớn mảng cảm biến (>= 4 cảm biến) đều nhìn thấy đen
        if (active_count >= 4) {
            data->intersection_detected = true;
        } else {
            data->intersection_detected = false;
        }
    }

    return ESP_OK;
}

