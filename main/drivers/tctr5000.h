#ifndef TCTR5000_H
#define TCTR5000_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "config/robot_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t raw_adc[LINE_SENSOR_COUNT];       // Giá trị ADC thô (12-bit: 0 - 4095)
    uint16_t calibrated[LINE_SENSOR_COUNT];    // Giá trị sau chuẩn hóa (0: trắng -> 1000: đen)
    uint16_t min_white[LINE_SENSOR_COUNT];     // Mức nền trắng
    uint16_t max_black[LINE_SENSOR_COUNT];     // Mức vạch đen
    float position_mm;                         // Độ lệch tâm e2 (mm) [-39.0 đến +39.0]
    float position_m;                          // Độ lệch tâm e2 (m)
    bool line_lost;                            // Cờ báo mất line
    bool intersection_detected;                // Cờ phát hiện vạch ngang / ngã rẽ nhánh
    uint8_t active_sensor_count;               // Số cảm biến đang nằm trên line đen
} line_sensor_data_t;

// Khởi tạo GPIO MUX và ADC1 đọc mảng TCRT5000
esp_err_t line_sensor_init(void);

// Đọc giá trị 7 cảm biến và tính toán độ lệch line e2
esp_err_t line_sensor_read(line_sensor_data_t *data);

// Thực hiện tự động hiệu chuẩn nền trắng / vạch đen
void line_sensor_set_calibration(const uint16_t white_vals[LINE_SENSOR_COUNT], const uint16_t black_vals[LINE_SENSOR_COUNT]);

#ifdef __cplusplus
}
#endif

#endif // TCTR5000_H

