#include "vl53l0x.h"
#include "config/pin_config.h"
#include "config/robot_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "VL53L0X";

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000     // 100 kHz I2C
#define I2C_MASTER_TIMEOUT_MS       100

#define VL53L0X_DEFAULT_ADDR        0x29

// Các thanh ghi chính của VL53L0X
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xC0
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS_GPIO    0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR          0x0B

static esp_err_t i2c_write_byte(uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, VL53L0X_DEFAULT_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

static esp_err_t i2c_read_byte(uint8_t reg, uint8_t *data) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, VL53L0X_DEFAULT_ADDR, &reg, 1, data, 1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

static esp_err_t i2c_read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, VL53L0X_DEFAULT_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

esp_err_t vl53l0x_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = VL53L0X_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = VL53L0X_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // Kiểm tra kết nối cảm biến qua Model ID
    uint8_t model_id = 0;
    err = i2c_read_byte(VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id);
    if (err == ESP_OK && model_id == 0xEE) {
        ESP_LOGI(TAG, "VL53L0X detected successfully (ID: 0x%02X)", model_id);
    } else {
        ESP_LOGW(TAG, "VL53L0X ID check returned: 0x%02X (err: %d)", model_id, err);
    }

    // Thiết lập cấu hình cơ bản cho single-shot ranging
    i2c_write_byte(0x88, 0x00);
    i2c_write_byte(0x80, 0x01);
    i2c_write_byte(0xFF, 0x01);
    i2c_write_byte(0x00, 0x00);
    i2c_write_byte(0x91, 0x3C);
    i2c_write_byte(0x00, 0x01);
    i2c_write_byte(0xFF, 0x00);
    i2c_write_byte(0x80, 0x00);

    return ESP_OK;
}

esp_err_t vl53l0x_read_distance_mm(uint16_t *raw_dist_mm, float *calib_dist_mm) {
    if (!raw_dist_mm) return ESP_ERR_INVALID_ARG;

    // 1. Kích hoạt phép đo single shot (ghi 0x01 vào thanh ghi 0x00)
    esp_err_t err = i2c_write_byte(VL53L0X_REG_SYSRANGE_START, 0x01);
    if (err != ESP_OK) return err;

    // 2. Chờ kết quả đo sẵn sàng (tối đa 100ms)
    uint8_t status = 0;
    int timeout = 50;
    while (timeout-- > 0) {
        err = i2c_read_byte(VL53L0X_REG_RESULT_RANGE_STATUS, &status);
        if (err == ESP_OK && (status & 0x01)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (timeout <= 0) {
        ESP_LOGW(TAG, "Measurement timeout");
        return ESP_ERR_TIMEOUT;
    }

    // 3. Đọc khoảng cách từ thanh ghi 0x1E (Range Millimeter High & Low)
    uint8_t dist_buf[2];
    err = i2c_read_bytes(0x1E, dist_buf, 2);
    if (err != ESP_OK) return err;

    // Xóa ngắt
    i2c_write_byte(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    uint16_t raw = ((uint16_t)dist_buf[0] << 8) | dist_buf[1];
    *raw_dist_mm = raw;

    // 4. Áp dụng bù sai số tĩnh thực nghiệm (Chương 5.4.3: 516.6 mm -> 475.2 mm, offset = -41.4 mm)
    if (calib_dist_mm) {
        float calib = (float)raw + OBSTACLE_CALIB_OFFSET_MM;
        if (calib < 0.0f) calib = 0.0f;
        *calib_dist_mm = calib;
    }

    return ESP_OK;
}

// Thuật toán phân nhánh theo Thuyết minh (Chương 5.4.1)
route_selection_t vl53l0x_decide_route(float *measured_distance) {
    const int sample_count = 20; // Lấy mẫu trung bình 20 lần để lọc nhiễu ngẫu nhiên
    float sum_dist = 0.0f;
    int valid_samples = 0;

    for (int i = 0; i < sample_count; i++) {
        uint16_t raw = 0;
        float calib = 0.0f;
        if (vl53l0x_read_distance_mm(&raw, &calib) == ESP_OK) {
            // Giá trị đo hợp lệ
            if (raw > 20 && raw < 2000) {
                sum_dist += calib;
                valid_samples++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    float avg_dist = 9999.0f;
    if (valid_samples > 0) {
        avg_dist = sum_dist / (float)valid_samples;
    }

    if (measured_distance) {
        *measured_distance = avg_dist;
    }

    // Ngưỡng phân biệt 700 mm:
    // - Nhánh gần lý thuyết ~ 475 mm
    // - Nếu đo được <= 700 mm: Có vật cản ở nhánh gần -> Chọn nhánh trên (ROUTE_UPPER)
    // - Nếu > 700 mm hoặc không thấy vật cản: Nhánh gần thông thoáng -> Chọn nhánh dưới (ROUTE_LOWER)
    if (avg_dist <= OBSTACLE_NEAR_DETECT_MAX_MM) {
        ESP_LOGI(TAG, "Vat can o nhanh gan (d = %.1f mm <= %.0f mm) -> Chon nhanh TREN", 
                 avg_dist, (float)OBSTACLE_NEAR_DETECT_MAX_MM);
        return ROUTE_UPPER;
    } else {
        ESP_LOGI(TAG, "Nhanh gan thong thoang (d = %.1f mm > %.0f mm) -> Chon nhanh DUOI", 
                 avg_dist, (float)OBSTACLE_NEAR_DETECT_MAX_MM);
        return ROUTE_LOWER;
    }
}
