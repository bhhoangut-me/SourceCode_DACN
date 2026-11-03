#ifndef VL53L0X_H
#define VL53L0X_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ROUTE_UNKNOWN = 0,
    ROUTE_UPPER,    // Case 1: Nhánh gần có vật cản -> Rẽ nhánh trên
    ROUTE_LOWER     // Case 2: Nhánh gần thông thoáng -> Rẽ nhánh dưới (vật cản ở nhánh xa)
} route_selection_t;

// Khởi tạo I2C và cảm biến VL53L0X
esp_err_t vl53l0x_init(void);

// Đọc khoảng cách thô và khoảng cách sau hiệu chuẩn bù sai số tĩnh
esp_err_t vl53l0x_read_distance_mm(uint16_t *raw_dist_mm, float *calib_dist_mm);

// Thuật toán ra quyết định nhị phân (Binary decision - Chương 5.4.1 Thuyết minh)
route_selection_t vl53l0x_decide_route(float *measured_distance);

#ifdef __cplusplus
}
#endif

#endif // VL53L0X_H
