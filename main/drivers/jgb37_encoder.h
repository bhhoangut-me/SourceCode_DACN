#ifndef JGB37_ENCODER_H
#define JGB37_ENCODER_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Số xung trên 1 vòng trục động cơ (xung/vòng)
// Thường là 11 xung/pha * số cạnh hoặc 330 xung sau hộp số
#ifndef ENCODER_PPR
#define ENCODER_PPR                 330.0f
#endif

// Khởi tạo đọc xung Encoder JGB37 qua GPIO ngắt
esp_err_t jgb37_encoder_init(void);

// Lấy tổng số xung đếm được (tính cả chiều)
int64_t jgb37_encoder_get_count(void);

// Reset số xung đếm về 0
void jgb37_encoder_reset_count(void);

// Tính toán tốc độ vòng/phút (RPM) trong chu kỳ delta_time_s
// Hàm này được gọi định kỳ trong task điều khiển (mỗi 10ms)
float jgb37_encoder_update_rpm(float delta_time_s);

// Lấy giá trị RPM được cập nhật gần nhất
float jgb37_encoder_get_rpm(void);

#ifdef __cplusplus
}
#endif

#endif // JGB37_ENCODER_H

