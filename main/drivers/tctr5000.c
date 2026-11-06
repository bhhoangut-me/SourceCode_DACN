#include "tctr5000.h"
#include "config/pin_config.h"
#include "config/robot_config.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"
#include <math.h>
#include <string.h>

// Tự động map GPIO sang kênh ADC1 tương ứng
#if (TCRT_ADC_GPIO == 36)
#define TCRT_ADC_CHANNEL ADC_CHANNEL_0 // GPIO 36 (SENSOR_VP) là ADC1_CHANNEL_0
#elif (TCRT_ADC_GPIO == 37)
#define TCRT_ADC_CHANNEL ADC_CHANNEL_1
#elif (TCRT_ADC_GPIO == 38)
#define TCRT_ADC_CHANNEL ADC_CHANNEL_2
#elif (TCRT_ADC_GPIO == 39)
#define TCRT_ADC_CHANNEL ADC_CHANNEL_3 // GPIO 39 (SENSOR_VN)
#elif (TCRT_ADC_GPIO == 32)
#define TCRT_ADC_CHANNEL ADC_CHANNEL_4 // GPIO 32 là ADC1_CHANNEL_4
#elif (TCRT_ADC_GPIO == 33)
#define TCRT_ADC_CHANNEL ADC_CHANNEL_5 // GPIO 33 là ADC1_CHANNEL_5
#elif (TCRT_ADC_GPIO == 34)
#define TCRT_ADC_CHANNEL ADC_CHANNEL_6 // GPIO 34
#elif (TCRT_ADC_GPIO == 35)
#define TCRT_ADC_CHANNEL ADC_CHANNEL_7 // GPIO 35
#else
#error                                                                         \
    "TCRT_ADC_GPIO phai thuoc ADC1 (GPIO 32 - 39) de doc duoc voi Wi-Fi/FreeRTOS"
#endif

#define SETTLE_DELAY_US 60 // Thời gian trễ ổn định sau khi chuyển kênh MUX

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;

// Hiệu chuẩn thực nghiệm chuẩn xác từ ảnh thực tế trên sa bàn của bạn:
// - Nền trắng (White floor): trung bình ~45 - 75 (Ảnh 1 & 2: các mắt trắng từ
// 35 đến 83)
// - Vạch đen (Black line): trung bình ~2800 - 2850 (Ảnh 1 Mắt 4 = 2802, Ảnh 2
// Mắt 4 = 2311, Mắt 5 = 1856)
static uint16_t s_default_white[LINE_SENSOR_COUNT] = {60, 45, 60, 60,
                                                      60, 75, 75};
static uint16_t s_default_black[LINE_SENSOR_COUNT] = {2800, 2800, 2800, 2850,
                                                      2850, 2800, 2800};

esp_err_t line_sensor_init(void) {
  // 1. Cấu hình các chân điều khiển MUX HEF4051B (ss1, ss2, ss3) làm ngõ ra
  gpio_config_t mux_gpio_conf = {.pin_bit_mask = (1ULL << TCRT_SEL1_GPIO) |
                                                 (1ULL << TCRT_SEL2_GPIO) |
                                                 (1ULL << TCRT_SEL3_GPIO),
                                 .mode = GPIO_MODE_OUTPUT,
                                 .pull_up_en = GPIO_PULLUP_DISABLE,
                                 .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                 .intr_type = GPIO_INTR_DISABLE};
  esp_err_t err = gpio_config(&mux_gpio_conf);
  if (err != ESP_OK)
    return err;

  gpio_set_level(TCRT_SEL1_GPIO, 0);
  gpio_set_level(TCRT_SEL2_GPIO, 0);
  gpio_set_level(TCRT_SEL3_GPIO, 0);

  // 2. Cấu hình ADC Oneshot đọc chân GPIO 32 (ADC1 Channel 4)
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  err = adc_oneshot_new_unit(&init_config1, &s_adc1_handle);
  if (err != ESP_OK)
    return err;

  adc_oneshot_chan_cfg_t chan_config = {
      .bitwidth = ADC_BITWIDTH_12,
      .atten = ADC_ATTEN_DB_12, // Dải đo lên đến ~3.3V
  };
  return adc_oneshot_config_channel(s_adc1_handle, TCRT_ADC_CHANNEL,
                                    &chan_config);
}

void line_sensor_set_calibration(const uint16_t white_vals[LINE_SENSOR_COUNT],
                                 const uint16_t black_vals[LINE_SENSOR_COUNT]) {
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    if (white_vals)
      s_default_white[i] = white_vals[i];
    if (black_vals)
      s_default_black[i] = black_vals[i];
  }
}

static void mux_select_channel(uint8_t ch) {
  gpio_set_level(TCRT_SEL1_GPIO, (ch & 0x01) ? 1 : 0);
  gpio_set_level(TCRT_SEL2_GPIO, (ch & 0x02) ? 1 : 0);
  gpio_set_level(TCRT_SEL3_GPIO, (ch & 0x04) ? 1 : 0);
  esp_rom_delay_us(SETTLE_DELAY_US);
}

// Hàm hiệu chuẩn phi tuyến từng đoạn (Piecewise Linear Calibration) từ dữ liệu
// thực tế sa bàn:
// - Lệch 0 cm thật  -> thô  0.0 mm -> ra chuẩn xác   0.0 mm
// - Lệch 1 cm thật  -> thô  6.3 mm -> ra chuẩn xác  10.0 mm
// - Lệch 2 cm thật  -> thô 19.5 mm -> ra chuẩn xác  20.0 mm
// - Lệch 3.9 cm thật -> thô 39.0 mm -> ra chuẩn xác  39.0 mm
static float calibrate_piecewise_position_mm(float raw_pos_mm) {
  float sign = (raw_pos_mm >= 0.0f) ? 1.0f : -1.0f;
  float abs_pos = fabsf(raw_pos_mm);
  float calibrated_pos = 0.0f;

  // Đoạn 1: Từ 0 đến 6.3 mm (ứng với lệch 0 đến 10 mm / 1 cm thực tế)
  // Độ dốc = 10.0 / 6.3 = 1.5873f
  if (abs_pos <= 6.3f) {
    calibrated_pos = abs_pos * 1.5873f;
  }
  // Đoạn 2: Từ 6.3 mm đến 19.5 mm (ứng với lệch 10 mm đến 20 mm / 2 cm thực tế)
  // Độ dốc = (20.0 - 10.0) / (19.5 - 6.3) = 10.0 / 13.2 = 0.7576f
  else if (abs_pos <= 19.5f) {
    calibrated_pos = 10.0f + (abs_pos - 6.3f) * 0.7576f;
  }
  // Đoạn 3: Ngoài 19.5 mm (ứng với lệch ngoài 20 mm đến mép cảm biến 39 mm)
  // Độ dốc = (39.0 - 20.0) / (39.0 - 19.5) = 19.0 / 19.5 = 0.9744f
  else {
    calibrated_pos = 20.0f + (abs_pos - 19.5f) * 0.9744f;
  }

  if (calibrated_pos > 39.0f)
    calibrated_pos = 39.0f;
  return sign * calibrated_pos;
}

esp_err_t line_sensor_read(line_sensor_data_t *data) {
  if (!data || !s_adc1_handle)
    return ESP_ERR_INVALID_STATE;

  float weighted_sum = 0.0f;
  float sum_weights = 0.0f;
  uint8_t active_count = 0;

  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    // Quét tuần tự kênh MUX từ Y0 đến Y6 (đã đảo chân chọn bit trong
    // pin_config.h)
    mux_select_channel(i);

    // Đọc bỏ lần đầu để mức điện áp sau khi chuyển kênh MUX được ổn định
    int dummy = 0;
    adc_oneshot_read(s_adc1_handle, TCRT_ADC_CHANNEL, &dummy);

    // Lấy trung bình 8 mẫu như test trên Arduino để triệt tiêu nhiễu
    int sum_adc = 0;
    const int samples = 8;
    for (int s = 0; s < samples; s++) {
      int val = 0;
      if (adc_oneshot_read(s_adc1_handle, TCRT_ADC_CHANNEL, &val) == ESP_OK) {
        sum_adc += val;
      }
    }
    int raw_val = sum_adc / samples;
    data->raw_adc[i] = (uint16_t)raw_val;

    // Chuẩn hóa tín hiệu về thang đo [0, 1000]
    // 0: Hoàn toàn là nền trắng; 1000: Hoàn toàn là vạch đen
    uint16_t w_min = s_default_white[i];
    uint16_t b_max = s_default_black[i];
    int32_t calib = 0;

    if (b_max > w_min) {
      calib = (int32_t)(raw_val - w_min) * 1000 / (int32_t)(b_max - w_min);
    }
    if (calib < 0)
      calib = 0;
    if (calib > 1000)
      calib = 1000;

    data->calibrated[i] = (uint16_t)calib;

    // Nếu giá trị > 300 (tương ứng raw > 900) coi như cảm biến này đang thấy
    // line
    if (calib > 300) {
      active_count++;
    }

    // Tích lũy trọng số theo vị trí tọa độ của mắt cảm biến
    // LINE_SENSOR_POS_MM[i]
    weighted_sum += (float)calib * LINE_SENSOR_POS_MM[i];
    sum_weights += (float)calib;
  }

  data->active_sensor_count = active_count;

  // Kiểm tra mất line: chỉ coi là mất line khi tổng trọng số quá nhỏ (< 150)
  if (sum_weights < 150.0f) {
    data->line_lost = true;
    data->intersection_detected = false;
    // Giữ vị trí mép ngoài cùng theo hướng lệch trước đó để xe tiếp tục ôm cua
    // tìm lại line
    if (data->position_mm > 10.0f) {
      data->position_mm = 39.0f;
      data->position_m = 0.039f;
    } else if (data->position_mm < -10.0f) {
      data->position_mm = -39.0f;
      data->position_m = -0.039f;
    }
  } else {
    data->line_lost = false;
    // 1. Tính sai số thô theo trung bình trọng số hình học
    float raw_pos_mm = weighted_sum / sum_weights;

    // 2. Hiệu chuẩn từng đoạn phi tuyến theo dữ liệu thực tế sa bàn:
    // Lệch 0cm -> 0.0 mm
    // Lệch 1cm -> 10.0 mm
    // Lệch 2cm -> 20.0 mm
    data->position_mm = calibrate_piecewise_position_mm(raw_pos_mm);
    data->position_m = data->position_mm / 1000.0f;

    // Phát hiện ngã rẽ nhánh / vạch giao cắt:
    // Khi gặp giao điểm hoặc vạch ngang, phần lớn mảng cảm biến (>= 4 cảm biến)
    // đều nhìn thấy đen
    if (active_count >= 4) {
      data->intersection_detected = true;
    } else {
      data->intersection_detected = false;
    }
  }

  return ESP_OK;
}
