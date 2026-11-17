#include "control_task.h"
#include "communication/web_server.h"
#include "config/robot_config.h"
#include "control/speed_control.h"
#include "drivers/jgb37_encoder.h"
#include "drivers/jgb37_motor.h"
#include "drivers/mg90s.h"
#include "drivers/tctr5000.h"
#include "esp_log.h"
#include "tasks/state_task.h"
#include <math.h>
#include <stdlib.h>

static const char *TAG = "CONTROL_TASK";

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
#endif

/* ==========================================================================
 * ADAPTIVE EMA (Exponential Moving Average) - Bộ lọc thích nghi
 * --------------------------------------------------------------------------
 * Thay thế bộ lọc IIR cố định (fixed-alpha) bằng bộ lọc EMA có alpha tự
 * điều chỉnh theo tốc độ biến thiên tín hiệu:
 *
 *   alpha = alpha_min + (alpha_max - alpha_min) * |delta| / (|delta| + S)
 *
 * Trong đó:
 *   delta = new_value - current_filtered_value  (biến thiên tức thời)
 *   S     = sensitivity  (hằng số điều chỉnh độ nhạy)
 *
 * - Đường thẳng:  |delta| << S  →  alpha ≈ alpha_min  →  lọc mạnh (mượt)
 * - Vào cua:      |delta| >> S  →  alpha ≈ alpha_max  →  phản ứng nhanh
 * ========================================================================== */

typedef struct {
  float alpha_min;   // Alpha tối thiểu (lọc mạnh, đường thẳng)
  float alpha_max;   // Alpha tối đa (phản ứng nhanh, vào cua)
  float sensitivity; // Hằng số nhạy S: |delta| = S → alpha ở giữa
  float value;       // Giá trị đã lọc hiện tại
} adaptive_ema_t;

/**
 * @brief Khởi tạo bộ lọc Adaptive EMA
 */
static inline void adaptive_ema_init(adaptive_ema_t *f, float alpha_min,
                                     float alpha_max, float sensitivity,
                                     float initial_value) {
  f->alpha_min = alpha_min;
  f->alpha_max = alpha_max;
  f->sensitivity = sensitivity;
  f->value = initial_value;
}

/**
 * @brief Cập nhật bộ lọc Adaptive EMA với mẫu mới
 * @return Giá trị đã lọc
 */
static inline float adaptive_ema_update(adaptive_ema_t *f, float new_value) {
  float delta = fabsf(new_value - f->value);
  // Logistic-like transition: delta = 0 → alpha_min, delta → ∞ → alpha_max
  float alpha = f->alpha_min + (f->alpha_max - f->alpha_min) * delta /
                                   (delta + f->sensitivity);
  f->value = (1.0f - alpha) * f->value + alpha * new_value;
  return f->value;
}

static speed_controller_t s_speed_ctrl;
static line_sensor_data_t s_line_data;

// Vận tốc dài danh định thiết kế (m/s)
static float s_target_linear_speed =
    0.4f; // 0.4 m/s (chạy mượt, có thể tăng lên đến 2.0 m/s)

// Chế độ PWM cố định (Manual/Fixed PWM): Mặc định bật để an toàn, chỉnh được
// qua code và GUI
static bool s_use_manual_pwm = true;
static float s_manual_pwm =
    45.0f; // Mức PWM mặc định (0 - 255), 45 ~ 18% công suất chạy rất êm

static float s_current_steer_rad = 0.0f;
static float s_prev_e2_m = 0.0f;
static uint32_t s_lost_line_counter = 0;
static uint32_t s_intersection_counter = 0;
static uint32_t s_single_line_confirm_cnt = 0;
static int64_t s_intersection_start_pulses = 0;
static bool s_has_passed_intersection = false;

void control_system_init(void) {
  // 1. Khởi tạo cảm biến dò line TCRT5000 + MUX HEF4051B
  line_sensor_init();

  // 2. Khởi tạo Servo bẻ lái MG90S
  mg90s_init();

  // 3. Khởi tạo driver động cơ kéo DRV8871
  jgb37_motor_init();

  // 4. Khởi tạo đọc xung Encoder
  jgb37_encoder_init();

  // 5. Khởi tạo bộ điều khiển PI vận tốc
  speed_control_init(&s_speed_ctrl);

  ESP_LOGI(TAG, "Khoi tao he thong dieu khien thanh cong!");
}

void control_set_target_speed(float speed_mps) {
  if (speed_mps < ROBOT_SPEED_MIN_MPS)
    speed_mps = ROBOT_SPEED_MIN_MPS;
  if (speed_mps > ROBOT_SPEED_MAX_MPS)
    speed_mps = ROBOT_SPEED_MAX_MPS;
  s_target_linear_speed = speed_mps;
}

float control_get_current_speed(void) {
  return s_speed_ctrl.actual_velocity_mps;
}

float control_get_lateral_error_mm(void) { return s_line_data.position_mm; }

float control_get_steer_angle_deg(void) {
  return s_current_steer_rad * RAD_TO_DEG;
}

float control_get_servo_angle_deg(void) { return mg90s_get_servo_angle(); }

float control_get_current_pwm(void) { return s_speed_ctrl.output_pwm; }

float control_get_current_rpm(void) { return s_speed_ctrl.actual_rpm; }

void control_set_manual_pwm(float pwm) {
  if (pwm < 0.0f)
    pwm = 0.0f;
  if (pwm > 255.0f)
    pwm = 255.0f;
  s_manual_pwm = pwm;
  s_use_manual_pwm = true;
  ESP_LOGI(TAG, "Da dat PWM thu cong: %.0f/255 (Che do Manual PWM)",
           s_manual_pwm);
}

void control_set_mode(bool manual_mode) {
  s_use_manual_pwm = manual_mode;
  ESP_LOGI(TAG, "Chuyen che do toc do: %s",
           manual_mode ? "MANUAL PWM" : "AUTO PI");
}

bool control_is_manual_mode(void) { return s_use_manual_pwm; }

float control_get_manual_pwm(void) { return s_manual_pwm; }

void Control_Task(void *pvParameters) {
  ESP_LOGI(TAG, "Control_Task khoi dong tren Core %d voi chu ky %d ms (100Hz)",
           xPortGetCoreID(), MOTOR_CONTROL_PERIOD_MS);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS);
  const float dt = (float)MOTOR_CONTROL_PERIOD_MS / 1000.0f; // 0.01s

  /* Khởi tạo 3 bộ lọc Adaptive EMA ------------------------------------ */

  // Bộ lọc 1: e2 (sai số lệch ngang, mảng 8 cảm biến bước 8mm)
  //   - Đường thẳng: alpha_min=0.25 → lọc nhiễu ADC nhưng giảm trễ pha
  //   - Vào cua: alpha_max=0.85 → bám sát tín hiệu thật tức thời
  //   - sensitivity=5.0 mm: ngưỡng chuyển tiếp mượt giữa thẳng và cua
  adaptive_ema_t ema_e2;
  adaptive_ema_init(&ema_e2, 0.25f, 0.85f, 5.0f, 0.0f);

  // Bộ lọc 2: de2/dt (đạo hàm sai số lệch ngang, đơn vị m/s)
  //   - Đường thẳng: alpha_min=0.35 → khử trễ pha 90ms để K3 trả lái sớm nhịp
  //   - Vào cua: alpha_max=0.75 → phản ánh nhanh tốc độ lệch ngang
  //   - sensitivity=0.15 m/s: tương đương e2 thay đổi ~1.5mm/chu kỳ
  adaptive_ema_t ema_de2;
  adaptive_ema_init(&ema_de2, 0.35f, 0.75f, 0.15f, 0.0f);

  // Bộ lọc 3: gamma (góc lái servo, đơn vị rad)
  //   - Đường thẳng: alpha_min=0.55 → không làm trễ lệnh giảm chấn trả lái
  //   - Vào cua: alpha_max=0.90 → bẻ lái gần như tức thời
  //   - sensitivity=0.10 rad (~5.7°): biến thiên 5.7° → alpha ở giữa
  adaptive_ema_t ema_gamma;
  adaptive_ema_init(&ema_gamma, 0.55f, 0.90f, 0.10f, 0.0f);

  while (1) {
    // Duy trì chu kỳ lấy mẫu chính xác 10 ms
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    robot_state_t state = state_get_current();

    // 1. Luôn đọc cảm biến TCRT5000 và phản hồi Encoder ở mọi trạng thái
    line_sensor_read(&s_line_data);
    float current_motor_rpm = jgb37_encoder_update_rpm(dt);

    // In log định kỳ 200 ms (20 chu kỳ x 10 ms) để kiểm tra, debug và hiệu
    // chuẩn (calibrate)
    static int s_log_counter = 0;
    if (++s_log_counter >= 20) {
      s_log_counter = 0;
      ESP_LOGI(
          TAG,
          "ADC: [S1:%4d S2:%4d S3:%4d S4:%4d S5:%4d S6:%4d S7:%4d S8:%4d] | "
          "e2:%5.1f "
          "mm | RPM:%5.0f | PWM:%3.0f | St:%d%s",
          s_line_data.raw_adc[0], s_line_data.raw_adc[1],
          s_line_data.raw_adc[2], s_line_data.raw_adc[3],
          s_line_data.raw_adc[4], s_line_data.raw_adc[5],
          s_line_data.raw_adc[6], s_line_data.raw_adc[7],
          s_line_data.position_mm, current_motor_rpm, s_speed_ctrl.output_pwm,
          (int)state,
          s_line_data.line_lost
              ? " (LOST)"
              : (s_line_data.intersection_detected ? " (INTERSECTION)" : ""));
    }

    // Gửi Telemetry qua WebSocket lên Web Dashboard mỗi 50 ms (20 Hz)
    static int s_telemetry_counter = 0;
    if (++s_telemetry_counter >= 5) {
      s_telemetry_counter = 0;
      web_server_send_telemetry(&s_line_data);
    }

    // 2. Nếu chưa chạy hoặc đang dừng -> ngắt động cơ và trả thẳng lái
    if (state == ROBOT_STATE_INIT || state == ROBOT_STATE_WAIT_START ||
        state == ROBOT_STATE_DETECT_OBSTACLE) {
      jgb37_motor_stop();
      mg90s_set_angle_deg(0.0f);
      speed_control_reset(&s_speed_ctrl);
      ema_e2.value = s_line_data.position_mm;
      ema_de2.value = 0.0f;
      ema_gamma.value = 0.0f;
      s_prev_e2_m = s_line_data.position_m;
      s_lost_line_counter = 0;
      s_intersection_counter = 0;
      s_single_line_confirm_cnt = 0;
      s_intersection_start_pulses = jgb37_encoder_get_count();
      s_has_passed_intersection = false;
      continue;
    }

    if (state == ROBOT_STATE_FINISH || state == ROBOT_STATE_STOPPED) {
      jgb37_motor_brake();
      mg90s_set_angle_deg(0.0f);
      speed_control_reset(&s_speed_ctrl);
      ema_e2.value = s_line_data.position_mm;
      ema_de2.value = 0.0f;
      ema_gamma.value = 0.0f;
      s_prev_e2_m = s_line_data.position_m;
      continue;
    }

    // 3. Xử lý logic tại ngã rẽ và phát hiện điểm kết thúc
    if (state == ROBOT_STATE_LINE_TRACKING) {
      // Kiểm tra ngã rẽ: Bắt ngay lập tức (1 chu kỳ = 10ms) khi phát hiện vạch tách nhánh (span >= 5 hoặc 2 cụm)
      // để xe không bị giằng co giữa 2 nhánh gây khựng xe
      if (s_line_data.intersection_detected && !s_has_passed_intersection) {
        route_selection_t r = state_get_selected_route();
        ESP_LOGI(
            TAG,
            "Phat hien nga 3 (clusters=%d, active=%d)! Bam nhanh VONG KIN: %s",
            s_line_data.cluster_count, s_line_data.active_sensor_count,
            (r == ROUTE_UPPER) ? "NHANH TREN (TRAI)" : "NHANH DUOI (PHAI)");
        s_intersection_start_pulses = jgb37_encoder_get_count();
        s_intersection_counter = 0;
        s_single_line_confirm_cnt = 0;

        // Khởi tạo mượt bộ lọc tại thời điểm chuyển sang bám nhánh để KHÔNG gây vọt đạo hàm de2/dt
        // (tránh làm Servo quật ngang đột ngột gây phanh cơ học làm khựng xe)
        float init_branch_e2 = (r == ROUTE_UPPER)
                                   ? s_line_data.left_branch_pos_mm
                                   : s_line_data.right_branch_pos_mm;
        ema_e2.value = init_branch_e2;
        ema_de2.value = 0.0f;
        s_prev_e2_m = init_branch_e2 / 1000.0f;

        state_set(ROBOT_STATE_INTERSECTION);
        state = ROBOT_STATE_INTERSECTION; // Áp dụng ngay trong chu kỳ hiện tại
      }

      // Kiểm tra mất line (khi đã đi qua ngã rẽ và tới vạch đích)
      if (s_line_data.line_lost) {
        s_lost_line_counter++;
        // Nếu mất line liên tục quá 35 chu kỳ (350ms) sau khi đã qua ngã rẽ ->
        // Đã về đích!
        if (s_has_passed_intersection && s_lost_line_counter >= 35) {
          ESP_LOGI(TAG,
                   "Da ve dich! Tien hanh dung xe (Thoa sai so dung <= 5mm)");
          state_set(ROBOT_STATE_FINISH);
          jgb37_motor_brake();
          continue;
        }
      } else {
        s_lost_line_counter = 0;
      }
    }

    // 5. Tính toán điều khiển bám quỹ đạo vòng kín Lyapunov (Chương 5 Thuyết minh)
    //    Kết hợp kỹ thuật Sensor Masking khi đi qua ngã 3 (ROBOT_STATE_INTERSECTION)
    float v_cmd = s_target_linear_speed;
    float omega_cmd = 0.0f;
    float gamma = 0.0f;

    if (state == ROBOT_STATE_INTERSECTION || state == ROBOT_STATE_LINE_TRACKING) {
      float e2_mm = s_line_data.position_mm;

      if (state == ROBOT_STATE_INTERSECTION) {
        // Đang đi trong vùng ngã 3:
        // Sử dụng Sensor Masking để chỉ lấy vị trí e2 của cụm mắt thuộc nhánh cần đi
        route_selection_t route = state_get_selected_route();
        if (route == ROUTE_UPPER) {
          // Nhánh trên (bên Trái - tọa độ âm [-28..-4 mm]):
          e2_mm = s_line_data.left_branch_pos_mm;
          // Trong giai đoạn đầu mới chớm vào ngã 3 (khi 2 nhánh chưa tách xa),
          // đặt ngưỡng dẫn hướng tối thiểu -14mm để xe nghiêng mượt vào nhánh trái
          if (s_intersection_counter < 15 && e2_mm > -14.0f) {
            e2_mm = -14.0f;
          }
        } else {
          // Nhánh dưới (bên Phải - tọa độ dương [+4..+28 mm]):
          e2_mm = s_line_data.right_branch_pos_mm;
          // Đặt ngưỡng dẫn hướng tối thiểu +14mm giai đoạn đầu để nghiêng mượt vào nhánh phải
          if (s_intersection_counter < 15 && e2_mm < 14.0f) {
            e2_mm = 14.0f;
          }
        }
      }

      // Lọc Adaptive EMA tín hiệu e2
      float filtered_e2_mm = adaptive_ema_update(&ema_e2, e2_mm);
      float filtered_e2_m = filtered_e2_mm / 1000.0f;

      // e3: Sai số góc hướng ước lượng từ đạo hàm sai số lệch ngang
      float de2_dt = (filtered_e2_m - s_prev_e2_m) / dt;
      s_prev_e2_m = filtered_e2_m;

      float filtered_de2 = adaptive_ema_update(&ema_de2, de2_dt);

      float sin_e3 = 0.0f;
      if (v_cmd > 0.01f) {
        sin_e3 = filtered_de2 / v_cmd;
        if (sin_e3 > 1.0f)
          sin_e3 = 1.0f;
        if (sin_e3 < -1.0f)
          sin_e3 = -1.0f;
      }
      // Trong 100ms đầu khi vừa tách nhánh ở ngã 3, giới hạn thành phần đạo hàm e3
      // để tránh đạo hàm bước nhảy làm Servo bẻ gắt gây khựng xe
      if (state == ROBOT_STATE_INTERSECTION && s_intersection_counter < 10) {
        sin_e3 *= 0.3f;
      }
      float e3 = asinf(sin_e3);
      float e1 = 0.0f;

      // Tính vận tốc góc omega theo công thức Lyapunov (5.21):
      float v_d_mms = s_target_linear_speed * 1000.0f;
      omega_cmd =
          (LYAPUNOV_K2 * v_d_mms * filtered_e2_mm) + (LYAPUNOV_K3 * sin_e3);

      // Vận tốc dài điều khiển (giữ nguyên 100% lực kéo qua ngã 3 để không bị khựng):
      v_cmd = s_target_linear_speed * cosf(e3) + (LYAPUNOV_K1 * e1);
      if (v_cmd < ROBOT_SPEED_MIN_MPS)
        v_cmd = ROBOT_SPEED_MIN_MPS;

      // Quy đổi sang góc lái Servo theo mô hình động học Bicycle Model:
      const float ROBOT_L_STEER_EFF_M = 0.12f;
      gamma = atan2f(ROBOT_L_STEER_EFF_M * omega_cmd, v_cmd);

      // Giới hạn góc lái tối đa tại ngã 3 ở mức vừa phải (~32 độ = 0.56 rad)
      // để bánh trước không bị bẻ ngang tạo lực cản cơ học làm khựng xe
      if (state == ROBOT_STATE_INTERSECTION) {
        const float MAX_INTERSECTION_STEER_RAD = 0.56f; // ~32 độ
        if (gamma > MAX_INTERSECTION_STEER_RAD)
          gamma = MAX_INTERSECTION_STEER_RAD;
        if (gamma < -MAX_INTERSECTION_STEER_RAD)
          gamma = -MAX_INTERSECTION_STEER_RAD;
      }

      // Chống văng khỏi khúc cua khi mất line tạm thời:
      if (s_line_data.line_lost) {
        if (filtered_e2_mm > 0.0f) {
          gamma = ROBOT_MAX_STEER_ANGLE_RAD;
        } else {
          gamma = -ROBOT_MAX_STEER_ANGLE_RAD;
        }
      }

      // Kiểm tra điều kiện hoàn tất vượt ngã 3 dựa trên Quãng đường Encoder + Cụm cảm biến
      if (state == ROBOT_STATE_INTERSECTION) {
        s_intersection_counter++;

        // Tính quãng đường thực tế xe đã lăn bánh từ lúc vào ngã 3 (mm)
        // 1 vòng bánh xe = pi * D = 251.33 mm ứng với ENCODER_PPR * DIFF_GEAR_RATIO = 1000 xung
        int64_t delta_pulses =
            llabs(jgb37_encoder_get_count() - s_intersection_start_pulses);
        float wheel_circumference_mm = (float)M_PI * WHEEL_DIAMETER_M * 1000.0f;
        float pulses_per_wheel_rev = ENCODER_PPR * DIFF_GEAR_RATIO;
        float dist_encoder_mm =
            ((float)delta_pulses / pulses_per_wheel_rev) * wheel_circumference_mm;

        // Quãng đường ước lượng theo thời gian (dự phòng nếu chạy Manual PWM không gắn Encoder)
        float dist_time_est_mm =
            (float)s_intersection_counter * dt * (s_target_linear_speed * 800.0f);
        float dist_traveled_mm =
            (dist_encoder_mm > dist_time_est_mm) ? dist_encoder_mm : dist_time_est_mm;

        // Kiểm tra mảng cảm biến đã thu gọn về đúng 1 vạch line đơn chưa
        if (!s_line_data.intersection_detected &&
            s_line_data.cluster_count == 1 &&
            s_line_data.active_sensor_count >= 1 &&
            s_line_data.active_sensor_count <= 4) {
          s_single_line_confirm_cnt++;
        } else {
          s_single_line_confirm_cnt = 0;
        }

        // Thoát ngã 3 khi:
        // 1. Đã đi qua vùng tách nhánh tối thiểu 120 mm VÀ đã khóa gọn vào 1 line đơn trong 30 ms; HOẶC
        // 2. Đã đi hết quãng đường tối đa của ngã rẽ (350 mm)
        if ((dist_traveled_mm >= 120.0f && s_single_line_confirm_cnt >= 3) ||
            dist_traveled_mm >= 350.0f) {
          ESP_LOGI(TAG,
                   "Da qua nga 3 thanh cong (Quang duong: %.0f mm, ticks: %lu), tiep tuc bam line don",
                   dist_traveled_mm, (unsigned long)s_intersection_counter);
          s_has_passed_intersection = true;
          s_intersection_counter = 0;
          s_single_line_confirm_cnt = 0;
          state_set(ROBOT_STATE_LINE_TRACKING);
        }
      }
    }

    // 6. Lọc Adaptive EMA góc lái servo: đường thẳng → servo chuyển mượt,
    //    vào cua → bẻ lái gần tức thời (anti-jitter thích nghi)
    float filtered_gamma = adaptive_ema_update(&ema_gamma, gamma);

    s_current_steer_rad = filtered_gamma;
    mg90s_set_angle_rad(filtered_gamma);

    // Tính hệ số tự động giảm tốc khi vào cua gắt để chống văng xe (Cornering
    // Speed Reduction). Riêng tại ngã 3 (INTERSECTION), giữ nguyên 100% lực kéo
    // để xe lướt qua nhánh rẽ dứt khoát, không bị khựng.
    float abs_gamma_deg = fabsf(gamma * RAD_TO_DEG);
    float cornering_factor = 1.0f;
    if (state != ROBOT_STATE_INTERSECTION && abs_gamma_deg > 10.0f) {
      // Góc lái từ 10 độ đến 55 độ: giảm ga từ 100% xuống 65% để ôm trọn cua
      // R500
      cornering_factor = 1.0f - 0.35f * ((abs_gamma_deg - 10.0f) / 45.0f);
      if (cornering_factor < 0.65f)
        cornering_factor = 0.65f;
    }

    // 7. Vòng điều khiển vận tốc (Hỗ trợ chế độ Manual PWM hoặc Auto PI)
    float pwm_output = 0.0f;
    if (s_use_manual_pwm) {
      pwm_output = s_manual_pwm * cornering_factor;
      // Giữ ngưỡng PWM tối thiểu (>= 38) khi bẻ lái để thắng lực cản ma sát bánh trước, chống khựng xe
      if (s_manual_pwm >= 38.0f && pwm_output < 38.0f) {
        pwm_output = 38.0f;
      }
      s_speed_ctrl.output_pwm = pwm_output;
      s_speed_ctrl.actual_rpm = current_motor_rpm;
      s_speed_ctrl.actual_velocity_mps =
          speed_control_rpm_to_linear(current_motor_rpm);
    } else {
      pwm_output = speed_control_update(&s_speed_ctrl, v_cmd * cornering_factor,
                                        current_motor_rpm);
    }

    // 8. Xuất xung PWM điều khiển động cơ JGB37
    jgb37_motor_set_pwm(pwm_output);
  }
}
