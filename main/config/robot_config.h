#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <math.h>

/* ==========================================================================
 * THUYẾT MINH KỸ THUẬT: ĐỒ ÁN CHUYÊN NGÀNH CƠ ĐIỆN TỬ (ME4071) - HCMUT
 * Đề tài: Robot dò line phát hiện và tránh vật cản (Topic 3)
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * 1. THÔNG SỐ HÌNH HỌC VÀ CƠ KHÍ (CHAPTER 3 & CHAPTER 5)
 * -------------------------------------------------------------------------- */
#define ROBOT_MASS_KG 1.5f    // Khối lượng xe M (kg)
#define ROBOT_WIDTH_M 0.15f   // Vệt bánh sau w (m) = 150 mm
#define ROBOT_LENGTH_M 0.225f // Chiều dài cơ sở l (m) = 225 mm
#define ROBOT_L1_M                                                             \
  0.05f // Khoảng cách từ trục sau đến trục lái / CG l1 (m) = 50 mm
#define ROBOT_L2_M                                                             \
  0.175f // Khoảng cách từ trọng tâm G / trục sau đến cảm biến l2 (m)
#define ROBOT_CG_HEIGHT_M 0.05f // Chiều cao trọng tâm h (m) = 50 mm

#define WHEEL_DIAMETER_M 0.08f // Đường kính bánh dẫn động sau D (m) = 80 mm
#define WHEEL_RADIUS_M 0.04f // Bán kính bánh xe Rw (m) = 40 mm
#define DIFF_GEAR_RATIO 2.5f // Tỉ số truyền cụm vi sai u_dg = 2.5

/* --------------------------------------------------------------------------
 * 2. CẢM BIẾN DÒ LINE TCRT5000 (CHAPTER 4)
 * -------------------------------------------------------------------------- */
#define LINE_SENSOR_COUNT 7 // 7 cảm biến TCRT5000 qua HEF4051B
#define LINE_SENSOR_PITCH_MM                                                   \
  13.0f // Bước cảm biến p = 13 mm (w_line / 2 = 26 / 2)
#define LINE_WIDTH_MM 26.0f // Bề rộng vạch line đen = 26 mm
#define LINE_SENSOR_SPAN_MM                                                    \
  78.0f // Chiều dài phủ mảng cảm biến (7-1)*13 = 78 mm
#define LINE_SENSOR_HEIGHT_MM                                                  \
  7.0f // Chiều cao tối ưu h = 7 mm (chênh lệch ADC max ~ 2439)

// Tọa độ các mắt cảm biến so với tâm xe (mm): {-39, -26, -13, 0, 13, 26, 39}
static const float LINE_SENSOR_POS_MM[LINE_SENSOR_COUNT] = {
    -39.0f, -26.0f, -13.0f, 0.0f, 13.0f, 26.0f, 39.0f};

/* --------------------------------------------------------------------------
 * 3. CẢM BIẾN VẬT CẢN VL53L0X (CHAPTER 5.4 & CHAPTER 6)
 * -------------------------------------------------------------------------- */
#define OBSTACLE_NEAR_DIST_THEORY_MM                                           \
  475.0f // Khoảng cách lý thuyết phản xạ nhánh gần (500mm - R_obs 25mm)
#define OBSTACLE_NEAR_DETECT_MAX_MM                                            \
  700.0f // Ngưỡng phân biệt nhánh gần vs nhánh xa (dưới 700mm -> nhánh gần có
         // vật)
#define OBSTACLE_CALIB_OFFSET_MM                                               \
  (-41.4f) // Bù sai số tĩnh (đo thô trung bình 516.6 mm -> chuẩn 475.2 mm)

/* --------------------------------------------------------------------------
 * 4. BỘ ĐIỀU KHIỂN BÁM QUỸ ĐẠO LYAPUNOV - OUTER LOOP (CHAPTER 5.2)
 * -------------------------------------------------------------------------- */
#define LYAPUNOV_K1 1.5f     // Hệ số bù sai số dọc e1
#define LYAPUNOV_K2 0.00018f // Hệ số ổn định sai số lệch ngang e2
#define LYAPUNOV_K3                                                            \
  7.0f // Hệ số ổn định góc hướng e3 (giảm từ 10.0 để chống lắc servo)

/* --------------------------------------------------------------------------
 * 5. BỘ ĐIỀU KHIỂN TỐC ĐỘ ĐỘNG CƠ PI - INNER LOOP (CHAPTER 5.6)
 * -------------------------------------------------------------------------- */
#define MOTOR_CONTROL_PERIOD_MS                                                \
  10 // Chu kỳ lấy mẫu điều khiển Ts = 10 ms (100 Hz)
#define MOTOR_DEADBAND_PWM 15 // Vùng chết PWM (0 - 15)

// Hệ số bộ điều khiển PI sau khi tinh chỉnh MATLAB PID Tuner & thực nghiệm
#define MOTOR_PI_KP 0.3975f
#define MOTOR_PI_KI 2.3751f

// Hệ số giải tích thiết kế lý thuyết (tham khảo)
#define MOTOR_PI_KP_ANALYTICAL 0.62f
#define MOTOR_PI_KI_ANALYTICAL 5.37f

/* --------------------------------------------------------------------------
 * 6. GIỚI HẠN VẬN HÀNH & AN TOÀN
 * -------------------------------------------------------------------------- */
#define ROBOT_SPEED_MIN_MPS 0.1f // 0.1 m/s
#define ROBOT_SPEED_MAX_MPS 2.0f // 2.0 m/s
#define ROBOT_MAX_ACCEL_MPS2                                                   \
  2.5f // Gia tốc thiết kế an toàn 2.5 m/s^2 (tránh bốc đầu)
#define ROBOT_MAX_STEER_ANGLE_RAD 0.7854f // ~ 45 độ giới hạn servo (pi/4)

#endif // ROBOT_CONFIG_H
