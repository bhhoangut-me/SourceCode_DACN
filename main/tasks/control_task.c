#include "control_task.h"
#include "control/speed_control.h"
#include "drivers/mg90s.h"
#include "drivers/jgb37_motor.h"
#include "drivers/jgb37_encoder.h"
#include "drivers/tctr5000.h"
#include "tasks/state_task.h"
#include "config/robot_config.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "CONTROL_TASK";

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
#endif

static speed_controller_t s_speed_ctrl;
static line_sensor_data_t s_line_data;

// Vận tốc dài danh định thiết kế (m/s)
static float s_target_linear_speed = 0.4f; // 0.4 m/s (chạy mượt, có thể tăng lên đến 2.0 m/s)
static float s_current_steer_rad = 0.0f;
static float s_prev_e2_m = 0.0f;
static uint32_t s_lost_line_counter = 0;
static uint32_t s_intersection_counter = 0;
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
    if (speed_mps < ROBOT_SPEED_MIN_MPS) speed_mps = ROBOT_SPEED_MIN_MPS;
    if (speed_mps > ROBOT_SPEED_MAX_MPS) speed_mps = ROBOT_SPEED_MAX_MPS;
    s_target_linear_speed = speed_mps;
}

float control_get_current_speed(void) {
    return s_speed_ctrl.actual_velocity_mps;
}

float control_get_lateral_error_mm(void) {
    return s_line_data.position_mm;
}

float control_get_steer_angle_deg(void) {
    return s_current_steer_rad * RAD_TO_DEG;
}

void Control_Task(void *pvParameters) {
    ESP_LOGI(TAG, "Control_Task khoi dong tren Core %d voi chu ky %d ms (100Hz)", 
             xPortGetCoreID(), MOTOR_CONTROL_PERIOD_MS);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS);
    const float dt = (float)MOTOR_CONTROL_PERIOD_MS / 1000.0f; // 0.01s

    while (1) {
        // Duy trì chu kỳ lấy mẫu chính xác 10 ms
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        robot_state_t state = state_get_current();

        // 1. Nếu chưa chạy hoặc đang dừng -> ngắt động cơ và trả thẳng lái
        if (state == ROBOT_STATE_INIT || state == ROBOT_STATE_WAIT_START || state == ROBOT_STATE_DETECT_OBSTACLE) {
            jgb37_motor_stop();
            mg90s_set_angle_deg(0.0f);
            speed_control_reset(&s_speed_ctrl);
            s_lost_line_counter = 0;
            s_intersection_counter = 0;
            s_has_passed_intersection = false;
            continue;
        }

        if (state == ROBOT_STATE_FINISH || state == ROBOT_STATE_STOPPED) {
            jgb37_motor_brake();
            mg90s_set_angle_deg(0.0f);
            speed_control_reset(&s_speed_ctrl);
            continue;
        }

        // 2. Đọc cảm biến vạch line TCRT5000 qua HEF4051B
        line_sensor_read(&s_line_data);

        // 3. Đọc phản hồi vận tốc từ Encoder
        float current_motor_rpm = jgb37_encoder_update_rpm(dt);

        // 4. Xử lý logic tại ngã rẽ và phát hiện điểm kết thúc
        if (state == ROBOT_STATE_LINE_TRACKING) {
            // Kiểm tra ngã rẽ (nhiều cảm biến trên mảng đồng thời thấy màu đen)
            if (s_line_data.intersection_detected && !s_has_passed_intersection) {
                s_intersection_counter++;
                if (s_intersection_counter >= 3) { // Lọc chống nhiễu 30ms
                    ESP_LOGI(TAG, "Phat hien nga re! Kich hoat re nhanh theo lo trinh da chon");
                    state_set(ROBOT_STATE_INTERSECTION);
                    s_intersection_counter = 0;
                }
            } else {
                s_intersection_counter = 0;
            }

            // Kiểm tra mất line (khi đã đi qua ngã rẽ và tới vạch đích)
            if (s_line_data.line_lost) {
                s_lost_line_counter++;
                // Nếu mất line liên tục quá 15 chu kỳ (150ms) sau khi đã qua ngã rẽ -> Đã về đích!
                if (s_has_passed_intersection && s_lost_line_counter >= 15) {
                    ESP_LOGI(TAG, "Da ve dich! Tien hanh dung xe (Thoa sai so dung <= 5mm)");
                    state_set(ROBOT_STATE_FINISH);
                    jgb37_motor_brake();
                    continue;
                }
            } else {
                s_lost_line_counter = 0;
            }
        }

        // 5. Tính toán điều khiển bám quỹ đạo (Chương 5 Thuyết minh)
        float v_cmd = s_target_linear_speed;
        float omega_cmd = 0.0f;
        float gamma = 0.0f;

        if (state == ROBOT_STATE_INTERSECTION) {
            // Đang ở vùng ngã ba: bẻ lái theo nhánh đã chọn
            route_selection_t route = state_get_selected_route();
            if (route == ROUTE_UPPER) {
                // Nhánh trên: Bẻ lái sang TRÁI (bán kính cong R = 800 mm)
                // gamma = arctan(l1 / R)
                gamma = atan2f(ROBOT_L1_M, 0.8f); 
            } else {
                // Nhánh dưới: Bẻ lái sang PHẢI / Giữ thẳng (bán kính cong R = 800 mm)
                gamma = -atan2f(ROBOT_L1_M, 0.8f);
            }

            // Vận tốc qua khúc rẽ duy trì ổn định
            v_cmd = s_target_linear_speed * 0.75f;

            s_intersection_counter++;
            // Sau khi bẻ lái một khoảng (~0.5s) hoặc khi mảng cảm biến đã bám lại 1 line duy nhất
            if (s_intersection_counter > 50 || (s_intersection_counter > 20 && s_line_data.active_sensor_count <= 2)) {
                ESP_LOGI(TAG, "Da qua nga re thanh cong, tiep tuc bam line don");
                s_has_passed_intersection = true;
                s_intersection_counter = 0;
                state_set(ROBOT_STATE_LINE_TRACKING);
            }
        } 
        else if (state == ROBOT_STATE_LINE_TRACKING) {
            // Bộ điều khiển phi tuyến Lyapunov Outer-Loop (Chương 5.2):
            // v = v_d * cos(e3) + k1 * e1
            // omega = omega_d + k2 * v_d * e2 + k3 * sin(e3)

            // e2: Sai số vị trí lệch ngang (mm trong thuyết minh mục 5.2.3)
            float e2_mm = s_line_data.position_mm;
            float e2_m = s_line_data.position_m;

            // e3: Sai số góc hướng (ước lượng từ đạo hàm của sai số lệch ngang: de2 / dt = v * sin(e3))
            float de2_dt = (e2_m - s_prev_e2_m) / dt;
            s_prev_e2_m = e2_m;

            float sin_e3 = 0.0f;
            if (v_cmd > 0.01f) {
                sin_e3 = de2_dt / v_cmd;
                if (sin_e3 > 1.0f) sin_e3 = 1.0f;
                if (sin_e3 < -1.0f) sin_e3 = -1.0f;
            }
            float e3 = asinf(sin_e3);

            // Giả thiết vệt line tham chiếu thẳng/uốn nhẹ: omega_d = 0; e1 ~ 0
            float e1 = 0.0f;

            // Tính vận tốc góc omega theo công thức (5.21) Thuyết minh:
            // omega = omega_d + k2 * (v_d in mm/s) * e2_mm + k3 * sin(e3)
            // (Trong mục 5.2.3: k1 = 1.5, k2 = 0.00015, k3 = 10.0 khi tính theo mm, s)
            float v_d_mms = s_target_linear_speed * 1000.0f;
            omega_cmd = (LYAPUNOV_K2 * v_d_mms * e2_mm) + (LYAPUNOV_K3 * sin_e3);

            // Vận tốc dài điều khiển:
            v_cmd = s_target_linear_speed * cosf(e3) + (LYAPUNOV_K1 * e1);
            if (v_cmd < ROBOT_SPEED_MIN_MPS) v_cmd = ROBOT_SPEED_MIN_MPS;

            // Quy đổi sang góc lái Servo (Chương 5.2.4):
            // gamma = arctan( (l1 * omega) / v )
            gamma = atan2f(ROBOT_L1_M * omega_cmd, v_cmd);
        }

        // 6. Cập nhật góc lái Servo MG90S
        s_current_steer_rad = gamma;
        mg90s_set_angle_rad(gamma);

        // 7. Vòng điều khiển vận tốc trong (Inner Loop PI Controller - Chương 5.6)
        float pwm_output = speed_control_update(&s_speed_ctrl, v_cmd, current_motor_rpm);

        // 8. Xuất xung PWM điều khiển động cơ JGB37
        jgb37_motor_set_pwm(pwm_output);
    }
}
