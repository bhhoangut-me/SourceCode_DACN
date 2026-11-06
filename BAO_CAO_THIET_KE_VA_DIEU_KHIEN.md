# BÁO CÁO KỸ THUẬT: THIẾT KẾ VÀ ĐIỀU KHIỂN ROBOT DÒ LINE TRÁNH VẬT CẢN

**Đề tài**: Đề tài 3 (Topic 3: Line-following robot with obstacle detection)  
**Môn học**: Dự án Cơ điện tử (ME4071) - Học kỳ: 261 (2026 - 2027)  
**Trường**: Đại học Bách Khoa - ĐHQG TP. Hồ Chí Minh (HCMUT) - Khoa Cơ Khí  
**GVHD**: PGS. TS. Nguyễn Tấn Tiến  
**Nhóm sinh viên thực hiện**:
- Đoàn Trần Phương Đông (MSSV: 2310755)
- Đặng Văn Đức
- Bùi Huy Hoàng
- Đỗ Lê Anh Khoa

---

## MỤC LỤC
1. [Tổng quan Đề tài & Yêu cầu Kỹ thuật](#1-tổng-quan-đề-tài--yêu-cầu-kỹ-thuật)
2. [Thiết kế Cơ khí & Thông số Động học](#2-thiết-kế-cơ-khí--thông-số-động-học)
3. [Thiết kế Hệ thống Điện & Cảm biến](#3-thiết-kế-hệ-thống-điện--cảm-biến)
4. [Mô hình Động học & Giải thuật Điều khiển](#4-mô-hình-động-học--giải-thuật-điều-khiển)
5. [Kiến trúc Phần mềm trên FreeRTOS (ESP-IDF)](#5-kiến-trúc-phần-mềm-trên-freertos-esp-idf)
6. [Bảng Tra cứu Hàm & Cấu hình Ngoại vi](#6-bảng-tra-cứu-hàm--cấu-hình-ngoại-vi)
7. [Kết luận & Khả năng Vận hành](#7-kết-luận--khả-năng-vận-hành)

---

## 1. TỔNG QUAN ĐỀ TÀI & YÊU CẦU KỸ THUẬT

### 1.1. Mục tiêu
Thiết kế, chế tạo và lập trình điều khiển một mẫu robot tự hành (AGV) 3 bánh có khả năng bám theo vạch kẻ dẫn hướng (line đen trên nền trắng), chủ động phát hiện vật cản hình trụ tại khu vực phân nhánh bằng cảm biến khoảng cách, từ đó tự động đưa ra quyết định chọn nhánh đường thông thoáng để đi về đích an toàn.

### 1.2. Quy cách Sa bàn & Tiêu chí Đánh giá
* **Đặc tính sa bàn**:
  * Nền: Màu trắng phẳng, khô ráo.
  * Đường dẫn hướng (guideline): Màu đen, bề rộng $w_l = 26\text{ mm}$.
  * Bán kính khúc cua nhỏ nhất: $R_{min} = 500\text{ mm}$, khúc rẽ nhánh $R = 800\text{ mm}$.
* **Quy cách vật cản**:
  * Hình trụ in 3D màu trắng, đường kính $d_{obs} = 50\text{ mm}$, chiều cao $h_{obs} = 200\text{ mm}$.
  * Đặt ngẫu nhiên tại 1 trong 2 nhánh:
    * Nhánh gần: Khoảng cách từ tâm robot đến tâm vật cản $\approx 500\text{ mm}$ (khoảng cách quang học tới bề mặt phản xạ $d \approx 475\text{ mm}$).
    * Nhánh xa: Khoảng cách $\approx 1500\text{ mm}$.
* **Chỉ tiêu vận hành**:
  * Vận tốc di chuyển: Tối thiểu $v_{min} = 0.1\text{ m/s}$, vận tốc thiết kế tối đa $v_{max} = 2.0\text{ m/s}$.
  * Sai số bám line (tính từ mép vạch): Không vượt quá $\pm 3\text{ mm}$ (tương đương $\pm 16\text{ mm}$ so với tâm mảng cảm biến).
  * Sai số dừng tại vạch kết thúc: Không vượt quá $\pm 5\text{ mm}$.

---

## 2. THIẾT KẾ CƠ KHÍ & THÔNG SỐ ĐỘNG HỌC

### 2.1. Cấu hình Di chuyển
Robot sử dụng cấu hình lai 3 bánh (**Hybrid Tricycle - Differential**):
* **Bánh trước**: 1 bánh dẫn hướng chủ động, điều khiển bằng động cơ RC Servo MG90S qua cơ cấu 4 khâu hình bình hành. Trục lái đi qua tâm tiếp xúc bánh xe (**Zero-trail configuration**), giúp servo kiểm soát trực tiếp góc lái mà không bị ảnh hưởng bởi mô-men phản lực hoàn nguyên.
* **Cụm cầu sau**: 2 bánh dẫn động thông qua bộ vi sai **WLToys 144001** (tỉ số truyền $u_{dg} = 2.5$), nhận mô-men xoắn từ 1 động cơ DC trục sau qua khớp nối mềm. Cơ cấu vi sai tự động phân phối tốc độ giữa hai bánh khi vào cua, loại bỏ nhu cầu sử dụng 2 động cơ riêng biệt, giảm trọng lượng và đơn giản hóa điều khiển.

### 2.2. Bảng Thông số Cơ học Hệ thống
| Tham số | Ký hiệu | Giá trị | Đơn vị | Ghi chú |
| :--- | :---: | :---: | :---: | :--- |
| Khối lượng toàn bộ xe | $M$ | $1.5$ | $\text{kg}$ | Khối lượng thiết kế |
| Chiều dài cơ sở | $l$ | $225$ | $\text{mm}$ | Trục sau đến trục lái trước |
| Vệt bánh sau | $w$ | $150$ | $\text{mm}$ | Khoảng cách 2 bánh sau |
| Vị trí trọng tâm (từ trục sau) | $l_1$ | $50$ | $\text{mm}$ | |
| Vị trí mảng cảm biến (từ trục sau) | $l_2$ | $175$ | $\text{mm}$ | Cự ly quan sát trước (look-ahead) |
| Chiều cao trọng tâm | $h$ | $50$ | $\text{mm}$ | Thấp hơn ngưỡng lật ($92\text{ mm}$) |
| Đường kính bánh dẫn động | $D$ | $80$ | $\text{mm}$ | Bán kính $R_w = 40\text{ mm}$ |
| Tỉ số truyền vi sai | $u_{dg}$ | $2.5$ | $-$ | Bộ vi sai WLToys 144001 |
| Gia tốc tối đa an toàn | $a$ | $2.5$ | $\text{m/s}^2$ | Thỏa mãn chống bốc đầu $a < 9.8\text{ m/s}^2$ |

### 2.3. Lựa chọn Động cơ & Cơ cấu Chấp hành
* **Động cơ kéo sau**: DC Gearmotor **JGB37-3530-24100**
  * Điện áp định mức: $24\text{ V}$.
  * Tốc độ không tải: $1600\text{ RPM}$, tốc độ có tải định mức: $1231\text{ RPM} > 1194\text{ RPM}$ (đáp ứng đủ vận tốc $2\text{ m/s}$).
  * Mô-men định mức: $0.5\text{ kg}\cdot\text{cm} > 0.408\text{ kg}\cdot\text{cm}$ yêu cầu.
* **Động cơ lái trước**: **RC Servo MG90S**
  * Mô-men giữ: $2.0\text{ kg}\cdot\text{cm} \gg 0.133\text{ kg}\cdot\text{cm}$ (mô-men cản quay yêu cầu).

---

## 3. THIẾT KẾ HỆ THỐNG ĐIỆN & CẢM BIẾN

### 3.1. Phân phối Nguồn 3 Nhánh Độc lập
Hệ thống sử dụng pin chính **LiPo 3S 11.1V (2200 mAh, 35C)**:
1. **Nhánh 3.3V (Logic & Sensing)**: Hạ áp qua IC Buck **LM2596S-3.3**, cấp nguồn cho MCU ESP32-WROOM-32E, mảng 7 cảm biến TCRT5000, MUX HEF4051B, cảm biến VL53L0X và Encoder. Công suất tiêu thụ $\approx 2.05\text{ W}$.
2. **Nhánh 5.0V (Steering Servo)**: Hạ áp qua IC Buck **LM2596S-5.0**, cấp nguồn riêng cho Servo MG90S nhằm cách ly sụt áp và xung nhiễu khi bẻ lái. Công suất thiết kế $10\text{ W}$ ($2\text{ A}$ đỉnh).
3. **Nhánh 24V (Motor Power)**: Nâng áp qua module **Boost DC-DC 400W**, cấp cho mạch cầu H DRV8871 và động cơ kéo JGB37 (công suất cực đại lúc khởi động/stall đạt $72\text{ W}$).

### 3.2. Mảng Cảm biến Dò Line (7 mắt TCRT5000 + MUX HEF4051B)
* **Số lượng**: 7 cảm biến hồng ngoại phản xạ TCRT5000 xếp ngang.
* **Độ cao lắp đặt**: $h = 7\text{ mm}$ (cho độ chênh lệch ADC lớn nhất giữa vạch đen $2859$ và nền trắng $420$, $\Delta = 2439$).
* **Bước mắt cảm biến**: $p = 13\text{ mm}$ ($w_l / 2 = 26 / 2$).
* **Tọa độ các cảm biến so với tâm xe**: $x_i = [-39, -26, -13, 0, 13, 26, 39]\text{ mm}$ (tổng bề rộng quét $L = 78\text{ mm}$).
* **Giao tiếp MUX**: 7 ngõ ra analog nối vào ngõ $Y0 \div Y6$ của IC **HEF4051B**, quét tuần tự qua 3 chân GPIO Select (ss1: GPIO 19, ss2: GPIO 18, ss3: GPIO 17) và đọc về 1 chân duy nhất `GPIO 36` (ADC1 Channel 0 của ESP32).

### 3.3. Cảm biến Khoảng cách Laser VL53L0X
* **Nguyên lý**: Đo thời gian bay của chùm laser (Time-of-Flight), bước sóng $940\text{ nm}$, không bị ảnh hưởng bởi màu sắc bề mặt.
* **Bù sai số tĩnh**: Thuyết minh mục 5.4.3 ghi nhận giá trị đo thô trung bình ở cự ly $475\text{ mm}$ là $516.6\text{ mm}$, phần mềm bù trừ offset `OBSTACLE_CALIB_OFFSET_MM = -41.4 mm` để đạt kết quả chuẩn xác $475.2\text{ mm}$.

---

## 4. MÔ HÌNH ĐỘNG HỌC & GIẢI THUẬT ĐIỀU KHIỂN

### 4.1. Hệ Điều khiển Tầng (Cascade Control Architecture)

```
[Vị trí Line tham chiếu]
          │
          ▼
   (Sai số e2, e3) ──► ┌────────────────────────────────────────┐
                       │     VÒNG NGOÀI (Outer Loop)           │
                       │ Bộ điều khiển bám quỹ đạo Lyapunov    │
                       └──────────────┬────────────────────────┘
                                      │
                   ┌──────────────────┴──────────────────┐
                   ▼                                     ▼
        Góc bẻ lái đặt γ                      Vận tốc dài đặt v_cmd
                   │                                     │
                   ▼                                     ▼
       ┌──────────────────────┐               ┌────────────────────────┐
       │   Servo Lái MG90S    │               │  Quy đổi ra n_ref(RPM) │
       └──────────────────────┘               └──────────┬─────────────┘
                                                         │ (Sai số tốc độ)
                                                         ▼
                                              ┌────────────────────────┐
                                              │  VÒNG TRONG (Inner)    │
                                              │ Bộ điều khiển PI Motor │
                                              └──────────┬─────────────┘
                                                         │ (PWM 0-255)
                                                         ▼
                                              ┌────────────────────────┐
                                              │   Động cơ kéo JGB37    │
                                              └──────────┬─────────────┘
                                                         │ (Đo RPM)
                                                         ▼
                                              ┌────────────────────────┐
                                              │     Encoder JGB37      │
                                              └────────────────────────┘
```

### 4.2. Bộ điều khiển Vòng Ngoài: Bám Quỹ Đạo Theo Lyapunov (Chương 5.2)
Hệ phương trình động học sai số trong hệ tọa độ thân xe:
$$\begin{cases}
\dot{e}_1 = v_d \cos e_3 - v + \omega e_2 \\
\dot{e}_2 = v_d \sin e_3 - \omega (l_2 + e_2) \\
\dot{e}_3 = \omega_d - \omega
\end{cases}$$

Chọn hàm thế năng Lyapunov:
$$V = \frac{1}{2}(e_1^2 + e_2^2) + \frac{1 - \cos e_3}{k_2} \ge 0$$

Luật điều khiển phi tuyến đảm bảo $\dot{V} \le 0$:
$$\begin{cases}
v = v_d \cos e_3 + k_1 e_1 \\
\omega = \omega_d + k_2 v_d e_2 + k_3 \sin e_3
\end{cases}$$
* **Hệ số đã tinh chỉnh thực nghiệm**: $k_1 = 1.5$, $k_2 = 0.00015$, $k_3 = 10.0$ (khi tính $e_2$ theo $\text{mm}$, vận tốc theo $\text{mm/s}$).
* **Quy đổi lệnh chấp hành**:
  $$\gamma = \arctan\left(\frac{l_1 \omega}{v}\right) \quad (\text{với } l_1 = 0.05\text{ m})$$
  $$\omega_m = \frac{v}{\pi D} \cdot 60 \cdot u_{dg} \quad (\text{với } D = 0.08\text{ m}, u_{dg} = 2.5)$$

### 4.3. Bộ điều khiển Vòng Trong: Tốc độ Động cơ PI (Chương 5.6)
* Hàm truyền nhận dạng thực nghiệm: $G(s) = \dfrac{33.0998}{s + 5.9748}$.
* Hệ số PI tinh chỉnh bằng MATLAB PID Tuner: $K_P = 0.3975$, $K_I = 2.3751$, deadband $= 15$, chu kỳ $T_s = 10\text{ ms}$.

### 4.4. Thuật toán Ra Quyết định Tránh Vật cản (Chương 5.4.1)
Do tín hiệu ToF suy giảm đối với trụ tròn $\varnothing 50\text{ mm}$ ở cự ly xa ($> 700\text{ mm}$), thuật toán quy đổi về bài toán **Quyết định nhị phân (Binary Decision)** đo tại nhánh gần:
* Nếu khoảng cách đo được $\le 700\text{ mm}$ (vật cản xuất hiện ở nhánh gần $\approx 475\text{ mm}$):  
  $\rightarrow$ Quyết định chọn **Nhánh trên (`ROUTE_UPPER`)**.
* Nếu khoảng cách đo $> 700\text{ mm}$ hoặc out of range (nhánh gần thông thoáng, vật cản nằm ở nhánh xa):  
  $\rightarrow$ Quyết định chọn **Nhánh dưới (`ROUTE_LOWER`)**.

---

## 5. KIẾN TRÚC PHẦN MỀM TRÊN FREERTOS (ESP-IDF)

### 5.1. Phân chia Lõi & Mức độ Ưu tiên Task
| Tên Task | Core | Priority | Chu kỳ / Cơ chế | Nhiệm vụ chính |
| :--- | :---: | :---: | :---: | :--- |
| **`Control_Task`** | **Core 1** | **22** (Cao nhất) | Định kỳ $10\text{ ms}$ (`vTaskDelayUntil`) | Đọc 7 TCRT5000 $\rightarrow$ tính $e_2, e_3$ $\rightarrow$ Lyapunov outer loop $\rightarrow$ góc lái MG90S; đọc Encoder $\rightarrow$ PI inner loop $\rightarrow$ PWM DRV8871. |
| **`Obstacle_Task`** | **Core 0** | **3** | Sự kiện (Event-driven) | Khi nhận lệnh `DETECT_OBSTACLE`, kích hoạt đo VL53L0X 20 mẫu, xác định nhánh `ROUTE_UPPER`/`ROUTE_LOWER`, chuyển cờ sang `LINE_TRACKING`. |
| **`State_Task`** | **Core 0** | **2** | Định kỳ $50 \div 200\text{ ms}$ | Quản lý máy trạng thái robot, giám sát nút `BOOT` (GPIO 0), điều khiển nhấp nháy LED Đỏ (GPIO 17) & LED Xanh (GPIO 15). |

### 5.2. Máy Trạng thái (FSM)
1. `ROBOT_STATE_INIT`: Khởi tạo ngoại vi, đặt góc lái $0^\circ$, động cơ dừng.
2. `ROBOT_STATE_WAIT_START`: LED xanh chớp chậm ($200\text{ ms}$), chờ nhấn nút `BOOT`.
3. `ROBOT_STATE_DETECT_OBSTACLE`: Bật đèn báo, quét ToF, lưu hướng rẽ.
4. `ROBOT_STATE_LINE_TRACKING`: Robot tăng tốc mượt, bám line theo Lyapunov + PI motor.
5. `ROBOT_STATE_INTERSECTION`: Phát hiện vạch ngã ba ($\ge 4$ cảm biến thấy vạch), bẻ lái cưỡng bức theo bán kính cong $R = 800\text{ mm}$ theo nhánh đã chọn.
6. `ROBOT_STATE_FINISH`: Phát hiện vạch dừng / mất line sau ngã rẽ $\rightarrow$ kích hoạt phanh điện động (`brake`), khóa lái $0^\circ$, LED xanh sáng cố định.

---

## 6. BẢNG TRA CỨU HÀM & CẤU HÌNH NGOẠI VI

### 6.1. File Cấu hình Hệ thống
* [`pin_config.h`](file:///c:/Users/Administrator/Desktop/From_GIT/DACN/DACN/main/config/pin_config.h): Định nghĩa toàn bộ chân GPIO chuẩn theo mạch schematic thực tế.
* [`robot_config.h`](file:///c:/Users/Administrator/Desktop/From_GIT/DACN/DACN/main/config/robot_config.h): Chứa các hằng số vật lý ($M, l, w, D$), tọa độ 7 mắt cảm biến, hệ số Lyapunov và hệ số PI.

### 6.2. Driver Modules
* **MG90S Servo** (`drivers/mg90s.h`):
  * `mg90s_init()`: Cấu hình LEDC PWM 50Hz.
  * `mg90s_set_angle_rad(float rad)`: Đặt góc bẻ lái bánh trước theo radian.
  * `mg90s_stop()`: Tắt xung lái.
* **JGB37 Motor** (`drivers/jgb37_motor.h`):
  * `jgb37_motor_init()`: Cấu hình PWM 20kHz cho 2 kênh cầu H.
  * `jgb37_motor_set_pwm(float pwm)`: Xuất xung từ $-255$ đến $+255$.
  * `jgb37_motor_brake()`: Hãm dừng điện động để đạt chuẩn sai số dừng $\le 5\text{ mm}$.
* **JGB37 Encoder** (`drivers/jgb37_encoder.h`):
  * `jgb37_encoder_init()`: Cài đặt ngắt GPIO trên GPIO 34, 35.
  * `jgb37_encoder_update_rpm(float dt)`: Tính toán vòng tua động cơ trong chu kỳ lấy mẫu $dt$.
* **TCRT5000 Line Sensor** (`drivers/tctr5000.h`):
  * `line_sensor_init()`: Thiết lập GPIO chọn kênh MUX HEF4051B và ADC1 Channel 4.
  * `line_sensor_read(line_sensor_data_t *data)`: Quét 7 kênh, tính sai số vị trí $e_2$, phát hiện ngã rẽ và mất line.
* **VL53L0X ToF Sensor** (`drivers/vl53l0x.h`):
  * `vl53l0x_init()`: Thiết lập bus I2C0 và cấu hình cảm biến.
  * `vl53l0x_decide_route(float *measured_dist)`: Thuật toán quyết định nhị phân chọn lộ trình.

### 6.3. Control Modules
* **PID Controller** (`control/pid.h`):
  * `pid_init()`: Khởi tạo thông số và giới hạn chống bão hòa tích phân.
  * `pid_compute()`: Tính ngõ ra PID có bù deadband.
* **Speed Controller** (`control/speed_control.h`):
  * `speed_control_init()`: Tải các hệ số $K_P = 0.3975, K_I = 2.3751$.
  * `speed_control_update()`: Vòng kín điều khiển tốc độ từ vận tốc $v$ mục tiêu sang xung PWM động cơ.

---

## 7. KẾT LUẬN & KHẢ NĂNG VẬN HÀNH
Toàn bộ mã nguồn đã được tổ chức theo kiến trúc module sạch, phân tách rõ ràng giữa tầng giao tiếp phần cứng (HAL/Driver), tầng thuật toán toán học (Kinematic/Control) và tầng quản lý tiến trình thời gian thực (FreeRTOS Tasks). Hệ thống đã sẵn sàng để nạp code và chạy thử nghiệm thực tế trên sa bàn thi đấu.

