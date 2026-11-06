# HỆ THỐNG ROBOT DÒ LINE TRÁNH VẬT CẢN (MECHATRONICS PROJECT - HCMUT)
> Tài liệu kỹ thuật tổng hợp & lưu trữ dữ liệu từ `Thuyết minh.pdf` (ME4071 - Đề tài 3).

---

## 1. THÔNG TIN CHUNG
- **Đề tài**: THIẾT KẾ ROBOT DÒ ĐƯỜNG PHÁT HIỆN VÀ TRÁNH VẬT CẢN (Topic 3: Line-following robot with obstacle detection).
- **Học kỳ**: 261 / Năm học: 2026 – 2027.
- **GVHD**: PGS. TS. Nguyễn Tấn Tiến.
- **Nhóm sinh viên**:
  - Đoàn Trần Phương Đông (MSSV: 2310755)
  - Đặng Văn Đức
  - Bùi Huy Hoàng
  - Đỗ Lê Anh Khoa
- **Mục tiêu**: Nghiên cứu, thiết kế, mô phỏng và chế tạo mẫu robot tự hành dò line theo quỹ đạo định trước, tự động nhận diện vật cản tại vùng rẽ nhánh và chủ động chọn đường đi an toàn về đích.

---

## 2. YÊU CẦU KỸ THUẬT & SA BÀN (ARENA)
- **Màu nền**: Trắng.
- **Đường line**: Màu đen, độ rộng line $w_l = 26\text{ mm}$.
- **Bán kính cong nhỏ nhất**: $R_{min} = 500\text{ mm}$.
- **Vận tốc**:
  - Vận tốc tối thiểu: $v_{min} = 0.1\text{ m/s}$.
  - Vận tốc thiết kế tối đa: $v_{max} = 2.0\text{ m/s}$.
- **Sai số cho phép**:
  - Sai số bám line (tính từ mép trái hoặc phải của vạch): $\pm 3\text{ mm}$ (tương đương $\pm 16\text{ mm}$ so với tâm cảm biến).
  - Sai số vị trí dừng tại điểm kết thúc: $\pm 5\text{ mm}$.
- **Quy cách vật cản**:
  - Hình trụ màu trắng (in 3D), đường kính $d_{obs} = 50\text{ mm}$, chiều cao $h_{obs} = 200\text{ mm}$.
- **Vị trí vật cản & bài toán chọn đường**:
  - Từ điểm "Bắt đầu", robot chạy theo line đến ngã rẽ nhánh dẫn đến 2 điểm "Kết thúc".
  - Nhánh gần: khoảng cách từ tâm robot đến tâm vật cản $\approx 500\text{ mm}$ (khoảng cách quang học tới bề mặt $\approx 475\text{ mm}$).
  - Nhánh xa: khoảng cách $\approx 1500\text{ mm}$.
  - Chiến lược: Do cảm biến ToF VL53L0X suy giảm tín hiệu ở cự ly xa ($>700\text{ mm}$ khó bắt trụ phi 50), thuật toán quy về bài toán nhị phân (Binary decision) tại khoảng cách kiểm tra $\approx 475\text{ mm}$:
    - Nếu phát hiện vật cản ở $\approx 475\text{ mm}$ (nhánh gần bị chặn) $\rightarrow$ Rẽ nhánh trên (Case 1).
    - Nếu không phát hiện vật cản ở nhánh gần ($>700\text{ mm}$ hoặc out of range) $\rightarrow$ Vật cản nằm ở nhánh xa $\rightarrow$ Rẽ nhánh dưới (Case 2).

---

## 3. THÔNG SỐ CƠ KHÍ & ĐỘNG LỰC HỌC
- **Cấu hình di chuyển**: Dạng xe 3 bánh lai Tricycle & Vi sai (Hybrid Tricycle - Differential):
  - 1 bánh trước dẫn hướng (chủ động bẻ lái qua RC Servo MG90S và cơ cấu 4 khâu hình bình hành, zero-trail geometry).
  - 2 bánh sau dẫn động qua cụm vi sai WLToys 144001, truyền động từ 1 động cơ DC phía sau qua khớp nối mềm.
- **Khối lượng xe**: $M = 1.5\text{ kg}$.
- **Hệ số ma sát**: $\mu = 0.4$, hệ số an toàn $h_s = 1.5$.
- **Hiệu suất truyền động**:
  - Khớp các đăng: $\eta_c = 0.96$.
  - Hộp số / vi sai: $\eta_g = 0.95$.
  - Tỉ số truyền vi sai: $u_{dg} = 2.5$.
- **Kích thước hình học sơ bộ**:
  - Chiều rộng vệt bánh sau: $w = 150\text{ mm} = 0.15\text{ m}$.
  - Chiều dài cơ sở (trục sau đến trục lái trước): $l = 225\text{ mm} = 0.225\text{ m}$ (tỷ lệ $l/w = 1.5$).
  - Khoảng cách từ trục sau đến trọng tâm G: $l_1 = 50\text{ mm} = 0.05\text{ m}$.
  - Khoảng cách từ trục trước đến trọng tâm G: $l_2 = 175\text{ mm} = 0.175\text{ m}$.
  - Chiều cao trọng tâm G: $h = 50\text{ mm} = 0.05\text{ m}$.
- **Điều kiện chống trượt lật (Anti-rollover)**:
  - Chiều cao trọng tâm cho phép: $h \le \frac{w g R}{2 v^2} = 92\text{ mm}$ (với $h = 50\text{ mm}$, vận tốc trượt lật an toàn lên đến $2.71\text{ m/s} > 2.0\text{ m/s}$).
- **Điều kiện chống bốc đầu (Vertical flip / Wheelie)**:
  - Để bánh trước không nhấc khỏi mặt đất ($N_2 > 0$): Gia tốc $a < \frac{g l_1}{h} = 9.8\text{ m/s}^2$.
  - Gia tốc chọn: $a = 2.5\text{ m/s}^2$.
- **Bánh xe dẫn động (Driving Wheel)**:
  - Đường kính bánh xe: $D = 80\text{ mm} = 0.08\text{ m}$ ($R_w = 40\text{ mm} = 0.04\text{ m}$).
  - Vòng tua bánh xe tại $v = 2\text{ m/s}$: $n = \frac{60 \times 2}{2\pi \times 0.04} \approx 477.5\text{ RPM}$.
  - Vòng tua động cơ yêu cầu (qua tỉ số truyền $u_{dg} = 2.5$): $n_{motor} \approx 1194\text{ RPM}$.
  - Công suất cơ học yêu cầu: $P_{req} \approx 5.48\text{ W}$.
- **Động cơ dẫn động được chọn**:
  - Loại: DC Gearmotor **JGB37-3530-24100**.
  - Điện áp định mức: $24\text{ V}$.
  - Tốc độ không tải: $1600\text{ RPM}$.
  - Tốc độ có tải định mức: $1231\text{ RPM}$.
  - Mô-men định mức: $0.5\text{ kg}\cdot\text{cm}$.
  - Dòng định mức: $0.63\text{ A}$, Dòng kẹt (stall): $\approx 3\text{ A}$.
- **Cơ cấu lái trước (Steering)**:
  - Mô-men lái cần thiết tại điểm A: $\tau_{steering} \approx 0.133\text{ kg}\cdot\text{cm}$.
  - Động cơ lái chọn: **RC Servo MG90S** (mô-men $2.0\text{ kg}\cdot\text{cm}$).

---

## 4. HỆ THỐNG ĐIỆN & CẢM BIẾN
- **Vi điều khiển trung tâm**: **ESP32-WROOM-32E** (3.3V, xung nhịp dual-core 240 MHz).
- **Phân chia 3 đường nguồn**:
  - **Nguồn chính**: Pin LiPo 3S (11.1V danh định, sạc đầy 12.6V, xả tối thiểu 10.0V), dung lượng $2200\text{ mAh}$, dòng xả $35\text{C}$.
  - **Nhánh 3.3V (Logic & Sensing)**: Hạ áp qua IC Buck **LM2596S-3.3** (cấp cho ESP32, mảng cảm biến TCRT5000, IC HEF4051B, VL53L0X, Encoder). Công suất tính toán $\approx 2.05\text{ W}$ (dự phòng thiết kế 1A / 3.3W).
  - **Nhánh 5V (Steering Servo)**: Hạ áp qua IC Buck **LM2596S-5.0** (cấp cho Servo MG90S, dòng cực đại $2\text{ A}$, công suất $10\text{ W}$).
  - **Nhánh 24V (Motor Drive)**: Tăng áp qua **Mạch DC-DC Boost 400W** từ 11.1V lên 24V cấp cho mạch cầu H và động cơ JGB37 (công suất đỉnh lúc stall $72\text{ W}$).
- **Mảng cảm biến dò line (TCRT5000 Array)**:
  - Số lượng: **7 cảm biến TCRT5000**.
  - Hướng đặt: **Position 2** (trục phát-thu song song với biên line đen-trắng để tăng độ mượt tín hiệu tương tự).
  - Độ cao lắp đặt tối ưu: $h_{sensor} = 7\text{ mm}$ (độ chênh lệch ADC lớn nhất: vạch đen 2859, vạch trắng 420, $\Delta = 2439$).
  - Khoảng cách giữa các cảm biến: bước $p = 13\text{ mm}$ ($w_l / 2 = 26 / 2$).
  - Tọa độ 7 cảm biến so với tâm: $x_i = [-39, -26, -13, 0, 13, 26, 39]\text{ mm}$. Tổng bề rộng quét $L = 78\text{ mm}$.
  - Mạch phân cực: Điện trở LED $R_1 = 220\,\Omega$ ($I_F = 10\text{ mA}, V_F = 1.1\text{ V}$), điện trở kéo Transistor $R_2 = 2.4\text{ k}\Omega$, tụ lọc RC $C = 100\text{ nF}$ ($f_c \approx 663\text{ Hz}$).
  - Mạch ghép kênh: IC **HEF4051B** (8-to-1 Multiplexer). 7 ngõ ra nối vào $Y0 \dots Y6$, điều khiển quét tuần tự bằng 3 chân GPIO Select (ss1, ss2, ss3), 1 chân ngõ ra $Z$ đưa về 1 kênh ADC của ESP32.
  - Thuật toán ước lượng sai số: Phương pháp trung bình trọng số (Weighted Average Method).
- **Cảm biến khoảng cách phát hiện vật cản**:
  - Loại: **VL53L0X** (Time-of-Flight Laser Sensor) qua giao thức I2C.
  - Hiệu chuẩn tĩnh tại cự ly $475\text{ mm}$: Đo thô trung bình $516.6\text{ mm} \rightarrow$ bù trừ offset để giá trị đo đạt chuẩn $475.2\text{ mm}$.
- **Mạch công suất động cơ**:
  - Mạch cầu H: **DRV8871** (hoặc chân tương đương PWM1/PWM2).

---

## 5. CẤU HÌNH SƠ ĐỒ CHÂN (PINOUT MAP)
Theo sơ đồ chân phần cứng thực tế và `pin_config.h`:
| Chức năng | Ký hiệu trong mạch | Chân GPIO ESP32 | Ghi chú |
| :--- | :--- | :--- | :--- |
| **Encoder Kênh A** | ENC_A | GPIO 34 | Input only |
| **Encoder Kênh B** | ENC_B | GPIO 35 | Input only |
| **Động cơ DC - PWM 1** | TB67_IN1 / PWM1 | GPIO 32 | Điều khiển tốc độ/chiều |
| **Động cơ DC - PWM 2** | TB67_IN2 / PWM2 | GPIO 33 | Điều khiển tốc độ/chiều |
| **Servo Lái MG90S** | MG90S_PWM | GPIO 25 | PWM 50 Hz |
| **Line Mux Select 1** | ss1 | GPIO 19 | HEF4051B S0/A (LSB) |
| **Line Mux Select 2** | ss2 | GPIO 18 | HEF4051B S1/B |
| **Line Mux Select 3** | ss3 | GPIO 17 | HEF4051B S2/C (MSB) |
| **Line Mux ADC Out** | z | GPIO 36 | ADC1 CH0 (SENSOR_VP) |
| **VL53L0X SDA** | SDA | GPIO 16 | I2C Data |
| **VL53L0X SCL** | SCL | GPIO 4 | I2C Clock |
| **Status LED Red** | LED_R | GPIO 27 | Báo trạng thái |
| **Status LED Green** | LED_G | GPIO 26 | Báo trạng thái |
| **UART RX / TX** | RX0 / TX0 | GPIO 3 / GPIO 1 | Nạp & Debug |
| **Boot Mode** | BOOT | GPIO 0 | Pull-up / Boot |

---

## 6. MÔ HÌNH HỌC & ĐỘNG HỌC (KINEMATICS MODEL)
- Điểm $A$: Trung điểm trục sau (vận tốc dài $v$).
- Điểm $B$: Trục quay bánh lái trước (vận tốc dài $v_B$, góc lái $\gamma$).
- Điểm $C$: Trung điểm mảng cảm biến line phía trước (cách $A$ khoảng cách $l_2$).
- Phương trình vi phân sai số trong hệ quy chiếu thân xe (Body frame):
  $$\begin{cases} \dot{e}_1 = v_d \cos e_3 - v + \omega e_2 \\ \dot{e}_2 = v_d \sin e_3 - \omega(l_2 + e_2) \\ \dot{e}_3 = \omega_d - \omega \end{cases}$$
  Trong đó:
  - $e_1$: Sai số dọc (longitudinal error).
  - $e_2$: Sai số bám ngang (lateral error).
  - $e_3 = \theta_R - \theta$: Sai số góc hướng (heading error).
  - $v_d, \omega_d$: Vận tốc dài và vận tốc góc đặt của quỹ đạo tham chiếu.

---

## 7. THIẾT KẾ BỘ ĐIỀU KHIỂN (CASCADE CONTROLLER)
Hệ thống sử dụng cấu trúc điều khiển phân tầng:

### 7.1. Vòng ngoài: Bộ điều khiển bám quỹ đạo Lyapunov (Kinematic Loop)
- **Hàm Lyapunov mục tiêu**:
  $$V = \frac{1}{2}(e_1^2 + e_2^2) + \frac{1 - \cos e_3}{k_2} \ge 0$$
- **Đạo hàm Lyapunov**:
  $$\dot{V} = -k_1 e_1^2 - k_2 l_2 v_d e_2^2 - \frac{k_3}{k_2}\sin^2 e_3 - e_2 l_2 (\omega_d + k_3 \sin e_3) \le 0$$
- **Luật điều khiển phi tuyến đề xuất**:
  $$\begin{cases} v = v_d \cos e_3 + k_1 e_1 \\ \omega = \omega_d + k_2 v_d e_2 + k_3 \sin e_3 \end{cases}$$
- **Bộ thông số độ lợi sơ bộ đã kiểm chứng**:
  - $k_1 = 1.5$
  - $k_2 = 0.00015$
  - $k_3 = 10.0$

### 7.2. Chuyển đổi lệnh điều khiển sang cơ cấu chấp hành:
1. **Góc bẻ lái bánh trước (Servo MG90S)**:
   $$\gamma = \arctan\left(\frac{l_1 \omega}{v}\right)$$
   *(Trong đó $l_1 = 0.05\text{ m}$ là khoảng cách từ trục sau đến trục lái).*
2. **Tốc độ góc động cơ DC bánh sau ($\omega_m$, đơn vị RPM)**:
   $$\omega_m = \frac{2 v}{D} \cdot \frac{60}{2\pi} \cdot u_{dg} = \frac{60 \cdot u_{dg}}{\pi D} v$$
   *(Với $D = 0.08\text{ m}$, $u_{dg} = 2.5$).*

### 7.3. Vòng trong: Bộ điều khiển tốc độ động cơ DC (Motor Speed PI Controller)
- **Hàm truyền động cơ nhận dạng thực nghiệm** (System Identification Toolbox, độ khớp 97.46%):
  $$G_m(s) = \frac{33.0998}{s + 5.9748}$$
  (Hằng số thời gian $\tau = 0.17\text{ s}$, đáp ứng ổn định $\sim 0.5\text{ s}$).
- **Chu kỳ lấy mẫu điều khiển**: $T_s = 10\text{ ms}$ ($100\text{ Hz}$).
- **Vùng chết động cơ (Deadband)**: Giá trị PWM từ $0 \dots 15$.
- **Bộ điều khiển PI**:
  $$G_c(s) = K_P + \frac{K_I}{s}$$
- **Hệ số PI thiết kế giải tích**: $K_P = 0.62$, $K_I = 5.37$.
- **Hệ số PI tinh chỉnh tối ưu (MATLAB PID Tuner & Thực nghiệm)**:
  - $K_P = 0.3975$
  - $K_I = 2.3751$
- **Kết quả thực nghiệm**: Đáp ứng bước với tốc độ đặt $717\text{ RPM}$ đạt thời gian xác lập nhanh, độ vọt lố nhỏ $\approx 4.6\%$, sai số xác lập cực thấp $\approx 0.50\%$ ($720.56\text{ RPM}$).

---

## 8. LƯU ĐỒ THUẬT TOÁN ĐIỀU KHIỂN HỆ THỐNG
1. **Khởi động**: Khởi tạo ngoại vi (GPIO, PWM, ADC Mux, I2C VL53L0X, Timer $10\text{ ms}$).
2. **Tiền trinh sát vật cản (Proactive Obstacle Detection)**:
   - Đọc khoảng cách từ VL53L0X trước khi xe xuất phát hoặc trước khi tới ngã rẽ.
   - Nếu khoảng cách trong phạm vi nhánh gần ($\sim 475\text{ mm} \pm 50\text{ mm}$) $\rightarrow$ Gán cờ `FLAG_BRANCH_UPPER` (Rẽ trái/trên).
   - Nếu khoảng cách $>700\text{ mm}$ (ngoài tầm bắt) $\rightarrow$ Gán cờ `FLAG_BRANCH_LOWER` (Rẽ phải/dưới).
3. **Vòng lặp bám line**:
   - Đọc 7 mắt TCRT5000 qua HEF4051B $\rightarrow$ Tính sai số lệch ngang $e_2$ và góc lệch $e_3$.
   - Tính toán luật điều khiển Lyapunov $\rightarrow (v, \omega)$.
   - Chuyển đổi $(v, \omega) \rightarrow (\gamma, \omega_m)$.
   - Điều khiển Servo góc $\gamma$.
   - Vòng lặp kín PI $10\text{ ms}$ cho động cơ DC bám theo $\omega_m$.
4. **Xử lý ngã rẽ (Intersection Handling)**:
   - Phát hiện vạch giao cắt ngã rẽ.
   - Thực hiện lệnh đánh lái theo hướng cờ đã định (`UPPER` hoặc `LOWER`).
   - Tái bắt line sau khi qua vùng rẽ.
5. **Dừng xe**: Khi phát hiện vạch kết thúc (End of Line) $\rightarrow$ hãm dừng động cơ trong phạm vi sai số $\pm 5\text{ mm}$.
