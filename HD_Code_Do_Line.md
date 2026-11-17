# HỆ THỐNG ĐIỀU KHIỂN ROBOT DÒ LINE VÀ NÉ VẬT CẢN (STM32F103C8T6)

Tài liệu này mô tả chi tiết kiến trúc phần cứng, cấu hình ngoại vi trên **STM32CubeMX**, cơ sở toán học của bộ điều khiển và cấu trúc mã nguồn C/HAL cho đồ án Robot dò line né vật cản. Các thành viên trong nhóm vui lòng đọc kỹ tài liệu này trước khi chỉnh sửa thông số hoặc nạp code xuống mạch thực tế.

---

## 1. Tổng quan Hệ thống & Thông số Kỹ thuật

### 1.1. Yêu cầu vận hành trên sa bàn
* **Đặc tính đường line:** Nền trắng, vạch dẫn hướng màu đen rộng $26\text{ mm}$.
* **Lộ trình di chuyển:** Bắt đầu $\to$ Bám line thẳng $\to$ Phát hiện và vòng qua vật cản (khối trụ đường kính $50\text{ mm}$) $\to$ Bám qua cung tròn $R500$ $\to$ Nhận diện ngã rẽ (chọn nhánh Trái hoặc Phải qua cung $R800$) $\to$ Bám line về đích và dừng xe.
* **Chỉ tiêu kỹ thuật:**
  * Vận tốc cơ bản: $v = 0.30\text{ m/s}$ (tối đa $0.50\text{ m/s}$, tối thiểu $0.10\text{ m/s}$).
  * Sai số bám đường dẫn: $\pm 3\text{ mm}$.
  * Sai số vị trí dừng cuối đường dẫn: $\pm 5\text{ mm}$.

### 1.2. Thông số cơ khí & Phần cứng
* **Cấu hình xe:** 2 bánh chủ động đặt phía trước, 2 bánh mắt trâu (caster) bị động đặt phía sau.
  * Bán kính bánh xe chủ động: $r = 40\text{ mm} = 0.040\text{ m}$.
  * Khoảng cách giữa hai bánh chủ động: $b = 210\text{ mm} = 0.210\text{ m}$.
  * Khoảng cách từ trục hai bánh chủ động đến tâm dãy cảm biến dò line (nhô về phía trước): $d = 50\text{ mm} = 0.050\text{ m}$.
* **Cụm cảm biến dò line:** 5 cảm biến hồng ngoại **TCRT5000** đặt dọc theo line, khoảng cách giữa hai mắt liên tiếp $L_{\text{cb}} = 12\text{ mm}$, lắp ở độ cao $h = 9\text{ mm}$. Đọc giá trị Analog qua 5 kênh ADC1 kết hợp DMA.
* **Cảm biến vật cản:** Cảm biến quang **E18-D80NK** (NPN thường hở, tích cực mức thấp `LOW`), tinh chỉnh biến trở phát hiện vật cản ở khoảng cách $30\text{ cm}$ ($0.30\text{ m}$).
* **Động cơ & Encoder:** 2 động cơ DC Servo **JGB37-520** ($12\text{ V}$, tốc độ định mức $250\text{--}333\text{ RPM}$, tỉ số truyền hộp số $1:30$). Encoder 2 kênh A-B có độ phân giải $11\text{ xung/vòng}$ ở trục động cơ; khi đọc ở chế độ *Encoder Mode TI1 and TI2* (nhân 4 sườn xung), số xung trên 1 vòng quay bánh xe là:
  $$N_{\text{CPR}} = 11 \times 4 \times 30 = 1320\text{ xung/vòng}$$
* **Mạch công suất & Cách ly:** Driver cầu H đôi **TB6612FNG** ($V_M = 12\text{ V}$, $V_{CC} = 5\text{ V}$, chân `STBY` nối lên $5\text{ V}$). Toàn bộ 6 đường tín hiệu điều khiển (`PWMA`, `AI1`, `AI2`, `PWMB`, `BI1`, `BI2`) từ STM32 ($3.3\text{ V}$) sang TB6612FNG được cách ly quang qua 6 IC **Opto PC817** với tần số băm xung PWM là $1\text{ kHz}$.

---

## 2. Cấu trúc Thư mục & Phân chia Module Phần mềm

Mã nguồn được thiết kế theo kiến trúc hướng module (Modular Programming), không sử dụng biến trôi nổi, không dùng `HAL_Delay()` trong vòng lặp điều khiển:

| Tệp tin | Nhiệm vụ chính |
| :--- | :--- |
| `Core/Inc/robot_config.h` | Tệp cấu hình trung tâm: chứa toàn bộ định nghĩa chân GPIO, hằng số hình học ($r, b, d, L_{\text{cb}}$), thông số hiệu chuẩn ADC, hệ số hàm tuyến tính hóa, hệ số bộ điều khiển Lyapunov và PI động cơ. |
| `Core/Inc/line_sensor.h` `Core/Src/line_sensor.c` | Quản lý đọc 5 kênh ADC qua DMA, đảo thứ tự mắt cảm biến theo PCB, tính công thức hiệu chuẩn từng mắt $y_{i,C}$, tính trung bình trọng số $x$, tuyến tính hóa ra sai số lệch ngang $e_2$, phát hiện ngã rẽ và lọc chống nhiễu cảm biến E18-D80NK. |
| `Core/Inc/control.h` `Core/Src/control.c` | Chứa thuật toán nội suy 3 sai số $[e_1, e_2, e_3]^T$, bộ điều khiển phi tuyến **Lyapunov** (vòng ngoài $50\text{ ms}$) và bộ điều khiển **PI rời rạc** có chống bão hòa tích phân Anti-windup (vòng trong $10\text{ ms}$). |
| `Core/Inc/drive.h` `Core/Src/drive.c` | Đọc xung từ 2 bộ Timer Encoder (`TIM3`, `TIM1`), xử lý tràn bộ đếm 16-bit, tính vận tốc góc thực tế ($\text{rad/s}$ và $\text{vòng/s}$), tính Odometry (quãng đường và góc quay thân xe), giải bài toán động học ngược và xuất xung PWM/Phanh cứng ra TB6612FNG. |
| `Core/Inc/robot_app.h` `Core/Src/robot_app.c` | Bộ lập lịch thời gian thực ($10\text{ ms}$ và $50\text{ ms}$) và Máy trạng thái hữu hạn (Finite State Machine) điều phối toàn bộ quy trình: Bám line $\to$ Né vật cản $\to$ Bắt ngã rẽ $\to$ Về đích. |
| `Core/Src/main.c` | Khởi tạo các ngoại vi phần cứng do CubeMX sinh ra và gọi hàm `RobotApp_Tick10ms_ISR()` bên trong ngắt tràn `TIM2` mỗi $10\text{ ms}$. |

---

## 3. Bảng Ánh xạ Chân (Pinout) & Cấu hình STM32CubeMX

### 3.1. Bảng tổng hợp chân vi điều khiển theo Schematic PCB

| Chân STM32 | Tên Net trên Schematic | Cấu hình CubeMX | Phần cứng kết nối | Vai trò trong hệ thống |
| :--- | :--- | :--- | :--- | :--- |
| `PA0` | `Line_sensor1` | `ADC1_IN0` | TCRT5000 mắt S1 | Cảm biến phải ngoài (trọng số $i = +2$) |
| `PA1` | `Line_sensor2` | `ADC1_IN1` | TCRT5000 mắt S2 | Cảm biến phải trong (trọng số $i = +1$) |
| `PA2` | `Line_sensor3` | `ADC1_IN2` | TCRT5000 mắt S3 | Cảm biến trung tâm (trọng số $i = 0$) |
| `PA3` | `Line_sensor4` | `ADC1_IN3` | TCRT5000 mắt S4 | Cảm biến trái trong (trọng số $i = -1$) |
| `PA4` | `Line_sensor5` | `ADC1_IN4` | TCRT5000 mắt S5 | Cảm biến trái ngoài (trọng số $i = -2$) |
| `PB15` | `Sensor_obstacle` | `GPIO_Input` (Pull-up) | E18-D80NK | Phát hiện vật cản ở cự ly $30\text{ cm}$ (`LOW` = có vật cản) |
| `PA6` | `MOTOR1_ENCODER_A` | `TIM3_CH1` | Encoder Trái A | Đếm xung tốc độ động cơ bánh trái |
| `PA7` | `MOTOR1_ENCODER_B` | `TIM3_CH2` | Encoder Trái B | Đếm xung tốc độ động cơ bánh trái |
| `PA8` | `MOTOR2_ENCODER_B` | `TIM1_CH1` | Encoder Phải B | Đếm xung tốc độ động cơ bánh phải |
| `PA9` | `MOTOR2_ENCODER_A` | `TIM1_CH2` | Encoder Phải A | Đếm xung tốc độ động cơ bánh phải |
| `PB4` | `MOTOR1_DIR1` | `GPIO_Output` | Opto OK2 (`AI1`) | Điều khiển chiều quay bánh trái 1 |
| `PB3` | `MOTOR1_DIR2` | `GPIO_Output` | Opto OK3 (`AI2`) | Điều khiển chiều quay bánh trái 2 |
| `PB6` | `MOTOR1_PWMA` | `TIM4_CH1` | Opto OK1 (`PWMA`) | Xuất xung PWM $1\text{ kHz}$ cho bánh trái |
| `PB8` | `MOTOR2_DIR1` | `GPIO_Output` | Opto OK5 (`BI1`) | Điều khiển chiều quay bánh phải 1 |
| `PB7` | `MOTOR2_DIR2` | `GPIO_Output` | Opto OK6 (`BI2`) | Điều khiển chiều quay bánh phải 2 |
| `PB9` | `MOTOR2_PWMB` | `TIM4_CH4` | Opto OK4 (`PWMB`) | Xuất xung PWM $1\text{ kHz}$ cho bánh phải |
| `PA13` | `SWDIO` | `SYS_JTMS-SWDIO` | Header nạp SWD | Chân dữ liệu nạp và gỡ lỗi ST-Link |
| `PA14` | `SWCLK` | `SYS_JTCK-SWCLK` | Header nạp SWD | Chân xung nhịp nạp và gỡ lỗi ST-Link |

### 3.2. Tóm tắt thông số cấu hình trên STM32CubeMX
1. **SYS & RCC:** Chọn `Debug: Serial Wire` (bắt buộc để giải phóng `PB3`, `PB4` khỏi JTAG) và `HSE: Crystal/Ceramic Resonator`. Thiết lập `HCLK = 72 MHz`, `ADC Prescaler = /6` ($12\text{ MHz}$).
2. **ADC1 & DMA:** Bật `IN0` $\to$ `IN4`, `Scan Conversion Mode: Enabled`, `Continuous Conversion Mode: Enabled`, `Sampling Time: 239.5 Cycles` cho cả 5 Rank. Thêm DMA `ADC1` ở chế độ `Circular`, Data Width `Half Word`.
3. **TIM3 & TIM1 (Encoder):** Chọn `Encoder Mode TI1 and TI2`, `Prescaler = 0`, `Counter Period (ARR) = 65535`, `Input Filter = 6`.
4. **TIM4 (PWM $1\text{ kHz}$):** Chọn `Internal Clock`, `Channel 1: PWM Generation CH1` (`PB6`), `Channel 4: PWM Generation CH4` (`PB9`), `Prescaler = 71`, `Counter Period (ARR) = 999`.
5. **TIM2 (Định thời lấy mẫu $10\text{ ms}$):** Chọn `Internal Clock`, `Prescaler = 71`, `Counter Period (ARR) = 9999`, `auto-reload preload = Enable`.
6. **NVIC (Cấu hình ngắt):**
   * **Bật** `TIM2 global interrupt` (Preemption Priority = 1).
   * **Tắt (Bỏ tích)** `DMA1 channel1 global interrupt` để tránh hiện tượng bão ngắt DMA làm treo vi điều khiển.

---

## 4. Cơ sở Toán học & Cấu trúc Điều khiển Phân tầng (Cascade Control)

Hệ thống điều khiển gồm 2 vòng lặp lồng nhau hoạt động ở hai tần số lấy mẫu khác nhau:

### 4.1. Xử lý tín hiệu cảm biến & Tuyến tính hóa (Cập nhật mỗi $10\text{ ms}$)
* **Bước 1 – Đảo thứ tự vật lý:** Trên PCB dò line, mắt S1 (`PA0`) nằm ngoài cùng bên phải và mắt S5 (`PA4`) nằm ngoài cùng bên trái. Phần mềm tự động đảo mảng ADC để chỉ số mảng $k \in \{0, 1, 2, 3, 4\}$ tương ứng với trọng số vị trí $i \in \{-2, -1, 0, +1, +2\}$ từ Trái sang Phải.
* **Bước 2 – Hiệu chuẩn từng mắt cảm biến:** Đưa cả 5 mắt về cùng một thang đo chuẩn $[y_{\text{min}C}, y_{\text{max}C}] = [0, 1000]$ theo công thức:
  $$y_{i,C} = y_{\text{min}C} + \frac{y_{\text{max}C} - y_{\text{min}C}}{x_{i,\text{max}} - x_{i,\text{min}}} \cdot (x_i - x_{i,\text{min}})$$
  Trong đó $x_i$ là giá trị ADC đọc về, $x_{i,\text{min}}$ là giá trị ADC trên nền trắng và $x_{i,\text{max}}$ là giá trị ADC trên vạch đen của mắt thứ $i$.
* **Bước 3 – Tính vị trí trung bình trọng số ($x$):** Với khoảng cách giữa 2 cảm biến $L_{\text{cb}} = 12\text{ mm}$:
  $$x = L_{\text{cb}} \cdot \frac{\sum_{i=-2}^{2} i \cdot y_{i,C}}{\sum_{i=-2}^{2} y_{i,C}} \quad (\text{đơn vị: mm})$$
* **Bước 4 – Tuyến tính hóa bậc nhất:** Quy đổi từ giá trị trung bình trọng số sang độ lệch ngang thực tế $e_2$ (với quy ước $e_2 > 0$ khi vạch line nằm về bên trái tâm xe):
  $$e_2\text{ (mm)} = a \cdot (-x) + b \implies e_2\text{ (m)} = e_2\text{ (mm)} \times 10^{-3}$$

### 4.2. Vòng điều khiển ngoài: Bộ điều khiển phi tuyến Lyapunov (Chu kỳ $T_{\text{đk}} = 50\text{ ms}$)
* **Nội suy 3 thành phần sai số $[e_1, e_2, e_3]^T$:**
  Do cảm biến đặt trước trục hai bánh chủ động một khoảng $d = 50\text{ mm} = 0.050\text{ m}$, từ độ lệch ngang $e_2$ đo được, phần mềm nội suy góc lệch hướng $e_3$ và độ lệch dọc trục $e_1$:
  $$e_3 = \arctan2(e_2, d), \qquad e_1 = d \cdot (1 - \cos e_3)$$
* **Luật điều khiển Lyapunov:** Tính toán vận tốc dài $v$ ($\text{m/s}$) và vận tốc góc thân xe $\omega$ ($\text{rad/s}$) với vận tốc tham chiếu $v_R = 0.30\text{ m/s}$, $\omega_R = 0\text{ rad/s}$:
  $$v = v_R \cos(e_3) + k_1 e_1$$
  $$\omega = \omega_R + k_2 v_R e_2 + k_3 \sin(e_3)$$

### 4.3. Động học ngược & Vòng điều khiển trong: Bộ điều khiển PI tốc độ (Chu kỳ $T_s = 10\text{ ms}$)
* **Động học ngược (Inverse Kinematics):** Quy đổi từ $(v, \omega)$ của thân xe sang vận tốc góc đặt ($\text{rad/s}$) cho bánh Trái và bánh Phải:
  $$\omega_{\text{ref},L} = \frac{v - 0.5 \cdot b \cdot \omega}{r}, \qquad \omega_{\text{ref},R} = \frac{v + 0.5 \cdot b \cdot \omega}{r}$$
* **Đo tốc độ thực từ Encoder:** Từ số xung chênh lệch $\Delta N$ đếm được trong $T_s = 0.01\text{ s}$:
  $$\text{RPS}_{\text{meas}} = \frac{\Delta N}{1320 \times 0.01}\text{ (vòng/s)}, \qquad \omega_{\text{meas}} = 2\pi \cdot \text{RPS}_{\text{meas}}\text{ (rad/s)}$$
* **Luật điều khiển PI rời rạc:** Tính điện áp điều khiển $U\text{ (V)}$ và quy đổi ra giá trị thanh ghi PWM $\text{CCR} \in [0, 999]$:
  $$e_\omega(k) = \omega_{\text{ref}}(k) - \omega_{\text{meas}}(k)$$
  $$I(k) = I(k-1) + e_\omega(k) \cdot T_s$$
  $$U(k) = K_P \cdot e_\omega(k) + K_I \cdot I(k) \implies \text{CCR} = \frac{|U(k)|}{12.0} \times 999$$

---

## 5. Máy Trạng thái (State Machine) Vận hành Toàn trình

Toàn bộ chu trình hoạt động của xe được quản lý trong tệp `robot_app.c` qua 11 trạng thái tuần tự:

1. `SM_FOLLOW_LINE_BEFORE_OBS`: Bám line bằng bộ điều khiển Lyapunov ở tốc độ $0.30\text{ m/s}$. Liên tục kiểm tra cảm biến E18-D80NK mỗi $10\text{ ms}$.
2. `SM_AVOID_STOP_INIT`: Ngay khi E18-D80NK phát hiện vật cản ở khoảng cách $30\text{ cm}$, xe phanh cứng trong $200\text{ ms}$ để triệt tiêu quán tính và xóa bộ đếm Odometry.
3. `SM_AVOID_TURN_LEFT_45`: Điều khiển 2 bánh quay ngược chiều nhau để thân xe xoay sang **Trái** một góc cố định $45^\circ$ (tích lũy góc quay từ Encoder).
4. `SM_AVOID_GO_STRAIGHT_30CM`: Điều khiển xe đi thẳng một quãng đường cố định $30\text{ cm}$ ($0.30\text{ m}$) tính từ Encoder để tiến lên ngang với vật cản.
5. `SM_AVOID_TURN_RIGHT_90`: Điều khiển xe xoay sang **Phải** một góc cố định $90^\circ$ (hướng mũi xe cắt xiên về phía đường line).
6. `SM_AVOID_STRAIGHT_FIND_LINE`: Cho xe đi thẳng về phía vạch đen, đồng thời kích hoạt lại việc đọc 5 mắt cảm biến dò line. Ngay khi cảm biến bắt được vạch đen, lập tức chuyển trạng thái.
7. `SM_AVOID_ALIGN_ON_LINE`: Bám line ở tốc độ giảm ($0.20\text{ m/s}$) trong đoạn ngắn $8\text{ cm}$ để ép thẳng đầu xe vào tâm vạch đen mà không bị vọt lố.
8. `SM_FOLLOW_LINE_TO_FORK`: Bám line ở tốc độ chuẩn $0.30\text{ m/s}$ qua khúc cua $R500$. Sau khi đi qua quãng đường an toàn tối thiểu ($0.45\text{ m}$), bắt đầu chờ nhận diện mẫu ngã rẽ ($\ge 4$ mắt cùng đen hoặc vạch tách đôi).
9. `SM_SELECT_BRANCH_TURN`: Tại ngã rẽ, tùy theo giá trị biến `robot_branch_select` (`1`: Rẽ nhánh Trái, `0`: Rẽ nhánh Phải), phần mềm chỉ lấy trọng số nửa dãy cảm biến tương ứng để dẫn xe tách hẳn vào nhánh đã chọn.
10. `SM_FOLLOW_LINE_TO_END`: Bám line trên nhánh đã chọn về đích. Khi cả 5 cảm biến ra khỏi vạch đen (hết line) sau khi đã đi đủ quãng đường tối thiểu, lập tức kích hoạt phanh cứng ngay ở nhịp $10\text{ ms}$ đầu tiên.
11. `SM_STOP_FINISHED`: Giữ phanh ngắn mạch (`AI1 = AI2 = 1`, `BI1 = BI2 = 1`) để xe đứng yên tại vạch đích với sai số dừng $\le \pm 5\text{ mm}$.

---

## 6. Hướng dẫn Kiểm tra Phần cứng & Hiệu chuẩn (Calibration) cho Thành viên

### Bước 1: Hai kiểm tra phần cứng bắt buộc trước khi cấp nguồn $12\text{ V}$
1. **Đo thông mạch ngõ ra động cơ 1 (`J4`):** Trên bản vẽ schematic cũ, hai chân `AO1` và `AO2` của TB6612FNG bị đặt trùng tên net `MOTOR1_A01`. Dùng đồng hồ VOM đo hai chân domino động cơ 1 xem có bị chạm nhau không trước khi cấp nguồn $12\text{ V}$.
2. **Kiểm tra điểm nối mass của cảm biến E18-D80NK (`PB15`):** Cảm biến E18-D80NK dùng nguồn $5\text{ V}$ (mass công suất `GND`), trong khi chân `PB15` thuộc miền mass vi điều khiển (`GND_MCU`). Đảm bảo tín hiệu từ E18-D80NK đã đi qua Opto cách ly hoặc hai miền mass đã có điểm tham chiếu chung hợp lệ.

### Bước 2: Đo và nhập thông số Hiệu chuẩn ADC cảm biến Line
1. Mở STM32CubeIDE, nạp chương trình và chạy chế độ **Debug**.
2. Mở tab **Live Expressions**, thêm biểu thức `s_sensor.raw_adc` để xem 5 giá trị ADC thô (theo thứ tự từ Trái `index 0` sang Phải `index 4`).
3. Đặt mảng cảm biến lên **nền trắng** ở đúng độ cao $9\text{ mm}$, ghi lại 5 giá trị vào macro `SENSOR_ADC_X_MIN_INIT` trong file `robot_config.h`.
4. Đặt lần lượt từng mắt cảm biến vào **tâm vạch đen**, ghi lại 5 giá trị vào macro `SENSOR_ADC_X_MAX_INIT` trong file `robot_config.h`.
5. Điền 2 hệ số hàm bậc nhất từ kết quả khảo sát tuyến tính hóa $-24\text{ mm} \to +24\text{ mm}$ của nhóm vào `ROBOT_LINE_LIN_A` và `ROBOT_LINE_LIN_B`.

### Bước 3: Kiểm tra chiều Encoder và chọn nhánh Ngã rẽ
* **Kiểm tra chiều Encoder:** Trong **Live Expressions**, thêm biến `s_drive.omega_meas_L_rads` và `s_drive.omega_meas_R_rads`. Dùng tay đẩy nhẹ xe tiến về phía trước: cả 2 biến phải cùng mang **giá trị dương ($> 0$)**. Nếu bánh nào mang giá trị âm, đổi `ENC_LEFT_DIR_SIGN` hoặc `ENC_RIGHT_DIR_SIGN` từ `1` thành `-1` trong `robot_config.h`.
* **Đổi nhánh rẽ Trái / Phải:** Có thể sửa trực tiếp `#define ROBOT_DEFAULT_BRANCH_LEFT` (`1` hoặc `0`) trong `robot_config.h`, hoặc gõ trực tiếp giá trị `1` (Trái) / `0` (Phải) vào biến `robot_branch_select` trong cửa sổ **Live Expressions** trước khi xe chạy tới ngã ba mà không cần biên dịch lại code.