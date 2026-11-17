/*
 * config.h
 *
 *  Created on: Nov 17, 2026
 *      Author: ASUS
 */

#ifndef INC_ROBOT_CONFIG_H_
#define INC_ROBOT_CONFIG_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ================= 1. CẤU HÌNH PHẦN CỨNG & CHÂN VI ĐIỀU KHIỂN ================= */
// Cảm biến vật cản E18-D80NK (PB15, LOW = có vật cản ở khoảng cách <= 30cm)
#define OBS_GPIO_PORT               GPIOB
#define OBS_GPIO_PIN                GPIO_PIN_15
#define OBS_ACTIVE_LOW              1

// Động cơ 1 - Bánh Trái (PB4: AI1, PB3: AI2, PB6: TIM4_CH1 PWMA)
#define M1_DIR1_PORT                GPIOB
#define M1_DIR1_PIN                 GPIO_PIN_4
#define M1_DIR2_PORT                GPIOB
#define M1_DIR2_PIN                 GPIO_PIN_3
#define M1_TIM_CHANNEL              TIM_CHANNEL_1

// Động cơ 2 - Bánh Phải (PB8: BI1, PB7: BI2, PB9: TIM4_CH4 PWMB)
#define M2_DIR1_PORT                GPIOB
#define M2_DIR1_PIN                 GPIO_PIN_8
#define M2_DIR2_PORT                GPIOB
#define M2_DIR2_PIN                 GPIO_PIN_7
#define M2_TIM_CHANNEL              TIM_CHANNEL_4

// Đảo chiều phần mềm độc lập cho Động cơ và Encoder (1 hoặc -1)
#define MOTOR_LEFT_DIR_SIGN         1
#define MOTOR_RIGHT_DIR_SIGN        1
#define ENC_LEFT_DIR_SIGN           1
#define ENC_RIGHT_DIR_SIGN          1

/* ================= 2. THÔNG SỐ HÌNH HỌC & ENCODER ================= */
#define ROBOT_PI                    3.14159265359f
#define ROBOT_WHEEL_RADIUS_M        0.040f      // Bán kính bánh xe r = 40 mm
#define ROBOT_WHEEL_BASE_M          0.210f      // Khoảng cách 2 bánh chủ động b = 210 mm
#define ROBOT_SENSOR_LOOKAHEAD_D_M  0.050f      // Khoảng cách tâm cảm biến -> trục bánh d = 50 mm
#define ROBOT_SENSOR_PITCH_MM       12.0f       // Khoảng cách 2 cảm biến liên tiếp L_cb = 12 mm

// Encoder: 11 xung/vòng * 4 (kênh A-B chế độ TI12) * tỉ số truyền 30 = 1320 xung/vòng
#define ROBOT_ENCODER_CPR           1320.0f

// Chu kỳ lấy mẫu của 2 vòng điều khiển
#define ROBOT_TS_MOTOR_S            0.01f       // Chu kỳ lấy mẫu PI động cơ: 10 ms (0.01s)
#define ROBOT_TS_LINE_S             0.05f       // Chu kỳ lấy mẫu Lyapunov bám line: 50 ms (0.05s)

// Thông số PWM (TIM4: PSC = 71, ARR = 999 -> Tần số 1 kHz qua Opto PC817)
#define ROBOT_PWM_ARR               999.0f
#define ROBOT_SUPPLY_VOLTAGE        12.0f

/* ================= 3. THÔNG SỐ HIỆU CHUẨN & TUYẾN TÍNH HÓA CẢM BIẾN LINE ================= */
/* Thứ tự PCB: PA0 (S1 - Phải ngoài) -> PA4 (S5 - Trái ngoài).
 * Đặt = 1 để đảo mảng về thứ tự logic Trái (-2) -> Phải (+2) */
#define ROBOT_LINE_ADC_REVERSED     1

// Giá trị y_minC và y_maxC mong muốn trong công thức hiệu chuẩn (Khuyến nghị: 0.0 -> 1000.0)
#define SENSOR_Y_MIN_C              0.0f
#define SENSOR_Y_MAX_C              1000.0f

/* Bảng giá trị ADC nhỏ nhất (nền trắng) và lớn nhất (vạch đen) của 5 cảm biến (CHỪA ĐỂ CALIB SAU)
 * Thứ tự sau khi remap: [0]: i=-2 (Trái ngoài), [1]: i=-1, [2]: i=0 (Giữa), [3]: i=+1, [4]: i=+2 (Phải ngoài) */
#define SENSOR_ADC_X_MIN_INIT       { 400.0f,  400.0f,  400.0f,  400.0f,  400.0f }
#define SENSOR_ADC_X_MAX_INIT       { 3500.0f, 3500.0f, 3500.0f, 3500.0f, 3500.0f }

/* Hệ số hàm bậc nhất tuyến tính hóa khoảng cách thực (mm): e2_mm = LIN_A * x_weighted_mm + LIN_B
 * (Nhóm điền hệ số hồi quy thực nghiệm từ đo đạc -24mm -> +24mm vào đây) */
#define ROBOT_LINE_LIN_A            1.000f
#define ROBOT_LINE_LIN_B            0.000f

// Ngưỡng tổng trọng số y_i,C tối thiểu để xác nhận có vạch đen dưới cảm biến
#define SENSOR_LINE_DETECT_SUM_MIN  250.0f
// Ngưỡng giá trị y_i,C của 1 mắt để tính là đang đè lên vạch đen (phục vụ bắt ngã rẽ)
#define SENSOR_SINGLE_BLACK_THRESH  550.0f

/* ================= 4. THÔNG SỐ VẬN TỐC & BỘ ĐIỀU KHIỂN LYAPUNOV (VÒNG NGOÀI 50ms) ================= */
#define ROBOT_V_BASE_MPS            0.30f       // Vận tốc cơ bản 0.3 m/s
#define ROBOT_V_MAX_MPS             0.50f       // Vận tốc tối đa 0.5 m/s
#define ROBOT_V_AVOID_MPS           0.20f       // Vận tốc khi đi thẳng né vật cản
#define ROBOT_W_MAX_RADS            4.50f       // Giới hạn vận tốc góc thân xe (rad/s)

/* Hệ số bộ điều khiển Lyapunov:
 * v = v_R * cos(e3) + K1 * e1
 * w = w_R + K2 * v_R * e2 + K3 * sin(e3) */
#define LYAPUNOV_K1                 1.50f
#define LYAPUNOV_K2                 85.0f       // Tác động lên sai số ngang e2 (m)
#define LYAPUNOV_K3                 4.20f       // Tác động lên sai số góc e3 (rad)

/* ================= 5. THÔNG SỐ BỘ ĐIỀU KHIỂN PI ĐỘNG CƠ (VÒNG TRONG 10ms) ================= */
/* Đầu vào: Sai số tốc độ góc (rad/s) [hoặc vòng/s] -> Đầu ra: Điện áp (V) quy đổi ra PWM (0..999)
 * (Giữ thông số từ Bảng 8.1, nhóm có thể tinh chỉnh lại thực nghiệm) */
#define PI_MOTOR_LEFT_KP            0.2330f
#define PI_MOTOR_LEFT_KI            2.6445f

#define PI_MOTOR_RIGHT_KP           0.2113f
#define PI_MOTOR_RIGHT_KI           2.8597f

/* ================= 6. THÔNG SỐ NÉ VẬT CẢN & NGÃ RẼ ================= */
// Khi E18-D80NK phát hiện vật cản ở cự ly 30cm:
#define ROBOT_AVOID_TURN_LEFT_DEG   45.0f       // Xoay trái cố định 45 độ
#define ROBOT_AVOID_STRAIGHT1_M     0.30f       // Đi thẳng 30cm (hoặc đặt 0.424f nếu tính theo 30cm/cos45)
#define ROBOT_AVOID_TURN_RIGHT_DEG  90.0f       // Xoay phải cố định 90 độ rồi đi thẳng tìm lại line
#define ROBOT_AVOID_FIND_MAX_M      0.90f       // Quãng đường tối đa đi thẳng tìm lại line trước khi báo lỗi

// Lựa chọn nhánh tại ngã rẽ (1: Rẽ Trái / Nhánh trên, 0: Rẽ Phải / Nhánh dưới)
#define ROBOT_DEFAULT_BRANCH_LEFT   1
#define ROBOT_MIN_DIST_TO_FORK_M    0.45f       // Quãng đường tối thiểu sau khi né vật cản mới bắt ngã rẽ

#endif /* INC_ROBOT_CONFIG_H_ */
