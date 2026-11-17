/*
 * control.h
 *
 *  Created on: Nov 17, 2026
 *      Author: ASUS
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include <robot_config.h>

/* Cấu trúc lưu 3 thành phần sai số theo Phương trình (5.10) */
typedef struct {
    float e1_m;         // Sai số dọc theo hướng di chuyển (vuông góc trục hai bánh)
    float e2_m;         // Sai số lệch ngang (song song trục hai bánh) đọc từ cảm biến
    float e3_rad;       // Sai số góc hướng xe so với tiếp tuyến line (nội suy từ d = 50mm)
} Tracking_Error_t;

/* Cấu trúc bộ điều khiển PI tốc độ động cơ */
typedef struct {
    float kp;
    float ki;
    float integral;
    float out_min;
    float out_max;
} PI_Motor_t;

void Control_Init_PI(PI_Motor_t *pi, float kp, float ki, float out_min, float out_max);
void Control_Reset_PI(PI_Motor_t *pi);
float Control_Step_PI(PI_Motor_t *pi, float setpoint, float measurement, float dt_s);

/* Nội suy 3 sai số (e1, e2, e3) từ sai số cảm biến e2 và thông số lắp đặt d = 50mm */
void Control_Interpolate_Errors(float e2_measured_m, float lookahead_d_m, Tracking_Error_t *err_out);

/* Bộ điều khiển vòng ngoài Lyapunov (Chu kỳ 50ms) -> Xuất v_cmd (m/s) và w_cmd (rad/s) */
void Control_Lyapunov_Step50ms(const Tracking_Error_t *err, float v_ref, float w_ref,
                               float *v_cmd_out, float *w_cmd_out);

#endif /* INC_CONTROL_H_ */
