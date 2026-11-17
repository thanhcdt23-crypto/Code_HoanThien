/*
 * control.c
 *
 *  Created on: Nov 17, 2026
 *      Author: ASUS
 */

#include "control.h"

static inline float Clamp_Control(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void Control_Init_PI(PI_Motor_t *pi, float kp, float ki, float out_min, float out_max) {
    pi->kp       = kp;
    pi->ki       = ki;
    pi->integral = 0.0f;
    pi->out_min  = out_min;
    pi->out_max  = out_max;
}

void Control_Reset_PI(PI_Motor_t *pi) {
    pi->integral = 0.0f;
}

/* Thực thi bộ điều khiển PI vận tốc động cơ (Chu kỳ 10ms) */
float Control_Step_PI(PI_Motor_t *pi, float setpoint, float measurement, float dt_s) {
    float err = setpoint - measurement;
    float p_term = pi->kp * err;

    // Tích phân có chống bão hòa (Anti-windup Clamping)
    float new_int = pi->integral + (err * dt_s);
    float u_unclamped = p_term + (pi->ki * new_int);

    if (u_unclamped > pi->out_max) {
        if (err < 0.0f) pi->integral = new_int;
        return pi->out_max;
    } else if (u_unclamped < pi->out_min) {
        if (err > 0.0f) pi->integral = new_int;
        return pi->out_min;
    } else {
        pi->integral = new_int;
        return u_unclamped;
    }
}

/* Nội suy bộ 3 sai số [e1, e2, e3]^T từ e2 đọc trên mảng cảm biến và khoảng cách lắp đặt d = 50mm */
void Control_Interpolate_Errors(float e2_measured_m, float lookahead_d_m, Tracking_Error_t *err_out) {
    err_out->e2_m = e2_measured_m;

    // Nội suy góc lệch hướng e3 từ vị trí tâm cảm biến đặt cách trục bánh xe d = 50mm
    if (lookahead_d_m < 0.005f) lookahead_d_m = 0.050f;
    err_out->e3_rad = atan2f(e2_measured_m, lookahead_d_m);

    // Nội suy sai số dọc trục e1 từ hình chiếu khoảng cách d khi xe lệch góc e3
    err_out->e1_m = lookahead_d_m * (1.0f - cosf(err_out->e3_rad));
}

/* Luật điều khiển phi tuyến Lyapunov (Chu kỳ 50ms):
 * v = v_R * cos(e3) + K1 * e1
 * w = w_R + K2 * v_R * e2 + K3 * sin(e3) */
void Control_Lyapunov_Step50ms(const Tracking_Error_t *err, float v_ref, float w_ref,
                               float *v_cmd_out, float *w_cmd_out) {
    float v_c = v_ref * cosf(err->e3_rad) + (LYAPUNOV_K1 * err->e1_m);
    float w_c = w_ref + (LYAPUNOV_K2 * v_ref * err->e2_m) + (LYAPUNOV_K3 * sinf(err->e3_rad));

    // Giới hạn tốc độ dài trong khoảng [0.1 m/s -> 0.5 m/s] và tốc độ góc tối đa
    *v_cmd_out = Clamp_Control(v_c, 0.0f, ROBOT_V_MAX_MPS);
    *w_cmd_out = Clamp_Control(w_c, -ROBOT_W_MAX_RADS, ROBOT_W_MAX_RADS);
}
