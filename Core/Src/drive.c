/*
 * drive.c
 *
 *  Created on: Nov 17, 2026
 *      Author: ASUS
 */

#include "drive.h"

static TIM_HandleTypeDef *s_htim_enc_L = NULL; // TIM3 (PA6, PA7)
static TIM_HandleTypeDef *s_htim_enc_R = NULL; // TIM1 (PA8, PA9)
static TIM_HandleTypeDef *s_htim_pwm   = NULL; // TIM4 (PB6: CH1, PB9: CH4)

static uint16_t s_prev_cnt_L = 0;
static uint16_t s_prev_cnt_R = 0;

static PI_Motor_t s_pi_L;
static PI_Motor_t s_pi_R;

static void Apply_Motor_PWM(GPIO_TypeDef *dir1_port, uint16_t dir1_pin,
                            GPIO_TypeDef *dir2_port, uint16_t dir2_pin,
                            uint32_t pwm_channel, int16_t signed_ccr) {
    if (signed_ccr >= 0) {
        HAL_GPIO_WritePin(dir1_port, dir1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(dir2_port, dir2_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(dir1_port, dir1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(dir2_port, dir2_pin, GPIO_PIN_SET);
        signed_ccr = -signed_ccr;
    }
    if (signed_ccr > (int16_t)ROBOT_PWM_ARR) signed_ccr = (int16_t)ROBOT_PWM_ARR;
    __HAL_TIM_SET_COMPARE(s_htim_pwm, pwm_channel, (uint16_t)signed_ccr);
}

void Drive_Init(TIM_HandleTypeDef *htim_enc_L, TIM_HandleTypeDef *htim_enc_R,
                TIM_HandleTypeDef *htim_pwm, Drive_State_t *drv) {
    s_htim_enc_L = htim_enc_L;
    s_htim_enc_R = htim_enc_R;
    s_htim_pwm   = htim_pwm;

    // Giải phóng PB3, PB4 khỏi JTAG để điều khiển chiều quay Motor 1
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    Control_Init_PI(&s_pi_L, PI_MOTOR_LEFT_KP,  PI_MOTOR_LEFT_KI,  -ROBOT_SUPPLY_VOLTAGE, ROBOT_SUPPLY_VOLTAGE);
    Control_Init_PI(&s_pi_R, PI_MOTOR_RIGHT_KP, PI_MOTOR_RIGHT_KI, -ROBOT_SUPPLY_VOLTAGE, ROBOT_SUPPLY_VOLTAGE);

    HAL_TIM_Encoder_Start(s_htim_enc_L, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(s_htim_enc_R, TIM_CHANNEL_ALL);
    s_prev_cnt_L = (uint16_t)__HAL_TIM_GET_COUNTER(s_htim_enc_L);
    s_prev_cnt_R = (uint16_t)__HAL_TIM_GET_COUNTER(s_htim_enc_R);

    HAL_TIM_PWM_Start(s_htim_pwm, M1_TIM_CHANNEL);
    HAL_TIM_PWM_Start(s_htim_pwm, M2_TIM_CHANNEL);

    drv->omega_ref_L_rads  = 0.0f;
    drv->omega_ref_R_rads  = 0.0f;
    drv->omega_meas_L_rads = 0.0f;
    drv->omega_meas_R_rads = 0.0f;
    drv->rps_meas_L        = 0.0f;
    drv->rps_meas_R        = 0.0f;
    drv->pwm_ccr_L         = 0;
    drv->pwm_ccr_R         = 0;
    drv->step_distance_m   = 0.0f;
    drv->step_angle_deg    = 0.0f;
    drv->brake_active      = true;
    Drive_HardBrake(drv);
}

void Drive_ResetOdometry(Drive_State_t *drv) {
    drv->step_distance_m = 0.0f;
    drv->step_angle_deg  = 0.0f;
}

/* Động học ngược: Đổi từ (v, w) của xe sang vận tốc góc đặt 2 bánh (rad/s) */
void Drive_SetChassisTwist(Drive_State_t *drv, float v_mps, float w_rads) {
    drv->brake_active = false;
    float v_L = v_mps - (0.5f * ROBOT_WHEEL_BASE_M * w_rads);
    float v_R = v_mps + (0.5f * ROBOT_WHEEL_BASE_M * w_rads);

    drv->omega_ref_L_rads = v_L / ROBOT_WHEEL_RADIUS_M;
    drv->omega_ref_R_rads = v_R / ROBOT_WHEEL_RADIUS_M;
}

/* Phanh ngắn mạch chủ động trên TB6612FNG (AI1=1, AI2=1, BI1=1, BI2=1) */
void Drive_HardBrake(Drive_State_t *drv) {
    drv->brake_active     = true;
    drv->omega_ref_L_rads = 0.0f;
    drv->omega_ref_R_rads = 0.0f;
    Control_Reset_PI(&s_pi_L);
    Control_Reset_PI(&s_pi_R);

    HAL_GPIO_WritePin(M1_DIR1_PORT, M1_DIR1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_DIR2_PORT, M1_DIR2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_DIR1_PORT, M2_DIR1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_DIR2_PORT, M2_DIR2_PIN, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(s_htim_pwm, M1_TIM_CHANNEL, (uint16_t)ROBOT_PWM_ARR);
    __HAL_TIM_SET_COMPARE(s_htim_pwm, M2_TIM_CHANNEL, (uint16_t)ROBOT_PWM_ARR);
}

/* Vòng lặp 10ms: Đọc Encoder (1320 xung/vòng), tính Odometry và thực thi PI tốc độ */
void Drive_Update10ms(Drive_State_t *drv) {
    uint16_t curr_L = (uint16_t)__HAL_TIM_GET_COUNTER(s_htim_enc_L);
    uint16_t curr_R = (uint16_t)__HAL_TIM_GET_COUNTER(s_htim_enc_R);

    // Xử lý tràn 16-bit bằng phép trừ số nguyên có dấu
    int16_t diff_L = (int16_t)(curr_L - s_prev_cnt_L) * ENC_LEFT_DIR_SIGN;
    int16_t diff_R = (int16_t)(curr_R - s_prev_cnt_R) * ENC_RIGHT_DIR_SIGN;
    s_prev_cnt_L = curr_L;
    s_prev_cnt_R = curr_R;

    // Tính số vòng quay/giây (RPS) và vận tốc góc (rad/s) với 1320 xung/vòng
    drv->rps_meas_L        = ((float)diff_L / ROBOT_ENCODER_CPR) / ROBOT_TS_MOTOR_S;
    drv->rps_meas_R        = ((float)diff_R / ROBOT_ENCODER_CPR) / ROBOT_TS_MOTOR_S;
    drv->omega_meas_L_rads = drv->rps_meas_L * (2.0f * ROBOT_PI);
    drv->omega_meas_R_rads = drv->rps_meas_R * (2.0f * ROBOT_PI);

    // Tích lũy quãng đường (m) và góc quay thân xe (độ) từ Encoder
    float ds_L = (drv->omega_meas_L_rads * ROBOT_WHEEL_RADIUS_M) * ROBOT_TS_MOTOR_S;
    float ds_R = (drv->omega_meas_R_rads * ROBOT_WHEEL_RADIUS_M) * ROBOT_TS_MOTOR_S;
    float ds_center = 0.5f * (ds_R + ds_L);
    float d_yaw_rad = (ds_R - ds_L) / ROBOT_WHEEL_BASE_M;

    drv->step_distance_m += fabsf(ds_center);
    drv->step_angle_deg  += d_yaw_rad * (180.0f / ROBOT_PI);

    if (drv->brake_active) {
        Drive_HardBrake(drv);
        return;
    }

    // Tính toán bộ điều khiển PI vận tốc 2 bánh
    float u_volt_L = Control_Step_PI(&s_pi_L, drv->omega_ref_L_rads, drv->omega_meas_L_rads, ROBOT_TS_MOTOR_S);
    float u_volt_R = Control_Step_PI(&s_pi_R, drv->omega_ref_R_rads, drv->omega_meas_R_rads, ROBOT_TS_MOTOR_S);

    // Quy đổi sang giá trị thanh ghi PWM (CCR: -999 .. +999)
    drv->pwm_ccr_L = (int16_t)((u_volt_L / ROBOT_SUPPLY_VOLTAGE) * ROBOT_PWM_ARR) * MOTOR_LEFT_DIR_SIGN;
    drv->pwm_ccr_R = (int16_t)((u_volt_R / ROBOT_SUPPLY_VOLTAGE) * ROBOT_PWM_ARR) * MOTOR_RIGHT_DIR_SIGN;

    Apply_Motor_PWM(M1_DIR1_PORT, M1_DIR1_PIN, M1_DIR2_PORT, M1_DIR2_PIN, M1_TIM_CHANNEL, drv->pwm_ccr_L);
    Apply_Motor_PWM(M2_DIR1_PORT, M2_DIR1_PIN, M2_DIR2_PORT, M2_DIR2_PIN, M2_TIM_CHANNEL, drv->pwm_ccr_R);
}
