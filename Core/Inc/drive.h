/*
 * drive.h
 *
 *  Created on: Nov 17, 2026
 *      Author: ASUS
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include "robot_config.h"
#include "control.h"

typedef struct {
    // Tốc độ đặt từ vòng ngoài Lyapunov
    float omega_ref_L_rads;     // Vận tốc góc đặt bánh trái (rad/s)
    float omega_ref_R_rads;     // Vận tốc góc đặt bánh phải (rad/s)

    // Tốc độ thực tế đo từ Encoder 1320 xung/vòng (cập nhật mỗi 10ms)
    float omega_meas_L_rads;    // Vận tốc góc thực bánh trái (rad/s)
    float omega_meas_R_rads;    // Vận tốc góc thực bánh phải (rad/s)
    float rps_meas_L;           // Số vòng quay/giây bánh trái (vòng/s)
    float rps_meas_R;           // Số vòng quay/giây bánh phải (vòng/s)

    // Giá trị PWM xuất ra động cơ (0 .. 999)
    int16_t pwm_ccr_L;
    int16_t pwm_ccr_R;

    // Odometry phục vụ đo quãng đường (m) và góc quay (độ) khi né vật cản
    float step_distance_m;
    float step_angle_deg;
    bool  brake_active;
} Drive_State_t;

void Drive_Init(TIM_HandleTypeDef *htim_enc_L, TIM_HandleTypeDef *htim_enc_R,
                TIM_HandleTypeDef *htim_pwm, Drive_State_t *drv);
void Drive_ResetOdometry(Drive_State_t *drv);
void Drive_SetChassisTwist(Drive_State_t *drv, float v_mps, float w_rads);
void Drive_HardBrake(Drive_State_t *drv);
void Drive_Update10ms(Drive_State_t *drv);

#endif /* INC_DRIVE_H_ */
