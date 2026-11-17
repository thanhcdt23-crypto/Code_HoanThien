/*
 * robot_app.h
 *
 *  Created on: Nov 17, 2026
 *      Author: ASUS
 */

#ifndef INC_ROBOT_APP_H_
#define INC_ROBOT_APP_H_

#include "robot_config.h"
#include "line_sensor.h"
#include "drive.h"
#include "control.h"

typedef enum {
    SM_FOLLOW_LINE_BEFORE_OBS = 0, // Bám line bằng Lyapunov trước khi gặp vật cản
    SM_AVOID_STOP_INIT,            // Dừng ngắn ổn định xe khi phát hiện vật cản ở 30cm
    SM_AVOID_TURN_LEFT_45,         // Xoay trái cố định 45 độ (đọc từ Encoder)
    SM_AVOID_GO_STRAIGHT_30CM,     // Đi thẳng 30cm tới ngang vật cản (đọc từ Encoder)
    SM_AVOID_TURN_RIGHT_90,        // Xoay phải cố định 90 độ hướng về phía line
    SM_AVOID_STRAIGHT_FIND_LINE,   // Đi thẳng và tin cậy lại cảm biến dò line để bắt lại line
    SM_AVOID_ALIGN_ON_LINE,        // Căn chỉnh xe vào lại tâm line trước khi tăng tốc
    SM_FOLLOW_LINE_TO_FORK,        // Bám line chờ đến ngã rẽ
    SM_SELECT_BRANCH_TURN,         // Rẽ nhánh Trái hoặc Phải tại ngã ba
    SM_FOLLOW_LINE_TO_END,         // Bám line về đích
    SM_STOP_FINISHED               // Dừng xe tại điểm kết thúc
} Robot_State_t;

extern volatile uint8_t robot_branch_select; // 1: Nhánh Trái, 0: Nhánh Phải

void RobotApp_Init(ADC_HandleTypeDef *hadc,
                   TIM_HandleTypeDef *htim_enc_L,
                   TIM_HandleTypeDef *htim_enc_R,
                   TIM_HandleTypeDef *htim_pwm);

/* Gọi hàm này mỗi 10ms (0.01s) từ ngắt Timer */
void RobotApp_Tick10ms_ISR(void);

Robot_State_t           RobotApp_GetState(void);
const LineSensor_Data_t* RobotApp_GetSensorData(void);
const Drive_State_t*     RobotApp_GetDriveData(void);
const Tracking_Error_t*  RobotApp_GetTrackingError(void);

#endif /* INC_ROBOT_APP_H_ */
