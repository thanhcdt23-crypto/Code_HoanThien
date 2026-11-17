/*
 * robot_app.c
 *
 *  Created on: Nov 17, 2026
 *      Author: ASUS
 */

#include "robot_app.h"

volatile uint8_t robot_branch_select = ROBOT_DEFAULT_BRANCH_LEFT;

static Robot_State_t     s_state = SM_FOLLOW_LINE_BEFORE_OBS;
static LineSensor_Data_t s_sensor;
static Drive_State_t     s_drive;
static Tracking_Error_t  s_err;

static uint8_t s_tick_50ms_cnt = 0;
static uint8_t s_pause_ticks   = 0;
static uint8_t s_end_line_cnt  = 0;

void RobotApp_Init(ADC_HandleTypeDef *hadc,
                   TIM_HandleTypeDef *htim_enc_L,
                   TIM_HandleTypeDef *htim_enc_R,
                   TIM_HandleTypeDef *htim_pwm) {
    LineSensor_Init(hadc, &s_sensor);
    Drive_Init(htim_enc_L, htim_enc_R, htim_pwm, &s_drive);

    s_state         = SM_FOLLOW_LINE_BEFORE_OBS;
    s_tick_50ms_cnt = 0;
    s_pause_ticks   = 0;
    s_end_line_cnt  = 0;
}

/* Vòng điều khiển ngoài thực thi mỗi 50ms (0.05s) */
static void RobotApp_OuterLoop50ms(void) {
    float v_cmd = 0.0f;
    float w_cmd = 0.0f;

    // Nội suy 3 sai số (e1, e2, e3) từ sai số cảm biến e2 và khoảng cách d = 50mm
    Control_Interpolate_Errors(s_sensor.e2_m, ROBOT_SENSOR_LOOKAHEAD_D_M, &s_err);

    switch (s_state) {
        /* 1. Bám line bằng bộ điều khiển Lyapunov và chờ phát hiện vật cản (30cm) */
        case SM_FOLLOW_LINE_BEFORE_OBS:
            Control_Lyapunov_Step50ms(&s_err, ROBOT_V_BASE_MPS, 0.0f, &v_cmd, &w_cmd);
            Drive_SetChassisTwist(&s_drive, v_cmd, w_cmd);
            break;

        /* 2. Dừng ngắn 200ms (4 nhịp 50ms) để triệt tiêu quán tính trước khi xoay 45 độ */
        case SM_AVOID_STOP_INIT:
            Drive_HardBrake(&s_drive);
            if (++s_pause_ticks >= 4) {
                Drive_ResetOdometry(&s_drive);
                s_state = SM_AVOID_TURN_LEFT_45;
            }
            break;

        /* 3. Xoay xe sang Trái một góc cố định 45 độ (đọc từ Encoder) */
        case SM_AVOID_TURN_LEFT_45:
            if (s_drive.step_angle_deg >= ROBOT_AVOID_TURN_LEFT_DEG) {
                Drive_ResetOdometry(&s_drive);
                s_state = SM_AVOID_GO_STRAIGHT_30CM;
            } else {
                Drive_SetChassisTwist(&s_drive, 0.0f, 1.8f); // Xoay trái tại chỗ (w > 0)
            }
            break;

        /* 4. Đi thẳng tới ngang vật cản (khoảng cách cố định 30cm tính từ Encoder) */
        case SM_AVOID_GO_STRAIGHT_30CM:
            if (s_drive.step_distance_m >= ROBOT_AVOID_STRAIGHT1_M) {
                Drive_ResetOdometry(&s_drive);
                s_state = SM_AVOID_TURN_RIGHT_90;
            } else {
                Drive_SetChassisTwist(&s_drive, ROBOT_V_AVOID_MPS, 0.0f);
            }
            break;

        /* 5. Xoay xe sang Phải một góc 90 độ hướng về phía line */
        case SM_AVOID_TURN_RIGHT_90:
            if (fabsf(s_drive.step_angle_deg) >= ROBOT_AVOID_TURN_RIGHT_DEG) {
                Drive_ResetOdometry(&s_drive);
                s_state = SM_AVOID_STRAIGHT_FIND_LINE;
            } else {
                Drive_SetChassisTwist(&s_drive, 0.0f, -1.8f); // Xoay phải tại chỗ (w < 0)
            }
            break;

        /* 6. Đi thẳng và tin cậy lại tín hiệu cảm biến dò line để nhận diện lại line */
        case SM_AVOID_STRAIGHT_FIND_LINE:
            if (s_sensor.line_detected) {
                Drive_ResetOdometry(&s_drive);
                s_state = SM_AVOID_ALIGN_ON_LINE;
            } else if (s_drive.step_distance_m >= ROBOT_AVOID_FIND_MAX_M) {
                // Quá 90cm không tìm thấy line -> Dừng an toàn
                Drive_HardBrake(&s_drive);
                s_state = SM_STOP_FINISHED;
            } else {
                Drive_SetChassisTwist(&s_drive, ROBOT_V_AVOID_MPS, 0.0f);
            }
            break;

        /* 7. Căn chỉnh xe mượt vào lại line sau khi cắt line ở góc xiên */
        case SM_AVOID_ALIGN_ON_LINE:
            Control_Lyapunov_Step50ms(&s_err, ROBOT_V_AVOID_MPS, 0.0f, &v_cmd, &w_cmd);
            Drive_SetChassisTwist(&s_drive, v_cmd, w_cmd);
            if (s_drive.step_distance_m >= 0.08f && fabsf(s_sensor.e2_mm) < 8.0f) {
                Drive_ResetOdometry(&s_drive);
                s_state = SM_FOLLOW_LINE_TO_FORK;
            }
            break;

        /* 8. Bám line qua cung R500 và kiểm tra ngã rẽ */
        case SM_FOLLOW_LINE_TO_FORK:
            if (s_drive.step_distance_m >= ROBOT_MIN_DIST_TO_FORK_M && s_sensor.fork_detected) {
                Drive_ResetOdometry(&s_drive);
                s_state = SM_SELECT_BRANCH_TURN;
                break;
            }
            Control_Lyapunov_Step50ms(&s_err, ROBOT_V_BASE_MPS, 0.0f, &v_cmd, &w_cmd);
            Drive_SetChassisTwist(&s_drive, v_cmd, w_cmd);
            break;

        /* 9. Chọn nhánh Rẽ Trái hoặc Rẽ Phải tại ngã ba */
        case SM_SELECT_BRANCH_TURN:
            Control_Lyapunov_Step50ms(&s_err, ROBOT_V_AVOID_MPS, 0.0f, &v_cmd, &w_cmd);
            Drive_SetChassisTwist(&s_drive, v_cmd, w_cmd);
            if (s_drive.step_distance_m >= 0.15f && !s_sensor.fork_detected) {
                Drive_ResetOdometry(&s_drive);
                s_end_line_cnt = 0;
                s_state = SM_FOLLOW_LINE_TO_END;
            }
            break;

        /* 10. Bám line về đích và dừng cuối line */
        case SM_FOLLOW_LINE_TO_END:
            Control_Lyapunov_Step50ms(&s_err, ROBOT_V_BASE_MPS, 0.0f, &v_cmd, &w_cmd);
            Drive_SetChassisTwist(&s_drive, v_cmd, w_cmd);
            break;

        case SM_STOP_FINISHED:
        default:
            Drive_HardBrake(&s_drive);
            break;
    }
}

/* Hàm thực thi mỗi 10ms (0.01s) từ ngắt Timer */
void RobotApp_Tick10ms_ISR(void) {
    // 1. Chọn chế độ đọc nửa dãy cảm biến nếu đang ở trạng thái tách nhánh ngã rẽ
    LineSensor_Mode_t sens_mode = LINE_TRACK_ALL;
    if (s_state == SM_SELECT_BRANCH_TURN) {
        sens_mode = (robot_branch_select != 0) ? LINE_TRACK_LEFT_HALF : LINE_TRACK_RIGHT_HALF;
    }

    // 2. Cập nhật ADC 5 cảm biến line, công thức hiệu chuẩn và E18-D80NK mỗi 10ms
    LineSensor_Update10ms(&s_sensor, sens_mode);

    // 3. Kiểm tra nhanh sự kiện Vật cản (30cm) và Cuối line ngay ở độ phân giải 10ms
    if (s_state == SM_FOLLOW_LINE_BEFORE_OBS && s_sensor.obstacle_detected) {
        Drive_HardBrake(&s_drive);
        s_pause_ticks = 0;
        s_state = SM_AVOID_STOP_INIT;
    } else if (s_state == SM_AVOID_STRAIGHT_FIND_LINE && s_sensor.line_detected) {
        // Bắt lại line ngay lập tức ở chu kỳ 10ms khi đang đi thẳng sau góc xoay 90 độ
        Drive_ResetOdometry(&s_drive);
        s_state = SM_AVOID_ALIGN_ON_LINE;
    } else if (s_state == SM_FOLLOW_LINE_TO_END && s_drive.step_distance_m > 0.20f) {
        if (!s_sensor.line_detected) {
            Drive_HardBrake(&s_drive); // Phanh ngay lập tức để đạt sai số dừng +-5mm
            if (++s_end_line_cnt >= 5) {
                s_state = SM_STOP_FINISHED;
            }
        } else {
            s_end_line_cnt = 0;
        }
    }

    // 4. Mỗi 5 nhịp 10ms = 50ms (0.05s): Thực thi bộ điều khiển ngoài Lyapunov
    s_tick_50ms_cnt++;
    if (s_tick_50ms_cnt >= 5) {
        s_tick_50ms_cnt = 0;
        RobotApp_OuterLoop50ms();
    }

    // 5. Mỗi 10ms (0.01s): Thực thi bộ điều khiển PI tốc độ động cơ vòng trong
    Drive_Update10ms(&s_drive);
}

Robot_State_t RobotApp_GetState(void) { return s_state; }
const LineSensor_Data_t* RobotApp_GetSensorData(void) { return &s_sensor; }
const Drive_State_t* RobotApp_GetDriveData(void) { return &s_drive; }
const Tracking_Error_t* RobotApp_GetTrackingError(void) { return &s_err; }
