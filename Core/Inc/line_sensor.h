/*
 * line_sensor.h
 *
 *  Created on: Nov 17, 2026
 *      Author: ASUS
 */

#ifndef INC_LINE_SENSOR_H_
#define INC_LINE_SENSOR_H_

#include <robot_config.h>

typedef enum {
    LINE_TRACK_ALL = 0,     // Dùng cả 5 cảm biến (Bám line thông thường)
    LINE_TRACK_LEFT_HALF,   // Chỉ ưu tiên nửa trái khi tách nhánh rẽ trái
    LINE_TRACK_RIGHT_HALF   // Chỉ ưu tiên nửa phải khi tách nhánh rẽ phải
} LineSensor_Mode_t;

typedef struct {
    uint16_t raw_adc[5];        // Giá trị ADC thô (sau khi đã sắp xếp từ Trái i=-2 đến Phải i=+2)
    float    x_min[5];          // Giá trị ADC nhỏ nhất (Trắng) của từng cảm biến x_i,min
    float    x_max[5];          // Giá trị ADC lớn nhất (Đen) của từng cảm biến x_i,max
    float    y_cal[5];          // Giá trị sau hiệu chuẩn y_i,C theo công thức (1)
    float    x_weighted_mm;     // Giá trị trung bình trọng số x (mm) theo công thức (2)
    float    e2_mm;             // Khoảng cách thực xấp xỉ so với tâm line sau tuyến tính hóa (mm)
    float    e2_m;              // Sai số lệch ngang e2 đổi sang đơn vị mét (m)
    uint8_t  black_count;       // Số mắt cảm biến đang nhận vạch đen
    bool     line_detected;     // Cờ phát hiện có line
    bool     fork_detected;     // Cờ phát hiện ngã rẽ (>= 4 mắt đen hoặc tách 2 nhánh)
    bool     obstacle_detected; // Cờ phát hiện vật cản từ E18-D80NK (lọc chống nhiễu)
} LineSensor_Data_t;

void LineSensor_Init(ADC_HandleTypeDef *hadc, LineSensor_Data_t *sens);
void LineSensor_SetCalibration(LineSensor_Data_t *sens, const float x_min[5], const float x_max[5]);
void LineSensor_Update10ms(LineSensor_Data_t *sens, LineSensor_Mode_t mode);

#endif /* INC_SENSOR_H_ */
