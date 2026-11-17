/*
 * line_sensor.c
 *
 *  Created on: Nov 17, 2026
 *      Author: ASUS
 */
#include "line_sensor.h"

// Buffer DMA nhận trực tiếp từ 5 kênh ADC1 (PA0 -> PA4)
static volatile uint16_t dma_adc_buf[5];
static uint8_t obs_debounce_cnt = 0;

// Trọng số i = {-2, -1, 0, 1, 2} theo đúng công thức ảnh 2
static const float SENSOR_WEIGHT_I[5] = { -2.0f, -1.0f, 0.0f, 1.0f, 2.0f };

static inline float ClampF(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void LineSensor_Init(ADC_HandleTypeDef *hadc, LineSensor_Data_t *sens) {
    const float default_xmin[5] = SENSOR_ADC_X_MIN_INIT;
    const float default_xmax[5] = SENSOR_ADC_X_MAX_INIT;

    for (int k = 0; k < 5; k++) {
        sens->raw_adc[k] = 0;
        sens->x_min[k]   = default_xmin[k];
        sens->x_max[k]   = default_xmax[k];
        sens->y_cal[k]   = SENSOR_Y_MIN_C;
    }
    sens->x_weighted_mm     = 0.0f;
    sens->e2_mm             = 0.0f;
    sens->e2_m              = 0.0f;
    sens->black_count       = 0;
    sens->line_detected     = false;
    sens->fork_detected     = false;
    sens->obstacle_detected = false;
    obs_debounce_cnt        = 0;

    HAL_ADCEx_Calibration_Start(hadc);
    HAL_ADC_Start_DMA(hadc, (uint32_t*)dma_adc_buf, 5);
}

void LineSensor_SetCalibration(LineSensor_Data_t *sens, const float x_min[5], const float x_max[5]) {
    for (int k = 0; k < 5; k++) {
        sens->x_min[k] = x_min[k];
        sens->x_max[k] = x_max[k];
    }
}

void LineSensor_Update10ms(LineSensor_Data_t *sens, LineSensor_Mode_t mode) {
    /* 1. Đọc và chống nhiễu cảm biến vật cản E18-D80NK (3 nhịp 10ms = 30ms) */
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(OBS_GPIO_PORT, OBS_GPIO_PIN);
    bool raw_obs = (OBS_ACTIVE_LOW) ? (pin_state == GPIO_PIN_RESET) : (pin_state == GPIO_PIN_SET);
    if (raw_obs) {
        if (obs_debounce_cnt < 3) obs_debounce_cnt++;
        if (obs_debounce_cnt >= 3) sens->obstacle_detected = true;
    } else {
        obs_debounce_cnt = 0;
        sens->obstacle_detected = false;
    }

    /* 2. Lấy snapshot 5 kênh ADC và sắp xếp thứ tự từ Trái (i=-2) sang Phải (i=+2) */
    for (int k = 0; k < 5; k++) {
#if ROBOT_LINE_ADC_REVERSED
        sens->raw_adc[k] = dma_adc_buf[4 - k]; // PA4 (Trái ngoài) -> index 0 (i=-2)
#else
        sens->raw_adc[k] = dma_adc_buf[k];
#endif
    }

    /* 3. Hiệu chuẩn từng cảm biến theo công thức:
     * y_i,C = y_minC + ((y_maxC - y_minC) / (x_i,max - x_i,min)) * (x_i - x_i,min) */
    uint8_t b_cnt = 0;
    for (int k = 0; k < 5; k++) {
        float denom = sens->x_max[k] - sens->x_min[k];
        if (fabsf(denom) < 1.0f) denom = 1.0f;

        float y_c = SENSOR_Y_MIN_C + ((SENSOR_Y_MAX_C - SENSOR_Y_MIN_C) / denom)
                                   * ((float)sens->raw_adc[k] - sens->x_min[k]);
        y_c = ClampF(y_c, SENSOR_Y_MIN_C, SENSOR_Y_MAX_C);

        // Cắt ngưỡng nhiễu nền trắng nhỏ (< 5% dải đo) để không làm lệch trung bình trọng số
        if (y_c < (SENSOR_Y_MIN_C + 0.05f * (SENSOR_Y_MAX_C - SENSOR_Y_MIN_C))) {
            y_c = 0.0f;
        }

        sens->y_cal[k] = y_c;
        if (y_c >= SENSOR_SINGLE_BLACK_THRESH) {
            b_cnt++;
        }
    }
    sens->black_count = b_cnt;

    /* 4. Nhận diện mẫu ngã rẽ: >= 4 mắt cùng đen hoặc 2 mắt hai bên đen mà mắt giữa trắng */
    bool split_pattern = (sens->y_cal[1] > SENSOR_SINGLE_BLACK_THRESH &&
                          sens->y_cal[3] > SENSOR_SINGLE_BLACK_THRESH &&
                          sens->y_cal[2] < (0.4f * SENSOR_Y_MAX_C));
    sens->fork_detected = (b_cnt >= 4) || split_pattern;

    /* 5. Tính trung bình có trọng số theo công thức:
     * x = L_cb * Sum(i * y_i,C) / Sum(y_i,C)   với i = {-2, -1, 0, 1, 2}, L_cb = 12mm */
    float num = 0.0f;
    float den = 0.0f;
    int start_idx = 0, end_idx = 4;

    if (mode == LINE_TRACK_LEFT_HALF) {
        end_idx = 2;   // Chỉ xét i = {-2, -1, 0} để bám nhánh trái
    } else if (mode == LINE_TRACK_RIGHT_HALF) {
        start_idx = 2; // Chỉ xét i = {0, +1, +2} để bám nhánh phải
    }

    for (int k = start_idx; k <= end_idx; k++) {
        num += SENSOR_WEIGHT_I[k] * sens->y_cal[k];
        den += sens->y_cal[k];
    }

    if (den < SENSOR_LINE_DETECT_SUM_MIN) {
        sens->line_detected = false;
        return; // Giữ nguyên dấu e2_m cuối cùng trước khi mất line để hỗ trợ tìm lại
    }

    sens->line_detected = true;
    sens->x_weighted_mm = ROBOT_SENSOR_PITCH_MM * (num / den);

    /* 6. Tuyến tính hóa bậc nhất giữa giá trị trung bình trọng số và khoảng cách thực (mm):
     * Qui ước dấu cho bộ điều khiển Lyapunov (w > 0 là quay trái):
     * Khi vạch line nằm bên Trái (i < 0 -> x_weighted_mm < 0), xe cần quay Trái (e2 > 0).
     * Do đó đổi dấu (-x_weighted_mm) để lệch Trái ứng với e2 > 0, lệch Phải ứng với e2 < 0. */
    float x_centered_mm = -sens->x_weighted_mm;
    sens->e2_mm = (ROBOT_LINE_LIN_A * x_centered_mm) + ROBOT_LINE_LIN_B;
    sens->e2_m  = sens->e2_mm * 0.001f; // Đổi từ mm sang mét (m)
}

