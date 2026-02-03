#pragma once

#include <stdint.h>
#include <stdbool.h>


typedef struct {
    float x;
    float y;
} data_point_t;

typedef struct {
    int num_points;
    data_point_t* points;
} approx_poly_line_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float deadzone;
    uint32_t pwm_freq_hz;
    float ema_alpha;
} params_t;

typedef struct {
    approx_poly_line_t actual_position_approx;
    approx_poly_line_t desired_position_approx;
    approx_poly_line_t current_loop_approx;
} calibration_data_t;

// Global params instance
extern params_t g_params;
extern calibration_data_t g_calibration;

// Initialize from NVS
void nvs_init();
void params_load(void);

// Save to NVS
void params_save(void);

// Setters with validation
void params_set_kp(float val);
void params_set_ki(float val);
void params_set_kd(float val);
void params_set_deadzone(float val);
void params_set_pwm_freq(uint32_t val);
void params_set_ema_alpha(float val);
