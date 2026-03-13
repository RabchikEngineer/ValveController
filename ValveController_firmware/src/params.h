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
volatile extern params_t g_params;
volatile extern calibration_data_t g_calibration;

// Initialize from NVS
void nvs_init();
void config_load(void);

// Save to NVS
void config_save(void);

// Setters with validation
void config_set_kp(float val);
void config_set_ki(float val);
void config_set_kd(float val);
void config_set_deadzone(float val);
void config_set_pwm_freq(uint32_t val);
void config_set_ema_alpha(float val);
void calibration_set_actual_position(approx_poly_line_t p);
void calibration_set_desired_position(approx_poly_line_t p);
void calibration_set_current_loop(approx_poly_line_t p);

// Function to calculate true value
float apply_calibration(approx_poly_line_t polyline, float raw_value);