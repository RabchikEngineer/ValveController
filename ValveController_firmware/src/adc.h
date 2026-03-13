#pragma once


typedef struct {
    float desiredPositionPercent;
    float actualPositionPercent;
} ValvePositionData_t;

void adc_start();

ValvePositionData_t get_calibrated_adc_values();
ValvePositionData_t get_raw_adc_values();

void set_desired_position_override(bool enabled);
void set_desired_position_override_value(float value);
