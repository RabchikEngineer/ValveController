#pragma once


typedef struct {
    float desiredPositionPercent;
    float actualPositionPercent;
} ValvePositionData_t;

void adc_start();
ValvePositionData_t get_adc_values();

