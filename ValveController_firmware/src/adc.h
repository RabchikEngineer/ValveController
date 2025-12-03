// adc.h - Minimal public interface
#ifndef ADC_H
#define ADC_H


typedef struct {
    float desiredPositionPercent;
    float actualPositionPercent;
} ValvePositionData_t;

void adc_start();
ValvePositionData_t get_adc_values();

#endif