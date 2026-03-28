#pragma once

#define VALVE_INC_PIN 0 
#define VALVE_DEC_PIN 1 
#define ACTUAL_POSITION_PIN     ADC_CHANNEL_3
#define DESIRED_POSITION_PIN    ADC_CHANNEL_4


// adc polling freq calculation: SAMPLE_FREQ_HZ/(READ_LEN/ADC_RESULT_BYTE)/2
// 5kHz - 4.88 Hz
// 10kHz - 9.76 Hz
// 20kHz - 19.53 Hz
#define ADC_RESULT_BYTE         SOC_ADC_DIGI_RESULT_BYTES  // ESP32-C3 specific
#define SAMPLE_FREQ_HZ          25000
#define BUFFER_SIZE             2048   // Must be multiple of ADC_RESULT_BYTE
#define READ_LEN                2048
#define ADC_UNIT                ADC_UNIT_1
#define ADC_CONV_MODE           ADC_CONV_SINGLE_UNIT_1
#define ADC_ATTEN               ADC_ATTEN_DB_2_5
#define ADC_BIT_WIDTH           SOC_ADC_DIGI_MAX_BITWIDTH
#define ADC_OUTPUT_TYPE         ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EMA_WEIGHT              0.4


#define PWM_FREQUENCY       10      // 10 Hz (100ms period)
#define PWM_RESOLUTION      LEDC_TIMER_14_BIT  // 0-1023 duty cycle
#define PWM_CHANNEL_INC     LEDC_CHANNEL_0
#define PWM_CHANNEL_DEC     LEDC_CHANNEL_1
#define PWM_TIMER           LEDC_TIMER_0


#define I2C_PORT        0
#define I2C_SDA_GPIO    8      
#define I2C_SCL_GPIO    9
#define I2C_FREQ_HZ     400000
#define I2C_RESPONSE_TIMEOUT_MS 100  


#define OLED_WIDTH             128
#define OLED_HEIGHT            64
// #define OLED_SCL_HZ            400000
#define OLED_RST_GPIO          -1     // set GPIO if you wired RST, else -1


#define MCP4725_I2C_ADDR     0x60   
#define PCF8574DWR_I2C_ADDR  0x20   
#define OLED_I2C_ADDR       0x3C 


#define POLLING_DELAY       30
#define VALVE_IDLE_DELAY    0

// UI SETTINGS
#define DEBOUNCE_MS        30
#define LONG_PRESS_MS      1000
#define REPEAT_START_MS    2000
#define REPEAT_INTERVAL_MS 100
#define ACTIVITY_TIMEOUT_US (60 * 1000000LL)
#define LOCK_COMBO_TIME_US  (2 * 1000000LL)
#define SCREEN_FPS          10
#define UI_CYCLE_FPS      50


// calibration settings
#define MAX_CAL_POINTS      5
#define CAL_MIN_AP_DELTA    5
#define CAL_MIN_DP_DELTA    5
#define CAL_MIN_CL_DELTA    5
#define CAL_CONFIRM_WAIT_TIME_MS 30000
#define CAL_DP_SET_VALUE_WAIT_TIME_MS 30000
#define CAL_CL_GET_VALUE_WAIT_TIME_MS 30000