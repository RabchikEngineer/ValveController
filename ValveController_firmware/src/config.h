
#define VALVE_INC_PIN 0 
#define VALVE_DEC_PIN 1 
#define CURRENT_POSITION_PIN ADC_CHANNEL_3
#define INPUT_PERCENT_PIN ADC_CHANNEL_4
// #define OUTPUT_PERCENT_PIN 2


#define ADC_RESULT_BYTE         4  // ESP32-C3 specific
#define SAMPLE_FREQ_HZ          5000  // 5 kHz sampling
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


typedef struct {
    float kp;
    float ki;
    float kd;
} PIDConfig_t;


static const PIDConfig_t DEFAULT_PID = {
    .kp = 2.0f,
    .ki = 1.0f,
    .kd = 2.0f
};
