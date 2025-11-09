#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "driver/ledc.h"


#define LED_PIN GPIO_NUM_1
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


#define PWM_FREQUENCY       10      // 10 Hz (100ms period)
#define PWM_RESOLUTION      LEDC_TIMER_14_BIT  // 0-1023 duty cycle
#define PWM_CHANNEL_INC     LEDC_CHANNEL_0
#define PWM_CHANNEL_DEC     LEDC_CHANNEL_1
#define PWM_TIMER           LEDC_TIMER_0


const char *TAG_PID = "PID System";
const char *TAG_INPUT = "Input Handler";
const char *TAG_VALVE = "Valve Switcher";
const char *TAG_PWM = "PWM System";
const char *TAG_ADC = "ADC";
const char *TAG = "Main Thread";


typedef struct {
    float desiredPositionPercent;
    float actualPositionPercent;
} ValvePositionData_t;

QueueHandle_t inputQueue;
QueueHandle_t outputQueue;


static adc_channel_t channels[2] = {ADC_CHANNEL_3, ADC_CHANNEL_4};
static adc_continuous_handle_t adc_handle = NULL;

const TickType_t polling_delay = pdMS_TO_TICKS(100);


volatile bool led_state = 0;
// volatile bool desired_position = 0;
// volatile bool current_position = 0;
static volatile float latest_actual_position = 0.0f;
static volatile float latest_desired_position = 0.0f;
volatile float valve_activation_threshold = 0.2; //for -1 --- 1 pid output
float adc_prev_value_weight = 0.6;
float adc_curr_value_weight = 0.4;


//zone in which integral doesnt calculates, so the system ceases to stabilize
//so accuracy shold be 20/4096=0.5%
float dead_zone_size = 20; 



// typedef enum {
//     CONTROL_INCREASE,
//     CONTROL_DECREASE,
//     CONTROL_STOP
// } ControlCommand_t;




// #include "driver/uart.h"

// // Define UART port and pins
// #define UART_PORT UART_NUM_1 // Example: Using UART1
// #define UART_TX_PIN GPIO_NUM_4
// #define UART_RX_PIN GPIO_NUM_5

// void uart_init() {
//     uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_APB,
//     };
//     // Configure UART parameters
//     uart_param_config(UART_PORT, &uart_config);
//     // Set UART pins
//     uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//     // Install UART driver, and get the queue.
//     uart_driver_install(UART_PORT, 256, 0, 0, NULL, 0);
// }


void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle) {
    adc_continuous_handle_t handle = NULL;

    // Step 1: Create handle with buffer configuration
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = BUFFER_SIZE,
        .conv_frame_size = READ_LEN,  // Replaced conv_num_each_intr
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    // Step 2: Configure ADC controller
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    // Step 3: Configure channel patterns
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT;
        adc_pattern[i].bit_width = ADC_BIT_WIDTH;

        ESP_LOGI(TAG_ADC, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG_ADC, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG_ADC, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

void valve_pwm_init(void) {
    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = PWM_RESOLUTION,
        .timer_num        = PWM_TIMER,
        .freq_hz          = PWM_FREQUENCY,
        .clk_cfg          = LEDC_USE_RC_FAST_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure channel for INCREASE valve
    ledc_channel_config_t ledc_channel_inc = {
        .gpio_num       = VALVE_INC_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = PWM_CHANNEL_INC,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = PWM_TIMER,
        .duty           = 0,  // Start at 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_inc));

    // Configure channel for DECREASE valve
    ledc_channel_config_t ledc_channel_dec = {
        .gpio_num       = VALVE_DEC_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = PWM_CHANNEL_DEC,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = PWM_TIMER,
        .duty           = 0,  // Start at 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_dec));

    ESP_LOGI(TAG_VALVE, "PWM initialized: %d Hz, %d-bit resolution", 
             PWM_FREQUENCY, PWM_RESOLUTION);
}


void adc_continuous_read_task(void *arg) {
    uint8_t result[READ_LEN] = {0};
    uint32_t ret_num = 0;
    esp_err_t ret;

    
    
    // Start continuous conversion
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    ESP_LOGI(TAG_ADC, "ADC continuous mode started");
    
    while (1) {

        // Read bytes - blocking with portMAX_DELAY
        ret = adc_continuous_read(adc_handle, result, READ_LEN, &ret_num, portMAX_DELAY);
        
        if (ret == ESP_OK) {
            int64_t sum_ch3 = 0, count_ch3 = READ_LEN/ADC_RESULT_BYTE/2; // 4 bytes per sample, 2 channels
            int64_t sum_ch4 = 0, count_ch4 = READ_LEN/ADC_RESULT_BYTE/2;
            
            // Process samples - accumulate for averaging
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                
                uint32_t chan_num = p->type2.channel;
                uint32_t data = p->type2.data;
                
                // Check valid channel
                if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT)) {
                    if (chan_num == 3) {
                        sum_ch3 += data;
                        // count_ch3++;
                    } else if (chan_num == 4) {
                        sum_ch4 += data;
                        // count_ch4++;
                    }
                }
            }
            
            // Average with float precision (effective resolution increase)
            // if (count_ch3 > 0) {
                latest_actual_position = latest_actual_position*adc_prev_value_weight+((float)sum_ch3 / (float)count_ch3)*adc_curr_value_weight;
            // }
            // if (count_ch4 > 0) {
                latest_desired_position = latest_desired_position*adc_prev_value_weight+((float)sum_ch4 / (float)count_ch4)*adc_curr_value_weight;
            // }
            
            // ESP_LOGI(TAG_ADC, "Pos: %.3f, Input: %.3f (samples: %lld)", 
            //          latest_actual_position, latest_desired_position, count_ch4);
                     
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG_ADC, "ADC read timeout");
        } else {
            ESP_LOGE(TAG_ADC, "ADC read error: %d", ret);
        }
        
        // Small delay to prevent watchdog timeout
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // Cleanup (never reached)
    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(adc_handle));
}









void input_task(void *params) {
    ValvePositionData_t data;
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        // Read desired position from ADC or 4-20mA interface
        // data.desiredPositionPercent = ReadDesiredPosition();
        // data.desiredPositionPercent = 1500;
        data.desiredPositionPercent = latest_desired_position;
        // Read actual valve position from ADC
        // data.actualPositionPercent = ReadActualPosition();
        data.actualPositionPercent = latest_actual_position;
        // Send to PID task
        xQueueSend(inputQueue, &data, portMAX_DELAY);

        xTaskDelayUntil(&last_wake_time,polling_delay); 
    }
}



void PID_task(void *params) {
    ValvePositionData_t receivedData;
    // PIDController pid;
    // PID_Init(&pid); // initialize PID with tuned params
    
    float error = 0;
    float pid_output = 0;
    float integral = 0.0f;
    float prev_error = 0.0f;
    float dt=0.1f;
    int64_t last_time=0;

    float Kp = 0.004f;
    float Ki = 0.0005f;
    float Kd = 0.002f;

    while (1) {
        if (xQueueReceive(inputQueue, &receivedData, portMAX_DELAY) == pdPASS) {

            // controlOutput = PID_Compute(&pid, receivedData.desiredPositionPercent, receivedData.actualPositionPercent);
            error = receivedData.desiredPositionPercent - receivedData.actualPositionPercent;

            int64_t current_time = esp_timer_get_time();
            dt = (current_time - last_time) / 1000000.0f; //calculating actual dt. however it might differs only for 1%
            last_time=current_time;


            float derivative = (error - prev_error) / dt;
            pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);
            prev_error = error;


            bool saturated = (pid_output > 100.0f) || (pid_output < -100.0f);
            bool in_dead_zone = (fabsf(error) < dead_zone_size); // doesnt calculate integral in small zone
            // Only integrate if not saturated
            if (!saturated && !in_dead_zone) {
                integral += error * dt;
                if (integral > 500.0f) integral = 500.0f; 
                if (integral < -500.0f) integral = -500.0f;
            } else {
                // Decay integral slowly when can't actuate
                integral *= 0.95f;
            }


            // Convert controlOutput to command for valves
            ESP_LOGI(TAG_PID, "A: %.3f/D: %.3f,I: %.1f , dt: %.5f, %.4f", 
                     receivedData.actualPositionPercent, receivedData.desiredPositionPercent, integral, dt, pid_output);
            xQueueSend(outputQueue, &pid_output, portMAX_DELAY);
        }
    }
}


void set_valve_pwm_task() {
    // PID output ranges (example: -100 to +100)
    // Negative = decrease valve, Positive = increase valve
    
    uint32_t max_duty = (1 << PWM_RESOLUTION) - 1;  // 1023 for 10-bit
    uint32_t duty_inc = 0;
    uint32_t duty_dec = 0;

    float pid_output=0;
    
    while (1) {
        if (xQueueReceive(outputQueue, &pid_output, portMAX_DELAY) == pdPASS) {

            if (pid_output-valve_activation_threshold > 0) {
                // Open INCREASE valve proportionally
                duty_inc = (uint32_t)(pid_output * max_duty);
                duty_dec = 0;
            } else if (pid_output+valve_activation_threshold < 0) {
                // Open DECREASE valve proportionally
                duty_inc = 0;
                duty_dec = (uint32_t)(-pid_output * max_duty);
            } else {
                // Stop both valves
                duty_inc = 0;
                duty_dec = 0;
            }
            
            // Clamp duty cycle
            if (duty_inc > max_duty) duty_inc = max_duty;
            if (duty_dec > max_duty) duty_dec = max_duty;
            
            // Set PWM duty cycles
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_INC, duty_inc));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_INC));
            
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_DEC, max_duty-duty_dec)); //inverting output
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_DEC));
            
            ESP_LOGI(TAG_VALVE, "PID: %.2f -> INC duty: %.2f, DEC duty: %.2f", 
                    pid_output, (float)duty_inc/max_duty, (float)duty_dec/max_duty);
        }
    }

    
}


void app_main(void)
{   
    ESP_LOGI(TAG,"Starting program");
    
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_direction(VALVE_INC_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_direction(VALVE_DEC_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_direction(CURRENT_POSITION_PIN, GPIO_MODE_INPUT);
    // gpio_set_direction(INPUT_PERCENT_PIN, GPIO_MODE_INPUT);


    inputQueue = xQueueCreate(10, sizeof(ValvePositionData_t));
    outputQueue = xQueueCreate(10, sizeof(float));


    continuous_adc_init(channels, sizeof(channels) / sizeof(adc_channel_t), &adc_handle);
    ESP_LOGI(TAG, "ADC configured");

    valve_pwm_init();
    ESP_LOGI(TAG, "PWM configured");    


    ESP_LOGI(TAG, "Starting tasks...");
    xTaskCreate(adc_continuous_read_task, "ADC Continuous", 8192, NULL, 10, NULL);
    xTaskCreate(input_task, "Input Task", 2048, NULL, 7, NULL);
    xTaskCreate(PID_task, "PID Task", 4096, NULL, 8, NULL);
    xTaskCreate(set_valve_pwm_task, "PWM&Output Task", 2048, NULL, 9, NULL);
    ESP_LOGI(TAG, "Tasks started...");


    // xTaskCreate(fan_control_task, "Fan task", 2048, NULL, 5, NULL);
    // xTaskCreate(rpm_task, "rpm_task", 2048, NULL, 5, NULL);
}

