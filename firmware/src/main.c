#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "math.h"

//for serial communication
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/ledc.h"


#include "config.h"
#include "adc.h"
#include "usb_comm.h"


const char *TAG_PID = "PID System";
const char *TAG_INPUT = "Input Handler";
const char *TAG_VALVE = "Valve Switcher";
const char *TAG_PWM = "PWM System";
// const char *TAG_ADC = "ADC";
static const char *TAG = "Main Thread";


typedef struct {
    float x;
    float y;
} DataPoint_t;

typedef struct {
    int num_points;
    DataPoint_t* points;
} ApproxPoly_t;


QueueHandle_t inputQueue;
QueueHandle_t outputQueue;




const TickType_t polling_delay = pdMS_TO_TICKS(100);
const float PID_ATTENUATION = 1000.0f; // so that we can use higher Kp,Ki,Kd


volatile bool led_state = 0;

volatile float valve_activation_threshold = 0.2; //for -1 --- 1 pid output


//zone in which integral doesnt calculates, so the system ceases to stabilize
//so accuracy shold be 20/4096=0.5%
float dead_zone_size = 20; 

PIDConfig_t g_config = {
    2.0f, 
    1.0f, 
    2.0f
};




// // Define UART port and pins
// #define UART_PORT UART_NUM_1 // Example: Using UART1
// #define UART_TX_PIN GPIO_NUM_20
// #define UART_RX_PIN GPIO_NUM_21

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







void input_task(void *params) {
    ValvePositionData_t data;
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        // Read desired position from ADC or 4-20mA interface
        // data.desiredPositionPercent = ReadDesiredPosition();
        // data.desiredPositionPercent = 1500;
        data = get_adc_values();
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

    // float Kp = 0.004f;
    // float Ki = 0.001f; // работают но с перелётом
    // float Kd = 0.002f;

    // float Kp = 2.0;
    // float Ki = 1.0;
    // float Kd = 2.0;

    while (1) {
        if (xQueueReceive(inputQueue, &receivedData, portMAX_DELAY) == pdPASS) {

            // controlOutput = PID_Compute(&pid, receivedData.desiredPositionPercent, receivedData.actualPositionPercent);
            error = receivedData.desiredPositionPercent - receivedData.actualPositionPercent;

            int64_t current_time = esp_timer_get_time();
            dt = (current_time - last_time) / 1000000.0f; //calculating actual dt. however it might differs only for 1%
            last_time=current_time;

            float derivative = (error - prev_error) / dt;
            prev_error = error;

            pid_output = ((g_config.kp * error) + (g_config.ki * integral) + (g_config.kd * derivative)) / PID_ATTENUATION;

            bool saturated = (pid_output > 1.0f) || (pid_output < -1.0f);
            bool in_dead_zone = (fabsf(error) < dead_zone_size); // doesnt calculate integral in small zone
            // Only integrate if not saturated
            if (!saturated && !in_dead_zone) {
                integral += error * dt;
                if (integral > 500.0f) integral = 500.0f; 
                if (integral < -500.0f) integral = -500.0f;
            } else if (saturated){
                // Decay integral slowly when can't actuate
                integral *= 0.95f;
            } else if (in_dead_zone){
                integral = 0;
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
    
    vTaskDelay(pdMS_TO_TICKS(5000));


    ESP_LOGI(TAG,"Starting program");


    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    printf("Current PID: %.6f %.6f %.6f\n", g_config.kp, g_config.ki, g_config.kd);

    // DataPoint_t* points[] = {{1,2},{2,4}};
    // // ApproxPoly_t input_approx = {3,points};
    // ApproxPoly_t input_approx = {
    // .num_points = 2,
    // .points = points
    // };

    // for (int i=0; i<2; i++) {

    //     printf("%f",input_approx.points[i]);

    // }

    // gpio_set_direction(VALVE_INC_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_direction(VALVE_DEC_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_direction(CURRENT_POSITION_PIN, GPIO_MODE_INPUT);
    // gpio_set_direction(INPUT_PERCENT_PIN, GPIO_MODE_INPUT);


    inputQueue = xQueueCreate(10, sizeof(ValvePositionData_t));
    outputQueue = xQueueCreate(10, sizeof(float));


   
    

    valve_pwm_init();
    ESP_LOGI(TAG, "PWM configured");

    
    
    usb_comm_init();
    ESP_LOGI(TAG, "USB input configured");

    ESP_LOGI(TAG, "Starting tasks...");
    
    adc_start();
    
    xTaskCreate(input_task, "Input Task", 2048, NULL, 7, NULL);
    xTaskCreate(PID_task, "PID Task", 4096, NULL, 8, NULL);
    xTaskCreate(set_valve_pwm_task, "PWM&Output Task", 2048, NULL, 9, NULL);
    // xTaskCreate(usb_input_task, "input_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Tasks started...");


    // xTaskCreate(fan_control_task, "Fan task", 2048, NULL, 5, NULL);
    // xTaskCreate(rpm_task, "rpm_task", 2048, NULL, 5, NULL);


}

