#include "esp_log.h"
#include "nvs_flash.h"
#include "cJSON.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "esp_adc/adc_oneshot.h"



#define LED_PIN GPIO_NUM_1
#define VALVE_INC_PIN 0 
#define VALVE_DEC_PIN 1 
#define CURRENT_POSITION_PIN ADC_CHANNEL_3
#define INPUT_PERCENT_PIN ADC_CHANNEL_4
// #define OUTPUT_PERCENT_PIN 2

const char *TAG_PID = "PID System";
const char *TAG_INPUT = "Input Handler";
const char *TAG_PISTON = "Piston Controller";
const char *TAG_ADC = "ADC";
const char *TAG = "Main Thread";

volatile bool led_state = 0;
volatile bool desired_position = 0;
volatile bool current_position = 0;

QueueHandle_t inputQueue;
QueueHandle_t outputQueue;


typedef struct {
    float desiredPositionPercent;
    float actualPositionPercent;
} ValvePositionData_t;

typedef enum {
    CONTROL_INCREASE,
    CONTROL_DECREASE,
    CONTROL_STOP
} ControlCommand_t;

adc_oneshot_unit_handle_t adc1_handle;


void fan_control_task(void *arg) {


    while (1) {

        

        ESP_LOGI(TAG_PISTON,"I'm alive...");
        

        vTaskDelay(pdMS_TO_TICKS(1000));
        
    }


}


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




void configure_adc(adc_atten_t ADC_ATTEN) {
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CURRENT_POSITION_PIN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, INPUT_PERCENT_PIN, &config));

}


float read_actual_position(adc_channel_t pin) {
    int adc_raw=0;

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, pin, &adc_raw));
    
    // ESP_LOGI(TAG_ADC, "Calibrated: %d",adc_calibrated);
    ESP_LOGI(TAG_ADC,"Value on %i: %i", pin, adc_raw);

    return adc_raw;


}



void input_task(void *params) {
    ValvePositionData_t data;
    while (1) {
        // Read desired position from ADC or 4-20mA interface
        // data.desiredPositionPercent = ReadDesiredPosition();
        // data.desiredPositionPercent = 1500;
        data.desiredPositionPercent = read_actual_position(INPUT_PERCENT_PIN);
        // Read actual valve position from ADC
        // data.actualPositionPercent = ReadActualPosition();
        data.actualPositionPercent = read_actual_position(CURRENT_POSITION_PIN);
        // Send to PID task
        xQueueSend(inputQueue, &data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(500)); // adjust sample rate
    }
}



void output_task(void *params) {
    ControlCommand_t command;
    while (1) {
        if (xQueueReceive(outputQueue, &command, portMAX_DELAY) == pdPASS) {
            ESP_LOGI(TAG_PISTON, "Received item");
            switch(command) {
                case CONTROL_INCREASE:
                    gpio_set_level(VALVE_INC_PIN, 1);
                    gpio_set_level(VALVE_DEC_PIN, 1);
                    break;
                case CONTROL_DECREASE:
                    gpio_set_level(VALVE_INC_PIN, 0);
                    gpio_set_level(VALVE_DEC_PIN, 0);
                    break;
                case CONTROL_STOP:
                    gpio_set_level(VALVE_INC_PIN, 0);
                    gpio_set_level(VALVE_DEC_PIN, 1);
                    break;
            }
        }
        ESP_LOGI(TAG_PISTON, "Next iteration");
    }
}


void PID_task(void *params) {
    ValvePositionData_t receivedData;
    ControlCommand_t command;
    // PIDController pid;
    // PID_Init(&pid); // initialize PID with tuned params
    float controlOutput;
    float threshold = 10;

    while (1) {
        if (xQueueReceive(inputQueue, &receivedData, portMAX_DELAY) == pdPASS) {
            // controlOutput = PID_Compute(&pid, receivedData.desiredPositionPercent, receivedData.actualPositionPercent);
            controlOutput=receivedData.actualPositionPercent-receivedData.desiredPositionPercent;
            // Convert controlOutput to command for valves
            if(controlOutput > threshold) {
                command = CONTROL_INCREASE;
            } else if(controlOutput < -threshold) {
                command = CONTROL_DECREASE;
            } else {
                command = CONTROL_STOP;
            }
            xQueueSend(outputQueue, &command, portMAX_DELAY);
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
    gpio_set_direction(VALVE_INC_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(VALVE_DEC_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_direction(CURRENT_POSITION_PIN, GPIO_MODE_INPUT);
    // gpio_set_direction(INPUT_PERCENT_PIN, GPIO_MODE_INPUT);


    inputQueue = xQueueCreate(10, sizeof(ValvePositionData_t));
    outputQueue = xQueueCreate(10, sizeof(ControlCommand_t));

    configure_adc(ADC_ATTEN_DB_2_5);
    ESP_LOGI(TAG, "ADC configured");


    ESP_LOGI(TAG, "Starting fan control task...");
    xTaskCreate(fan_control_task, "Fan task", 2048, NULL, 5, NULL);
    xTaskCreate(input_task, "Input Task", 2048, NULL, 5, NULL);
    xTaskCreate(PID_task, "PID Task", 2048, NULL, 5, NULL);
    xTaskCreate(output_task, "Output Task", 2048, NULL, 5, NULL);



    // xTaskCreate(rpm_task, "rpm_task", 2048, NULL, 5, NULL);
}

