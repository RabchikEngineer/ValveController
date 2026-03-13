
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "params.h"
#include "calibration.h"
#include "adc.h"

// static const char* TAG = "Calibrator";
static const char *TAG = "Main Thread";

static TaskHandle_t s_polling_task_handler;
static QueueHandle_t s_set_position_queue;




void calibrate_actual_position(char* text) {

    ESP_LOGI(TAG, "calibrate_actual_position started");

    float pid_output_override, actual_position;
    approx_poly_line_t calibration_points={.num_points=2, .points=(data_point_t[2]){}};

    // 1: Test borders and make 2-point calibration
    vTaskSuspend(s_polling_task_handler);

    text = "Calibrating upper limit";
    // Move to the upper position
    pid_output_override=1;
    xQueueSend(s_set_position_queue,&pid_output_override,portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(5000)); // wait for 5 seconds to stabilize 
    actual_position = get_raw_adc_values().actualPositionPercent;
    calibration_points.points[0]=(data_point_t){actual_position, 1}; // assuming that valve is in upper position
    
    text = "Calibrating lower limit";
    pid_output_override=0;
    xQueueSend(s_set_position_queue,&pid_output_override,portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(5000));
    actual_position = get_raw_adc_values().actualPositionPercent;
    calibration_points.points[1]=(data_point_t){actual_position, 0}; 


    vTaskResume(s_polling_task_handler);

    text="";
    ESP_LOGI(TAG, "%s",text);

    ESP_LOGI(TAG, "calibrate_actual_position ended");

    // vTaskDelete( NULL );

}


void calibration_init(TaskHandle_t polling_task_handler, QueueHandle_t set_position_queue) {

    s_polling_task_handler= polling_task_handler;
    s_set_position_queue = set_position_queue;
    
    ESP_LOGI(TAG, "Calibration initialized");

}


void run_calibration_task(TaskFunction_t func) {

    xTaskCreate(func, "Calibration task", 2048, NULL, 8, NULL);
    ESP_LOGI(TAG, "Calibration task started"); 

}


void test() {


}











