
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "math.h"

#include "params.h"
#include "calibration.h"
#include "adc.h"
#include "usb_comm.h"
#include "ui.h"
#include "dac.h"
#include "main.h"

// static const char* TAG = "Calibrator";
static const char *TAG = "Calibration";

static QueueHandle_t s_set_position_queue;
static QueueHandle_t s_set_cl_queue;
// static cal_msg_t s_message;

QueueHandle_t calibration_messages_queue;
QueueHandle_t calibration_start_queue;
QueueHandle_t calibration_commands_queue;



static void cal_send_status(cal_status_t type, float value)
{
    cal_msg_t st = {
        .status = type,
        .value = value,
    };
    // snprintf(st.text, sizeof(st.text), "%s", text ? text : "");
    xQueueSend(calibration_messages_queue, &st, 100);
}



bool usb_request_float(float *out_value_ptr, TickType_t timeout)
{
    usb_value_request_t req = USB_GET_FLOAT_VALUE;

    if (xQueueSend(s_usb_float_req_q, &req, 200) != pdTRUE) {
        return false;
    }

    if (xQueueReceive(s_usb_float_resp_q, out_value_ptr, timeout) != pdTRUE) {
        return false;
    }

    return true;
}


void print_polyline(approx_poly_line_t line) {
    
    ESP_LOGI(TAG, "--------------PolyLine--------------");

    for (int i=0;i<line.num_points;i++) {


        ESP_LOGI(TAG, "%.2f,%.2f", line.points[i].x, line.points[i].y);

    }

    ESP_LOGI(TAG, "--------------End--------------");
}


void calibration_task() {

    ESP_LOGI(TAG, "calibration_task started");

    cal_start_cmd_t start_cmd;
    cal_control_cmd_t ctrl_cmd;

    while (1) {
        if (xQueueReceive(calibration_start_queue, &start_cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        
        cal_send_status(CAL_ST_STARTED, 0);

        ESP_LOGI(TAG, "Calibration started");
        approx_poly_line_t calibration_points;
        float desired_value;

        switch (start_cmd) {
            case CAL_CMD_START_ACTUAL: {

                float pid_output_override, actual_position;
                calibration_points=(approx_poly_line_t){.num_points=2, .points={}};

                // 1: Test borders and make 2-point calibration
                suspend_polling();
                vTaskDelay(pdMS_TO_TICKS(VALVE_IDLE_DELAY*2));


                // text = "Calibrating upper limit";
                // Move to the upper position
                pid_output_override=1;
                cal_send_status(CAL_AP_UPPER_LIMIT, 0);
                xQueueSend(s_set_position_queue,&pid_output_override, portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(5000)); // wait for 5 seconds to stabilize 
                actual_position = get_raw_adc_values().actualPositionPercent;
                calibration_points.points[0]=(data_point_t){actual_position, 1}; // assuming that valve is in upper position
                
                // text = "Calibrating lower limit";
                pid_output_override=-1;
                cal_send_status(CAL_AP_LOWER_LIMIT, 0);
                xQueueSend(s_set_position_queue,&pid_output_override, portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(5000));
                actual_position = get_raw_adc_values().actualPositionPercent;
                calibration_points.points[1]=(data_point_t){actual_position, 0}; 

                print_polyline(calibration_points);

                qsort(calibration_points.points, calibration_points.num_points, sizeof(data_point_t), compare_data_points);

                print_polyline(calibration_points);

                resume_polling();
                
                if (fabs(calibration_points.points[0].x-calibration_points.points[calibration_points.num_points-1].x)<CAL_MIN_AP_DELTA) {
                    ESP_LOGW(TAG, "Calibration incorrect, canceling");
                    cal_send_status(CAL_ST_ERROR, 0);
                    break;
                }


                cal_send_status(CAL_ST_WAIT_FOR_CONFIRM, 0);
                if (xQueueReceive(calibration_commands_queue, &ctrl_cmd, pdMS_TO_TICKS(CAL_CONFIRM_WAIT_TIME_MS)) != pdTRUE) {
                    ESP_LOGW(TAG, "Confirmation timeout");
                    cal_send_status(CAL_ST_ERROR, 0);
                    break;
                }
                ESP_LOGI(TAG, "Received accept response");

                if (ctrl_cmd==CAL_CMD_ACCEPT) {
                    calibration_set_actual_position(calibration_points);
                    calibration_save();
                    // ui_send_custom_notification("Calib success", 3, 100);
                    cal_send_status(CAL_ST_DONE, 0);
                    ESP_LOGI(TAG, "Calibration accepted");
                } else {
                    cal_send_status(CAL_ST_REJECTED, 0);
                    ESP_LOGI(TAG, "Calibration rejected");
                    // ui_send_custom_notification("Calib rejected", 3, 100);
                }
                
                
                ESP_LOGI(TAG, "calibrate_actual_position ended");
                break;
            }

            case CAL_CMD_START_DESIRED: {
                
                float desired_position;
                calibration_points=(approx_poly_line_t){.num_points=2, .points={}};

                suspend_polling();
                vTaskDelay(pdMS_TO_TICKS(VALVE_IDLE_DELAY*2));
                
                desired_value=0;
                cal_send_status(CAL_DP_SET_VALUE, desired_value);
                if (xQueueReceive(calibration_commands_queue, &ctrl_cmd, pdMS_TO_TICKS(CAL_DP_SET_VALUE_WAIT_TIME_MS)) != pdTRUE) {
                    ESP_LOGW(TAG, "Confirmation timeout");
                    cal_send_status(CAL_ST_ERROR, 0);
                    break;
                }
                if (ctrl_cmd==CAL_CMD_CANCEL) {
                    cal_send_status(CAL_ST_CANCELLED, 0);
                    break;
                }
                desired_position = get_raw_adc_values().desiredPositionPercent;
                calibration_points.points[0]=(data_point_t){desired_position, desired_value};

                desired_value=1;
                cal_send_status(CAL_DP_SET_VALUE, desired_value);
                if (xQueueReceive(calibration_commands_queue, &ctrl_cmd, pdMS_TO_TICKS(CAL_DP_SET_VALUE_WAIT_TIME_MS)) != pdTRUE) {
                    ESP_LOGW(TAG, "Confirmation timeout");
                    cal_send_status(CAL_ST_ERROR, 0);
                    resume_polling();
                    break;
                }
                if (ctrl_cmd==CAL_CMD_CANCEL) {
                    cal_send_status(CAL_ST_CANCELLED, 0);
                    resume_polling();
                    break;
                }
                desired_position = get_raw_adc_values().desiredPositionPercent;
                calibration_points.points[1]=(data_point_t){desired_position, desired_value};

                resume_polling();
                
                print_polyline(calibration_points);

                qsort(calibration_points.points, calibration_points.num_points, sizeof(data_point_t), compare_data_points);

                print_polyline(calibration_points);
                
                if (fabs(calibration_points.points[0].x-calibration_points.points[calibration_points.num_points-1].x)<CAL_MIN_DP_DELTA) {
                    ESP_LOGW(TAG, "Calibration incorrect, canceling");
                    cal_send_status(CAL_ST_ERROR, 0);
                    break;
                }
                
                cal_send_status(CAL_ST_WAIT_FOR_CONFIRM, 0);
                if (xQueueReceive(calibration_commands_queue, &ctrl_cmd, pdMS_TO_TICKS(CAL_CONFIRM_WAIT_TIME_MS)) != pdTRUE) {
                    ESP_LOGW(TAG, "Confirmation timeout");
                    cal_send_status(CAL_ST_ERROR, 0);
                    break;
                }
                ESP_LOGI(TAG, "Received accept response");

                if (ctrl_cmd==CAL_CMD_ACCEPT) {
                    calibration_set_desired_position(calibration_points);
                    calibration_save();
                    cal_send_status(CAL_ST_DONE, 0);
                    ESP_LOGI(TAG, "Calibration accepted");
                } else {
                    cal_send_status(CAL_ST_REJECTED, 0);
                    ESP_LOGI(TAG, "Calibration rejected");
                }


                ESP_LOGI(TAG, "calibrate_desired_position ended");
                break;
            }
            case CAL_CMD_START_CURRENT: {

                float measured_input;
                cl_set_output cl_val;
                calibration_points=(approx_poly_line_t){.num_points=2, .points={}};
                
                suspend_polling();
                vTaskDelay(pdMS_TO_TICKS(VALVE_IDLE_DELAY*2));

                
                desired_value=0;
                cl_val=(cl_set_output){.value=desired_value, .is_raw=true};
                cal_send_status(CAL_CL_NEEDS_VALUE, 0);
                xQueueSend(s_set_cl_queue, &cl_val, pdMS_TO_TICKS(100));
                if (!usb_request_float(&measured_input, pdMS_TO_TICKS(CAL_CL_GET_VALUE_WAIT_TIME_MS))) {
                    cal_send_status(CAL_ST_ERROR, 0);
                    resume_polling();
                    break;
                }
                calibration_points.points[0]=(data_point_t){(measured_input-4.0)/16.0, desired_value};

                desired_value=4095;
                cl_val=(cl_set_output){.value=desired_value, .is_raw=true};
                cal_send_status(CAL_CL_NEEDS_VALUE, 0);
                xQueueSend(s_set_cl_queue, &cl_val, pdMS_TO_TICKS(100));
                if (!usb_request_float(&measured_input, pdMS_TO_TICKS(CAL_CL_GET_VALUE_WAIT_TIME_MS))) {
                    cal_send_status(CAL_ST_ERROR, 0);
                    resume_polling();
                    break;
                }
                calibration_points.points[1]=(data_point_t){(measured_input-4.0)/16.0, desired_value};

                resume_polling();


                print_polyline(calibration_points);

                qsort(calibration_points.points, calibration_points.num_points, sizeof(data_point_t), compare_data_points);

                print_polyline(calibration_points);
                
                if (fabs(calibration_points.points[0].y-calibration_points.points[calibration_points.num_points-1].y)<CAL_MIN_CL_DELTA) {
                    ESP_LOGW(TAG, "Calibration incorrect, canceling");
                    cal_send_status(CAL_ST_ERROR, 0);
                    break;
                }
                
                cal_send_status(CAL_ST_WAIT_FOR_CONFIRM, 0);
                if (xQueueReceive(calibration_commands_queue, &ctrl_cmd, pdMS_TO_TICKS(CAL_CONFIRM_WAIT_TIME_MS)) != pdTRUE) {
                    ESP_LOGW(TAG, "Confirmation timeout");
                    cal_send_status(CAL_ST_ERROR, 0);
                    break;
                }
                ESP_LOGI(TAG, "Received accept response");

                if (ctrl_cmd==CAL_CMD_ACCEPT) {
                    calibration_set_current_loop(calibration_points);
                    calibration_save();
                    cal_send_status(CAL_ST_DONE, 0);
                    ESP_LOGI(TAG, "Calibration accepted");
                } else {
                    cal_send_status(CAL_ST_REJECTED, 0);
                    ESP_LOGI(TAG, "Calibration rejected");
                }
                

                ESP_LOGI(TAG, "calibrate_current_loop ended");
                break;
            }

            default: ESP_LOGW(TAG, "Not impemented yet");
        }

    }

}


void calibration_init(QueueHandle_t set_position_queue) {

    s_set_position_queue = set_position_queue;
    s_set_cl_queue = current_loop_queue;

    calibration_messages_queue = xQueueCreate(2, sizeof(cal_msg_t));
    calibration_start_queue = xQueueCreate(2, sizeof(cal_start_cmd_t));
    calibration_commands_queue = xQueueCreate(2, sizeof(cal_control_cmd_t));
    
    ESP_LOGI(TAG, "Calibration initialized");

}


// void run_calibration_task(TaskFunction_t func) {

//     xTaskCreate(func, "Calibration task", 2048, NULL, 8, NULL);
//     ESP_LOGI(TAG, "Calibration task started"); 

// }











