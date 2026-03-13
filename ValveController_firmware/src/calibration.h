#pragma once


void calibration_init(TaskHandle_t polling_task_handler, QueueHandle_t set_position_queue);

void calibrate_actual_position(char* text);
void run_calibration_task(TaskFunction_t func);

// typedef int (*CalibrationFunc)(char*);


typedef enum {
    CAL_CMD_START_ACTUAL,
    CAL_CMD_START_DESIRED,
    CAL_CMD_START_CURRENT,
    CAL_CMD_CANCEL
} cal_cmd_t;

typedef enum {
    CAL_ST_IDLE,
    CAL_ST_STARTED,
    CAL_ST_STEP_UPPER,
    CAL_ST_STEP_LOWER,
    CAL_ST_STEP_INPUT_LOW,
    CAL_ST_STEP_INPUT_HIGH,
    CAL_ST_STEP_OUTPUT_LOW,
    CAL_ST_STEP_OUTPUT_HIGH,
    CAL_ST_DONE,
    CAL_ST_ERROR,
    CAL_ST_CANCELLED
} cal_status_type_t;