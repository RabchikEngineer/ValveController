#pragma once

#include "freertos/queue.h"

typedef enum {
    CAL_CMD_START_ACTUAL,
    CAL_CMD_START_DESIRED,
    CAL_CMD_START_CURRENT,
} cal_start_cmd_t;

typedef enum {
    CAL_CMD_NEXT,
    CAL_CMD_CANCEL,
    CAL_CMD_ACCEPT,
    CAL_CMD_REJECT,
} cal_control_cmd_t;

typedef enum {
    CAL_ST_IDLE,
    CAL_ST_STARTED,
    CAL_ST_DONE,
    CAL_ST_ERROR,
    CAL_ST_CANCELLED,
    CAL_ST_REJECTED,
    CAL_ST_WAIT_FOR_CONFIRM,
    CAL_AP_UPPER_LIMIT,
    CAL_AP_LOWER_LIMIT,
    CAL_DP_SET_VALUE,
    CAL_CL_NEEDS_VALUE,
} cal_status_t;

typedef struct {
    cal_status_t status;
    float value;
} cal_msg_t;


extern QueueHandle_t calibration_messages_queue;
extern QueueHandle_t calibration_start_queue;
extern QueueHandle_t calibration_commands_queue;


void calibration_init(QueueHandle_t set_position_queue);
void calibration_task();

// typedef int (*CalibrationFunc)(char*);

