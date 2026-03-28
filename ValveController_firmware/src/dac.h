#pragma once

#include "freertos/queue.h"
#include "driver/i2c_master.h"

extern QueueHandle_t current_loop_queue;

typedef struct {
    float value;
    bool is_raw;
} cl_set_output;


void current_loop_output_task();
void current_loop_init(i2c_master_dev_handle_t dac_handle);


