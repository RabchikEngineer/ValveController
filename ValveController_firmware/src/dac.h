#pragma once

#include "freertos/queue.h"

extern QueueHandle_t current_loop_queue;


void current_loop_output_task();
void current_loop_init(i2c_master_dev_handle_t dac_handle);


