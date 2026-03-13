#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    BTN_UP     = 1,
    BTN_DOWN   = 2,
    BTN_OK     = 0,
    BTN_BACK   = 3
} button_mask_t;

typedef enum {
    BTN_EVENT_PRESS,
    BTN_EVENT_RELEASE,
    BTN_EVENT_LONG,
    BTN_EVENT_REPEAT  
} button_event_type_t;

typedef struct {
    button_mask_t button;
    button_event_type_t event;
} button_event_t;


extern QueueHandle_t g_button_queue;

TaskHandle_t* button_reader_init(void *pcf_handle);
void button_reader_install_isr(int int_gpio);
void read_buttons_task();