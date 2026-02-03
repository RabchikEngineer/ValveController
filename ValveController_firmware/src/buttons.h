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

// Очередь событий кнопок (создаётся в read_buttons_init)
extern QueueHandle_t g_button_queue;

// pcf_handle - из твоего main.c; int_gpio - GPIO10
void read_buttons_init(void *pcf_handle, int int_gpio);
