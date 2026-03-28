
#pragma once

#include "freertos/queue.h"


typedef enum {
    USB_MODE_NORMAL,
    USB_MODE_WAIT_FLOAT
} usb_mode_t;

typedef enum {
    USB_GET_VALUE_CANCEL,
    USB_GET_FLOAT_VALUE,
} usb_value_request_t;

extern QueueHandle_t s_usb_float_req_q;
extern QueueHandle_t s_usb_float_resp_q;

void usb_comm_init();
void usb_input_task();
// extern QueueHandle_t usb_queue;


