
#ifndef USB_H
#define USB_H

#include "freertos/queue.h"

void usb_comm_init();
extern QueueHandle_t usb_queue;

#endif

