#pragma once

#include "freertos/queue.h"

// extern QueueHandle_t notifications_queue;

typedef struct {
    char text[50];
    float display_time;
} ui_notif_t;


void ui_init(void *lcd_panel_handle);
void ui_run(void);  // UI main loop (call as task)

void ui_send_custom_notification(const char* text, uint8_t display_time, uint16_t timeout_ms);
