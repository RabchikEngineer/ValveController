#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"

#include "config.h"
#include "params.h"
#include "usb_comm.h"
#include "calibration.h"
#include "ui.h"

QueueHandle_t usb_queue;

static const char *TAG = "USB";


QueueHandle_t s_usb_float_req_q;    // calibration -> usb
QueueHandle_t s_usb_float_resp_q;   // usb -> calibration

static usb_mode_t s_usb_mode = USB_MODE_NORMAL;


void usb_jtag_init() {

    // Install USB driver
    usb_serial_jtag_driver_config_t usb_cfg = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
}


static void process_usb_line(const char *line)
{
    
    if (s_usb_mode == USB_MODE_WAIT_FLOAT) {
        float value;
        if (sscanf(line, "%f", &value) == 1) {
            xQueueOverwrite(s_usb_float_resp_q, &value);
            ESP_LOGI(TAG, "Float value received: %.4f", value);
            s_usb_mode = USB_MODE_NORMAL;
        } else {
            ESP_LOGW(TAG, "Expected single float value");
        }
        return;
    }

    float kp, ki, kd;

    if (sscanf(line, "set-pid %f %f %f", &kp, &ki, &kd) == 3) {
        config_set_kp(kp);
        config_set_ki(ki);
        config_set_kd(kd);

        ESP_LOGI(TAG, "PID updated");
        printf("OK\n");
        fflush(stdout);
        return;
    }

    if (strcmp(line, "get-pid") == 0) {
        printf("Current PID: %.4f %.4f %.4f\n", g_config.kp, g_config.ki, g_config.kd);
        fflush(stdout);
        return;
    }

    if (strcmp(line, "actual-calib") == 0) {
        cal_start_cmd_t cmd = CAL_CMD_START_ACTUAL;
        xQueueSend(calibration_start_queue, &cmd, pdMS_TO_TICKS(100));
        return;
    }

    char notif_text[50]={};
    if (sscanf(line, "notif %49s", notif_text) == 1) {
        ui_send_custom_notification(notif_text, 1, 100);
        ESP_LOGI(TAG, "notif text: %s", notif_text);
        return;
    }

    ESP_LOGW(TAG, "Invalid input: %s", line);
    printf("ERR: unknown command\n");
    fflush(stdout);
}

void usb_input_task(void *arg)
{
    uint8_t buffer[256];
    char line_buffer[256];
    int line_pos = 0;

    ESP_LOGI(TAG, "USB input task started");

    while (1) {
        usb_value_request_t req;
        if (xQueueReceive(s_usb_float_req_q, &req, 0) == pdTRUE) {

            switch (req) {
                case USB_GET_FLOAT_VALUE:
                    s_usb_mode = USB_MODE_WAIT_FLOAT;
                    ESP_LOGI(TAG, "Write current value in mA (decimal delimiter is dot)");
                    break;
                case USB_GET_VALUE_CANCEL:
                    s_usb_mode = USB_MODE_NORMAL;
                    break;
                default:
                    ESP_LOGW(TAG, "Unsupported usb mode");
            }
            // fflush(stdout);

        }

        int len = usb_serial_jtag_read_bytes(buffer, sizeof(buffer), pdMS_TO_TICKS(100));

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)buffer[i];

                if (c == '\n' || c == '\r') {
                    if (line_pos > 0) {
                        line_buffer[line_pos] = '\0';
                        process_usb_line(line_buffer);
                        line_pos = 0;
                    }
                } else if (c >= 32 && c < 127) {
                    if (line_pos < (int)sizeof(line_buffer) - 1) {
                        line_buffer[line_pos++] = c;
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void usb_comm_init() {

    s_usb_float_req_q  = xQueueCreate(1, sizeof(usb_value_request_t));
    s_usb_float_resp_q = xQueueCreate(1, sizeof(float));

    usb_jtag_init();
    ESP_LOGI(TAG, "USB JTAG configured");


    // xTaskCreate(usb_input_task, "USB Comm Task", 4096, NULL, 10, NULL);
    // ESP_LOGI(TAG, "USB input started");

}


