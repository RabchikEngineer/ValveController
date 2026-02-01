#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include "driver/usb_serial_jtag.h"

#include "config.h"


QueueHandle_t usb_queue;
PIDConfig_t* pid_config;

static const char *TAG = "USB";


void usb_jtag_init() {

    // Install USB driver
    usb_serial_jtag_driver_config_t usb_cfg = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
}



void usb_input_task() {
    uint8_t buffer[256];
    char line_buffer[256];
    int line_pos = 0;
    
    ESP_LOGI(TAG, "Usb input task starting...");
    // printf(">>> \n");
    // fflush(stdout);
    
    while (1) {
        int len = usb_serial_jtag_read_bytes(buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = buffer[i];
                
                // Echo character
                // usb_serial_jtag_write_bytes((const char*)&c, 1, 0);
                
                if (c == '\n' || c == '\r') {
                    if (line_pos > 0) {
                        line_buffer[line_pos] = '\0';
                        
                        // Parse command
                        float kp, ki, kd;
                        if (sscanf(line_buffer, "%f %f %f", &kp, &ki, &kd) == 3) {
                            pid_config->kp = kp;
                            pid_config->ki = ki;
                            pid_config->kd = kd;
                            
                            ESP_LOGI(TAG, "Config received: Kp=%.4f Ki=%.4f Kd=%.4f", 
                                     kp, ki, kd);

                            // xQueueSend(usb_queue,&pid_config, portMAX_DELAY);
                            
                        } else {
                            ESP_LOGW(TAG, "Invalid input: %s", line_buffer);
                        }
                        
                        line_pos = 0;
                        // printf(">>> \n");
                        // fflush(stdout);
                    }
                } else if (c >= 32 && c < 127) {
                    if (line_pos < sizeof(line_buffer) - 1) {
                        line_buffer[line_pos++] = c;
                    }
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void usb_comm_init(PIDConfig_t* config) {

    // usb_queue = xQueueCreate(10, sizeof(PIDConfig_t));
    pid_config = config;

    usb_jtag_init();
    ESP_LOGI(TAG, "USB JTAG configured");


    // xTaskCreate(usb_input_task, "USB Comm Task", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "USB input started");

}


