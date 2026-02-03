#include "buttons.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "config.h"

static const char *TAG = "Buttons";

QueueHandle_t g_button_queue;

static void *s_pcf_handle = NULL;
static TaskHandle_t s_input_task = NULL;

static IRAM_ATTR void int_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(s_input_task, &hp);
    if (hp == pdTRUE) portYIELD_FROM_ISR();
}

static uint8_t read_pcf(void)
{
    uint8_t val = 0x00;
    i2c_master_receive((i2c_master_dev_handle_t)s_pcf_handle, &val, 1, 100);
    return val;
}

static void read_buttons_task(void *arg)
{
    uint8_t last_raw = 0xFF;
    uint8_t stable   = 0xFF;
    TickType_t press_time[8] = {0};  // for each bit
    bool long_fired[8] = {false};
    bool repeat_armed[8] = {false};
    TickType_t last_repeat[8] = {0};

    while (1) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)); // wake up on INT or every 50ms
        // ESP_LOGI(TAG, "Notifytake return %d",(int)ret); // uint32_t ret = 

        uint8_t raw = read_pcf();
        uint8_t pressed = (raw & 0x0F);

        // ESP_LOGI(TAG, "Buttons pressed: %d%d%d%d",(pressed & 0x0001),(pressed & 0x0002)>>1,(pressed & 0x0004)>>2,(pressed & 0x0008)>>3);
        

        // Debounce
        if (raw != last_raw) {
            last_raw = raw;
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            raw = read_pcf(); // debounce check even on one cycle
            if (raw!=last_raw) {continue;}
        }

        uint8_t changed = pressed ^ stable;
        TickType_t now = xTaskGetTickCount();

        for (int i = 0; i < 4; i++) {
            uint8_t mask = (1 << i);
            if (changed & mask) {
                if (pressed & mask) {
                    // PRESS
                    press_time[i] = now;
                    long_fired[i] = false;
                    repeat_armed[i] = false;
                    button_event_t ev = {.button = i, .event = BTN_EVENT_PRESS};
                    xQueueSend(g_button_queue, &ev, 0);
                } else {
                    // RELEASE
                    long_fired[i] = false;
                    repeat_armed[i] = false;
                    button_event_t ev = {.button = i, .event = BTN_EVENT_RELEASE};
                    xQueueSend(g_button_queue, &ev, 0);
                }
            } else if (pressed & mask) {
                // hold
                TickType_t hold_ms = pdTICKS_TO_MS(now - press_time[i]);
                if (!long_fired[i] && hold_ms >= LONG_PRESS_MS) {
                    button_event_t ev = {.button = i, .event = BTN_EVENT_LONG};
                    xQueueSend(g_button_queue, &ev, 0);
                    long_fired[i] = true;
                }
                if (!repeat_armed[i] && hold_ms >= REPEAT_START_MS) {
                    repeat_armed[i] = true;
                    last_repeat[i] = now;
                }
                if (repeat_armed[i] && pdTICKS_TO_MS(now - last_repeat[i]) >= REPEAT_INTERVAL_MS) {
                    button_event_t ev = {.button = i, .event = BTN_EVENT_REPEAT};
                    xQueueSend(g_button_queue, &ev, 0);
                    last_repeat[i] = now;
                }
            }
        }
        stable = pressed;
    }
}

void read_buttons_init(void *pcf_handle, int int_gpio)
{
    s_pcf_handle = pcf_handle;
    g_button_queue = xQueueCreate(16, sizeof(button_event_t));

    xTaskCreate(read_buttons_task, "input", 3072, NULL, 10, &s_input_task);

    // GPIO INT set up 
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << int_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_cfg);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(int_gpio, int_isr, NULL));

    ESP_LOGI(TAG, "Input initialized (INT GPIO %d)", int_gpio);
}
