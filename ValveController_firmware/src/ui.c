#include "ui.h"
#include "ui_render.h"
#include "buttons.h"
#include "params.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include "config.h"

static const char *TAG = "UI";

typedef enum {
    STATE_LOCKED,
    STATE_MENU_LIST,
    STATE_EDIT_VALUE
} ui_state_t;

typedef enum {
    MENU_KP = 0,
    MENU_KI,
    MENU_KD,
    MENU_DEADZONE,
    MENU_PWM_FREQ,
    MENU_EMA_ALPHA,
    MENU_COUNT
} menu_item_t;

static const char *menu_labels[MENU_COUNT] = {
    "Kp", "Ki", "Kd", "Deadzone", "PWM Freq", "EMA Alpha"
};

static ui_state_t s_state = STATE_LOCKED;
static int s_menu_cursor = 0;
static menu_item_t s_edit_item;
static float s_edit_float_backup;
static uint32_t s_edit_uint_backup;

static float *s_current_position = NULL;
static int64_t s_last_activity_us = 0;
static int64_t s_lock_hold_start_us = 0;
static bool s_ok_down = false;
static bool s_back_down = false;

static void mark_activity(void) { s_last_activity_us = esp_timer_get_time(); }

static void enter_locked(void)
{
    s_state = STATE_LOCKED;
    s_ok_down = s_back_down = false;
    s_lock_hold_start_us = 0;
    ESP_LOGI(TAG, "→ LOCKED");
}

static void enter_menu(void)
{
    s_state = STATE_MENU_LIST;
    s_menu_cursor = 0;
    mark_activity();
    ESP_LOGI(TAG, "→ MENU_LIST");
}

static void enter_edit(menu_item_t item)
{
    s_state = STATE_EDIT_VALUE;
    s_edit_item = item;
    mark_activity();

    // backup текущего значения
    switch (item) {
        case MENU_KP:        s_edit_float_backup = g_params.kp; break;
        case MENU_KI:        s_edit_float_backup = g_params.ki; break;
        case MENU_KD:        s_edit_float_backup = g_params.kd; break;
        case MENU_DEADZONE:  s_edit_float_backup = g_params.deadzone; break;
        case MENU_PWM_FREQ:  s_edit_uint_backup  = g_params.pwm_freq_hz; break;
        case MENU_EMA_ALPHA: s_edit_float_backup = g_params.ema_alpha; break;
        default: break;
    }
    ESP_LOGI(TAG, "→ EDIT_VALUE (%s)", menu_labels[item]);
}

static void modify_value(int delta)
{
    switch (s_edit_item) {
        case MENU_KP:
            params_set_kp(g_params.kp + delta * 0.1f);
            break;
        case MENU_KI:
            params_set_ki(g_params.ki + delta * 0.1f);
            break;
        case MENU_KD:
            params_set_kd(g_params.kd + delta * 0.1f);
            break;
        case MENU_DEADZONE:
            params_set_deadzone(g_params.deadzone + delta * 0.01f);
            break;
        case MENU_PWM_FREQ:
            params_set_pwm_freq((int)g_params.pwm_freq_hz + delta);
            break;
        case MENU_EMA_ALPHA:
            params_set_ema_alpha(g_params.ema_alpha + delta * 0.01f);
            break;
        default: break;
    }
}

static void cancel_edit(void)
{
    // restore backup
    switch (s_edit_item) {
        case MENU_KP:        g_params.kp = s_edit_float_backup; break;
        case MENU_KI:        g_params.ki = s_edit_float_backup; break;
        case MENU_KD:        g_params.kd = s_edit_float_backup; break;
        case MENU_DEADZONE:  g_params.deadzone = s_edit_float_backup; break;
        case MENU_PWM_FREQ:  g_params.pwm_freq_hz = s_edit_uint_backup; break;
        case MENU_EMA_ALPHA: g_params.ema_alpha = s_edit_float_backup; break;
        default: break;
    }
    s_state = STATE_MENU_LIST;
    ESP_LOGI(TAG, "Edit cancelled → MENU_LIST");
}

static void save_edit(void)
{
    s_state = STATE_MENU_LIST;
    ESP_LOGI(TAG, "Edit saved → MENU_LIST");
}

static void handle_event_locked(button_event_t *ev)
{
    // Комбинация OK+BACK hold 2s для разблокировки
    if (ev->button == BTN_OK) {
        if (ev->event == BTN_EVENT_PRESS) s_ok_down = true;
        else if (ev->event == BTN_EVENT_RELEASE) s_ok_down = false;
    }
    if (ev->button == BTN_BACK) {
        if (ev->event == BTN_EVENT_PRESS) s_back_down = true;
        else if (ev->event == BTN_EVENT_RELEASE) s_back_down = false;
    }
    
    if (s_ok_down && s_back_down) {
        if (s_lock_hold_start_us == 0) {
            s_lock_hold_start_us = esp_timer_get_time();
        } else {
            int64_t hold_us = esp_timer_get_time() - s_lock_hold_start_us;
            if (hold_us >= LOCK_COMBO_TIME_US) {
                enter_menu();
                s_lock_hold_start_us = 0;
            }
        }
    } else {
        s_lock_hold_start_us = 0;
    }
}

static void handle_event_menu(button_event_t *ev)
{
    if (ev->event == BTN_EVENT_PRESS || ev->event == BTN_EVENT_REPEAT || ev->event == BTN_EVENT_LONG) {
        mark_activity();
        if (ev->button == BTN_UP && ev->event != BTN_EVENT_LONG) {
            s_menu_cursor = (s_menu_cursor > 0) ? s_menu_cursor - 1 : MENU_COUNT - 1;
        } else if (ev->button == BTN_DOWN && ev->event != BTN_EVENT_LONG) {
            s_menu_cursor = (s_menu_cursor < MENU_COUNT - 1) ? s_menu_cursor + 1 : 0;
        } else if (ev->button == BTN_OK && ev->event == BTN_EVENT_PRESS) {
            enter_edit(s_menu_cursor);
        } else if (ev->button == BTN_BACK && ev->event == BTN_EVENT_LONG) {
            params_save();
            enter_locked();
        }
    }
}

static void handle_event_edit(button_event_t *ev)
{
    if (ev->event == BTN_EVENT_PRESS || ev->event == BTN_EVENT_REPEAT || ev->event == BTN_EVENT_LONG) {
        mark_activity();
        if (ev->button == BTN_UP) {
            modify_value(+1);
        } else if (ev->button == BTN_DOWN) {
            modify_value(-1);
        } else if (ev->button == BTN_OK && ev->event == BTN_EVENT_LONG) {
            save_edit();
        } else if (ev->button == BTN_BACK && ev->event == BTN_EVENT_LONG) {
            cancel_edit();
        }
    }
}

static void render_locked(void)
{
    render_clear();
    
    // Крупным шрифтом позиция
    char buf[32];
    snprintf(buf, sizeof(buf), "Pos: %.2f", *s_current_position);
    render_text_large(2, 10, buf);

    // Подсказка мелким
    render_text(6, 0, "Hold OK+BACK 2s", false);
    render_text(7, 0, "to unlock", false);

    render_flush();
}

static void render_menu(void)
{
    render_clear();
    render_text(0, 0, "=== MENU ===", false);

    int start_idx = (s_menu_cursor > 2) ? s_menu_cursor - 2 : 0;
    int visible = 5;
    for (int i = 0; i < visible && (start_idx + i) < MENU_COUNT; i++) {
        int idx = start_idx + i;
        bool selected = (idx == s_menu_cursor);
        char line[32];
        
        // Формат: "  Label: value"
        switch (idx) {
            case MENU_KP:        snprintf(line, sizeof(line), "  Kp: %.2f", g_params.kp); break;
            case MENU_KI:        snprintf(line, sizeof(line), "  Ki: %.2f", g_params.ki); break;
            case MENU_KD:        snprintf(line, sizeof(line), "  Kd: %.2f", g_params.kd); break;
            case MENU_DEADZONE:  snprintf(line, sizeof(line), "  DZ: %.3f", g_params.deadzone); break;
            case MENU_PWM_FREQ:  snprintf(line, sizeof(line), "  PWM: %luHz", (unsigned long)g_params.pwm_freq_hz); break;
            case MENU_EMA_ALPHA: snprintf(line, sizeof(line), "  EMA: %.2f", g_params.ema_alpha); break;
            default: snprintf(line, sizeof(line), "  ???"); break;
        }

        if (selected) line[0] = '>'; // курсор
        render_text(1 + i, 0, line, selected);
    }

    render_text(7, 0, "OK=Edit BACK=Exit", false);
    render_flush();
}

static void render_edit(void)
{
    render_clear();
    render_text(0, 0, "EDIT:", false);
    render_text(1, 0, menu_labels[s_edit_item], false);

    char val_str[32];
    switch (s_edit_item) {
        case MENU_KP:        snprintf(val_str, sizeof(val_str), "%.2f", g_params.kp); break;
        case MENU_KI:        snprintf(val_str, sizeof(val_str), "%.2f", g_params.ki); break;
        case MENU_KD:        snprintf(val_str, sizeof(val_str), "%.2f", g_params.kd); break;
        case MENU_DEADZONE:  snprintf(val_str, sizeof(val_str), "%.3f", g_params.deadzone); break;
        case MENU_PWM_FREQ:  snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)g_params.pwm_freq_hz); break;
        case MENU_EMA_ALPHA: snprintf(val_str, sizeof(val_str), "%.2f", g_params.ema_alpha); break;
        default: snprintf(val_str, sizeof(val_str), "???"); break;
    }
    render_text_large(3, 20, val_str);

    render_text(6, 0, "UP/DN=Change", false);
    render_text(7, 0, "OK=Save BACK=Cancel", false);
    render_flush();
}

void ui_init(void *lcd_panel_handle, float *current_position_ptr)
{
    s_current_position = current_position_ptr;
    render_init(lcd_panel_handle);
    s_state = STATE_LOCKED;
    s_last_activity_us = esp_timer_get_time();
    ESP_LOGI(TAG, "UI initialized");
}

void ui_run(void)
{
    button_event_t ev;
    TickType_t last_render = 0;

    while (1) {
        // Обработка событий кнопок (неблокирующая проверка)
        while (xQueueReceive(g_button_queue, &ev, 0) == pdTRUE) {
            // ESP_LOGI(TAG, "Received %d on %d", ev.event, ev.button);
            switch (s_state) {
                case STATE_LOCKED:    handle_event_locked(&ev); break;
                case STATE_MENU_LIST: handle_event_menu(&ev); break;
                case STATE_EDIT_VALUE: handle_event_edit(&ev); break;
            }
            // render_clear();
            // render_flush();
        }

        // Проверка таймаута неактивности (только в меню/edit)
        if (s_state != STATE_LOCKED) {
            int64_t idle_us = esp_timer_get_time() - s_last_activity_us;
            if (idle_us >= ACTIVITY_TIMEOUT_US) {
                params_save();
                enter_locked();
            }
        }

        // Перерисовка (не чаще 10 Гц, чтобы не забивать I2C)
        TickType_t now = xTaskGetTickCount();
        if (pdTICKS_TO_MS(now - last_render) >= 100) {
            switch (s_state) {
                case STATE_LOCKED:    render_locked(); break;
                case STATE_MENU_LIST: render_menu(); break;
                case STATE_EDIT_VALUE: render_edit(); break;
            }
            last_render = now;
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Гц цикл событий
        // ESP_LOGI(TAG,"ui cycle");
    }
}
