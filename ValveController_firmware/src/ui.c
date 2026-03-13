#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "ui.h"
#include "ui_render.h"
#include "buttons.h"
#include "params.h"
#include "adc.h"
#include "calibration.h"

static const char *TAG = "UI";

typedef enum {
    STATE_LOCKED,
    STATE_MAIN_MENU,
    STATE_CONFIG_MENU,
    STATE_CALIBRATION_MENU,
    STATE_CALIBRATION,
    STATE_EDIT_VALUE
} ui_state_t;

typedef enum {
    ENTER_CONFIG_MENU,
    ENTER_CALIBRATION_MENU,
    MAIN_MENU_COUNT
} main_menu_item_t;

typedef enum {
    CONFIG_KP = 0,
    CONFIG_KI,
    CONFIG_KD,
    CONFIG_DEADZONE,
    CONFIG_PWM_FREQ,
    CONFIG_EMA_ALPHA,
    CONFIG_MENU_COUNT
} config_menu_item_t;

typedef enum {
    CALIBRATE_ACTUAL_POSITION,
    CALIBRATE_DESIRED_POSITION,
    CALIBRATE_CURRENT_LOOP_OUTPUT,
    CALIBRATION_MENU_COUNT
} calibration_menu_item_t;


static const char *main_menu_labels[MAIN_MENU_COUNT] = {
    "Edit config", "Calibrate"
};
static const char *config_menu_labels[CONFIG_MENU_COUNT] = {
    "Kp", "Ki", "Kd", "Deadzone", "PWM Freq", "EMA Alpha"
};
static const char *calibration_menu_labels[CALIBRATION_MENU_COUNT] = {
    "Internal calibration", "Calibrate input signal", "Calibrate output signal"
};

static ui_state_t s_state = STATE_LOCKED;
static int s_menu_cursor = 0;
static config_menu_item_t s_edit_item;
static float s_edit_float_backup;
static uint32_t s_edit_uint_backup;
static calibration_menu_item_t s_calibration_type;

char* displayed_text_large="";
char* displayed_text_small="";

static float s_current_position;
static int64_t s_last_activity_us = 0;
static int64_t s_lock_hold_start_us = 0;
static bool s_ok_down = false;
static bool s_back_down = false;

static void mark_activity(void) { s_last_activity_us = esp_timer_get_time(); }


static void enter_state(ui_state_t state)
{   
    s_state = state;
    mark_activity();
    switch (state) {

        case (STATE_MAIN_MENU): {
            s_menu_cursor = 0;
            ESP_LOGI(TAG, "State → MAIN_MENU");
            break;
        }
        case (STATE_CONFIG_MENU): {
            s_menu_cursor = 0;
            ESP_LOGI(TAG, "State → CONFIG_MENU");
            break;
        }
        case (STATE_CALIBRATION_MENU): {
            s_menu_cursor = 0;
            ESP_LOGI(TAG, "State → CALIBRATION_MENU");
            break;
        }
        case (STATE_LOCKED): {
            s_ok_down = s_back_down = false;
            s_lock_hold_start_us = 0;
            ESP_LOGI(TAG, "State → LOCKED");
            break;
        }
        case (STATE_CALIBRATION): {
            ESP_LOGI(TAG, "State → CALIBRARTION");
            // switch (s_calibration_type) {
            //     case (CALIBRATE_ACTUAL_POSITION): run_calibration_task((TaskFunction_t)calibrate_actual_position); break;
            //     default: {ESP_LOGI(TAG,"Calibration not implemented"); state=s_state;}
            // }
            break;
        }
        default: ESP_LOGW(TAG, "Unexpected argument in enter_state"); break;
    }
    


    
}

static void enter_edit(config_menu_item_t item)
{
    s_state = STATE_EDIT_VALUE;
    s_edit_item = item;
    mark_activity();

    // backup текущего значения
    switch (item) {
        case CONFIG_KP:        s_edit_float_backup = g_params.kp; break;
        case CONFIG_KI:        s_edit_float_backup = g_params.ki; break;
        case CONFIG_KD:        s_edit_float_backup = g_params.kd; break;
        case CONFIG_DEADZONE:  s_edit_float_backup = g_params.deadzone; break;
        case CONFIG_PWM_FREQ:  s_edit_uint_backup  = g_params.pwm_freq_hz; break;
        case CONFIG_EMA_ALPHA: s_edit_float_backup = g_params.ema_alpha; break;
        default: break;
    }
    ESP_LOGI(TAG, "→ EDIT_VALUE (%s)", config_menu_labels[item]);
}

static void modify_value(int delta)
{
    switch (s_edit_item) {
        case CONFIG_KP:
            config_set_kp(g_params.kp + delta * 0.1f);
            break;
        case CONFIG_KI:
            config_set_ki(g_params.ki + delta * 0.1f);
            break;
        case CONFIG_KD:
            config_set_kd(g_params.kd + delta * 0.1f);
            break;
        case CONFIG_DEADZONE:
            config_set_deadzone(g_params.deadzone + delta * 0.01f);
            break;
        case CONFIG_PWM_FREQ:
            config_set_pwm_freq((int)g_params.pwm_freq_hz + delta);
            break;
        case CONFIG_EMA_ALPHA:
            config_set_ema_alpha(g_params.ema_alpha + delta * 0.01f);
            break;
        default: break;
    }
}

static void cancel_edit(void)
{
    // restore backup
    switch (s_edit_item) {
        case CONFIG_KP:        g_params.kp = s_edit_float_backup; break;
        case CONFIG_KI:        g_params.ki = s_edit_float_backup; break;
        case CONFIG_KD:        g_params.kd = s_edit_float_backup; break;
        case CONFIG_DEADZONE:  g_params.deadzone = s_edit_float_backup; break;
        case CONFIG_PWM_FREQ:  g_params.pwm_freq_hz = s_edit_uint_backup; break;
        case CONFIG_EMA_ALPHA: g_params.ema_alpha = s_edit_float_backup; break;
        default: break;
    }
    enter_state(STATE_CONFIG_MENU);
    ESP_LOGI(TAG, "Edit cancelled");
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
                enter_state(STATE_MAIN_MENU);
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

        switch (s_state) {

            case (STATE_MAIN_MENU): {
                if (ev->button == BTN_UP && ev->event != BTN_EVENT_LONG) {
                    s_menu_cursor = (s_menu_cursor > 0) ? s_menu_cursor - 1 : MAIN_MENU_COUNT - 1;
                } else if (ev->button == BTN_DOWN && ev->event != BTN_EVENT_LONG) {
                    s_menu_cursor = (s_menu_cursor < MAIN_MENU_COUNT - 1) ? s_menu_cursor + 1 : 0;
                } else if (ev->button == BTN_OK && ev->event == BTN_EVENT_LONG) {
                    ui_state_t state;
                    switch ((main_menu_item_t)s_menu_cursor) {
                        case (ENTER_CONFIG_MENU): state=STATE_CONFIG_MENU; break;
                        case (ENTER_CALIBRATION_MENU): state=STATE_CALIBRATION_MENU; break;
                        default: state=STATE_MAIN_MENU; break;
                    }
                    enter_state(state);
                } else if (ev->button == BTN_BACK && ev->event == BTN_EVENT_LONG) {
                    enter_state(STATE_LOCKED);
                }
                break;
            }
            case (STATE_CONFIG_MENU): {
                if (ev->button == BTN_UP && ev->event != BTN_EVENT_LONG) {
                    s_menu_cursor = (s_menu_cursor > 0) ? s_menu_cursor - 1 : CONFIG_MENU_COUNT - 1;
                } else if (ev->button == BTN_DOWN && ev->event != BTN_EVENT_LONG) {
                    s_menu_cursor = (s_menu_cursor < CONFIG_MENU_COUNT - 1) ? s_menu_cursor + 1 : 0;
                } else if (ev->button == BTN_OK && ev->event == BTN_EVENT_LONG) {
                    enter_edit(s_menu_cursor);
                } else if (ev->button == BTN_BACK && ev->event == BTN_EVENT_LONG) {
                    config_save();
                    enter_state(STATE_MAIN_MENU);
                }
                break;
            }
            case (STATE_CALIBRATION_MENU): {
                if (ev->button == BTN_UP && ev->event != BTN_EVENT_LONG) {
                    s_menu_cursor = (s_menu_cursor > 0) ? s_menu_cursor - 1 : CALIBRATION_MENU_COUNT - 1;
                } else if (ev->button == BTN_DOWN && ev->event != BTN_EVENT_LONG) {
                    s_menu_cursor = (s_menu_cursor < CALIBRATION_MENU_COUNT - 1) ? s_menu_cursor + 1 : 0;
                } else if (ev->button == BTN_OK && ev->event == BTN_EVENT_LONG) {
                    // switch ((calibration_menu_item_t)s_menu_cursor) {
                    //     case (CALIBRATE_ACTUAL_POSITION): calibrate_actual_position(); break;
                    //     case (CALIBRATE_DESIRED_POSITION): calibrate_desired_position(); break;
                    //     case (CALIBRATE_CURRENT_LOOP_OUTPUT): calibrate_current_loop_output(); break;
                    //     default: break;
                    // }
                    s_calibration_type=(calibration_menu_item_t)s_menu_cursor;
                    enter_state(STATE_CALIBRATION);
                    calibrate_actual_position(displayed_text_large);
                    enter_state(STATE_CALIBRATION_MENU);
                } else if (ev->button == BTN_BACK && ev->event == BTN_EVENT_LONG) {
                    enter_state(STATE_MAIN_MENU);
                }
                break;
            }
            default: ESP_LOGW(TAG, "Unexpected argument in handle_event_menu"); break;

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
            ESP_LOGI(TAG, "Edit saved");
            enter_state(STATE_CONFIG_MENU);
        } else if (ev->button == BTN_BACK && ev->event == BTN_EVENT_LONG) {
            cancel_edit();
        }
    }
}

static void handle_event_calibration(button_event_t *ev) {

    ESP_LOGI(TAG, "In-calibration event handler");

}

static void render_locked(void)
{
    render_clear();

    s_current_position = get_calibrated_adc_values().actualPositionPercent;
    
    // Крупным шрифтом позиция
    char buf[32];
    snprintf(buf, sizeof(buf), "Pos: %.2f", s_current_position);
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
    char* bottom_text="";
    int items_count;
    switch (s_state) {
        case (STATE_MAIN_MENU): items_count=MAIN_MENU_COUNT; bottom_text="BACK=Exit OK=Enter"; break;
        case (STATE_CONFIG_MENU): items_count=CONFIG_MENU_COUNT; bottom_text="BACK=Exit OK=Edit"; break;
        case (STATE_CALIBRATION_MENU): items_count=CALIBRATION_MENU_COUNT; bottom_text="BACK=Exit OK=Start"; break;
        default: items_count=1; ESP_LOGW(TAG, "Unexpected state in render_menu"); break;
    }
    for (int i = 0; i < visible && (start_idx + i) < items_count; i++) {
        int idx = start_idx + i;
        bool selected = (idx == s_menu_cursor);
        char line[32];
        
        // Формат: "  Label: value"
        switch (s_state) {

            case STATE_MAIN_MENU: {
                switch (idx) {
                    case ENTER_CONFIG_MENU:        snprintf(line, sizeof(line), "  %s", main_menu_labels[0]); break;
                    case ENTER_CALIBRATION_MENU:   snprintf(line, sizeof(line), "  %s", main_menu_labels[1]); break;
                    default: snprintf(line, sizeof(line), "  ???"); break;
                }
                break;
            }

            case STATE_CONFIG_MENU: {
                switch (idx) {
                    case CONFIG_KP:        snprintf(line, sizeof(line), "  Kp: %.2f", g_params.kp); break;
                    case CONFIG_KI:        snprintf(line, sizeof(line), "  Ki: %.2f", g_params.ki); break;
                    case CONFIG_KD:        snprintf(line, sizeof(line), "  Kd: %.2f", g_params.kd); break;
                    case CONFIG_DEADZONE:  snprintf(line, sizeof(line), "  DZ: %.3f", g_params.deadzone); break;
                    case CONFIG_PWM_FREQ:  snprintf(line, sizeof(line), "  PWM: %luHz", (unsigned long)g_params.pwm_freq_hz); break;
                    case CONFIG_EMA_ALPHA: snprintf(line, sizeof(line), "  EMA: %.2f", g_params.ema_alpha); break;
                    default: snprintf(line, sizeof(line), "  ???"); break;
                }
                break;
            }

            case STATE_CALIBRATION_MENU: {
                switch (idx) {
                    case CALIBRATE_ACTUAL_POSITION:        snprintf(line, sizeof(line), "  %s", calibration_menu_labels[0]); break;
                    case CALIBRATE_DESIRED_POSITION:       snprintf(line, sizeof(line), "  %s", calibration_menu_labels[1]); break;
                    case CALIBRATE_CURRENT_LOOP_OUTPUT:    snprintf(line, sizeof(line), "  %s", calibration_menu_labels[2]); break;
                    default: snprintf(line, sizeof(line), "  ???"); break;
                }
                break;
            }

            default: {snprintf(line, sizeof(line), "??? State"); ESP_LOGW(TAG, "Unexpected state in render_menu");}
        } 
        

        if (selected) line[0] = '>'; // курсор
        render_text(1 + i, 0, line, selected);
    }

    render_text(7, 0, bottom_text, false);
    render_flush();
}

static void render_edit(void)
{
    render_clear();
    render_text(0, 0, "EDIT:", false);
    render_text(1, 0, config_menu_labels[s_edit_item], false);

    char val_str[32];
    switch (s_edit_item) {
        case CONFIG_KP:        snprintf(val_str, sizeof(val_str), "%.2f", g_params.kp); break;
        case CONFIG_KI:        snprintf(val_str, sizeof(val_str), "%.2f", g_params.ki); break;
        case CONFIG_KD:        snprintf(val_str, sizeof(val_str), "%.2f", g_params.kd); break;
        case CONFIG_DEADZONE:  snprintf(val_str, sizeof(val_str), "%.3f", g_params.deadzone); break;
        case CONFIG_PWM_FREQ:  snprintf(val_str, sizeof(val_str), "%lu", (unsigned long)g_params.pwm_freq_hz); break;
        case CONFIG_EMA_ALPHA: snprintf(val_str, sizeof(val_str), "%.2f", g_params.ema_alpha); break;
        default: snprintf(val_str, sizeof(val_str), "???"); break;
    }
    render_text_large(3, 20, val_str);

    render_text(6, 0, "UP/DN=Change", false);
    render_text(7, 0, "BACK=Cancel OK=Save", false);
    render_flush();
}

static void render_calibration(void) {
    render_clear();
    render_text(0,0,"Calibration:",0);
    render_text(2,5,displayed_text_large,0);
    // render_text(0, 0, "Points num: %s",num, false);
    render_flush();
    ESP_LOGI(TAG, "Rendered calibration");
}

void ui_init(void *lcd_panel_handle)
{
    render_init(lcd_panel_handle);
    s_state = STATE_LOCKED;
    s_last_activity_us = esp_timer_get_time();
    ESP_LOGI(TAG, "UI initialized");
}

void ui_run(void)
{
    button_event_t ev;
    TickType_t last_render = 0;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        // Обработка событий кнопок (неблокирующая проверка)
        while (xQueueReceive(g_button_queue, &ev, 0) == pdTRUE) {
            // ESP_LOGI(TAG, "Received %d on %d", ev.event, ev.button);
            switch (s_state) {
                case STATE_LOCKED:    handle_event_locked(&ev); break;
                case STATE_MAIN_MENU: 
                case STATE_CONFIG_MENU:
                case STATE_CALIBRATION_MENU: handle_event_menu(&ev); break;
                case STATE_EDIT_VALUE: handle_event_edit(&ev); break;
                case STATE_CALIBRATION: handle_event_calibration(&ev); break;
            }
            // render_clear();
            // render_flush();
        }

        // Inactivity timeout check
        if (s_state != STATE_LOCKED) {
            int64_t idle_us = esp_timer_get_time() - s_last_activity_us;
            if (idle_us >= ACTIVITY_TIMEOUT_US) {
                if (s_state==STATE_CONFIG_MENU || s_state==STATE_EDIT_VALUE) {
                    config_save();
                }
                enter_state(STATE_LOCKED);
            }
        }

        // Rendering
        TickType_t now = xTaskGetTickCount();
        if (pdTICKS_TO_MS(now - last_render) >= 1000/SCREEN_FPS) {
            switch (s_state) {
                case STATE_LOCKED:    render_locked(); break;
                case STATE_MAIN_MENU: 
                case STATE_CONFIG_MENU:
                case STATE_CALIBRATION_MENU: render_menu(); break;
                case STATE_EDIT_VALUE: render_edit(); break;
                case STATE_CALIBRATION: render_calibration(); break;
            }
            last_render = now;
        }

        xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(UI_CYCLE_DELAY)); 
        // vTaskDelay(pdMS_TO_TICKS(UI_CYCLE_DELAY));  // 50 Гц цикл событий
        // ESP_LOGI(TAG,"ui cycle");
    }
}
