#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "config.h"
#include "ui.h"
#include "ui_render.h"
#include "buttons.h"
#include "params.h"
#include "adc.h"
#include "calibration.h"
#include "main.h"


static const char *TAG = "UI";

typedef enum {
    STATE_LOCKED,
    STATE_MAIN_MENU,
    STATE_CONFIG_MENU,
    STATE_CALIBRATION_MENU,
    STATE_CALIBRATION,
    STATE_EDIT_VALUE,
    STATE_CONFIRM_ACTION,
    STATE_MANUAL_CONTROL
} ui_state_t;

typedef enum {
    ENTER_CONFIG_MENU,
    ENTER_CALIBRATION_MENU,
    ENTER_MANUAL_CONTROL,
    VERSION_INFO,
    MAIN_MENU_COUNT
} main_menu_item_t;

typedef enum {
    CONFIG_KP = 0,
    CONFIG_KI,
    CONFIG_KD,
    CONFIG_DEADZONE,
    CONFIG_PWM_FREQ,
    CONFIG_EMA_ALPHA,
    CONFIG_RESET,
    CONFIG_MENU_COUNT
} config_menu_item_t;

typedef enum {
    CALIBRATE_ACTUAL_POSITION,
    CALIBRATE_DESIRED_POSITION,
    CALIBRATE_CURRENT_LOOP_OUTPUT,
    CALIBRATION_RESET,
    CALIBRATION_MENU_COUNT
} calibration_menu_item_t;


typedef enum {
    CONFIG_DATA,
    CALIBRATION_DATA,
} reset_item_t;


typedef enum {
    ACTION_RESET,
    ACTION_CONFIRM_CALIBRATION,
} user_action_t;


static const char *main_menu_labels[MAIN_MENU_COUNT] = {
    "Edit config", "Calibrate", "Manual control", "Version v1.1.2"
};
static const char *config_menu_labels[CONFIG_MENU_COUNT] = {
    "Kp", "Ki", "Kd", "Deadzone", "PWM Freq", "EMA Alpha"
};
static const char *calibration_menu_labels[CALIBRATION_MENU_COUNT] = {
    "Internal calibration", "Calib in 4-20mA", "Calib out 4-20mA", "Reset calibration"
};

static ui_state_t s_state = STATE_LOCKED;
static int s_menu_cursor = 0;
static config_menu_item_t s_edit_item;
static float s_edit_float_backup;
static uint32_t s_edit_uint_backup;
static calibration_menu_item_t s_calibration_type;
static int64_t s_last_activity_us = 0;
static int64_t s_lock_hold_start_us = 0;
static bool s_ok_down = false;
static bool s_back_down = false;

char* displayed_text_large="";
char* displayed_text_small="";
char s_calibration_text[RENDER_TEXT_BUF_SZ];

static ValvePositionData_t s_valve_positions;
static float s_manual_control_position=0;
static float s_stabilization_disabled=0;


// const struct notifications_conf_t {
//     data_point_t rect_coords[2];
//     data_point_t text_pos_rel;
//     int characters_in_row;
// } notifications_conf = {.rect_coords={{19,20},{109,40}}, .text_pos_rel={3,3}, .characters_in_row=14};


QueueHandle_t notifications_queue;


void ui_send_custom_notification(const char* text, uint8_t display_time, uint16_t timeout_ms) {

    ui_notif_t notif ={
        .display_time = display_time,
    };
    snprintf(notif.text, sizeof(notif.text), "%s", text);

    if (xQueueSend(notifications_queue, &notif, pdMS_TO_TICKS(timeout_ms))!=pdTRUE) {
        ESP_LOGW(TAG, "Notification queue is full");
    };

}


static void mark_activity(void) { s_last_activity_us = esp_timer_get_time(); }


static void enter_state(ui_state_t state)
{   
    if (s_state == STATE_MANUAL_CONTROL) {
        if (s_stabilization_disabled==true) {
            resume_polling();
            s_stabilization_disabled=false;
        }
    }

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
        case (STATE_MANUAL_CONTROL): {
            ESP_LOGI(TAG, "State → MANUAL");
            set_desired_position_override(true);
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
        case CONFIG_KP:        s_edit_float_backup = g_config.kp; break;
        case CONFIG_KI:        s_edit_float_backup = g_config.ki; break;
        case CONFIG_KD:        s_edit_float_backup = g_config.kd; break;
        case CONFIG_DEADZONE:  s_edit_float_backup = g_config.deadzone; break;
        case CONFIG_PWM_FREQ:  s_edit_uint_backup  = g_config.pwm_freq_hz; break;
        case CONFIG_EMA_ALPHA: s_edit_float_backup = g_config.ema_alpha; break;
        default: break;
    }
    ESP_LOGI(TAG, "→ EDIT_VALUE (%s)", config_menu_labels[item]);
}

static void modify_value(int delta)
{
    switch (s_edit_item) {
        case CONFIG_KP:
            config_set_kp(g_config.kp + delta * 0.1f);
            break;
        case CONFIG_KI:
            config_set_ki(g_config.ki + delta * 0.1f);
            break;
        case CONFIG_KD:
            config_set_kd(g_config.kd + delta * 0.1f);
            break;
        case CONFIG_DEADZONE:
            config_set_deadzone(g_config.deadzone + delta * 0.01f);
            break;
        case CONFIG_PWM_FREQ:
            config_set_pwm_freq((int)g_config.pwm_freq_hz + delta);
            break;
        case CONFIG_EMA_ALPHA:
            config_set_ema_alpha(g_config.ema_alpha + delta * 0.01f);
            break;
        default: break;
    }
}

static void cancel_edit()
{
    // restore backup
    switch (s_edit_item) {
        case CONFIG_KP:        g_config.kp = s_edit_float_backup; break;
        case CONFIG_KI:        g_config.ki = s_edit_float_backup; break;
        case CONFIG_KD:        g_config.kd = s_edit_float_backup; break;
        case CONFIG_DEADZONE:  g_config.deadzone = s_edit_float_backup; break;
        case CONFIG_PWM_FREQ:  g_config.pwm_freq_hz = s_edit_uint_backup; break;
        case CONFIG_EMA_ALPHA: g_config.ema_alpha = s_edit_float_backup; break;
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
                        case (ENTER_MANUAL_CONTROL): state=STATE_MANUAL_CONTROL; break;
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
                    switch (s_menu_cursor) {
                        case CONFIG_RESET:
                            reset_config();
                            ui_send_custom_notification("Resetted", 3, 100);
                            break;
                        default:
                            enter_edit(s_menu_cursor);
                    }
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
                    switch ((calibration_menu_item_t)s_menu_cursor) {
                        case CALIBRATE_ACTUAL_POSITION: 
                            s_calibration_type=CAL_CMD_START_ACTUAL;
                            break;
                        case CALIBRATE_DESIRED_POSITION:
                            s_calibration_type=CAL_CMD_START_DESIRED;
                            break;
                        case CALIBRATE_CURRENT_LOOP_OUTPUT:
                            s_calibration_type=CAL_CMD_START_CURRENT;
                            break;
                        case CALIBRATION_RESET:
                            reset_calibration();
                            ui_send_custom_notification("Resetted", 3, 100);
                            return;
                        default:
                            ESP_LOGW(TAG, "Unexpected value of s_menu_cursor in handle_event_menu"); break;

                    }
                    xQueueSend(calibration_start_queue, &s_calibration_type, 100);
                    
                } else if (ev->button == BTN_BACK && ev->event == BTN_EVENT_LONG) {
                    enter_state(STATE_MAIN_MENU);
                }
                break;
            }
            default: ESP_LOGW(TAG, "Unexpected value of s_state in handle_event_menu"); break;

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

    cal_control_cmd_t cmd;

    // ESP_LOGI(TAG, "In-calibration event handler");
    

    if (ev->event == BTN_EVENT_PRESS || ev->event == BTN_EVENT_REPEAT || ev->event == BTN_EVENT_LONG) {

        if (ev->button == BTN_BACK && ev->event==BTN_EVENT_LONG) {
            // ui_send_custom_notification("button pressed", 1, 100);
            cmd = CAL_CMD_CANCEL;
            xQueueSend(calibration_commands_queue, &cmd, pdMS_TO_TICKS(100));
        }

        if (ev->button == BTN_OK && ev->event==BTN_EVENT_LONG) {
            // ui_send_custom_notification("button pressed", 1, 100);
            cmd = CAL_CMD_NEXT;
            xQueueSend(calibration_commands_queue, &cmd, pdMS_TO_TICKS(100));
        }

    }

}


static void handle_event_confirm_action(button_event_t *ev) {

    cal_control_cmd_t cmd;

    // ESP_LOGI(TAG, "Confirm action event handler");

    if (ev->event == BTN_EVENT_LONG) {

        if (ev->button == BTN_BACK) {   
            ESP_LOGI(TAG, "sending reject command");
            cmd = CAL_CMD_REJECT;
            xQueueSend(calibration_commands_queue, &cmd, pdMS_TO_TICKS(100));

        } else if (ev->button == BTN_OK) {
            ESP_LOGI(TAG, "sending accept command");
            cmd = CAL_CMD_ACCEPT;
            xQueueSend(calibration_commands_queue, &cmd, pdMS_TO_TICKS(100));
        }
        

    }

}

static void handle_event_manual_control(button_event_t* ev) {

    if (ev->event == BTN_EVENT_PRESS || ev->event == BTN_EVENT_REPEAT || ev->event == BTN_EVENT_LONG) {
        if (ev->button == BTN_UP && ev->event != BTN_EVENT_LONG) {
            s_manual_control_position+=0.01;
            set_desired_position_override_value(s_manual_control_position);
            if (s_manual_control_position>1) {s_manual_control_position=1;}
        } else if (ev->button == BTN_DOWN && ev->event != BTN_EVENT_LONG) {
            s_manual_control_position-=0.01;
            set_desired_position_override_value(s_manual_control_position);
            if (s_manual_control_position<0) {s_manual_control_position=0;}
        } else if (ev->button == BTN_OK && ev->event == BTN_EVENT_LONG) {
            if (s_stabilization_disabled==true) {
                resume_polling();
                s_stabilization_disabled=false;
            } else {
                suspend_polling();
                s_stabilization_disabled=true;
            }
            // s_stabilization_disabled=!s_stabilization_disabled;
        } else if (ev->button == BTN_BACK && ev->event == BTN_EVENT_LONG) {
            if (s_stabilization_disabled) {
                resume_polling();
                s_stabilization_disabled=false;
            }
            enter_state(STATE_MAIN_MENU);
            set_desired_position_override(false);
        }
    }
    

}


static void render_locked()
{
    render_clear();

    s_valve_positions = get_calibrated_adc_values();
    
    // Крупным шрифтом позиция
    char buf[32]={};
    snprintf(buf, sizeof(buf), "%5.1f%%", s_valve_positions.actualPositionPercent*100);
    render_text_large_page_col(1, 12, buf);
    snprintf(buf, sizeof(buf), "Required: %5.1f%%", s_valve_positions.desiredPositionPercent*100);
    render_text_page_col(4, 12, 0, buf);
    // render_textf_page_col(4, 10, 0, 1, "Requir\n ed: %.2f", s_valve_positions.desiredPositionPercent);

    // Подсказка мелким
    render_text_page_col(6, 0, false, "Hold OK+BACK 2s");
    render_text_page_col(7, 0, false, "to unlock");

}

static void render_menu()
{
    render_clear();
    render_text_page_col(0, 0, false, "=== MENU ===");

    int start_idx = (s_menu_cursor > 2) ? s_menu_cursor - 2 : 0;
    int visible = 6;
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
                    case ENTER_MANUAL_CONTROL:     snprintf(line, sizeof(line), "  %s", main_menu_labels[2]); break;
                    case VERSION_INFO:             snprintf(line, sizeof(line), "  %s", main_menu_labels[3]); break;
                    default: snprintf(line, sizeof(line), "  ???"); break;
                }
                break;
            }

            case STATE_CONFIG_MENU: {
                switch (idx) {
                    case CONFIG_KP:        snprintf(line, sizeof(line), "  Kp: %.2f", g_config.kp); break;
                    case CONFIG_KI:        snprintf(line, sizeof(line), "  Ki: %.2f", g_config.ki); break;
                    case CONFIG_KD:        snprintf(line, sizeof(line), "  Kd: %.2f", g_config.kd); break;
                    case CONFIG_DEADZONE:  snprintf(line, sizeof(line), "  DZ: %3.0f%%", g_config.deadzone*100); break;
                    case CONFIG_PWM_FREQ:  snprintf(line, sizeof(line), "  PWM: %dHz", (int)g_config.pwm_freq_hz); break;
                    case CONFIG_EMA_ALPHA: snprintf(line, sizeof(line), "  EMA: %.2f", g_config.ema_alpha); break;
                    case CONFIG_RESET:     snprintf(line, sizeof(line), "  Reset values"); break;
                    default: snprintf(line, sizeof(line), "  ???"); break;
                }
                break;
            }

            case STATE_CALIBRATION_MENU: {
                switch (idx) {
                    case CALIBRATE_ACTUAL_POSITION:        snprintf(line, sizeof(line), "  %s", calibration_menu_labels[0]); break;
                    case CALIBRATE_DESIRED_POSITION:       snprintf(line, sizeof(line), "  %s", calibration_menu_labels[1]); break;
                    case CALIBRATE_CURRENT_LOOP_OUTPUT:    snprintf(line, sizeof(line), "  %s", calibration_menu_labels[2]); break;
                    case CALIBRATION_RESET:                snprintf(line, sizeof(line), "  %s", calibration_menu_labels[3]); break;
                    default: snprintf(line, sizeof(line), "  ???"); break;
                }
                break;
            }

            default: {snprintf(line, sizeof(line), "??? State"); ESP_LOGW(TAG, "Unexpected state in render_menu");}
        } 
        

        if (selected) line[0] = '>'; // курсор
        render_text_page_col(1 + i, 0, selected, line);
    }

    render_text_page_col(7, 0, false, bottom_text);
}

static void render_edit()
{
    render_clear();
    render_text_page_col(0, 0, false, "EDIT:");
    render_text_page_col(1, 0, config_menu_labels[s_edit_item], false);

    char val_str[32];
    switch (s_edit_item) {
        case CONFIG_KP:        snprintf(val_str, sizeof(val_str), "%.2f", g_config.kp); break;
        case CONFIG_KI:        snprintf(val_str, sizeof(val_str), "%.2f", g_config.ki); break;
        case CONFIG_KD:        snprintf(val_str, sizeof(val_str), "%.2f", g_config.kd); break;
        case CONFIG_DEADZONE:  snprintf(val_str, sizeof(val_str), "%3.0f%%", g_config.deadzone*100); break;
        case CONFIG_PWM_FREQ:  snprintf(val_str, sizeof(val_str), "%d%%",   g_config.pwm_freq_hz*100); break;
        case CONFIG_EMA_ALPHA: snprintf(val_str, sizeof(val_str), "%.2f", g_config.ema_alpha); break;
        default: snprintf(val_str, sizeof(val_str), "???"); break;
    }
    render_text_large_page_col(3, 20, val_str);

    render_text_page_col(6, 0, false, "UP/DN=Change");
    render_text_page_col(7, 0, false, "BACK=Cancel OK=Save");
}

static void render_calibration() {

    s_valve_positions = get_raw_adc_values();

    render_clear();
    render_text_page_col(0,false,false,"Calibration:");
    render_textf_page_col(2,5,false,true,s_calibration_text);


    render_textf_page_col(4, 5, 0, 1, "AP: %5.1f%% / %.2fmA\nDP: %5.1f%% / %.2fmA", 
        s_valve_positions.actualPositionPercent*100/4096,  4.0+16.0*s_valve_positions.actualPositionPercent/4096,
        s_valve_positions.desiredPositionPercent*100/4096, 4.0+16.0*s_valve_positions.desiredPositionPercent/4096);
    
    // ESP_LOGI(TAG, "Rendered calibration");
}

static void render_confirm_action() {
    render_clear();
    // render_textf_page_col(1,5, false, true, "Are you sure that\nyou wanna exit to\nthe main menu?\nAll unsaved progess\nwill be lost");
    render_textf_page_col(3,5, false, true, "Save calibration?");
    render_textf_page_col(7,6, false, false, "BACK=Reject, OK=Save");

}


// {.rect_coords={{19,20},{109,40}}, .text_pos_rel={3,3}, .characters_in_row=14};

static void render_notification(ui_notif_t notif) {
    render_clear_rect(19, 20, 109, 40);
    render_rect(19, 20, 109, 40, true);
    // render_text_page_col(0,40, true, notif.text);
    char buf[15]={};
    int pos=0;
    int line=0;
    for (int i=0; i<strlen(notif.text);i++) {
        buf[pos]=notif.text[i];
        pos+=1;
        if (pos>=14) {
            buf[14]='\0';
            render_text_xy(22, 23+line*8, false, buf);
            // buf[i%14]
            pos=0;
            line+=1;
        }

    }
    buf[pos]='\0';
    render_text_xy(22, 23+line*8, false, buf);

    // render_text_xy(20, 25, false, notif.text);

}

static void render_manual_control() {

    render_clear();
    char buf[15]={};
    // s_valve_positions = get_raw_adc_values();
    render_text_page_col(2,6, 0, "Manual control");
    snprintf(buf, sizeof(buf), "%5.1f%%", s_manual_control_position*100);
    render_text_large_page_col(3,12, buf);

    render_textf_page_col(6,6, false, false, "BACK=Exit");
    render_textf_page_col(7,6, false, false, "OK=Stop/Resume");
}


void ui_init(void *lcd_panel_handle)
{
    notifications_queue = xQueueCreate(10, sizeof(ui_notif_t));
    renderer_init(lcd_panel_handle);
    s_state = STATE_LOCKED;
    s_last_activity_us = esp_timer_get_time();
    ESP_LOGI(TAG, "UI initialized");
}

void ui_run(void)
{
    button_event_t ev;
    cal_msg_t cal_msg;
    ui_notif_t notification;
    TickType_t last_render = 0;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {

        // Обработка событий кнопок (неблокирующая проверка)
        while (xQueueReceive(g_button_queue, &ev, 0) == pdTRUE) {
            ESP_LOGV(TAG, "Received %d on %d", ev.event, ev.button);
            switch (s_state) {
                case STATE_LOCKED:    handle_event_locked(&ev); break;
                case STATE_MAIN_MENU: 
                case STATE_CONFIG_MENU:
                case STATE_CALIBRATION_MENU: handle_event_menu(&ev); break;
                case STATE_EDIT_VALUE: handle_event_edit(&ev); break;
                case STATE_CALIBRATION: handle_event_calibration(&ev); break;
                case STATE_CONFIRM_ACTION: handle_event_confirm_action(&ev); break;
                case STATE_MANUAL_CONTROL: handle_event_manual_control(&ev); break;
                // default: break;  // !!!!!!!!!!!!!!!!
            }
            // render_clear();
            // render_flush();
        }


        while (xQueueReceive(calibration_messages_queue, &cal_msg, 0) == pdTRUE) {

            switch (cal_msg.status) {
                case CAL_ST_STARTED:
                    snprintf(s_calibration_text, sizeof(s_calibration_text), "Calibration\nin progress");
                    enter_state(STATE_CALIBRATION);
                    break;

                case CAL_ST_DONE:
                    ui_send_custom_notification("Calib successful", 3, 100);
                    enter_state(STATE_CALIBRATION_MENU);
                    break;

                case CAL_ST_REJECTED:
                    ui_send_custom_notification("Calib rejected", 3, 100);
                    enter_state(STATE_CALIBRATION_MENU);
                    break;

                case CAL_ST_ERROR:
                case CAL_ST_CANCELLED:
                    ui_send_custom_notification("Calib unsuccessful", 3, 100);
                    enter_state(STATE_CALIBRATION_MENU);
                    break;

                case CAL_ST_WAIT_FOR_CONFIRM:
                    enter_state(STATE_CONFIRM_ACTION);
                    break;

                case CAL_AP_UPPER_LIMIT:
                    snprintf(s_calibration_text, sizeof(s_calibration_text), "Testing\n upper limit");
                    break;

                case CAL_AP_LOWER_LIMIT:
                    snprintf(s_calibration_text, sizeof(s_calibration_text), "Testing\n lower limit");
                    break;

                case CAL_DP_SET_VALUE:
                    snprintf(s_calibration_text, sizeof(s_calibration_text), "Set input value\n %.1f%% / %0.1fmA", cal_msg.value, 4.0+16.0*cal_msg.value);
                    break;

                case CAL_CL_NEEDS_VALUE:
                    snprintf(s_calibration_text, sizeof(s_calibration_text), "Print value\nthat you see");
                    break;

                default:
                    break;
            }
        }

        
        if (notification.display_time<=0 && (xQueueReceive(notifications_queue, &notification, 0) == pdTRUE)) {
            // ESP_LOGI(TAG, "Notification received succesfully: \"%s\"", notification.text);
        }

        // Inactivity timeout check
        if (s_state != STATE_LOCKED && s_state != STATE_MANUAL_CONTROL) {
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
        uint16_t time_since_last_render_ms = pdTICKS_TO_MS(now - last_render);
        if (time_since_last_render_ms >= 1000/SCREEN_FPS) {
            switch (s_state) {
                case STATE_LOCKED:    render_locked(); break;
                case STATE_MAIN_MENU: 
                case STATE_CONFIG_MENU:
                case STATE_CALIBRATION_MENU: render_menu(); break;
                case STATE_EDIT_VALUE: render_edit(); break;
                case STATE_CALIBRATION: render_calibration(); break;
                case STATE_CONFIRM_ACTION: render_confirm_action(); break;
                case STATE_MANUAL_CONTROL: render_manual_control(); break;
                // default: break; /// !!!!!!!!!!!!!!!!!!!
            }

            // ESP_LOGI(TAG, "%f", notification.display_time);
            if (notification.display_time>0) {
                render_notification(notification);
                notification.display_time-=(float)time_since_last_render_ms/1000;
            }
            render_flush();
            last_render = now;
        }

        xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000/UI_CYCLE_FPS)); 
        // vTaskDelay(pdMS_TO_TICKS(UI_CYCLE_FPS));  // 50 Гц цикл событий
        // ESP_LOGI(TAG,"ui cycle");
    }
}
