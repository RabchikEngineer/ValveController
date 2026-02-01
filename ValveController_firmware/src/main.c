#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "math.h"

//for serial communication
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "driver/usb_serial_jtag.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"


#include "config.h"
#include "adc.h"
#include "usb_comm.h"


const char *TAG_PID = "PID System";
const char *TAG_INPUT = "Input Handler";
const char *TAG_VALVE = "Valve Switcher";
const char *TAG_PWM = "PWM System";
// const char *TAG_ADC = "ADC";
static const char *TAG = "Main Thread";


typedef struct {
    float x;
    float y;
} DataPoint_t;

typedef struct {
    int num_points;
    DataPoint_t* points;
} ApproxPoly_t;

typedef struct {
    float frequency;
    int resolution; 
} PWMConfig_t;


QueueHandle_t inputQueue;
QueueHandle_t outputQueue;



const TickType_t polling_delay = pdMS_TO_TICKS(POLLING_DELAY);
// const float PID_ATTENUATION = 1000.0f; // so that we can use higher Kp,Ki,Kd
const float PID_ATTENUATION = 1.0f; // so that we can use higher Kp,Ki,Kd


volatile bool led_state = 0;

volatile float valve_activation_threshold = 0.17; //for -1 --- 1 pid output


//zone in which integral doesnt calculates, so the system ceases to stabilize
//so accuracy shold be 20/4096=0.5%
float dead_zone_size = 0.01; 

volatile PIDConfig_t g_config = {
    2.0f, 
    1.0f, 
    2.0f
};

PWMConfig_t pwm_10_hz_config = {
    .frequency = 10,
    .resolution = 14
};

PWMConfig_t pwm_5_hz_config = {
    .frequency = 5,
    .resolution = 14
};

PWMConfig_t pwm_2_hz_config = {
    .frequency = 2,
    .resolution = 14
};


// PIDConfig_t g_config = {
//     0.5f, 
//     0.0f, 
//     0.0f
// };






i2c_master_bus_handle_t i2c_bus_create(const void *bus_cfg)
{
    // i2c_master_bus_handle_t bus = calloc(1, sizeof(bus));
    // if (!bus) return NULL;

    i2c_master_bus_handle_t bus = NULL;

    ESP_ERROR_CHECK(i2c_new_master_bus((const i2c_master_bus_config_t*)bus_cfg,
                                      (i2c_master_bus_handle_t*)&bus));
    return bus;
}

void i2c_bus_destroy(i2c_master_bus_handle_t bus)
{
    if (!bus) return;
    ESP_ERROR_CHECK(i2c_del_master_bus((i2c_master_bus_handle_t)bus));
    free(bus);
}

i2c_master_dev_handle_t* i2c_bus_add_device(i2c_master_bus_handle_t bus, const void *dev_cfg)
{
    void *dev = NULL; // i2c_master_dev_handle_t
    if (!bus) return NULL;

    ESP_ERROR_CHECK(i2c_master_bus_add_device((i2c_master_bus_handle_t)bus,
                                              (const i2c_device_config_t*)dev_cfg,
                                              (i2c_master_dev_handle_t*)&dev));
    return dev;
}

void i2c_bus_remove_device(i2c_master_bus_handle_t bus, void *dev)
{
    (void)bus;
    if (!dev) return;
    ESP_ERROR_CHECK(i2c_master_bus_rm_device((i2c_master_dev_handle_t)dev));
}

int i2c_bus_probe(i2c_master_bus_handle_t bus, uint16_t addr_7bit, int timeout_ms)
{
    if (!bus) return -1;
    return i2c_master_probe(bus, addr_7bit, timeout_ms);
}





typedef struct {
    i2c_master_dev_handle_t *i2c_dev;      // i2c_master_dev_handle_t
    uint8_t addr_7bit;  // informational
} mcp4725_t;


static inline uint8_t clamp_pd(uint8_t pd)    { return pd & 0x03; }
static inline uint16_t clamp_code(uint16_t c) { return c & 0x0FFF; }

int mcp4725_init(mcp4725_t *d, void *i2c_dev_handle, uint8_t addr_7bit)
{
    if (!d || !i2c_dev_handle) return -1;
    d->i2c_dev = i2c_dev_handle;
    d->addr_7bit = addr_7bit;
    return 0;
}

// Fast mode write (C2=0,C1=0,C0=don't care): 2 bytes after address.
// Byte0 = [PD1 PD0 D11 D10 D9 D8], Byte1 = [D7..D0]. [file:1]
int mcp4725_write_fast(mcp4725_t *d, uint16_t code12, uint8_t pd_bits, int timeout_ms)
{
    if (!d || !d->i2c_dev) return -1;

    code12 = clamp_code(code12);
    pd_bits = clamp_pd(pd_bits);

    uint8_t buf[2];
    buf[0] = (uint8_t)((pd_bits << 4) | ((code12 >> 8) & 0x0F));
    buf[1] = (uint8_t)(code12 & 0xFF);

    return i2c_master_transmit((i2c_master_dev_handle_t)d->i2c_dev, buf, sizeof(buf), timeout_ms);
}

// Write DAC register + EEPROM (C2=0,C1=1,C0=1): 3 bytes after address. [file:1]
// Byte0 = [0 1 1 X X PD1 PD0 X]
// Byte1 = [D11..D4]
// Byte2 = [D3..D0 XXXX] [file:1]
int mcp4725_write_eeprom(mcp4725_t *d, uint16_t code12, uint8_t pd_bits, int timeout_ms)
{
    if (!d || !d->i2c_dev) return -1;

    code12 = clamp_code(code12);
    pd_bits = clamp_pd(pd_bits);

    uint8_t buf[3];
    buf[0] = (uint8_t)((0b011 << 5) | (pd_bits << 1));  // command + PD bits [file:1]
    buf[1] = (uint8_t)(code12 >> 4);
    buf[2] = (uint8_t)((code12 & 0x0F) << 4);

    // EEPROM write takes up to ~50 ms; ensure timeout_ms is big enough. [file:1]
    return i2c_master_transmit((i2c_master_dev_handle_t)d->i2c_dev, buf, sizeof(buf), timeout_ms);
}




 i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, // ok for bring-up; external pullups still recommended [attached_file:1]
    };



mcp4725_t dac;

int current_loop_value = 0;



esp_err_t pcf8574_read_u8(i2c_master_dev_handle_t dev, uint8_t *value, int timeout_ms)
{
    if (!dev || !value) return ESP_ERR_INVALID_ARG;

    return i2c_master_receive(dev, value, 1, timeout_ms);

}





void valve_pwm_init(PWMConfig_t* config) {
    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = config->resolution,
        .timer_num        = PWM_TIMER,
        .freq_hz          = config->frequency,
        .clk_cfg          = LEDC_USE_RC_FAST_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure channel for INCREASE valve
    ledc_channel_config_t ledc_channel_inc = {
        .gpio_num       = VALVE_INC_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = PWM_CHANNEL_INC,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = PWM_TIMER,
        .duty           = 0,  // Start at 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_inc));

    ledc_channel_config_t ledc_channel_dec = {
        .gpio_num       = VALVE_DEC_PIN,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = PWM_CHANNEL_DEC,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = PWM_TIMER,
        .duty           = 1,  // Start at 100%
        .hpoint         = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_dec));

    ESP_LOGI(TAG_VALVE, "PWM initialized: %d Hz, %d-bit resolution", 
             PWM_FREQUENCY, PWM_RESOLUTION);
}







void input_task() {
    ValvePositionData_t data;
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        // Read desired position from ADC or 4-20mA interface
        // data.desiredPositionPercent = ReadDesiredPosition();
        // data.desiredPositionPercent = 1500;
        data = get_adc_values();

        
        data.desiredPositionPercent=(data.desiredPositionPercent-705)/(3590-705);
        data.actualPositionPercent=(data.actualPositionPercent-1560)/(2480-1560);


        // Send to PID task
        xQueueSend(inputQueue, &data, portMAX_DELAY);

        mcp4725_write_fast(&dac,(uint16_t)(current_loop_value),0x00,100);
        // ESP_LOGI(TAG,"%d",(uint16_t)(data.actualPositionPercent*(1<<12)));
        // ESP_LOGI(TAG,"%d",((uint16_t)(data.actualPositionPercent*(1<<12)) & 0x0FFF));

        xTaskDelayUntil(&last_wake_time,polling_delay); 
    }
}



void PID_task() {
    ValvePositionData_t receivedData;
    // PIDController pid;
    // PID_Init(&pid); // initialize PID with tuned params
    
    float error = 0;
    float pid_output = 0;
    float integral = 0.0f;
    float prev_error = 0.0f;
    float dt=0.1f;
    int64_t last_time=0;

    // float Kp = 0.004f;
    // float Ki = 0.001f; // работают но с перелётом
    // float Kd = 0.002f;

    // float Kp = 2.0;
    // float Ki = 1.0;
    // float Kd = 2.0;

    while (1) {
        if (xQueueReceive(inputQueue, &receivedData, portMAX_DELAY) == pdPASS) {

            // controlOutput = PID_Compute(&pid, receivedData.desiredPositionPercent, receivedData.actualPositionPercent);
            error = receivedData.desiredPositionPercent - receivedData.actualPositionPercent;

            int64_t current_time = esp_timer_get_time();
            dt = (current_time - last_time) / 1000000.0f; //calculating actual dt. however it might differs only for 1%
            last_time=current_time;

            float derivative = (error - prev_error) / dt;
            prev_error = error;

            pid_output = ((g_config.kp * error) + (g_config.ki * integral) + (g_config.kd * derivative)) / PID_ATTENUATION;

            bool saturated = (pid_output > 1.0f) || (pid_output < -1.0f);
            bool in_dead_zone = (fabsf(error) < dead_zone_size); // doesnt calculate integral in small zone
            // Only integrate if not saturated
            if (!saturated && !in_dead_zone) {
                integral += error * dt;
                if (integral > 1.0f) integral = 1.0f; 
                if (integral < -1.0f) integral = -1.0f;
            } else if (saturated){
                // Decay integral slowly when can't actuate
                integral *= 0.95f;
            } else if (in_dead_zone){
                integral = 0;
            }


            // Convert controlOutput to command for valves
            ESP_LOGI(TAG_PID, "A: %.3f/D: %.3f,I: %.3f , dt: %.5f, %.4f", 
                     receivedData.actualPositionPercent, receivedData.desiredPositionPercent, integral, dt, pid_output);
            xQueueSend(outputQueue, &pid_output, portMAX_DELAY);
        }
    }
}


void set_valve_pwm_task() {
    // PID output ranges (example: -100 to +100)
    // Negative = decrease valve, Positive = increase valve
    
    uint32_t max_duty = (1 << PWM_RESOLUTION) - 1;  // 1023 for 10-bit
    uint32_t duty_inc = 0;
    uint32_t duty_dec = 0;

    TickType_t disabled_until = 0;

    float pid_output=0;
    
    while (1) {
        if (xQueueReceive(outputQueue, &pid_output, portMAX_DELAY) == pdPASS) {


            if (fabsf(pid_output)<valve_activation_threshold) { // dead zone near center
                pid_output=0.0;
            } else if (pid_output+valve_activation_threshold>1.0) { // consider high values as 1
                pid_output=1.0;
            } else if (pid_output-valve_activation_threshold<-1.0) {
                pid_output=-1.0;
            }

            // if (pid_output>0) {pid_output=0.1;} else {pid_output=-0.1;}
            
            duty_inc = (uint32_t)(pid_output * max_duty);
            duty_dec = (uint32_t)(-pid_output * max_duty);

            // ESP_LOGI(TAG_VALVE, "PID: %.2f -> INC duty: %ld, DEC duty: %ld", 
            //     pid_output, duty_inc, max_duty-duty_dec);

            // if (duty_inc<0) {duty_inc=0;}
            // if (duty_dec<0) {duty_dec=0;}

            // if (pid_output > 0) {
            //     // Open INCREASE valve proportionally
            //     duty_inc = (uint32_t)(pid_output * max_duty);
            //     duty_dec = 0;
            // } else if (pid_output < 0) {
            //     // Open DECREASE valve proportionally
            //     duty_inc = 0;
            //     duty_dec = (uint32_t)(-pid_output * max_duty);
            // } else {
            //     // Stop both valves
            //     duty_inc = 0;
            //     duty_dec = 0;
            // }


            
            float abs_output = fabsf(pid_output);
            
            // // Choose frequency based on required control precision
            // if (abs_output > 0.5f) {
            // // Large error: fast response (10Hz)
            //     frequency = 10;
            // } else if (abs_output > 0.2f) {
            // // Medium error: balanced (5Hz)
            //     frequency = 5;
            // } else if (abs_output > 0.05f) {
            // // Small error: fine control (2Hz)
            //     frequency = 2;
            // } else {
            //     abs_output=0
            // }
            
            TickType_t now=xTaskGetTickCount();
            
            if (now>disabled_until) {

                // Set PWM duty cycles
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_INC, duty_inc));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_INC));
                
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_DEC, max_duty-duty_dec)); //inverting output max_duty-duty_dec
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_DEC));
                
                // ESP_LOGI(TAG_VALVE, "PID: %.2f -> INC duty: %.2f, DEC duty: %.2f", 
                //         pid_output, (float)duty_inc/max_duty, (float)duty_dec/max_duty);
                
                disabled_until=now+pdMS_TO_TICKS(100);
            }
        }
    }

    
}



static void oled_draw_checkerboard(esp_lcd_panel_handle_t panel, int size)
{
    static uint8_t fb[OLED_WIDTH * OLED_HEIGHT / 8]; // 1024 bytes for 128x64, 1bpp [web:209]

    // Fill framebuffer with a simple pattern: alternating 8x8 blocks
    for (int y = 0; y < OLED_HEIGHT; y++) {
        for (int x = 0; x < OLED_WIDTH; x++) {
            bool on = (((x / size) ^ (y / size)) & 1);
            int byte_index = x + (y / 8) * OLED_WIDTH;   // SSD1306 page layout
            uint8_t bit = 1 << (y & 7);
            if (on) fb[byte_index] |= bit;
            else    fb[byte_index] &= (uint8_t)~bit;
        }
    }

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, 0, OLED_WIDTH, OLED_HEIGHT, fb));
}



static void oled_draw_color(esp_lcd_panel_handle_t panel, bool color) {

    const int arr_size = OLED_WIDTH * OLED_HEIGHT / 8;

    uint8_t fb[arr_size];

    uint8_t value_to_fill = 0xFF * color;

    // ESP_LOGI(TAG, "%x", value_to_fill);

    // for (int i = 0; i < arr_size; i++) {
        
    //     fb[i] = value_to_fill;

    // }

    memset(fb, value_to_fill, sizeof(fb));

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, 0, OLED_WIDTH, OLED_HEIGHT, fb));
    
}



void app_main(void)
{   
    
    vTaskDelay(pdMS_TO_TICKS(5000));


    ESP_LOGI(TAG,"Starting program");


    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    printf("Current PID: %.6f %.6f %.6f\n", g_config.kp, g_config.ki, g_config.kd);

    // DataPoint_t* points[] = {{1,2},{2,4}};
    // // ApproxPoly_t input_approx = {3,points};
    // ApproxPoly_t input_approx = {
    // .num_points = 2,
    // .points = points
    // };

    // for (int i=0; i<2; i++) {

    //     printf("%f",input_approx.points[i]);

    // }

    // gpio_set_direction(VALVE_INC_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_direction(VALVE_DEC_PIN, GPIO_MODE_OUTPUT);
    // gpio_set_direction(CURRENT_POSITION_PIN, GPIO_MODE_INPUT);
    // gpio_set_direction(INPUT_PERCENT_PIN, GPIO_MODE_INPUT);


    inputQueue = xQueueCreate(10, sizeof(ValvePositionData_t));
    outputQueue = xQueueCreate(10, sizeof(float));




    valve_pwm_init(&pwm_10_hz_config);
    ESP_LOGI(TAG, "PWM configured");

    
    
    // usb_comm_init(&g_config);
    // usb_input_task();
    // xTaskCreate(usb_input_task, "USB Comm Task", 4096, NULL, 11, NULL); 

    
    ESP_LOGI(TAG, "USB input configured");

    ESP_LOGI(TAG, "Starting tasks...");
    
    adc_start();
    
    xTaskCreate(input_task, "Input Task", 2048, NULL, 7, NULL);
    xTaskCreate(PID_task, "PID Task", 4096, NULL, 8, NULL);
    xTaskCreate(set_valve_pwm_task, "PWM&Output Task", 2048, NULL, 9, NULL);
    // xTaskCreate(usb_input_task, "input_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Tasks started...");


    // xTaskCreate(fan_control_task, "Fan task", 2048, NULL, 5, NULL);
    // xTaskCreate(rpm_task, "rpm_task", 2048, NULL, 5, NULL);


    i2c_master_bus_handle_t i2c_bus = i2c_bus_create(&bus_cfg);

    i2c_device_config_t dac_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MCP4725_ADDR_7B,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    i2c_master_dev_handle_t* dac_dev_handle = i2c_bus_add_device(i2c_bus, &dac_dev_cfg);


    
    mcp4725_init(&dac, dac_dev_handle, dac_dev_cfg.device_address);



    i2c_device_config_t pcf_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCF8574DWR_ADDR_7B,
        .scl_speed_hz = I2C_FREQ_HZ,
    };



    i2c_master_dev_handle_t pcf_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &pcf_cfg, &pcf_handle));

    


   





    // PCF READ
    // uint8_t pcf_val;
    // while (1) {

    //     pcf8574_read_u8(pcf_handle, &pcf_val, 100);



    //     ESP_LOGI(TAG,"%d",pcf_val);
    //     ESP_LOGI(TAG,"%d,%d,%d,%d",(pcf_val & 0x0001),(pcf_val & 0x0002)>>1,(pcf_val & 0x0004)>>2,(pcf_val & 0x0008)>>3);
        

    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }



    // i2c_master_bus_handle_t i2c_bus = NULL;
    // i2c_master_bus_config_t bus_cfg = {
    //     .i2c_port = I2C_PORT,
    //     .sda_io_num = I2C_SDA_GPIO,
    //     .scl_io_num = I2C_SCL_GPIO,
    //     .clk_source = I2C_CLK_SRC_DEFAULT,
    //     .glitch_ignore_cnt = 7,
    //     .flags.enable_internal_pullup = true,
    // };
    // ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    // (optional) keep using this same bus for MCP4725 / PCF8574 like you already do [file:207]

    // 2) Create esp_lcd I2C panel IO
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr = OLED_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_cfg, &io_handle));

    // 3) Create SSD1306 panel
    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_dev_config_t panel_cfg = {
        .bits_per_pixel = 1,
        .reset_gpio_num = OLED_RST_GPIO,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_cfg, &panel));

    // 4) Init + turn on
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    // 5) Draw something
    
    // int size=1;
    // while (1) {
    //     oled_draw_checkerboard(panel, size);
    //     vTaskDelay(pdMS_TO_TICKS(500));
    //     size+=1;
    //     if (size>30) {
    //         size=1;
    //     }
    // }

    bool color=1;
    while (1) {
        oled_draw_color(panel, color);
        vTaskDelay(pdMS_TO_TICKS(10000));
        color=!color;
    }


    // MCP TESTING
    // mcp4725_write_fast(&dac, 0x0800, 0 /*normal*/, 100);

    // while (1) {
    //     esp_err_t ret = i2c_bus_probe(bus, pcf_cfg.device_address, 100);
    //     if (ret==ESP_OK){
    //         ESP_LOGI(TAG,"Device is responding");
    //     } else {
    //         ESP_LOGI(TAG,"Device is not responding");
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    // int state = 0;
    // while (1) {

    //     mcp4725_write_fast(&dac,(uint16_t)state,0x00,100);

    //     state +=10;
    //     if (state>4096) {state=0;}

    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    // CURRENT LOOP VALUE OVVERIDE
    // // Install USB driver
    // usb_serial_jtag_driver_config_t usb_cfg = {
    //     .rx_buffer_size = 1024,
    //     .tx_buffer_size = 1024,
    // };
    // ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    // uint8_t buffer[256];
    // char line_buffer[256];
    // int line_pos = 0;
    // int val;
    // while (1) {
    //     int len = usb_serial_jtag_read_bytes(buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(100));
        
    //     if (len > 0) {
    //         for (int i = 0; i < len; i++) {
    //             char c = buffer[i];
                
    //             // Echo character
    //             // usb_serial_jtag_write_bytes((const char*)&c, 1, 0);
                
    //             if (c == '\n' || c == '\r') {
    //                 if (line_pos > 0) {
    //                     line_buffer[line_pos] = '\0';
                        
                        
    //                     if (sscanf(line_buffer, "%d", &val) == 1) {
    //                         current_loop_value=val;
                            
    //                         ESP_LOGI(TAG, "New value: %d", val);

    //                         // xQueueSend(usb_queue,&pid_config, portMAX_DELAY);
                            
    //                     } else {
    //                         ESP_LOGW(TAG, "Invalid input: %s", line_buffer);
    //                     }
                        
    //                     line_pos = 0;
    //                     // printf(">>> \n");
    //                     // fflush(stdout);
    //                 }
    //             } else if (c >= 32 && c < 127) {
    //                 if (line_pos < sizeof(line_buffer) - 1) {
    //                     line_buffer[line_pos++] = c;
    //                 }
    //             }
    //         }
    //     }
        
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }



}

