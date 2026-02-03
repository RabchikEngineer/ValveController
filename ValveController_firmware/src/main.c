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
#include "params.h"
#include "buttons.h"
#include "ui.h"


const char *TAG_PID = "PID System";
const char *TAG_INPUT = "Input Handler";
const char *TAG_VALVE = "Valve Switcher";
const char *TAG_PWM = "PWM System";
// const char *TAG_ADC = "ADC";
static const char *TAG = "Main Thread";


typedef struct {
    float frequency;
    int resolution; 
} PWMConfig_t;

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t pcf;
    i2c_master_dev_handle_t dac;
    esp_lcd_panel_handle_t  panel;
    esp_lcd_panel_io_handle_t panel_io;
} i2c_periphery_t;


QueueHandle_t inputQueue;
QueueHandle_t outputQueue;


static i2c_periphery_t i2c_periphery;


const TickType_t polling_delay = pdMS_TO_TICKS(POLLING_DELAY);
// const float PID_ATTENUATION = 1000.0f; // so that we can use higher Kp,Ki,Kd
const float PID_ATTENUATION = 1.0f; // so that we can use higher Kp,Ki,Kd


volatile bool led_state = 0;

volatile float valve_activation_threshold = 0.17; //for -1 --- 1 pid output

volatile float current_position=0.0f;


//zone in which integral doesnt calculates, so the system ceases to stabilize
//so accuracy shold be 20/4096=0.5%
float dead_zone_size = 0.01; 


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






void i2c_bus_init(i2c_master_bus_handle_t* bus)
{
     i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, 
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, bus));
    ESP_LOGI(TAG, "I2C bus init completed");
}

void i2c_bus_destroy(i2c_master_bus_handle_t bus)
{
    if (!bus) return;
    ESP_ERROR_CHECK(i2c_del_master_bus((i2c_master_bus_handle_t)bus));
    free(bus);
}

// void i2c_bus_remove_device(i2c_master_bus_handle_t bus, void *dev)
// {
//     (void)bus;
//     if (!dev) return;
//     ESP_ERROR_CHECK(i2c_master_bus_rm_device((i2c_master_dev_handle_t)dev));
// }

int i2c_bus_probe(i2c_master_bus_handle_t bus, uint16_t addr_7bit, int timeout_ms)
{
    if (!bus) return -1;
    return i2c_master_probe(bus, addr_7bit, timeout_ms);
}


void i2c_periphery_init(i2c_periphery_t* i2c_periphery) {

    i2c_device_config_t dac_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MCP4725_ADDR_7B,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_periphery->bus, &dac_dev_cfg, &i2c_periphery->dac));


    i2c_device_config_t pcf_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCF8574DWR_ADDR_7B,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t pcf_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_periphery->bus, &pcf_cfg, &i2c_periphery->pcf));


    esp_lcd_panel_io_i2c_config_t panel_io_cfg = {
        .dev_addr = OLED_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_periphery->bus, &panel_io_cfg, &i2c_periphery->panel_io));


    esp_lcd_panel_dev_config_t panel_cfg = {
        .bits_per_pixel = 1,
        .reset_gpio_num = OLED_RST_GPIO,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(i2c_periphery->panel_io, &panel_cfg, &i2c_periphery->panel));


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

        data = get_adc_values();

        
        data.desiredPositionPercent=(data.desiredPositionPercent-705)/(3590-705);
        data.actualPositionPercent=(data.actualPositionPercent-1560)/(2480-1560);


        // Send to PID task
        xQueueSend(inputQueue, &data, portMAX_DELAY);

        
        current_position=data.actualPositionPercent;
        // ESP_LOGI(TAG,"%d",(uint16_t)(data.actualPositionPercent*(1<<12)));
        // ESP_LOGI(TAG,"%d",((uint16_t)(data.actualPositionPercent*(1<<12)) & 0x0FFF));

        xTaskDelayUntil(&last_wake_time,polling_delay); 
    }
}



void PID_task() {
    ValvePositionData_t receivedData;
    
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
            dt = (current_time - last_time) / 1000000.0f; // calculating actual dt. however it might differs only for 1%
            last_time=current_time;

            float derivative = (error - prev_error) / dt;
            prev_error = error;

            pid_output = ((g_params.kp * error) + (g_params.ki * integral) + (g_params.kd * derivative)) / PID_ATTENUATION;

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




void app_main(void)
{   
    
    vTaskDelay(pdMS_TO_TICKS(5000));


    ESP_LOGI(TAG,"Starting program");

    inputQueue = xQueueCreate(10, sizeof(ValvePositionData_t));
    outputQueue = xQueueCreate(10, sizeof(float));

    nvs_init();
    params_load();

    printf("Current PID: %.6f %.6f %.6f\n", g_params.kp, g_params.ki, g_params.kd);

    // data_point_t* points[] = {{1,2},{2,4}};
    // // ApproxPoly_t input_approx = {3,points};
    // ApproxPoly_t input_approx = {
    // .num_points = 2,
    // .points = points
    // };

    // for (int i=0; i<2; i++) {

    //     printf("%f",input_approx.points[i]);

    // }



    


    i2c_bus_init(&i2c_periphery.bus);
    i2c_periphery_init(&i2c_periphery);
    ESP_LOGI(TAG,"I2C init completed");



    valve_pwm_init(&pwm_10_hz_config);
    ESP_LOGI(TAG, "PWM configured");


    

    
    
    usb_comm_init(&g_params);
    ESP_LOGI(TAG, "USB input configured");




    ESP_LOGI(TAG, "Starting tasks...");
    
    adc_start();
    
    read_buttons_init(i2c_periphery.pcf, 10);
    xTaskCreate(input_task, "Input Task", 2048, NULL, 7, NULL);
    xTaskCreate(PID_task, "PID Task", 4096, NULL, 8, NULL);
    xTaskCreate(set_valve_pwm_task, "PWM&Output Task", 2048, NULL, 9, NULL);
    xTaskCreate(usb_input_task, "input_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Tasks started...");


    
    ui_init(i2c_periphery.panel, &current_position);
    xTaskCreate((TaskFunction_t)ui_run, "ui", 4096, NULL, 5, NULL);

        


    
    

    


   





    

}

