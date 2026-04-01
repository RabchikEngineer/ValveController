#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_continuous.h"

#include "config.h"
#include "adc.h"
#include "params.h"

static const char *TAG = "ADC";


static volatile float latest_actual_position = 0.0f;
static volatile float latest_desired_position = 0.0f;
static bool s_desired_position_override=false;
static float s_desired_position_override_value=0;

static adc_channel_t channels[2] = {ACTUAL_POSITION_PIN, DESIRED_POSITION_PIN};
static adc_continuous_handle_t adc_handle = NULL;



void set_desired_position_override(bool enabled) {
    s_desired_position_override=enabled;
}

void set_desired_position_override_value(float value) {
    s_desired_position_override_value=value;
}



void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle) {
    adc_continuous_handle_t handle = NULL;

    // Step 1: Create handle with buffer configuration
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = BUFFER_SIZE,
        .conv_frame_size = READ_LEN,  // Replaced conv_num_each_intr
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    // Step 2: Configure ADC controller
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    // Step 3: Configure channel patterns
    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT;
        adc_pattern[i].bit_width = ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}




void adc_continuous_read_task(void *arg) {
    uint8_t result[READ_LEN] = {0};
    uint32_t ret_num = 0;
    esp_err_t ret;

    ESP_LOGI(TAG, "adc_continuous_read_task started");

    // int test_pos=0;
    
    // Start continuous conversion
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    ESP_LOGI(TAG, "ADC continuous mode started");
    
    while (1) {

        // Read bytes - blocking with portMAX_DELAY
        ret = adc_continuous_read(adc_handle, result, READ_LEN, &ret_num, portMAX_DELAY);
        
        if (ret == ESP_OK) {
            int64_t sum_ch3 = 0, count_ch3 = READ_LEN/ADC_RESULT_BYTE/2; // 4 bytes per sample, 2 channels
            int64_t sum_ch4 = 0, count_ch4 = READ_LEN/ADC_RESULT_BYTE/2;
            
            // Process samples - accumulate for averaging
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                
                uint32_t chan_num = p->type2.channel;
                uint32_t data = p->type2.data;
                
                // Check valid channel
                if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT)) {
                    if (chan_num == 3) {
                        sum_ch3 += data;
                        // count_ch3++;
                    } else if (chan_num == 4) {
                        sum_ch4 += data;
                        // count_ch4++;
                    }
                }
            }

            
            latest_actual_position =  latest_actual_position* (1-EMA_WEIGHT)+((float)sum_ch3 / (float)count_ch3)*EMA_WEIGHT;
            latest_desired_position = latest_desired_position*(1-EMA_WEIGHT)+((float)sum_ch4 / (float)count_ch4)*EMA_WEIGHT;
            // ESP_LOGI(TAG, "%f", (double)latest_actual_position);

            // latest_desired_position = latest_desired_position*(1-EMA_WEIGHT)+test_desired_data[test_pos/4]*EMA_WEIGHT;
            // test_pos++;
            // if (test_pos>=200*4) {
            //     test_pos=0;
            // }
            // latest_desired_position = 2000;
            
            // ESP_LOGI(TAG, "Pos: %.3f, Input: %.3f (samples: %lld)", 
            //          latest_actual_position, latest_desired_position, count_ch4);
                     
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "ADC read timeout");
        } else {
            ESP_LOGE(TAG, "ADC read error: %d", ret);
        }
        
        // Small delay to prevent watchdog timeout
        // ESP_LOGW(TAG, "Adc timer:%lld", esp_timer_get_time());
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // Cleanup (never reached)
    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(adc_handle));
}


ValvePositionData_t get_calibrated_adc_values() {
    if (s_desired_position_override) {
        latest_desired_position = s_desired_position_override_value;
    }
    ValvePositionData_t data = {
        .desiredPositionPercent=s_desired_position_override ? s_desired_position_override_value : 
                                apply_calibration(g_calibration.desired_position_approx, latest_desired_position),
        .actualPositionPercent=apply_calibration(g_calibration.actual_position_approx, latest_actual_position)
    }; 
    
    return data;
}

ValvePositionData_t get_raw_adc_values() {
    ValvePositionData_t data = {
        .desiredPositionPercent=latest_desired_position,
        .actualPositionPercent=latest_actual_position
    }; 
    return data;
}


void adc_start() {

    continuous_adc_init(channels, sizeof(channels) / sizeof(adc_channel_t), &adc_handle);
    ESP_LOGI(TAG, "ADC configured");

}

