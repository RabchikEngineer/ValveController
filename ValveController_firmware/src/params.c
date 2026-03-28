#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>
#include "math.h"

#include "params.h"
#include "config.h"

static const char *TAG = "PARAMS";

static const char *NVS_NAMESPACE = "params";
static const char *NVS_CONFIG_KEY = "config";
static const char *NVS_CALIBRATION_KEY = "calibration";


params_t g_config_default = {
    .kp = 4.0f,
    .ki = 2.0f,
    .kd = 1.0f,
    .deadzone = 0.01f,
    .pwm_freq_hz = 10,
    .ema_alpha = 0.3f
};

volatile params_t g_config;

calibration_data_t g_calibration_default = {

    .actual_position_approx =  {.num_points=2,.points={{0,0},{4096,1}}},
    .desired_position_approx = {.num_points=2,.points={{0,0},{4096,1}}},
    .current_loop_approx =     {.num_points=2,.points={{0,820},{1, 4096}}}

};


volatile calibration_data_t g_calibration;


void nvs_init() 
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}


void config_load(void)
{   
    g_config = g_config_default;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_OK) {
        size_t sz = sizeof(params_t);
        err = nvs_get_blob(h, NVS_CONFIG_KEY, (params_t*)&g_config, &sz);
        if (err!=ESP_OK) {
            ESP_LOGW(TAG, "nvs_get_blob error");
        } else {
            ESP_LOGI(TAG, "Loaded: Kp=%.3f Ki=%.3f Kd=%.3f", g_config.kp, g_config.ki, g_config.kd);
        }
        nvs_close(h);
    } else {
        ESP_LOGW(TAG, "NVS open failed, using defaults");
    }
}

void config_save(void)
{   
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        nvs_set_blob(h, NVS_CONFIG_KEY, (params_t*)&g_config, sizeof(params_t));
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Saved config to NVS");
    }
}

void calibration_load(void)
{   
    g_calibration = g_calibration_default;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_OK) {
        size_t sz = sizeof(calibration_data_t);
        nvs_get_blob(h, NVS_CALIBRATION_KEY, (calibration_data_t*)&g_calibration, &sz);
        nvs_close(h);
        ESP_LOGI(TAG, "Loaded calibration data");
        ESP_LOGI(TAG, "%d", g_calibration.actual_position_approx.num_points);
        ESP_LOGI(TAG, "%.1f, %.1f", g_calibration.actual_position_approx.points[0].x, g_calibration.actual_position_approx.points[0].y);
        ESP_LOGI(TAG, "%.1f, %.1f", g_calibration.actual_position_approx.points[1].x, g_calibration.actual_position_approx.points[1].y);
        ESP_LOGI(TAG, "%.1f, %.1f", g_calibration.current_loop_approx.points[0].x, g_calibration.current_loop_approx.points[0].y);
        ESP_LOGI(TAG, "%.1f, %.1f", g_calibration.current_loop_approx.points[1].x, g_calibration.current_loop_approx.points[1].y);
    } else {
        ESP_LOGW(TAG, "NVS open failed, using defaults");
    }
}

void calibration_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        nvs_set_blob(h, NVS_CALIBRATION_KEY, (calibration_data_t*)&g_calibration, sizeof(calibration_data_t));
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Saved calibration to NVS");
    }
}

void reset_config() {
    
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_erase_key(h, NVS_CONFIG_KEY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "nvs_erase_key error");
        }
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Config resetted");
    } else {
        ESP_LOGE(TAG, "Config reset error");
    }
    config_load();
}

void reset_calibration() {
    
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_erase_key(h, NVS_CALIBRATION_KEY);
        nvs_commit(h);
        nvs_close(h);
    }
    calibration_load();
    ESP_LOGI(TAG, "Calibration resetted");
}


float apply_calibration(approx_poly_line_t polyline, float raw_value) {


    size_t length = polyline.num_points;
    data_point_t left_point=polyline.points[0], right_point=polyline.points[length-1];
    float value;

    if (raw_value<=polyline.points[0].x) {
        left_point=polyline.points[0];
        right_point=polyline.points[1];
    } else if (raw_value>=polyline.points[length-1].x) {
        left_point=polyline.points[length-2];
        right_point=polyline.points[length-1];
    } else {
        for (int i=1; i<polyline.num_points; i++) {
            if (raw_value < polyline.points[i].x) {
                left_point=polyline.points[i-1];
                right_point=polyline.points[i];
                break;
            }
        }
    }

    value=left_point.y+(right_point.y-left_point.y)/(right_point.x - left_point.x)*(raw_value-left_point.x);

    // if (isnan(value)) {value=0;}

    return value;
}


int compare_data_points(const void *a, const void *b) {

    data_point_t dp_a = *((const data_point_t*) a);
    data_point_t dp_b = *((const data_point_t*) b);

    if (dp_a.x<dp_b.x) return -1;
    if (dp_a.x>dp_b.x) return 1;
    return 0;


}




void config_set_kp(float v)    { g_config.kp = (v < 0) ? 0 : (v > 100) ? 100 : v; }
void config_set_ki(float v)    { g_config.ki = (v < 0) ? 0 : (v > 100) ? 100 : v; }
void config_set_kd(float v)    { g_config.kd = (v < 0) ? 0 : (v > 100) ? 100 : v; }
void config_set_deadzone(float v) { g_config.deadzone = (v < 0) ? 0 : (v > 1) ? 1 : v; }
void config_set_pwm_freq(uint32_t v) { g_config.pwm_freq_hz = (v < 2) ? 2 : (v > 20) ? 20 : v; }
void config_set_ema_alpha(float v) { g_config.ema_alpha = (v < 0.01f) ? 0.01f : (v > 1.0f) ? 1.0f : v; }
void calibration_set_actual_position(approx_poly_line_t p) { g_calibration.actual_position_approx = p; }
void calibration_set_desired_position(approx_poly_line_t p) {g_calibration.desired_position_approx = p;}
void calibration_set_current_loop(approx_poly_line_t p) { g_calibration.current_loop_approx = p; }

