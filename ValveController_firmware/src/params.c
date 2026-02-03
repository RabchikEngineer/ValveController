#include "params.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "PARAMS";
static const char *NVS_NAMESPACE = "params";

params_t g_params = {
    .kp = 2.0f,
    .ki = 1.0f,
    .kd = 2.0f,
    .deadzone = 0.01f,
    .pwm_freq_hz = 10,
    .ema_alpha = 0.3f
};

calibration_data_t g_calibration = {

    .actual_position_approx =  {.num_points=2,.points=(data_point_t[]){{0,0},{4096,1}}},
    .desired_position_approx = {.num_points=2,.points=(data_point_t[]){{0,0},{4096,1}}},
    .current_loop_approx =     {.num_points=2,.points=(data_point_t[]){{0,820},{1, 4096}}}

};


void nvs_init() 
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}


void params_load(void)
{   
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_OK) {
        size_t sz = sizeof(params_t);
        nvs_get_blob(h, "config", &g_params, &sz);
        nvs_close(h);
        ESP_LOGI(TAG, "Loaded: Kp=%.3f Ki=%.3f Kd=%.3f", g_params.kp, g_params.ki, g_params.kd);
    } else {
        ESP_LOGW(TAG, "NVS open failed, using defaults");
    }
}

void params_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        nvs_set_blob(h, "config", &g_params, sizeof(params_t));
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Saved to NVS");
    }
}

void params_set_kp(float v)    { g_params.kp = (v < 0) ? 0 : (v > 100) ? 100 : v; }
void params_set_ki(float v)    { g_params.ki = (v < 0) ? 0 : (v > 100) ? 100 : v; }
void params_set_kd(float v)    { g_params.kd = (v < 0) ? 0 : (v > 100) ? 100 : v; }
void params_set_deadzone(float v) { g_params.deadzone = (v < 0) ? 0 : (v > 1) ? 1 : v; }
void params_set_pwm_freq(uint32_t v) { g_params.pwm_freq_hz = (v < 1) ? 1 : (v > 100) ? 100 : v; }
void params_set_ema_alpha(float v) { g_params.ema_alpha = (v < 0.01f) ? 0.01f : (v > 1.0f) ? 1.0f : v; }
