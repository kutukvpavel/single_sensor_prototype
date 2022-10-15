#include "my_params.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"
#include "esp_log.h"

#define MY_DAC_MAX 6.0 //V
#define MY_DAC_RESOLUTION 1024.0 //Steps
#define MY_DAC_CAL (MY_DAC_RESOLUTION / MY_DAC_MAX)
#define HEATER_COEF 0.0025
#define R4 100000.0 //Ohms
#define V_H_MON_MULT 4.0
#define V_H_OFFSET 0.000
#define CURRENT_SHUNT 2.0 //Ohms
#define CURRENT_AMPLIFICATION 1.95 //Times
#define CURRENT_OFFSET -0.02535
#define I_H_MULT (1.0/(CURRENT_SHUNT*CURRENT_AMPLIFICATION))
#define OVERSAMPLING_LEN 32
#define SAMPLING_RATE 10
#define OVERSAMPLING_RATE 500

static const char TAG[] = "NVS";

struct my_param_storage
{
    my_adc_cal_t adc_cals[MY_ADC_CHANNEL_NUM];
    my_dac_cal_t dac_cal;
    my_timings_t timings;
    float heater_coef;
    float ref_res;
    float rt_res;
    my_pid_params_t pid_params;
};
static const char storage_nvs_id[] = "storage";
static const char storage_nvs_namespace[] = "my";
my_param_storage storage = 
{
    .adc_cals = {
        {I_H_MULT, CURRENT_OFFSET},
        {V_H_MON_MULT, V_H_OFFSET},
        my_params::default_adc_cal,
        my_params::default_adc_cal
    },
    .dac_cal = my_params::default_dac_cal,
    .timings = {OVERSAMPLING_LEN, SAMPLING_RATE, OVERSAMPLING_RATE},
    .heater_coef = HEATER_COEF,
    .ref_res = R4,
    .rt_res = 10,
    .pid_params = {
        .kI = 0,
        .limI = 1,
        .kPE = 0.1,
        .kPD = 0.00,
        .setpoint_tolerance = 1,
        .timing_factor = 1.0f / OVERSAMPLING_RATE,
        .ambient_temp = my_params::rt_temp + 25
    }
};

namespace my_params
{
    const my_dac_cal_t default_dac_cal = {MY_DAC_CAL, 0};
    const my_adc_cal_t default_adc_cal = {1, 0};
    const float rt_temp = 273;

    bool enable_pid_dbg = false;

    float get_ref_resistance()
    {
        return storage.ref_res;
    }
    void set_ref_resistance(float val)
    {
        storage.ref_res = val;
    }
    float get_heater_coef()
    {
        return storage.heater_coef;
    }
    void set_heater_coef(float val)
    {
        storage.heater_coef = val;
    }
    float get_rt_resistance()
    {
        return storage.rt_res;
    }
    void set_rt_resistance(float val, float temp)
    {
        storage.rt_res = val / (1 + get_heater_coef() * (temp - rt_temp));
    }
    const my_adc_cal_t* get_adc_channel_cal(size_t index)
    {
        return &(storage.adc_cals[index]);
    }
    void set_adc_channel_cal(size_t index, my_adc_cal_t* c)
    {
        storage.adc_cals[index] = *c;
    }
    const my_dac_cal_t* get_dac_cal()
    {
        return &(storage.dac_cal);
    }
    void set_dac_cal(my_dac_cal_t* c)
    {
        storage.dac_cal = *c;
    }
    const my_timings_t* get_timings()
    {
        return &(storage.timings);
    }
    const my_pid_params_t* get_pid_params()
    {
        return &(storage.pid_params);
    }
    void set_pid_params(my_pid_params_t* p)
    {
        storage.pid_params = *p;
    }
    esp_err_t open_helper(nvs_handle_t* handle, nvs_open_mode_t mode)
    {
        esp_err_t err = nvs_open("my", mode, handle);
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGW(TAG, "NVS namespace doesn't exist and will be created (first run?)");
            err = nvs_open(storage_nvs_namespace, NVS_READWRITE, handle); // retry with write permissions
        }
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(TAG, "NVS handle opening SUCCESS.");
        }
        return err;
    }
    esp_err_t save_helper(nvs_handle_t handle, my_param_storage* val)
    {
        return nvs_set_blob(handle, storage_nvs_id, val, sizeof(my_param_storage));
    }
    esp_err_t init()
    {
        // Initialize NVS
        ESP_LOGI(TAG, "NVS Init...");
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_LOGW(TAG, "NVS had been truncated and had to be erased! Retrying...");
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);

        // Open
        nvs_handle_t handle;
        err = open_helper(&handle, NVS_READONLY);
        if (err != ESP_OK) return err;
        // Read the size of memory space required for blob
        my_param_storage tmp;
        size_t required_size = sizeof(tmp); // value will default to sizeof(tmp), if not set yet in NVS
        err = nvs_get_blob(handle, storage_nvs_id, &tmp, &required_size);
        if (err == ESP_ERR_NVS_NOT_FOUND) 
        {
            save_helper(handle, &storage); //If not found, write defaults
            ESP_LOGW(TAG, "NVS reset to defaults.");
            return ESP_OK; 
        }
        else if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error reading NVS: %s", esp_err_to_name(err));
            return err;
        }
        storage = tmp;

        nvs_close(handle);
        return ESP_OK;
    }
    esp_err_t save()
    {
        nvs_handle_t handle;
        esp_err_t err = open_helper(&handle, NVS_READWRITE);
        if (err != ESP_OK) return err;
        return save_helper(handle, &storage);
    }
    uint8_t* get_nvs_dump(size_t* len)
    {
        *len = sizeof(storage);
        return reinterpret_cast<uint8_t*>(&storage);
    }
    esp_err_t factory_reset()
    {
        nvs_handle_t handle;
        auto err = open_helper(&handle, NVS_READWRITE);
        if (err != ESP_OK) return err;
        return nvs_erase_key(handle, storage_nvs_id);
    }
}
