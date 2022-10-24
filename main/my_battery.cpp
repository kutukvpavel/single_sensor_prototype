#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "my_battery.h"
#include "my_hal.h"
#include "my_params.h"

enum charge_state_t
{
    allowed,
    inhibited,
    protection_tripped
};

static const char TAG[] = "Battery";
static TaskHandle_t monitoring_task_handle = NULL;

void check_protection(charge_state_t* state)
{
    if (my_hal::get_charger_emergency())
    {
        *state = charge_state_t::protection_tripped;
        ESP_LOGW(TAG, "Battery overvoltage protection has been tripped!");
    }
    float t = my_hal::adc_read_channel(my_hal::adc_channel_types::bat_t);
    if (!my_params::check_bat_t(t))
    {
        *state = charge_state_t::protection_tripped;
        ESP_LOGW(TAG, "Battery temperature protection has been tripped!");
    }
}

namespace my_battery
{
    void monitoring_task(void* arg)
    {
        while (1)
        {
            static charge_state_t state = charge_state_t::allowed;

            vTaskDelay(pdMS_TO_TICKS(1500));
            float v = my_hal::adc_read_channel(my_hal::adc_channel_types::bat_v);
            auto threshold = my_params::get_battery_threshold();
            switch (state)
            {
            case charge_state_t::allowed:
                if (my_hal::get_usb_detected()) my_hal::set_led_enabled(true);
                if (v >= threshold.high)
                {
                    my_hal::set_charge_enable(false);
                    state = charge_state_t::inhibited;
                }
                check_protection(&state);
                break;
            case charge_state_t::inhibited:
                if (my_hal::get_usb_detected()) my_hal::set_led_enabled(false);
                if (v <= threshold.low)
                {
                    my_hal::set_charge_enable(true);
                    state = charge_state_t::allowed;
                }
                check_protection(&state);
                break;
            case charge_state_t::protection_tripped:
                if (my_hal::get_usb_detected()) my_hal::set_led_enabled(false); //TODO: pulsed led HAL
                if (!my_hal::get_usb_detected())
                {
                    state = allowed;
                    check_protection(&state);
                }
                break;
            default:
                state = charge_state_t::protection_tripped;
                ESP_LOGE(TAG, "Charge state out of range!");
                break;
            }
        }
    }
    
    void init()
    {
        xTaskCreate(monitoring_task, "battery_mon", 4096, NULL, 1, &monitoring_task_handle);
        assert(monitoring_task_handle);
    }
}