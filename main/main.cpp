#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

#include "my_adc_channel.h"
#include "my_dac.h"
#include "my_uart.h"
#include "my_params.h"
#include "my_pid.h"
#include "my_dbg_menu.h"
#include "macros.h"
#include "my_model.h"

static const char *TAG = "MAIN";

extern "C" {
    void app_main(void);
}

float calc_resistance(float vr_ref, float vdiv, float r_ref)
{
    return r_ref * (vdiv - vr_ref) / vr_ref;
}

float calc_temperature(float voltage, float current, float rt_res, float rt_temp, float tempco)
{
    auto res = rt_temp + (voltage / current - rt_res) / (tempco * rt_res);
    if (res > 1000)
    {
        ESP_LOGW(TAG, "Calc temp too high: v=%f, i=%f, rt_r=%f, rt_t=%f, alpha=%f", voltage, current, rt_res, rt_temp, tempco);
    }
    return res > rt_temp ? res : rt_temp;
}

float calc_voltage(float power, float setpoint, float rt_res, float rt_temp, float tempco)
{
    float res = sqrtf(power * rt_res * (1 + tempco * (setpoint - rt_temp)));
    if (!isfinite(res))
    {
        ESP_LOGW(TAG, "Heater V is infinite: setpoint=%f, power=%f", setpoint, power);
    }
    return res;
}

void app_main(void)
{
    static uint counter = 0;
    static float buffer[ARRAY_SIZE(my_adc::channels)];

    //vTaskDelay(pdMS_TO_TICKS(1000)); //For the voltages to stabilize
    ets_delay_us(100000);

    my_params::init();
    my_uart::init();
    my_adc::init();
    my_model::init();

    static my_pid pid = my_pid(my_params::get_pid_params());

    bool init_ok = true;
    for (size_t i = 0; i < ARRAY_SIZE(my_adc::channels); i++)
    {
        init_ok = init_ok && my_adc::channels[i].init(my_params::get_adc_channel_cal(i));
    }
    if (!init_ok) my_uart::raise_error(my_error_codes::software_init);
    my_dac::init(my_params::get_dac_cal());
    my_dac::set(my_uart::first());
    auto timings = my_params::get_timings();

    ESP_LOGI(TAG, "Setup complete.");
    my_dbg_menu::init();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000 / timings->oversampling_rate));
        for (size_t i = 0; i < ARRAY_SIZE(buffer); i++)
        {
            buffer[i] = my_adc::channels[i].get_value();
        }
        if (my_uart::get_operate() || my_dbg_menu::operate)
        {
            float current_temp = calc_temperature(buffer[my_adc_channels::v_h_mon], buffer[my_adc_channels::i_h],
                my_params::get_rt_resistance(), my_params::rt_temp, my_params::get_heater_coef());
            if (counter++ % (timings->oversampling_rate / timings->sampling_rate) == 0) 
            {
                printf("mV: %6.1f; %6.1f; %6.1f (%6.1f); mA: %6.1f (%3.0f)\n", 
                    buffer[my_adc_channels::v_r4] * 1000,
                    buffer[my_adc_channels::v_div] * 1000,
                    buffer[my_adc_channels::v_h_mon] * 1000, my_dac::get() * 1000,
                    buffer[my_adc_channels::i_h] * 1000, current_temp
                    );
                pid.set(my_uart::next(current_temp, 
                    calc_resistance(buffer[my_adc_channels::v_r4], buffer[my_adc_channels::v_div], my_params::get_ref_resistance())
                    ));
            }
            float pid_next = pid.next(current_temp);
            if (!isfinite(pid_next)) ESP_LOGW(TAG, "PID is infinite: %f, %f", pid_next, current_temp);
            float commanded_voltage = calc_voltage(pid_next, pid.get_setpoint(),
                my_params::get_rt_resistance(), my_params::rt_temp, my_params::get_heater_coef());
            /*printf("Commanded: %6.1f, setpoint: %f, pwr: %f, temp: %3.0f, i=%f\n", commanded_voltage, pid.get_setpoint(), pid_next, current_temp,
                buffer[my_adc_channels::i_h] * 1000);*/
            my_dac::set(commanded_voltage);
            if (my_params::enable_pid_dbg)
            {
                my_uart::send_pid_dbg(current_temp, commanded_voltage);
            }
        }
        else
        {
            my_dac::set(0);
            pid.set(my_params::rt_temp);
            counter = 0;
        }
    }
}
