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
#include "macros.h"

//static const char *TAG = "ADC";
static float rt_resistance = 0;

extern "C" {
    void app_main(void);
}

float calc_resistance(float vr4, float vdiv, float ref)
{
    return ref * (vdiv - vr4) / vr4;
}

float calc_temperature(float voltage, float current, float rt, float coef)
{
    return (voltage / current - rt) * coef;
}

void app_main(void)
{
    uint counter = 0;
    float buffer[ARRAY_SIZE(my_adc::channels)];

    //vTaskDelay(pdMS_TO_TICKS(1000)); //For the voltages to stabilize
    ets_delay_us(100000);

    my_params::init();
    my_uart::init();
    my_adc::init();
    bool init_ok = true;
    for (size_t i = 0; i < ARRAY_SIZE(my_adc::channels); i++)
    {
        init_ok = init_ok && my_adc::channels[i].init(my_params::get_adc_channel_cal(i));
    }
    if (!init_ok) my_uart::raise_error(my_error_codes::software_init);
    my_dac::init(my_params::get_dac_cal());
    my_dac::set(my_uart::first());
    auto timings = my_params::get_timings();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000 / timings->oversampling_rate));
        for (size_t i = 0; i < ARRAY_SIZE(buffer); i++)
        {
            buffer[i] = my_adc::channels[i].get_value();
        }
        if (my_uart::get_operate())
        {
            if (counter++ % (timings->oversampling_rate / timings->sampling_rate) == 0) 
            {
                printf("mV: %6.1f; %6.1f; %6.1f; %6.1f\n", 
                    buffer[my_adc_channels::v_r4],
                    buffer[my_adc_channels::v_div],
                    buffer[my_adc_channels::v_h_mon],
                    buffer[my_adc_channels::i_h]
                    );
                my_dac::set(my_uart::next(
                    calc_temperature(buffer[my_adc_channels::v_h_mon], buffer[my_adc_channels::i_h], rt_resistance, my_params::get_heater_coef()), 
                    calc_resistance(buffer[my_adc_channels::v_r4], buffer[my_adc_channels::v_div], my_params::get_r4())
                    ));
            }
        }
        else
        {
            my_dac::set(0);
            counter = 0;
        }
    }
}
