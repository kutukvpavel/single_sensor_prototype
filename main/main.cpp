#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

#include "my_adc_channel.h"
#include "my_dac.h"
#include "macros.h"

#define OVERSAMPLING_LEN 32
#define SAMPLING_RATE 10
#define OVERSAMPLING_RATE 500
#define MY_DAC_MAX 6.0 //V
#define MY_DAC_RESOLUTION 1024.0 //Steps
#define MY_DAC_CAL (MY_DAC_RESOLUTION / MY_DAC_MAX)

//static const char *TAG = "ADC";

enum my_adc_channels {
    i_h,
    v_h_mon,
    v_r4,
    v_div
};
static my_adc_channel channels[] = {
    my_adc_channel(ADC1_CHANNEL_3, ADC_ATTEN_DB_0, "I_h", OVERSAMPLING_LEN),
    my_adc_channel(ADC1_CHANNEL_4, ADC_ATTEN_DB_6, "V_h_mon", OVERSAMPLING_LEN),
    my_adc_channel(ADC1_CHANNEL_8, ADC_ATTEN_DB_0, "V_r4", OVERSAMPLING_LEN),
    my_adc_channel(ADC1_CHANNEL_9, ADC_ATTEN_DB_0, "V_div", OVERSAMPLING_LEN)
};

extern "C" {
    void app_main(void);
}

void app_main(void)
{
    uint counter = 0;
    float buffer[ARRAY_SIZE(channels)];

    //vTaskDelay(pdMS_TO_TICKS(1000)); //For the voltages to stabilize
    ets_delay_us(100000);

    my_adc::init();
    for (auto &&i : channels)
    {
        i.init(); //TODO: handle errors
    }
    my_dac::init(MY_DAC_CAL);

    while (1) {
        for (size_t i = 0; i < ARRAY_SIZE(buffer); i++)
        {
            buffer[i] = channels[i].get_value();
        }
        if (counter++ % (OVERSAMPLING_RATE / SAMPLING_RATE) == 0) 
        {
            printf("mV: %6.1f; %6.1f; %6.1f; %6.1f\n", 
                buffer[my_adc_channels::v_r4],
                buffer[my_adc_channels::v_div],
                buffer[my_adc_channels::v_h_mon],
                buffer[my_adc_channels::i_h]
                );
            float setpoint = my_dac::get() + 0.1;
            if (setpoint >= MY_DAC_MAX) setpoint = 0;
            my_dac::set(setpoint);
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / OVERSAMPLING_RATE));
    }
}
