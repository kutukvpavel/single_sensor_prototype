#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "my_adc_channel.h"

#define OVERSAMPLING_LEN 32

//static const char *TAG = "ADC";

enum my_adc_channels {
    v_r4,
    v_div,
    v_h_mon,
    i_h
};
static my_adc_channel channels[] = {
    my_adc_channel(ADC1_CHANNEL_3, ADC_ATTEN_DB_0, "V_r4", OVERSAMPLING_LEN),
    my_adc_channel(ADC1_CHANNEL_4, ADC_ATTEN_DB_0, "V_div", OVERSAMPLING_LEN),
    my_adc_channel(ADC1_CHANNEL_5, ADC_ATTEN_DB_6, "V_h_mon", OVERSAMPLING_LEN),
    my_adc_channel(ADC1_CHANNEL_6, ADC_ATTEN_DB_0, "I_h", OVERSAMPLING_LEN)
};

extern "C" {
    void app_main(void);
}

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

void app_main(void)
{
    uint counter = 0;
    float buffer[ARRAY_SIZE(channels)];

    my_adc_init();
    for (auto &&i : channels)
    {
        i.init();
    }

    while (1) {
        for (size_t i = 0; i < ARRAY_SIZE(buffer); i++)
        {
            buffer[i] = channels[i].get_value();
        }
        if (counter++ % 10 == 0) printf("mV: %6.1f; %6.1f; %6.1f; %6.1f\n", 
            buffer[my_adc_channels::v_r4],
            buffer[my_adc_channels::v_div],
            buffer[my_adc_channels::v_h_mon],
            buffer[my_adc_channels::i_h]
            );
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
