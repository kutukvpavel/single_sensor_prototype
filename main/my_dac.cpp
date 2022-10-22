#include "my_dac.h"
#include <esp_log.h>

#include "my_params.h"
#include "macros.h"
#include "my_hal.h"

#define MY_DAC_REF 3.0 //V
#define MY_DAC_FULL_SCALE 0x0FFF
#define MY_DAC_ZERO_SCALE 0x0000

static const char* TAG = "MY_DAC";

const my_dac_cal_t* calibration = &my_params::default_dac_cal;
float last = 0;

namespace my_dac
{
    void init(const my_dac_cal_t* cal)
    {
        calibration = cal;
    }
    void set(float volt)
    {
        if (!isfinite(volt))
        {
            ESP_LOGW(TAG, "DAC ignored infinte value: %f", volt);
            return;
        }
        last = volt;
        volt = volt * calibration->gain + 0.5 + calibration->offset;
        if (volt > MY_DAC_FULL_SCALE) volt = MY_DAC_FULL_SCALE;
        else if (volt < MY_DAC_ZERO_SCALE) volt = MY_DAC_ZERO_SCALE;
        my_hal::adc_code_t code = static_cast<my_hal::adc_code_t>(volt);
        my_hal::sr_write(my_hal::sr_types::SR_DAC, reinterpret_cast<uint8_t*>(&code));
    }
    float get()
    {
        return last;
    }
}