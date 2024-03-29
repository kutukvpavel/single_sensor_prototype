#include "my_dac.h"
#include <driver/gpio.h>
#include <rom/ets_sys.h>
#include <esp_log.h>

#include "my_params.h"
#include "macros.h"

#define MY_DAC_REF 3.0 //V
#define MY_DAC_FULL_SCALE 0x0FFF
#define MY_DAC_ZERO_SCALE 0x0000

typedef uint16_t my_adc_code_t;

static const char* TAG = "MY_DAC";

gpio_num_t dac_pins[] = //LSB->MSB
{
    GPIO_NUM_38,
    GPIO_NUM_37,
    GPIO_NUM_11,
    GPIO_NUM_12,
    GPIO_NUM_21,
    GPIO_NUM_47,
    GPIO_NUM_35,
    GPIO_NUM_36,
    GPIO_NUM_2,
    GPIO_NUM_1
};
gpio_num_t clk_pin = GPIO_NUM_13;

const my_dac_cal_t* calibration = &my_params::default_dac_cal;
float last = 0;

namespace my_dac
{
    void init(const my_dac_cal_t* cal)
    {
        calibration = cal;
        gpio_config_t io_conf = {};
        //disable interrupt
        io_conf.intr_type = GPIO_INTR_DISABLE;
        //set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        //bit mask of the pins that you want to set
        for (auto &&i : dac_pins)
        {
            io_conf.pin_bit_mask |= (1ULL << i);  
        }
        io_conf.pin_bit_mask |= (1ULL << clk_pin);
        //disable pull-down mode
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        //disable pull-up mode
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        //configure GPIO with the given settings
        gpio_config(&io_conf);
        set(0);
        ESP_LOGI(TAG, "DAC initialized.");
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
        my_adc_code_t code = static_cast<my_adc_code_t>(volt);
        gpio_set_level(clk_pin, 0);
        for (size_t i = 0; i < ARRAY_SIZE(dac_pins); i++)
        {
            gpio_set_level(dac_pins[i], (code & (1u << i)) > 0);
        }
        ets_delay_us(1);
        gpio_set_level(clk_pin, 1);
    }
    float get()
    {
        return last;
    }
}