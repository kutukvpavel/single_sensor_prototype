#include "my_hal.h"
#include "macros.h"
#include "esp_log.h"
#include "esp_err.h"
#include <rom/ets_sys.h>
#include <driver/ledc.h>

/**
 * Configuration Section: Pin Numbers
 * 
 */

const gpio_num_t pin_btn = GPIO_NUM_41;
const gpio_num_t pin_vent = GPIO_NUM_42;
const gpio_num_t pin_usb = GPIO_NUM_10;
const gpio_num_t pin_charger_mon = GPIO_NUM_47;
const gpio_num_t pin_charge_enable = GPIO_NUM_38;
const gpio_num_t pin_oe = GPIO_NUM_14;
const gpio_num_t pin_led = GPIO_NUM_40;
const gpio_num_t pin_buzzer = GPIO_NUM_39;
const gpio_num_t pin_shdn_en = GPIO_NUM_17;
const gpio_num_t pin_shdn_clk = GPIO_NUM_18;
const gpio_num_t pin_p6v_mon = GPIO_NUM_8;
const gpio_num_t pin_n6v_mon = GPIO_NUM_48;
const gpio_num_t pin_bat_mon_enable = GPIO_NUM_21;

/**
 * @brief Shift Registers
 * 
 */
struct my_sr
{
    gpio_num_t d;
    gpio_num_t clk;
    gpio_num_t latch;
    bool msb_first; //Bit order
    size_t len; //Bytes
};

const my_sr regs[] = 
{
    { GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_13, true, 2 }
};

/**
 * PWM channels
 * 
 */

#define BUZZER_TIM LEDC_TIMER_0
#define BUZZER_CH LEDC_CHANNEL_0
ledc_timer_config_t buzzer_timer =
{
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_1_BIT,
    .timer_num = BUZZER_TIM,
    .freq_hz = 3000,
    .clk_cfg = LEDC_AUTO_CLK
};
ledc_channel_config_t buzzer_channel =
{
    .gpio_num = pin_buzzer,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = BUZZER_CH,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = BUZZER_TIM,
    .duty = 1,
    .hpoint = 0
};

#define SHDN_TIM LEDC_TIMER_1
#define SHDN_CH LEDC_CHANNEL_1
ledc_timer_config_t shdn_timer =
{
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_1_BIT,
    .timer_num = SHDN_TIM,
    .freq_hz = 30000,
    .clk_cfg = LEDC_AUTO_CLK
};
ledc_channel_config_t shdn_channel =
{
    .gpio_num = pin_shdn_clk,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = SHDN_CH,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = SHDN_TIM,
    .duty = 1,
    .hpoint = 0
};

struct pwm_config
{
    ledc_timer_config_t* tim;
    ledc_channel_config_t* ch;
};
enum pwm_types
{
    PWM_BUZZER,
    PWM_SHDN
};
const pwm_config pwms[] = 
{
    { &buzzer_timer, &buzzer_channel },
    { &shdn_timer, &shdn_channel }
};

/**
 * ADC
 * 
 */

my_adc_channel channels[MY_ADC_CHANNEL_NUM] =
    {
        my_adc_channel(ADC1_CHANNEL_3, ADC_ATTEN_DB_0, "I_h"),
        my_adc_channel(ADC1_CHANNEL_4, ADC_ATTEN_DB_6, "V_h_mon"),
        my_adc_channel(ADC1_CHANNEL_8, ADC_ATTEN_DB_0, "V_r4"),
        my_adc_channel(ADC1_CHANNEL_9, ADC_ATTEN_DB_0, "V_div"),
        my_adc_channel(ADC1_CHANNEL_0, ADC_ATTEN_DB_6, "Bat_V"),
        my_adc_channel(ADC1_CHANNEL_1, ADC_ATTEN_DB_6, "Bat_T")
    };

/**
 * Decls
 */

void pwm_enable_internal(pwm_types t, bool v);

/**
 * PUBLIC routines
 * 
 */

namespace my_hal
{
    bool init(uint buzzer_freq, uint shdn_pump_freq, const my_adc_cal_t* (*get_adc_cal)(size_t))
    {
        const uint32_t zero = 0;
        const uint8_t* const zero_ptr = reinterpret_cast<const uint8_t*>(&zero);

        //Set shift register pins as outputs and load all zeros
        for (size_t i = 0; i < ARRAY_SIZE(regs); i++)
        {
            gpio_set_direction(regs[i].d, GPIO_MODE_OUTPUT);
            gpio_set_direction(regs[i].clk, GPIO_MODE_OUTPUT);
            gpio_set_direction(regs[i].latch, GPIO_MODE_OUTPUT);
            assert(regs[i].len <= sizeof(zero));
            sr_write(static_cast<sr_types>(i), zero_ptr);
        }
        
        //Init PWM channels
        if (buzzer_freq > 500 && buzzer_freq < 10000) pwms[pwm_types::PWM_BUZZER].tim->freq_hz = buzzer_freq;
        if (shdn_pump_freq > 1000 && shdn_pump_freq < 100000) pwms[pwm_types::PWM_SHDN].tim->freq_hz = shdn_pump_freq;
        for (size_t i = 0; i < ARRAY_SIZE(pwms); i++)
        {
            ESP_ERROR_CHECK(ledc_timer_config(pwms[i].tim));
            ESP_ERROR_CHECK(ledc_channel_config(pwms[i].ch)); //This starts signal generation
            ledc_timer_pause(LEDC_LOW_SPEED_MODE, pwms[i].tim->timer_num);
        }

        //ADC
        static_assert(MY_ADC_SENSING_CHANNEL_NUM <= MY_ADC_CHANNEL_NUM);
        static_assert(MY_ADC_CHANNEL_NUM <= ARRAY_SIZE(channels));
        ESP_ERROR_CHECK(adc1_config_width(ADC_BITS));
        bool init_ok = true;
        for (size_t i = 0; i < MY_ADC_CHANNEL_NUM; i++)
        {
            init_ok = init_ok && channels[i].init(get_adc_cal(i));
        }
        return init_ok;
    }

    adc_code_t adc_read_channel(adc_channel_types t)
    {
        assert(t < ARRAY_SIZE(channels));
        return channels[t].get_value();
    }
    void sr_write(sr_types t, const uint8_t* contents)
    {
        const size_t byte_len = 8;
        assert(t < ARRAY_SIZE(regs));
        static_assert(sizeof(adc_code_t) == 2, "Warning: check DAC shift register length!");

        auto sr = regs[t];
        gpio_set_level(sr.latch, 0);
        for (size_t i = 0; i < sr.len; i++)
        {
            for (size_t j = 0; j < byte_len; j++)
            {
                uint32_t mask = 1u << (sr.msb_first ? (byte_len - 1 - j) : j);
                gpio_set_level(sr.clk, 0);
                gpio_set_level(sr.d, (contents[sr.msb_first ? (sr.len - 1 - i) : i] & mask) > 0);
                ets_delay_us(1);
                gpio_set_level(sr.clk, 1);
                ets_delay_us(1);
            }
        }
        gpio_set_level(sr.latch, 1);
    }
    bool get_btn_pressed()
    {
        return gpio_get_level(pin_btn) == 0; //Active LOW
    }
    bool get_vent_closed()
    {
        return gpio_get_level(pin_vent) == 0; //Active LOW (?)
    }
    bool get_usb_detected()
    {
        return gpio_get_level(pin_usb) > 0;
    }
    bool get_charger_emergency()
    {
        return gpio_get_level(pin_charger_mon) > 0;
    }
    bool get_p6v_mon()
    {
        return gpio_get_level(pin_p6v_mon) > 0;
    }
    bool get_n6v_mon()
    {
        return gpio_get_level(pin_n6v_mon) > 0;
    }

    void set_charge_enable(bool v)
    {
        gpio_set_level(pin_charge_enable, !v); //Active LOW
    }
    void set_output_enable(bool v)
    {
        gpio_set_level(pin_oe, !v); //Active LOW
    }
    void set_led_enabled(bool v)
    {
        gpio_set_level(pin_led, v);
    }
    void set_buzzer_enable(bool v)
    {
        pwm_enable_internal(pwm_types::PWM_BUZZER, v);
    }
    void set_shutdown_analog(bool v)
    {
        gpio_set_level(pin_shdn_en, !v); //PNP inverter
        pwm_enable_internal(pwm_types::PWM_SHDN, v);
    }
    void set_battery_monitor_enable(bool v)
    {
        gpio_set_level(pin_bat_mon_enable, v);
    }
}

/**
 * PRIVATE routines
 * 
 */

void pwm_enable_internal(pwm_types t, bool v)
{
    if (v)
    {
        ledc_timer_resume(LEDC_LOW_SPEED_MODE, pwms[t].tim->timer_num);
    }
    else
    {
        ledc_timer_pause(LEDC_LOW_SPEED_MODE, pwms[t].tim->timer_num);
    }
}