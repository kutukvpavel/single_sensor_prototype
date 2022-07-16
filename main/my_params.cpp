#include "my_params.h"

#define MY_DAC_MAX 6.0 //V
#define MY_DAC_RESOLUTION 1024.0 //Steps
#define MY_DAC_CAL (MY_DAC_RESOLUTION / MY_DAC_MAX)
#define HEATER_COEF 0.0025
#define R4 100000.0 //Ohms
#define V_H_MON_MULT 4.0
#define V_H_OFFSET 67.0
#define CURRENT_SHUNT 2.0 //Ohms
#define CURRENT_AMPLIFICATION 2.0 //Times
#define CURRENT_OFFSET -25.35
#define I_H_MULT (1.0/(CURRENT_SHUNT*CURRENT_AMPLIFICATION))
#define OVERSAMPLING_LEN 32
#define SAMPLING_RATE 10
#define OVERSAMPLING_RATE 500

static my_adc_cal_t cals[MY_ADC_CHANNEL_NUM] =
{
        {I_H_MULT, CURRENT_OFFSET},
        {V_H_MON_MULT, V_H_OFFSET},
        {1, 0},
        {1, 0}
};
static my_timings_t timings = {OVERSAMPLING_LEN, SAMPLING_RATE, OVERSAMPLING_RATE};
static float heater_coef = HEATER_COEF;
static float r4 = R4;

namespace my_params
{
    const my_dac_cal_t default_dac_cal = {MY_DAC_CAL, 0};
    const my_adc_cal_t default_adc_cal = {1, 0};

    float get_r4()
    {
        return r4;
    }
    void set_r4(float val)
    {
        r4 = val;
    }
    float get_heater_coef()
    {
        return heater_coef;
    }
    void set_heater_coef(float val)
    {
        heater_coef = val;
    }
    const my_adc_cal_t* get_adc_channel_cal(size_t index)
    {
        return &(cals[index]);
    }
    const my_dac_cal_t* get_dac_cal()
    {
        return &default_dac_cal;
    }
    const my_timings_t* get_timings()
    {
        return &timings;
    }
    void init()
    {
        //TODO: EEPROM?
    }
    void save()
    {
        //TODO
    }
}
