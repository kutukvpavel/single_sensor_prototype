#pragma once

#include <inttypes.h>

struct my_dac_cal_t
{
    float gain;
    float offset;
};

namespace my_dac
{
    void init(const my_dac_cal_t* cal);
    void set(float volt);
    float get();
}