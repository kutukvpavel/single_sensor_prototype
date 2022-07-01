#pragma once

#include <inttypes.h>

namespace my_dac
{
    void init(float cal);
    void set(float volt);
    float get();
}