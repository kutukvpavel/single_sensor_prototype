#include "my_pid.h"
#include "my_uart.h"

#include <math.h>

my_pid::my_pid(const my_pid_params_t* p)
{
    params = p;
}

float my_pid::next(float current_temp)
{
    float e = last_setpoint - current_temp;
    integral_term += e * params->timing_factor;
    if (integral_term > params->limI) integral_term = params->limI;
    float res = params->kPE * e + params->kPD * (current_temp - params->ambient_temp) + params->kI * integral_term;
    if (!isfinite(res))
    {
        res = 0;
        my_uart::raise_error(my_error_codes::heater);
    }
    else if (res < 0)
    {
        res = 0;
    }
    return res;
}

void my_pid::set(float setpoint)
{
    if (abs(setpoint - last_setpoint) < params->setpoint_tolerance) return;
    if (setpoint < last_setpoint) integral_term = 0;
    last_setpoint = setpoint;
}

float my_pid::get_setpoint()
{
    return last_setpoint;
}