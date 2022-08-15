#pragma once

struct my_pid_params_t
{
    float kI; // integral term coef
    float limI; // integral term limit
    float kPE; // proportional on error
    float kPD; // proportional on power dissipation (absolute temp)
    float setpoint_tolerance; // ignore setpoint adjustments less than this value
    float timing_factor; // scaling factor for I-term coef (in case timescale changes)
    float ambient_temp; // for kPD
};

class my_pid // Actually, a PI (yet)
{
private:
    const my_pid_params_t* params;
    float last_setpoint;
    float integral_term;
public:
    my_pid(const my_pid_params_t* p);
    //void init(const my_pid_params_t* p);
    float next(float current_temp); // Returns next power setting
    void set(float setpoint); // Temperature in Kelvin
    float get_setpoint();
};