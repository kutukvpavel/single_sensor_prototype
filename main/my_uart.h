#pragma once

#include <stdint.h>

#define _BV(s) (1u << (s))

enum my_error_codes : uint32_t
{
    none = 0,
    heater = _BV(0),
    measure = _BV(1),
    software_init = _BV(2),
    unknown_cmd = _BV(3),
    missed_packet = _BV(4),
    uart_parser_error = _BV(5),
    incorrect_command_format = _BV(6)
};
inline my_error_codes operator|(my_error_codes a, my_error_codes b)
{
    return static_cast<my_error_codes>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
inline my_error_codes operator|=(my_error_codes a, my_error_codes b)
{
    return a | b;
}

namespace my_uart
{
    void init();
    float next(float temp, float res);
    float first();
    bool get_operate();
    void raise_error(my_error_codes err);
} // namespace my_uart
