#ifndef _SINE_FUNCTION_HPP_
#define _SINE_FUNCTION_HPP_

#include <cmath>

namespace tools
{

struct SineFunction
{
    double A;   // 振幅，m/s
    double f;   // 角频率，rad/s
    double phi; // 初始相位

    SineFunction(double amplitude, double frequency, double phase)
        : A(amplitude), f(frequency), phi(phase) {}

    double evaluate(double t) const
    {
        return A * std::sin(f * t + phi);
    }

};

} // namespace tools

#endif // _SINE_FUNCTION_HPP_