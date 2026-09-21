#ifndef _SINE_FUNCTION_HPP_
#define _SINE_FUNCTION_HPP_

#include <cmath>
#include <chrono>

namespace tools
{

struct SineFunction
{
    double A;   // 振幅，m/s
    double f;   // 角频率，rad/s
    double phi; // 初始相位
    double x;   // 初始位置

    SineFunction(double amplitude, double frequency, double phase, double x)
        : A(amplitude), f(frequency), phi(phase), x(x) {}

    double evaluate(double t) const
    {
        return A * std::sin(f * t + phi) + x;
    }

    double integral(const std::chrono::steady_clock::time_point& time)
    {
        auto t = std::chrono::duration<double>(time.time_since_epoch()).count();
        return A * f * std::cos(f * t + phi);
    }

    double get_all_theta(const std::chrono::steady_clock::time_point& time1, const std::chrono::steady_clock::time_point& time2)
    {
        return integral(time1) - integral(time2);
    }

};

} // namespace tools

#endif // _SINE_FUNCTION_HPP_