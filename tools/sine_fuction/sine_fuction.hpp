#ifndef _SINE_FUNCTION_HPP_
#define _SINE_FUNCTION_HPP_

#include <chrono>
#include <cmath>

namespace tools
{

// y(t) = A * sin(f * t + phi) + x
// f is angular frequency in rad/s and t is elapsed time in seconds.
struct SineFunction
{
    double A;   // amplitude
    double f;   // angular frequency, rad/s
    double phi; // initial phase, rad
    double x;   // DC offset

    SineFunction(double amplitude, double frequency, double phase, double offset)
        : A(amplitude), f(frequency), phi(phase), x(offset) {}

    double evaluate(double elapsed_seconds) const
    {
        return A * std::sin(f * elapsed_seconds + phi) + x;
    }

    double evaluate(const std::chrono::steady_clock::time_point& time) const
    {
        const double t = std::chrono::duration<double>(
            time.time_since_epoch()).count();
        return evaluate(t);
    }

    double derivative(double elapsed_seconds) const
    {
        return A * f * std::cos(f * elapsed_seconds + phi);
    }

    double derivative(const std::chrono::steady_clock::time_point& time) const
    {
        const double t = std::chrono::duration<double>(
            time.time_since_epoch()).count();
        return derivative(t);
    }

    // Kept for compatibility with the existing spin generator. This method
    // historically returned the derivative, despite being named integral.
    double integral(const std::chrono::steady_clock::time_point& time) const
    {
        return derivative(time);
    }
};

} // namespace tools

#endif // _SINE_FUNCTION_HPP_
