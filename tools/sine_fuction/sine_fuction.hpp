#ifndef _SINE_FUNCTION_HPP_
#define _SINE_FUNCTION_HPP_

#include <cmath>

namespace tools
{

// w(t) = A * sin(f * t + phi) + x
// f is angular frequency in rad/s and t is elapsed time in seconds.
struct SineFunction
{
    double A;   // angular velocity amplitude, rad/s
    double f;   // angular frequency, rad/s
    double phi; // initial phase, rad
    double x;   // angular velocity offset, rad/s

    SineFunction(double amplitude, double frequency, double phase, double offset)
        : A(amplitude), f(frequency), phi(phase), x(offset) {}

    double evaluate(double elapsed_seconds) const
    {
        return A * std::sin(f * elapsed_seconds + phi) + x;
    }

    double derivative(double elapsed_seconds) const
    {
        return A * f * std::cos(f * elapsed_seconds + phi);
    }

    double second_derivative(double elapsed_seconds) const
    {
        return -A * f * f * std::sin(f * elapsed_seconds + phi);
    }

    // Integral of angular velocity from 0 to elapsed_seconds.
    double integral(double elapsed_seconds) const
    {
        if (std::abs(f) < 1e-12)
        {
            return (A * std::sin(phi) + x) * elapsed_seconds;
        }
        return A / f * (std::cos(phi)
            - std::cos(f * elapsed_seconds + phi))
            + x * elapsed_seconds;
    }

};

} // namespace tools

#endif // _SINE_FUNCTION_HPP_
