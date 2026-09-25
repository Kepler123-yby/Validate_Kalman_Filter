#ifndef _SINE_FUNCTION_HPP_
#define _SINE_FUNCTION_HPP_

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

    SineFunction(
        double amplitude,
        double frequency,
        double phase,
        double offset);

    double evaluate(double elapsed_seconds) const;

    double derivative(double elapsed_seconds) const;

    double second_derivative(double elapsed_seconds) const;

    // Integral of angular velocity from 0 to elapsed_seconds.
    double integral(double elapsed_seconds) const;
};

} // namespace tools

#endif // _SINE_FUNCTION_HPP_
