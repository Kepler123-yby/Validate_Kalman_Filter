#include "sine_fuction.hpp"

#include <cmath>

namespace tools
{

SineFunction::SineFunction(
    double amplitude,
    double frequency,
    double phase,
    double offset)
    : A(amplitude), f(frequency), phi(phase), x(offset)
{
}

double SineFunction::evaluate(double elapsed_seconds) const
{
    return A * std::sin(f * elapsed_seconds + phi) + x;
}

double SineFunction::derivative(double elapsed_seconds) const
{
    return A * f * std::cos(f * elapsed_seconds + phi);
}

double SineFunction::second_derivative(double elapsed_seconds) const
{
    return -A * f * f * std::sin(f * elapsed_seconds + phi);
}

double SineFunction::integral(double elapsed_seconds) const
{
    if (std::abs(f) < 1e-12)
    {
        return (A * std::sin(phi) + x) * elapsed_seconds;
    }

    return A / f * (
        std::cos(phi) - std::cos(f * elapsed_seconds + phi))
        + x * elapsed_seconds;
}

} // namespace tools
