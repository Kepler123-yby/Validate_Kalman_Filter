/**
 * @file sine_function.cpp
 * @brief @ref tools::SineFunction 的实现。
 */

#include "sine_function.hpp"

#include <cmath>

namespace tools
{

SineFunction::SineFunction(
    const double amplitude,
    const double angular_frequency,
    const double phase,
    const double offset)
    : amplitude_(amplitude),
      angular_frequency_(angular_frequency),
      phase_(phase),
      offset_(offset)
{
}

double SineFunction::angular_velocity(const double elapsed_seconds) const
{
    return amplitude_ * std::sin(angular_frequency_ * elapsed_seconds + phase_)
        + offset_;
}

double SineFunction::angular_acceleration(const double elapsed_seconds) const
{
    return amplitude_ * angular_frequency_
        * std::cos(angular_frequency_ * elapsed_seconds + phase_);
}

double SineFunction::angle(const double elapsed_seconds) const
{
    // 角频率趋近于 0 时不能直接除以 f，此时角速度近似为常数。
    if (std::abs(angular_frequency_) < 1e-12)
    {
        return (amplitude_ * std::sin(phase_) + offset_) * elapsed_seconds;
    }

    return amplitude_ / angular_frequency_
        * (std::cos(phase_)
            - std::cos(angular_frequency_ * elapsed_seconds + phase_))
        + offset_ * elapsed_seconds;
}

} // namespace tools
