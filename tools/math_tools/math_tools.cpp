#include "math_tools.hpp"

namespace tools
{

double limit_euler(const double angle)
{
    return std::fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

double delta_time(const std::chrono::steady_clock::time_point& start_time, const std::chrono::steady_clock::time_point& end_time)
{
    return std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
}

double delta_euler(const double euler1, const double euler2)
{
    double delta = euler2 - euler1;
    return limit_euler(delta);
}

double angle_to_euler(const double angle)
{
    return angle * 180.0 / M_PI;
}

}