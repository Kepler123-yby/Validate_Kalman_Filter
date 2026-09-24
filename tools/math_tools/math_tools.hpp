#ifndef _MATH_TOOLS_HPP_
#define _MATH_TOOLS_HPP_

#include <Eigen/Dense>

#include <chrono>
#include <cmath>

namespace tools
{

double limit_euler(const double angle);

double delta_time(const std::chrono::steady_clock::time_point& start_time, const std::chrono::steady_clock::time_point& end_time);

double delta_euler(const double euler1, const double euler2);

double angle_to_euler(const double angle);

Eigen::Vector3d xyz_to_ypd(const Eigen::Vector3d& xyz);
} // namespace tools

#endif // _MATH_TOOLS_HPP_
