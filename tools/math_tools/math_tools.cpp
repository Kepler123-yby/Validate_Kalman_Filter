/**
 * @file math_tools.cpp
 * @brief @ref math_tools.hpp 中声明的数学与时间工具函数的实现。
 */

#include "math_tools.hpp"

#include <cmath>

namespace tools
{

double normalize_angle(const double angle)
{
    // std::remainder 将结果限制在 [-pi, pi]；再利用条件分支排除正 pi 端点，
    // 使最终区间严格为 [-pi, pi)。
    const double wrapped = std::remainder(angle, 2.0 * M_PI);
    return wrapped >= M_PI ? wrapped - 2.0 * M_PI : wrapped;
}

double delta_time(
    const std::chrono::steady_clock::time_point& start,
    const std::chrono::steady_clock::time_point& end)
{
    return std::chrono::duration_cast<std::chrono::duration<double>>(end - start)
        .count();
}

double angle_difference(const double from, const double to)
{
    return normalize_angle(to - from);
}

double radians_to_degrees(const double radians)
{
    return radians * 180.0 / M_PI;
}

Eigen::Vector3d cartesian_to_ypd(const Eigen::Vector3d& xyz)
{
    const double x = xyz[0];
    const double y = xyz[1];
    const double z = xyz[2];

    const double yaw = std::atan2(y, x);
    const double distance_2d = std::hypot(x, y);
    const double pitch = std::atan2(z, distance_2d);
    const double distance = std::hypot(distance_2d, z);

    return Eigen::Vector3d(yaw, pitch, distance);
}

} // namespace tools
