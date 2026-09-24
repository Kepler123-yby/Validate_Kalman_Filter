#include "math_tools.hpp"
#include <cmath>
#include <Eigen/Dense>

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

Eigen::Vector3d xyz_to_ypd(const Eigen::Vector3d& xyz)
{
    double x = xyz[0], y = xyz[1], z = xyz[2];
    double yaw = std::atan2(y, x);
    double distance_2d = std::hypot(x, y);
    double pitch = std::atan2(z, distance_2d);
    double distance = std::hypot(distance_2d, z);
    Eigen::Vector3d ypd = Eigen::Vector3d(yaw, pitch, distance);
    return ypd;
}

} // namespace tools
