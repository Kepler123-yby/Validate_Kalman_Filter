#include "robot.hpp"

namespace motion_generator
{

bool Armor::get_observation(const Eigen::Vector3d& center_in_world, const double radius, const double yaw)
{
    try
    {
        tvec = center_in_world + Eigen::Vector3d(radius * std::cos(yaw), radius * std::sin(yaw), 0);
        rvec = Eigen::Vector3d(yaw, center_pitch, 0);
        ypda = Eigen::Vector4d(tvec(0), tvec(1), tvec(2), rvec(0));
    }
    catch (const std::exception& e)
    {
        return false;
    }

    return true;
}

} // namespace motion_generator