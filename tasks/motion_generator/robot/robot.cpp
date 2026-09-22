#include "robot.hpp"

namespace motion_generator
{

bool Armor::get_observation(const int id, const Eigen::Vector3d& center_in_world, const double radius, const double height_diff, const double yaw)
{
    try
    {
        id_ = id;
        tvec_ = center_in_world + Eigen::Vector3d(radius * std::cos(yaw), radius * std::sin(yaw), 0);
        rvec_ = Eigen::Vector3d(yaw, center_pitch, 0);
        ypda_ = Eigen::Vector4d(tvec_(0), tvec_(1), tvec_(2), rvec_(0));
    }
    catch (const std::exception& e)
    {
        return false;
    }

    return true;
}

Robot::Robot(const double x, const double y, const double z, const double radius_short, const double radius_long, const double height_diff, const int armor_num = 4)
: center_location_(x, y, z), radius_short_(radius_short), radius_long_(radius_long), height_diff_(height_diff)
{
    armors_.reserve(armor_num);
    for (int i = 0; i < armor_num; ++i)
    {
        double yaw = (i - 1) * M_PI / 2;
        Armor armor;
        double radius = i % 2 == 0 ? radius_short_ : radius_long_;
        armor.get_observation(i, center_location_, radius, height_diff_, yaw);
        armors_.push_back(armor);
    }
    start_time_ = std::chrono::steady_clock::now();
}

void Robot::get_states(const cv::Mat& trans_mat, const Eigen::VectorXd& states) const
{
    auto current_time = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(start_time_, current_time);
    
}

} // namespace motion_generator