#ifndef _ROBOT_HPP_
#define _ROBOT_HPP_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "tools/math_tools/math_tools.hpp"
#include "tools/sine_fuction/sine_fuction.hpp"

namespace motion_generator
{
struct Armor
{
    const double center_pitch = 15; // 装甲板到旋转中心的pitch上的偏移

    int id_;                         // 当前装甲板id
    Eigen::Vector3d tvec_;           // 世界系xyz坐标
    Eigen::Vector3d rvec_;           // 世界系ypr

    Eigen::Vector4d ypda_;           // 观测值

    bool get_observation(const int id, const Eigen::Vector3d& center_in_world, const double radius, const double height_diff, const double yaw);
};

class Robot
{
public:
    Robot(const double x, const double y, const double z, const double radius_short, const double radius_long, const double, const int armor_num);
    ~Robot() = default;

    void get_states(const cv::Mat& trans_mat, const Eigen::VectorXd& states) const;

    void get_observation(const cv::Mat& trans_mat) const;
private:
    Eigen::Vector3d center_location_;
    std::chrono::steady_clock::time_point start_time_;
    double radius_short_;
    double radius_long_;
    double height_diff_;
    std::vector<Armor> armors_;
    std::vector<tools::SineFunction> sine_function_;

private:
    double calculate_spin_state() const;
};
} // namespace motion_generator

#endif // _ROBOT_HPP_