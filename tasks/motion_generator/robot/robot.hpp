#ifndef _ROBOT_HPP_
#define _ROBOT_HPP_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "tools/math_tools/math_tools.hpp"

namespace motion_generator
{
struct Armor
{
    const double center_pitch = 15; // 装甲板到旋转中心的pitch上的偏移

    int id;                         // 当前装甲板id
    Eigen::Vector3d tvec;           // 世界系xyz坐标
    Eigen::Vector3d rvec;           // 世界系ypr

    Eigen::Vector4d ypda;           // 观测值

    bool get_observation(const Eigen::Vector3d& center_in_world, const double radius, const double yaw);
};

class Robot
{
public:
    
private:
    std::vector<Armor> armor_;
};
} // namespace motion_generator

#endif // _ROBOT_HPP_