#ifndef _ROBOT_HPP_
#define _ROBOT_HPP_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>

#include "tools/math_tools/math_tools.hpp"
#include "tools/sine_fuction/sine_fuction.hpp"

namespace motion_generator
{

struct Armor
{
    int id_;
    Eigen::VectorXd observation;

    Armor(const int id);
};

class Robot
{
public:
    Robot() = default;
    Robot(const YAML::Node&);
    ~Robot() = default;

    // 禁止拷贝
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    Eigen::VectorXd get_states() { return this->states_; };

    const Eigen::VectorXd& get_observation() const;
    Eigen::VectorXd& get_observation();

private:
    std::vector<Armor> armors_;
    
    Eigen::VectorXd states_;
    int locked_id_{0};

    cv::Mat raw2states_mat_;
    cv::Mat raw2observation_mat_;

private:
    // raw_states: x vx ax y vy ay z vz az angle w aw r dl height 
    void raw2states(const Eigen::VectorXd& raw_states);

    void raw2observation(const Eigen::VectorXd& raw_states);

};

} // namespace motion_generator

#endif // _ROBOT_HPP_