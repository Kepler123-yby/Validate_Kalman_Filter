#ifndef _SPIN_BASE_HPP_
#define _SPIN_BASE_HPP_

#include <chrono>
#include <vector>

#include "tools/sine_fuction/sine_fuction.hpp"
#include <yaml-cpp/yaml.h>

namespace motion_generator
{

struct SpinState 
{
    double speed{0};
    double yaw{0};
    double acceleration{0};
    double forword_radius{0};
    double beside_radius{0};
    double height_diff{0};
};


class SpinGenerator
{
public:
    explicit SpinGenerator(const YAML::Node& config);
    SpinGenerator() = default;
    ~SpinGenerator() = default;

    SpinState get_states(const std::chrono::steady_clock::time_point& time) const;

private:
    int number_of_sine_functions_; // 正弦函数的数量
    std::vector<tools::SineFunction> sine_functions_;

    // 正弦函数运动参数
    std::vector<double> A_lists_;
    std::vector<double> f_lists_;
    std::vector<double> phi_lists_;
    std::vector<double> x_lists_;

    // 物理尺寸
    double forword_radius_;
    double beside_radius_;
    double height_diff_;

    std::chrono::steady_clock::time_point start_time_;

private:
    SpinState evaluate(double elapsed_seconds) const;
};

} // namespace motion_generator

#endif // _SPIN_BASE_HPP_
