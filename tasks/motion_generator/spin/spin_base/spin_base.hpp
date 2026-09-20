#ifndef _SPIN_BASE_HPP_
#define _SPIN_BASE_HPP_

#include <vector>

#include "tools/sine_fuction/sine_fuction.hpp"
#include <yaml-cpp/yaml.h>

namespace motion_generator
{

class SpinBase
{
public:
    SpinBase(const YAML::Node& config);
    virtual ~SpinBase() = default;
private:
    int number_of_sine_functions_; // 正弦函数的数量
    std::vector<tools::SineFunction> sine_functions_;
};

} // namespace motion_generator

#endif // _SPIN_BASE_HPP_