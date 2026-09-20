#include "spin_base.hpp"

namespace motion_generator
{
SpinBase::SpinBase(const YAML::Node& config)
{
    number_of_sine_functions_ = config["number_of_sine_functions"] ? config["number_of_sine_functions"].as<int>() : 1;
    
    // 初始化正弦函数
    sine_functions_.reserve(number_of_sine_functions_);
    A_lists_.reserve(number_of_sine_functions_);
    f_lists_.reserve(number_of_sine_functions_);
    phi_lists_.reserve(number_of_sine_functions_);
    x_lists_.reserve(number_of_sine_functions_);
    
    for (int i = 0; i < number_of_sine_functions_; i++)
    {   
        tools::SineFunction sine_function{A_lists_[i], f_lists_[i], phi_lists_[i], x_lists_[i]};
        sine_functions_.emplace_back(sine_function);
    }
}
} // namespace motion_generator