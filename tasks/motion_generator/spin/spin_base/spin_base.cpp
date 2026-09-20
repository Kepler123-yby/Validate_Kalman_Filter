#include "spin_base.hpp"

namespace motion_generator
{
SpinBase::SpinBase(const YAML::Node& config)
{
    number_of_sine_functions_ = config["number_of_sine_functions"] ? config["number_of_sine_functions"].as<int>() : 1;
    
}
} // namespace motion_generator