#include "generator.hpp"

namespace motion_generator
{

Generator::Generator(const YAML::Node& config)
{
    int motion_id = config["motion_type"].as<int>();
    MotionType motion_type = static_cast<MotionType>(motion_id);

    if(init_motion_generator())
    {
        tools::logger()->info("Motion generator initialized successfully.");
    } else {
        tools::logger()->error("Failed to initialize motion generator.");
    }
}



} // namespace motion_generator