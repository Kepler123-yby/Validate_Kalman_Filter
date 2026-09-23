#include "robot.hpp"

namespace motion_generator
{
Armor::Armor(const int id)
{
    id_ = id;
    observation = Eigen::VectorXd::Zero();
}

Robot::Robot(const YAML::Node& config)
{
    auto robot_config_ = config["robot"];
    
    int armor_nums = robot_config_["armor_nums"] ? robot_config_["armor_nums"].as<int>() : 4;
    armors_.reserve(armor_nums);
    for (int i = 0; i < armor_nums; ++i)
    {
        armors_.emplace_back(i);
    }
    
    detect_min_threshold_ = robot_config_["detect_min_threshold"] ? robot_config_["detect_min_threshold"].as<double>() : -60.0;
    detect_max_threshold_ = robot_config_["detect_max_threshold"] ? robot_config_["detect_max_threshold"].as<double>() : 60.0;

    
}

const Eigen::VectorXd& Robot::get_observation() const
{
    return armors_.at(locked_id_).observation;
}

Eigen::VectorXd& Robot::get_observation()
{
    return armors_.at(locked_id_).observation;
}

} // namespace motion_generator