#include "robot.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace motion_generator
{

using MatRowMajor = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

Armor::Armor(const int id)
    : id_(id), observation(Eigen::Vector4d::Zero())
{
}

Robot::Robot(const YAML::Node& config)
{
    auto robot_config = config["robot"];

    armor_nums_ = robot_config["armor_nums"]
        ? robot_config["armor_nums"].as<int>()
        : 4;
    if (armor_nums_ <= 0)
    {
        throw std::invalid_argument("robot.armor_nums must be positive");
    }

    armors_.reserve(armor_nums_);
    for (int i = 0; i < armor_nums_; ++i)
    {
        armors_.emplace_back(i);
    }

    detect_min_threshold_ = robot_config["detect_min_threshold"]
        ? robot_config["detect_min_threshold"].as<double>()
        : -60.0;
    detect_max_threshold_ = robot_config["detect_max_threshold"]
        ? robot_config["detect_max_threshold"].as<double>()
        : 60.0;
    if (detect_min_threshold_ > detect_max_threshold_)
    {
        throw std::invalid_argument(
            "robot.detect_min_threshold must not exceed detect_max_threshold");
    }

    // state转置矩阵
    auto state_config = robot_config["raw2state_mat"];
    std::array<int, 2> state_mat_size = state_config["size"]
        ? state_config["size"].as<std::array<int, 2>>()
        : std::array<int, 2>{11, 15};
    auto raw2states_mat_data = state_config["data"].as<std::vector<double>>();

    if (state_mat_size[0] <= 0 || state_mat_size[1] <= 0
        || raw2states_mat_data.size()
            != static_cast<std::size_t>(state_mat_size[0] * state_mat_size[1]))
    {
        throw std::invalid_argument("robot.raw2state_mat size does not match data");
    }

    raw2states_mat_ = Eigen::Map<MatRowMajor>(
        raw2states_mat_data.data(), state_mat_size[0], state_mat_size[1]);
    if (raw2states_mat_.cols() != 15)
    {
        throw std::invalid_argument("robot raw state must contain 15 values");
    }

    auto init_states_data = robot_config["init_states"]
        ? robot_config["init_states"].as<std::vector<double>>()
        : std::vector<double>(raw2states_mat_.cols(), 0.0);
    if (init_states_data.size() != static_cast<std::size_t>(raw2states_mat_.cols()))
    {
        throw std::invalid_argument("robot.init_states size is invalid");
    }

    update_state(Eigen::Map<const Eigen::VectorXd>(
        init_states_data.data(), init_states_data.size()));
}

const Eigen::Vector4d& Robot::get_observation() const
{
    return armors_.at(locked_id_).observation;
}

Eigen::Vector4d& Robot::get_observation()
{
    return armors_.at(locked_id_).observation;
}

void Robot::update_state(const Eigen::VectorXd& raw_state)
{
    if (raw_state.size() != raw2states_mat_.cols())
    {
        throw std::invalid_argument("raw_state size is invalid");
    }

    raw2states(raw_state);
    raw2observation(raw_state);
    update_locked_id();
}

void Robot::raw2states(const Eigen::VectorXd& raw_state)
{
    states_ = raw2states_mat_ * raw_state;
}

void Robot::raw2observation(const Eigen::VectorXd& raw_state)
{
    double yaw = raw_state[9];
    double center_x = raw_state[0];
    double center_y = raw_state[3];
    double center_z = raw_state[6];
    double forward_radius = raw_state[12];
    double beside_radius = raw_state[13];
    double beside_height_diff = raw_state[14];

    for (auto& armor : armors_)
    {
        double angle = yaw + armor.id_ * 2.0 * M_PI / armor_nums_;
        double cos_angle = std::cos(angle);
        double sin_angle = std::sin(angle);

        if (armor.id_ % 2 == 0)
        {
            double x = center_x - cos_angle * forward_radius;
            double y = center_y - sin_angle * forward_radius;
            Eigen::Vector3d ypd = tools::xyz_to_ypd(
                Eigen::Vector3d(x, y, center_z));
            armor.observation = Eigen::Vector4d(
                ypd[0], ypd[1], ypd[2], angle);
        }
        else
        {
            double x = center_x - cos_angle * beside_radius;
            double y = center_y - sin_angle * beside_radius;
            double z = center_z + beside_height_diff;
            Eigen::Vector3d ypd = tools::xyz_to_ypd(
                Eigen::Vector3d(x, y, z));
            armor.observation = Eigen::Vector4d(
                ypd[0], ypd[1], ypd[2], angle);
        }
    }
}

void Robot::update_locked_id()
{
    int new_locked_id = locked_id_;
    double min_detect_angle = std::numeric_limits<double>::max();
    bool armor_found = false;

    for (const auto& armor : armors_)
    {
        double observation_yaw = armor.observation[0];
        double armor_yaw = armor.observation[3];

        double detect_angle = tools::angle_to_euler(
            std::remainder(observation_yaw - armor_yaw, 2.0 * M_PI));

        if (detect_angle < detect_min_threshold_
            || detect_angle > detect_max_threshold_)
        {
            continue;
        }

        if (std::abs(detect_angle) < min_detect_angle)
        {
            min_detect_angle = std::abs(detect_angle);
            new_locked_id = armor.id_;
            armor_found = true;
        }
    }

    if (armor_found && new_locked_id != locked_id_)
    {
        locked_id_ = new_locked_id;
        ++switch_times_;
    }
}

} // namespace motion_generator
