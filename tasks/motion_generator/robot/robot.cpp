/**
 * @file robot.cpp
 * @brief @ref motion_generator::Robot 与 @ref motion_generator::Armor 的实现。
 */

#include "robot.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace motion_generator
{

namespace
{

/// 按行主序映射 YAML 中的矩阵数据，避免列主序导致的排列错误。
using RowMajorMatrix =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

} // namespace

Armor::Armor(const int armor_id)
    : id(armor_id), observation(Eigen::Vector4d::Zero())
{
}

Robot::Robot(const YAML::Node& config)
{
    const auto robot_config = config["robot"];

    const int armor_count = robot_config["armor_nums"].as<int>(4);
    armors_.reserve(armor_count);
    for (int i = 0; i < armor_count; ++i)
    {
        armors_.emplace_back(i);
    }

    detect_min_threshold_ =
        robot_config["detect_min_threshold"].as<double>(-M_PI / 3.0);
    detect_max_threshold_ =
        robot_config["detect_max_threshold"].as<double>(M_PI / 3.0);

    // 原始状态到滤波状态的映射，维数与数据长度由配置保证。
    const auto state_config = robot_config["raw2state_mat"];
    const auto state_mat_size = state_config["size"].as<std::array<int, 2>>(
        std::array<int, 2>{tools::robot_state::kSize, RawStateIndex::kRawSize});
    const auto matrix_data = state_config["data"].as<std::vector<double>>();
    raw_to_state_ = Eigen::Map<const RowMajorMatrix>(
        matrix_data.data(), state_mat_size[0], state_mat_size[1]);

    const auto initial_states = robot_config["init_states"].as<std::vector<double>>(
        std::vector<double>(raw_to_state_.cols(), 0.0));
    update_state(Eigen::Map<const Eigen::VectorXd>(
        initial_states.data(), initial_states.size()));
}

void Robot::update_state(const Eigen::VectorXd& raw_state)
{
    Eigen::VectorXd normalized_state = raw_state;
    normalized_state[RawStateIndex::kRawYaw] =
        tools::normalize_angle(normalized_state[RawStateIndex::kRawYaw]);

    compute_states(normalized_state);
    compute_observations(normalized_state);
    update_locked_id();
}

void Robot::compute_states(const Eigen::VectorXd& raw_state)
{
    states_ = raw_to_state_ * raw_state;
}

void Robot::compute_observations(const Eigen::VectorXd& raw_state)
{
    const double yaw = raw_state[RawStateIndex::kRawYaw];
    const double center_x = raw_state[RawStateIndex::kRawCenterX];
    const double center_y = raw_state[RawStateIndex::kRawCenterY];
    const double center_z = raw_state[RawStateIndex::kRawCenterZ];
    const double forward_radius = raw_state[RawStateIndex::kRawForwardRadius];
    const double beside_radius = raw_state[RawStateIndex::kRawBesideRadius];
    const double height_difference = raw_state[RawStateIndex::kRawHeightDifference];

    const double armor_angle_step = 2.0 * M_PI / armors_.size();

    for (auto& armor : armors_)
    {
        const double angle =
            tools::normalize_angle(yaw + armor.id * armor_angle_step);
        const double cos_angle = std::cos(angle);
        const double sin_angle = std::sin(angle);

        // 正面装甲板使用 forward_radius，且与中心等高；
        // 侧面装甲板使用 beside_radius，并带有一个高度差。
        const double radius =
            tools::robot_state::is_front_armor(armor.id) ? forward_radius
                                                         : beside_radius;
        const double z = tools::robot_state::is_front_armor(armor.id)
            ? center_z
            : center_z + height_difference;

        const double x = center_x - cos_angle * radius;
        const double y = center_y - sin_angle * radius;

        const Eigen::Vector3d ypd = tools::cartesian_to_ypd({x, y, z});
        armor.observation =
            Eigen::Vector4d(ypd[0], ypd[1], ypd[2], angle);
    }
}

void Robot::update_locked_id()
{
    int new_locked_id = locked_id_;
    double min_detect_angle = std::numeric_limits<double>::max();

    for (const auto& armor : armors_)
    {
        const double observation_yaw =
            armor.observation[tools::robot_state::kBearing];
        const double armor_yaw =
            armor.observation[tools::robot_state::kArmorAngle];

        const double detect_angle = tools::angle_difference(armor_yaw, observation_yaw);

        // 超出可见范围（±detect threshold）的装甲板不参与锁定。
        if (detect_angle < detect_min_threshold_
            || detect_angle > detect_max_threshold_)
        {
            continue;
        }

        if (std::abs(detect_angle) < min_detect_angle)
        {
            min_detect_angle = std::abs(detect_angle);
            new_locked_id = armor.id;
        }
    }

    if (new_locked_id != locked_id_)
    {
        locked_id_ = new_locked_id;
        ++switch_times_;
    }
}

} // namespace motion_generator
