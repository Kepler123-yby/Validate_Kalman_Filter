/**
 * @file armor_model.cpp
 * @brief @ref kalman_filter::ArmorModel 的实现。
 */

#include "armor_model.hpp"

#include "tools/math_tools/math_tools.hpp"
#include "tools/robot_state/robot_state.hpp"

#include <cmath>
#include <vector>

namespace kalman_filter
{

namespace
{

constexpr double pi = 3.14159265358979323846;

/**
 * @brief 将 YAML 序列读取为动态向量。
 * @param node 形如 @c [a,b,c] 的序列节点。
 * @return 对应的向量。
 */
Eigen::VectorXd read_vector(const YAML::Node& node)
{
    const auto values = node.as<std::vector<double>>();
    return Eigen::Map<const Eigen::VectorXd>(values.data(), values.size());
}

} // namespace

ArmorModel::ArmorModel(const YAML::Node& config)
{
    const auto ekf_config = config["ekf"];
    acceleration_noise_ = read_vector(ekf_config["Q"]["a"]);
    angular_acceleration_noise_ = ekf_config["Q"]["wa"].as<double>();
    physical_dimension_noise_ = read_vector(ekf_config["Q"]["physical_dimensions"]);
    physical_dimensions_ = read_vector(config["physical_dimensions"]);
}

Eigen::VectorXd ArmorModel::initial_state(
    const Eigen::VectorXd& observation, const int armor_id) const
{
    using namespace tools::robot_state;

    const double armor_angle = observation[kArmorAngle];
    const bool front = is_front_armor(armor_id);
    const double radius = physical_dimensions_[front ? 0 : 1];
    const double height_difference = front ? 0.0 : physical_dimensions_[2];
    const double yaw = tools::normalize_angle(armor_angle - armor_id * pi / 2.0);

    // 由球坐标观测恢复装甲板的三维位置。
    const double cos_pitch = std::cos(observation[kPitch]);
    const Eigen::Vector3d armor_xyz{
        observation[kDistance] * cos_pitch * std::cos(observation[kBearing]),
        observation[kDistance] * cos_pitch * std::sin(observation[kBearing]),
        observation[kDistance] * std::sin(observation[kPitch])};

    // 装甲板位置 = 中心位置 - 半径方向偏移；据此反解中心位置。
    Eigen::VectorXd state = Eigen::VectorXd::Zero(kSize);
    state[kCenterX] = armor_xyz[0] + radius * std::cos(armor_angle);
    state[kCenterY] = armor_xyz[1] + radius * std::sin(armor_angle);
    state[kCenterZ] = armor_xyz[2] - height_difference;
    state[kYaw] = yaw;
    state[kForwardRadius] = physical_dimensions_[0];
    state[kBesideRadius] = physical_dimensions_[1];
    state[kHeightDifference] = physical_dimensions_[2];
    return state;
}

Eigen::VectorXd ArmorModel::initial_covariance(
    const Eigen::VectorXd& observation, const int armor_id,
    const Eigen::VectorXd& base_covariance,
    const Eigen::MatrixXd& measurement_covariance) const
{
    using namespace tools::robot_state;

    const double bearing = observation[kBearing];
    const double pitch = observation[kPitch];
    const double distance = observation[kDistance];
    const double armor_angle = observation[kArmorAngle];
    const double radius =
        physical_dimensions_[is_front_armor(armor_id) ? 0 : 1];

    // 首帧状态由观测反解得到，这里计算观测对 (x, y, z, yaw) 的逆雅可比，
    // 再把观测噪声传播到位置与偏航角的方差。P 对外保持对角形式，因此只保留
    // 各状态的边缘方差。
    Eigen::MatrixXd inverse_jacobian = Eigen::MatrixXd::Zero(4, 4);
    inverse_jacobian(0, 0) = -distance * std::cos(pitch) * std::sin(bearing);
    inverse_jacobian(0, 1) = -distance * std::sin(pitch) * std::cos(bearing);
    inverse_jacobian(0, 2) = std::cos(pitch) * std::cos(bearing);
    inverse_jacobian(0, 3) = -radius * std::sin(armor_angle);
    inverse_jacobian(1, 0) = distance * std::cos(pitch) * std::cos(bearing);
    inverse_jacobian(1, 1) = -distance * std::sin(pitch) * std::sin(bearing);
    inverse_jacobian(1, 2) = std::cos(pitch) * std::sin(bearing);
    inverse_jacobian(1, 3) = radius * std::cos(armor_angle);
    inverse_jacobian(2, 1) = distance * std::cos(pitch);
    inverse_jacobian(2, 2) = std::sin(pitch);
    inverse_jacobian(3, 3) = 1.0;

    const Eigen::VectorXd measurement_variance = measurement_covariance.diagonal();
    Eigen::VectorXd covariance = base_covariance;
    covariance[kCenterX] += (inverse_jacobian.row(0).array().square()
        * measurement_variance.transpose().array()).sum();
    covariance[kCenterY] += (inverse_jacobian.row(1).array().square()
        * measurement_variance.transpose().array()).sum();
    covariance[kCenterZ] += (inverse_jacobian.row(2).array().square()
        * measurement_variance.transpose().array()).sum();
    covariance[kYaw] += measurement_variance[kArmorAngle];
    return covariance;
}

Eigen::MatrixXd ArmorModel::process_covariance(const double dt) const
{
    using namespace tools::robot_state;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(kSize, kSize);
    const auto add_axis = [&Q, dt](const int index, const double acceleration)
    {
        const Eigen::Vector2d noise{
            0.5 * acceleration * dt * dt, acceleration * dt};
        Q.block<2, 2>(index, index) = noise * noise.transpose();
    };

    add_axis(kCenterX, acceleration_noise_[0]);
    add_axis(kCenterY, acceleration_noise_[1]);
    add_axis(kCenterZ, acceleration_noise_[2]);
    add_axis(kYaw, angular_acceleration_noise_);

    Q.block<3, 3>(kForwardRadius, kForwardRadius) =
        physical_dimension_noise_.cwiseProduct(physical_dimension_noise_)
            .asDiagonal();
    return Q;
}

Eigen::Vector3d ArmorModel::armor_position(
    const Eigen::VectorXd& state, const int armor_id)
{
    using namespace tools::robot_state;

    const bool front = is_front_armor(armor_id);
    const double angle = state[kYaw] + armor_id * pi / 2.0;
    const double radius = state[front ? kForwardRadius : kBesideRadius];

    return Eigen::Vector3d(
        state[kCenterX] - radius * std::cos(angle),
        state[kCenterY] - radius * std::sin(angle),
        state[kCenterZ] + (front ? 0.0 : state[kHeightDifference]));
}

Eigen::VectorXd ArmorModel::predict_observation(
    const Eigen::VectorXd& state, const int armor_id) const
{
    using namespace tools::robot_state;

    const double angle = state[kYaw] + armor_id * pi / 2.0;
    const Eigen::Vector3d ypd =
        tools::cartesian_to_ypd(armor_position(state, armor_id));

    Eigen::VectorXd observation(kObservationSize);
    observation << ypd[0], ypd[1], ypd[2], tools::normalize_angle(angle);
    return observation;
}

Eigen::MatrixXd ArmorModel::observation_jacobian(
    const Eigen::VectorXd& state, const int armor_id) const
{
    using namespace tools::robot_state;

    const bool front = is_front_armor(armor_id);
    const int radius_index = front ? kForwardRadius : kBesideRadius;
    const double angle = state[kYaw] + armor_id * pi / 2.0;
    const double radius = state[radius_index];
    const Eigen::Vector3d xyz = armor_position(state, armor_id);

    // 装甲板位置对状态的偏导：位置状态直接对应，yaw 与半径带来耦合项。
    Eigen::MatrixXd state_to_xyz = Eigen::MatrixXd::Zero(3, kSize);
    state_to_xyz(0, kCenterX) = 1.0;
    state_to_xyz(0, kYaw) = radius * std::sin(angle);
    state_to_xyz(0, radius_index) = -std::cos(angle);
    state_to_xyz(1, kCenterY) = 1.0;
    state_to_xyz(1, kYaw) = -radius * std::cos(angle);
    state_to_xyz(1, radius_index) = -std::sin(angle);
    state_to_xyz(2, kCenterZ) = 1.0;
    state_to_xyz(2, kHeightDifference) = front ? 0.0 : 1.0;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(kObservationSize, kSize);
    H.topRows<3>() = ypd_jacobian(xyz) * state_to_xyz;
    H(3, kYaw) = 1.0;
    return H;
}

Eigen::MatrixXd ArmorModel::ypd_jacobian(const Eigen::Vector3d& xyz)
{
    const double x = xyz[0];
    const double y = xyz[1];
    const double z = xyz[2];
    const double xy = x * x + y * y;
    const double distance = std::sqrt(xy + z * z);
    const double xy_distance = std::sqrt(xy);

    Eigen::MatrixXd jacobian(3, 3);
    jacobian <<
        -y / xy, x / xy, 0.0,
        -x * z / (distance * distance * xy_distance),
        -y * z / (distance * distance * xy_distance),
        xy_distance / (distance * distance),
        x / distance, y / distance, z / distance;
    return jacobian;
}

Eigen::VectorXd ArmorModel::add_state(
    const Eigen::VectorXd& state, const Eigen::VectorXd& delta)
{
    Eigen::VectorXd result = state + delta;
    result[tools::robot_state::kYaw] =
        tools::normalize_angle(result[tools::robot_state::kYaw]);
    return result;
}

Eigen::VectorXd ArmorModel::subtract_state(
    const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
    Eigen::VectorXd result = a - b;
    result[tools::robot_state::kYaw] =
        tools::normalize_angle(result[tools::robot_state::kYaw]);
    return result;
}

Eigen::VectorXd ArmorModel::subtract_observation(
    const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
    using namespace tools::robot_state;

    Eigen::VectorXd result = a - b;
    result[kBearing] = tools::normalize_angle(result[kBearing]);
    result[kArmorAngle] = tools::normalize_angle(result[kArmorAngle]);
    return result;
}

} // namespace kalman_filter
