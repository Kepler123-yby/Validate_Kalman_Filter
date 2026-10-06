#include "kalman.hpp"

#include "tools/math_tools/math_tools.hpp"

#include <cmath>
#include <vector>

namespace kalman_flitter
{

namespace
{

Eigen::VectorXd read_vector(const YAML::Node& config)
{
    const auto values = config.as<std::vector<double>>();
    return Eigen::Map<const Eigen::VectorXd>(values.data(), values.size());
}

constexpr double pi = 3.14159265358979323846;

} // namespace

Kalman::Kalman(const YAML::Node& config)
{
    const auto kalman_config = config["kalman_flitter"];
    const auto ekf_config = kalman_config["ekf"];

    P_ = read_vector(ekf_config["P"]);
    acceleration_noise_ = read_vector(ekf_config["Q"]["a"]);
    angular_acceleration_noise_ = ekf_config["Q"]["wa"].as<double>();
    physical_dimension_noise_ = read_vector(ekf_config["Q"]["physical_dimensions"]);
    physical_dimensions_ = read_vector(kalman_config["physical_dimensions"]);
    R_ = read_vector(ekf_config["R"]).asDiagonal();
    predict_time_ = kalman_config["predict_time"].as<double>();
}

void Kalman::update(
    const Eigen::VectorXd& observation, const int armor_id,
    const TimePoint& timestamp)
{
    if (!initialized_)
    {
        initialize(observation, armor_id, timestamp);
    }
    else
    {
        predict(std::chrono::duration<double>(timestamp - filter_time_).count());
    }

    update_ekf(observation, armor_id);
    predict(predict_time_);
    filter_time_ = timestamp + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(predict_time_));
}

void Kalman::initialize(
    const Eigen::VectorXd& observation, const int armor_id,
    const TimePoint& timestamp)
{
    ekf_ = EKF(
        initial_state(observation, armor_id, physical_dimensions_),
        initial_covariance(observation, armor_id),
        state_add, state_subtract);
    filter_time_ = timestamp;
    initialized_ = true;
}

void Kalman::predict(const double dt)
{
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(11, 11);
    F(0, 1) = dt;
    F(2, 3) = dt;
    F(4, 5) = dt;
    F(6, 7) = dt;

    auto transition = [&F](const Eigen::VectorXd& state)
    {
        Eigen::VectorXd predicted = F * state;
        predicted[6] = tools::limit_euler(predicted[6]);
        return predicted;
    };
    ekf_.predict(F, process_covariance(dt), transition);
}

void Kalman::update_ekf(
    const Eigen::VectorXd& observation, const int armor_id)
{
    auto h = [armor_id](const Eigen::VectorXd& state)
    {
        return observation_function(state, armor_id);
    };
    ekf_.update(
        observation_jacobian(ekf_.get_x(), armor_id), observation, R_, h,
        observation_subtract);
}

Eigen::MatrixXd Kalman::process_covariance(const double dt) const
{
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(11, 11);
    auto add_covariance = [&Q, dt](const int index, const double acceleration)
    {
        Eigen::Vector2d noise{
            0.5 * acceleration * dt * dt, acceleration * dt};
        Q.block<2, 2>(index, index) = noise * noise.transpose();
    };
    add_covariance(0, acceleration_noise_[0]);
    add_covariance(2, acceleration_noise_[1]);
    add_covariance(4, acceleration_noise_[2]);
    add_covariance(6, angular_acceleration_noise_);
    Q.block<3, 3>(8, 8) =
        physical_dimension_noise_.cwiseProduct(physical_dimension_noise_).asDiagonal();
    return Q;
}

Eigen::VectorXd Kalman::initial_covariance(
    const Eigen::VectorXd& observation, const int armor_id) const
{
    // 首帧状态由观测反解得到，将观测噪声传播到位置和偏航方差。
    // P 对外保持对角向量，因此这里只保留各状态的边缘方差。
    const double bearing = observation[0];
    const double pitch = observation[1];
    const double distance = observation[2];
    const double armor_angle = observation[3];
    const bool is_forward_armor = armor_id % 2 == 0;
    const double radius = physical_dimensions_[is_forward_armor ? 0 : 1];

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

    const Eigen::VectorXd measurement_variance = R_.diagonal();
    Eigen::VectorXd covariance = P_;
    covariance[0] += (inverse_jacobian.row(0).array().square()
        * measurement_variance.transpose().array()).sum();
    covariance[2] += (inverse_jacobian.row(1).array().square()
        * measurement_variance.transpose().array()).sum();
    covariance[4] += (inverse_jacobian.row(2).array().square()
        * measurement_variance.transpose().array()).sum();
    covariance[6] += measurement_variance[3];
    return covariance;
}

Eigen::VectorXd Kalman::initial_state(
    const Eigen::VectorXd& observation, const int armor_id,
    const Eigen::Vector3d& physical_dimensions)
{
    const double armor_angle = observation[3];
    const bool is_forward_armor = armor_id % 2 == 0;
    const double radius = physical_dimensions[is_forward_armor ? 0 : 1];
    const double height_diff = is_forward_armor ? 0.0 : physical_dimensions[2];
    const double yaw = tools::limit_euler(armor_angle - armor_id * pi / 2.0);
    const double cos_pitch = std::cos(observation[1]);
    const Eigen::Vector3d armor_xyz{
        observation[2] * cos_pitch * std::cos(observation[0]),
        observation[2] * cos_pitch * std::sin(observation[0]),
        observation[2] * std::sin(observation[1])};

    Eigen::VectorXd state(11);
    state << armor_xyz[0] + radius * std::cos(armor_angle), 0,
        armor_xyz[1] + radius * std::sin(armor_angle), 0,
        armor_xyz[2] - height_diff, 0, yaw, 0,
        physical_dimensions[0], physical_dimensions[1], physical_dimensions[2];
    return state;
}

Eigen::VectorXd Kalman::observation_function(
    const Eigen::VectorXd& state, const int armor_id)
{
    const bool is_forward_armor = armor_id % 2 == 0;
    const double angle = state[6] + armor_id * pi / 2.0;
    const double radius = state[is_forward_armor ? 8 : 9];
    const Eigen::Vector3d xyz{
        state[0] - radius * std::cos(angle),
        state[2] - radius * std::sin(angle),
        state[4] + (is_forward_armor ? 0.0 : state[10])};
    const Eigen::Vector3d ypd = tools::xyz_to_ypd(xyz);

    Eigen::VectorXd observation(4);
    observation << ypd[0], ypd[1], ypd[2], tools::limit_euler(angle);
    return observation;
}

Eigen::MatrixXd Kalman::observation_jacobian(
    const Eigen::VectorXd& state, const int armor_id)
{
    const bool is_forward_armor = armor_id % 2 == 0;
    const int radius_index = is_forward_armor ? 8 : 9;
    const double angle = state[6] + armor_id * pi / 2.0;
    const double radius = state[radius_index];
    const Eigen::Vector3d xyz{
        state[0] - radius * std::cos(angle),
        state[2] - radius * std::sin(angle),
        state[4] + (is_forward_armor ? 0.0 : state[10])};

    Eigen::MatrixXd state_to_xyz = Eigen::MatrixXd::Zero(3, 11);
    state_to_xyz(0, 0) = 1;
    state_to_xyz(0, 6) = radius * std::sin(angle);
    state_to_xyz(0, radius_index) = -std::cos(angle);
    state_to_xyz(1, 2) = 1;
    state_to_xyz(1, 6) = -radius * std::cos(angle);
    state_to_xyz(1, radius_index) = -std::sin(angle);
    state_to_xyz(2, 4) = 1;
    state_to_xyz(2, 10) = is_forward_armor ? 0 : 1;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 11);
    H.topRows<3>() = xyz2ypd_jacobian(xyz) * state_to_xyz;
    H(3, 6) = 1;
    return H;
}

Eigen::MatrixXd Kalman::xyz2ypd_jacobian(const Eigen::Vector3d& xyz)
{
    const double x = xyz[0], y = xyz[1], z = xyz[2];
    const double xy = x * x + y * y;
    const double distance = std::sqrt(xy + z * z);
    const double xy_distance = std::sqrt(xy);

    Eigen::MatrixXd jacobian(3, 3);
    jacobian <<
        -y / xy, x / xy, 0,
        -x * z / (distance * distance * xy_distance),
        -y * z / (distance * distance * xy_distance), xy_distance / (distance * distance),
        x / distance, y / distance, z / distance;
    return jacobian;
}

Eigen::VectorXd Kalman::state_add(
    const Eigen::VectorXd& state, const Eigen::VectorXd& delta)
{
    Eigen::VectorXd result = state + delta;
    result[6] = tools::limit_euler(result[6]);
    return result;
}

Eigen::VectorXd Kalman::state_subtract(
    const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
    Eigen::VectorXd result = a - b;
    result[6] = tools::limit_euler(result[6]);
    return result;
}

Eigen::VectorXd Kalman::observation_subtract(
    const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
    Eigen::VectorXd result = a - b;
    result[0] = tools::limit_euler(result[0]);
    result[3] = tools::limit_euler(result[3]);
    return result;
}

} // namespace kalman_flitter
