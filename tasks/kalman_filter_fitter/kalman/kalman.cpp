/**
 * @file kalman.cpp
 * @brief @ref kalman_filter::Kalman 的实现。
 */

#include "kalman.hpp"

#include "tools/math_tools/math_tools.hpp"
#include "tools/robot_state/robot_state.hpp"

#include <chrono>
#include <vector>

namespace kalman_filter
{

namespace
{

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

Kalman::Kalman(const YAML::Node& config)
    : model_(config["kalman_filter"])
{
    const auto ekf_config = config["kalman_filter"]["ekf"];
    base_covariance_ = read_vector(ekf_config["P"]);
    measurement_covariance_ = read_vector(ekf_config["R"]).asDiagonal();
    predict_time_ = config["kalman_filter"]["predict_time"].as<double>();
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
        predict(tools::delta_time(filter_time_, timestamp));
    }

    correct(observation, armor_id);
    predict(predict_time_);

    // 记录外推后的滤波时刻，让下一次预测的 dt 与输出时刻对齐。
    filter_time_ = timestamp + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(predict_time_));
}

void Kalman::initialize(
    const Eigen::VectorXd& observation, const int armor_id,
    const TimePoint& timestamp)
{
    ekf_ = EKF(
        model_.initial_state(observation, armor_id),
        model_.initial_covariance(
            observation, armor_id, base_covariance_, measurement_covariance_),
        ArmorModel::add_state, ArmorModel::subtract_state);
    filter_time_ = timestamp;
    initialized_ = true;
}

void Kalman::predict(const double dt)
{
    using namespace tools::robot_state;

    // 匀速模型：位置对速度积分，yaw 对角速度积分，其余状态保持不变。
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(kSize, kSize);
    F(kCenterX, kVelocityX) = dt;
    F(kCenterY, kVelocityY) = dt;
    F(kCenterZ, kVelocityZ) = dt;
    F(kYaw, kYawRate) = dt;

    const EKF::StateFunction transition = [&F](const Eigen::VectorXd& state)
    {
        Eigen::VectorXd predicted = F * state;
        predicted[tools::robot_state::kYaw] =
            tools::normalize_angle(predicted[tools::robot_state::kYaw]);
        return predicted;
    };

    ekf_.predict(F, model_.process_covariance(dt), transition);
}

void Kalman::correct(
    const Eigen::VectorXd& observation, const int armor_id)
{
    const EKF::StateFunction h = [this, armor_id](const Eigen::VectorXd& state)
    {
        return model_.predict_observation(state, armor_id);
    };

    ekf_.update(
        model_.observation_jacobian(ekf_.state(), armor_id), observation,
        measurement_covariance_, h, ArmorModel::subtract_observation);
}

} // namespace kalman_filter
