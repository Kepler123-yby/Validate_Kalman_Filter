#ifndef _KALMAN_HPP_
#define _KALMAN_HPP_

#include "ekf/ekf.hpp"

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <chrono>

namespace kalman_flitter
{

class Kalman
{
public:
    using TimePoint = std::chrono::steady_clock::time_point;

    explicit Kalman(const YAML::Node& config);

    // 输入一帧已关联观测，内部完成预测、更新和固定时长的纯预测。
    void update(
        const Eigen::VectorXd& observation, int armor_id,
        const TimePoint& timestamp);

    const Eigen::VectorXd get_x() const { return ekf_.get_x(); }
    const Eigen::MatrixXd get_P() const { return ekf_.get_P(); }
    double get_nis() const { return ekf_.get_nis(); }
    double get_nees() const { return ekf_.get_nees(); }
    const ConsistencyStatistics& get_nis_statistics() const
    {
        return ekf_.get_nis_statistics();
    }
    const ConsistencyStatistics& get_nees_statistics() const
    {
        return ekf_.get_nees_statistics();
    }
    double evaluate_nees(const Eigen::VectorXd& truth)
    {
        return ekf_.evaluate_nees(truth);
    }

private:
    EKF ekf_;
    Eigen::VectorXd P_;
    Eigen::VectorXd acceleration_noise_;
    Eigen::VectorXd physical_dimension_noise_;
    Eigen::Vector3d physical_dimensions_;
    Eigen::MatrixXd R_;
    double angular_acceleration_noise_;
    double predict_time_;
    TimePoint filter_time_;
    bool initialized_{false};

private:
    void initialize(
        const Eigen::VectorXd& observation, int armor_id,
        const TimePoint& timestamp);
    void predict(double dt);
    void update_ekf(const Eigen::VectorXd& observation, int armor_id);

    Eigen::MatrixXd process_covariance(double dt) const;
    Eigen::VectorXd initial_covariance(
        const Eigen::VectorXd& observation, int armor_id) const;
    static Eigen::VectorXd initial_state(
        const Eigen::VectorXd& observation, int armor_id,
        const Eigen::Vector3d& physical_dimensions);
    static Eigen::VectorXd observation_function(
        const Eigen::VectorXd& state, int armor_id);
    static Eigen::MatrixXd observation_jacobian(
        const Eigen::VectorXd& state, int armor_id);
    static Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d& xyz);
    static Eigen::VectorXd state_add(
        const Eigen::VectorXd& state, const Eigen::VectorXd& delta);
    static Eigen::VectorXd state_subtract(
        const Eigen::VectorXd& a, const Eigen::VectorXd& b);
    static Eigen::VectorXd observation_subtract(
        const Eigen::VectorXd& a, const Eigen::VectorXd& b);
};

} // namespace kalman_flitter

#endif // _KALMAN_HPP_
