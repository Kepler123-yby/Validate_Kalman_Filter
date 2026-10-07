/**
 * @file ekf.cpp
 * @brief @ref kalman_filter::EKF 的实现。
 */

#include "ekf.hpp"

#include <utility>

namespace kalman_filter
{

EKF::EKF(
    const Eigen::VectorXd& x, const Eigen::VectorXd& P_diagonal,
    VectorOperation x_add, VectorOperation x_sub)
    : x_(x),
      P_(P_diagonal.asDiagonal()),
      x_add_(std::move(x_add)),
      x_sub_(std::move(x_sub))
{
}

void EKF::predict(
    const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q,
    const StateFunction& f)
{
    x_ = f(x_);
    P_ = symmetrize(F * P_ * F.transpose() + Q);
}

void EKF::predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q)
{
    const StateFunction f = [&F](const Eigen::VectorXd& x) -> Eigen::VectorXd
    {
        return F * x;
    };
    predict(F, Q, f);
}

void EKF::update(
    const Eigen::MatrixXd& H, const Eigen::VectorXd& z,
    const Eigen::MatrixXd& R, const StateFunction& h,
    const VectorOperation& z_sub)
{
    // 更新前的创新；角度分量由 z_sub 取最短角差。
    innovation_ = z_sub(z, h(x_));
    innovation_covariance_ = symmetrize(H * P_ * H.transpose() + R);

    // NIS 与卡尔曼增益共用 S 的 LDLT 分解，通过解方程代替显式求逆。
    const auto S_factor = innovation_covariance_.ldlt();
    const double nis = innovation_.dot(S_factor.solve(innovation_));
    const Eigen::MatrixXd K = S_factor.solve(H * P_).transpose();
    x_ = x_add_(x_, K * innovation_);

    // Joseph 形式更新协方差，保留数值稳定性。
    const Eigen::MatrixXd I_KH =
        Eigen::MatrixXd::Identity(x_.size(), x_.size()) - K * H;
    P_ = symmetrize(I_KH * P_ * I_KH.transpose() + K * R * K.transpose());

    record_statistics(nis_statistics_, nis, z.size());
}

void EKF::update(
    const Eigen::MatrixXd& H, const Eigen::VectorXd& z,
    const Eigen::MatrixXd& R)
{
    const StateFunction h = [&H](const Eigen::VectorXd& x) -> Eigen::VectorXd
    {
        return H * x;
    };
    update(H, z, R, h);
}

double EKF::evaluate_nees(const Eigen::VectorXd& truth)
{
    // NEES = error^T * P^{-1} * error，使用当前状态及其协方差。
    const Eigen::VectorXd error = x_sub_(truth, x_);
    const double nees = error.dot(P_.ldlt().solve(error));
    record_statistics(nees_statistics_, nees, x_.size());
    return nees;
}

void EKF::reset_evaluation()
{
    nis_statistics_ = ConsistencyStatistics{};
    nees_statistics_ = ConsistencyStatistics{};
    innovation_.resize(0);
    innovation_covariance_.resize(0, 0);
}

Eigen::VectorXd EKF::default_add(
    const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
    return a + b;
}

Eigen::VectorXd EKF::default_subtract(
    const Eigen::VectorXd& a, const Eigen::VectorXd& b)
{
    return a - b;
}

Eigen::MatrixXd EKF::symmetrize(const Eigen::MatrixXd& value)
{
    return 0.5 * value + 0.5 * value.transpose();
}

void EKF::record_statistics(
    ConsistencyStatistics& statistics, const double value,
    const Eigen::Index dimension)
{
    if (statistics.samples == 0)
    {
        statistics.mean = 0.0;
        statistics.normalized_mean = 0.0;
    }

    statistics.value = value;
    statistics.degrees_of_freedom = dimension;
    ++statistics.samples;

    // 在线更新均值，每个样本按自己的维数归一化。
    const double count = static_cast<double>(statistics.samples);
    const double normalized_value = value / static_cast<double>(dimension);
    statistics.mean += (value - statistics.mean) / count;
    statistics.normalized_mean +=
        (normalized_value - statistics.normalized_mean) / count;
}

} // namespace kalman_filter
