/**
 * @file kalman.hpp
 * @brief 自瞄装甲板 EKF 拟合器：负责时间调度与滤波调用。
 *
 * @ref kalman_filter::Kalman 把 @ref kalman_filter::ArmorModel（物理模型）与
 * @ref kalman_filter::EKF（通用滤波器）组合起来，完成一次观测的完整处理流程：
 *
 * 1. 首帧由观测反解并初始化滤波器；
 * 2. 用两帧观测的时间差做一次时域预测；
 * 3. 用当前观测做后验更新；
 * 4. 额外做固定时长 @c predict_time 的纯预测，输出给决策层使用。
 *
 * 这样设计使“模型”“滤波算法”“时序调度”三者职责清晰、可独立替换。
 */

#ifndef VALIDATE_KALMAN_FILTER_FITTER_KALMAN_HPP_
#define VALIDATE_KALMAN_FILTER_FITTER_KALMAN_HPP_

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <chrono>

#include "ekf/ekf.hpp"
#include "model/armor_model.hpp"

namespace kalman_filter
{

/**
 * @brief 装甲板机器人 EKF 拟合器。
 */
class Kalman
{
public:
    /// 用于时间戳的时钟类型。
    using TimePoint = std::chrono::steady_clock::time_point;

    /**
     * @brief 从 YAML 配置构造拟合器。
     *
     * @param config 顶层配置节点，滤波器参数位于 @c kalman_filter 子节点，
     *               需包含 @c ekf.P、@c ekf.R、@c predict_time、
     *               @c physical_dimensions 与 @c ekf.Q。
     */
    explicit Kalman(const YAML::Node& config);

    /**
     * @brief 使用一帧观测推进整个滤波流程。
     *
     * @param observation 4 维关联观测 @f$ [\text{bearing}, \text{pitch}, \text{distance}, \text{armor\_angle}] @f$。
     * @param armor_id    观测对应的装甲板编号，需与仿真器一致。
     * @param timestamp   观测时间戳。
     */
    void update(
        const Eigen::VectorXd& observation, int armor_id,
        const TimePoint& timestamp);

    /// @return 当前状态估计。
    Eigen::VectorXd state() const { return ekf_.state(); }

    /// @return 当前估计协方差。
    Eigen::MatrixXd covariance() const { return ekf_.covariance(); }

    /// @return 最近一次更新的 NIS。
    double nis() const { return ekf_.nis(); }

    /// @return 最近一次评估的 NEES。
    double nees() const { return ekf_.nees(); }

    /// @return NIS 统计量。
    const ConsistencyStatistics& nis_statistics() const
    {
        return ekf_.nis_statistics();
    }

    /// @return NEES 统计量。
    const ConsistencyStatistics& nees_statistics() const
    {
        return ekf_.nees_statistics();
    }

    /**
     * @brief 使用真值计算并记录一次 NEES。
     *
     * @param truth 同一时刻、同一排列的真值状态。
     * @return 本次 NEES 值。
     */
    double evaluate_nees(const Eigen::VectorXd& truth)
    {
        return ekf_.evaluate_nees(truth);
    }

    /// @return 底层的物理模型，便于测试访问几何参数。
    const ArmorModel& model() const { return model_; }

private:
    /**
     * @brief 用首帧观测初始化滤波器。
     */
    void initialize(
        const Eigen::VectorXd& observation, int armor_id,
        const TimePoint& timestamp);

    /**
     * @brief 执行一步时域预测。
     * @param dt 距上一次滤波的时间差，单位 s。
     */
    void predict(double dt);

    /**
     * @brief 使用观测执行一步后验更新。
     */
    void correct(const Eigen::VectorXd& observation, int armor_id);

    ArmorModel model_;                       ///< 物理模型。
    EKF ekf_;                                ///< 通用滤波器。

    Eigen::VectorXd base_covariance_;        ///< 配置中的基础协方差对角线。
    Eigen::MatrixXd measurement_covariance_; ///< 观测噪声协方差 @f$ R @f$。
    double predict_time_{0.15};              ///< 输出前的纯预测时长，单位 s。

    TimePoint filter_time_;                  ///< 上一次滤波时刻（含预测外推）。
    bool initialized_{false};                ///< 是否已完成初始化。
};

} // namespace kalman_filter

#endif // VALIDATE_KALMAN_FILTER_FITTER_KALMAN_HPP_
