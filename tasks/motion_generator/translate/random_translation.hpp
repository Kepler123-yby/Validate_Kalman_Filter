/**
 * @file random_translation.hpp
 * @brief 基于“随机目标速度 + 加速度/jerk 约束”的随机平移策略。
 *
 * 该策略不直接对位置加噪声，而是每隔一段时间随机生成一个目标速度，再通过
 * 一阶响应、加速度限幅与 jerk 限幅平滑逼近目标速度，最后积分得到速度与
 * 位置。这样可自然产生匀速、加减速、急停、变向等连续机动，用于评估 EKF
 * 在复杂场景下的鲁棒性，同时保证结果可由随机种子复现。
 */

#ifndef VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_RANDOM_TRANSLATION_HPP_
#define VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_RANDOM_TRANSLATION_HPP_

#include "translation_motion.hpp"

#include <yaml-cpp/yaml.h>

#include <random>

namespace motion_generator
{

/**
 * @brief 随机机动平移策略。
 *
 * 由 @c translation.random 配置，主要参数包括：
 * - @c velocity_min / @c velocity_max：各轴速度上下限；
 * - @c max_acceleration / @c max_jerk：加速度与加加速度上限；
 * - @c response_time：速度跟踪目标速度的时间常数；
 * - @c hold_time_min / @c hold_time_max：目标速度保持时间范围；
 * - @c stop_probability / @c reverse_probability：急停与反向事件概率；
 * - @c integration_step：内部积分步长；
 * - @c seed：随机种子，相同种子可复现轨迹。
 *
 * 初值取自 @c translation.initial 中的 @c position 与 @c velocity。
 */
class RandomTranslation : public TranslationMotion
{
public:
    /**
     * @brief 从 YAML 配置构造随机平移策略。
     *
     * @param config @c translation 配置节点，需包含 @c random 与 @c initial 子节点。
     */
    explicit RandomTranslation(const YAML::Node& config);

    TranslationState advance(double dt) override;
    void reset() override;
    TranslationState state() const override;

private:
    /**
     * @brief 随机抽取新的目标速度与保持时长。
     *
     * 以 @c stop_probability、@c reverse_probability 决定急停/反向事件，
     * 否则在各轴速度范围内均匀采样。
     */
    void set_random_target();

    /**
     * @brief 将向量模长限制在 @p limit 以内，保持方向不变。
     *
     * @param value 待限幅向量。
     * @param limit 模长上限，非负。
     * @return 限幅后的向量。
     */
    static Eigen::Vector3d limit_norm(const Eigen::Vector3d& value, double limit);

    Eigen::Vector3d initial_position_{Eigen::Vector3d::Zero()}; ///< 初始位置。
    Eigen::Vector3d initial_velocity_{Eigen::Vector3d::Zero()}; ///< 初始速度。
    Eigen::Vector3d velocity_min_{Eigen::Vector3d::Constant(-1.0)}; ///< 速度下限。
    Eigen::Vector3d velocity_max_{Eigen::Vector3d::Constant(1.0)};  ///< 速度上限。
    Eigen::Vector3d target_velocity_{Eigen::Vector3d::Zero()};  ///< 当前目标速度。

    double max_acceleration_{2.0};   ///< 最大加速度，单位 m/s²。
    double max_jerk_{10.0};          ///< 最大 jerk，单位 m/s³。
    double response_time_{0.5};      ///< 速度响应时间常数，单位 s。
    double hold_time_min_{0.5};      ///< 目标速度最短保持时间，单位 s。
    double hold_time_max_{2.0};      ///< 目标速度最长保持时间，单位 s。
    double stop_probability_{0.0};   ///< 急停事件概率，范围 [0, 1]。
    double reverse_probability_{0.0};///< 反向事件概率，范围 [0, 1]。
    double integration_step_{0.01};  ///< 内部积分步长，单位 s。
    unsigned int random_seed_{42};   ///< 随机种子。

    std::mt19937 random_engine_;     ///< 随机数引擎。
    double hold_time_{0.0};          ///< 当前目标速度剩余保持时间，单位 s。
    TranslationState state_;         ///< 当前真值状态。
};

} // namespace motion_generator

#endif // VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_RANDOM_TRANSLATION_HPP_
