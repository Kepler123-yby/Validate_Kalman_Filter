/**
 * @file translate.hpp
 * @brief 平移运动生成器：对外统一接口，对内委托给具体运动策略。
 *
 * @ref motion_generator::TranslationGenerator 负责两件事：
 * 1. 根据 YAML 中的 @c translation_mode 选择并创建具体的
 *    @ref motion_generator::TranslationMotion 策略；
 * 2. 处理真实时钟到时间增量的转换（首次调用只记录时间戳）。
 *
 * 具体的运动学计算完全封装在策略内部，上层无需关心当前是直线运动还是
 * 随机机动。
 */

#ifndef VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_TRANSLATE_HPP_
#define VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_TRANSLATE_HPP_

#include "translation_motion.hpp"
#include "translation_state.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <memory>

namespace motion_generator
{

/**
 * @brief 平移运动模式枚举。
 */
enum class TranslationMode
{
    LINE,   ///< 两点之间确定性往返。
    RANDOM  ///< 带加速度/jerk 约束的随机机动。
};

/**
 * @brief 平移运动生成器（策略模式的上下文）。
 *
 * 通过 YAML 配置 @c translation.translation_mode 选择策略：
 * - @c 0 对应 @ref TranslationMode::LINE（默认）；
 * - @c 1 对应 @ref TranslationMode::RANDOM。
 *
 * @note 该类以真实时钟驱动，因此适合实时仿真与演示；若需要严格可复现的
 *       离线回放，建议直接使用具体策略并按固定 @p dt 调用 @c advance。
 */
class TranslationGenerator
{
public:
    /**
     * @brief 根据 YAML 配置创建平移策略。
     *
     * @param config @c translation 配置节点。
     */
    explicit TranslationGenerator(const YAML::Node& config);

    /**
     * @brief 按显式时间增量推进运动。
     *
     * @param dt 时间增量，单位秒。
     * @return 推进后的平移状态。
     */
    TranslationState update(double dt);

    /**
     * @brief 按真实时间戳推进运动。
     *
     * 首次调用仅记录 @p time 并返回当前（初始）状态，之后的调用根据与上次
     * 时间戳的差值推进。该设计可避免启动瞬间出现一个巨大的 @p dt。
     *
     * @param time 当前时间戳。
     * @return 推进后的平移状态。
     */
    TranslationState update(const std::chrono::steady_clock::time_point& time);

    /**
     * @brief 将运动重置到初始状态，并清除已记录的时间戳。
     */
    void reset();

    /**
     * @brief 读取当前平移状态，不推进时间。
     *
     * @return 当前平移状态。
     */
    TranslationState state() const;

    /// @return 当前使用的平移模式。
    TranslationMode mode() const { return mode_; }

private:
    TranslationMode mode_{TranslationMode::LINE};         ///< 当前平移模式。
    std::unique_ptr<TranslationMotion> motion_;           ///< 具体运动策略。
    bool has_last_time_{false};                           ///< 是否已记录时间戳。
    std::chrono::steady_clock::time_point last_time_;     ///< 上一次时间戳。
};

} // namespace motion_generator

#endif // VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_TRANSLATE_HPP_
