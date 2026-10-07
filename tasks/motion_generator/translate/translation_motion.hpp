/**
 * @file translation_motion.hpp
 * @brief 平移运动的抽象策略接口。
 *
 * 不同的平移模式（确定性直线往返、随机机动等）在物理模型与内部状态上差异
 * 很大，但对外都只需要提供“按时间推进”和“读取当前状态”两个能力。这里用
 * 运行时多态（策略模式）把具体实现与上层 @ref motion_generator::TranslationGenerator
 * 解耦，新增运动模式时无需修改原有代码。
 */

#ifndef VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_TRANSLATION_MOTION_HPP_
#define VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_TRANSLATION_MOTION_HPP_

#include "translation_state.hpp"

namespace motion_generator
{

/**
 * @brief 所有平移运动策略的公共接口。
 *
 * 派生类需要维护自己的内部状态（如累计时间、随机目标速度等），并按 @p dt
 * 逐帧推进。@p dt 可能大于内部积分步长，实现应自行细分保证数值稳定。
 */
class TranslationMotion
{
public:
    virtual ~TranslationMotion() = default;

    /**
     * @brief 将运动推进 @p dt 秒并返回推进后的状态。
     *
     * @param dt 距离上一次推进的时间增量，单位秒，必须为非负。
     * @return 推进后的平移真值状态。
     */
    virtual TranslationState advance(double dt) = 0;

    /**
     * @brief 将运动重置到初始状态。
     *
     * 重置后应与刚构造完成时的内部状态完全一致，保证同一配置可复现。
     */
    virtual void reset() = 0;

    /**
     * @brief 读取当前状态，不推进时间。
     *
     * @return 当前平移真值状态的副本。
     */
    virtual TranslationState state() const = 0;
};

} // namespace motion_generator

#endif // VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_TRANSLATION_MOTION_HPP_
