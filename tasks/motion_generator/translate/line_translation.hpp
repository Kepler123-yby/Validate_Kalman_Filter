/**
 * @file line_translation.hpp
 * @brief 两点之间往返的确定性直线平移策略。
 *
 * 机器人在 @c line.start 与 @c line.end 之间往复运动，单程耗时
 * @c line.one_way_time 秒，端点速度为 0。速度曲线使用半个余弦周期，
 * 保证位置、速度、加速度均连续。
 */

#ifndef VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_LINE_TRANSLATION_HPP_
#define VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_LINE_TRANSLATION_HPP_

#include "translation_motion.hpp"

#include <yaml-cpp/yaml.h>

namespace motion_generator
{

/**
 * @brief 在固定两端点之间以余弦速度曲线往返的平移策略。
 *
 * 运动由 @c translation.line 配置：
 * - @c start         起点位置，默认取 @c translation.initial.position；
 * - @c end           终点位置，默认取起点沿 X 轴正方向 1 m；
 * - @c one_way_time  单程时间，单位秒，默认 2.0。
 *
 * 该策略完全由累计时间解析求解，不依赖数值积分，因此重复播放结果一致。
 */
class LineTranslation : public TranslationMotion
{
public:
    /**
     * @brief 从 YAML 配置构造直线往返运动。
     *
     * @param config @c translation 配置节点，需包含 @c line 与 @c initial 子节点。
     */
    explicit LineTranslation(const YAML::Node& config);

    TranslationState advance(double dt) override;
    void reset() override;
    TranslationState state() const override;

private:
    /**
     * @brief 根据一个往返周期内的时间解析计算状态。
     *
     * @param time_in_cycle 距本次往返起点的秒数，范围 @f$ [0, 2 T) @f$。
     * @return 对应时刻的平移状态。
     */
    TranslationState evaluate(double time_in_cycle) const;

    Eigen::Vector3d line_start_{Eigen::Vector3d::Zero()};      ///< 往返起点。
    Eigen::Vector3d line_end_{Eigen::Vector3d::UnitX()};       ///< 往返终点。
    double one_way_time_{2.0};                                 ///< 单程时间，单位 s。
    double elapsed_time_{0.0};                                 ///< 累计运动时间，单位 s。
    TranslationState state_;                                   ///< 当前真值状态。
};

} // namespace motion_generator

#endif // VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_LINE_TRANSLATION_HPP_
