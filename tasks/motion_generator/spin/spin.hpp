/**
 * @file spin.hpp
 * @brief 小陀螺（自旋）运动生成器。
 *
 * 自旋运动只负责机器人的绕竖轴旋转以及装甲板相对旋转中心的几何偏移，
 * 不涉及机器人中心的平移。角速度由若干个 @ref tools::SineFunction 叠加而成，
 * 角加速度与累计角度分别通过解析求导与积分得到。
 */

#ifndef VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_SPIN_HPP_
#define VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_SPIN_HPP_

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <vector>

#include "tools/sine_function/sine_function.hpp"

namespace motion_generator
{

/**
 * @brief 某一时刻的自旋真值状态。
 */
struct SpinState
{
    double yaw{0.0};                 ///< 累计偏航角，单位 rad，归一化到 @f$ [-\pi,\pi) @f$。
    double angular_velocity{0.0};    ///< 偏航角速度，单位 rad/s。
    double angular_acceleration{0.0};///< 偏航角加速度，单位 rad/s²。
    double forward_radius{0.0};      ///< 正面装甲板到旋转中心的距离，单位 m。
    double beside_radius{0.0};       ///< 侧面装甲板到旋转中心的距离，单位 m。
    double height_difference{0.0};   ///< 侧面装甲板相对中心的高度差，单位 m。
};

/**
 * @brief 由多个正弦角速度叠加得到的自旋运动生成器。
 *
 * 配置来自 @c spin 节点：
 * - @c number_of_sine_functions：叠加的正弦函数个数；
 * - @c A_lists / @c f_lists / @c phi_lists / @c x_lists：各正弦函数的参数；
 * - @c physical_dimensions：@f$ [r_\text{forward}, r_\text{beside}, \Delta h] @f$。
 *
 * 第 @f$ i @f$ 个正弦函数的四个参数列表长度应不小于 @c number_of_sine_functions，
 * 缺省值按 0 处理。
 */
class SpinGenerator
{
public:
    /**
     * @brief 从 YAML 配置构造自旋生成器，并以构造时刻作为运动起点。
     *
     * @param config @c spin 配置节点。
     */
    explicit SpinGenerator(const YAML::Node& config);

    /**
     * @brief 计算给定时刻的自旋状态。
     *
     * @param time 当前绝对时间戳。
     * @return 自旋真值状态。
     */
    SpinState state_at(const std::chrono::steady_clock::time_point& time) const;

private:
    /**
     * @brief 根据相对运动起点的秒数计算自旋状态。
     *
     * @param elapsed_seconds 相对起点的秒数。
     * @return 自旋真值状态。
     */
    SpinState evaluate(double elapsed_seconds) const;

    std::vector<tools::SineFunction> sine_functions_; ///< 叠加的正弦角速度分量。

    double forward_radius_{0.0};    ///< 正面装甲板半径，单位 m。
    double beside_radius_{0.0};     ///< 侧面装甲板半径，单位 m。
    double height_difference_{0.0}; ///< 侧面装甲板高度差，单位 m。

    std::chrono::steady_clock::time_point start_time_; ///< 运动起点。
};

} // namespace motion_generator

#endif // VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_SPIN_HPP_
