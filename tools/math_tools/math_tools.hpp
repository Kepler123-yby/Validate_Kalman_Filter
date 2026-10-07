/**
 * @file math_tools.hpp
 * @brief 项目通用的角度、坐标与时间工具函数。
 *
 * 该文件只包含无状态的纯函数，避免任何全局状态，便于在仿真、滤波器与测试中复用。
 * 全项目统一约定：
 * - 角度一律使用弧度（rad）；
 * - 三维姿态观测使用 (yaw, pitch, distance) 即 @ref tools::cartesian_to_ypd 的输出。
 */

#ifndef VALIDATE_KALMAN_FILTER_TOOLS_MATH_TOOLS_HPP_
#define VALIDATE_KALMAN_FILTER_TOOLS_MATH_TOOLS_HPP_

#include <Eigen/Dense>

#include <chrono>
#include <cmath>

namespace tools
{

/**
 * @brief 将任意角度归一化到 @f$ [-\pi, \pi) @f$ 区间。
 *
 * 该函数用于处理 yaw、pitch 等周期性角度，保证角度差与角度求和不产生跳变。
 *
 * @param angle 待归一化的角度，单位 rad。
 * @return 归一化后的角度，范围 @f$ [-\pi, \pi) @f$。
 */
double normalize_angle(double angle);

/**
 * @brief 计算两个时刻之间的时间差。
 *
 * @param start 起始时刻。
 * @param end   结束时刻。
 * @return 时间差，单位秒（可正可负）。
 */
double delta_time(
    const std::chrono::steady_clock::time_point& start,
    const std::chrono::steady_clock::time_point& end);

/**
 * @brief 计算两个角度之间的最短有符号角差。
 *
 * 结果等价于 @f$ \mathrm{normalize\_angle}(to - from) @f$，取值范围
 * @f$ [-\pi, \pi) @f$。
 *
 * @param from 起始角度，单位 rad。
 * @param to   目标角度，单位 rad。
 * @return 从 @p from 到 @p to 的最短角差，单位 rad。
 */
double angle_difference(double from, double to);

/**
 * @brief 将弧度转换为角度。
 *
 * @param radians 弧度值。
 * @return 对应的角度值，单位度。
 */
double radians_to_degrees(double radians);

/**
 * @brief 将笛卡尔坐标转换为 (yaw, pitch, distance) 球坐标。
 *
 * 在摄像机/云台坐标系下：
 * @f[
 *   \text{yaw}      = \operatorname{atan2}(y, x) \\
 *   \text{pitch}    = \operatorname{atan2}(z, \sqrt{x^2 + y^2}) \\
 *   \text{distance} = \sqrt{x^2 + y^2 + z^2}
 * @f]
 *
 * @param xyz 笛卡尔坐标，单位 m。
 * @return 形如 @f$ [\text{yaw}, \text{pitch}, \text{distance}] @f$ 的向量，
 *         角度单位为 rad，距离单位为 m。
 */
Eigen::Vector3d cartesian_to_ypd(const Eigen::Vector3d& xyz);

} // namespace tools

#endif // VALIDATE_KALMAN_FILTER_TOOLS_MATH_TOOLS_HPP_
