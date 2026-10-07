/**
 * @file translate_config.hpp
 * @brief 平移运动策略共享的 YAML 读取辅助函数。
 *
 * 该文件为 header-only，只包含内联的小工具函数，避免在多个 .cpp 中重复
 * 实现相同的配置解析逻辑。
 */

#ifndef VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_TRANSLATE_CONFIG_HPP_
#define VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_TRANSLATE_CONFIG_HPP_

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <array>

namespace motion_generator
{
namespace detail
{

/**
 * @brief 读取长度为 3 的 YAML 序列为三维向量。
 *
 * @param node     待读取的节点，若为空或格式不符则返回 @p fallback。
 * @param fallback 缺省值。
 * @return 解析得到的三维向量。
 */
inline Eigen::Vector3d read_vector3(
    const YAML::Node& node, const Eigen::Vector3d& fallback)
{
    const auto data = node.as<std::array<double, 3>>(
        std::array<double, 3>{fallback[0], fallback[1], fallback[2]});
    return Eigen::Vector3d(data[0], data[1], data[2]);
}

/**
 * @brief 读取子配置段，缺失时返回空 Map 节点。
 *
 * @param config 父配置节点。
 * @param name   子节点名称。
 * @return 子节点，若不存在则为空 Map。
 */
inline YAML::Node section(const YAML::Node& config, const char* name)
{
    return config[name] ? config[name] : YAML::Node(YAML::NodeType::Map);
}

} // namespace detail
} // namespace motion_generator

#endif // VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_TRANSLATE_CONFIG_HPP_
