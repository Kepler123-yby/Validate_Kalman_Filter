/**
 * @file translation_state.hpp
 * @brief 机器人中心的平移运动状态定义。
 */

#ifndef VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_TRANSLATION_STATE_HPP_
#define VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_TRANSLATION_STATE_HPP_

#include <Eigen/Dense>

namespace motion_generator
{

/**
 * @brief 机器人中心在世界坐标系下的平移状态。
 *
 * 位置、速度、加速度均为三维量，单位分别为 m、m/s、m/s²。该结构是运动的
 * 真值，会与 EKF 的状态量直接对比。
 */
struct TranslationState
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};     ///< 位置，单位 m。
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};     ///< 速度，单位 m/s。
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()}; ///< 加速度，单位 m/s²。
};

} // namespace motion_generator

#endif // VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_TRANSLATION_STATE_HPP_
