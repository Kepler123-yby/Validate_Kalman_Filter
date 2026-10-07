/**
 * @file robot_state.hpp
 * @brief 全项目共享的机器人状态与观测向量布局约定。
 *
 * 仿真真值、EKF 状态与观测都使用定长 @c Eigen::VectorXd 存储，但若在各处
 * 直接写 @c state[6] 之类的魔术下标，可读性与可维护性都很差。该头文件把
 * 下标集中定义为具名常量，供运动生成器、滤波器与测试共同引用，避免出现
 * 两套彼此不一致的“状态排列”。
 *
 * @note 这些常量只描述布局，不持有数据；本文件为 header-only。
 */

#ifndef VALIDATE_KALMAN_FILTER_TOOLS_ROBOT_STATE_HPP_
#define VALIDATE_KALMAN_FILTER_TOOLS_ROBOT_STATE_HPP_

namespace tools
{
namespace robot_state
{

/**
 * @brief 11 维滤波/真值状态的下标。
 *
 * 排列顺序为：
 * @f[
 *   [x, v_x, y, v_y, z, v_z, yaw, \dot{yaw}, r_\text{forward},
 *    r_\text{beside}, \Delta h]
 * @f]
 * 位置单位为 m，速度单位为 m/s，yaw 单位为 rad，角速度单位为 rad/s，
 * 三个几何量单位为 m。
 */
enum Index
{
    kCenterX = 0,         ///< 中心 x 坐标，单位 m。
    kVelocityX = 1,       ///< x 方向速度，单位 m/s。
    kCenterY = 2,         ///< 中心 y 坐标，单位 m。
    kVelocityY = 3,       ///< y 方向速度，单位 m/s。
    kCenterZ = 4,         ///< 中心 z 坐标，单位 m。
    kVelocityZ = 5,       ///< z 方向速度，单位 m/s。
    kYaw = 6,             ///< 偏航角，单位 rad。
    kYawRate = 7,         ///< 偏航角速度，单位 rad/s。
    kForwardRadius = 8,   ///< 正面装甲板半径，单位 m。
    kBesideRadius = 9,    ///< 侧面装甲板半径，单位 m。
    kHeightDifference = 10, ///< 侧面装甲板高度差，单位 m。
    kSize = 11            ///< 状态维数。
};

/**
 * @brief 4 维装甲板观测的下标。
 *
 * 排列顺序为 @f$ [\text{bearing}, \text{pitch}, \text{distance}, \text{armor\_angle}] @f$，
 * 前两者单位为 rad，距离单位为 m，最后一个为装甲板法线角度（rad）。
 */
enum ObservationIndex
{
    kBearing = 0,     ///< 方位角 yaw，单位 rad。
    kPitch = 1,       ///< 俯仰角 pitch，单位 rad。
    kDistance = 2,    ///< 距离，单位 m。
    kArmorAngle = 3,  ///< 装甲板角度，单位 rad。
    kObservationSize = 4 ///< 观测维数。
};

/**
 * @brief 判断装甲板编号是否属于正面装甲板。
 *
 * 约定编号为偶数的装甲板为正面装甲板，奇数编号为侧面装甲板。
 *
 * @param armor_id 装甲板编号。
 * @return 正面装甲板返回 @c true，否则返回 @c false。
 */
inline bool is_front_armor(const int armor_id)
{
    return armor_id % 2 == 0;
}

} // namespace robot_state
} // namespace tools

#endif // VALIDATE_KALMAN_FILTER_TOOLS_ROBOT_STATE_HPP_
