/**
 * @file robot.hpp
 * @brief 机器人刚体模型：把运动生成器的原始状态转换为滤波状态与装甲板观测。
 *
 * @ref motion_generator::Robot 位于运动生成器的末端，职责是：
 * 1. 通过配置的线性映射把 15 维“原始运动状态”投影为 11 维滤波状态；
 * 2. 依据机器人中心的平移/自旋状态计算每块装甲板在相机坐标系下的观测
 *    @f$ (\text{bearing}, \text{pitch}, \text{distance}, \text{armor\_angle}) @f$；
 * 3. 依据可见性阈值选择当前被跟踪的装甲板，并统计切换次数。
 *
 * 距离较近或几何尺寸的具体数值均由配置给出，便于构造不同的测试场景。
 */

#ifndef VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_ROBOT_HPP_
#define VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_ROBOT_HPP_

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <vector>

#include "tools/math_tools/math_tools.hpp"
#include "tools/robot_state/robot_state.hpp"

namespace motion_generator
{

/**
 * @brief 15 维原始运动状态的下标。
 *
 * 排列顺序为：
 * @f[
 *   [x, v_x, a_x, y, v_y, a_y, z, v_z, a_z,
 *    yaw, \dot{yaw}, \ddot{yaw}, r_f, r_b, \Delta h]
 * @f]
 * 由 @ref Generator 组合平移模块与自旋模块的输出得到。
 */
enum RawStateIndex
{
    kRawCenterX = 0,              ///< 中心 x。
    kRawVelocityX = 1,            ///< x 方向速度。
    kRawAccelerationX = 2,        ///< x 方向加速度。
    kRawCenterY = 3,              ///< 中心 y。
    kRawVelocityY = 4,            ///< y 方向速度。
    kRawAccelerationY = 5,        ///< y 方向加速度。
    kRawCenterZ = 6,              ///< 中心 z。
    kRawVelocityZ = 7,            ///< z 方向速度。
    kRawAccelerationZ = 8,        ///< z 方向加速度。
    kRawYaw = 9,                  ///< 偏航角。
    kRawAngularVelocity = 10,     ///< 偏航角速度。
    kRawAngularAcceleration = 11, ///< 偏航角加速度。
    kRawForwardRadius = 12,       ///< 正面装甲板半径。
    kRawBesideRadius = 13,        ///< 侧面装甲板半径。
    kRawHeightDifference = 14,    ///< 侧面装甲板高度差。
    kRawSize = 15                 ///< 原始状态维数。
};

/**
 * @brief 单块装甲板的编号与观测值。
 */
struct Armor
{
    /**
     * @brief 构造指定编号的装甲板，观测值初始化为 0。
     * @param armor_id 装甲板编号。
     */
    explicit Armor(int armor_id);

    int id;                             ///< 装甲板编号，偶数表示正面。
    Eigen::Vector4d observation;        ///< 观测 @f$ [\text{bearing}, \text{pitch}, \text{distance}, \text{armor\_angle}] @f$。
};

/**
 * @brief 四装甲板机器人刚体模型。
 */
class Robot
{
public:
    Robot() = default;

    /**
     * @brief 从 YAML 配置构造机器人并初始化内部状态。
     *
     * @param config 顶层配置节点，机器人参数位于 @c robot 子节点。
     * @note 配置中 @c raw2state_mat 的行数、列数必须分别为 11 与 15；
     *       装甲板数量、阈值与初始状态均由该节点给出。
     */
    explicit Robot(const YAML::Node& config);
    ~Robot() = default;

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    /**
     * @brief 使用一帧原始运动状态更新机器人真值、装甲板观测与锁定目标。
     *
     * @param raw_state 长度为 @ref kRawSize 的原始状态向量。
     */
    void update_state(const Eigen::VectorXd& raw_state);

    /// @return 当前 11 维滤波状态（真值）。
    const Eigen::VectorXd& states() const { return states_; }

    /// @return 当前被锁定装甲板的观测值。
    const Eigen::Vector4d& observation() const
    {
        return armors_[locked_id_].observation;
    }

    /// @return 当前被锁定的装甲板编号。
    int locked_id() const { return locked_id_; }

    /// @return 自上次构造以来装甲板切换的次数。
    int switch_times() const { return switch_times_; }

    /// @return 装甲板数量。
    int armor_count() const { return static_cast<int>(armors_.size()); }

private:
    /**
     * @brief 通过 @c raw2state_mat_ 计算 11 维滤波状态。
     * @param raw_state 归一化后的原始状态。
     */
    void compute_states(const Eigen::VectorXd& raw_state);

    /**
     * @brief 根据机器人中心与自旋状态计算全部装甲板观测。
     * @param raw_state 归一化后的原始状态。
     */
    void compute_observations(const Eigen::VectorXd& raw_state);

    /**
     * @brief 依据可见性阈值重新选择锁定装甲板，并更新切换次数。
     */
    void update_locked_id();

    std::vector<Armor> armors_;      ///< 全部装甲板。
    Eigen::VectorXd states_;         ///< 11 维真值状态。
    int locked_id_{0};               ///< 当前锁定装甲板编号。
    int switch_times_{0};            ///< 锁定目标切换次数。

    double detect_min_threshold_{-M_PI / 3.0}; ///< 可见角差下限，单位 rad。
    double detect_max_threshold_{M_PI / 3.0};  ///< 可见角差上限，单位 rad。

    Eigen::MatrixXd raw_to_state_;   ///< 原始状态到滤波状态的线性映射。
};

} // namespace motion_generator

#endif // VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_ROBOT_HPP_
