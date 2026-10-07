/**
 * @file armor_model.hpp
 * @brief 自瞄场景下装甲板机器人的运动/观测模型。
 *
 * @ref kalman_filter::ArmorModel 只包含与物理模型有关的纯计算，不持有滤波器
 * 状态，也不处理时间。把模型从 @ref kalman_filter::Kalman 中分离出来，可以让
 * “模型定义”和“滤波调度”各自独立演化，也方便单独对雅可比、观测函数做单元测试。
 *
 * @par 状态定义
 * 采用 tools::robot_state 中定义的 11 维排列：
 * @f[
 *   [x, v_x, y, v_y, z, v_z, yaw, \dot{yaw}, r_f, r_b, \Delta h]
 * @f]
 * 其中 xyz 为机器人旋转中心位置，yaw 为机体偏航角，@f$ r_f, r_b @f$ 为
 * 正面/侧面装甲板半径，@f$ \Delta h @f$ 为侧面装甲板高度差。
 *
 * @par 过程模型
 * 位置-速度为匀速模型，yaw 由常值角速度积分，几何尺寸为随机游走。
 *
 * @par 观测模型
 * 每块装甲板先由中心状态推出其在相机坐标系下的位置，再转换为
 * @f$ (\text{bearing}, \text{pitch}, \text{distance}, \text{armor\_angle}) @f$。
 */

#ifndef VALIDATE_KALMAN_FILTER_FITTER_MODEL_ARMOR_MODEL_HPP_
#define VALIDATE_KALMAN_FILTER_FITTER_MODEL_ARMOR_MODEL_HPP_

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace kalman_filter
{

/**
 * @brief 装甲板机器人的非线性运动与观测模型。
 */
class ArmorModel
{
public:
    /**
     * @brief 从 YAML 配置读取几何尺寸与过程噪声。
     *
     * @param config @c kalman_filter 配置节点，需包含 @c physical_dimensions
     *               与 @c ekf.Q 子节点。
     */
    explicit ArmorModel(const YAML::Node& config);

    /// @return 物理尺寸 @f$ [r_f, r_b, \Delta h] @f$，单位 m。
    const Eigen::Vector3d& physical_dimensions() const
    {
        return physical_dimensions_;
    }

    /**
     * @brief 由首帧观测反解初始状态。
     *
     * @param observation 首帧观测 @f$ [\text{bearing}, \text{pitch}, \text{distance}, \text{armor\_angle}] @f$。
     * @param armor_id    该观测对应的装甲板编号。
     * @return 11 维初始状态，速度与角速度置零。
     */
    Eigen::VectorXd initial_state(
        const Eigen::VectorXd& observation, int armor_id) const;

    /**
     * @brief 由首帧观测与观测噪声推导初始协方差对角线。
     *
     * 通过观测函数对状态的逆雅可比把观测噪声传播到位置与偏航角的方差，
     * 其余状态保持配置给定的基础方差。
     *
     * @param observation            首帧观测。
     * @param armor_id               装甲板编号。
     * @param base_covariance        配置中的基础协方差对角线 @c P。
     * @param measurement_covariance 观测噪声协方差矩阵 @c R。
     * @return 初始协方差矩阵的对角线向量。
     */
    Eigen::VectorXd initial_covariance(
        const Eigen::VectorXd& observation, int armor_id,
        const Eigen::VectorXd& base_covariance,
        const Eigen::MatrixXd& measurement_covariance) const;

    /**
     * @brief 计算离散时间 @p dt 下的过程噪声协方差 @f$ Q @f$。
     *
     * 各轴采用连续白噪声加速度模型：
     * @f[
     *   Q_{\text{axis}} =
     *   \begin{bmatrix} \tfrac{1}{2}a\,dt^2 \\ a\,dt \end{bmatrix}
     *   \begin{bmatrix} \tfrac{1}{2}a\,dt^2 & a\,dt \end{bmatrix}
     * @f]
     * 几何尺寸状态采用独立随机游走。
     *
     * @param dt 预测时间，单位 s。
     * @return 11x11 过程噪声协方差矩阵。
     */
    Eigen::MatrixXd process_covariance(double dt) const;

    /**
     * @brief 由状态预测指定装甲板的观测。
     *
     * @param state    11 维状态。
     * @param armor_id 装甲板编号。
     * @return 4 维预测观测。
     */
    Eigen::VectorXd predict_observation(
        const Eigen::VectorXd& state, int armor_id) const;

    /**
     * @brief 计算观测函数在 @p state 处的雅可比矩阵 @f$ H @f$。
     *
     * @param state    11 维状态。
     * @param armor_id 装甲板编号。
     * @return 4x11 观测雅可比矩阵。
     */
    Eigen::MatrixXd observation_jacobian(
        const Eigen::VectorXd& state, int armor_id) const;

    /**
     * @brief 状态加法，并对 yaw 分量做角度归一化。
     * @param state 原状态。
     * @param delta 增量。
     * @return 相加并归一化后的状态。
     */
    static Eigen::VectorXd add_state(
        const Eigen::VectorXd& state, const Eigen::VectorXd& delta);

    /**
     * @brief 状态减法，并对 yaw 分量做最短角差归一化。
     * @param a 被减状态。
     * @param b 减数状态。
     * @return 相减并归一化后的状态。
     */
    static Eigen::VectorXd subtract_state(
        const Eigen::VectorXd& a, const Eigen::VectorXd& b);

    /**
     * @brief 观测减法，并对 bearing、armor_angle 做最短角差归一化。
     * @param a 被减观测。
     * @param b 减数观测。
     * @return 相减并归一化后的观测残差。
     */
    static Eigen::VectorXd subtract_observation(
        const Eigen::VectorXd& a, const Eigen::VectorXd& b);

private:
    /**
     * @brief 计算球坐标 (yaw, pitch, distance) 对笛卡尔坐标的雅可比。
     *
     * @param xyz 笛卡尔坐标。
     * @return 3x3 雅可比矩阵 @f$ \partial(\text{yaw},\text{pitch},\text{distance})/\partial xyz @f$。
     */
    static Eigen::MatrixXd ypd_jacobian(const Eigen::Vector3d& xyz);

    /**
     * @brief 由状态与装甲板编号计算该装甲板的三维位置。
     *
     * @param state    11 维状态。
     * @param armor_id 装甲板编号。
     * @return 装甲板在相机坐标系下的位置。
     */
    static Eigen::Vector3d armor_position(
        const Eigen::VectorXd& state, int armor_id);

    Eigen::Vector3d acceleration_noise_{Eigen::Vector3d::Constant(10.0)};
    Eigen::Vector3d physical_dimension_noise_{Eigen::Vector3d::Zero()};
    double angular_acceleration_noise_{20.0};
    Eigen::Vector3d physical_dimensions_{0.27, 0.25, 0.5};
};

} // namespace kalman_filter

#endif // VALIDATE_KALMAN_FILTER_FITTER_MODEL_ARMOR_MODEL_HPP_
