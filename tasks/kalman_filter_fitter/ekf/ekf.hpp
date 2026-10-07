/**
 * @file ekf.hpp
 * @brief 通用扩展卡尔曼滤波器（EKF）。
 *
 * @ref kalman_filter::EKF 刻意不依赖任何具体机器人模型：预测、观测函数、
 * 雅可比矩阵以及角度相关的加减法全部由调用方传入。这样同一份滤波器实现
 * 既可用于本项目的自瞄模型，也可用于其他非线性系统。
 *
 * 该实现包含以下特性：
 * - 使用 Joseph 形式更新协方差，提升数值稳定性；
 * - 通过 LDLT 分解求解，避免显式求逆；
 * - 内置 NIS / NEES 一致性统计，便于评估模型与参数。
 */

#ifndef VALIDATE_KALMAN_FILTER_KALMAN_FILTER_FITTER_EKF_HPP_
#define VALIDATE_KALMAN_FILTER_KALMAN_FILTER_FITTER_EKF_HPP_

#include <Eigen/Dense>

#include <cstddef>
#include <functional>
#include <limits>

namespace kalman_filter
{

/**
 * @brief 一致性检验统计量。
 *
 * 没有样本时所有数值字段均为 NaN，避免把“尚未评估”误判为“误差为零”。
 * @c mean 用于观察整体趋势，但结论性的通过/失败判断应结合卡方分布分位数。
 */
struct ConsistencyStatistics
{
    double value{std::numeric_limits<double>::quiet_NaN()}; ///< 最近一次评估值。
    Eigen::Index degrees_of_freedom{0};   ///< 最近一次评估的维数。
    std::size_t samples{0};               ///< 累计有效样本数。
    double mean{std::numeric_limits<double>::quiet_NaN()}; ///< 评估值的均值。
    double normalized_mean{
        std::numeric_limits<double>::quiet_NaN()}; ///< 按维数归一化后的均值。
};

/**
 * @brief 通用扩展卡尔曼滤波器。
 *
 * 状态方程与观测方程分别为：
 * @f[
 *   x_k = f(x_{k-1}) + w, \qquad z_k = h(x_k) + v
 * @f]
 * 其中 @f$ w \sim \mathcal{N}(0, Q) @f$，@f$ v \sim \mathcal{N}(0, R) @f$。
 *
 * @par 线程安全
 * 单实例非线程安全；每个线程应使用独立的滤波器实例。
 */
class EKF
{
public:
    /**
     * @brief 非线性状态转移函数 @f$ f(x) @f$。
     */
    using StateFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;

    /**
     * @brief 自定义向量运算，用于处理角度等具有周期性的状态分量。
     *
     * 第一个参数为左操作数，第二个为右操作数。
     */
    using VectorOperation = std::function<Eigen::VectorXd(
        const Eigen::VectorXd&, const Eigen::VectorXd&)>;

    EKF() = default;

    /**
     * @brief 构造并初始化滤波器。
     *
     * @param x          初始状态估计。
     * @param P_diagonal 初始协方差矩阵的对角线（按方差给出）。
     * @param x_add      状态加法回调，默认普通加法；含角度状态应传入自定义实现。
     * @param x_sub      状态减法回调，默认普通减法；含角度状态应传入自定义实现。
     */
    EKF(
        const Eigen::VectorXd& x, const Eigen::VectorXd& P_diagonal,
        VectorOperation x_add = default_add,
        VectorOperation x_sub = default_subtract);

    /// @return 当前状态估计 @f$ \hat{x} @f$ 的常量引用。
    const Eigen::VectorXd& state() const { return x_; }

    /// @return 当前估计协方差 @f$ P @f$ 的常量引用。
    const Eigen::MatrixXd& covariance() const { return P_; }

    /**
     * @brief 执行一步预测。
     *
     * @par 公式
     * @f[
     *   \hat{x}^- = f(\hat{x}), \qquad P^- = F P F^\top + Q
     * @f]
     *
     * @param F 状态转移雅可比矩阵 @f$ \partial f / \partial x @f$。
     * @param Q 过程噪声协方差矩阵。
     * @param f 非线性状态转移函数 @f$ f(\cdot) @f$。
     *
     * @note 省略 @p f 时退化为线性模型 @f$ F x @f$，见
     *       @ref predict(const Eigen::MatrixXd&, const Eigen::MatrixXd&)。
     */
    void predict(
        const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q,
        const StateFunction& f);

    /**
     * @brief 使用线性状态转移模型执行一步预测。
     *
     * @param F 状态转移矩阵。
     * @param Q 过程噪声协方差矩阵。
     */
    void predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q);

    /**
     * @brief 使用一帧观测执行一步更新。
     *
     * @par 公式
     * @f[
     *   S = H P^- H^\top + R, \qquad
     *   K = P^- H^\top S^{-1} \\
     *   \hat{x} = \hat{x}^- \oplus K(z - h(\hat{x}^-)), \qquad
     *   P = (I - KH) P^- (I - KH)^\top + K R K^\top
     * @f]
     *
     * @param H     观测函数雅可比矩阵 @f$ \partial h / \partial x @f$。
     * @param z     观测向量。
     * @param R     观测噪声协方差矩阵。
     * @param h     非线性观测函数 @f$ h(\cdot) @f$。
     * @param z_sub 观测残差计算回调，默认普通减法；含角度观测应传入最短角差实现。
     *
     * @note 新息的 NIS 使用更新前的创新与创新协方差计算。
     */
    void update(
        const Eigen::MatrixXd& H, const Eigen::VectorXd& z,
        const Eigen::MatrixXd& R, const StateFunction& h,
        const VectorOperation& z_sub = default_subtract);

    /**
     * @brief 使用线性观测模型执行一步更新。
     *
     * @param H 观测矩阵。
     * @param z 观测向量。
     * @param R 观测噪声协方差矩阵。
     */
    void update(
        const Eigen::MatrixXd& H, const Eigen::VectorXd& z,
        const Eigen::MatrixXd& R);

    /**
     * @brief 计算并记录状态估计的 NEES。
     *
     * @par 公式
     * @f[
     *   \text{NEES} = (x_\text{truth} \ominus \hat{x})^\top
     *                 P^{-1} (x_\text{truth} \ominus \hat{x})
     * @f]
     *
     * 评估后验一致性时，应在 @ref update 之后、且真值与当前状态处于同一时刻
     * 时调用。@p truth 的状态排列与装甲板参考必须与滤波器一致，角度误差由
     * 构造时传入的 @c x_sub 归一化。
     *
     * @param truth 与当前状态同维的真值向量。
     * @return 本次评估得到的 NEES 值。
     */
    double evaluate_nees(const Eigen::VectorXd& truth);

    /// @return 最近一次更新的 NIS。
    double nis() const { return nis_statistics_.value; }

    /// @return 最近一次评估的 NEES。
    double nees() const { return nees_statistics_.value; }

    /// @return NIS 统计量的常量引用。
    const ConsistencyStatistics& nis_statistics() const { return nis_statistics_; }

    /// @return NEES 统计量的常量引用。
    const ConsistencyStatistics& nees_statistics() const { return nees_statistics_; }

    /// @return 最近一次成功更新的创新向量 @f$ z - h(\hat{x}^-) @f$。
    const Eigen::VectorXd& innovation() const { return innovation_; }

    /// @return 最近一次成功更新的创新协方差 @f$ S @f$。
    const Eigen::MatrixXd& innovation_covariance() const
    {
        return innovation_covariance_;
    }

    /**
     * @brief 清空 NIS/NEES 评估记录，但不改变滤波状态。
     */
    void reset_evaluation();

private:
    static Eigen::VectorXd default_add(
        const Eigen::VectorXd& a, const Eigen::VectorXd& b);
    static Eigen::VectorXd default_subtract(
        const Eigen::VectorXd& a, const Eigen::VectorXd& b);

    /**
     * @brief 消除浮点运算造成的微小不对称，保证协方差矩阵对称。
     * @param value 待对称化的矩阵。
     * @return @f$ \tfrac{1}{2}(M + M^\top) @f$。
     */
    static Eigen::MatrixXd symmetrize(const Eigen::MatrixXd& value);

    /**
     * @brief 将一次评估结果累加进统计量。
     *
     * @param statistics 目标统计量。
     * @param value      本次评估值。
     * @param dimension  本次评估的维数，用于归一化。
     */
    static void record_statistics(
        ConsistencyStatistics& statistics, double value,
        Eigen::Index dimension);

    Eigen::VectorXd x_;              ///< 状态估计。
    Eigen::MatrixXd P_;              ///< 估计协方差。

    VectorOperation x_add_;          ///< 状态加法回调。
    VectorOperation x_sub_;          ///< 状态减法回调。

    Eigen::VectorXd innovation_;             ///< 最近一次创新。
    Eigen::MatrixXd innovation_covariance_;  ///< 最近一次创新协方差。
    ConsistencyStatistics nis_statistics_;   ///< NIS 统计量。
    ConsistencyStatistics nees_statistics_;  ///< NEES 统计量。
};

} // namespace kalman_filter

#endif // VALIDATE_KALMAN_FILTER_KALMAN_FILTER_FITTER_EKF_HPP_
