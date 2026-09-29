#ifndef _EKF_HPP_
#define _EKF_HPP_

#include <Eigen/Dense>
#include <cstddef>
#include <functional>
#include <limits>

namespace kalman_flitter
{

// 一致性统计：没有样本时使用 NaN，避免将“未评估”误认为“误差为零”。
// 均值用于观察一致性，不直接作为卡方检验的通过结论。
struct ConsistencyStatistics
{
    double value{std::numeric_limits<double>::quiet_NaN()}; // 最近一次评估值
    Eigen::Index degrees_of_freedom{0}; // 最近一次评估的维数
    std::size_t samples{0}; // 有效样本数
    double mean{std::numeric_limits<double>::quiet_NaN()}; // 评估值的均值
    double normalized_mean{std::numeric_limits<double>::quiet_NaN()}; // 各样本除以自身维数后的均值
};

// 初始 P 以对角方差向量传入，Q、R 使用完整协方差矩阵。
// 调用方保证状态已初始化、维数匹配、回调有效，且评估使用的 P、S 正定。
class EKF
{
public:
    using StateFunction = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
    using VectorOperation = std::function<Eigen::VectorXd(
        const Eigen::VectorXd&, const Eigen::VectorXd&)>;

    EKF() = default;
    EKF(
        const Eigen::VectorXd& x, const Eigen::VectorXd& P,
        VectorOperation x_add = default_add,
        VectorOperation x_sub = default_subtract);

    const Eigen::VectorXd get_x() const { return x_; }
    const Eigen::MatrixXd get_P() const { return P_; }

    // 非线性预测由 f 计算状态，F 为雅可比；Q 为过程噪声协方差矩阵。
    // 省略 f 时使用线性模型 F * x。
    void predict(
        const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q,
        const StateFunction& f);
    void predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q);

    // h 计算预测观测，H 为雅可比；z_sub 计算观测残差，角度分量需取最短角差。
    // R 为测量噪声协方差矩阵；NIS 使用更新前的创新和协方差。
    void update(
        const Eigen::MatrixXd& H, const Eigen::VectorXd& z,
        const Eigen::MatrixXd& R, const StateFunction& h,
        const VectorOperation& z_sub = default_subtract);
    void update(
        const Eigen::MatrixXd& H, const Eigen::VectorXd& z,
        const Eigen::MatrixXd& R);

    // 使用当前 x/P 计算 NEES；评估后验时，在 update 后传入同一时刻的真值。
    // truth 必须与状态排列、装甲参考一致；角度误差由 x_sub 归一化。
    // 每次调用记录一个样本，仅读取上次结果时使用 get_nees。
    double evaluate_nees(const Eigen::VectorXd& truth);
    double get_nis() const { return nis_statistics_.value; }
    double get_nees() const { return nees_statistics_.value; }
    const ConsistencyStatistics& get_nis_statistics() const { return nis_statistics_; }
    const ConsistencyStatistics& get_nees_statistics() const { return nees_statistics_; }

    // 最近一次成功更新的创新与 S；predict 不会清空这两个记录。
    const Eigen::VectorXd& get_innovation() const { return innovation_; }
    const Eigen::MatrixXd& get_innovation_covariance() const { return innovation_covariance_; }

    // 只清空评估记录，不改变滤波状态。
    void reset_evaluation();

private:
    // 滤波状态及完整协方差；初值为对角阵，递推后通常不再是对角阵。
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;

    // 默认使用普通向量加减；含角度的状态需传入自定义回调。
    VectorOperation x_add_;
    VectorOperation x_sub_;

    // 观测创新与一致性评估结果
    Eigen::VectorXd innovation_;
    Eigen::MatrixXd innovation_covariance_;
    ConsistencyStatistics nis_statistics_;
    ConsistencyStatistics nees_statistics_;

private:
    static Eigen::VectorXd default_add(
        const Eigen::VectorXd& a, const Eigen::VectorXd& b);
    static Eigen::VectorXd default_subtract(
        const Eigen::VectorXd& a, const Eigen::VectorXd& b);

    // 协方差对称化与统计更新
    static Eigen::MatrixXd symmetrize(const Eigen::MatrixXd& value);
    static void record_statistics(
        ConsistencyStatistics& statistics, double value, Eigen::Index dimension);
};

} // namespace kalman_flitter

#endif // _EKF_HPP_
