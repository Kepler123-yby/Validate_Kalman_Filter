/**
 * @file sine_function.hpp
 * @brief 单频正弦角速度模型。
 *
 * 该模型用于描述小陀螺（自旋）运动：物体绕自身竖直轴以正弦规律改变角速度，
 * 从而得到平滑且可解析求导/积分的角度历史。
 *
 * 角速度定义：
 * @f[
 *   \omega(t) = A \sin(f t + \phi) + x
 * @f]
 * 其中：
 * - @f$ A @f$ 为角速度幅值（rad/s）；
 * - @f$ f @f$ 为角频率（rad/s）；
 * - @f$ \phi @f$ 为初相位（rad）；
 * - @f$ x @f$ 为角速度偏置（rad/s）。
 */

#ifndef VALIDATE_KALMAN_FILTER_TOOLS_SINE_FUNCTION_HPP_
#define VALIDATE_KALMAN_FILTER_TOOLS_SINE_FUNCTION_HPP_

namespace tools
{

/**
 * @brief 表示 @f$ \omega(t) = A \sin(f t + \phi) + x @f$ 的单频正弦角速度函数。
 *
 * 该类是不可变的纯值对象：一旦构造完成，其参数不再改变，所有查询函数
 * 均可安全并发调用。
 */
class SineFunction
{
public:
    /**
     * @brief 构造正弦角速度函数。
     *
     * @param amplitude         角速度幅值 @f$ A @f$，单位 rad/s。
     * @param angular_frequency 角频率 @f$ f @f$，单位 rad/s。
     * @param phase             初相位 @f$ \phi @f$，单位 rad。
     * @param offset            角速度偏置 @f$ x @f$，单位 rad/s。
     */
    SineFunction(
        double amplitude,
        double angular_frequency,
        double phase,
        double offset);

    /**
     * @brief 计算时刻 @p elapsed_seconds 的角速度 @f$ \omega(t) @f$。
     *
     * @param elapsed_seconds 相对运动起点的秒数。
     * @return 角速度，单位 rad/s。
     */
    double angular_velocity(double elapsed_seconds) const;

    /**
     * @brief 计算时刻 @p elapsed_seconds 的角加速度 @f$ \dot{\omega}(t) @f$。
     *
     * @param elapsed_seconds 相对运动起点的秒数。
     * @return 角加速度，单位 rad/s²。
     */
    double angular_acceleration(double elapsed_seconds) const;

    /**
     * @brief 计算从 0 到 @p elapsed_seconds 的角速度积分，即累计角度。
     *
     * @f[
     *   \theta(t) = \int_0^t \omega(\tau)\,\mathrm{d}\tau
     * @f]
     *
     * @param elapsed_seconds 相对运动起点的秒数。
     * @return 累计角度，单位 rad。
     */
    double angle(double elapsed_seconds) const;

    /// @return 角速度幅值 @f$ A @f$，单位 rad/s。
    double amplitude() const { return amplitude_; }

    /// @return 角频率 @f$ f @f$，单位 rad/s。
    double angular_frequency() const { return angular_frequency_; }

    /// @return 初相位 @f$ \phi @f$，单位 rad。
    double phase() const { return phase_; }

    /// @return 角速度偏置 @f$ x @f$，单位 rad/s。
    double offset() const { return offset_; }

private:
    double amplitude_;         ///< 角速度幅值 @f$ A @f$，单位 rad/s。
    double angular_frequency_; ///< 角频率 @f$ f @f$，单位 rad/s。
    double phase_;             ///< 初相位 @f$ \phi @f$，单位 rad。
    double offset_;            ///< 角速度偏置 @f$ x @f$，单位 rad/s。
};

} // namespace tools

#endif // VALIDATE_KALMAN_FILTER_TOOLS_SINE_FUNCTION_HPP_
