/**
 * @file generator.hpp
 * @brief 运动状态生成器：在后台线程按固定周期合成完整的机器人真值。
 *
 * @ref motion_generator::Generator 组合平移模块、自旋模块与机器人模型，并在
 * 独立线程中按 @c generator.update_rate（单位 ms）持续更新。消费者通过
 * @ref motion_generator::Generator::generate 或
 * @ref motion_generator::Generator::wait_for_next 获取带序号的快照，
 * 从而安全地与仿真时钟同步。
 */

#ifndef VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_GENERATOR_HPP_
#define VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_GENERATOR_HPP_

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include "robot/robot.hpp"
#include "spin/spin.hpp"
#include "translate/translate.hpp"

namespace motion_generator
{

/**
 * @brief 某一更新周期结束时的完整真值快照。
 *
 * 快照一旦被复制出来就不会再被后台线程修改，因此消费者可以安全地读取。
 */
struct GeneratorState
{
    Eigen::VectorXd truth_state;      ///< 11 维真值状态。
    Eigen::Vector4d observation;      ///< 当前锁定装甲板的观测值。
    int armor_id{0};                  ///< 当前锁定装甲板编号。
    uint64_t sequence{0};             ///< 单调递增的更新序号。
    std::chrono::steady_clock::time_point timestamp; ///< 该快照对应的时间戳。
};

/**
 * @brief 后台线程驱动的运动状态生成器。
 */
class Generator
{
public:
    /**
     * @brief 从 YAML 配置构造生成器并启动后台运动线程。
     *
     * @param config 顶层配置节点，需包含 @c spin、@c translation、@c robot
     *               与可选的 @c generator 子节点。
     */
    explicit Generator(const YAML::Node& config);

    /**
     * @brief 停止后台线程并回收资源。
     */
    ~Generator();

    Generator(const Generator&) = delete;
    Generator& operator=(const Generator&) = delete;

    /**
     * @brief 获取当前最新的真值快照，不阻塞。
     *
     * @return 最新快照。
     */
    GeneratorState generate();

    /**
     * @brief 阻塞等待产生比 @p sequence 更新的快照。
     *
     * @param sequence 调用方已处理的最新序号。
     * @return 序号大于 @p sequence 的最新快照。
     */
    GeneratorState wait_for_next(uint64_t sequence);

private:
    /// 后台线程主循环：按固定周期推进运动并发布快照。
    void motion_loop();

    /**
     * @brief 将平移与自旋状态拼装为 15 维原始状态。
     *
     * @param translation 平移真值。
     * @param spin        自旋真值。
     * @return 15 维原始状态向量。
     */
    Eigen::VectorXd make_raw_states(
        const TranslationState& translation, const SpinState& spin) const;

    /**
     * @brief 在已持有 @c state_mutex_ 的前提下复制当前快照。
     *
     * @return 当前快照。
     */
    GeneratorState copy_state() const;

    std::unique_ptr<TranslationGenerator> translation_; ///< 平移模块。
    std::unique_ptr<SpinGenerator> spin_;               ///< 自旋模块。
    std::unique_ptr<Robot> target_;                     ///< 机器人刚体模型。

    int update_period_ms_{10};                          ///< 后台更新周期，单位 ms。

    std::mutex state_mutex_;                            ///< 保护下方共享状态。
    std::condition_variable state_condition_;           ///< 用于等待新快照。
    std::thread motion_thread_;                         ///< 后台运动线程。

    TranslationState translation_state_;                ///< 最近一次平移状态。
    SpinState spin_state_;                              ///< 最近一次自旋状态。
    uint64_t sequence_{0};                              ///< 最新快照序号。
    std::chrono::steady_clock::time_point timestamp_;   ///< 最新快照时间戳。

    std::atomic<bool> stop_motion_{false};              ///< 线程退出标志。
};

} // namespace motion_generator

#endif // VALIDATE_KALMAN_FILTER_MOTION_GENERATOR_GENERATOR_HPP_
