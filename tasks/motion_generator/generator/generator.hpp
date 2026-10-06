#ifndef _GENERATOR_HPP_
#define _GENERATOR_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <cstdint>

#include "spin/spin.hpp"
#include "translate/translate.hpp"
#include "robot/robot.hpp"

namespace motion_generator
{

struct GeneratorState
{
    Eigen::VectorXd state;
    Eigen::Vector4d observation;
    int armor_id;
    uint64_t sequence;
    std::chrono::steady_clock::time_point timestamp;
};

class Generator
{
public:
    explicit Generator(const YAML::Node& config);
    ~Generator();

    GeneratorState generate();
    GeneratorState wait_for_next(uint64_t sequence);

private:
    std::unique_ptr<TranslationGenerator> translation_;
    std::unique_ptr<SpinGenerator> spin_;
    std::mutex state_mutex_;
    std::condition_variable state_condition_;
    std::thread motion_thread_;

    std::unique_ptr<Robot> target_;
    int update_rate_;

    TranslationState translation_state_;
    SpinState spin_state_;
    uint64_t sequence_{0};
    std::chrono::steady_clock::time_point timestamp_;

    std::atomic<bool> stop_motion_{false};

private:
    void motion_loop();

    Eigen::VectorXd make_raw_states(
        const TranslationState& translation_state,
        const SpinState& spin_state) const;
    GeneratorState copy_state() const;
};

} // namespace motion_generator

#endif // _GENERATOR_HPP_
