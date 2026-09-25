#ifndef _GENERATOR_HPP_
#define _GENERATOR_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>

#include "spin/spin.hpp"
#include "translate/translate.hpp"
#include "robot/robot.hpp"

namespace motion_generator
{

using GeneratorState = std::pair<Eigen::VectorXd, Eigen::Vector4d>;

class Generator
{
public:
    explicit Generator(const YAML::Node& config);
    ~Generator();

    GeneratorState generate();

private:
    std::unique_ptr<TranslationGenerator> translation_;
    std::unique_ptr<SpinGenerator> spin_;
    std::mutex state_mutex_;
    std::thread motion_thread_;

    std::unique_ptr<Robot> target_;
    int update_rate_;

    TranslationState translation_state_;
    SpinState spin_state_;

    std::atomic<bool> stop_motion_{false};

private:
    void motion_loop();

    Eigen::VectorXd make_raw_states(
        const TranslationState& translation_state,
        const SpinState& spin_state) const;
};

} // namespace motion_generator

#endif // _GENERATOR_HPP_
