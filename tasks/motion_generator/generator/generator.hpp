#ifndef _GENERATOR_HPP_
#define _GENERATOR_HPP_

#include <yaml-cpp/yaml.h>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>

#include "spin/spin.hpp"
#include "translate/translate.hpp"
#include "robot/robot.hpp"
#include "tools/logger/logger.hpp"

namespace motion_generator
{


class Generator
{
public:
    explicit Generator(const YAML::Node& config);
    ~Generator() = default;

private:
    std::unique_ptr<TranslationGenerator> translation_;
    std::unique_ptr<SpinGenerator> spin_;
    std::mutex state_mutex_;
    std::thread motion_thread_;

    std::unique_ptr<Robot> target_;
    double update_rate_;

    TranslationState translation_state_;
    SpinState spin_state_;

private:
    void motion_loop();
};

} // namespace motion_generator

#endif // _GENERATOR_HPP_