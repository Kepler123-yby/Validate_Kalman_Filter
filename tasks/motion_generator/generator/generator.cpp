/**
 * @file generator.cpp
 * @brief @ref motion_generator::Generator 的实现。
 */

#include "generator.hpp"

#include <chrono>

namespace motion_generator
{

Generator::Generator(const YAML::Node& config)
{
    // 三个子模块各自从配置中读取自己关心的字段。
    translation_ = std::make_unique<TranslationGenerator>(config["translation"]);
    spin_ = std::make_unique<SpinGenerator>(config["spin"]);
    target_ = std::make_unique<Robot>(config);

    const auto generator_config = config["generator"];
    update_period_ms_ = generator_config
        ? generator_config["update_rate"].as<int>(10)
        : 10;

    motion_thread_ = std::thread(&Generator::motion_loop, this);
}

Generator::~Generator()
{
    stop_motion_ = true;
    state_condition_.notify_all();
    motion_thread_.join();
}

void Generator::motion_loop()
{
    while (!stop_motion_)
    {
        const auto time = std::chrono::steady_clock::now();
        const TranslationState translation_state = translation_->update(time);
        const SpinState spin_state = spin_->state_at(time);
        const Eigen::VectorXd raw_state =
            make_raw_states(translation_state, spin_state);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            translation_state_ = translation_state;
            spin_state_ = spin_state;
            target_->update_state(raw_state);
            timestamp_ = time;
            ++sequence_;
        }
        state_condition_.notify_all();

        std::this_thread::sleep_for(std::chrono::milliseconds(update_period_ms_));
    }
}

Eigen::VectorXd Generator::make_raw_states(
    const TranslationState& translation, const SpinState& spin) const
{
    Eigen::VectorXd raw_states(RawStateIndex::kRawSize);
    raw_states <<
        translation.position[0], translation.velocity[0], translation.acceleration[0],
        translation.position[1], translation.velocity[1], translation.acceleration[1],
        translation.position[2], translation.velocity[2], translation.acceleration[2],
        spin.yaw, spin.angular_velocity, spin.angular_acceleration,
        spin.forward_radius, spin.beside_radius, spin.height_difference;
    return raw_states;
}

GeneratorState Generator::generate()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return copy_state();
}

GeneratorState Generator::wait_for_next(const uint64_t sequence)
{
    std::unique_lock<std::mutex> lock(state_mutex_);
    state_condition_.wait(lock, [this, sequence]
    {
        return sequence_ > sequence;
    });
    return copy_state();
}

GeneratorState Generator::copy_state() const
{
    return GeneratorState{
        target_->states(), target_->observation(), target_->locked_id(),
        sequence_, timestamp_};
}

} // namespace motion_generator
