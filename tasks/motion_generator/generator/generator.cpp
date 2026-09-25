#include "generator.hpp"

#include <stdexcept>

namespace motion_generator
{

Generator::Generator(const YAML::Node& config)
{
    auto spin_config = config["spin"];
    auto translation_config = config["translation"];
    auto generator_config = config["generator"];

    translation_ = std::make_unique<TranslationGenerator>(translation_config);
    spin_ = std::make_unique<SpinGenerator>(spin_config);
    target_ = std::make_unique<Robot>(config);

    update_rate_ = generator_config && generator_config["update_rate"]
        ? generator_config["update_rate"].as<int>()
        : 10;
    if (update_rate_ <= 0)
    {
        throw std::invalid_argument("generator.update_rate must be positive");
    }

    motion_thread_ = std::thread(&Generator::motion_loop, this);
}

Generator::~Generator()
{
    stop_motion_ = true;
    if (motion_thread_.joinable())
    {
        motion_thread_.join();
    }
}

void Generator::motion_loop()
{
    while (!stop_motion_)
    {
        const auto time = std::chrono::steady_clock::now();
        const auto translation_state = translation_->update(time);
        const auto spin_state = spin_->get_states(time);
        const auto raw_state = make_raw_states(translation_state, spin_state);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            translation_state_ = translation_state;
            spin_state_ = spin_state;
            target_->update_state(raw_state);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(update_rate_));
    }
}

Eigen::VectorXd Generator::make_raw_states(
    const TranslationState& translation_state,
    const SpinState& spin_state) const
{
    Eigen::VectorXd raw_states(15);
    raw_states <<
        translation_state.position[0],
        translation_state.speed[0],
        translation_state.acceleration[0],
        translation_state.position[1],
        translation_state.speed[1],
        translation_state.acceleration[1],
        translation_state.position[2],
        translation_state.speed[2],
        translation_state.acceleration[2],
        spin_state.yaw,
        spin_state.speed,
        spin_state.acceleration,
        spin_state.forword_radius,
        spin_state.beside_radius,
        spin_state.height_diff;
    return raw_states;
}

GeneratorState Generator::generate()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return GeneratorState{
        target_->get_states(),
        target_->get_observation()};
}

} // namespace motion_generator
