#include "generator.hpp"

namespace motion_generator
{

Generator::Generator(const YAML::Node& config)
{
    auto spinConfig = config["spin"];
    auto translationConfig = config["translation"];
    auto robotConfig = config["robot"];

    translation_ = std::make_unique<TranslationGenerator>(translationConfig);
    spin_ = std::make_unique<SpinGenerator>(spinConfig);
    target_ = std::make_unique<Robot>(robotConfig);
}

void Generator::motion_loop()
{
    while (true) {
        auto time = std::chrono::steady_clock::now();
        auto translarion_states = translation_->update(time);
        auto spin_state = spin_->get_states(time);
        
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            translation_state_ = translarion_states;
            spin_state_ = spin_state;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
}

} // namespace motion_generator