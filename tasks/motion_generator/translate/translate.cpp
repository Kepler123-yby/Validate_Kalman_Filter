/**
 * @file translate.cpp
 * @brief @ref motion_generator::TranslationGenerator 的实现。
 */

#include "translate.hpp"

#include "line_translation.hpp"
#include "random_translation.hpp"

#include <chrono>

namespace motion_generator
{

TranslationGenerator::TranslationGenerator(const YAML::Node& config)
{
    const int mode = config["translation_mode"].as<int>(
        config["mode"].as<int>(0));
    mode_ = static_cast<TranslationMode>(mode);

    switch (mode_)
    {
    case TranslationMode::RANDOM:
        motion_ = std::make_unique<RandomTranslation>(config);
        break;
    case TranslationMode::LINE:
    default:
        mode_ = TranslationMode::LINE;
        motion_ = std::make_unique<LineTranslation>(config);
        break;
    }
}

TranslationState TranslationGenerator::update(const double dt)
{
    return motion_->advance(dt);
}

TranslationState TranslationGenerator::update(
    const std::chrono::steady_clock::time_point& time)
{
    if (!has_last_time_)
    {
        last_time_ = time;
        has_last_time_ = true;
        return motion_->state();
    }

    const double dt =
        std::chrono::duration<double>(time - last_time_).count();
    last_time_ = time;
    return motion_->advance(dt);
}

void TranslationGenerator::reset()
{
    motion_->reset();
    has_last_time_ = false;
}

TranslationState TranslationGenerator::state() const
{
    return motion_->state();
}

} // namespace motion_generator
