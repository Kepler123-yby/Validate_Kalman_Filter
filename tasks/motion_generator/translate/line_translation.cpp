/**
 * @file line_translation.cpp
 * @brief @ref motion_generator::LineTranslation 的实现。
 */

#include "line_translation.hpp"

#include "translate_config.hpp"

#include <cmath>

namespace motion_generator
{

LineTranslation::LineTranslation(const YAML::Node& config)
{
    const auto line = detail::section(config, "line");
    const auto initial = detail::section(config, "initial");

    const Eigen::Vector3d initial_position =
        detail::read_vector3(initial["position"], Eigen::Vector3d::Zero());

    line_start_ = detail::read_vector3(line["start"], initial_position);
    line_end_ = detail::read_vector3(
        line["end"], initial_position + Eigen::Vector3d::UnitX());
    one_way_time_ = line["one_way_time"].as<double>(2.0);

    reset();
}

void LineTranslation::reset()
{
    elapsed_time_ = 0.0;
    state_.position = line_start_;
    state_.velocity.setZero();
    state_.acceleration.setZero();
}

TranslationState LineTranslation::state() const
{
    return state_;
}

TranslationState LineTranslation::advance(const double dt)
{
    elapsed_time_ += dt;
    const double cycle_time = 2.0 * one_way_time_;
    state_ = evaluate(std::fmod(elapsed_time_, cycle_time));
    return state_;
}

TranslationState LineTranslation::evaluate(const double time_in_cycle) const
{
    constexpr double pi = 3.14159265358979323846;

    const Eigen::Vector3d distance = line_end_ - line_start_;
    // 位置/速度/加速度的公共系数，下标 0 为位置，1 为速度，2 为加速度。
    const double position_scale = 0.5;
    const double velocity_scale = 0.5 * pi / one_way_time_;
    const double acceleration_scale =
        0.5 * pi * pi / (one_way_time_ * one_way_time_);

    TranslationState result;
    if (time_in_cycle <= one_way_time_)
    {
        // 去程：从起点平滑加速到中点速度，再减速到终点。
        const double phase = pi * time_in_cycle / one_way_time_;
        result.position = line_start_ + distance * position_scale
            * (1.0 - std::cos(phase));
        result.velocity = distance * velocity_scale * std::sin(phase);
        result.acceleration = distance * acceleration_scale * std::cos(phase);
    }
    else
    {
        // 回程：反向重复同一速度曲线。
        const double phase = pi * (time_in_cycle - one_way_time_) / one_way_time_;
        result.position = line_end_ - distance * position_scale
            * (1.0 - std::cos(phase));
        result.velocity = -distance * velocity_scale * std::sin(phase);
        result.acceleration = -distance * acceleration_scale * std::cos(phase);
    }

    return result;
}

} // namespace motion_generator
