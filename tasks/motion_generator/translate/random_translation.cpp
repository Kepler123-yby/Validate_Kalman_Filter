/**
 * @file random_translation.cpp
 * @brief @ref motion_generator::RandomTranslation 的实现。
 */

#include "random_translation.hpp"

#include "translate_config.hpp"

#include <algorithm>
#include <cmath>

namespace motion_generator
{

RandomTranslation::RandomTranslation(const YAML::Node& config)
{
    const auto random = detail::section(config, "random");
    const auto initial = detail::section(config, "initial");

    initial_position_ =
        detail::read_vector3(initial["position"], Eigen::Vector3d::Zero());
    initial_velocity_ =
        detail::read_vector3(initial["velocity"], Eigen::Vector3d::Zero());

    velocity_min_ =
        detail::read_vector3(random["velocity_min"], Eigen::Vector3d::Constant(-1.0));
    velocity_max_ =
        detail::read_vector3(random["velocity_max"], Eigen::Vector3d::Constant(1.0));

    max_acceleration_ = random["max_acceleration"].as<double>(2.0);
    max_jerk_ = random["max_jerk"].as<double>(10.0);
    response_time_ = random["response_time"].as<double>(0.5);
    hold_time_min_ = random["hold_time_min"].as<double>(0.5);
    hold_time_max_ = random["hold_time_max"].as<double>(2.0);
    stop_probability_ = random["stop_probability"].as<double>(0.0);
    reverse_probability_ = random["reverse_probability"].as<double>(0.0);
    integration_step_ = random["integration_step"].as<double>(0.01);
    random_seed_ = random["seed"].as<unsigned int>(42);

    reset();
}

void RandomTranslation::reset()
{
    hold_time_ = 0.0;
    target_velocity_.setZero();
    random_engine_.seed(random_seed_);

    state_.position = initial_position_;
    state_.velocity = initial_velocity_;
    state_.acceleration.setZero();
}

TranslationState RandomTranslation::state() const
{
    return state_;
}

TranslationState RandomTranslation::advance(double dt)
{
    // 将外层较大的 dt 拆分为不超过 integration_step_ 的若干小步，
    // 保证数值积分的精度与稳定性。
    while (dt > 0.0)
    {
        if (hold_time_ <= 0.0)
        {
            set_random_target();
        }

        const double step = std::min({dt, integration_step_, hold_time_});

        const Eigen::Vector3d commanded_acceleration = limit_norm(
            (target_velocity_ - state_.velocity) / response_time_,
            max_acceleration_);
        state_.acceleration += limit_norm(
            commanded_acceleration - state_.acceleration, max_jerk_ * step);

        state_.position += state_.velocity * step
            + 0.5 * state_.acceleration * step * step;
        state_.velocity += state_.acceleration * step;

        for (int axis = 0; axis < 3; ++axis)
        {
            const bool out_of_range = state_.velocity(axis) < velocity_min_(axis)
                || state_.velocity(axis) > velocity_max_(axis);
            if (out_of_range)
            {
                state_.velocity(axis) = std::clamp(
                    state_.velocity(axis), velocity_min_(axis), velocity_max_(axis));
                state_.acceleration(axis) = 0.0;
            }
        }

        dt -= step;
        hold_time_ -= step;
    }

    return state_;
}

void RandomTranslation::set_random_target()
{
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double event = unit(random_engine_);

    if (event < stop_probability_)
    {
        target_velocity_.setZero();
    }
    else if (event < stop_probability_ + reverse_probability_)
    {
        target_velocity_ = -state_.velocity;
    }
    else
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            std::uniform_real_distribution<double> speed(
                velocity_min_(axis), velocity_max_(axis));
            target_velocity_(axis) = speed(random_engine_);
        }
    }

    for (int axis = 0; axis < 3; ++axis)
    {
        target_velocity_(axis) = std::clamp(
            target_velocity_(axis), velocity_min_(axis), velocity_max_(axis));
    }

    std::uniform_real_distribution<double> hold(hold_time_min_, hold_time_max_);
    hold_time_ = hold(random_engine_);
}

Eigen::Vector3d RandomTranslation::limit_norm(
    const Eigen::Vector3d& value, const double limit)
{
    const double norm = value.norm();
    return norm > limit ? value * (limit / norm) : value;
}

} // namespace motion_generator
