#include "translate.hpp"

#include <algorithm>
#include <cmath>

namespace motion_generator
{

namespace
{

Eigen::Vector3d read_vector(const YAML::Node& node, const Eigen::Vector3d& value)
{
    if (!node || !node.IsSequence() || node.size() != 3)
    {
        return value;
    }
    return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

} // namespace

TranslationGenerator::TranslationGenerator(const YAML::Node& config)
{
    const YAML::Node line = config["line"];
    const YAML::Node random = config["random"];
    const YAML::Node initial = config["initial"];

    const int mode = config["translation_mode"]
        ? config["translation_mode"].as<int>()
        : config["mode"].as<int>(0);
    translation_mode_ = static_cast<TranslationMode>(mode);

    initial_position_ = read_vector(
        initial && initial.IsMap() ? initial["position"] : YAML::Node(),
        Eigen::Vector3d::Zero());
    initial_speed_ = read_vector(
        initial && initial.IsMap() ? initial["velocity"] : YAML::Node(),
        Eigen::Vector3d::Zero());

    line_start_ = read_vector(
        line && line.IsMap() ? line["start"] : YAML::Node(), initial_position_);
    line_end_ = read_vector(
        line && line.IsMap() ? line["end"] : YAML::Node(),
        initial_position_ + Eigen::Vector3d::UnitX());
    line_time_ = line && line.IsMap()
        ? line["one_way_time"].as<double>(2.0)
        : 2.0;

    speed_min_ = read_vector(
        random && random.IsMap() ? random["velocity_min"] : YAML::Node(),
        Eigen::Vector3d::Constant(-1.0));
    speed_max_ = read_vector(
        random && random.IsMap() ? random["velocity_max"] : YAML::Node(),
        Eigen::Vector3d::Constant(1.0));
    max_acceleration_ = random && random.IsMap()
        ? random["max_acceleration"].as<double>(2.0) : 2.0;
    max_jerk_ = random && random.IsMap()
        ? random["max_jerk"].as<double>(10.0) : 10.0;
    response_time_ = random && random.IsMap()
        ? random["response_time"].as<double>(0.5) : 0.5;
    hold_time_min_ = random && random.IsMap()
        ? random["hold_time_min"].as<double>(0.5) : 0.5;
    hold_time_max_ = random && random.IsMap()
        ? random["hold_time_max"].as<double>(2.0) : 2.0;
    stop_probability_ = random && random.IsMap()
        ? random["stop_probability"].as<double>(0.0) : 0.0;
    reverse_probability_ = random && random.IsMap()
        ? random["reverse_probability"].as<double>(0.0) : 0.0;
    integration_step_ = random && random.IsMap()
        ? random["integration_step"].as<double>(0.01) : 0.01;
    random_seed_ = random && random.IsMap()
        ? random["seed"].as<unsigned int>(42) : 42;

    reset();
}

TranslationState TranslationGenerator::update(double dt)
{
    if (translation_mode_ == TranslationMode::LINE)
    {
        return update_line(dt);
    }
    return update_random(dt);
}

TranslationState TranslationGenerator::update(
    const std::chrono::steady_clock::time_point& time)
{
    if (!has_last_time_)
    {
        last_time_ = time;
        has_last_time_ = true;
        return state_;
    }

    const double dt = std::chrono::duration<double>(time - last_time_).count();
    last_time_ = time;
    return update(dt);
}

void TranslationGenerator::reset()
{
    elapsed_time_ = 0.0;
    hold_time_ = 0.0;
    target_speed_.setZero();
    random_engine_.seed(random_seed_);
    has_last_time_ = false;

    state_.position = translation_mode_ == TranslationMode::LINE
        ? line_start_
        : initial_position_;
    state_.speed = translation_mode_ == TranslationMode::LINE
        ? Eigen::Vector3d::Zero()
        : initial_speed_;
    state_.acceleration.setZero();
}

TranslationState TranslationGenerator::state() const
{
    return state_;
}

TranslationMode TranslationGenerator::mode() const
{
    return translation_mode_;
}

TranslationState TranslationGenerator::update_line(double dt)
{
    elapsed_time_ += dt;

    const double cycle_time = 2.0 * line_time_;
    const double time = std::fmod(elapsed_time_, cycle_time);
    const Eigen::Vector3d distance = line_end_ - line_start_;
    const double pi = std::acos(-1.0);

    if (time <= line_time_)
    {
        const double phase = pi * time / line_time_;
        state_.position = line_start_ + distance * 0.5 * (1.0 - std::cos(phase));
        state_.speed = distance * 0.5 * pi / line_time_ * std::sin(phase);
        state_.acceleration = distance * 0.5 * pi * pi
            / (line_time_ * line_time_) * std::cos(phase);
    }
    else
    {
        const double phase = pi * (time - line_time_) / line_time_;
        state_.position = line_end_ - distance * 0.5 * (1.0 - std::cos(phase));
        state_.speed = -distance * 0.5 * pi / line_time_ * std::sin(phase);
        state_.acceleration = -distance * 0.5 * pi * pi
            / (line_time_ * line_time_) * std::cos(phase);
    }

    return state_;
}

TranslationState TranslationGenerator::update_random(double dt)
{
    while (dt > 0.0)
    {
        if (hold_time_ <= 0.0)
        {
            set_random_target();
        }

        const double step = std::min({dt, integration_step_, hold_time_});
        Eigen::Vector3d acceleration = limit_norm(
            (target_speed_ - state_.speed) / response_time_,
            max_acceleration_);
        state_.acceleration += limit_norm(
            acceleration - state_.acceleration, max_jerk_ * step);
        state_.position += state_.speed * step
            + 0.5 * state_.acceleration * step * step;
        state_.speed += state_.acceleration * step;

        for (int i = 0; i < 3; ++i)
        {
            if (state_.speed(i) < speed_min_(i) || state_.speed(i) > speed_max_(i))
            {
                state_.speed(i) = std::clamp(
                    state_.speed(i), speed_min_(i), speed_max_(i));
                state_.acceleration(i) = 0.0;
            }
        }

        dt -= step;
        hold_time_ -= step;
    }

    return state_;
}

void TranslationGenerator::set_random_target()
{
    std::uniform_real_distribution<double> random(0.0, 1.0);
    const double event = random(random_engine_);

    if (event < stop_probability_)
    {
        target_speed_.setZero();
    }
    else if (event < stop_probability_ + reverse_probability_)
    {
        target_speed_ = -state_.speed;
    }
    else
    {
        for (int i = 0; i < 3; ++i)
        {
            std::uniform_real_distribution<double> speed(
                speed_min_(i), speed_max_(i));
            target_speed_(i) = speed(random_engine_);
        }
    }

    for (int i = 0; i < 3; ++i)
    {
        target_speed_(i) = std::clamp(
            target_speed_(i), speed_min_(i), speed_max_(i));
    }

    std::uniform_real_distribution<double> hold(
        hold_time_min_, hold_time_max_);
    hold_time_ = hold(random_engine_);
}

Eigen::Vector3d TranslationGenerator::limit_norm(
    const Eigen::Vector3d& value,
    double limit)
{
    return value.norm() > limit ? value * limit / value.norm() : value;
}

} // namespace motion_generator
