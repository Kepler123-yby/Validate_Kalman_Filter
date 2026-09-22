#include "translate.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>

namespace motion_generator
{

namespace
{

Eigen::Vector3d read_vector3(
    const YAML::Node& node,
    const Eigen::Vector3d& default_value,
    const std::string& name)
{
    if (!node)
    {
        return default_value;
    }

    if (!node.IsSequence() || node.size() != 3)
    {
        throw std::invalid_argument(name + " must contain exactly three numbers");
    }

    return {
        node[0].as<double>(),
        node[1].as<double>(),
        node[2].as<double>()};
}

YAML::Node find_node(
    YAML::Node root,
    YAML::Node group,
    const char* grouped_key,
    const char* flat_key)
{
    if (group && group[grouped_key])
    {
        return group[grouped_key];
    }
    return root[flat_key];
}

template <typename T>
T read_value(
    YAML::Node root,
    YAML::Node group,
    const char* grouped_key,
    const char* flat_key,
    const T& default_value)
{
    const YAML::Node node = find_node(root, group, grouped_key, flat_key);
    return node ? node.as<T>() : default_value;
}

TranslationMode read_mode(YAML::Node root, YAML::Node config)
{
    YAML::Node mode_node;
    if (root["mode"])
    {
        mode_node = root["mode"];
    }
    else if (root["translation_mode"])
    {
        mode_node = root["translation_mode"];
    }
    else if (config["translation_mode"])
    {
        mode_node = config["translation_mode"];
    }
    if (!mode_node)
    {
        return TranslationMode::LINE;
    }

    std::string mode = mode_node.Scalar();
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char character) {
        return static_cast<char>(std::toupper(character));
    });

    if (mode == "LINE" || mode == "0")
    {
        return TranslationMode::LINE;
    }
    if (mode == "RANDOM" || mode == "1")
    {
        return TranslationMode::RANDOM;
    }

    throw std::invalid_argument("translation mode must be LINE/0 or RANDOM/1");
}

void require_positive(double value, const char* name)
{
    if (!std::isfinite(value) || value <= 0.0)
    {
        throw std::invalid_argument(std::string(name) + " must be positive");
    }
}

void require_probability(double value, const char* name)
{
    if (!std::isfinite(value) || value < 0.0 || value > 1.0)
    {
        throw std::invalid_argument(std::string(name) + " must be in [0, 1]");
    }
}

} // namespace

TranslationGenerator::TranslationGenerator(const YAML::Node& config)
{
    YAML::Node full_config = config;
    YAML::Node root = full_config["translation"]
        ? full_config["translation"]
        : full_config;
    YAML::Node initial = root["initial"];
    YAML::Node line = root["line"];
    YAML::Node random = root["random"];

    translation_mode_ = read_mode(root, full_config);

    initial_position_ = read_vector3(
        find_node(root, initial, "position", "initial_position"),
        Eigen::Vector3d::Zero(),
        "translation.initial.position");
    initial_velocity_ = read_vector3(
        find_node(root, initial, "velocity", "initial_velocity"),
        Eigen::Vector3d::Zero(),
        "translation.initial.velocity");

    line_start_ = read_vector3(
        find_node(root, line, "start", "line_start"),
        initial_position_,
        "translation.line.start");
    line_end_ = read_vector3(
        find_node(root, line, "end", "line_end"),
        initial_position_ + Eigen::Vector3d::UnitX(),
        "translation.line.end");
    line_one_way_time_ = read_value<double>(
        root, line, "one_way_time", "line_one_way_time", 2.0);

    velocity_min_ = read_vector3(
        find_node(root, random, "velocity_min", "random_velocity_min"),
        Eigen::Vector3d::Constant(-1.0),
        "translation.random.velocity_min");
    velocity_max_ = read_vector3(
        find_node(root, random, "velocity_max", "random_velocity_max"),
        Eigen::Vector3d::Constant(1.0),
        "translation.random.velocity_max");
    max_acceleration_ = read_value<double>(
        root, random, "max_acceleration", "random_max_acceleration", 2.0);
    max_jerk_ = read_value<double>(
        root, random, "max_jerk", "random_max_jerk", 10.0);
    response_time_ = read_value<double>(
        root, random, "response_time", "random_response_time", 0.5);
    hold_time_min_ = read_value<double>(
        root, random, "hold_time_min", "random_hold_time_min", 0.5);
    hold_time_max_ = read_value<double>(
        root, random, "hold_time_max", "random_hold_time_max", 2.0);
    stop_probability_ = read_value<double>(
        root, random, "stop_probability", "random_stop_probability", 0.0);
    reverse_probability_ = read_value<double>(
        root, random, "reverse_probability", "random_reverse_probability", 0.0);
    integration_step_ = read_value<double>(
        root, random, "integration_step", "random_integration_step", 0.01);
    random_seed_ = read_value<std::uint32_t>(
        root, random, "seed", "random_seed", 42U);

    if (translation_mode_ == TranslationMode::LINE)
    {
        require_positive(line_one_way_time_, "translation.line.one_way_time");
    }
    else
    {
        require_positive(max_acceleration_, "translation.random.max_acceleration");
        require_positive(max_jerk_, "translation.random.max_jerk");
        require_positive(response_time_, "translation.random.response_time");
        require_positive(hold_time_min_, "translation.random.hold_time_min");
        require_positive(hold_time_max_, "translation.random.hold_time_max");
        require_positive(integration_step_, "translation.random.integration_step");
        require_probability(stop_probability_, "translation.random.stop_probability");
        require_probability(reverse_probability_, "translation.random.reverse_probability");

        if ((velocity_min_.array() > velocity_max_.array()).any())
        {
            throw std::invalid_argument(
                "translation.random.velocity_min must not exceed velocity_max");
        }
        if (hold_time_min_ > hold_time_max_)
        {
            throw std::invalid_argument(
                "translation.random.hold_time_min must not exceed hold_time_max");
        }
        if (stop_probability_ + reverse_probability_ > 1.0)
        {
            throw std::invalid_argument(
                "translation random stop and reverse probabilities must sum to at most 1");
        }
        if ((initial_velocity_.array() < velocity_min_.array()).any()
            || (initial_velocity_.array() > velocity_max_.array()).any())
        {
            throw std::invalid_argument(
                "translation.initial.velocity must be inside the random velocity range");
        }
    }

    reset();
}

TranslationState TranslationGenerator::update(double dt)
{
    if (!std::isfinite(dt) || dt < 0.0)
    {
        throw std::invalid_argument("translation update dt must be finite and non-negative");
    }
    if (dt == 0.0)
    {
        return state_;
    }

    if (translation_mode_ == TranslationMode::LINE)
    {
        return update_line(dt);
    }
    return update_random(dt);
}

TranslationState TranslationGenerator::update(
    const std::chrono::steady_clock::time_point& time)
{
    if (!has_last_update_time_)
    {
        last_update_time_ = time;
        has_last_update_time_ = true;
        return state_;
    }

    const double dt = std::chrono::duration<double>(time - last_update_time_).count();
    if (dt < 0.0)
    {
        throw std::invalid_argument("translation update time must be monotonic");
    }

    last_update_time_ = time;
    return update(dt);
}

void TranslationGenerator::reset()
{
    elapsed_time_ = 0.0;
    target_hold_remaining_ = 0.0;
    target_velocity_.setZero();
    random_engine_.seed(random_seed_);
    has_last_update_time_ = false;

    state_.position = translation_mode_ == TranslationMode::LINE
        ? line_start_
        : initial_position_;
    state_.velocity = translation_mode_ == TranslationMode::LINE
        ? Eigen::Vector3d::Zero()
        : initial_velocity_;
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

    const Eigen::Vector3d midpoint = 0.5 * (line_start_ + line_end_);
    const Eigen::Vector3d half_displacement = 0.5 * (line_end_ - line_start_);
    const double angular_speed = std::acos(-1.0) / line_one_way_time_;
    const double phase = angular_speed * elapsed_time_;

    state_.position = midpoint - half_displacement * std::cos(phase);
    state_.velocity = half_displacement * angular_speed * std::sin(phase);
    state_.acceleration = half_displacement * angular_speed * angular_speed
        * std::cos(phase);
    return state_;
}

TranslationState TranslationGenerator::update_random(double dt)
{
    double remaining_time = dt;

    while (remaining_time > 0.0)
    {
        if (target_hold_remaining_ <= 0.0)
        {
            sample_target_velocity();
        }

        const double step = std::min(
            {remaining_time, integration_step_, target_hold_remaining_});

        Eigen::Vector3d commanded_acceleration =
            (target_velocity_ - state_.velocity) / response_time_;
        commanded_acceleration = clamp_norm(
            commanded_acceleration, max_acceleration_);

        const Eigen::Vector3d acceleration_delta = clamp_norm(
            commanded_acceleration - state_.acceleration,
            max_jerk_ * step);
        state_.acceleration += acceleration_delta;

        state_.position += state_.velocity * step
            + 0.5 * state_.acceleration * step * step;
        state_.velocity += state_.acceleration * step;

        for (Eigen::Index axis = 0; axis < 3; ++axis)
        {
            const double clamped_velocity = std::clamp(
                state_.velocity(axis), velocity_min_(axis), velocity_max_(axis));
            if (clamped_velocity != state_.velocity(axis))
            {
                state_.velocity(axis) = clamped_velocity;
                state_.acceleration(axis) = 0.0;
            }
        }

        remaining_time -= step;
        target_hold_remaining_ -= step;
    }

    return state_;
}

void TranslationGenerator::sample_target_velocity()
{
    std::uniform_real_distribution<double> event_distribution(0.0, 1.0);
    const double event = event_distribution(random_engine_);

    if (event < stop_probability_)
    {
        target_velocity_.setZero();
    }
    else if (event < stop_probability_ + reverse_probability_)
    {
        target_velocity_ = -state_.velocity;
        for (Eigen::Index axis = 0; axis < 3; ++axis)
        {
            target_velocity_(axis) = std::clamp(
                target_velocity_(axis), velocity_min_(axis), velocity_max_(axis));
        }
    }
    else
    {
        for (Eigen::Index axis = 0; axis < 3; ++axis)
        {
            std::uniform_real_distribution<double> velocity_distribution(
                velocity_min_(axis), velocity_max_(axis));
            target_velocity_(axis) = velocity_distribution(random_engine_);
        }
    }

    std::uniform_real_distribution<double> hold_distribution(
        hold_time_min_, hold_time_max_);
    target_hold_remaining_ = hold_distribution(random_engine_);
}

Eigen::Vector3d TranslationGenerator::clamp_norm(
    const Eigen::Vector3d& value,
    double max_norm)
{
    const double norm = value.norm();
    if (norm <= max_norm || norm == 0.0)
    {
        return value;
    }
    return value * (max_norm / norm);
}

} // namespace motion_generator
