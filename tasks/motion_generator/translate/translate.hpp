#ifndef MOTION_GENERATOR_TRANSLATE_HPP_
#define MOTION_GENERATOR_TRANSLATE_HPP_

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cstdint>
#include <random>

namespace motion_generator
{

enum class TranslationMode
{
    LINE = 0,
    RANDOM = 1
};

struct TranslationState
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
};

class TranslationGenerator
{
public:
    explicit TranslationGenerator(const YAML::Node& config);
    ~TranslationGenerator() = default;

    TranslationState update(double dt);
    TranslationState update(const std::chrono::steady_clock::time_point& time);
    void reset();

    TranslationState state() const;
    TranslationMode mode() const;

private:
    TranslationState update_line(double dt);
    TranslationState update_random(double dt);
    void sample_target_velocity();

    static Eigen::Vector3d clamp_norm(
        const Eigen::Vector3d& value,
        double max_norm);

    TranslationMode translation_mode_{TranslationMode::LINE};
    TranslationState state_;

    Eigen::Vector3d initial_position_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d initial_velocity_{Eigen::Vector3d::Zero()};

    Eigen::Vector3d line_start_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d line_end_{Eigen::Vector3d::UnitX()};
    double line_one_way_time_{2.0};
    double elapsed_time_{0.0};

    Eigen::Vector3d velocity_min_{Eigen::Vector3d::Constant(-1.0)};
    Eigen::Vector3d velocity_max_{Eigen::Vector3d::Constant(1.0)};
    Eigen::Vector3d target_velocity_{Eigen::Vector3d::Zero()};
    double max_acceleration_{2.0};
    double max_jerk_{10.0};
    double response_time_{0.5};
    double hold_time_min_{0.5};
    double hold_time_max_{2.0};
    double target_hold_remaining_{0.0};
    double stop_probability_{0.0};
    double reverse_probability_{0.0};
    double integration_step_{0.01};
    std::uint32_t random_seed_{42};
    std::mt19937 random_engine_;

    bool has_last_update_time_{false};
    std::chrono::steady_clock::time_point last_update_time_;
};

} // namespace motion_generator

#endif // MOTION_GENERATOR_TRANSLATE_HPP_
