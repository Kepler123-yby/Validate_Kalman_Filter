#ifndef _TRANSLATE_HPP_
#define _TRANSLATE_HPP_

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <random>

namespace motion_generator
{

enum class TranslationMode
{
    LINE,
    RANDOM
};

struct TranslationState
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d speed{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
};

class TranslationGenerator
{
public:
    explicit TranslationGenerator(const YAML::Node& config);
    TranslationGenerator() = default;
    ~TranslationGenerator() = default;

    TranslationState update(double dt);
    TranslationState update(const std::chrono::steady_clock::time_point& time);
    void reset();

    TranslationState state() const;
    TranslationMode mode() const;

private:
    TranslationState update_line(double dt);
    TranslationState update_random(double dt);
    void set_random_target();

    static Eigen::Vector3d limit_norm(
        const Eigen::Vector3d& value,
        double limit);

    TranslationMode translation_mode_{TranslationMode::LINE};
    TranslationState state_;

    Eigen::Vector3d initial_position_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d initial_speed_{Eigen::Vector3d::Zero()};

    Eigen::Vector3d line_start_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d line_end_{Eigen::Vector3d::UnitX()};
    double line_time_{2.0};
    double elapsed_time_{0.0};

    Eigen::Vector3d speed_min_{Eigen::Vector3d::Constant(-1.0)};
    Eigen::Vector3d speed_max_{Eigen::Vector3d::Constant(1.0)};
    Eigen::Vector3d target_speed_{Eigen::Vector3d::Zero()};
    double max_acceleration_{2.0};
    double max_jerk_{10.0};
    double response_time_{0.5};
    double hold_time_min_{0.5};
    double hold_time_max_{2.0};
    double hold_time_{0.0};
    double stop_probability_{0.0};
    double reverse_probability_{0.0};
    double integration_step_{0.01};
    unsigned int random_seed_{42};
    std::mt19937 random_engine_;

    bool has_last_time_{false};
    std::chrono::steady_clock::time_point last_time_;
};

} // namespace motion_generator

#endif // _TRANSLATE_HPP_
