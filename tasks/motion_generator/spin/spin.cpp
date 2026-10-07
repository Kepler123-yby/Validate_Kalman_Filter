/**
 * @file spin.cpp
 * @brief @ref motion_generator::SpinGenerator 的实现。
 */

#include "spin.hpp"

#include "tools/math_tools/math_tools.hpp"

#include <vector>

namespace motion_generator
{

SpinGenerator::SpinGenerator(const YAML::Node& config)
{
    const int number_of_sine_functions =
        config["number_of_sine_functions"].as<int>(1);

    // 未配置或为空的参数列表按零处理，长度以 number_of_sine_functions 为准。
    auto read_list = [&config, number_of_sine_functions](const char* name)
    {
        auto list = config[name].as<std::vector<double>>(std::vector<double>{});
        if (list.empty())
        {
            list.resize(number_of_sine_functions, 0.0);
        }
        return list;
    };

    const auto amplitudes = read_list("A_lists");
    const auto frequencies = read_list("f_lists");
    const auto phases = read_list("phi_lists");
    const auto offsets = read_list("x_lists");

    sine_functions_.reserve(number_of_sine_functions);
    for (int i = 0; i < number_of_sine_functions; ++i)
    {
        sine_functions_.emplace_back(
            amplitudes[i], frequencies[i], phases[i], offsets[i]);
    }

    const auto physical_dimensions = config["physical_dimensions"].as<
        std::vector<double>>(std::vector<double>{0.27, 0.25, 0.5});
    forward_radius_ = physical_dimensions[0];
    beside_radius_ = physical_dimensions[1];
    height_difference_ = physical_dimensions[2];

    start_time_ = std::chrono::steady_clock::now();
}

SpinState SpinGenerator::state_at(
    const std::chrono::steady_clock::time_point& time) const
{
    const double elapsed_seconds =
        std::chrono::duration<double>(time - start_time_).count();
    return evaluate(elapsed_seconds);
}

SpinState SpinGenerator::evaluate(const double elapsed_seconds) const
{
    SpinState state;
    state.forward_radius = forward_radius_;
    state.beside_radius = beside_radius_;
    state.height_difference = height_difference_;

    for (const auto& sine_function : sine_functions_)
    {
        state.yaw += sine_function.angle(elapsed_seconds);
        state.angular_velocity += sine_function.angular_velocity(elapsed_seconds);
        state.angular_acceleration +=
            sine_function.angular_acceleration(elapsed_seconds);
    }

    state.yaw = tools::normalize_angle(state.yaw);
    return state;
}

} // namespace motion_generator
