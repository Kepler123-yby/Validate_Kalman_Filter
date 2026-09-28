#include "spin.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace motion_generator
{

SpinGenerator::SpinGenerator(const YAML::Node& config)
{
    number_of_sine_functions_ = config["number_of_sine_functions"].as<int>(1);

    // 未配置或为空的参数列表按零处理，其余列表长度由配置保证。
    auto read_list = [this, &config](const char* name)
    {
        auto list = config[name].as<std::vector<double>>(std::vector<double>{});
        if (list.empty())
        {
            list.resize(number_of_sine_functions_, 0.0);
        }
        return list;
    };

    A_lists_ = read_list("A_lists");
    f_lists_ = read_list("f_lists");
    phi_lists_ = read_list("phi_lists");
    x_lists_ = read_list("x_lists");

    auto physical_dimensions = config["physical_dimensions"].as<std::vector<double>>(
        std::vector<double>{27, 25, 5});
    forword_radius_ = physical_dimensions[0];
    beside_radius_ = physical_dimensions[1];
    height_diff_ = physical_dimensions[2];

    sine_functions_.reserve(number_of_sine_functions_);
    for (int i = 0; i < number_of_sine_functions_; ++i)
    {
        sine_functions_.emplace_back(
            A_lists_[i], f_lists_[i], phi_lists_[i], x_lists_[i]);
    }

    start_time_ = std::chrono::steady_clock::now();
}

SpinState SpinGenerator::get_states(
    const std::chrono::steady_clock::time_point& time) const
{
    const double elapsed_seconds = std::chrono::duration<double>(
        time - start_time_).count();
    return evaluate(elapsed_seconds);
}

SpinState SpinGenerator::evaluate(double elapsed_seconds) const
{
    SpinState state;
    state.forword_radius = forword_radius_;
    state.beside_radius = beside_radius_;
    state.height_diff = height_diff_;

    for (const auto& sine_function : sine_functions_)
    {
        state.yaw += sine_function.integral(elapsed_seconds);
        state.speed += sine_function.evaluate(elapsed_seconds);
        state.acceleration += sine_function.derivative(elapsed_seconds);
    }
    state.yaw = tools::limit_euler(state.yaw);
    return state;
}

} // namespace motion_generator
