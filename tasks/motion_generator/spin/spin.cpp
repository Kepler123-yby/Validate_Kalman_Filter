#include "spin.hpp"

#include <string>
#include <stdexcept>

namespace motion_generator
{

SpinGenerator::SpinGenerator(const YAML::Node& config)
{
    number_of_sine_functions_ = config["number_of_sine_functions"]
        ? config["number_of_sine_functions"].as<int>()
        : 1;

    auto read_list = [this, &config](const char* name)
    {
        if (!config[name] || config[name].size() == 0)
        {
            return std::vector<double>(number_of_sine_functions_, 0.0);
        }

        auto list = config[name].as<std::vector<double>>();
        if (list.size() != static_cast<std::size_t>(number_of_sine_functions_))
        {
            throw std::invalid_argument(
                std::string(name) + " size does not match number_of_sine_functions");
        }
        return list;
    };

    A_lists_ = read_list("A_lists");
    f_lists_ = read_list("f_lists");
    phi_lists_ = read_list("phi_lists");
    x_lists_ = read_list("x_lists");

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
    for (const auto& sine_function : sine_functions_)
    {
        state.yaw += sine_function.integral(elapsed_seconds);
        state.speed += sine_function.evaluate(elapsed_seconds);
        state.acceleration += sine_function.derivative(elapsed_seconds);
    }
    return state;
}

} // namespace motion_generator
