#include "tasks/kalman_flitter_fitter/kalman/kalman.hpp"
#include "tasks/motion_generator/generator/generator.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/plotjuggler/plotjuggler.hpp"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <random>

const std::string keys =
"{ help h usage ? | | 输出命令行参数说明 }"
"{ config-path c | configs/motion_generator_test.yaml | yaml配置文件 }"
"{ duration d | 60 | 测试时间，单位：秒 }";

Eigen::VectorXd read_vector(const YAML::Node& config)
{
    const auto values = config.as<std::vector<double>>();
    return Eigen::Map<const Eigen::VectorXd>(values.data(), values.size());
}

Eigen::VectorXd add_measurement_noise(
    const Eigen::VectorXd& observation, const Eigen::VectorXd& R,
    std::mt19937& random_engine)
{
    Eigen::VectorXd noisy_observation = observation;
    for (int i = 0; i < observation.size(); ++i)
    {
        noisy_observation[i] += std::normal_distribution<double>(
            0, std::sqrt(R[i]))(random_engine);
    }
    noisy_observation[0] = tools::limit_euler(noisy_observation[0]);
    noisy_observation[3] = tools::limit_euler(noisy_observation[3]);
    return noisy_observation;
}

int main(int argc, char* argv[])
{
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help"))
    {
        cli.printMessage();
        return 0;
    }

    const auto config = YAML::LoadFile(cli.get<std::string>("config-path"));
    const auto ekf_config = config["kalman_flitter"]["ekf"];
    const double duration = cli.get<double>("duration");
    const double predict_time = config["kalman_flitter"]["predict_time"].as<double>();
    const auto measurement_R = read_vector(ekf_config["measurement_R"]);
    const auto predict_duration = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(predict_time));

    tools::PlotJuggler plotter;
    motion_generator::Generator generator(config);
    kalman_flitter::Kalman kalman(config);
    std::mt19937 random_engine(42);
    auto states = generator.wait_for_next(0);
    const auto end_time = states.timestamp + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(std::chrono::duration<double>(duration));
    Eigen::VectorXd state_error = Eigen::VectorXd::Zero(11);
    int prediction_samples = 0;

    auto update = [&]()
    {
        const auto noisy_observation = add_measurement_noise(
            states.observation, measurement_R, random_engine);
        kalman.update(
            noisy_observation,
            states.armor_id, states.timestamp);
    };
    auto evaluate_prediction = [&]()
    {
        const auto x = kalman.get_x();
        Eigen::VectorXd error = x - states.state;
        error[6] = tools::limit_euler(error[6]);
        state_error += error.cwiseProduct(error);
        kalman.evaluate_nees(states.state);
        ++prediction_samples;

        plotter.plot({
            {"center_x", states.state[0]}, {"center_y", states.state[2]},
            {"center_z", states.state[4]}, {"predicted_center_x", x[0]},
            {"predicted_center_y", x[2]}, {"predicted_center_z", x[4]},
            {"vx", states.state[1]}, {"vy", states.state[3]}, {"vz", states.state[5]},
            {"predicted_vx", x[1]}, {"predicted_vy", x[3]}, {"predicted_vz", x[5]},
            {"a", states.state[6]}, {"predicted_a", x[6]},
            {"w", states.state[7]}, {"predicted_w", x[7]},
            {"forward_radius", states.state[8]}, {"predicted_forward_radius", x[8]},
            {"beside_radius", states.state[9]}, {"predicted_beside_radius", x[9]},
            {"beside_height_diff", states.state[10]}, {"predicted_beside_height_diff", x[10]},
            {"armor_id", states.armor_id},
            {"nis", kalman.get_nis()}, {"nees", kalman.get_nees()}});
    };

    update();
    while (true)
    {
        const auto prediction_time = states.timestamp + predict_duration;
        do
        {
            states = generator.wait_for_next(states.sequence);
        } while (states.timestamp < prediction_time);

        if (states.timestamp >= end_time)
        {
            break;
        }
        evaluate_prediction();
        update();
    }

    const char* state_names[] = {
        "center_x", "vx", "center_y", "vy", "center_z", "vz",
        "a", "w", "forward_radius", "beside_radius", "beside_height_diff"};
    for (int i = 0; i < 11; ++i)
    {
        std::cout << state_names[i] << " RMSE: "
                  << std::sqrt(state_error[i] / prediction_samples) << '\n';
    }
    std::cout << "mean NIS: " << kalman.get_nis_statistics().mean << '\n'
              << "mean NEES: " << kalman.get_nees_statistics().mean << '\n';
    return 0;
}
