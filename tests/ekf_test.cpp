/**
 * @file ekf_test.cpp
 * @brief EKF 拟合器验证程序：对仿真真值叠加观测噪声，统计状态误差与一致性。
 *
 * 程序按 @c predict_time 的节拍把噪声观测送入 @ref kalman_filter::Kalman，
 * 与真值对比计算各状态 RMSE，并汇总 NIS / NEES 的均值；同时把真值、估计与
 * 一致性指标发送给 PlotJuggler 便于观察收敛过程。
 *
 * @code
 * ./ekf_test --config-path=configs/motion_generator_test.yaml --duration=60
 * @endcode
 */

#include "tasks/kalman_filter_fitter/kalman/kalman.hpp"
#include "tasks/motion_generator/generator/generator.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/plotjuggler/plotjuggler.hpp"
#include "tools/robot_state/robot_state.hpp"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

namespace
{

const std::string keys =
    "{ help h usage ? | | 输出命令行参数说明 }"
    "{ config-path c | configs/motion_generator_test.yaml | yaml配置文件 }"
    "{ duration d | 60 | 测试时间，单位：秒 }";

/**
 * @brief 将 YAML 序列读取为动态向量。
 */
Eigen::VectorXd read_vector(const YAML::Node& node)
{
    const auto values = node.as<std::vector<double>>();
    return Eigen::Map<const Eigen::VectorXd>(values.data(), values.size());
}

/**
 * @brief 给观测叠加高斯噪声，并对角度分量做归一化。
 *
 * @param observation 无噪声观测。
 * @param variance    各分量的噪声方差。
 * @param engine      随机数引擎。
 * @return 叠加噪声后的观测。
 */
Eigen::VectorXd add_measurement_noise(
    const Eigen::VectorXd& observation, const Eigen::VectorXd& variance,
    std::mt19937& engine)
{
    using namespace tools::robot_state;

    Eigen::VectorXd noisy = observation;
    for (int i = 0; i < observation.size(); ++i)
    {
        noisy[i] += std::normal_distribution<double>(0.0, std::sqrt(variance[i]))(engine);
    }
    noisy[kBearing] = tools::normalize_angle(noisy[kBearing]);
    noisy[kArmorAngle] = tools::normalize_angle(noisy[kArmorAngle]);
    return noisy;
}

/// 各状态分量的名称，用于打印 RMSE。
const char* const kStateNames[] = {
    "center_x", "vx", "center_y", "vy", "center_z", "vz",
    "a", "w", "forward_radius", "beside_radius", "beside_height_diff"};

} // namespace

int main(int argc, char* argv[])
{
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help"))
    {
        cli.printMessage();
        return 0;
    }

    const auto config = YAML::LoadFile(cli.get<std::string>("config-path"));
    const auto filter_config = config["kalman_filter"];
    const auto ekf_config = filter_config["ekf"];
    const double duration = cli.get<double>("duration");
    const double predict_time = filter_config["predict_time"].as<double>();
    const auto measurement_variance = read_vector(ekf_config["measurement_R"]);

    const auto predict_duration = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(predict_time));

    tools::PlotJuggler plotter;
    motion_generator::Generator generator(config);
    kalman_filter::Kalman kalman(config);
    std::mt19937 random_engine(42);

    using namespace tools::robot_state;

    auto snapshot = generator.wait_for_next(0);
    const auto end_time = snapshot.timestamp + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(duration));

    Eigen::VectorXd squared_error = Eigen::VectorXd::Zero(kSize);
    int samples = 0;

    const auto update = [&]()
    {
        const auto noisy_observation = add_measurement_noise(
            snapshot.observation, measurement_variance, random_engine);
        kalman.update(noisy_observation, snapshot.armor_id, snapshot.timestamp);
    };

    const auto evaluate_prediction = [&]()
    {
        const Eigen::VectorXd estimate = kalman.state();
        Eigen::VectorXd error = estimate - snapshot.truth_state;
        error[kYaw] = tools::normalize_angle(error[kYaw]);
        squared_error += error.cwiseProduct(error);
        kalman.evaluate_nees(snapshot.truth_state);
        ++samples;

        const auto& truth = snapshot.truth_state;
        plotter.plot({
            {"center_x", truth[kCenterX]}, {"center_y", truth[kCenterY]},
            {"center_z", truth[kCenterZ]}, {"predicted_center_x", estimate[kCenterX]},
            {"predicted_center_y", estimate[kCenterY]},
            {"predicted_center_z", estimate[kCenterZ]},
            {"vx", truth[kVelocityX]}, {"vy", truth[kVelocityY]},
            {"vz", truth[kVelocityZ]}, {"predicted_vx", estimate[kVelocityX]},
            {"predicted_vy", estimate[kVelocityY]},
            {"predicted_vz", estimate[kVelocityZ]},
            {"a", truth[kYaw]}, {"predicted_a", estimate[kYaw]},
            {"w", truth[kYawRate]}, {"predicted_w", estimate[kYawRate]},
            {"forward_radius", truth[kForwardRadius]},
            {"predicted_forward_radius", estimate[kForwardRadius]},
            {"beside_radius", truth[kBesideRadius]},
            {"predicted_beside_radius", estimate[kBesideRadius]},
            {"beside_height_diff", truth[kHeightDifference]},
            {"predicted_beside_height_diff", estimate[kHeightDifference]},
            {"armor_id", snapshot.armor_id},
            {"nis", kalman.nis()}, {"nees", kalman.nees()}});
    };

    update();
    while (true)
    {
        const auto prediction_time = snapshot.timestamp + predict_duration;
        do
        {
            snapshot = generator.wait_for_next(snapshot.sequence);
        } while (snapshot.timestamp < prediction_time);

        if (snapshot.timestamp >= end_time)
        {
            break;
        }

        evaluate_prediction();
        update();
    }

    for (int i = 0; i < kSize; ++i)
    {
        std::cout << kStateNames[i] << " RMSE: "
                  << std::sqrt(squared_error[i] / samples) << '\n';
    }
    std::cout << "mean NIS: " << kalman.nis_statistics().mean << '\n'
              << "mean NEES: " << kalman.nees_statistics().mean << '\n';
    return 0;
}
