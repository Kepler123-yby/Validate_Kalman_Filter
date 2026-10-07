/**
 * @file generator_test.cpp
 * @brief 运动生成器可视化示例：把真值状态与装甲板观测实时发送给 PlotJuggler。
 *
 * 运行后按 Ctrl-C 退出。用法：
 * @code
 * ./generator_test --config-path=configs/motion_generator_test.yaml
 * @endcode
 */

#include "tasks/motion_generator/generator/generator.hpp"
#include "tools/exiter/exiter.hpp"
#include "tools/plotjuggler/plotjuggler.hpp"
#include "tools/robot_state/robot_state.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

/// 命令行参数说明。
const std::string keys =
    "{help h usage ? |                   | 输出命令行参数说明 }"
    "{config-path c  | configs/motion_generator_test.yaml | yaml配置文件的路径}";

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

    motion_generator::Generator generator(config);
    tools::Exiter exiter;
    tools::PlotJuggler plotter;
    nlohmann::json data;

    while (!exiter.exit())
    {
        const auto snapshot = generator.generate();
        const auto& state = snapshot.truth_state;
        const auto& observation = snapshot.observation;

        using namespace tools::robot_state;

        data["center_x"] = state[kCenterX];
        data["vx"] = state[kVelocityX];
        data["center_y"] = state[kCenterY];
        data["vy"] = state[kVelocityY];
        data["center_z"] = state[kCenterZ];
        data["vz"] = state[kVelocityZ];
        data["a"] = state[kYaw];
        data["w"] = state[kYawRate];
        data["forward_radius"] = state[kForwardRadius];
        data["beside_radius"] = state[kBesideRadius];
        data["beside_height_diff"] = state[kHeightDifference];

        data["armor_yaw"] = observation[kBearing];
        data["armor_pitch"] = observation[kPitch];
        data["armor_distance"] = observation[kDistance];
        data["armor_angle"] = observation[kArmorAngle];

        plotter.plot(data);
    }

    return 0;
}
