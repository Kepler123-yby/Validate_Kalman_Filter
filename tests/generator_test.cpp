#include "tasks/motion_generator/generator/generator.hpp"
#include "tools/exiter/exiter.hpp"
#include "tools/plotjuggler/plotjuggler.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

const std::string keys = 
"{help h usage ? |                   | 输出命令行参数说明 }"
"{config-path c  | configs/motion_generator_test.yaml | yaml配置文件的路径}";

int main(int argc, char* argv[])
{
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help"))
    {
        cli.printMessage();
        return 0;
    }

    auto config_path = cli.get<std::string>("config-path");

    YAML::Node config = YAML::LoadFile(config_path);

    motion_generator::Generator generator_{config};
    tools::Exiter exiter;
    tools::PlotJuggler plotter;
    nlohmann::json data;
    
    
    while(!exiter.exit())
    {
        auto motion_states = generator_.generate();

        {
            data["center_x"] = motion_states.state[0];
            data["vx"] = motion_states.state[1];
            data["center_y"] = motion_states.state[2];
            data["vy"] = motion_states.state[3];
            data["center_z"] = motion_states.state[4];
            data["vz"] = motion_states.state[5];
            data["a"] = motion_states.state[6];
            data["w"] = motion_states.state[7];
            data["forward_radius"] = motion_states.state[8];
            data["beside_radius"] = motion_states.state[9];
            data["beside_height_diff"]  = motion_states.state[10];
            data["armor_yaw"] = motion_states.observation[0];
            data["armor_pitch"] = motion_states.observation[1];
            data["armor_distance"] = motion_states.observation[2];
            data["armor_angle"] = motion_states.observation[3];
        }

        plotter.plot(data);
    }

    return 0;
}
