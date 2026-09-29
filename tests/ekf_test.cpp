#include "tasks/motion_generator/generator/generator.hpp"
#include "tasks/kalman_flitter_fitter/ekf/ekf.hpp"
#include "tools/exiter/exiter.hpp"
#include "tools/plotjuggler/plotjuggler.hpp"

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

std::string keys = 
"{ help h usage ? |                                    | 输出命令行参数说明 }"
"{ config-path c  | configs/motion_generator_test.yaml | yaml配置文件 }";

int main(int argc, char* argv[])
{
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help"))
    {
        cli.printMessage();
        return 0;
    }

    auto config_path = cli.get<std::string>("config-path");
    auto config = YAML::LoadFile(config_path);


    tools::Exiter exiter;
    tools::PlotJuggler plotter;
    motion_generator::Generator generator{config};
    kalman_flitter::EKF ekf;

    // ekf相关参数
    int generator_times = 0;
    while (exiter.exit())
    {
        auto states = generator.generate();
        if (generator_times == 0)
        {
            ekf = kalman_flitter::EKF();
        } else {

        }
    }
    
    
    return 0;
}
