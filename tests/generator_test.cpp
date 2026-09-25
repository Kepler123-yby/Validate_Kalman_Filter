#include "tasks/motion_generator/generator/generator.hpp"
#include <opencv2/opencv.hpp>

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

    
}
