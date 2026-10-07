cmake_minimum_required(VERSION 3.8)

find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

# 无状态工具集合：数学函数、正弦运动模型、PlotJuggler 客户端、退出标志。
add_library(tools STATIC
    math_tools/math_tools.cpp
    sine_function/sine_function.cpp
    plotjuggler/plotjuggler.cpp
    exiter/exiter.cpp
)

target_include_directories(tools
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(tools
    PUBLIC
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)
