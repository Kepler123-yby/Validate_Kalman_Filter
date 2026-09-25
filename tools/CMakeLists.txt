cmake_minimum_required(VERSION 3.8)

find_package(Eigen3 REQUIRED)

add_library(tools STATIC
    math_tools/math_tools.cpp
    sine_fuction/sine_fuction.cpp
    plotjuggler/plotjuggler.cpp
)

target_include_directories(tools
    PUBLIC
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(tools
    PUBLIC
        Eigen3::Eigen
)
