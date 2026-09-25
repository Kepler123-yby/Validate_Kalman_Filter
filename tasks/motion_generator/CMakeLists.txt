cmake_minimum_required(VERSION 3.8)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

add_library(motion_generator OBJECT
    generator/generator.cpp
    robot/robot.cpp
    spin/spin.cpp
    translate/translate.cpp
)

target_include_directories(motion_generator
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(motion_generator
    PUBLIC
        tools
        Eigen3::Eigen
        yaml-cpp
        Threads::Threads
)
