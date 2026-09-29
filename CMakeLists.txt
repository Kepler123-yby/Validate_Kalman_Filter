cmake_minimum_required(VERSION 3.8)

project(validate_kalman_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

add_subdirectory(tools)
add_subdirectory(tasks/motion_generator)
add_subdirectory(tasks/kalman_flitter_fitter)

# test

add_executable(generator_test tests/generator_test.cpp)
target_link_libraries(generator_test ${OpenCV_LIBS} nlohmann_json::nlohmann_json yaml-cpp tools motion_generator)
