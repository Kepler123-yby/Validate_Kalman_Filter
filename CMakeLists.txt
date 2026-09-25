cmake_minimum_required(VERSION 3.8)

project(validate_kalman_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(tools)
add_subdirectory(tasks/motion_generator)
