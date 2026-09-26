cmake_minimum_required(VERSION 3.8)

project(validate_kalman_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

add_subdirectory(tools)
add_subdirectory(tasks/motion_generator)

# test

add_executable(generator_test tests/generator_test.cpp)
target_link_libraries(generator_test ${OpenCV_LIBS} nlohmann_json::nlohmann_json yaml-cpp tools motion_generator)

enable_testing()
add_executable(angle_test tests/angle_test.cpp)
target_link_libraries(angle_test PRIVATE motion_generator)
add_test(NAME angle_test COMMAND angle_test)
