cmake_minimum_required(VERSION 3.8)

project(validate_kalman_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

add_subdirectory(tools)
add_subdirectory(tasks/motion_generator)
add_subdirectory(tasks/kalman_filter_fitter)

# 测试/示例程序
add_executable(generator_test tests/generator_test.cpp)
target_link_libraries(generator_test
    PRIVATE
        ${OpenCV_LIBS}
        nlohmann_json::nlohmann_json
        yaml-cpp
        tools
        motion_generator)

add_executable(ekf_test tests/ekf_test.cpp)
target_include_directories(ekf_test PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(ekf_test
    PRIVATE
        ${OpenCV_LIBS}
        nlohmann_json::nlohmann_json
        yaml-cpp
        tools
        motion_generator
        kalman_filter_fitter)

# 可选：生成 Doxygen API 文档（需已安装 doxygen）。
find_package(Doxygen)
if(DOXYGEN_FOUND)
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "使用 Doxygen 生成 API 文档")
endif()
