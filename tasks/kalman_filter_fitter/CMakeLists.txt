cmake_minimum_required(VERSION 3.8)

find_package(Eigen3 REQUIRED)

# EKF 拟合器：通用 EKF + 装甲板物理模型 + 时序调度。
add_library(kalman_filter_fitter OBJECT
    ekf/ekf.cpp
    model/armor_model.cpp
    kalman/kalman.cpp
)

target_include_directories(kalman_filter_fitter
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(kalman_filter_fitter
    PUBLIC
        Eigen3::Eigen
        tools
)
