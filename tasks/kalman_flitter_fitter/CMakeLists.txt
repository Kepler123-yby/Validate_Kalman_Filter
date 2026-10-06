cmake_minimum_required(VERSION 3.8)

find_package(Eigen3 REQUIRED)

add_library(kalman_flitter_fitter OBJECT
            kalman/kalman.cpp
            ekf/ekf.cpp
            ukf/ukf.cpp
)

target_include_directories(kalman_flitter_fitter
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(kalman_flitter_fitter
                      PUBLIC
                        Eigen3::Eigen
                        tools
                      )
