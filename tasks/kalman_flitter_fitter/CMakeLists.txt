cmake_minimum_required(VERSION 3.8)

add_library(kalman_flitter_fitter OBJECT
            base_kf/kalman.cpp
            ekf/ekf.cpp
            ukf/ukf.cpp
)

target_include_directories(
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(kalman_flitter_fitter
                      PUBLIC
                      )