project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(SRC_FILES
        src/FusionEKF.cpp
        src/kalman_filter.cpp
        src/main.cpp
        src/tools.cpp)
        
add_executable(ExtendedKF ${SRC_FILES})