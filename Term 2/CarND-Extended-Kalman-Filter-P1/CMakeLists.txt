SET(CMAKE_C_COMPILER C:/MinGW/bin/gcc)
SET(CMAKE_CXX_COMPILER C:/MinGW/bin/g++)
project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    src/FusionEKF.cpp
    src/kalman_filter.cpp
    src/main.cpp
    src/tools.cpp)

add_executable(ExtendedKF ${sources})
