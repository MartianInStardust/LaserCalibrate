cmake_minimum_required(VERSION 3.19)
project(LaserCalibrate)

set(CMAKE_CXX_STANDARD 14)

add_executable(LaserCalibrate main.cpp cmake-build-debug/src/CloudPointExaction.cpp cmake-build-debug/include/CloudPointExaction.h)