cmake_minimum_required(VERSION 3.10)
project(planar_robot_2d)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

add_executable(planar_robot
    src/main.cc
    src/robot_model.cc
)