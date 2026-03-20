cmake_minimum_required(VERSION 3.10)
project(planar_robot_2d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(planar_robot
    src/main.cc
    src/robot_model.cc
)

target_include_directories(planar_robot PRIVATE include)
target_link_libraries(planar_robot PRIVATE Eigen3::Eigen)