cmake_minimum_required(VERSION 3.0.2)
project(gen_trajectory)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


SET(CMAKE_BUILD_TYPE Release)

include_directories()

add_executable(gen_trajectory 
    generate_traj.cpp
)
