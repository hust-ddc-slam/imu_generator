cmake_minimum_required(VERSION 3.0)
project(imu_generator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(
    include
    /usr/local/include/eigen3          # Eigen
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)


add_executable(gen_imu
    src/main.cpp
    src/BsplineSE3.cpp
    src/print.cpp
    src/myInterface.cpp
    src/Simulator.cpp
)
target_link_libraries(gen_imu)

add_executable(vicon_convertor
    src/vicon_data_convertor/convertor.cpp
)
target_link_libraries(vicon_convertor)