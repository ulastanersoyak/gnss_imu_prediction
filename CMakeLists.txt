cmake_minimum_required(VERSION 3.22)

project(gnss_imu_prediction VERSION 0.1.0 LANGUAGES CXX)

# The submodule by default; -DLOCALIZATION_CORE_DIR=../localization_core uses
# a side by side checkout while working on both (see README).
set(LOCALIZATION_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/localization_core
    CACHE PATH "localization_core source directory")
set(localization_core_dir ${LOCALIZATION_CORE_DIR})
if(NOT EXISTS ${localization_core_dir}/CMakeLists.txt)
  message(FATAL_ERROR "external/localization_core is empty, run: git submodule update --init")
endif()
include(${localization_core_dir}/cmake/common.cmake)
add_subdirectory(${localization_core_dir} ${CMAKE_BINARY_DIR}/localization_core)

find_package(Threads REQUIRED)

add_subdirectory(src)

enable_testing()
add_subdirectory(test/cpp)

# Simulation program: a ROS 2 node for Cosys-AirSim, built when ROS 2 is
# sourced (sim/run.sh builds it in the ros2 distrobox).
find_package(rclcpp QUIET)
if(rclcpp_FOUND)
  add_subdirectory(sim)
endif()
