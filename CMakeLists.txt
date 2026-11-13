cmake_minimum_required(VERSION 3.22)

project(gnss_imu_prediction VERSION 0.1.0 LANGUAGES CXX)

set(gnss_imu_fusion_dir ${CMAKE_CURRENT_SOURCE_DIR}/external/gnss-imu-fusion)
if(NOT EXISTS ${gnss_imu_fusion_dir}/CMakeLists.txt)
  message(FATAL_ERROR "external/gnss-imu-fusion is empty, run: git submodule update --init")
endif()
include(${gnss_imu_fusion_dir}/cmake/common.cmake)
add_subdirectory(${gnss_imu_fusion_dir})

find_package(Threads REQUIRED)

add_subdirectory(src)

enable_testing()
add_subdirectory(test/cpp)

# Gazebo simulation bridge, see sim/run.sh.
find_package(gz-transport13 QUIET)
find_package(gz-msgs10 QUIET)
if(gz-transport13_FOUND AND gz-msgs10_FOUND)
  add_subdirectory(sim)
endif()
