# The simulation program: a ROS 2 node for Cosys-AirSim (localization_core
# airsim/). Built where ROS 2 Jazzy is sourced (the ros2
# distrobox; sim/run.sh does that).
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
find_package(GeographicLib REQUIRED)

add_executable(positioning_sim src/positioning_sim.cpp)
target_include_directories(positioning_sim PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(positioning_sim PRIVATE kalman utm xbus rmc
                      time_and_location localization_core_sim
                      rclcpp::rclcpp tf2_ros::tf2_ros tf2_eigen::tf2_eigen
                      Eigen3::Eigen ${GeographicLib_LIBRARIES}
                      ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS}
                      ${geometry_msgs_TARGETS} ${visualization_msgs_TARGETS})
