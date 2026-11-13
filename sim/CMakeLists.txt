add_executable(positioning_sim src/positioning_sim.cpp)
target_link_libraries(positioning_sim PRIVATE kalman utm xbus rmc
                      time_and_location gnss_imu_fusion_sim
                      gz-transport13::core gz-msgs10::core)
