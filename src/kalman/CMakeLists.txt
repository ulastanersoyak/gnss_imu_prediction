add_library(kalman STATIC position_estimator.cpp)
target_include_directories(kalman PUBLIC ${PROJECT_SOURCE_DIR}/includes)
target_link_libraries(kalman PUBLIC gnss_imu_fusion_headers utm xbus)
