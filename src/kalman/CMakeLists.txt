add_library(kalman STATIC position_estimator.cpp)
target_include_directories(kalman PUBLIC ${PROJECT_SOURCE_DIR}/includes)
target_link_libraries(kalman PUBLIC localization_core_headers utm)
