# GoogleTest: each TEST is its own ctest test (gtest_discover_tests).
find_package(GTest REQUIRED)
include(GoogleTest)
add_executable(tests estimator_test.cpp)
target_link_libraries(tests PRIVATE kalman GTest::gtest_main)
gtest_discover_tests(tests)
