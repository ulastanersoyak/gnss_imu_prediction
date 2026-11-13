add_executable(tests test.cpp)
target_include_directories(tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(tests PRIVATE kalman localization_core_test_support)
add_test(NAME tests COMMAND tests)
