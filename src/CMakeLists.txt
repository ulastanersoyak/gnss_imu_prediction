add_subdirectory(kalman)

# The device program needs the serial port readers, which need Boost.
if(TARGET gps AND TARGET mti_630)
  add_executable(main main.cpp)
  target_link_libraries(main PRIVATE gps mti_630 kalman Threads::Threads)
else()
  message(WARNING "Boost not found, only building the libraries and tests")
endif()
