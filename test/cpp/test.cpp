#include "kalman/estimator.hpp"

#include <exception>
#include <iostream>

int
main ()
{
  int failures = 0;
  const auto run = [&failures] (const char *name, void (*test) ())
    {
      try
        {
          test ();
          std::cout << "[ OK ] " << name << "\n";
        }
      catch (const std::exception &e)
        {
          ++failures;
          std::cout << "[FAIL] " << name << ": " << e.what () << "\n";
        }
    };

  run ("estimator_gps_only", estimator_gps_only_test);
  run ("estimator_course", estimator_course_test);
  run ("estimator_imu", estimator_imu_test);
  run ("estimator_altitude", estimator_altitude_test);
  run ("estimator_vertical_imu", estimator_vertical_imu_test);
  return failures == 0 ? 0 : 1;
}
