#include "kalman/position_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
constexpr double DEG_TO_RAD = std::numbers::pi / 180;

using vector2 = Eigen::Vector2d;
using vector3 = Eigen::Vector3d;
using matrix2 = Eigen::Matrix2d;
using matrix3 = Eigen::Matrix3d;
using vector6 = kalman_filter<6>::vector;
using matrix6 = kalman_filter<6>::matrix;

// Position measurement with altitude.
const Eigen::Matrix<double, 3, 6> H_position{ { 1, 0, 0, 0, 0, 0 },
                                              { 0, 1, 0, 0, 0, 0 },
                                              { 0, 0, 1, 0, 0, 0 } };
// Position measurement without altitude.
const Eigen::Matrix<double, 2, 6> H_horizontal{ { 1, 0, 0, 0, 0, 0 },
                                                { 0, 1, 0, 0, 0, 0 } };
// Horizontal velocity measurement, all that GPS speed and course give.
const Eigen::Matrix<double, 2, 6> H_velocity{ { 0, 0, 0, 1, 0, 0 },
                                              { 0, 0, 0, 0, 1, 0 } };

// Until a fix with altitude arrives the altitude state is effectively unknown.
constexpr double unknown_altitude_sigma_m = 1e4;

matrix2
diagonal_noise (double sigma)
{
  return vector2::Constant (sigma * sigma).asDiagonal ();
}

// Rotates a vector from true east/north to UTM grid east/north. The grid
// convergence is the clockwise angle from true north to grid north.
vector2
true_to_grid (double east, double north, double convergence)
{
  return Eigen::Rotation2Dd (convergence) * vector2 (east, north);
}

double
horizontal_sigma (const gps_fix &fix, const estimator_config &config)
{
  return fix.hdop
             ? std::max (config.gps_min_sigma_m, *fix.hdop * config.gps_uere_m)
             : config.gps_default_sigma_m;
}

std::optional<vector2>
velocity_from_fix (const gps_fix &fix, double convergence)
{
  if (!fix.course_deg)
    {
      return std::nullopt;
    }
  const double course = *fix.course_deg * DEG_TO_RAD;
  return true_to_grid (fix.speed_mps * std::sin (course),
                       fix.speed_mps * std::cos (course), convergence);
}
} // namespace

position_estimator::position_estimator (estimator_config config)
    : config_ (config)
{
}

void
position_estimator::initialize (const gps_fix &fix)
{
  const double sigma_pos = horizontal_sigma (fix, config_);
  const double sigma_alt = fix.altitude_m
                               ? sigma_pos * config_.gps_vertical_factor
                               : unknown_altitude_sigma_m;

  convergence_ = fix.position.convergence ();
  vector6 x0 = vector6::Zero ();
  x0.head<3> () << fix.position.easting (), fix.position.northing (),
      fix.altitude_m.value_or (0);
  double sigma_vel = 5.0; // unknown, allow anything a vehicle could do
  if (fix.speed_mps < config_.stationary_speed_mps)
    {
      sigma_vel = 1.0;
    }
  else if (const auto v = velocity_from_fix (fix, convergence_))
    {
      x0.segment<2> (3) = *v;
      sigma_vel = config_.gps_velocity_sigma_mps * 2;
    }

  // A slow horizontal speed says nothing about the climb rate.
  constexpr double sigma_vel_up = 5.0;
  vector6 variances;
  variances << sigma_pos * sigma_pos, sigma_pos * sigma_pos,
      sigma_alt * sigma_alt, sigma_vel * sigma_vel, sigma_vel * sigma_vel,
      sigma_vel_up * sigma_vel_up;
  const matrix6 P0 = variances.asDiagonal ();

  filter_ = kalman_filter<6>{ x0, P0 };
  zone_number_ = fix.position.zone_number ();
  zone_char_ = fix.position.zone_char ();
  last_fix_time_ = fix.time_s;
  seconds_since_fix_ = 0;
  imu_samples_since_fix_ = 0;
  rejected_fixes_ = 0;
  recent_accel_grid_.reset ();
  initialized_ = true;
}

void
position_estimator::propagate (kalman_filter<6> &filter, double dt,
                               const vector3 *accel_grid) const
{
  matrix6 F = matrix6::Identity ();
  F.topRightCorner<3, 3> () = matrix3::Identity () * dt;

  const double sigma = accel_grid ? config_.imu_accel_sigma_mps2
                                  : config_.maneuver_accel_sigma_mps2;
  // Continuous white noise acceleration: the uncertainty added over a time
  // span does not depend on how many steps it is split into, so 100 Hz IMU
  // steps and 1 Hz GPS-only steps grow the covariance equally.
  const double q = sigma * sigma;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  matrix6 Q;
  Q << matrix3::Identity () * (q * dt3 / 3), matrix3::Identity () * (q * dt2 / 2),
      matrix3::Identity () * (q * dt2 / 2), matrix3::Identity () * (q * dt);
  vector6 u = vector6::Zero ();
  if (accel_grid)
    {
      u << *accel_grid * (dt2 / 2), *accel_grid * dt;
    }
  filter.predict (F, Q, u);
}

bool
position_estimator::add_imu_sample (const imu_sample &sample)
{
  double dt = config_.imu_nominal_dt_s;
  if (last_imu_time_)
    {
      dt = sample.time_s - *last_imu_time_;
    }
  last_imu_time_ = sample.time_s;
  if (dt <= 0 || dt > config_.imu_max_dt_s)
    {
      dt = config_.imu_nominal_dt_s;
    }

  if (!initialized_)
    {
      return false;
    }

  const vector3 accel = sample.acceleration_enu (config_.gravity_mps2);
  vector3 grid;
  grid << true_to_grid (accel.x (), accel.y (), convergence_), accel.z ();
  propagate (filter_, dt, &grid);

  const double alpha = dt / (config_.prediction_accel_tau_s + dt);
  if (!recent_accel_grid_)
    {
      recent_accel_grid_ = grid;
    }
  *recent_accel_grid_ += alpha * (grid - *recent_accel_grid_);

  seconds_since_fix_ += dt;
  ++imu_samples_since_fix_;
  return true;
}

position_estimator::fix_result
position_estimator::add_gps_fix (const gps_fix &fix)
{
  if (!initialized_ || fix.position.zone_number () != zone_number_
      || fix.position.is_southern () != (zone_char_ < 'N'))
    {
      // TODO: carry the state across UTM zone boundaries instead of
      // restarting.
      initialize (fix);
      return fix_result::initialized;
    }

  const double dt = fix.time_s - last_fix_time_;
  if (dt <= 0)
    {
      return fix_result::rejected; // duplicate or out of order
    }
  if (imu_samples_since_fix_ == 0)
    {
      // No IMU: the GPS fixes themselves drive the prediction.
      propagate (filter_, dt, nullptr);
    }

  last_fix_time_ = fix.time_s;
  seconds_since_fix_ = 0;
  imu_samples_since_fix_ = 0;
  convergence_ = fix.position.convergence ();

  const double sigma_pos = horizontal_sigma (fix, config_);
  // Gates and applies the position part of the fix, with or without altitude.
  const auto apply
      = [this] (const auto &z, const auto &H, const auto &R, double gate)
    {
      if (filter_.nis (z, H, R) > gate)
        {
          return false;
        }
      filter_.update (z, H, R);
      return true;
    };

  bool passed{ false };
  if (fix.altitude_m)
    {
      const double sigma_alt = sigma_pos * config_.gps_vertical_factor;
      const matrix3 R
          = vector3 (sigma_pos * sigma_pos, sigma_pos * sigma_pos,
                     sigma_alt * sigma_alt)
                .asDiagonal ();
      passed = apply (vector3 (fix.position.easting (),
                               fix.position.northing (), *fix.altitude_m),
                      H_position, R, config_.gate_nis_3d);
    }
  else
    {
      passed = apply (
          vector2 (fix.position.easting (), fix.position.northing ()),
          H_horizontal, diagonal_noise (sigma_pos), config_.gate_nis);
    }

  if (!passed)
    {
      if (++rejected_fixes_ > config_.max_rejected_fixes)
        {
          initialize (fix);
          return fix_result::reset;
        }
      return fix_result::rejected;
    }
  rejected_fixes_ = 0;

  if (fix.speed_mps < config_.stationary_speed_mps)
    {
      filter_.update (vector2::Zero ().eval (), H_velocity,
                      diagonal_noise (config_.stationary_velocity_sigma_mps));
    }
  else if (const auto v = velocity_from_fix (fix, convergence_))
    {
      const auto R_v = diagonal_noise (config_.gps_velocity_sigma_mps);
      if (filter_.nis (*v, H_velocity, R_v) <= config_.gate_nis)
        {
          filter_.update (*v, H_velocity, R_v);
        }
    }
  return fix_result::accepted;
}

position_estimate
position_estimator::to_estimate (const kalman_filter<6> &filter,
                                 double age_s) const
{
  const auto &x = filter.state ();
  const auto &P = filter.covariance ();
  return { .position = utm (x (0), x (1), zone_number_, zone_char_),
           .altitude_m = x (2),
           .age_s = age_s,
           .velocity_east_mps = x (3),
           .velocity_north_mps = x (4),
           .velocity_up_mps = x (5),
           .sigma_east_m = std::sqrt (P (0, 0)),
           .sigma_north_m = std::sqrt (P (1, 1)),
           .sigma_up_m = std::sqrt (P (2, 2)),
           .sigma_velocity_east_mps = std::sqrt (P (3, 3)),
           .sigma_velocity_north_mps = std::sqrt (P (4, 4)),
           .sigma_velocity_up_mps = std::sqrt (P (5, 5)) };
}

std::optional<position_estimate>
position_estimator::estimate () const
{
  if (!initialized_)
    {
      return std::nullopt;
    }
  return to_estimate (filter_, seconds_since_fix_);
}

std::optional<position_estimate>
position_estimator::predict_ahead (double horizon_s) const
{
  if (!initialized_)
    {
      return std::nullopt;
    }
  auto future = filter_;
  propagate (future, horizon_s,
             recent_accel_grid_ ? &*recent_accel_grid_ : nullptr);
  return to_estimate (future, seconds_since_fix_ + horizon_s);
}
