// A ROS 2 node between Cosys-AirSim (its airsim_ros_pkgs node) and the
// position estimator (localization_core airsim/):
//
//   /airsim_node/drone/imu/imu     (sensor_msgs/Imu)       -> imu_sample -> add_imu_sample
//   /airsim_node/drone/gps/gps     (sensor_msgs/NavSatFix) -> gps_fix    -> add_gps_fix
//   /airsim_node/drone/odom_local  (nav_msgs/Odometry)     -> the autopilot,
//                                  velocity for the GPS fix
//   /cmd_vel (geometry_msgs/Twist, body frame) <- the figure-eight autopilot
//   /positioning/estimate, /prediction, /baseline, /gps (geometry_msgs/
//     PoseStamped, frame enu)  -> rosbag2, evaluate.py, rviz2
//   /positioning/markers (visualization_msgs/Marker) -> rviz2
//
// Frames come from tf2 (localization_core sim/src/ros_sim.hpp): AirSim's
// north-west-up world is turned into enu with the static transform
// airsim.launch.py publishes. AirSim stamps run on its own clock, behind
// the wall clock; the estimator gets seconds since the first message, the
// published estimates keep the original stamps so they line up with the
// truth (TF enu -> drone).
//
// After every IMU sample the filter predicts where the drone will be
// `horizon` seconds later. That prediction is published stamped with the
// time it is for (and drawn as a translucent ghost), so evaluate.py can
// compare it with the truth at that time. The baseline is the last raw fix
// extrapolated with its velocity, the same horizon ahead.
//
// Options are ROS parameters (`ros2 param describe /positioning_sim <name>`);
// sim/run.sh turns --name value into name:=value.

#include "ros_sim.hpp"

#include "kalman/position_estimator.hpp"
#include "kalman/measurements.hpp"
#include "utility/queue.hpp"

#include <GeographicLib/LocalCartesian.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Geometry>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numbers>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <variant>

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void
handle_signal (int /*signal*/)
{
  stop_requested = 1;
}

template <typename... Ts> struct overloaded : Ts...
{
  using Ts::operator()...;
};

constexpr double pi = std::numbers::pi;
constexpr double deg = pi / 180;
// AirSim's gravity (msr::airlib::EarthUtils, SimpleFlight).
constexpr double world_gravity = 9.8;

using vec3 = Eigen::Vector3d;

struct options
{
  double horizon_s;
  double gps_noise_m;
  double gps_drift_m;
  double gps_drift_tau_s;
  double speed_noise_mps;
  double gps_vertical_factor;
  double accel_bias_mps2;
  double tilt_error_deg;
  double heading_error_deg;
  double orientation_error_tau_s;
  double origin_lat;
  double origin_lon;
  double origin_alt;
  double duration_s;
  double radius_m;
  double altitude_m;
  double altitude_swing_m;
  unsigned seed;
  bool manual;
};

options
read_options (rclcpp::Node &node)
{
  using ros_sim::declare;
  options o{};
  o.horizon_s = declare (node, "horizon", 1.0, "predict this many seconds ahead");
  o.gps_noise_m = declare (node, "gps_noise", 1.5, "white GPS position noise, metres");
  o.gps_drift_m = declare (node, "gps_drift", 0.0,
                           "slowly wandering (Gauss-Markov) GPS error, metres");
  o.gps_drift_tau_s = declare (node, "gps_drift_tau", 60.0, "its correlation time, seconds");
  o.speed_noise_mps = declare (node, "speed_noise", 0.05, "GPS velocity noise, m/s");
  o.gps_vertical_factor
      = declare (node, "gps_vfactor", 1.7,
                 "altitude noise and drift relative to the horizontal ones");
  // IMU errors on top of AirSim's white noise. The bias is a constant per
  // sensor axis drawn at startup, the orientation error a slowly wandering
  // rotation (first order Gauss-Markov) in the local frame.
  o.accel_bias_mps2 = declare (node, "accel_bias", 0.02, "accelerometer bias per axis, m/s^2");
  o.tilt_error_deg = declare (node, "tilt_error", 0.2, "roll/pitch error, degrees");
  o.heading_error_deg = declare (node, "heading_error", 1.0, "heading error, degrees");
  o.orientation_error_tau_s
      = declare (node, "orientation_error_tau", 30.0, "orientation error correlation time, s");
  o.origin_lat = declare (node, "origin_latitude", 39.97656,
                          "the enu frame's origin: AirSim's OriginGeopoint (the launch file passes it)");
  o.origin_lon = declare (node, "origin_longitude", 32.73812, "see origin_latitude");
  o.origin_alt = declare (node, "origin_altitude", 894.0, "see origin_latitude");
  o.duration_s = declare (node, "duration", 0.0, "stop after this many seconds (0: at Ctrl-C)");
  o.radius_m = declare (node, "radius", 12.0, "figure-eight loop radius, metres");
  o.altitude_m = declare (node, "altitude", 8.0, "cruise height above the ground, metres");
  o.altitude_swing_m = declare (node, "altitude_swing", 3.0,
                                "climb and descend this far around it, metres");
  o.seed = static_cast<unsigned> (declare (node, "seed", std::int64_t{ 1 }, "noise seed"));
  o.manual = declare (node, "manual", false,
                      "do not fly: fly it yourself (airsim/keyboard.sh publishes /cmd_vel)");
  if (o.horizon_s <= 0)
    {
      throw std::invalid_argument ("horizon must be positive");
    }
  return o;
}

// Rotation by the angle |r| about the axis r.
Eigen::Quaterniond
from_rotation_vector (const vec3 &r)
{
  const double angle = r.norm ();
  return angle < 1e-12 ? Eigen::Quaterniond::Identity ()
                       : Eigen::Quaterniond (Eigen::AngleAxisd (angle, r / angle));
}

// ---- events from the ROS callback thread ----------------------------------

struct imu_event
{
  double t;
  rclcpp::Time stamp;
  imu_sample sample;
};

struct navsat_event
{
  double t;
  rclcpp::Time stamp;
  double lat;
  double lon;
  double alt;
  vec3 velocity; // enu, from the odometry: a receiver's velocity output
};

struct truth_event
{
  double t;
  vec3 position; // enu
  double yaw;    // from east, counterclockwise
};

using sim_event = std::variant<imu_event, navsat_event, truth_event>;

// ---- the bridge -----------------------------------------------------------

class positioning_sim
{
  // A prediction whose time has not come yet: drawn against the truth then.
  struct prediction
  {
    rclcpp::Time target; // on AirSim's clock
    double target_t;     // the same in seconds since the first message
    vec3 position;
  };

  rclcpp::Node::SharedPtr node_;
  options opt_;
  ros_sim::frames frames_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_{};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_{};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimate_pub_{};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr prediction_pub_{};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr baseline_pub_{};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gps_pub_{};
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_{};
  std::thread spin_thread_{};
  // ROS callback thread state: the time base and the latest true velocity.
  std::mutex ros_mutex_{};
  std::optional<double> t0_{};
  vec3 velocity_enu_{ vec3::Zero () };
  std::int64_t last_gps_stamp_{ -1 };
  queue<sim_event> events_{};
  GeographicLib::LocalCartesian frame_;
  position_estimator estimator_;

  std::mt19937 rng_;
  std::normal_distribution<double> unit_noise_{ 0.0, 1.0 };
  vec3 gps_drift_{ vec3::Zero () };
  double last_gps_t_{ -1 };

  // Separate generator for the altitude and IMU errors, so that turning
  // them off reproduces the horizontal GPS noise of earlier versions.
  std::mt19937 error_rng_;
  std::normal_distribution<double> error_noise_{ 0.0, 1.0 };
  vec3 accel_bias_{};
  vec3 orientation_error_{ vec3::Zero () }; // rotation vector, enu [rad]
  double last_imu_t_{ -1 };

  struct raw_fix
  {
    rclcpp::Time stamp;
    vec3 position;
    vec3 velocity;
  };
  std::optional<raw_fix> last_raw_fix_{};
  std::deque<prediction> pending_{};
  std::size_t gps_fixes_{ 0 };
  std::size_t gps_rejected_{ 0 };
  std::optional<double> last_error_{};

  double sim_t_{ 0 };
  double last_print_t_{ 0 };
  double last_ghost_t_{ -1 };
  double last_cmd_t_{ -1 };

  // figure-eight flying
  double turn_direction_{ 1 };
  double turned_{ 0 };
  bool airborne_{ false };
  std::optional<double> last_yaw_{};

public:
  explicit positioning_sim (const rclcpp::Node::SharedPtr &node)
      : node_ (node), opt_ (read_options (*node)), frames_ (*node),
        frame_ (opt_.origin_lat, opt_.origin_lon, opt_.origin_alt),
        estimator_ (make_estimator_config (opt_)), rng_ (opt_.seed),
        error_rng_ (opt_.seed + 1)
  {
    for (auto &b : accel_bias_)
      {
        b = opt_.accel_bias_mps2 * error_noise_ (error_rng_);
      }
  }

  static estimator_config
  make_estimator_config (const options &opt)
  {
    estimator_config config{};
    config.gravity_mps2 = world_gravity;
    config.gps_vertical_factor = opt.gps_vertical_factor;
    return config;
  }

  void
  start ()
  {
    using geometry_msgs::msg::PoseStamped;
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist> ("/cmd_vel", 10);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker> (
        "/positioning/markers", 50);
    estimate_pub_ = node_->create_publisher<PoseStamped> ("/positioning/estimate", 100);
    prediction_pub_ = node_->create_publisher<PoseStamped> ("/positioning/prediction", 100);
    baseline_pub_ = node_->create_publisher<PoseStamped> ("/positioning/baseline", 100);
    gps_pub_ = node_->create_publisher<PoseStamped> ("/positioning/gps", 10);
    const std::string prefix = "/airsim_node/drone/";
    subscriptions_.push_back (node_->create_subscription<sensor_msgs::msg::Imu> (
        prefix + "imu/imu", 100, [this] (const sensor_msgs::msg::Imu &m) { on_imu (m); }));
    subscriptions_.push_back (node_->create_subscription<sensor_msgs::msg::NavSatFix> (
        prefix + "gps/gps", 10, [this] (const sensor_msgs::msg::NavSatFix &m) { on_gps (m); }));
    subscriptions_.push_back (node_->create_subscription<nav_msgs::msg::Odometry> (
        prefix + "odom_local", 100, [this] (const nav_msgs::msg::Odometry &m) { on_odom (m); }));
    spin_thread_ = std::thread ([this] { rclcpp::spin (node_); });
  }

  void
  run ()
  {
    std::cout << "waiting for AirSim's ROS 2 node (/airsim_node/drone/...)\n";
    bool announced = false;
    while (stop_requested == 0)
      {
        auto event = events_.pop_for (std::chrono::milliseconds (200));
        if (!event)
          {
            continue;
          }
        if (!announced)
          {
            announced = true;
            std::cout << "connected, predicting " << opt_.horizon_s << " s ahead\n";
          }
        std::visit (
            overloaded{
                [this] (const imu_event &e) { handle (e); },
                [this] (const navsat_event &e) { handle (e); },
                [this] (const truth_event &e) { handle (e); },
            },
            *event);

        if (sim_t_ - last_print_t_ >= 1.0)
          {
            last_print_t_ = sim_t_;
            print_status ();
          }
        if (opt_.duration_s > 0 && sim_t_ >= opt_.duration_s)
          {
            break;
          }
      }
    hover ();
    rclcpp::shutdown ();
    if (spin_thread_.joinable ())
      spin_thread_.join ();
    std::cout << "done after " << std::fixed << std::setprecision (1) << sim_t_
              << " s; gps fixes " << gps_fixes_ << ", rejected by the filter "
              << gps_rejected_ << "\n";
  }

private:
  // -- ROS callbacks (spin thread): only convert and enqueue --------------

  // Seconds since the first message (AirSim stamps run on its own clock).
  double
  sim_time (const rclcpp::Time &stamp)
  {
    std::lock_guard lock (ros_mutex_);
    if (!t0_)
      t0_ = stamp.seconds ();
    return stamp.seconds () - *t0_;
  }

  void
  on_imu (const sensor_msgs::msg::Imu &msg)
  {
    // The orientation is body -> AirSim's world (msg frame "drone": the
    // node reports it against world); enu <- body = (enu <- world) * that.
    const auto world = frames_.fixed ("world");
    if (!world)
      {
        return; // the static transforms are not there yet
      }
    const rclcpp::Time stamp (msg.header.stamp);
    const double t = sim_time (stamp);
    Eigen::Quaterniond q;
    tf2::fromMsg (msg.orientation, q);
    q = Eigen::Quaterniond (world->rotation ()) * q;
    imu_sample sample{};
    sample.time_s = t;
    sample.orientation = q;
    // Specific force is in the body frame (forward, left, up), the frame the
    // estimator expects.
    const auto &a = msg.linear_acceleration;
    sample.specific_force = { a.x, a.y, a.z };
    events_.push (imu_event{ t, stamp, sample });
  }

  void
  on_gps (const sensor_msgs::msg::NavSatFix &msg)
  {
    // The node republishes the latest GPS output at its own rate; a new fix
    // has a new stamp (the receiver's update time, 1 Hz in the settings).
    const rclcpp::Time stamp (msg.header.stamp);
    if (stamp.nanoseconds () == last_gps_stamp_)
      return;
    last_gps_stamp_ = stamp.nanoseconds ();
    const double t = sim_time (stamp);
    vec3 v;
    {
      std::lock_guard lock (ros_mutex_);
      v = velocity_enu_;
    }
    events_.push (navsat_event{ t, stamp, msg.latitude, msg.longitude, msg.altitude, v });
  }

  void
  on_odom (const nav_msgs::msg::Odometry &msg)
  {
    // The pose is in the odometry frame (drone/odom_local), which tf2
    // places in enu; AirSim's linear velocity is in its world axes.
    const auto odom = frames_.fixed (msg.header.frame_id);
    if (!odom)
      {
        return;
      }
    Eigen::Isometry3d body;
    tf2::fromMsg (msg.pose.pose, body);
    const Eigen::Isometry3d pose = *odom * body;
    vec3 v;
    tf2::fromMsg (msg.twist.twist.linear, v);
    const double t = sim_time (rclcpp::Time (msg.header.stamp));
    {
      std::lock_guard lock (ros_mutex_);
      velocity_enu_ = odom->rotation () * v;
    }
    const vec3 forward = pose.rotation () * vec3::UnitX ();
    events_.push (truth_event{ t, pose.translation (), std::atan2 (forward.y (), forward.x ()) });
  }

  // -- main thread -----------------------------------------------------

  [[nodiscard]] vec3
  to_local (double lat, double lon, double altitude) const
  {
    vec3 p;
    frame_.Forward (lat, lon, altitude, p.x (), p.y (), p.z ());
    return p;
  }

  [[nodiscard]] vec3
  to_local (const utm &position, double altitude) const
  {
    const auto [lat, lon] = position.to_latlon ();
    return to_local (lat, lon, altitude);
  }

  // The IMU model: AirSim gives white noise and the exact orientation, a
  // real AHRS also has bias and orientation error. A tilt error leaks
  // gravity into the horizontal acceleration.
  [[nodiscard]] imu_sample
  with_imu_errors (const imu_event &e)
  {
    const vec3 sigma{ opt_.tilt_error_deg * deg, opt_.tilt_error_deg * deg,
                      opt_.heading_error_deg * deg };
    const double dt = last_imu_t_ < 0 ? 0 : e.t - last_imu_t_;
    // The first sample starts from the steady state distribution.
    const double phi = last_imu_t_ < 0 ? 0 : std::exp (-dt / opt_.orientation_error_tau_s);
    last_imu_t_ = e.t;
    for (int i = 0; i < 3; ++i)
      {
        orientation_error_[i] = (phi * orientation_error_[i])
                                + (sigma[i] * std::sqrt (1 - phi * phi)
                                   * error_noise_ (error_rng_));
      }

    imu_sample sample = e.sample;
    sample.specific_force += accel_bias_;
    // The orientation maps sensor to local, so the local frame error goes on
    // the left.
    sample.orientation = from_rotation_vector (orientation_error_) * sample.orientation;
    return sample;
  }

  void
  handle (const imu_event &e)
  {
    sim_t_ = std::max (sim_t_, e.t);
    if (!estimator_.add_imu_sample (with_imu_errors (e)))
      {
        return;
      }

    const auto now = estimator_.estimate ();
    const auto ahead = estimator_.predict_ahead (opt_.horizon_s);
    if (!now || !ahead)
      {
        return;
      }
    const vec3 now_local = to_local (now->position, now->altitude_m);
    const vec3 ahead_local = to_local (ahead->position, ahead->altitude_m);
    const rclcpp::Time target = e.stamp + rclcpp::Duration::from_seconds (opt_.horizon_s);

    estimate_pub_->publish (ros_sim::pose_msg (e.stamp, now_local));
    prediction_pub_->publish (ros_sim::pose_msg (target, ahead_local));
    if (last_raw_fix_)
      {
        const double dt = (target - last_raw_fix_->stamp).seconds ();
        baseline_pub_->publish (ros_sim::pose_msg (
            target, last_raw_fix_->position + (last_raw_fix_->velocity * dt)));
      }
    pending_.push_back ({ target, e.t + opt_.horizon_s, ahead_local });

    if (e.t - last_ghost_t_ >= 1.0 / 30.0)
      {
        last_ghost_t_ = e.t;
        draw_ghost (now_local, ahead_local);
      }
  }

  void
  handle (const navsat_event &e)
  {
    sim_t_ = std::max (sim_t_, e.t);
    const double dt = last_gps_t_ < 0 ? 0 : e.t - last_gps_t_;
    last_gps_t_ = e.t;

    // Receiver model: white noise plus an optional slowly wandering
    // (first order Gauss-Markov) error.
    if (opt_.gps_drift_m > 0 && dt > 0)
      {
        const double phi = std::exp (-dt / opt_.gps_drift_tau_s);
        const double s = opt_.gps_drift_m * std::sqrt (1 - phi * phi);
        gps_drift_.x () = phi * gps_drift_.x () + s * unit_noise_ (rng_);
        gps_drift_.y () = phi * gps_drift_.y () + s * unit_noise_ (rng_);
        gps_drift_.z () = phi * gps_drift_.z ()
                          + opt_.gps_vertical_factor * s * error_noise_ (error_rng_);
      }
    const double noise_up
        = gps_drift_.z ()
          + opt_.gps_vertical_factor * opt_.gps_noise_m * error_noise_ (error_rng_);
    const double noise_e = gps_drift_.x () + opt_.gps_noise_m * unit_noise_ (rng_);
    const double noise_n = gps_drift_.y () + opt_.gps_noise_m * unit_noise_ (rng_);
    const double v_east = e.velocity.x () + opt_.speed_noise_mps * unit_noise_ (rng_);
    const double v_north = e.velocity.y () + opt_.speed_noise_mps * unit_noise_ (rng_);
    // Only the raw baseline gets a climb rate: NMEA has no vertical velocity,
    // so the filter never sees this.
    const double v_up = e.velocity.z () + opt_.speed_noise_mps * error_noise_ (error_rng_);
    const double speed = std::hypot (v_east, v_north);

    gps_fix fix{};
    fix.time_s = e.t;
    try
      {
        fix.position = utm (e.lat, e.lon);
      }
    catch (const std::exception &ex)
      {
        std::cerr << "navsat: " << ex.what () << "\n";
        return;
      }
    fix.position.update_easting (noise_e);
    fix.position.update_northing (noise_n);
    fix.altitude_m = e.alt + noise_up;
    fix.speed_mps = speed;
    if (speed >= 0.3) // receivers leave the course empty when slow
      {
        double course = std::atan2 (v_east, v_north) * 180 / pi;
        fix.course_deg = course < 0 ? course + 360 : course;
      }
    fix.hdop = std::max (opt_.gps_noise_m, 0.5) / 2.0; // sigma = hdop * 2 m

    const vec3 raw = to_local (fix.position, *fix.altitude_m);
    last_raw_fix_ = raw_fix{ e.stamp, raw, { v_east, v_north, v_up } };
    gps_pub_->publish (ros_sim::pose_msg (e.stamp, raw));

    ++gps_fixes_;
    const auto result = estimator_.add_gps_fix (fix);
    if (result == position_estimator::fix_result::rejected
        || result == position_estimator::fix_result::reset)
      {
        ++gps_rejected_;
      }
    draw_gps_fix (raw);
  }

  void
  handle (const truth_event &e)
  {
    sim_t_ = std::max (sim_t_, e.t);
    check_predictions ();
    fly (e);
  }

  // Draws the latest prediction whose time has come next to where the drone
  // really was then (tf2 interpolates the truth). Only for the eye and the
  // status line: evaluate.py scores the recording.
  void
  check_predictions ()
  {
    std::optional<std::pair<vec3, vec3>> latest;
    while (!pending_.empty ())
      {
        const auto truth = frames_.lookup ("drone", pending_.front ().target);
        if (!truth)
          {
            // Not there yet, or already too old for tf2's buffer. (AirSim's
            // clock is not the wall clock: compare with the simulation's.)
            if (sim_t_ - pending_.front ().target_t > 5.0)
              {
                pending_.pop_front ();
                continue;
              }
            break;
          }
        latest = { pending_.front ().position, truth->translation () };
        pending_.pop_front ();
      }
    if (latest)
      {
        last_error_ = (latest->first - latest->second).norm ();
        draw_check (latest->first, latest->second);
      }
  }

  // Takes off, then flies a figure eight: full circles alternating left and
  // right, with the speed varying between 1 and 4 m/s and the height swinging
  // around the cruise altitude, so the filter sees acceleration on all axes.
  // The commands are the autopilot's job and use the true pose; the filter
  // only ever gets the IMU and the noisy GPS.
  void
  fly (const truth_event &e)
  {
    if (last_yaw_ && airborne_)
      {
        turned_ += std::remainder (e.yaw - *last_yaw_, 2 * pi);
      }
    last_yaw_ = e.yaw;

    if (opt_.manual || e.t - last_cmd_t_ < 0.05)
      {
        return;
      }
    last_cmd_t_ = e.t;

    geometry_msgs::msg::Twist cmd;
    if (e.t > 3.0) // sit on the ground for the first fix
      {
        constexpr double swing_period_s = 17.0;
        constexpr double max_climb_mps = 2.0;
        constexpr double climb_gain = 1.0;

        const double swing_rate = 2 * pi / swing_period_s;
        const double target
            = opt_.altitude_m + opt_.altitude_swing_m * std::sin (swing_rate * e.t);
        const double feed_forward
            = airborne_ ? opt_.altitude_swing_m * swing_rate * std::cos (swing_rate * e.t)
                        : 0.0;
        cmd.linear.z = std::clamp (feed_forward + climb_gain * (target - e.position.z ()),
                                   -max_climb_mps, max_climb_mps);

        airborne_ = airborne_ || e.position.z () > opt_.altitude_m / 2;
        if (airborne_)
          {
            const double speed = 2.5 + 1.5 * std::sin (2 * pi * e.t / 23.0);
            if (std::abs (turned_) >= 2 * pi)
              {
                turn_direction_ = -turn_direction_;
                turned_ = 0;
              }
            cmd.linear.x = speed;
            cmd.angular.z = turn_direction_ * speed / opt_.radius_m;
          }
      }
    cmd_pub_->publish (cmd);
  }

  void
  hover ()
  {
    if (!opt_.manual)
      {
        cmd_pub_->publish (geometry_msgs::msg::Twist{});
      }
  }

  // -- drawing (rviz2, fixed frame enu) -------------------------------------

  using marker_msg = visualization_msgs::msg::Marker;

  static marker_msg
  marker (const std::string &ns, std::uint64_t id, int type, const vec3 &position,
          const vec3 &scale, const std::array<float, 4> &rgba)
  {
    marker_msg m;
    m.header.frame_id = ros_sim::world_frame;
    m.ns = ns;
    m.id = static_cast<int> (id);
    m.type = type;
    m.action = marker_msg::ADD;
    m.pose = ros_sim::pose_msg (rclcpp::Time (), position).pose;
    m.scale.x = scale.x ();
    m.scale.y = scale.y ();
    m.scale.z = scale.z ();
    m.color.r = rgba[0];
    m.color.g = rgba[1];
    m.color.b = rgba[2];
    m.color.a = rgba[3];
    return m;
  }

  static marker_msg
  line (const std::string &ns, std::uint64_t id, const vec3 &a, const vec3 &b,
        const std::array<float, 4> &rgba)
  {
    auto m = marker (ns, id, marker_msg::LINE_LIST, vec3::Zero (), vec3::Constant (0.05), rgba);
    m.points = { tf2::toMsg (a), tf2::toMsg (b) };
    return m;
  }

  void
  draw_ghost (const vec3 &now, const vec3 &ahead)
  {
    // Ghost: the airframe's size, heading along the predicted motion.
    const vec3 d = ahead - now;
    const double yaw = std::hypot (d.x (), d.y ()) > 0.05 ? std::atan2 (d.y (), d.x ())
                                                          : last_yaw_.value_or (0);
    auto ghost = marker ("ghost", 1, marker_msg::CUBE, ahead, { 0.8, 0.8, 0.15 },
                         { 0.2F, 1.0F, 0.2F, 0.55F });
    ghost.pose.orientation = tf2::toMsg (Eigen::Quaterniond (Eigen::AngleAxisd (yaw, vec3::UnitZ ())));
    marker_pub_->publish (ghost);
    marker_pub_->publish (line ("ghost", 2, now, ahead, { 1, 1, 1, 0.8F }));
  }

  // Where an earlier ghost said we would be now, and where we really are.
  void
  draw_check (const vec3 &predicted, const vec3 &truth)
  {
    marker_pub_->publish (marker ("check", 1, marker_msg::SPHERE, predicted,
                                  vec3::Constant (0.25), { 1, 0.5F, 0, 0.9F }));
    marker_pub_->publish (line ("check", 2, predicted, truth, { 1, 0.1F, 0.1F, 1 }));
  }

  void
  draw_gps_fix (const vec3 &raw)
  {
    // The last 20 raw GPS fixes as small dots.
    const vec3 p{ raw.x (), raw.y (), std::max (raw.z (), 0.1) };
    marker_pub_->publish (marker ("gps", gps_fixes_ % 20, marker_msg::SPHERE, p,
                                  vec3::Constant (0.3), { 0.9F, 0.9F, 0.1F, 0.9F }));
  }

  void
  print_status () const
  {
    std::cout << std::fixed << std::setprecision (2) << "t=" << std::setw (6) << sim_t_
              << "s | ghost +" << opt_.horizon_s << "s, latest 3D error "
              << (last_error_ ? std::to_string (*last_error_) : std::string ("-"))
              << " m | gps fixes " << gps_fixes_ << " (rejected " << gps_rejected_ << ")"
              << std::endl;
  }
};
} // namespace

int
main (int argc, char *argv[])
{
  rclcpp::init (argc, argv, rclcpp::InitOptions (), rclcpp::SignalHandlerOptions::None);
  std::signal (SIGINT, handle_signal);
  std::signal (SIGTERM, handle_signal);
  try
    {
      positioning_sim sim (rclcpp::Node::make_shared ("positioning_sim"));
      sim.start ();
      sim.run ();
    }
  catch (const std::exception &e)
    {
      std::cerr << "positioning_sim: " << e.what () << "\n";
      rclcpp::shutdown ();
      return 2;
    }
  return 0;
}
