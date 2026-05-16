#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "quad_midbridge_msgs/msg/executable_reference.hpp"
#include "quad_midbridge_msgs/msg/local_intent.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.81;

inline double clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(x, hi));
}

inline double wrapAngle(double a)
{
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

inline double norm2(double x, double y)
{
  return std::sqrt(x * x + y * y);
}

inline double norm3(double x, double y, double z)
{
  return std::sqrt(x * x + y * y + z * z);
}
}  // namespace

class GovernorNode : public rclcpp::Node
{
public:
  GovernorNode() : Node("governor_node")
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    raw_ref_timeout_sec_ = declare_parameter<double>("raw_ref_timeout_sec", 0.3);
    intent_timeout_sec_ = declare_parameter<double>("intent_timeout_sec", 0.6);
    local_pos_timeout_sec_ = declare_parameter<double>("local_pos_timeout_sec", 0.5);

    max_step_xy_ = declare_parameter<double>("max_step_xy", 0.03);
    max_step_z_ = declare_parameter<double>("max_step_z", 0.10);
    max_vel_xy_ = declare_parameter<double>("max_vel_xy", 0.45);
    max_vel_z_ = declare_parameter<double>("max_vel_z", 0.7);
    max_accel_xy_ = declare_parameter<double>("max_accel_xy", 0.9);
    max_accel_z_ = declare_parameter<double>("max_accel_z", 2.0);
    max_jerk_xy_ = declare_parameter<double>("max_jerk_xy", 1.8);
    max_jerk_z_ = declare_parameter<double>("max_jerk_z", 4.0);
    max_yaw_step_ = declare_parameter<double>("max_yaw_step", 0.10);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.3);

    min_thrust_nominal_ = declare_parameter<double>("min_thrust_nominal", 4.5);
    max_thrust_nominal_ = declare_parameter<double>("max_thrust_nominal", 15.0);
    max_tilt_nominal_ = declare_parameter<double>("max_tilt_nominal", 0.35);

    hover_z_ned_ = declare_parameter<double>("hover_z_ned", -1.5);
    terminal_xy_tol_ = declare_parameter<double>("terminal_xy_tol", 0.25);
    terminal_z_tol_ = declare_parameter<double>("terminal_z_tol", 0.20);
    enable_terminal_bridge_ = declare_parameter<bool>("enable_terminal_bridge", true);
    terminal_bridge_duration_sec_ = declare_parameter<double>("terminal_bridge_duration_sec", 4.0);
    terminal_bridge_min_distance_m_ = declare_parameter<double>("terminal_bridge_min_distance_m", 0.03);
    hover_on_timeout_ = declare_parameter<bool>("hover_on_timeout", true);
    timeout_hover_feasible_ = declare_parameter<bool>("timeout_hover_feasible", true);

    const auto height_mode_name = declare_parameter<std::string>("height_mode", "takeoff_relative");
    height_mode_ = parseHeightMode(height_mode_name);
    path_z_kp_ = declare_parameter<double>("path_z_kp", 2.0);
    path_z_kd_ = declare_parameter<double>("path_z_kd", 1.5);
    path_z_accel_correction_min_ = declare_parameter<double>("path_z_accel_correction_min", -1.2);
    path_z_accel_correction_max_ = declare_parameter<double>("path_z_accel_correction_max", 1.2);

    enable_takeoff_gate_ = declare_parameter<bool>("enable_takeoff_gate", true);
    takeoff_target_height_m_ = declare_parameter<double>("takeoff_target_height_m", 1.0);
    takeoff_release_height_m_ = declare_parameter<double>("takeoff_release_height_m", 0.95);
    takeoff_kp_z_ = declare_parameter<double>("takeoff_kp_z", 3.0);
    takeoff_kd_z_ = declare_parameter<double>("takeoff_kd_z", 1.0);
    takeoff_accel_z_min_ = declare_parameter<double>("takeoff_accel_z_min", -2.2);  // upward in NED
    takeoff_accel_z_max_ = declare_parameter<double>("takeoff_accel_z_max", 0.3);   // downward in NED

    // Do not release the gate immediately after reaching height.  The vehicle must
    // also be dynamically calm, otherwise the handover to MPCC can introduce a jump.
    takeoff_release_vz_abs_max_ = declare_parameter<double>("takeoff_release_vz_abs_max", 0.15);
    takeoff_release_vxy_abs_max_ = declare_parameter<double>("takeoff_release_vxy_abs_max", 0.20);
    takeoff_release_z_error_abs_max_ = declare_parameter<double>("takeoff_release_z_error_abs_max", 0.18);
    takeoff_release_stable_time_sec_ = declare_parameter<double>("takeoff_release_stable_time_sec", 1.20);

    // Lightweight XY damping during takeoff gate.
    // This is needed because PX4 attitude-thrust offboard does not directly track position.
    takeoff_kp_xy_ = declare_parameter<double>("takeoff_kp_xy", 0.35);
    takeoff_kd_xy_ = declare_parameter<double>("takeoff_kd_xy", 0.45);
    takeoff_accel_xy_max_ = declare_parameter<double>("takeoff_accel_xy_max", 0.25);

    takeoff_lock_xy_ = declare_parameter<bool>("takeoff_lock_xy", true);
    takeoff_lock_yaw_ = declare_parameter<bool>("takeoff_lock_yaw", true);
    post_takeoff_blend_time_sec_ = declare_parameter<double>("post_takeoff_blend_time_sec", 4.0);

    raw_ref_sub_ = create_subscription<quad_midbridge_msgs::msg::ExecutableReference>(
      "/midbridge/raw_reference", 10,
      std::bind(&GovernorNode::rawRefCallback, this, std::placeholders::_1));

    intent_sub_ = create_subscription<quad_midbridge_msgs::msg::LocalIntent>(
      "/midbridge/local_intent", 10,
      std::bind(&GovernorNode::intentCallback, this, std::placeholders::_1));

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
      std::bind(&GovernorNode::localPositionCallback, this, std::placeholders::_1));

    governed_pub_ = create_publisher<quad_midbridge_msgs::msg::ExecutableReference>(
      "/midbridge/governed_reference", 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&GovernorNode::timerCallback, this));

    last_out_.position.z = hover_z_ned_;
    last_out_.feasible = false;
    RCLCPP_INFO(
      get_logger(),
      "governor_node started with safety envelope, takeoff gate, height_mode=%s.",
      heightModeName());
  }

private:
  enum class HeightMode
  {
    TakeoffRelative,
    PathZ,
    Passthrough
  };

  HeightMode parseHeightMode(const std::string & mode) const
  {
    if (mode == "takeoff_relative") {
      return HeightMode::TakeoffRelative;
    }
    if (mode == "path_z") {
      return HeightMode::PathZ;
    }
    if (mode == "passthrough") {
      return HeightMode::Passthrough;
    }
    RCLCPP_WARN(
      get_logger(),
      "Unknown height_mode=\"%s\"; falling back to takeoff_relative. Valid modes are: takeoff_relative, path_z, passthrough.",
      mode.c_str());
    return HeightMode::TakeoffRelative;
  }

  const char * heightModeName() const
  {
    switch (height_mode_) {
      case HeightMode::TakeoffRelative: return "takeoff_relative";
      case HeightMode::PathZ: return "path_z";
      case HeightMode::Passthrough: return "passthrough";
    }
    return "unknown";
  }

  // Keep governor flags in high bits to avoid conflicts with MPCC backend flags.
  static constexpr uint32_t FLAG_RAW_TIMEOUT = 1u << 20;
  static constexpr uint32_t FLAG_INTENT_TIMEOUT = 1u << 21;
  static constexpr uint32_t FLAG_POS_CLAMP = 1u << 22;
  static constexpr uint32_t FLAG_VEL_CLAMP = 1u << 23;
  static constexpr uint32_t FLAG_ACC_CLAMP = 1u << 24;
  static constexpr uint32_t FLAG_JERK_CLAMP = 1u << 25;
  static constexpr uint32_t FLAG_YAW_CLAMP = 1u << 26;
  static constexpr uint32_t FLAG_THRUST_CLAMP = 1u << 27;
  static constexpr uint32_t FLAG_TILT_CLAMP = 1u << 28;
  static constexpr uint32_t FLAG_TERMINAL_HOLD = 1u << 29;
  static constexpr uint32_t FLAG_TAKEOFF_GATE = 1u << 30;
  static constexpr uint32_t FLAG_LOCAL_POS_TIMEOUT = 1u << 31;

  bool rawRefTimedOut() const
  {
    if (!latest_raw_) return true;
    return (now() - last_raw_ref_time_).seconds() > raw_ref_timeout_sec_;
  }

  bool intentTimedOut() const
  {
    if (!latest_intent_) return true;
    return (now() - last_intent_time_).seconds() > intent_timeout_sec_;
  }

  bool localPositionTimedOut() const
  {
    if (!have_local_position_) return true;
    return (now() - last_local_pos_time_).seconds() > local_pos_timeout_sec_;
  }

  void rawRefCallback(const quad_midbridge_msgs::msg::ExecutableReference::SharedPtr msg)
  {
    latest_raw_ = msg;
    last_raw_ref_time_ = now();
  }

  void intentCallback(const quad_midbridge_msgs::msg::LocalIntent::SharedPtr msg)
  {
    latest_intent_ = msg;
    last_intent_time_ = now();
  }

  void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    const double x = static_cast<double>(msg->x);
    const double y = static_cast<double>(msg->y);
    const double z = static_cast<double>(msg->z);
    const double vx = static_cast<double>(msg->vx);
    const double vy = static_cast<double>(msg->vy);
    const double vz = static_cast<double>(msg->vz);
    const double heading = static_cast<double>(msg->heading);

    const bool valid = msg->xy_valid && msg->z_valid && msg->v_xy_valid && msg->v_z_valid &&
                       std::isfinite(x) && std::isfinite(y) &&
                       std::isfinite(z) && std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz);
    if (!valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring invalid VehicleLocalPosition sample: xy_valid=%s z_valid=%s v_xy_valid=%s v_z_valid=%s",
        msg->xy_valid ? "true" : "false",
        msg->z_valid ? "true" : "false",
        msg->v_xy_valid ? "true" : "false",
        msg->v_z_valid ? "true" : "false");
      return;
    }

    local_x_ = x;
    local_y_ = y;
    local_z_ = z;
    local_vx_ = vx;
    local_vy_ = vy;
    local_vz_ = vz;
    local_heading_ = std::isfinite(heading) ? heading : local_heading_;
    last_local_pos_time_ = now();

    if (!have_local_position_) {
      have_local_position_ = true;
      latchTakeoffState();
    }
  }

  void latchTakeoffState()
  {
    takeoff_lock_x_ = local_x_;
    takeoff_lock_y_ = local_y_;
    takeoff_lock_z0_ = local_z_;
    takeoff_yaw_ = std::isfinite(local_heading_) ? wrapAngle(local_heading_) : 0.0;

    const double current_height = 0.0;
    takeoff_gate_released_ = false;
    takeoff_release_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    takeoff_release_candidate_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    last_out_.position.x = static_cast<float>(takeoff_lock_x_);
    last_out_.position.y = static_cast<float>(takeoff_lock_y_);
    last_out_.position.z = static_cast<float>(local_z_);
    last_out_.yaw = static_cast<float>(takeoff_yaw_);

    RCLCPP_INFO(
      get_logger(),
      "Latched takeoff state: x=%.2f y=%.2f z=%.2f height=%.2f yaw=%.3f released=%s",
      takeoff_lock_x_, takeoff_lock_y_, takeoff_lock_z0_, current_height, takeoff_yaw_,
      takeoff_gate_released_ ? "true" : "false");
  }

  bool takeoffGateActive() const
  {
    if (!enable_takeoff_gate_ || takeoff_gate_released_) {
      return false;
    }
    if (!have_local_position_ || localPositionTimedOut()) {
      return false;
    }
    // Once armed, keep the gate active until the full release condition is met.
    // Height alone is not enough; see takeoffReleaseReady().
    return true;
  }

  bool takeoffReleaseCandidateStarted() const
  {
    return takeoff_release_candidate_since_.nanoseconds() != 0;
  }

  bool takeoffReleaseReady(double & height)
  {
    height = std::max(0.0, takeoff_lock_z0_ - local_z_);

    const double target_z = takeoff_lock_z0_ - std::abs(takeoff_target_height_m_);
    const double z_error = std::abs(local_z_ - target_z);
    const double vxy = norm2(local_vx_, local_vy_);
    const double vz_abs = std::abs(local_vz_);

    const bool sample_ready =
      height >= takeoff_release_height_m_ &&
      z_error <= takeoff_release_z_error_abs_max_ &&
      vxy <= takeoff_release_vxy_abs_max_ &&
      vz_abs <= takeoff_release_vz_abs_max_;

    if (!sample_ready) {
      takeoff_release_candidate_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return false;
    }

    if (takeoff_release_stable_time_sec_ <= 1.0e-6) {
      return true;
    }

    if (!takeoffReleaseCandidateStarted()) {
      takeoff_release_candidate_since_ = now();
      return false;
    }

    return (now() - takeoff_release_candidate_since_).seconds() >= takeoff_release_stable_time_sec_;
  }

  quad_midbridge_msgs::msg::ExecutableReference makeHoverReference(bool feasible) const
  {
    quad_midbridge_msgs::msg::ExecutableReference out;
    out.header.stamp = now();
    out.header.frame_id = "map";
    out.position = last_out_.position;
    if (have_local_position_ && !localPositionTimedOut()) {
      out.position.x = static_cast<float>(local_x_);
      out.position.y = static_cast<float>(local_y_);
      out.position.z = static_cast<float>(local_z_);
    }
    if (!std::isfinite(out.position.z) || std::abs(out.position.z) < 1e-6) {
      out.position.z = hover_z_ned_;
    }
    out.velocity.x = 0.0f;
    out.velocity.y = 0.0f;
    out.velocity.z = 0.0f;
    out.acceleration.x = 0.0f;
    out.acceleration.y = 0.0f;
    out.acceleration.z = 0.0f;
    out.jerk.x = 0.0f;
    out.jerk.y = 0.0f;
    out.jerk.z = 0.0f;
    out.yaw = last_out_.yaw;
    out.yaw_rate = 0.0f;
    out.progress = last_out_.progress;
    out.contour_error = last_out_.contour_error;
    out.lag_error = last_out_.lag_error;
    const double hover_thrust = std::isfinite(last_out_.thrust_nominal) && last_out_.thrust_nominal > 1e-3 ?
      static_cast<double>(last_out_.thrust_nominal) : kGravity;
    out.thrust_nominal = static_cast<float>(clamp(hover_thrust, min_thrust_nominal_, max_thrust_nominal_));
    out.tilt_nominal = 0.0f;
    out.mode = last_out_.mode;
    out.feasible = feasible;
    out.violation_flags = last_out_.violation_flags;
    return out;
  }

  quad_midbridge_msgs::msg::ExecutableReference makeTakeoffGateReference(
    const quad_midbridge_msgs::msg::ExecutableReference & raw,
    uint32_t & flags)
  {
    quad_midbridge_msgs::msg::ExecutableReference out;
    out.header.stamp = now();
    out.header.frame_id = raw.header.frame_id.empty() ? "map" : raw.header.frame_id;

    const double target_z = takeoff_lock_z0_ - std::abs(takeoff_target_height_m_);
    const double err_z = target_z - local_z_;
    const double acc_z = clamp(
      takeoff_kp_z_ * err_z - takeoff_kd_z_ * local_vz_,
      takeoff_accel_z_min_, takeoff_accel_z_max_);

    double acc_x = 0.0;
    double acc_y = 0.0;
    if (takeoff_lock_xy_) {
      const double err_x = takeoff_lock_x_ - local_x_;
      const double err_y = takeoff_lock_y_ - local_y_;

      acc_x = takeoff_kp_xy_ * err_x - takeoff_kd_xy_ * local_vx_;
      acc_y = takeoff_kp_xy_ * err_y - takeoff_kd_xy_ * local_vy_;

      const double acc_xy = norm2(acc_x, acc_y);
      if (acc_xy > takeoff_accel_xy_max_) {
        const double s = takeoff_accel_xy_max_ / std::max(acc_xy, 1.0e-9);
        acc_x *= s;
        acc_y *= s;
      }
    }

    out.position.x = static_cast<float>(takeoff_lock_xy_ ? takeoff_lock_x_ : local_x_);
    out.position.y = static_cast<float>(takeoff_lock_xy_ ? takeoff_lock_y_ : local_y_);
    out.position.z = static_cast<float>(target_z);

    out.velocity.x = 0.0f;
    out.velocity.y = 0.0f;
    out.velocity.z = 0.0f;

    out.acceleration.x = static_cast<float>(acc_x);
    out.acceleration.y = static_cast<float>(acc_y);
    out.acceleration.z = static_cast<float>(acc_z);

    out.jerk.x = 0.0f;
    out.jerk.y = 0.0f;
    out.jerk.z = 0.0f;

    out.yaw = static_cast<float>(takeoff_lock_yaw_ ? takeoff_yaw_ : raw.yaw);
    out.yaw_rate = 0.0f;

    out.progress = raw.progress;
    out.contour_error = raw.contour_error;
    out.lag_error = raw.lag_error;

    // In NED, negative acc_z increases upward thrust: T/m ~= g - acc_z.
    const double thrust_nominal = norm3(acc_x, acc_y, kGravity - acc_z);
    const double tilt_nominal = std::atan2(norm2(acc_x, acc_y), std::max(1.0e-6, kGravity - acc_z));
    out.thrust_nominal = static_cast<float>(clamp(thrust_nominal, min_thrust_nominal_, max_thrust_nominal_));
    out.tilt_nominal = static_cast<float>(tilt_nominal);
    out.mode = raw.mode;
    out.feasible = raw.feasible;
    flags |= FLAG_TAKEOFF_GATE;
    out.violation_flags = flags;

    double height = 0.0;
    if (takeoffReleaseReady(height)) {
      markTakeoffReleased(height);
    }

    return out;
  }

  void markTakeoffReleased(double height)
  {
    if (!takeoff_gate_released_) {
      takeoff_gate_released_ = true;
      takeoff_release_time_ = now();
      RCLCPP_INFO(get_logger(), "Takeoff gate released at height %.2f m; smooth MPCC blending starts.", height);
    }
  }

  double postTakeoffBlendAlpha()
  {
    if (!takeoff_gate_released_ || post_takeoff_blend_time_sec_ <= 1e-6) {
      return 1.0;
    }
    return clamp((now() - takeoff_release_time_).seconds() / post_takeoff_blend_time_sec_, 0.0, 1.0);
  }

  void applyPostTakeoffBlend(quad_midbridge_msgs::msg::ExecutableReference & out, uint32_t & flags)
  {
    const double alpha = postTakeoffBlendAlpha();
    if (alpha >= 1.0) {
      return;
    }

    out.position.x = static_cast<float>(takeoff_lock_x_ + alpha * (static_cast<double>(out.position.x) - takeoff_lock_x_));
    out.position.y = static_cast<float>(takeoff_lock_y_ + alpha * (static_cast<double>(out.position.y) - takeoff_lock_y_));

    out.velocity.x = static_cast<float>(alpha * static_cast<double>(out.velocity.x));
    out.velocity.y = static_cast<float>(alpha * static_cast<double>(out.velocity.y));

    out.acceleration.x = static_cast<float>(alpha * static_cast<double>(out.acceleration.x));
    out.acceleration.y = static_cast<float>(alpha * static_cast<double>(out.acceleration.y));

    out.jerk.x = static_cast<float>(alpha * static_cast<double>(out.jerk.x));
    out.jerk.y = static_cast<float>(alpha * static_cast<double>(out.jerk.y));

    const double yaw_err = wrapAngle(static_cast<double>(out.yaw) - takeoff_yaw_);
    out.yaw = static_cast<float>(wrapAngle(takeoff_yaw_ + alpha * yaw_err));
    out.yaw_rate = static_cast<float>(alpha * static_cast<double>(out.yaw_rate));

    out.tilt_nominal = static_cast<float>(alpha * static_cast<double>(out.tilt_nominal));

    flags |= FLAG_TAKEOFF_GATE;
  }

  bool terminalHoldRequested(const quad_midbridge_msgs::msg::ExecutableReference & /*out*/) const
  {
    return latest_intent_ &&
      latest_intent_->terminal_hold &&
      !latest_intent_->centerline_points.empty();
  }

  struct QuinticSample
  {
    double p{0.0};
    double v{0.0};
    double a{0.0};
    double j{0.0};
  };

  static QuinticSample sampleQuintic(
    double p0,
    double v0,
    double a0,
    double p1,
    double v1,
    double a1,
    double duration,
    double elapsed)
  {
    const double T = std::max(duration, 1.0e-3);
    const double t = clamp(elapsed, 0.0, T);
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;

    const double c0 = p0;
    const double c1 = v0;
    const double c2 = 0.5 * a0;
    const double A = p1 - (c0 + c1 * T + c2 * T2);
    const double B = v1 - (c1 + 2.0 * c2 * T);
    const double C = a1 - 2.0 * c2;

    const double c3 = 10.0 * A / T3 - 4.0 * B / T2 + 0.5 * C / T;
    const double c4 = -15.0 * A / T4 + 7.0 * B / T3 - C / T2;
    const double c5 = 6.0 * A / T5 - 3.0 * B / T4 + 0.5 * C / T3;

    QuinticSample s;
    s.p = c0 + c1 * t + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5;
    s.v = c1 + 2.0 * c2 * t + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t4;
    s.a = 2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t2 + 20.0 * c5 * t3;
    s.j = 6.0 * c3 + 24.0 * c4 * t + 60.0 * c5 * t2;
    return s;
  }

  bool finiteReferencePose(const quad_midbridge_msgs::msg::ExecutableReference & ref) const
  {
    return std::isfinite(ref.position.x) &&
      std::isfinite(ref.position.y) &&
      std::isfinite(ref.position.z) &&
      std::isfinite(ref.yaw);
  }

  quad_midbridge_msgs::msg::ExecutableReference makeTerminalBridgeStart(
    const quad_midbridge_msgs::msg::ExecutableReference & target) const
  {
    auto start = last_out_;
    if (!start.feasible || !finiteReferencePose(start)) {
      start = target;
      if (have_local_position_ && !localPositionTimedOut()) {
        start.position.x = local_x_;
        start.position.y = local_y_;
        start.position.z = local_z_;
        start.velocity.x = local_vx_;
        start.velocity.y = local_vy_;
        start.velocity.z = local_vz_;
        start.acceleration.x = 0.0;
        start.acceleration.y = 0.0;
        start.acceleration.z = 0.0;
        start.yaw = static_cast<float>(wrapAngle(local_heading_));
        start.yaw_rate = 0.0f;
      }
    }
    return start;
  }

  void resetTerminalBridge()
  {
    terminal_bridge_active_ = false;
    terminal_hold_was_requested_ = false;
    terminal_bridge_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void maybeStartTerminalBridge(const quad_midbridge_msgs::msg::ExecutableReference & target)
  {
    terminal_hold_was_requested_ = true;
    if (!enable_terminal_bridge_ || terminal_bridge_duration_sec_ <= 1.0e-3) {
      terminal_bridge_active_ = false;
      return;
    }

    terminal_bridge_start_ = makeTerminalBridgeStart(target);
    terminal_bridge_target_ = target;
    terminal_bridge_start_time_ = now();

    const double dx = static_cast<double>(terminal_bridge_target_.position.x) -
      static_cast<double>(terminal_bridge_start_.position.x);
    const double dy = static_cast<double>(terminal_bridge_target_.position.y) -
      static_cast<double>(terminal_bridge_start_.position.y);
    const double dz = static_cast<double>(terminal_bridge_target_.position.z) -
      static_cast<double>(terminal_bridge_start_.position.z);
    const double distance = norm3(dx, dy, dz);
    const double speed = norm3(
      terminal_bridge_start_.velocity.x,
      terminal_bridge_start_.velocity.y,
      terminal_bridge_start_.velocity.z);

    if (distance < terminal_bridge_min_distance_m_ && speed < 0.03) {
      terminal_bridge_active_ = false;
      return;
    }

    terminal_bridge_active_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Terminal bridge started: duration=%.2fs distance=%.3fm speed=%.3fm/s",
      terminal_bridge_duration_sec_, distance, speed);
  }

  bool applyTerminalBridge(quad_midbridge_msgs::msg::ExecutableReference & out, uint32_t & flags)
  {
    if (!terminal_bridge_active_) {
      return false;
    }

    const double elapsed = (now() - terminal_bridge_start_time_).seconds();
    if (elapsed >= terminal_bridge_duration_sec_) {
      terminal_bridge_active_ = false;
      return false;
    }

    const auto sx = sampleQuintic(
      terminal_bridge_start_.position.x,
      terminal_bridge_start_.velocity.x,
      terminal_bridge_start_.acceleration.x,
      terminal_bridge_target_.position.x,
      0.0,
      0.0,
      terminal_bridge_duration_sec_,
      elapsed);
    const auto sy = sampleQuintic(
      terminal_bridge_start_.position.y,
      terminal_bridge_start_.velocity.y,
      terminal_bridge_start_.acceleration.y,
      terminal_bridge_target_.position.y,
      0.0,
      0.0,
      terminal_bridge_duration_sec_,
      elapsed);
    const auto sz = sampleQuintic(
      terminal_bridge_start_.position.z,
      terminal_bridge_start_.velocity.z,
      terminal_bridge_start_.acceleration.z,
      terminal_bridge_target_.position.z,
      0.0,
      0.0,
      terminal_bridge_duration_sec_,
      elapsed);

    out.position.x = sx.p;
    out.position.y = sy.p;
    out.position.z = sz.p;
    out.velocity.x = sx.v;
    out.velocity.y = sy.v;
    out.velocity.z = sz.v;
    out.acceleration.x = sx.a;
    out.acceleration.y = sy.a;
    out.acceleration.z = sz.a;
    out.jerk.x = sx.j;
    out.jerk.y = sy.j;
    out.jerk.z = sz.j;

    const double yaw_delta = wrapAngle(
      static_cast<double>(terminal_bridge_target_.yaw) -
      static_cast<double>(terminal_bridge_start_.yaw));
    const auto syaw = sampleQuintic(
      0.0,
      terminal_bridge_start_.yaw_rate,
      0.0,
      yaw_delta,
      0.0,
      0.0,
      terminal_bridge_duration_sec_,
      elapsed);
    out.yaw = static_cast<float>(wrapAngle(static_cast<double>(terminal_bridge_start_.yaw) + syaw.p));
    out.yaw_rate = static_cast<float>(syaw.v);

    flags |= FLAG_TERMINAL_HOLD;
    return true;
  }

  void clampPositionStep(quad_midbridge_msgs::msg::ExecutableReference & out, uint32_t & flags) const
  {
    const double dx = static_cast<double>(out.position.x) - static_cast<double>(last_out_.position.x);
    const double dy = static_cast<double>(out.position.y) - static_cast<double>(last_out_.position.y);
    const double dz = static_cast<double>(out.position.z) - static_cast<double>(last_out_.position.z);
    const double dxy = norm2(dx, dy);
    if (dxy > max_step_xy_) {
      const double s = max_step_xy_ / std::max(dxy, 1e-9);
      out.position.x = static_cast<float>(last_out_.position.x + dx * s);
      out.position.y = static_cast<float>(last_out_.position.y + dy * s);
      flags |= FLAG_POS_CLAMP;
    }
    if (std::abs(dz) > max_step_z_) {
      out.position.z = static_cast<float>(last_out_.position.z + clamp(dz, -max_step_z_, max_step_z_));
      flags |= FLAG_POS_CLAMP;
    }
  }

  void clampVelocity(quad_midbridge_msgs::msg::ExecutableReference & out, uint32_t & flags) const
  {
    const double vxy = norm2(out.velocity.x, out.velocity.y);
    if (vxy > max_vel_xy_) {
      const double s = max_vel_xy_ / std::max(vxy, 1e-9);
      out.velocity.x = static_cast<float>(out.velocity.x * s);
      out.velocity.y = static_cast<float>(out.velocity.y * s);
      flags |= FLAG_VEL_CLAMP;
    }
    if (std::abs(out.velocity.z) > max_vel_z_) {
      out.velocity.z = static_cast<float>(clamp(out.velocity.z, -max_vel_z_, max_vel_z_));
      flags |= FLAG_VEL_CLAMP;
    }
  }

  void clampAccel(quad_midbridge_msgs::msg::ExecutableReference & out, uint32_t & flags) const
  {
    const double axy = norm2(out.acceleration.x, out.acceleration.y);
    if (axy > max_accel_xy_) {
      const double s = max_accel_xy_ / std::max(axy, 1e-9);
      out.acceleration.x = static_cast<float>(out.acceleration.x * s);
      out.acceleration.y = static_cast<float>(out.acceleration.y * s);
      flags |= FLAG_ACC_CLAMP;
    }
    if (std::abs(out.acceleration.z) > max_accel_z_) {
      out.acceleration.z = static_cast<float>(clamp(out.acceleration.z, -max_accel_z_, max_accel_z_));
      flags |= FLAG_ACC_CLAMP;
    }
  }

  void clampJerk(quad_midbridge_msgs::msg::ExecutableReference & out, uint32_t & flags) const
  {
    const double jxy = norm2(out.jerk.x, out.jerk.y);
    if (jxy > max_jerk_xy_) {
      const double s = max_jerk_xy_ / std::max(jxy, 1e-9);
      out.jerk.x = static_cast<float>(out.jerk.x * s);
      out.jerk.y = static_cast<float>(out.jerk.y * s);
      flags |= FLAG_JERK_CLAMP;
    }
    if (std::abs(out.jerk.z) > max_jerk_z_) {
      out.jerk.z = static_cast<float>(clamp(out.jerk.z, -max_jerk_z_, max_jerk_z_));
      flags |= FLAG_JERK_CLAMP;
    }
  }

  void clampYaw(quad_midbridge_msgs::msg::ExecutableReference & out, uint32_t & flags) const
  {
    const double dyaw = wrapAngle(static_cast<double>(out.yaw) - static_cast<double>(last_out_.yaw));
    if (std::abs(dyaw) > max_yaw_step_) {
      out.yaw = static_cast<float>(wrapAngle(static_cast<double>(last_out_.yaw) + clamp(dyaw, -max_yaw_step_, max_yaw_step_)));
      flags |= FLAG_YAW_CLAMP;
    }
    if (std::abs(out.yaw_rate) > max_yaw_rate_) {
      out.yaw_rate = static_cast<float>(clamp(out.yaw_rate, -max_yaw_rate_, max_yaw_rate_));
      flags |= FLAG_YAW_CLAMP;
    }
  }

  void refreshFlatnessDiagnostics(quad_midbridge_msgs::msg::ExecutableReference & out) const
  {
    const double ax = static_cast<double>(out.acceleration.x);
    const double ay = static_cast<double>(out.acceleration.y);
    const double az = static_cast<double>(out.acceleration.z);
    const double vertical = std::max(1.0e-6, kGravity - az);
    const double thrust = norm3(ax, ay, vertical);

    out.thrust_nominal = static_cast<float>(clamp(thrust, min_thrust_nominal_, max_thrust_nominal_));
    out.tilt_nominal = static_cast<float>(std::atan2(norm2(ax, ay), vertical));
  }

  void clampFlatnessEnvelope(quad_midbridge_msgs::msg::ExecutableReference & out, uint32_t & flags) const
  {
    // The flatness mapper recomputes attitude/thrust from acceleration, so the
    // envelope must limit acceleration itself, not only diagnostic fields.
    double ax = static_cast<double>(out.acceleration.x);
    double ay = static_cast<double>(out.acceleration.y);
    double az = static_cast<double>(out.acceleration.z);
    double vertical = std::max(1.0e-6, kGravity - az);
    double axy = norm2(ax, ay);

    const double tilt = std::atan2(axy, vertical);
    if (tilt > max_tilt_nominal_) {
      const double axy_max = std::tan(max_tilt_nominal_) * vertical;
      const double scale = axy_max / std::max(axy, 1.0e-9);
      ax *= scale;
      ay *= scale;
      axy = norm2(ax, ay);
      flags |= FLAG_TILT_CLAMP;
    }

    double thrust = norm3(ax, ay, vertical);
    if (thrust > max_thrust_nominal_) {
      const double scale = max_thrust_nominal_ / std::max(thrust, 1.0e-9);
      ax *= scale;
      ay *= scale;
      vertical *= scale;
      az = kGravity - vertical;
      thrust = norm3(ax, ay, vertical);
      flags |= FLAG_THRUST_CLAMP;
    }

    out.acceleration.x = static_cast<float>(ax);
    out.acceleration.y = static_cast<float>(ay);
    out.acceleration.z = static_cast<float>(az);
    refreshFlatnessDiagnostics(out);
  }

  void applyHeightMode(quad_midbridge_msgs::msg::ExecutableReference & out, uint32_t & flags, bool terminal_hold) const
  {
    (void)flags;
    if (!have_local_position_ || localPositionTimedOut()) {
      return;
    }

    switch (height_mode_) {
      case HeightMode::TakeoffRelative: {
        // Default SITL mode: keep the task height relative to the takeoff lock.
        // This shields flat hover/forward tests from PX4 local-z origin changes.
        const double target_z_hold = takeoff_lock_z0_ - std::abs(takeoff_target_height_m_);
        out.position.z = static_cast<float>(target_z_hold);

        const double err_z_hold = target_z_hold - local_z_;
        const double acc_z_hold = clamp(
          2.4 * err_z_hold - 2.2 * local_vz_ - 0.35,
          takeoff_accel_z_min_, takeoff_accel_z_max_);

        out.velocity.z = 0.0f;
        out.acceleration.z = static_cast<float>(acc_z_hold);
        return;
      }

      case HeightMode::PathZ: {
        // Future 3D/terrain-aware mode: preserve the z supplied by the path/MPCC
        // and add only a bounded vertical correction for PX4 execution robustness.
        // In NED, target_z - local_z < 0 means the vehicle is below the desired
        // altitude and needs upward acceleration.
        const double desired_z = static_cast<double>(out.position.z);
        if (!std::isfinite(desired_z)) {
          return;
        }
        const double err_z = desired_z - local_z_;
        const double correction = clamp(
          path_z_kp_ * err_z - path_z_kd_ * local_vz_,
          path_z_accel_correction_min_, path_z_accel_correction_max_);
        out.acceleration.z = static_cast<float>(static_cast<double>(out.acceleration.z) + correction);
        if (terminal_hold) {
          out.velocity.z = 0.0f;
        }
        return;
      }

      case HeightMode::Passthrough:
        // Debug mode: do not impose additional height semantics. The generic
        // acceleration/thrust/tilt envelopes are still applied later.
        if (terminal_hold) {
          out.velocity.z = 0.0f;
        }
        return;
    }
  }

  void applyTerminalHold(quad_midbridge_msgs::msg::ExecutableReference & out, uint32_t & flags) const
  {
    if (!have_local_position_ || localPositionTimedOut()) {
      return;
    }

    applyHeightMode(out, flags, true);

    out.velocity.x = 0.0f;
    out.velocity.y = 0.0f;
    out.velocity.z = 0.0f;

    const double err_x = static_cast<double>(out.position.x) - local_x_;
    const double err_y = static_cast<double>(out.position.y) - local_y_;

    // Use vector saturation rather than independent x/y clamps so diagonal
    // corrections do not exceed the intended XY acceleration envelope.
    double acc_x = takeoff_kp_xy_ * err_x - takeoff_kd_xy_ * local_vx_;
    double acc_y = takeoff_kp_xy_ * err_y - takeoff_kd_xy_ * local_vy_;
    const double acc_xy = norm2(acc_x, acc_y);
    if (acc_xy > takeoff_accel_xy_max_) {
      const double s = takeoff_accel_xy_max_ / std::max(acc_xy, 1.0e-9);
      acc_x *= s;
      acc_y *= s;
      flags |= FLAG_ACC_CLAMP;
    }

    out.acceleration.x = static_cast<float>(acc_x);
    out.acceleration.y = static_cast<float>(acc_y);

    out.jerk.x = 0.0f;
    out.jerk.y = 0.0f;
    out.jerk.z = 0.0f;

    flags |= FLAG_TERMINAL_HOLD;
  }

  void timerCallback()
  {
    if (!latest_raw_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for /midbridge/raw_reference ...");
      return;
    }

    auto out = *latest_raw_;
    out.header.stamp = now();
    if (out.header.frame_id.empty()) {
      out.header.frame_id = "map";
    }
    uint32_t flags = out.violation_flags;

    const bool raw_timeout = rawRefTimedOut();
    const bool intent_timeout = intentTimedOut();
    const bool local_pos_timeout = localPositionTimedOut();

    const bool using_timeout_hover = raw_timeout && hover_on_timeout_;
    if (using_timeout_hover) {
      out = makeHoverReference(timeout_hover_feasible_);
      flags |= FLAG_RAW_TIMEOUT;
    }
    if (intent_timeout) {
      flags |= FLAG_INTENT_TIMEOUT;
    }
    if (enable_takeoff_gate_ && local_pos_timeout) {
      flags |= FLAG_LOCAL_POS_TIMEOUT;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Takeoff gate enabled but /fmu/out/vehicle_local_position is not valid; suppressing governed reference.");
      out = makeHoverReference(false);
      out.violation_flags = flags;
      out.feasible = false;
      last_out_ = out;
      governed_pub_->publish(out);
      return;
    }

    if (enable_takeoff_gate_ && have_local_position_ && !local_pos_timeout && !takeoff_gate_released_) {
      double height = 0.0;
      if (takeoffReleaseReady(height)) {
        markTakeoffReleased(height);
      }
    }

    if (!raw_timeout && takeoffGateActive()) {
      out = makeTakeoffGateReference(*latest_raw_, flags);
      clampFlatnessEnvelope(out, flags);
    } else {
      const bool terminal_hold_requested = terminalHoldRequested(out);
      if (terminal_hold_requested) {
        if (latest_intent_ && !latest_intent_->centerline_points.empty()) {
          applyHeightMode(out, flags, false);
        }
        applyPostTakeoffBlend(out, flags);

        if (!terminal_hold_was_requested_) {
          maybeStartTerminalBridge(out);
        }
        if (applyTerminalBridge(out, flags)) {
          clampVelocity(out, flags);
          clampAccel(out, flags);
          clampJerk(out, flags);
          clampYaw(out, flags);
        } else {
          clampPositionStep(out, flags);
          clampVelocity(out, flags);
          clampAccel(out, flags);
          clampJerk(out, flags);
          clampYaw(out, flags);
          applyTerminalHold(out, flags);
        }
      } else {
        resetTerminalBridge();
        clampPositionStep(out, flags);
        clampVelocity(out, flags);
        clampAccel(out, flags);
        clampJerk(out, flags);
        clampYaw(out, flags);
        applyPostTakeoffBlend(out, flags);

        if (latest_intent_ && !latest_intent_->centerline_points.empty()) {
          applyHeightMode(out, flags, false);
        }
      }

      // Mode-dependent shaping may have changed acceleration after the generic
      // clamps. Re-apply acceleration and flatness envelopes once at the end so
      // attitude/thrust diagnostics match the actual reference sent downstream.
      clampAccel(out, flags);
      clampFlatnessEnvelope(out, flags);

      out.violation_flags = flags;
      out.feasible = using_timeout_hover ? timeout_hover_feasible_ : (latest_raw_->feasible && !raw_timeout);
    }

    last_out_ = out;
    governed_pub_->publish(out);
  }

  rclcpp::Subscription<quad_midbridge_msgs::msg::ExecutableReference>::SharedPtr raw_ref_sub_;
  rclcpp::Subscription<quad_midbridge_msgs::msg::LocalIntent>::SharedPtr intent_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Publisher<quad_midbridge_msgs::msg::ExecutableReference>::SharedPtr governed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  quad_midbridge_msgs::msg::ExecutableReference::SharedPtr latest_raw_;
  quad_midbridge_msgs::msg::LocalIntent::SharedPtr latest_intent_;
  quad_midbridge_msgs::msg::ExecutableReference last_out_{};
  rclcpp::Time last_raw_ref_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_intent_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_pos_time_{0, 0, RCL_ROS_TIME};

  double publish_rate_hz_{50.0};
  double raw_ref_timeout_sec_{0.3};
  double intent_timeout_sec_{0.6};
  double local_pos_timeout_sec_{0.5};
  double max_step_xy_{0.03};
  double max_step_z_{0.10};
  double max_vel_xy_{0.45};
  double max_vel_z_{0.7};
  double max_accel_xy_{0.9};
  double max_accel_z_{2.0};
  double max_jerk_xy_{1.8};
  double max_jerk_z_{4.0};
  double max_yaw_step_{0.10};
  double max_yaw_rate_{0.3};
  double min_thrust_nominal_{4.5};
  double max_thrust_nominal_{15.0};
  double max_tilt_nominal_{0.35};
  double hover_z_ned_{-1.5};
  double terminal_xy_tol_{0.25};
  double terminal_z_tol_{0.20};
  bool enable_terminal_bridge_{true};
  double terminal_bridge_duration_sec_{4.0};
  double terminal_bridge_min_distance_m_{0.03};
  bool hover_on_timeout_{true};
  bool timeout_hover_feasible_{true};

  HeightMode height_mode_{HeightMode::TakeoffRelative};
  double path_z_kp_{2.0};
  double path_z_kd_{1.5};
  double path_z_accel_correction_min_{-1.2};
  double path_z_accel_correction_max_{1.2};

  bool enable_takeoff_gate_{true};
  double takeoff_target_height_m_{1.0};
  double takeoff_release_height_m_{0.95};
  double takeoff_kp_z_{3.0};
  double takeoff_kd_z_{1.0};
  double takeoff_accel_z_min_{-2.2};
  double takeoff_accel_z_max_{0.3};
  double takeoff_release_vz_abs_max_{0.15};
  double takeoff_release_vxy_abs_max_{0.20};
  double takeoff_release_z_error_abs_max_{0.18};
  double takeoff_release_stable_time_sec_{1.20};
  double takeoff_kp_xy_{0.35};
  double takeoff_kd_xy_{0.45};
  double takeoff_accel_xy_max_{0.25};
  bool takeoff_lock_xy_{true};
  bool takeoff_lock_yaw_{true};
  double post_takeoff_blend_time_sec_{4.0};
  bool takeoff_gate_released_{false};
  rclcpp::Time takeoff_release_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time takeoff_release_candidate_since_{0, 0, RCL_ROS_TIME};

  bool have_local_position_{false};
  double local_x_{0.0};
  double local_y_{0.0};
  double local_z_{0.0};
  double local_vx_{0.0};
  double local_vy_{0.0};
  double local_vz_{0.0};
  double local_heading_{0.0};
  double takeoff_lock_x_{0.0};
  double takeoff_lock_y_{0.0};
  double takeoff_lock_z0_{0.0};
  double takeoff_yaw_{0.0};

  bool terminal_hold_was_requested_{false};
  bool terminal_bridge_active_{false};
  rclcpp::Time terminal_bridge_start_time_{0, 0, RCL_ROS_TIME};
  quad_midbridge_msgs::msg::ExecutableReference terminal_bridge_start_{};
  quad_midbridge_msgs::msg::ExecutableReference terminal_bridge_target_{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GovernorNode>());
  rclcpp::shutdown();
  return 0;
}
