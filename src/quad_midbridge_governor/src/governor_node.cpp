#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
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
    hover_on_timeout_ = declare_parameter<bool>("hover_on_timeout", true);

    enable_takeoff_gate_ = declare_parameter<bool>("enable_takeoff_gate", true);
    takeoff_target_height_m_ = declare_parameter<double>("takeoff_target_height_m", 1.0);
    takeoff_release_height_m_ = declare_parameter<double>("takeoff_release_height_m", 0.95);
    takeoff_kp_z_ = declare_parameter<double>("takeoff_kp_z", 3.0);
    takeoff_kd_z_ = declare_parameter<double>("takeoff_kd_z", 1.0);
    takeoff_accel_z_min_ = declare_parameter<double>("takeoff_accel_z_min", -2.2);  // upward in NED
    takeoff_accel_z_max_ = declare_parameter<double>("takeoff_accel_z_max", 0.3);   // downward in NED
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
    RCLCPP_INFO(get_logger(), "governor_node started with safety envelope and takeoff gate.");
  }

private:
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
    local_x_ = static_cast<double>(msg->x);
    local_y_ = static_cast<double>(msg->y);
    local_z_ = static_cast<double>(msg->z);
    local_vz_ = static_cast<double>(msg->vz);
    local_heading_ = static_cast<double>(msg->heading);
    last_local_pos_time_ = now();

    const bool valid = msg->xy_valid && msg->z_valid && msg->v_z_valid &&
                       std::isfinite(local_x_) && std::isfinite(local_y_) &&
                       std::isfinite(local_z_) && std::isfinite(local_vz_);
    if (!valid) {
      return;
    }

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

    const double current_height = std::max(0.0, -local_z_);
    takeoff_gate_released_ = current_height >= takeoff_release_height_m_;

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
    const double height = std::max(0.0, -local_z_);
    return height < takeoff_release_height_m_;
  }

  quad_midbridge_msgs::msg::ExecutableReference makeHoverReference() const
  {
    quad_midbridge_msgs::msg::ExecutableReference out;
    out.header.stamp = now();
    out.header.frame_id = "map";
    out.position = last_out_.position;
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
    out.thrust_nominal = static_cast<float>(clamp(last_out_.thrust_nominal, min_thrust_nominal_, max_thrust_nominal_));
    out.tilt_nominal = static_cast<float>(clamp(last_out_.tilt_nominal, 0.0, max_tilt_nominal_));
    out.mode = last_out_.mode;
    out.feasible = false;
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

    const double target_z = -std::abs(takeoff_target_height_m_);
    const double err_z = target_z - local_z_;
    const double acc_z = clamp(
      takeoff_kp_z_ * err_z - takeoff_kd_z_ * local_vz_,
      takeoff_accel_z_min_, takeoff_accel_z_max_);

    out.position.x = static_cast<float>(takeoff_lock_xy_ ? takeoff_lock_x_ : local_x_);
    out.position.y = static_cast<float>(takeoff_lock_xy_ ? takeoff_lock_y_ : local_y_);
    out.position.z = static_cast<float>(target_z);

    out.velocity.x = 0.0f;
    out.velocity.y = 0.0f;
    out.velocity.z = 0.0f;

    out.acceleration.x = 0.0f;
    out.acceleration.y = 0.0f;
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
    out.thrust_nominal = static_cast<float>(clamp(kGravity - acc_z, min_thrust_nominal_, max_thrust_nominal_));
    out.tilt_nominal = 0.0f;
    out.mode = raw.mode;
    out.feasible = raw.feasible;
    flags |= FLAG_TAKEOFF_GATE;
    out.violation_flags = flags;

    const double height = std::max(0.0, -local_z_);
    if (height >= takeoff_release_height_m_) {
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

  bool terminalHoldRequested(const quad_midbridge_msgs::msg::ExecutableReference & out) const
  {
    if (!latest_intent_ || !latest_intent_->terminal_hold || latest_intent_->centerline_points.empty()) {
      return false;
    }
    const auto & goal = latest_intent_->centerline_points.back();
    const double dx = static_cast<double>(out.position.x) - static_cast<double>(goal.x);
    const double dy = static_cast<double>(out.position.y) - static_cast<double>(goal.y);
    const double dz = static_cast<double>(out.position.z) - static_cast<double>(goal.z);
    return norm2(dx, dy) <= terminal_xy_tol_ && std::abs(dz) <= terminal_z_tol_;
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

  void clampFlatnessEnvelope(quad_midbridge_msgs::msg::ExecutableReference & out, uint32_t & flags) const
  {
    if (out.thrust_nominal < min_thrust_nominal_ || out.thrust_nominal > max_thrust_nominal_) {
      out.thrust_nominal = static_cast<float>(clamp(out.thrust_nominal, min_thrust_nominal_, max_thrust_nominal_));
      flags |= FLAG_THRUST_CLAMP;
    }
    if (out.tilt_nominal > max_tilt_nominal_) {
      out.tilt_nominal = static_cast<float>(max_tilt_nominal_);
      flags |= FLAG_TILT_CLAMP;
    }
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

    if (raw_timeout && hover_on_timeout_) {
      out = makeHoverReference();
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
      out = makeHoverReference();
      out.violation_flags = flags;
      out.feasible = false;
      last_out_ = out;
      governed_pub_->publish(out);
      return;
    }

    if (enable_takeoff_gate_ && have_local_position_ && !local_pos_timeout && !takeoff_gate_released_) {
      const double height = std::max(0.0, -local_z_);
      if (height >= takeoff_release_height_m_) {
        takeoff_gate_released_ = true;
        RCLCPP_INFO(get_logger(), "Takeoff gate released at height %.2f m", height);
      }
    }

    if (!raw_timeout && takeoffGateActive()) {
      out = makeTakeoffGateReference(*latest_raw_, flags);
    } else {
      clampPositionStep(out, flags);
      clampVelocity(out, flags);
      clampAccel(out, flags);
      clampJerk(out, flags);
      clampYaw(out, flags);
      clampFlatnessEnvelope(out, flags);
      applyPostTakeoffBlend(out, flags);

      if (terminalHoldRequested(out)) {
        out.velocity.x = 0.0f;
        out.velocity.y = 0.0f;
        out.velocity.z = 0.0f;
        out.acceleration.x = 0.0f;
        out.acceleration.y = 0.0f;
        out.acceleration.z = 0.0f;
        out.jerk.x = 0.0f;
        out.jerk.y = 0.0f;
        out.jerk.z = 0.0f;
        flags |= FLAG_TERMINAL_HOLD;
      }
      out.violation_flags = flags;
      out.feasible = latest_raw_->feasible && !raw_timeout;
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
  bool hover_on_timeout_{true};

  bool enable_takeoff_gate_{true};
  double takeoff_target_height_m_{1.0};
  double takeoff_release_height_m_{0.95};
  double takeoff_kp_z_{3.0};
  double takeoff_kd_z_{1.0};
  double takeoff_accel_z_min_{-2.2};
  double takeoff_accel_z_max_{0.3};
  bool takeoff_lock_xy_{true};
  bool takeoff_lock_yaw_{true};
  double post_takeoff_blend_time_sec_{4.0};
  bool takeoff_gate_released_{false};
  rclcpp::Time takeoff_release_time_{0, 0, RCL_ROS_TIME};

  bool have_local_position_{false};
  double local_x_{0.0};
  double local_y_{0.0};
  double local_z_{0.0};
  double local_vz_{0.0};
  double local_heading_{0.0};
  double takeoff_lock_x_{0.0};
  double takeoff_lock_y_{0.0};
  double takeoff_lock_z0_{0.0};
  double takeoff_yaw_{0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GovernorNode>());
  rclcpp::shutdown();
  return 0;
}
