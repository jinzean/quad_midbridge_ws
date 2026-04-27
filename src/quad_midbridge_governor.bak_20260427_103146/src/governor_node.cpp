#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "quad_midbridge_msgs/msg/executable_reference.hpp"
#include "quad_midbridge_msgs/msg/local_intent.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;

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

    max_step_xy_ = declare_parameter<double>("max_step_xy", 0.15);
    max_step_z_ = declare_parameter<double>("max_step_z", 0.10);
    max_vel_xy_ = declare_parameter<double>("max_vel_xy", 1.2);
    max_vel_z_ = declare_parameter<double>("max_vel_z", 0.7);
    max_accel_xy_ = declare_parameter<double>("max_accel_xy", 3.0);
    max_accel_z_ = declare_parameter<double>("max_accel_z", 2.0);
    max_jerk_xy_ = declare_parameter<double>("max_jerk_xy", 6.0);
    max_jerk_z_ = declare_parameter<double>("max_jerk_z", 4.0);
    max_yaw_step_ = declare_parameter<double>("max_yaw_step", 0.10);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.8);

    min_thrust_nominal_ = declare_parameter<double>("min_thrust_nominal", 4.5);
    max_thrust_nominal_ = declare_parameter<double>("max_thrust_nominal", 15.0);
    max_tilt_nominal_ = declare_parameter<double>("max_tilt_nominal", 0.60);

    hover_z_ned_ = declare_parameter<double>("hover_z_ned", -1.5);
    terminal_xy_tol_ = declare_parameter<double>("terminal_xy_tol", 0.25);
    terminal_z_tol_ = declare_parameter<double>("terminal_z_tol", 0.20);
    hover_on_timeout_ = declare_parameter<bool>("hover_on_timeout", true);

    raw_ref_sub_ = create_subscription<quad_midbridge_msgs::msg::ExecutableReference>(
      "/midbridge/raw_reference", 10,
      std::bind(&GovernorNode::rawRefCallback, this, std::placeholders::_1));

    intent_sub_ = create_subscription<quad_midbridge_msgs::msg::LocalIntent>(
      "/midbridge/local_intent", 10,
      std::bind(&GovernorNode::intentCallback, this, std::placeholders::_1));

    governed_pub_ = create_publisher<quad_midbridge_msgs::msg::ExecutableReference>(
      "/midbridge/governed_reference", 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&GovernorNode::timerCallback, this));

    last_out_.position.z = hover_z_ned_;
    last_out_.feasible = false;
    RCLCPP_INFO(get_logger(), "governor_node v2 executable-envelope mode started.");
  }

private:
  static constexpr uint32_t FLAG_RAW_TIMEOUT = 1u << 8;
  static constexpr uint32_t FLAG_INTENT_TIMEOUT = 1u << 9;
  static constexpr uint32_t FLAG_POS_CLAMP = 1u << 10;
  static constexpr uint32_t FLAG_VEL_CLAMP = 1u << 11;
  static constexpr uint32_t FLAG_ACC_CLAMP = 1u << 12;
  static constexpr uint32_t FLAG_JERK_CLAMP = 1u << 13;
  static constexpr uint32_t FLAG_YAW_CLAMP = 1u << 14;
  static constexpr uint32_t FLAG_THRUST_CLAMP = 1u << 15;
  static constexpr uint32_t FLAG_TILT_CLAMP = 1u << 16;
  static constexpr uint32_t FLAG_TERMINAL_HOLD = 1u << 17;

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

  quad_midbridge_msgs::msg::ExecutableReference makeHoverReference() const
  {
    quad_midbridge_msgs::msg::ExecutableReference out;
    out.header.stamp = now();
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
    uint32_t flags = out.violation_flags;

    const bool raw_timeout = rawRefTimedOut();
    const bool intent_timeout = intentTimedOut();
    if (raw_timeout && hover_on_timeout_) {
      out = makeHoverReference();
      flags |= FLAG_RAW_TIMEOUT;
    }
    if (intent_timeout) {
      flags |= FLAG_INTENT_TIMEOUT;
    }

    clampPositionStep(out, flags);
    clampVelocity(out, flags);
    clampAccel(out, flags);
    clampJerk(out, flags);
    clampYaw(out, flags);
    clampFlatnessEnvelope(out, flags);

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

    out.feasible = (flags == 0u) && latest_raw_->feasible && !raw_timeout;
    out.violation_flags = flags;
    last_out_ = out;
    governed_pub_->publish(out);
  }

  rclcpp::Subscription<quad_midbridge_msgs::msg::ExecutableReference>::SharedPtr raw_ref_sub_;
  rclcpp::Subscription<quad_midbridge_msgs::msg::LocalIntent>::SharedPtr intent_sub_;
  rclcpp::Publisher<quad_midbridge_msgs::msg::ExecutableReference>::SharedPtr governed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  quad_midbridge_msgs::msg::ExecutableReference::SharedPtr latest_raw_;
  quad_midbridge_msgs::msg::LocalIntent::SharedPtr latest_intent_;
  quad_midbridge_msgs::msg::ExecutableReference last_out_{};
  rclcpp::Time last_raw_ref_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_intent_time_{0, 0, RCL_ROS_TIME};

  double publish_rate_hz_{50.0};
  double raw_ref_timeout_sec_{0.3};
  double intent_timeout_sec_{0.6};
  double max_step_xy_{0.15};
  double max_step_z_{0.10};
  double max_vel_xy_{1.2};
  double max_vel_z_{0.7};
  double max_accel_xy_{3.0};
  double max_accel_z_{2.0};
  double max_jerk_xy_{6.0};
  double max_jerk_z_{4.0};
  double max_yaw_step_{0.10};
  double max_yaw_rate_{0.8};
  double min_thrust_nominal_{4.5};
  double max_thrust_nominal_{15.0};
  double max_tilt_nominal_{0.60};
  double hover_z_ned_{-1.5};
  double terminal_xy_tol_{0.25};
  double terminal_z_tol_{0.20};
  bool hover_on_timeout_{true};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GovernorNode>());
  rclcpp::shutdown();
  return 0;
}
