#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "quad_midbridge_msgs/msg/attitude_thrust_reference.hpp"
#include "rclcpp/rclcpp.hpp"

class AttitudeOffboardNode : public rclcpp::Node
{
public:
  AttitudeOffboardNode() : Node("attitude_offboard_node")
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 190.0);
    ref_timeout_sec_ = declare_parameter<double>("ref_timeout_sec", 0.5);
    hard_timeout_sec_ = declare_parameter<double>("hard_timeout_sec", 5.0);
    warmup_cycles_before_arm_ = declare_parameter<int>("warmup_cycles_before_arm", 40);
    auto_arm_ = declare_parameter<bool>("auto_arm", false);
    auto_set_mode_ = declare_parameter<bool>("auto_set_mode", false);
    require_valid_reference_ = declare_parameter<bool>("require_valid_reference", true);
    hold_last_valid_on_timeout_ = declare_parameter<bool>("hold_last_valid_on_timeout", true);
    publish_control_mode_without_reference_ =
      declare_parameter<bool>("publish_control_mode_without_reference", false);

    control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    attitude_sp_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      "/fmu/in/vehicle_attitude_setpoint", 10);
    vehicle_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

    ref_sub_ = create_subscription<quad_midbridge_msgs::msg::AttitudeThrustReference>(
      "/midbridge/attitude_thrust_reference", 10,
      std::bind(&AttitudeOffboardNode::refCallback, this, std::placeholders::_1));

    running_.store(true);
    publish_thread_ = std::thread(&AttitudeOffboardNode::publishLoop, this);

    RCLCPP_INFO(
      get_logger(),
      "attitude_offboard_node started. publish_rate=%.1f Hz, ref_timeout=%.2f s, hard_timeout=%.2f s, hold_last_valid=%s, auto_set_mode=%s",
      publish_rate_hz_,
      ref_timeout_sec_,
      hard_timeout_sec_,
      hold_last_valid_on_timeout_ ? "true" : "false",
      auto_set_mode_ ? "true" : "false");
  }

  ~AttitudeOffboardNode() override
  {
    running_.store(false);
    if (publish_thread_.joinable()) {
      publish_thread_.join();
    }
  }

private:
  void refCallback(const quad_midbridge_msgs::msg::AttitudeThrustReference::SharedPtr msg)
  {
    if ((require_valid_reference_ && !msg->valid) || !finiteReference(*msg)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received invalid or non-finite AttitudeThrustReference; keeping last valid setpoint.");
      return;
    }

    std::lock_guard<std::mutex> lock(ref_mutex_);
    cached_ref_ = *msg;
    have_cached_ref_ = true;
    last_ref_steady_time_ = std::chrono::steady_clock::now();
  }

  static bool finiteReference(const quad_midbridge_msgs::msg::AttitudeThrustReference & ref)
  {
    return std::isfinite(ref.attitude.w) &&
           std::isfinite(ref.attitude.x) &&
           std::isfinite(ref.attitude.y) &&
           std::isfinite(ref.attitude.z) &&
           std::isfinite(ref.bodyrates.x) &&
           std::isfinite(ref.bodyrates.y) &&
           std::isfinite(ref.bodyrates.z) &&
           std::isfinite(ref.collective_thrust) &&
           std::isfinite(ref.yaw);
  }

  static std::array<float, 4> normalizedQuat(
    const quad_midbridge_msgs::msg::AttitudeThrustReference & ref)
  {
    const double w = static_cast<double>(ref.attitude.w);
    const double x = static_cast<double>(ref.attitude.x);
    const double y = static_cast<double>(ref.attitude.y);
    const double z = static_cast<double>(ref.attitude.z);

    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n < 1.0e-8 || !std::isfinite(n)) {
      return {1.0f, 0.0f, 0.0f, 0.0f};
    }

    return {
      static_cast<float>(w / n),
      static_cast<float>(x / n),
      static_cast<float>(y / n),
      static_cast<float>(z / n)};
  }

  void publishLoop()
  {
    const double rate = std::max(publish_rate_hz_, 1.0);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate));

    auto next_time = std::chrono::steady_clock::now() + period;

    while (running_.load() && rclcpp::ok()) {
      std::this_thread::sleep_until(next_time);
      next_time += period;

      quad_midbridge_msgs::msg::AttitudeThrustReference ref{};
      bool have_ref = false;
      double ref_age = 1.0e9;

      {
        std::lock_guard<std::mutex> lock(ref_mutex_);
        have_ref = have_cached_ref_;
        if (have_cached_ref_) {
          ref = cached_ref_;
          ref_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_ref_steady_time_).count();
        }
      }

      if (!have_ref) {
        if (publish_control_mode_without_reference_) {
          publishOffboardControlMode();
        }
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for first valid AttitudeThrustReference...");
        continue;
      }

      if (ref_age > ref_timeout_sec_) {
        if (!hold_last_valid_on_timeout_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "AttitudeThrustReference timed out age=%.3fs; not publishing setpoint because hold_last_valid_on_timeout=false.",
            ref_age);
          continue;
        }

        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "AttitudeThrustReference stale age=%.3fs; continuing to stream last valid PX4 setpoint.",
          ref_age);
      }

      if (ref_age > hard_timeout_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "AttitudeThrustReference hard timeout age=%.3fs; still streaming last valid setpoint to preserve PX4 Offboard proof-of-life.",
          ref_age);
      }

      publishOffboardControlMode();
      publishVehicleAttitudeSetpoint(ref);

      if (!armed_and_offboard_sent_) {
        warmup_counter_++;

        if (warmup_counter_ >= warmup_cycles_before_arm_) {
          if (auto_set_mode_) {
            publishVehicleCommand(
              px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
              1.0f,
              6.0f);
          }

          if (auto_arm_) {
            publishVehicleCommand(
              px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
              1.0f,
              0.0f);
          }

          armed_and_offboard_sent_ = true;
        }
      }
    }
  }

  void publishOffboardControlMode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = true;
    msg.body_rate = false;
    msg.actuator = false;
    control_mode_pub_->publish(msg);
  }

  void publishVehicleAttitudeSetpoint(
    const quad_midbridge_msgs::msg::AttitudeThrustReference & ref)
  {
    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.yaw_body = ref.yaw;
    msg.yaw_sp_move_rate = ref.bodyrates.z;
    msg.q_d = normalizedQuat(ref);

    const float thrust = std::clamp(ref.collective_thrust, 0.0f, 1.0f);
    msg.thrust_body = {0.0f, 0.0f, -thrust};

    msg.reset_integral = false;
    msg.fw_control_yaw_wheel = false;
    attitude_sp_pub_->publish(msg);
  }

  void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_cmd_pub_->publish(msg);
  }

  rclcpp::Subscription<quad_midbridge_msgs::msg::AttitudeThrustReference>::SharedPtr ref_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_sp_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

  std::mutex ref_mutex_;
  quad_midbridge_msgs::msg::AttitudeThrustReference cached_ref_{};
  bool have_cached_ref_{false};
  std::chrono::steady_clock::time_point last_ref_steady_time_{std::chrono::steady_clock::now()};

  std::atomic_bool running_{false};
  std::thread publish_thread_;

  double publish_rate_hz_{190.0};
  double ref_timeout_sec_{0.5};
  double hard_timeout_sec_{5.0};
  int warmup_cycles_before_arm_{40};
  int warmup_counter_{0};

  bool auto_arm_{false};
  bool auto_set_mode_{false};
  bool require_valid_reference_{true};
  bool hold_last_valid_on_timeout_{true};
  bool publish_control_mode_without_reference_{false};
  bool armed_and_offboard_sent_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AttitudeOffboardNode>());
  rclcpp::shutdown();
  return 0;
}
