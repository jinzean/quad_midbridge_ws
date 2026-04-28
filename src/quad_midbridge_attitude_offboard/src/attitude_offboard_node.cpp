#include <algorithm>
#include <chrono>
#include <memory>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "quad_midbridge_msgs/msg/attitude_thrust_reference.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class AttitudeOffboardNode : public rclcpp::Node
{
public:
  AttitudeOffboardNode() : Node("attitude_offboard_node")
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    ref_timeout_sec_ = declare_parameter<double>("ref_timeout_sec", 0.5);
    warmup_cycles_before_arm_ = declare_parameter<int>("warmup_cycles_before_arm", 20);
    auto_arm_ = declare_parameter<bool>("auto_arm", false);
    auto_set_mode_ = declare_parameter<bool>("auto_set_mode", false);
    require_valid_reference_ = declare_parameter<bool>("require_valid_reference", true);

    control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    attitude_sp_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      "/fmu/in/vehicle_attitude_setpoint", 10);
    vehicle_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

    ref_sub_ = create_subscription<quad_midbridge_msgs::msg::AttitudeThrustReference>(
      "/midbridge/attitude_thrust_reference", 10,
      std::bind(&AttitudeOffboardNode::refCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&AttitudeOffboardNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "attitude_offboard_node started.");
  }

private:
  void refCallback(const quad_midbridge_msgs::msg::AttitudeThrustReference::SharedPtr msg)
  {
    latest_ref_ = msg;
    last_ref_time_ = now();
  }

  bool refTimedOut() const
  {
    if (!latest_ref_) {
      return true;
    }
    return (now() - last_ref_time_).seconds() > ref_timeout_sec_;
  }

  void timerCallback()
  {
    if (!latest_ref_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for AttitudeThrustReference...");
      return;
    }
    if (refTimedOut()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "AttitudeThrustReference timed out; not publishing attitude setpoint.");
      warmup_counter_ = 0;
      return;
    }

    if (require_valid_reference_ && !latest_ref_->valid) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "AttitudeThrustReference invalid; not publishing attitude setpoint.");
      warmup_counter_ = 0;
      return;
    }

    publishOffboardControlMode();
    publishVehicleAttitudeSetpoint();

    if (!armed_and_offboard_sent_) {
      warmup_counter_++;
      if (warmup_counter_ >= warmup_cycles_before_arm_) {
        if (auto_set_mode_) {
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        }
        if (auto_arm_) {
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        }
        armed_and_offboard_sent_ = true;
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

  void publishVehicleAttitudeSetpoint()
  {
    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    msg.yaw_body = latest_ref_->yaw;
    msg.yaw_sp_move_rate = latest_ref_->bodyrates.z;
    msg.q_d = {
      static_cast<float>(latest_ref_->attitude.w),
      static_cast<float>(latest_ref_->attitude.x),
      static_cast<float>(latest_ref_->attitude.y),
      static_cast<float>(latest_ref_->attitude.z)
    };
    const float thrust = std::clamp(latest_ref_->collective_thrust, 0.0f, 1.0f);
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
  rclcpp::TimerBase::SharedPtr timer_;

  quad_midbridge_msgs::msg::AttitudeThrustReference::SharedPtr latest_ref_;
  rclcpp::Time last_ref_time_{0, 0, RCL_ROS_TIME};

  double publish_rate_hz_{100.0};
  double ref_timeout_sec_{0.5};
  int warmup_cycles_before_arm_{20};
  int warmup_counter_{0};
  bool auto_arm_{false};
  bool auto_set_mode_{false};
  bool require_valid_reference_{true};
  bool armed_and_offboard_sent_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AttitudeOffboardNode>());
  rclcpp::shutdown();
  return 0;
}
