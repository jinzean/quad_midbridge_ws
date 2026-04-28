#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#include "quad_midbridge_msgs/msg/attitude_thrust_reference.hpp"
#include "quad_midbridge_msgs/msg/executable_reference.hpp"
#include "rclcpp/rclcpp.hpp"

class FlatnessMapperNode : public rclcpp::Node
{
public:
  FlatnessMapperNode() : Node("flatness_mapper_node")
  {
    gravity_ = declare_parameter<double>("gravity", 9.81);
    thrust_scale_ = declare_parameter<double>("thrust_scale", 15.0);
    max_collective_thrust_ = declare_parameter<double>("max_collective_thrust", 1.0);

    declare_parameter<std::string>("input_reference_topic", "/midbridge/governed_reference");
    const auto input_reference_topic =
      get_parameter("input_reference_topic").as_string();

    ref_sub_ = create_subscription<quad_midbridge_msgs::msg::ExecutableReference>(
      input_reference_topic, 10,
      std::bind(&FlatnessMapperNode::refCallback, this, std::placeholders::_1));

    att_pub_ = create_publisher<quad_midbridge_msgs::msg::AttitudeThrustReference>(
      "/midbridge/attitude_thrust_reference", 10);

    RCLCPP_INFO(get_logger(), "flatness_mapper_node started.");
  }

private:
  static std::array<double, 3> normalize(const std::array<double, 3> & v)
  {
    const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n < 1e-8) {
      return {0.0, 0.0, 1.0};
    }
    return {v[0] / n, v[1] / n, v[2] / n};
  }

  static std::array<double, 3> cross(const std::array<double, 3> & a, const std::array<double, 3> & b)
  {
    return {
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0]
    };
  }

  static std::array<double, 4> rotmatToQuat(
    double r00, double r01, double r02,
    double r10, double r11, double r12,
    double r20, double r21, double r22)
  {
    std::array<double, 4> q{};
    const double tr = r00 + r11 + r22;
    if (tr > 0.0) {
      const double s = std::sqrt(tr + 1.0) * 2.0;
      q[0] = 0.25 * s;
      q[1] = (r21 - r12) / s;
      q[2] = (r02 - r20) / s;
      q[3] = (r10 - r01) / s;
    } else if ((r00 > r11) && (r00 > r22)) {
      const double s = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
      q[0] = (r21 - r12) / s;
      q[1] = 0.25 * s;
      q[2] = (r01 + r10) / s;
      q[3] = (r02 + r20) / s;
    } else if (r11 > r22) {
      const double s = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
      q[0] = (r02 - r20) / s;
      q[1] = (r01 + r10) / s;
      q[2] = 0.25 * s;
      q[3] = (r12 + r21) / s;
    } else {
      const double s = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
      q[0] = (r10 - r01) / s;
      q[1] = (r02 + r20) / s;
      q[2] = (r12 + r21) / s;
      q[3] = 0.25 * s;
    }
    return q;
  }

  void refCallback(const quad_midbridge_msgs::msg::ExecutableReference::SharedPtr msg)
  {
    quad_midbridge_msgs::msg::AttitudeThrustReference out;
    out.header = msg->header;
    out.yaw = msg->yaw;
    out.mode = msg->mode;
    out.from_governor = true;
    out.valid = msg->feasible;

    // NED/FRD mapping for PX4 attitude-thrust setpoints.
    // PX4 local position uses NED. For multirotor dynamics:
    //   a = g * e3 - (T/m) * b3
    // therefore (T/m) * b3 = g * e3 - a.
    // Negative acceleration.z is upward acceleration and must increase thrust.
    const std::array<double, 3> fd = {
      -static_cast<double>(msg->acceleration.x),
      -static_cast<double>(msg->acceleration.y),
      gravity_ - static_cast<double>(msg->acceleration.z)
    };
    const auto b3 = normalize(fd);

    const std::array<double, 3> xc = {
      std::cos(static_cast<double>(msg->yaw)),
      std::sin(static_cast<double>(msg->yaw)),
      0.0
    };
    auto b2 = normalize(cross(b3, xc));
    auto b1 = normalize(cross(b2, b3));

    const auto q = rotmatToQuat(
      b1[0], b2[0], b3[0],
      b1[1], b2[1], b3[1],
      b1[2], b2[2], b3[2]);

    out.attitude.w = static_cast<float>(q[0]);
    out.attitude.x = static_cast<float>(q[1]);
    out.attitude.y = static_cast<float>(q[2]);
    out.attitude.z = static_cast<float>(q[3]);

    out.bodyrates.x = 0.0f;
    out.bodyrates.y = 0.0f;
    out.bodyrates.z = msg->yaw_rate;

    const double thrust = std::clamp(std::sqrt(fd[0] * fd[0] + fd[1] * fd[1] + fd[2] * fd[2]) / thrust_scale_, 0.0, max_collective_thrust_);
    out.collective_thrust = static_cast<float>(thrust);

    att_pub_->publish(out);
  }

  rclcpp::Subscription<quad_midbridge_msgs::msg::ExecutableReference>::SharedPtr ref_sub_;
  rclcpp::Publisher<quad_midbridge_msgs::msg::AttitudeThrustReference>::SharedPtr att_pub_;

  double gravity_{9.81};
  double thrust_scale_{15.0};
  double max_collective_thrust_{1.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlatnessMapperNode>());
  rclcpp::shutdown();
  return 0;
}
