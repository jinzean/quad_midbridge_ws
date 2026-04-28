#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "quad_midbridge_msgs/msg/local_intent.hpp"

class AlignedLocalIntentNode : public rclcpp::Node
{
public:
  AlignedLocalIntentNode()
  : Node("aligned_local_intent_node")
  {
    pub_rate_ = declare_parameter<double>("pub_rate", 5.0);
    target_z_ = declare_parameter<double>("target_z", -1.0);
    path_spacing_ = declare_parameter<double>("path_spacing", 0.5);
    num_points_ = declare_parameter<int>("num_points", 5);

    speed_pref_ = declare_parameter<double>("speed_pref", 0.06);
    speed_max_ = declare_parameter<double>("speed_max", 0.12);

    corridor_half_width_ = declare_parameter<double>("corridor_half_width", 0.8);
    corridor_half_height_ = declare_parameter<double>("corridor_half_height", 0.5);

    risk_level_ = declare_parameter<double>("risk_level", 0.1);
    observation_priority_ = declare_parameter<double>("observation_priority", 0.5);
    progress_weight_ = declare_parameter<double>("progress_weight", 0.5);
    contour_weight_ = declare_parameter<double>("contour_weight", 1.0);

    terminal_hold_ = declare_parameter<bool>("terminal_hold", false);
    allow_reverse_progress_ = declare_parameter<bool>("allow_reverse_progress", false);

    // true: 每次都以当前位姿重新生成局部路径，最适合当前测试，能减少固定路径导致的偏移。
    // false: 只在启动时锁定一次起点，更接近固定路径跟踪。
    reanchor_each_publish_ = declare_parameter<bool>("reanchor_each_publish", true);

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position",
      rclcpp::SensorDataQoS(),
      std::bind(&AlignedLocalIntentNode::localPosCb, this, std::placeholders::_1));

    intent_pub_ = create_publisher<quad_midbridge_msgs::msg::LocalIntent>(
      "/midbridge/local_intent", 10);

    const auto period_ms = static_cast<int>(1000.0 / pub_rate_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&AlignedLocalIntentNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "aligned_local_intent_node started. reanchor_each_publish=%s, speed_pref=%.3f, speed_max=%.3f",
      reanchor_each_publish_ ? "true" : "false",
      speed_pref_,
      speed_max_);
  }

private:
  void localPosCb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    current_x_ = static_cast<double>(msg->x);
    current_y_ = static_cast<double>(msg->y);
    current_z_ = static_cast<double>(msg->z);
    current_heading_ = static_cast<double>(msg->heading);
    last_local_time_ = now();
    have_local_position_ = true;

    if (!anchor_initialized_) {
      anchor_x_ = current_x_;
      anchor_y_ = current_y_;
      anchor_heading_ = std::isfinite(current_heading_) ? current_heading_ : 0.0;
      anchor_initialized_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Local intent anchor initialized at x=%.3f, y=%.3f, heading=%.3f",
        anchor_x_,
        anchor_y_,
        anchor_heading_);
    }
  }

  bool localPositionTimeout() const
  {
    if (!have_local_position_) {
      return true;
    }
    return (now() - last_local_time_).seconds() > 0.5;
  }

  void updateAnchorIfNeeded()
  {
    if (!reanchor_each_publish_) {
      return;
    }

    anchor_x_ = current_x_;
    anchor_y_ = current_y_;
    anchor_heading_ = std::isfinite(current_heading_) ? current_heading_ : anchor_heading_;
  }

  void onTimer()
  {
    if (!have_local_position_ || localPositionTimeout()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for valid /fmu/out/vehicle_local_position...");
      return;
    }

    updateAnchorIfNeeded();

    quad_midbridge_msgs::msg::LocalIntent intent;
    intent.header.stamp = now();
    intent.header.frame_id = "map";

    const double dx = std::cos(anchor_heading_);
    const double dy = std::sin(anchor_heading_);

    const int n = std::max(2, num_points_);
    intent.centerline_points.clear();
    intent.centerline_s.clear();

    for (int i = 0; i < n; ++i) {
      const double s = static_cast<double>(i) * path_spacing_;

      geometry_msgs::msg::Point p;
      p.x = anchor_x_ + s * dx;
      p.y = anchor_y_ + s * dy;
      p.z = target_z_;

      intent.centerline_points.push_back(p);
      intent.centerline_s.push_back(s);
    }

    intent.corridor_half_width = static_cast<float>(corridor_half_width_);
    intent.corridor_half_height = static_cast<float>(corridor_half_height_);

    intent.speed_pref = static_cast<float>(speed_pref_);
    intent.speed_max = static_cast<float>(speed_max_);

    intent.yaw_pref = static_cast<float>(anchor_heading_);
    intent.yaw_rate_pref = 0.0f;

    intent.risk_level = static_cast<float>(risk_level_);
    intent.observation_priority = static_cast<float>(observation_priority_);
    intent.progress_weight = static_cast<float>(progress_weight_);
    intent.contour_weight = static_cast<float>(contour_weight_);

    intent.terminal_hold = terminal_hold_;
    intent.allow_reverse_progress = allow_reverse_progress_;

    intent_pub_->publish(intent);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "publish aligned intent: anchor=(%.2f, %.2f), heading=%.3f, first=(%.2f, %.2f), speed=%.2f",
      anchor_x_,
      anchor_y_,
      anchor_heading_,
      intent.centerline_points.front().x,
      intent.centerline_points.front().y,
      speed_pref_);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Publisher<quad_midbridge_msgs::msg::LocalIntent>::SharedPtr intent_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool have_local_position_{false};
  bool anchor_initialized_{false};

  rclcpp::Time last_local_time_{0, 0, RCL_ROS_TIME};

  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};
  double current_heading_{0.0};

  double anchor_x_{0.0};
  double anchor_y_{0.0};
  double anchor_heading_{0.0};

  double pub_rate_{5.0};
  double target_z_{-1.0};
  double path_spacing_{0.5};
  int num_points_{5};

  double speed_pref_{0.06};
  double speed_max_{0.12};

  double corridor_half_width_{0.8};
  double corridor_half_height_{0.5};

  double risk_level_{0.1};
  double observation_priority_{0.5};
  double progress_weight_{0.5};
  double contour_weight_{1.0};

  bool terminal_hold_{false};
  bool allow_reverse_progress_{false};
  bool reanchor_each_publish_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AlignedLocalIntentNode>());
  rclcpp::shutdown();
  return 0;
}
