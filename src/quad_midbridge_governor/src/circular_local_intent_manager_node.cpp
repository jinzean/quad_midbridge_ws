#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "quad_midbridge_msgs/msg/local_intent.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

inline double clamp01(double x)
{
  return std::max(0.0, std::min(x, 1.0));
}

inline double wrapAngle(double a)
{
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

inline bool finite2(double x, double y)
{
  return std::isfinite(x) && std::isfinite(y);
}
}  // namespace

class CircularLocalIntentManagerNode : public rclcpp::Node
{
public:
  CircularLocalIntentManagerNode()
  : Node("circular_local_intent_manager_node")
  {
    pub_rate_ = declare_parameter<double>("pub_rate", 10.0);
    local_pos_timeout_sec_ = declare_parameter<double>("local_pos_timeout_sec", 0.5);

    target_z_ = declare_parameter<double>("target_z", -1.5);
    radius_m_ = declare_parameter<double>("radius_m", 2.0);
    path_spacing_ = declare_parameter<double>("path_spacing", 0.35);
    arc_num_points_ = declare_parameter<int>("arc_num_points", 12);
    final_hold_num_points_ = declare_parameter<int>("final_hold_num_points", 4);
    orbit_ccw_ = declare_parameter<bool>("orbit_ccw", true);

    hover_duration_sec_ = declare_parameter<double>("hover_duration_sec", 30.0);
    orbit_duration_sec_ = declare_parameter<double>("orbit_duration_sec", 80.0);
    decel_duration_sec_ = declare_parameter<double>("decel_duration_sec", 8.0);

    orbit_speed_pref_ = declare_parameter<double>("orbit_speed_pref", 0.16);
    orbit_speed_max_ = declare_parameter<double>("orbit_speed_max", 0.25);
    orbit_progress_weight_ = declare_parameter<double>("orbit_progress_weight", 0.22);
    contour_weight_ = declare_parameter<double>("contour_weight", 0.8);
    corridor_half_width_ = declare_parameter<double>("corridor_half_width", 1.0);
    corridor_half_height_ = declare_parameter<double>("corridor_half_height", 1.5);
    risk_level_ = declare_parameter<double>("risk_level", 0.1);
    observation_priority_ = declare_parameter<double>("observation_priority", 0.5);
    allow_reverse_progress_ = declare_parameter<bool>("allow_reverse_progress", false);

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
      std::bind(&CircularLocalIntentManagerNode::localPosCb, this, std::placeholders::_1));

    intent_pub_ = create_publisher<quad_midbridge_msgs::msg::LocalIntent>(
      "/midbridge/local_intent", 10);

    const auto period_ms = static_cast<int>(1000.0 / std::max(pub_rate_, 1.0));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(period_ms, 1)),
      std::bind(&CircularLocalIntentManagerNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "circular_local_intent_manager_node started: hover %.1fs, orbit %.1fs, radius %.2fm, speed %.3f",
      hover_duration_sec_, orbit_duration_sec_, radius_m_, orbit_speed_pref_);
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  enum class Stage
  {
    Hover,
    Orbit,
    Decelerate,
    FinalHold
  };

  void localPosCb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    const double x = static_cast<double>(msg->x);
    const double y = static_cast<double>(msg->y);
    const double z = static_cast<double>(msg->z);
    const double heading = static_cast<double>(msg->heading);

    const bool valid = msg->xy_valid && msg->z_valid &&
                       std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    if (!valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring invalid VehicleLocalPosition in circular intent manager: xy_valid=%s z_valid=%s",
        msg->xy_valid ? "true" : "false",
        msg->z_valid ? "true" : "false");
      return;
    }

    current_x_ = x;
    current_y_ = y;
    current_z_ = z;
    if (std::isfinite(heading)) {
      current_heading_ = heading;
    }
    last_local_steady_time_ = SteadyClock::now();
    last_local_time_ = now();
    have_local_position_ = true;

    if (!mission_started_) {
      mission_started_ = true;
      mission_start_steady_time_ = last_local_steady_time_;
      mission_start_time_ = now();
      latchCircleFromCurrentPose();
      latchFinalHoldAtCurrentPose();
      RCLCPP_INFO(
        get_logger(),
        "Circle mission started. center=(%.3f, %.3f), radius=%.2f, heading=%.3f, direction=%s",
        circle_center_x_, circle_center_y_, radius_m_, current_heading_, orbit_ccw_ ? "ccw" : "cw");
    }
  }

  bool localPositionTimeout() const
  {
    if (!have_local_position_) {
      return true;
    }
    const auto age = std::chrono::duration<double>(
      SteadyClock::now() - last_local_steady_time_).count();
    return age > local_pos_timeout_sec_;
  }

  const char * stageName(Stage stage) const
  {
    switch (stage) {
      case Stage::Hover: return "hover";
      case Stage::Orbit: return "orbit";
      case Stage::Decelerate: return "decelerate";
      case Stage::FinalHold: return "final_hold";
    }
    return "unknown";
  }

  Stage desiredStage(double elapsed) const
  {
    const double hover_end = std::max(0.0, hover_duration_sec_);
    const double orbit_end = hover_end + std::max(0.0, orbit_duration_sec_);
    const double decel_end = orbit_end + std::max(0.0, decel_duration_sec_);

    if (elapsed < hover_end) {
      return Stage::Hover;
    }
    if (elapsed < orbit_end) {
      return Stage::Orbit;
    }
    if (elapsed < decel_end) {
      return Stage::Decelerate;
    }
    return Stage::FinalHold;
  }

  void updateStage(double elapsed)
  {
    const Stage next_stage = desiredStage(elapsed);
    if (next_stage == current_stage_) {
      return;
    }

    const Stage previous_stage = current_stage_;
    current_stage_ = next_stage;

    if (current_stage_ == Stage::Orbit && !circle_latched_) {
      latchCircleFromCurrentPose();
    }
    if (current_stage_ == Stage::FinalHold) {
      latchFinalHoldAtCurrentPose();
    }

    RCLCPP_INFO(
      get_logger(),
      "Circular LocalIntent stage switch: %s -> %s at t=%.1fs",
      stageName(previous_stage), stageName(current_stage_), elapsed);
  }

  double decelScale(double elapsed) const
  {
    if (decel_duration_sec_ <= 1.0e-6) {
      return 0.0;
    }
    const double hover_end = std::max(0.0, hover_duration_sec_);
    const double orbit_end = hover_end + std::max(0.0, orbit_duration_sec_);
    const double tau = clamp01((elapsed - orbit_end) / decel_duration_sec_);
    return 1.0 - tau;
  }

  void latchCircleFromCurrentPose()
  {
    const double radius = std::max(0.3, radius_m_);
    const double hx = std::cos(current_heading_);
    const double hy = std::sin(current_heading_);
    const double left_x = -hy;
    const double left_y = hx;
    const double side = orbit_ccw_ ? 1.0 : -1.0;

    circle_center_x_ = current_x_ + side * radius * left_x;
    circle_center_y_ = current_y_ + side * radius * left_y;
    circle_latched_ = true;
  }

  void latchFinalHoldAtCurrentPose()
  {
    final_hold_x_ = current_x_;
    final_hold_y_ = current_y_;
    final_hold_heading_ = std::isfinite(current_heading_) ? current_heading_ : final_hold_heading_;
    final_hold_latched_ = true;
  }

  double currentCircleAngle() const
  {
    const double dx = current_x_ - circle_center_x_;
    const double dy = current_y_ - circle_center_y_;
    if (finite2(dx, dy) && std::hypot(dx, dy) > 1.0e-3) {
      return std::atan2(dy, dx);
    }
    const double hx = std::cos(current_heading_);
    const double hy = std::sin(current_heading_);
    const double radial_x = orbit_ccw_ ? hy : -hy;
    const double radial_y = orbit_ccw_ ? -hx : hx;
    return std::atan2(radial_y, radial_x);
  }

  void fillCommon(
    quad_midbridge_msgs::msg::LocalIntent & intent,
    double speed_pref,
    double speed_max,
    double progress_weight,
    bool terminal_hold) const
  {
    intent.corridor_half_width = static_cast<float>(corridor_half_width_);
    intent.corridor_half_height = static_cast<float>(corridor_half_height_);
    intent.speed_pref = static_cast<float>(std::max(0.0, speed_pref));
    intent.speed_max = static_cast<float>(std::max(0.0, speed_max));
    intent.risk_level = static_cast<float>(risk_level_);
    intent.observation_priority = static_cast<float>(observation_priority_);
    intent.progress_weight = static_cast<float>(std::max(0.0, progress_weight));
    intent.contour_weight = static_cast<float>(contour_weight_);
    intent.terminal_hold = terminal_hold;
    intent.allow_reverse_progress = allow_reverse_progress_;
  }

  void fillHoldIntent(quad_midbridge_msgs::msg::LocalIntent & intent) const
  {
    const int n = std::max(2, final_hold_num_points_);
    const double heading = final_hold_latched_ ? final_hold_heading_ : current_heading_;
    const double start_x = final_hold_latched_ ? final_hold_x_ : current_x_;
    const double start_y = final_hold_latched_ ? final_hold_y_ : current_y_;
    const double dx = std::cos(heading);
    const double dy = std::sin(heading);

    intent.centerline_points.clear();
    intent.centerline_s.clear();
    intent.centerline_points.reserve(static_cast<size_t>(n));
    intent.centerline_s.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      const double s = static_cast<double>(i) * path_spacing_;
      geometry_msgs::msg::Point p;
      p.x = start_x + s * dx;
      p.y = start_y + s * dy;
      p.z = target_z_;
      intent.centerline_points.push_back(p);
      intent.centerline_s.push_back(static_cast<float>(s));
    }

    intent.yaw_pref = static_cast<float>(heading);
    intent.yaw_rate_pref = 0.0f;
    fillCommon(intent, 0.0, orbit_speed_max_, 0.0, true);
  }

  void fillOrbitIntent(
    quad_midbridge_msgs::msg::LocalIntent & intent,
    double speed_pref,
    double speed_max,
    double progress_weight) const
  {
    const int n = std::max(3, arc_num_points_);
    const double radius = std::max(0.3, radius_m_);
    const double direction = orbit_ccw_ ? 1.0 : -1.0;
    const double theta0 = currentCircleAngle();

    intent.centerline_points.clear();
    intent.centerline_s.clear();
    intent.centerline_points.reserve(static_cast<size_t>(n));
    intent.centerline_s.reserve(static_cast<size_t>(n));

    for (int i = 0; i < n; ++i) {
      const double s = static_cast<double>(i) * path_spacing_;
      const double theta = theta0 + direction * s / radius;

      geometry_msgs::msg::Point p;
      p.x = circle_center_x_ + radius * std::cos(theta);
      p.y = circle_center_y_ + radius * std::sin(theta);
      p.z = target_z_;
      intent.centerline_points.push_back(p);
      intent.centerline_s.push_back(static_cast<float>(s));
    }

    const double tangent = theta0 + (orbit_ccw_ ? 0.5 * kPi : -0.5 * kPi);
    intent.yaw_pref = static_cast<float>(wrapAngle(tangent));
    intent.yaw_rate_pref = static_cast<float>(direction * speed_pref / radius);
    fillCommon(intent, speed_pref, speed_max, progress_weight, false);
  }

  void onTimer()
  {
    if (!have_local_position_ || localPositionTimeout()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for valid /fmu/out/vehicle_local_position in circular intent manager...");
      return;
    }

    if (!mission_started_) {
      mission_started_ = true;
      mission_start_steady_time_ = SteadyClock::now();
      mission_start_time_ = now();
      latchCircleFromCurrentPose();
      latchFinalHoldAtCurrentPose();
    }

    const double elapsed = std::chrono::duration<double>(
      SteadyClock::now() - mission_start_steady_time_).count();
    updateStage(elapsed);

    quad_midbridge_msgs::msg::LocalIntent intent;
    intent.header.stamp = now();
    intent.header.frame_id = "map";

    switch (current_stage_) {
      case Stage::Hover:
        fillHoldIntent(intent);
        break;
      case Stage::Orbit:
        fillOrbitIntent(intent, orbit_speed_pref_, orbit_speed_max_, orbit_progress_weight_);
        break;
      case Stage::Decelerate: {
        const double scale = decelScale(elapsed);
        fillOrbitIntent(
          intent,
          scale * orbit_speed_pref_,
          orbit_speed_max_,
          scale * orbit_progress_weight_);
        break;
      }
      case Stage::FinalHold:
        fillHoldIntent(intent);
        break;
    }

    if (intent.centerline_points.empty() ||
      !finite2(intent.centerline_points.front().x, intent.centerline_points.front().y))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Refusing to publish non-finite circular LocalIntent.");
      return;
    }

    intent_pub_->publish(intent);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "circular intent: stage=%s elapsed=%.1f first=(%.2f, %.2f, %.2f) center=(%.2f, %.2f) speed=%.3f progress_w=%.3f terminal=%s",
      stageName(current_stage_), elapsed,
      intent.centerline_points.front().x,
      intent.centerline_points.front().y,
      intent.centerline_points.front().z,
      circle_center_x_,
      circle_center_y_,
      intent.speed_pref,
      intent.progress_weight,
      intent.terminal_hold ? "true" : "false");
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Publisher<quad_midbridge_msgs::msg::LocalIntent>::SharedPtr intent_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool have_local_position_{false};
  bool mission_started_{false};
  bool circle_latched_{false};
  bool final_hold_latched_{false};
  SteadyClock::time_point mission_start_steady_time_{};
  SteadyClock::time_point last_local_steady_time_{};
  rclcpp::Time mission_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_time_{0, 0, RCL_ROS_TIME};
  Stage current_stage_{Stage::Hover};

  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};
  double current_heading_{0.0};
  double circle_center_x_{0.0};
  double circle_center_y_{0.0};
  double final_hold_x_{0.0};
  double final_hold_y_{0.0};
  double final_hold_heading_{0.0};

  double pub_rate_{10.0};
  double local_pos_timeout_sec_{0.5};
  double target_z_{-1.5};
  double radius_m_{2.0};
  double path_spacing_{0.35};
  int arc_num_points_{12};
  int final_hold_num_points_{4};
  bool orbit_ccw_{true};

  double hover_duration_sec_{30.0};
  double orbit_duration_sec_{80.0};
  double decel_duration_sec_{8.0};

  double orbit_speed_pref_{0.16};
  double orbit_speed_max_{0.25};
  double orbit_progress_weight_{0.22};
  double contour_weight_{0.8};
  double corridor_half_width_{1.0};
  double corridor_half_height_{1.5};
  double risk_level_{0.1};
  double observation_priority_{0.5};
  bool allow_reverse_progress_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircularLocalIntentManagerNode>());
  rclcpp::shutdown();
  return 0;
}
