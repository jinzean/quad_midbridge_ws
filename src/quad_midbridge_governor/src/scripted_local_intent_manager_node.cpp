#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "quad_midbridge_msgs/msg/local_intent.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
inline double clamp01(double x)
{
  return std::max(0.0, std::min(x, 1.0));
}

inline bool finite2(double x, double y)
{
  return std::isfinite(x) && std::isfinite(y);
}
}  // namespace

class ScriptedLocalIntentManagerNode : public rclcpp::Node
{
public:
  ScriptedLocalIntentManagerNode()
  : Node("scripted_local_intent_manager_node")
  {
    pub_rate_ = declare_parameter<double>("pub_rate", 10.0);
    local_pos_timeout_sec_ = declare_parameter<double>("local_pos_timeout_sec", 0.5);

    target_z_ = declare_parameter<double>("target_z", -1.5);
    corridor_half_width_ = declare_parameter<double>("corridor_half_width", 1.0);
    corridor_half_height_ = declare_parameter<double>("corridor_half_height", 1.5);
    path_spacing_ = declare_parameter<double>("path_spacing", 0.4);
    risk_level_ = declare_parameter<double>("risk_level", 0.1);
    observation_priority_ = declare_parameter<double>("observation_priority", 0.5);
    contour_weight_ = declare_parameter<double>("contour_weight", 0.5);
    allow_reverse_progress_ = declare_parameter<bool>("allow_reverse_progress", false);

    hover_duration_sec_ = declare_parameter<double>("hover_duration_sec", 45.0);
    forward_duration_sec_ = declare_parameter<double>("forward_duration_sec", 67.0);
    decel_duration_sec_ = declare_parameter<double>("decel_duration_sec", 8.0);

    hover_num_points_ = declare_parameter<int>("hover_num_points", 4);
    forward_num_points_ = declare_parameter<int>("forward_num_points", 8);
    final_hold_num_points_ = declare_parameter<int>("final_hold_num_points", 4);

    forward_speed_pref_ = declare_parameter<double>("forward_speed_pref", 0.06);
    forward_speed_max_ = declare_parameter<double>("forward_speed_max", 0.10);
    forward_progress_weight_ = declare_parameter<double>("forward_progress_weight", 0.12);

    latch_heading_on_stage_entry_ = declare_parameter<bool>("latch_heading_on_stage_entry", true);

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
      std::bind(&ScriptedLocalIntentManagerNode::localPosCb, this, std::placeholders::_1));

    intent_pub_ = create_publisher<quad_midbridge_msgs::msg::LocalIntent>(
      "/midbridge/local_intent", 10);

    const auto period_ms = static_cast<int>(1000.0 / std::max(pub_rate_, 1.0));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(period_ms, 1)),
      std::bind(&ScriptedLocalIntentManagerNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "scripted_local_intent_manager_node started: hover %.1fs, forward %.1fs, decel %.1fs, speed %.3f",
      hover_duration_sec_, forward_duration_sec_, decel_duration_sec_, forward_speed_pref_);
  }

private:
  enum class Stage
  {
    Hover,
    Forward,
    Decelerate,
    FinalHold
  };

  struct Anchor
  {
    double x{0.0};
    double y{0.0};
    double heading{0.0};
    bool valid{false};
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
        "Ignoring invalid VehicleLocalPosition in scripted intent manager: xy_valid=%s z_valid=%s",
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
    last_local_time_ = now();
    have_local_position_ = true;

    if (!mission_started_) {
      mission_started_ = true;
      mission_start_time_ = now();
      current_stage_ = Stage::Hover;
      latchAnchor(hover_anchor_);
      RCLCPP_INFO(
        get_logger(),
        "Mission clock started. Hover anchor=(%.3f, %.3f), heading=%.3f",
        hover_anchor_.x, hover_anchor_.y, hover_anchor_.heading);
    }
  }

  bool localPositionTimeout() const
  {
    if (!have_local_position_) {
      return true;
    }
    return (now() - last_local_time_).seconds() > local_pos_timeout_sec_;
  }

  void latchAnchor(Anchor & anchor)
  {
    anchor.x = current_x_;
    anchor.y = current_y_;
    if (latch_heading_on_stage_entry_ && std::isfinite(current_heading_)) {
      anchor.heading = current_heading_;
    }
    anchor.valid = true;
  }

  Stage desiredStage(double elapsed) const
  {
    const double hover_end = std::max(0.0, hover_duration_sec_);
    const double forward_end = hover_end + std::max(0.0, forward_duration_sec_);
    const double decel_end = forward_end + std::max(0.0, decel_duration_sec_);

    if (elapsed < hover_end) {
      return Stage::Hover;
    }
    if (elapsed < forward_end) {
      return Stage::Forward;
    }
    if (elapsed < decel_end) {
      return Stage::Decelerate;
    }
    return Stage::FinalHold;
  }

  const char * stageName(Stage stage) const
  {
    switch (stage) {
      case Stage::Hover: return "hover";
      case Stage::Forward: return "forward";
      case Stage::Decelerate: return "decelerate";
      case Stage::FinalHold: return "final_hold";
    }
    return "unknown";
  }

  void updateStage(double elapsed)
  {
    const Stage next_stage = desiredStage(elapsed);
    if (next_stage == current_stage_) {
      return;
    }

    const Stage previous_stage = current_stage_;
    current_stage_ = next_stage;

    if (current_stage_ == Stage::Forward && !forward_anchor_.valid) {
      latchAnchor(forward_anchor_);
    }
    if (current_stage_ == Stage::Decelerate && !decel_anchor_.valid) {
      // Keep the forward path anchor for deceleration so the vehicle slows down
      // on the same segment rather than jumping to a new path.
      decel_anchor_ = forward_anchor_.valid ? forward_anchor_ : hover_anchor_;
    }
    if (current_stage_ == Stage::FinalHold && !final_hold_anchor_.valid) {
      latchAnchor(final_hold_anchor_);
    }

    RCLCPP_INFO(
      get_logger(),
      "LocalIntent stage switch: %s -> %s at t=%.1fs",
      stageName(previous_stage), stageName(current_stage_), elapsed);
  }

  double decelScale(double elapsed) const
  {
    if (decel_duration_sec_ <= 1.0e-6) {
      return 0.0;
    }
    const double hover_end = std::max(0.0, hover_duration_sec_);
    const double forward_end = hover_end + std::max(0.0, forward_duration_sec_);
    const double tau = clamp01((elapsed - forward_end) / decel_duration_sec_);
    return 1.0 - tau;
  }

  void fillPath(
    quad_midbridge_msgs::msg::LocalIntent & intent,
    const Anchor & anchor,
    int num_points,
    double speed_pref,
    double speed_max,
    double progress_weight,
    bool terminal_hold) const
  {
    const int n = std::max(2, num_points);
    const double dx = std::cos(anchor.heading);
    const double dy = std::sin(anchor.heading);

    intent.centerline_points.clear();
    intent.centerline_s.clear();
    intent.centerline_points.reserve(static_cast<size_t>(n));
    intent.centerline_s.reserve(static_cast<size_t>(n));

    for (int i = 0; i < n; ++i) {
      const double s = static_cast<double>(i) * path_spacing_;
      geometry_msgs::msg::Point p;
      p.x = anchor.x + s * dx;
      p.y = anchor.y + s * dy;
      p.z = target_z_;
      intent.centerline_points.push_back(p);
      intent.centerline_s.push_back(static_cast<float>(s));
    }

    intent.corridor_half_width = static_cast<float>(corridor_half_width_);
    intent.corridor_half_height = static_cast<float>(corridor_half_height_);
    intent.speed_pref = static_cast<float>(std::max(0.0, speed_pref));
    intent.speed_max = static_cast<float>(std::max(0.0, speed_max));
    intent.yaw_pref = static_cast<float>(anchor.heading);
    intent.yaw_rate_pref = 0.0f;
    intent.risk_level = static_cast<float>(risk_level_);
    intent.observation_priority = static_cast<float>(observation_priority_);
    intent.progress_weight = static_cast<float>(std::max(0.0, progress_weight));
    intent.contour_weight = static_cast<float>(contour_weight_);
    intent.terminal_hold = terminal_hold;
    intent.allow_reverse_progress = allow_reverse_progress_;
  }

  void onTimer()
  {
    if (!have_local_position_ || localPositionTimeout()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for valid /fmu/out/vehicle_local_position in scripted intent manager...");
      return;
    }

    if (!mission_started_) {
      mission_started_ = true;
      mission_start_time_ = now();
      latchAnchor(hover_anchor_);
    }

    const double elapsed = (now() - mission_start_time_).seconds();
    updateStage(elapsed);

    quad_midbridge_msgs::msg::LocalIntent intent;
    intent.header.stamp = now();
    intent.header.frame_id = "map";

    switch (current_stage_) {
      case Stage::Hover:
        fillPath(intent, hover_anchor_, hover_num_points_, 0.0, forward_speed_max_, 0.0, true);
        break;
      case Stage::Forward:
        if (!forward_anchor_.valid) {
          latchAnchor(forward_anchor_);
        }
        fillPath(
          intent, forward_anchor_, forward_num_points_,
          forward_speed_pref_, forward_speed_max_, forward_progress_weight_, false);
        break;
      case Stage::Decelerate: {
        if (!decel_anchor_.valid) {
          decel_anchor_ = forward_anchor_.valid ? forward_anchor_ : hover_anchor_;
        }
        const double scale = decelScale(elapsed);
        fillPath(
          intent, decel_anchor_, forward_num_points_,
          scale * forward_speed_pref_, forward_speed_max_,
          scale * forward_progress_weight_, false);
        break;
      }
      case Stage::FinalHold:
        if (!final_hold_anchor_.valid) {
          latchAnchor(final_hold_anchor_);
        }
        fillPath(intent, final_hold_anchor_, final_hold_num_points_, 0.0, forward_speed_max_, 0.0, true);
        break;
    }

    if (!finite2(intent.centerline_points.front().x, intent.centerline_points.front().y)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Refusing to publish non-finite LocalIntent.");
      return;
    }

    intent_pub_->publish(intent);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "scripted intent: stage=%s elapsed=%.1f first=(%.2f, %.2f, %.2f) speed=%.3f progress_w=%.3f terminal=%s",
      stageName(current_stage_), elapsed,
      intent.centerline_points.front().x,
      intent.centerline_points.front().y,
      intent.centerline_points.front().z,
      intent.speed_pref,
      intent.progress_weight,
      intent.terminal_hold ? "true" : "false");
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Publisher<quad_midbridge_msgs::msg::LocalIntent>::SharedPtr intent_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool have_local_position_{false};
  bool mission_started_{false};
  rclcpp::Time mission_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_time_{0, 0, RCL_ROS_TIME};
  Stage current_stage_{Stage::Hover};

  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};
  double current_heading_{0.0};

  Anchor hover_anchor_{};
  Anchor forward_anchor_{};
  Anchor decel_anchor_{};
  Anchor final_hold_anchor_{};

  double pub_rate_{10.0};
  double local_pos_timeout_sec_{0.5};
  double target_z_{-1.5};
  double corridor_half_width_{1.0};
  double corridor_half_height_{1.5};
  double path_spacing_{0.4};
  double risk_level_{0.1};
  double observation_priority_{0.5};
  double contour_weight_{0.5};
  bool allow_reverse_progress_{false};

  double hover_duration_sec_{45.0};
  double forward_duration_sec_{67.0};
  double decel_duration_sec_{8.0};

  int hover_num_points_{4};
  int forward_num_points_{8};
  int final_hold_num_points_{4};

  double forward_speed_pref_{0.06};
  double forward_speed_max_{0.10};
  double forward_progress_weight_{0.12};
  bool latch_heading_on_stage_entry_{true};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScriptedLocalIntentManagerNode>());
  rclcpp::shutdown();
  return 0;
}
