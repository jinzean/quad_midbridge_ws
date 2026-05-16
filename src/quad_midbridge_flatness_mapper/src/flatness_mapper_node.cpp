#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "quad_midbridge_msgs/msg/attitude_thrust_reference.hpp"
#include "quad_midbridge_msgs/msg/executable_reference.hpp"
#include "quad_midbridge_msgs/msg/flatness_debug.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class FlatnessMapperNode : public rclcpp::Node
{
public:
  FlatnessMapperNode() : Node("flatness_mapper_node")
  {
    gravity_ = declare_parameter<double>("gravity", 9.81);
    thrust_scale_ = declare_parameter<double>("thrust_scale", 15.0);
    max_collective_thrust_ = declare_parameter<double>("max_collective_thrust", 1.0);

    enable_thrust_adaptation_ = declare_parameter<bool>("enable_thrust_adaptation", false);
    hover_thrust_norm_init_ = declare_parameter<double>("hover_thrust_norm_init", -1.0);
    hover_thrust_norm_min_ = declare_parameter<double>("hover_thrust_norm_min", 0.35);
    hover_thrust_norm_max_ = declare_parameter<double>("hover_thrust_norm_max", 0.92);
    hover_thrust_adapt_rate_ = declare_parameter<double>("hover_thrust_adapt_rate", 0.035);
    hover_thrust_error_deadband_m_ = declare_parameter<double>("hover_thrust_error_deadband_m", 0.03);
    hover_thrust_error_clip_m_ = declare_parameter<double>("hover_thrust_error_clip_m", 0.35);
    hover_thrust_vz_gain_sec_ = declare_parameter<double>("hover_thrust_vz_gain_sec", 0.35);
    adapt_max_ref_speed_ = declare_parameter<double>("adapt_max_ref_speed", 0.25);
    adapt_max_actual_speed_ = declare_parameter<double>("adapt_max_actual_speed", 0.45);
    adapt_max_lateral_accel_ = declare_parameter<double>("adapt_max_lateral_accel", 1.25);
    adapt_max_vertical_accel_ = declare_parameter<double>("adapt_max_vertical_accel", 1.80);
    adapt_max_abs_z_error_m_ = declare_parameter<double>("adapt_max_abs_z_error_m", 0.35);
    enable_overshoot_thrust_guard_ = declare_parameter<bool>("enable_overshoot_thrust_guard", true);
    overshoot_guard_z_error_m_ = declare_parameter<double>("overshoot_guard_z_error_m", 0.45);
    overshoot_guard_vz_up_mps_ = declare_parameter<double>("overshoot_guard_vz_up_mps", 0.35);
    overshoot_guard_thrust_max_ = declare_parameter<double>("overshoot_guard_thrust_max", 0.58);
    adapt_z_reset_pause_sec_ = declare_parameter<double>("adapt_z_reset_pause_sec", 1.0);
    local_pos_timeout_sec_ = declare_parameter<double>("local_pos_timeout_sec", 0.5);

    publish_flatness_debug_ = declare_parameter<bool>("publish_flatness_debug", true);
    flatness_debug_throttle_sec_ = declare_parameter<double>("flatness_debug_throttle_sec", 0.10);

    declare_parameter<std::string>("input_reference_topic", "/midbridge/governed_reference");
    declare_parameter<std::string>("local_position_topic", "/fmu/out/vehicle_local_position");
    declare_parameter<std::string>("hover_thrust_estimate_topic", "/midbridge/hover_thrust_estimate");
    declare_parameter<std::string>("flatness_debug_topic", "/midbridge/flatness_debug");

    const auto input_reference_topic = get_parameter("input_reference_topic").as_string();
    const auto local_position_topic = get_parameter("local_position_topic").as_string();
    const auto hover_thrust_estimate_topic = get_parameter("hover_thrust_estimate_topic").as_string();
    const auto flatness_debug_topic = get_parameter("flatness_debug_topic").as_string();

    initializeHoverThrustEstimate();

    ref_sub_ = create_subscription<quad_midbridge_msgs::msg::ExecutableReference>(
      input_reference_topic, 10,
      std::bind(&FlatnessMapperNode::refCallback, this, std::placeholders::_1));

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      local_position_topic, rclcpp::SensorDataQoS(),
      std::bind(&FlatnessMapperNode::localPositionCallback, this, std::placeholders::_1));

    att_pub_ = create_publisher<quad_midbridge_msgs::msg::AttitudeThrustReference>(
      "/midbridge/attitude_thrust_reference", 10);
    hover_thrust_est_pub_ = create_publisher<std_msgs::msg::Float32>(
      hover_thrust_estimate_topic, 10);
    flatness_debug_pub_ = create_publisher<quad_midbridge_msgs::msg::FlatnessDebug>(
      flatness_debug_topic, 10);

    RCLCPP_INFO(
      get_logger(),
      "flatness_mapper_node started. thrust_scale=%.3f, nominal_hover=%.3f, hover_thrust_est=%.3f, adaptation=%s, debug=%s",
      thrust_scale_, nominalHoverThrust(), hover_thrust_norm_est_,
      enable_thrust_adaptation_ ? "enabled" : "disabled",
      publish_flatness_debug_ ? "enabled" : "disabled");
  }

private:
  struct AdaptationDiagnostics
  {
    bool enabled{false};
    bool active{false};
    bool local_valid{false};
    bool z_reset_paused{false};
    double z_error_up{0.0};
    double adapt_signal{0.0};
    double dt{0.0};
    double desired_z{0.0};
    double local_z{0.0};
    double local_vz{0.0};
    double ref_speed{0.0};
    double actual_speed{0.0};
    double lateral_accel{0.0};
    double vertical_accel{0.0};
    std::string status{"not_updated"};
  };

  struct MappingDiagnostics
  {
    double desired_force_norm{0.0};
    double desired_tilt_rad{0.0};
    double collective_thrust_raw{0.0};
    double collective_thrust_clamped{0.0};
    double nominal_hover_thrust{0.0};
    bool thrust_saturated{false};
    AdaptationDiagnostics adaptation{};
  };

  static double norm2(double x, double y)
  {
    return std::sqrt(x * x + y * y);
  }

  static double norm3(const std::array<double, 3> & v)
  {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  }

  static std::array<double, 3> normalize(const std::array<double, 3> & v)
  {
    const double n = norm3(v);
    if (n < 1e-8 || !std::isfinite(n)) {
      return {0.0, 0.0, 1.0};
    }
    return {v[0] / n, v[1] / n, v[2] / n};
  }

  static std::array<double, 4> normalizeQuat(const std::array<double, 4> & q)
  {
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-8 || !std::isfinite(n)) {
      return {1.0, 0.0, 0.0, 0.0};
    }
    return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
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
    return normalizeQuat(q);
  }

  static bool finiteReference(const quad_midbridge_msgs::msg::ExecutableReference & msg)
  {
    return std::isfinite(msg.position.x) && std::isfinite(msg.position.y) &&
           std::isfinite(msg.position.z) && std::isfinite(msg.velocity.x) &&
           std::isfinite(msg.velocity.y) && std::isfinite(msg.velocity.z) &&
           std::isfinite(msg.acceleration.x) && std::isfinite(msg.acceleration.y) &&
           std::isfinite(msg.acceleration.z) && std::isfinite(msg.yaw) &&
           std::isfinite(msg.yaw_rate);
  }

  double nominalHoverThrust() const
  {
    const double scale = std::max(thrust_scale_, 1e-6);
    return std::clamp(gravity_ / scale, 0.0, std::max(0.0, max_collective_thrust_));
  }

  void initializeHoverThrustEstimate()
  {
    if (hover_thrust_norm_init_ > 0.0 && std::isfinite(hover_thrust_norm_init_)) {
      hover_thrust_norm_est_ = hover_thrust_norm_init_;
    } else {
      hover_thrust_norm_est_ = nominalHoverThrust();
    }
    hover_thrust_norm_min_ = std::clamp(hover_thrust_norm_min_, 0.0, std::max(0.0, max_collective_thrust_));
    hover_thrust_norm_max_ = std::clamp(hover_thrust_norm_max_, hover_thrust_norm_min_, std::max(0.0, max_collective_thrust_));
    hover_thrust_norm_est_ = std::clamp(hover_thrust_norm_est_, hover_thrust_norm_min_, hover_thrust_norm_max_);
  }

  bool localPositionTimedOut() const
  {
    if (!local_position_valid_) {
      return true;
    }
    return (now() - last_local_pos_time_).seconds() > local_pos_timeout_sec_;
  }

  void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    const bool valid = msg->z_valid && msg->v_z_valid &&
      std::isfinite(msg->z) && std::isfinite(msg->vz);
    if (!valid) {
      local_position_valid_ = false;
      return;
    }

    if (have_z_reset_counter_ && msg->z_reset_counter != last_z_reset_counter_) {
      z_reset_pause_until_sec_ = now().seconds() + std::max(0.0, adapt_z_reset_pause_sec_);
    }
    last_z_reset_counter_ = msg->z_reset_counter;
    have_z_reset_counter_ = true;

    local_z_ = msg->z;
    local_vz_ = msg->vz;
    local_vx_ = (msg->v_xy_valid && std::isfinite(msg->vx)) ? msg->vx : 0.0;
    local_vy_ = (msg->v_xy_valid && std::isfinite(msg->vy)) ? msg->vy : 0.0;
    local_position_valid_ = true;
    last_local_pos_time_ = now();
  }

  void publishHoverThrustEstimate()
  {
    if (!hover_thrust_est_pub_) {
      return;
    }
    const double t_sec = now().seconds();
    if (have_last_est_pub_ && (t_sec - last_est_pub_sec_) < 0.5) {
      return;
    }
    last_est_pub_sec_ = t_sec;
    have_last_est_pub_ = true;

    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(hover_thrust_norm_est_);
    hover_thrust_est_pub_->publish(msg);
  }

  AdaptationDiagnostics updateHoverThrustEstimate(
    const quad_midbridge_msgs::msg::ExecutableReference & msg)
  {
    publishHoverThrustEstimate();

    AdaptationDiagnostics diag;
    diag.enabled = enable_thrust_adaptation_;
    diag.local_valid = local_position_valid_ && !localPositionTimedOut();
    diag.desired_z = static_cast<double>(msg.position.z);
    diag.local_z = local_z_;
    diag.local_vz = local_vz_;
    diag.ref_speed = std::sqrt(
      static_cast<double>(msg.velocity.x) * static_cast<double>(msg.velocity.x) +
      static_cast<double>(msg.velocity.y) * static_cast<double>(msg.velocity.y) +
      static_cast<double>(msg.velocity.z) * static_cast<double>(msg.velocity.z));
    diag.actual_speed = std::sqrt(local_vx_ * local_vx_ + local_vy_ * local_vy_ + local_vz_ * local_vz_);
    diag.lateral_accel = norm2(msg.acceleration.x, msg.acceleration.y);
    diag.vertical_accel = std::abs(static_cast<double>(msg.acceleration.z));

    const double t_sec = now().seconds();
    diag.z_reset_paused = have_z_reset_counter_ && (t_sec < z_reset_pause_until_sec_);

    if (!enable_thrust_adaptation_) {
      have_last_adapt_time_ = false;
      diag.status = "disabled";
      return diag;
    }
    if (!diag.local_valid) {
      have_last_adapt_time_ = false;
      diag.status = "no_local_position";
      return diag;
    }
    if (diag.z_reset_paused) {
      have_last_adapt_time_ = false;
      diag.status = "z_reset_pause";
      return diag;
    }

    if (!have_last_adapt_time_) {
      last_adapt_time_sec_ = t_sec;
      have_last_adapt_time_ = true;
      diag.status = "warmup";
      return diag;
    }

    diag.dt = std::clamp(t_sec - last_adapt_time_sec_, 0.0, 0.10);
    last_adapt_time_sec_ = t_sec;
    if (diag.dt <= 1e-6) {
      diag.status = "dt_too_small";
      return diag;
    }

    if (diag.ref_speed > adapt_max_ref_speed_) {
      diag.status = "gate_ref_speed";
      return diag;
    }
    if (diag.actual_speed > adapt_max_actual_speed_) {
      diag.status = "gate_actual_speed";
      return diag;
    }
    if (diag.lateral_accel > adapt_max_lateral_accel_) {
      diag.status = "gate_lateral_accel";
      return diag;
    }
    if (diag.vertical_accel > adapt_max_vertical_accel_) {
      diag.status = "gate_vertical_accel";
      return diag;
    }

    // Never learn hover thrust while the vehicle is still far from the commanded
    // height. During takeoff this error is intentionally large, and adapting here
    // can drive the normalized hover estimate to its upper bound before the vehicle
    // has a chance to settle.
    const double raw_z_error_up = local_z_ - static_cast<double>(msg.position.z);
    if (std::abs(raw_z_error_up) > adapt_max_abs_z_error_m_) {
      diag.status = "gate_z_error";
      return diag;
    }

    // NED convention: z grows downward. local_z_ - desired_z is positive when the
    // vehicle is lower than commanded, so the hover thrust estimate should increase.
    double z_error_up = raw_z_error_up;
    if (std::abs(z_error_up) < hover_thrust_error_deadband_m_) {
      z_error_up = 0.0;
    } else {
      z_error_up -= std::copysign(hover_thrust_error_deadband_m_, z_error_up);
    }
    diag.z_error_up = std::clamp(z_error_up, -hover_thrust_error_clip_m_, hover_thrust_error_clip_m_);
    diag.adapt_signal = diag.z_error_up + hover_thrust_vz_gain_sec_ * local_vz_;

    if (!std::isfinite(diag.adapt_signal)) {
      diag.status = "nonfinite_signal";
      return diag;
    }

    const double delta = hover_thrust_adapt_rate_ * diag.dt * diag.adapt_signal;
    hover_thrust_norm_est_ = std::clamp(
      hover_thrust_norm_est_ + delta, hover_thrust_norm_min_, hover_thrust_norm_max_);

    diag.active = true;
    diag.status = "active";
    return diag;
  }

  double mapCollectiveThrust(
    double fd_norm,
    const quad_midbridge_msgs::msg::ExecutableReference & msg,
    MappingDiagnostics & diag)
  {
    diag.adaptation = updateHoverThrustEstimate(msg);
    diag.desired_force_norm = fd_norm;
    diag.nominal_hover_thrust = nominalHoverThrust();

    if (enable_thrust_adaptation_) {
      const double g = std::max(gravity_, 1e-6);
      diag.collective_thrust_raw = hover_thrust_norm_est_ * fd_norm / g;
    } else {
      const double thrust_scale = std::max(thrust_scale_, 1e-6);
      diag.collective_thrust_raw = fd_norm / thrust_scale;
    }

    const double max_thrust = std::max(0.0, max_collective_thrust_);
    diag.collective_thrust_clamped = std::clamp(diag.collective_thrust_raw, 0.0, max_thrust);

    // Last-resort vertical safety guard. In PX4 NED, local_z < desired_z means the
    // vehicle is above the reference, and local_vz < 0 means it is still climbing.
    // If both are true by a clear margin, cap thrust below hover to prevent an
    // upward flyaway caused by stale or over-adapted hover-thrust estimates.
    if (enable_overshoot_thrust_guard_ &&
        diag.adaptation.local_valid &&
        (local_z_ < static_cast<double>(msg.position.z) - overshoot_guard_z_error_m_) &&
        (local_vz_ < -std::abs(overshoot_guard_vz_up_mps_))) {
      diag.collective_thrust_clamped = std::min(
        diag.collective_thrust_clamped,
        std::clamp(overshoot_guard_thrust_max_, 0.0, max_thrust));
    }

    diag.thrust_saturated = std::abs(diag.collective_thrust_clamped - diag.collective_thrust_raw) > 1e-5;
    return diag.collective_thrust_clamped;
  }

  void publishFlatnessDebug(
    const quad_midbridge_msgs::msg::ExecutableReference & ref,
    const MappingDiagnostics & diag)
  {
    if (!publish_flatness_debug_ || !flatness_debug_pub_) {
      return;
    }
    const double t_sec = now().seconds();
    if (have_last_flatness_debug_pub_ &&
      (t_sec - last_flatness_debug_pub_sec_) < std::max(0.0, flatness_debug_throttle_sec_))
    {
      return;
    }
    last_flatness_debug_pub_sec_ = t_sec;
    have_last_flatness_debug_pub_ = true;

    quad_midbridge_msgs::msg::FlatnessDebug out;
    out.header = ref.header;
    out.desired_force_norm = static_cast<float>(diag.desired_force_norm);
    out.desired_tilt_rad = static_cast<float>(diag.desired_tilt_rad);
    out.desired_tilt_deg = static_cast<float>(diag.desired_tilt_rad * 57.29577951308232);
    out.collective_thrust_raw = static_cast<float>(diag.collective_thrust_raw);
    out.collective_thrust_clamped = static_cast<float>(diag.collective_thrust_clamped);
    out.hover_thrust_norm_est = static_cast<float>(hover_thrust_norm_est_);
    out.nominal_hover_thrust_norm = static_cast<float>(diag.nominal_hover_thrust);
    out.thrust_scale = static_cast<float>(thrust_scale_);

    out.desired_z = static_cast<float>(diag.adaptation.desired_z);
    out.local_z = static_cast<float>(diag.adaptation.local_z);
    out.local_vz = static_cast<float>(diag.adaptation.local_vz);
    out.z_error_up = static_cast<float>(diag.adaptation.z_error_up);
    out.adapt_signal = static_cast<float>(diag.adaptation.adapt_signal);
    out.adapt_dt = static_cast<float>(diag.adaptation.dt);

    out.ref_speed = static_cast<float>(diag.adaptation.ref_speed);
    out.actual_speed = static_cast<float>(diag.adaptation.actual_speed);
    out.lateral_accel = static_cast<float>(diag.adaptation.lateral_accel);
    out.vertical_accel = static_cast<float>(diag.adaptation.vertical_accel);

    out.thrust_adaptation_enabled = diag.adaptation.enabled;
    out.thrust_adaptation_active = diag.adaptation.active;
    out.local_position_valid = diag.adaptation.local_valid;
    out.z_reset_paused = diag.adaptation.z_reset_paused;
    out.thrust_saturated = diag.thrust_saturated;
    out.adaptation_status = diag.adaptation.status;
    flatness_debug_pub_->publish(out);
  }

  void publishInvalid(const quad_midbridge_msgs::msg::ExecutableReference & msg)
  {
    quad_midbridge_msgs::msg::AttitudeThrustReference out;
    out.header = msg.header;
    out.attitude.w = 1.0f;
    out.attitude.x = 0.0f;
    out.attitude.y = 0.0f;
    out.attitude.z = 0.0f;
    out.bodyrates.x = 0.0f;
    out.bodyrates.y = 0.0f;
    out.bodyrates.z = 0.0f;
    out.collective_thrust = 0.0f;
    out.yaw = std::isfinite(msg.yaw) ? msg.yaw : 0.0f;
    out.mode = msg.mode;
    out.valid = false;
    out.from_governor = true;
    att_pub_->publish(out);
  }

  void refCallback(const quad_midbridge_msgs::msg::ExecutableReference::SharedPtr msg)
  {
    if (!msg->feasible || !finiteReference(*msg)) {
      publishInvalid(*msg);
      return;
    }

    quad_midbridge_msgs::msg::AttitudeThrustReference out;
    out.header = msg->header;
    out.yaw = msg->yaw;
    out.mode = msg->mode;
    out.from_governor = true;

    // NED/FRD differential-flatness mapping for PX4 attitude-thrust setpoints.
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

    const double yaw = static_cast<double>(msg->yaw);
    const std::array<double, 3> xc = {std::cos(yaw), std::sin(yaw), 0.0};
    const auto b2_raw = cross(b3, xc);
    const std::array<double, 3> b2 = norm3(b2_raw) > 1e-6 ?
      normalize(b2_raw) : std::array<double, 3>{-std::sin(yaw), std::cos(yaw), 0.0};
    const auto b1 = normalize(cross(b2, b3));

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

    MappingDiagnostics diag;
    const double fd_norm = norm3(fd);
    diag.desired_tilt_rad = std::acos(std::clamp(b3[2], -1.0, 1.0));
    const double thrust = mapCollectiveThrust(fd_norm, *msg, diag);
    out.collective_thrust = static_cast<float>(thrust);

    out.valid = std::isfinite(fd_norm) && std::isfinite(thrust) &&
                std::isfinite(q[0]) && std::isfinite(q[1]) &&
                std::isfinite(q[2]) && std::isfinite(q[3]);

    if (!out.valid) {
      publishInvalid(*msg);
      return;
    }

    att_pub_->publish(out);
    publishFlatnessDebug(*msg, diag);
  }

  rclcpp::Subscription<quad_midbridge_msgs::msg::ExecutableReference>::SharedPtr ref_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Publisher<quad_midbridge_msgs::msg::AttitudeThrustReference>::SharedPtr att_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hover_thrust_est_pub_;
  rclcpp::Publisher<quad_midbridge_msgs::msg::FlatnessDebug>::SharedPtr flatness_debug_pub_;

  double gravity_{9.81};
  double thrust_scale_{15.0};
  double max_collective_thrust_{1.0};

  bool enable_thrust_adaptation_{false};
  double hover_thrust_norm_init_{-1.0};
  double hover_thrust_norm_min_{0.35};
  double hover_thrust_norm_max_{0.92};
  double hover_thrust_norm_est_{0.68};
  double hover_thrust_adapt_rate_{0.035};
  double hover_thrust_error_deadband_m_{0.03};
  double hover_thrust_error_clip_m_{0.35};
  double hover_thrust_vz_gain_sec_{0.35};
  double adapt_max_ref_speed_{0.25};
  double adapt_max_actual_speed_{0.45};
  double adapt_max_lateral_accel_{1.25};
  double adapt_max_vertical_accel_{1.80};
  double adapt_max_abs_z_error_m_{0.35};
  bool enable_overshoot_thrust_guard_{true};
  double overshoot_guard_z_error_m_{0.45};
  double overshoot_guard_vz_up_mps_{0.35};
  double overshoot_guard_thrust_max_{0.58};
  double adapt_z_reset_pause_sec_{1.0};
  double local_pos_timeout_sec_{0.5};

  bool publish_flatness_debug_{true};
  double flatness_debug_throttle_sec_{0.10};

  bool local_position_valid_{false};
  rclcpp::Time last_local_pos_time_{0, 0, RCL_ROS_TIME};
  double local_z_{0.0};
  double local_vz_{0.0};
  double local_vx_{0.0};
  double local_vy_{0.0};
  bool have_z_reset_counter_{false};
  uint8_t last_z_reset_counter_{0};
  double z_reset_pause_until_sec_{0.0};

  bool have_last_adapt_time_{false};
  double last_adapt_time_sec_{0.0};
  bool have_last_est_pub_{false};
  double last_est_pub_sec_{0.0};
  bool have_last_flatness_debug_pub_{false};
  double last_flatness_debug_pub_sec_{0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlatnessMapperNode>());
  rclcpp::shutdown();
  return 0;
}
