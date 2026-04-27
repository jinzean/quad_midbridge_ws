#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "quad_midbridge_mpcc/mpcc_path_utils.hpp"
#include "quad_midbridge_mpcc/mpcc_solver_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using quad_midbridge_mpcc::KinematicState;
using quad_midbridge_mpcc::MpccParams;
using quad_midbridge_mpcc::SolverInput;
using quad_midbridge_mpcc::Vec3;
using quad_midbridge_mpcc::createSolver;
using quad_midbridge_mpcc::projectToPath;

namespace
{
double yawFromQuaternionNed(const std::array<float, 4> & q)
{
  const double qw = q[0];
  const double qx = q[1];
  const double qy = q[2];
  const double qz = q[3];
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

class MpccNode : public rclcpp::Node
{
public:
  MpccNode() : Node("mpcc_node")
  {
    solve_rate_hz_ = declare_parameter<double>("solve_rate_hz", 30.0);
    params_.horizon_steps = declare_parameter<int>("horizon_steps", 20);
    params_.horizon_dt = declare_parameter<double>("horizon_dt", 0.05);
    params_.preview_time = declare_parameter<double>("preview_time", 0.35);
    params_.gravity = declare_parameter<double>("gravity", 9.81);
    params_.thrust_min = declare_parameter<double>("thrust_min", 4.0);
    params_.thrust_max = declare_parameter<double>("thrust_max", 16.0);
    params_.tilt_max_rad = declare_parameter<double>("tilt_max_rad", 0.65);
    params_.max_speed = declare_parameter<double>("max_speed", 1.5);
    params_.max_accel = declare_parameter<double>("max_accel", 4.0);
    params_.max_jerk = declare_parameter<double>("max_jerk", 8.0);
    params_.max_yaw_rate = declare_parameter<double>("max_yaw_rate", 0.8);
    params_.weight_contour = declare_parameter<double>("weight_contour", 8.0);
    params_.weight_lag = declare_parameter<double>("weight_lag", 2.0);
    params_.weight_progress = declare_parameter<double>("weight_progress", 1.0);
    params_.weight_yaw = declare_parameter<double>("weight_yaw", 0.8);
    params_.weight_speed = declare_parameter<double>("weight_speed", 0.5);
    params_.weight_accel = declare_parameter<double>("weight_accel", 0.1);
    params_.weight_jerk = declare_parameter<double>("weight_jerk", 0.05);
    solver_backend_ = declare_parameter<std::string>("solver_backend", "acados_skeleton");
    fallback_backend_ = declare_parameter<std::string>("fallback_backend", "full_state_placeholder");
    use_local_position_z_ = declare_parameter<bool>("use_local_position_z", true);

    solver_ = createSolver(solver_backend_);

    intent_sub_ = create_subscription<quad_midbridge_msgs::msg::LocalIntent>(
      "/midbridge/local_intent", 10,
      std::bind(&MpccNode::intentCallback, this, std::placeholders::_1));

    auto px4_out_qos = rclcpp::SensorDataQoS();

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", px4_out_qos,
      std::bind(&MpccNode::localPositionCallback, this, std::placeholders::_1));

    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", px4_out_qos,
      std::bind(&MpccNode::attitudeCallback, this, std::placeholders::_1));

    ref_pub_ = create_publisher<quad_midbridge_msgs::msg::ExecutableReference>(
      "/midbridge/raw_reference", 10);

    debug_pub_ = create_publisher<quad_midbridge_msgs::msg::MpccDebug>(
      "/midbridge/mpcc_debug", 10);

    const auto period = std::chrono::duration<double>(1.0 / solve_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MpccNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "mpcc_node started with backend=%s",
      solver_ ? solver_->name().c_str() : "null");
  }

private:
  void intentCallback(const quad_midbridge_msgs::msg::LocalIntent::SharedPtr msg)
  {
    latest_intent_ = msg;
  }

  void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    const Vec3 prev_v = state_.v;
    const bool had_prev_sample = have_prev_velocity_sample_;
    const double prev_t = last_local_pos_time_sec_;

    state_.p.x = msg->x;
    state_.p.y = msg->y;
    if (use_local_position_z_) {
      state_.p.z = msg->z;
    }
    state_.v.x = msg->vx;
    state_.v.y = msg->vy;
    state_.v.z = msg->vz;
    state_.valid = std::isfinite(msg->x) && std::isfinite(msg->y) && std::isfinite(msg->z) &&
                   std::isfinite(msg->vx) && std::isfinite(msg->vy) && std::isfinite(msg->vz);

    const double now_sec = static_cast<double>(msg->timestamp) * 1e-6;
    if (had_prev_sample && now_sec > prev_t + 1e-4) {
      const double dt = now_sec - prev_t;
      state_.a.x = (state_.v.x - prev_v.x) / dt;
      state_.a.y = (state_.v.y - prev_v.y) / dt;
      state_.a.z = (state_.v.z - prev_v.z) / dt;
    }
    last_local_pos_time_sec_ = now_sec;
    have_prev_velocity_sample_ = true;

    if (latest_intent_ && latest_intent_->centerline_points.size() >= 2) {
      const auto frame = projectToPath(*latest_intent_, state_.p);
      if (frame.valid) {
        state_.s_progress = frame.s;
      }
    }
  }

  void attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    std::array<float, 4> q{};
    q[0] = msg->q[0];
    q[1] = msg->q[1];
    q[2] = msg->q[2];
    q[3] = msg->q[3];
    state_.psi = yawFromQuaternionNed(q);
  }

  void timerCallback()
  {
    if (!solver_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "MPCC solver backend is null.");
      return;
    }

    if (!latest_intent_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for LocalIntent...");
      return;
    }

    SolverInput in;
    in.stamp = now();
    in.state = state_;
    in.intent = latest_intent_;
    in.params = params_;
    in.fallback_backend = fallback_backend_;

    const auto t0 = now();
    auto result = solver_->solve(in);
    result.debug.solver_time_ms = static_cast<float>((now() - t0).seconds() * 1e3);
    ref_pub_->publish(result.ref);
    debug_pub_->publish(result.debug);
  }

  rclcpp::Subscription<quad_midbridge_msgs::msg::LocalIntent>::SharedPtr intent_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Publisher<quad_midbridge_msgs::msg::ExecutableReference>::SharedPtr ref_pub_;
  rclcpp::Publisher<quad_midbridge_msgs::msg::MpccDebug>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  quad_midbridge_msgs::msg::LocalIntent::SharedPtr latest_intent_;
  KinematicState state_{};
  MpccParams params_{};
  std::unique_ptr<quad_midbridge_mpcc::MpccSolverInterface> solver_;

  double solve_rate_hz_{30.0};
  bool use_local_position_z_{true};
  std::string solver_backend_{"acados_skeleton"};
  std::string fallback_backend_{"full_state_placeholder"};
  double last_local_pos_time_sec_{0.0};
  bool have_prev_velocity_sample_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpccNode>());
  rclcpp::shutdown();
  return 0;
}
