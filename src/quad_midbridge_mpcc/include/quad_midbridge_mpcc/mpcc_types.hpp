#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "quad_midbridge_msgs/msg/executable_reference.hpp"
#include "quad_midbridge_msgs/msg/local_intent.hpp"
#include "quad_midbridge_msgs/msg/mpcc_debug.hpp"
#include "rclcpp/rclcpp.hpp"

namespace quad_midbridge_mpcc
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct PathFrame
{
  Vec3 position{};
  Vec3 tangent{1.0, 0.0, 0.0};
  double s{0.0};
  double contour_error{0.0};
  double lag_error{0.0};
  double distance_to_path{0.0};
  bool valid{false};
};

struct KinematicState
{
  Vec3 p{0.0, 0.0, -1.5};
  Vec3 v{0.0, 0.0, 0.0};
  Vec3 a{0.0, 0.0, 0.0};
  double psi{0.0};
  double s_progress{0.0};
  bool valid{false};
};

struct MpccParams
{
  int horizon_steps{20};
  double horizon_dt{0.05};
  double preview_time{0.35};
  double gravity{9.81};
  double thrust_min{4.0};
  double thrust_max{16.0};
  double tilt_max_rad{0.65};
  double max_speed{1.5};
  double max_accel{4.0};
  double max_jerk{8.0};
  double max_yaw_rate{0.8};

  double weight_contour{8.0};
  double weight_lag{2.0};
  double weight_progress{1.0};
  double weight_yaw{0.8};
  double weight_speed{0.5};
  double weight_accel{0.1};
  double weight_jerk{0.05};
  double weight_terminal_contour{12.0};
  double weight_terminal_lag{4.0};
  double weight_terminal_yaw{1.2};
  double weight_delta_jerk{0.02};
  double weight_delta_yaw_rate{0.02};
};

struct SolverInput
{
  rclcpp::Time stamp;
  KinematicState state{};
  quad_midbridge_msgs::msg::LocalIntent::SharedPtr intent;
  MpccParams params{};
  std::string fallback_backend{"full_state_placeholder"};
};

struct SolverOutput
{
  quad_midbridge_msgs::msg::ExecutableReference ref;
  quad_midbridge_msgs::msg::MpccDebug debug;
};

struct AcadosModelLayout
{
  static constexpr int kNx = 11;  // [p(3), v(3), a(3), psi, s]
  static constexpr int kNu = 5;   // [j(3), psi_dot, s_dot]
};

struct AcadosCostWeights
{
  double contour{0.0};
  double lag{0.0};
  double progress{0.0};
  double yaw{0.0};
  double speed{0.0};
  double accel{0.0};
  double jerk{0.0};
  double delta_jerk{0.0};
  double delta_yaw_rate{0.0};
  double terminal_contour{0.0};
  double terminal_lag{0.0};
  double terminal_yaw{0.0};
};

struct AcadosConstraintBounds
{
  double speed_max{0.0};
  double accel_max{0.0};
  double jerk_max{0.0};
  double yaw_rate_max{0.0};
  double thrust_min{0.0};
  double thrust_max{0.0};
  double tilt_max_rad{0.0};
  double corridor_half_width{0.0};
  double corridor_half_height{0.0};
  bool terminal_hold{false};
  bool allow_reverse_progress{false};
};

struct AcadosStageData
{
  std::array<double, AcadosModelLayout::kNx> x_ref{};
  std::array<double, AcadosModelLayout::kNu> u_ref{};
  std::array<double, 3> path_point{};
  std::array<double, 3> path_tangent{1.0, 0.0, 0.0};
  double s_ref{0.0};
  double yaw_ref{0.0};
  double speed_ref{0.0};
  double risk_level{0.0};
  AcadosCostWeights weights{};
  AcadosConstraintBounds bounds{};
};

struct AcadosTerminalData
{
  std::array<double, AcadosModelLayout::kNx> x_ref{};
  std::array<double, 3> path_point{};
  std::array<double, 3> path_tangent{1.0, 0.0, 0.0};
  double s_ref{0.0};
  double yaw_ref{0.0};
  AcadosCostWeights weights{};
  AcadosConstraintBounds bounds{};
};

struct AcadosProblemData
{
  std::string backend_status{"uninitialized"};
  std::array<double, AcadosModelLayout::kNx> x0{};
  std::array<double, AcadosModelLayout::kNu> u_prev{};
  std::vector<AcadosStageData> stages{};
  AcadosTerminalData terminal{};
  AcadosCostWeights nominal_weights{};
  AcadosConstraintBounds nominal_bounds{};
  double path_length{0.0};
  double risk_level{0.0};
  double desired_speed{0.0};
  double desired_yaw{0.0};
  bool use_speed_preference{true};
};

}  // namespace quad_midbridge_mpcc
