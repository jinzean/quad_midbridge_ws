#include "quad_midbridge_mpcc/acados_problem_builder.hpp"

#include <algorithm>
#include <cmath>

#include "quad_midbridge_mpcc/mpcc_path_utils.hpp"

namespace quad_midbridge_mpcc
{
namespace
{
std::array<double, AcadosModelLayout::kNx> packState(const KinematicState & s)
{
  return {s.p.x, s.p.y, s.p.z,
          s.v.x, s.v.y, s.v.z,
          s.a.x, s.a.y, s.a.z,
          s.psi, s.s_progress};
}

AcadosCostWeights makeWeights(const SolverInput & input, double risk)
{
  AcadosCostWeights w;
  const double contour_gain = 1.0 + 1.5 * risk;
  const double progress_gain = std::max(0.2, 1.0 - 0.6 * risk);
  w.contour = input.params.weight_contour * contour_gain;
  w.lag = input.params.weight_lag;
  w.progress = input.params.weight_progress * progress_gain;
  w.yaw = input.params.weight_yaw * (1.0 + 0.5 * risk);
  w.speed = input.params.weight_speed;
  w.accel = input.params.weight_accel;
  w.jerk = input.params.weight_jerk;
  w.delta_jerk = input.params.weight_delta_jerk;
  w.delta_yaw_rate = input.params.weight_delta_yaw_rate;
  w.terminal_contour = input.params.weight_terminal_contour * contour_gain;
  w.terminal_lag = input.params.weight_terminal_lag;
  w.terminal_yaw = input.params.weight_terminal_yaw * (1.0 + 0.3 * risk);
  return w;
}

AcadosConstraintBounds makeBounds(const SolverInput & input, const quad_midbridge_msgs::msg::LocalIntent & intent, double risk)
{
  AcadosConstraintBounds b;
  b.speed_max = std::min(
    static_cast<double>(intent.speed_max > 1e-3 ? intent.speed_max : input.params.max_speed),
    input.params.max_speed * (1.0 - 0.35 * risk));
  b.accel_max = input.params.max_accel;
  b.jerk_max = input.params.max_jerk;
  b.yaw_rate_max = std::min(
    input.params.max_yaw_rate,
    static_cast<double>(intent.yaw_rate_pref > 1e-3 ? intent.yaw_rate_pref : input.params.max_yaw_rate));
  b.thrust_min = input.params.thrust_min;
  b.thrust_max = input.params.thrust_max;
  b.tilt_max_rad = input.params.tilt_max_rad * (1.0 - 0.2 * risk);
  b.corridor_half_width = std::max(0.2, static_cast<double>(intent.corridor_half_width));
  b.corridor_half_height = std::max(0.2, static_cast<double>(intent.corridor_half_height > 1e-3 ? intent.corridor_half_height : intent.corridor_half_width));
  b.terminal_hold = intent.terminal_hold;
  b.allow_reverse_progress = intent.allow_reverse_progress;
  return b;
}

std::array<double, AcadosModelLayout::kNu> makeControlRef(const PathFrame & frame, double desired_speed, double yaw_ref)
{
  (void)frame;
  return {0.0, 0.0, 0.0, yaw_ref, desired_speed};
}
}  // namespace

AcadosProblemData buildAcadosProblem(const SolverInput & input)
{
  AcadosProblemData out;
  out.x0 = packState(input.state);

  if (!input.intent || input.intent->centerline_points.size() < 2 || !input.state.valid) {
    out.backend_status = "problem_build_failed";
    return out;
  }

  const auto & intent = *input.intent;
  out.risk_level = std::clamp(static_cast<double>(intent.risk_level), 0.0, 1.0);
  out.nominal_weights = makeWeights(input, out.risk_level);
  out.nominal_bounds = makeBounds(input, intent, out.risk_level);

  const double path_len = estimatePathLength(intent);
  out.path_length = path_len;
  if (path_len <= 1e-6) {
    out.backend_status = "path_too_short";
    return out;
  }

  const PathFrame nearest = projectToPath(intent, input.state.p);
  if (!nearest.valid) {
    out.backend_status = "projection_failed";
    return out;
  }

  out.desired_speed = clamp(static_cast<double>(intent.speed_pref), 0.0, std::max(0.1, out.nominal_bounds.speed_max));
  const double tangent_yaw = std::atan2(nearest.tangent.y, nearest.tangent.x);
  const double obs_priority = clamp(static_cast<double>(intent.observation_priority), 0.0, 1.0);
  out.desired_yaw = wrapAngle(obs_priority * static_cast<double>(intent.yaw_pref) + (1.0 - obs_priority) * tangent_yaw);
  out.use_speed_preference = intent.speed_pref > 1e-3f;

  const double ds = out.desired_speed * input.params.horizon_dt;
  out.stages.reserve(static_cast<size_t>(std::max(1, input.params.horizon_steps)));

  for (int k = 0; k < std::max(1, input.params.horizon_steps); ++k) {
    const double s_ref = clamp(nearest.s + static_cast<double>(k + 1) * ds, 0.0, path_len);
    const PathFrame frame = samplePathAtS(intent, s_ref);

    AcadosStageData stage;
    stage.path_point = {frame.position.x, frame.position.y, frame.position.z};
    stage.path_tangent = {frame.tangent.x, frame.tangent.y, frame.tangent.z};
    stage.s_ref = s_ref;
    stage.yaw_ref = out.desired_yaw;
    stage.speed_ref = out.desired_speed;
    stage.risk_level = out.risk_level;
    stage.weights = out.nominal_weights;
    stage.bounds = out.nominal_bounds;
    stage.u_ref = makeControlRef(frame, out.desired_speed, out.desired_yaw);

    const Vec3 v_ref = mul(frame.tangent, out.desired_speed);
    stage.x_ref = {
      frame.position.x, frame.position.y, frame.position.z,
      v_ref.x, v_ref.y, v_ref.z,
      0.0, 0.0, 0.0,
      out.desired_yaw, s_ref};

    out.stages.push_back(stage);
  }

  const double s_terminal = clamp(nearest.s + static_cast<double>(std::max(1, input.params.horizon_steps)) * ds, 0.0, path_len);
  const PathFrame terminal_frame = samplePathAtS(intent, s_terminal);
  const Vec3 v_terminal = mul(terminal_frame.tangent, out.desired_speed);
  out.terminal.path_point = {terminal_frame.position.x, terminal_frame.position.y, terminal_frame.position.z};
  out.terminal.path_tangent = {terminal_frame.tangent.x, terminal_frame.tangent.y, terminal_frame.tangent.z};
  out.terminal.s_ref = s_terminal;
  out.terminal.yaw_ref = out.desired_yaw;
  out.terminal.weights = out.nominal_weights;
  out.terminal.bounds = out.nominal_bounds;
  out.terminal.x_ref = {
    terminal_frame.position.x, terminal_frame.position.y, terminal_frame.position.z,
    v_terminal.x, v_terminal.y, v_terminal.z,
    0.0, 0.0, 0.0,
    out.desired_yaw, s_terminal};

  out.u_prev = {0.0, 0.0, 0.0, 0.0, out.desired_speed};
  out.backend_status = "problem_ready";
  return out;
}

}  // namespace quad_midbridge_mpcc
