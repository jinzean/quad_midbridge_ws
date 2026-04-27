#include "quad_midbridge_mpcc/full_state_placeholder_solver.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "quad_midbridge_mpcc/mpcc_path_utils.hpp"

namespace quad_midbridge_mpcc
{
namespace
{
constexpr uint32_t FLAG_EMPTY_PATH = 1u << 0;
constexpr uint32_t FLAG_BAD_PROGRESS = 1u << 1;
constexpr uint32_t FLAG_THRUST_CLAMP = 1u << 2;
constexpr uint32_t FLAG_TILT_CLAMP = 1u << 3;
constexpr uint32_t FLAG_YAW_CLAMP = 1u << 4;

class FullStatePlaceholderSolver final : public MpccSolverInterface
{
public:
  std::string name() const override
  {
    return "full_state_placeholder";
  }

  SolverOutput solve(const SolverInput & input) override
  {
    SolverOutput out;
    out.ref.header.stamp = input.stamp;
    out.ref.header.frame_id = "map";
    out.debug.header = out.ref.header;

    if (!input.intent) {
      out.ref.feasible = false;
      out.ref.violation_flags |= FLAG_EMPTY_PATH;
      out.debug.feasible = false;
      out.debug.status = "intent_missing";
      return out;
    }
    if (!input.state.valid || input.intent->centerline_points.size() < 2) {
      out.ref.feasible = false;
      out.ref.violation_flags |= FLAG_EMPTY_PATH;
      out.debug.feasible = false;
      out.debug.status = input.state.valid ? "path_too_short" : "state_invalid";
      return out;
    }

    const auto & intent = *input.intent;
    const PathFrame nearest = projectToPath(intent, input.state.p);
    const double path_len = estimatePathLength(intent);
    if (!nearest.valid || path_len <= 1e-6) {
      out.ref.feasible = false;
      out.ref.violation_flags |= FLAG_BAD_PROGRESS;
      out.debug.feasible = false;
      out.debug.status = "projection_failed";
      return out;
    }

    const double risk = clamp(intent.risk_level, 0.0, 1.0);
    const double contour_w = intent.contour_weight > 1e-3 ? intent.contour_weight : input.params.weight_contour;
    const double progress_w = intent.progress_weight > 1e-3 ? intent.progress_weight : input.params.weight_progress;
    const double speed_cap = std::min(
      static_cast<double>(intent.speed_max > 1e-3 ? intent.speed_max : input.params.max_speed),
      input.params.max_speed * (1.0 - 0.35 * risk));
    const double desired_speed = clamp(static_cast<double>(intent.speed_pref), 0.0, std::max(0.1, speed_cap));

    const double s0 = clamp(std::max(input.state.s_progress, nearest.s), 0.0, path_len);
    const double ds = desired_speed * input.params.horizon_dt;
    const double preview_s = clamp(s0 + std::max(ds, 0.0) * std::max(1, input.params.horizon_steps / 4), 0.0, path_len);
    const PathFrame target = samplePathAtS(intent, preview_s);
    if (!target.valid) {
      out.ref.feasible = false;
      out.ref.violation_flags |= FLAG_BAD_PROGRESS;
      out.debug.feasible = false;
      out.debug.status = "sample_failed";
      return out;
    }

    const Vec3 pos_err = sub(target.position, input.state.p);
    Vec3 v_des = mul(target.tangent, desired_speed);
    Vec3 a_cmd = add(mul(pos_err, 2.0 / std::max(input.params.preview_time * input.params.preview_time, 1e-3)),
                     mul(sub(v_des, input.state.v), 1.2 / std::max(input.params.preview_time, 1e-3)));
    const double a_norm_raw = norm(a_cmd);
    if (a_norm_raw > input.params.max_accel) {
      a_cmd = mul(a_cmd, input.params.max_accel / std::max(a_norm_raw, 1e-6));
    }

    Vec3 j_cmd = mul(sub(a_cmd, input.state.a), 1.0 / std::max(input.params.horizon_dt, 1e-3));
    const double j_norm = norm(j_cmd);
    if (j_norm > input.params.max_jerk) {
      j_cmd = mul(j_cmd, input.params.max_jerk / std::max(j_norm, 1e-6));
    }

    const double tangent_yaw = std::atan2(target.tangent.y, target.tangent.x);
    const double obs_priority = clamp(intent.observation_priority, 0.0, 1.0);
    const double desired_yaw = wrapAngle(obs_priority * intent.yaw_pref + (1.0 - obs_priority) * tangent_yaw);
    double yaw_rate_cmd = wrapAngle(desired_yaw - input.state.psi) / std::max(input.params.preview_time, input.params.horizon_dt);
    if (std::abs(yaw_rate_cmd) > input.params.max_yaw_rate) {
      yaw_rate_cmd = clamp(yaw_rate_cmd, -input.params.max_yaw_rate, input.params.max_yaw_rate);
      out.ref.violation_flags |= FLAG_YAW_CLAMP;
    }

    out.ref.mode = intent.task_mode;
    out.ref.position.x = target.position.x;
    out.ref.position.y = target.position.y;
    out.ref.position.z = target.position.z;
    out.ref.velocity = toRosVector3(v_des);
    out.ref.acceleration = toRosVector3(a_cmd);
    out.ref.jerk = toRosVector3(j_cmd);
    out.ref.yaw = static_cast<float>(desired_yaw);
    out.ref.yaw_rate = static_cast<float>(yaw_rate_cmd);
    out.ref.progress = static_cast<float>(target.s);
    out.ref.contour_error = static_cast<float>(nearest.contour_error);
    out.ref.lag_error = static_cast<float>(nearest.lag_error);

    const Vec3 total_accel{a_cmd.x, a_cmd.y, a_cmd.z + input.params.gravity};
    double thrust = norm(total_accel);
    if (thrust < input.params.thrust_min) {
      thrust = input.params.thrust_min;
      out.ref.violation_flags |= FLAG_THRUST_CLAMP;
    }
    if (thrust > input.params.thrust_max) {
      thrust = input.params.thrust_max;
      out.ref.violation_flags |= FLAG_THRUST_CLAMP;
    }
    out.ref.thrust_nominal = static_cast<float>(thrust);

    const double cos_tilt = clamp(total_accel.z / std::max(norm(total_accel), 1e-6), -1.0, 1.0);
    double tilt = std::acos(cos_tilt);
    if (tilt > input.params.tilt_max_rad) {
      tilt = input.params.tilt_max_rad;
      out.ref.violation_flags |= FLAG_TILT_CLAMP;
    }
    out.ref.tilt_nominal = static_cast<float>(tilt);
    out.ref.feasible = true;

    const double speed_err = norm(sub(v_des, input.state.v));
    const double yaw_err = wrapAngle(desired_yaw - input.state.psi);
    out.debug.objective_value = static_cast<float>(
      contour_w * nearest.contour_error * nearest.contour_error +
      input.params.weight_lag * nearest.lag_error * nearest.lag_error -
      progress_w * desired_speed +
      input.params.weight_yaw * yaw_err * yaw_err +
      input.params.weight_speed * speed_err * speed_err +
      input.params.weight_accel * dot(a_cmd, a_cmd) +
      input.params.weight_jerk * dot(j_cmd, j_cmd));
    out.debug.contour_error = out.ref.contour_error;
    out.debug.lag_error = out.ref.lag_error;
    out.debug.progress_rate = static_cast<float>(desired_speed);
    out.debug.predicted_min_thrust = out.ref.thrust_nominal;
    out.debug.predicted_max_thrust = out.ref.thrust_nominal;
    out.debug.predicted_max_tilt = out.ref.tilt_nominal;
    out.debug.feasible = true;
    out.debug.status = "full_state_placeholder_backend";
    return out;
  }
};
}  // namespace

std::unique_ptr<MpccSolverInterface> createFullStatePlaceholderSolver()
{
  return std::make_unique<FullStatePlaceholderSolver>();
}

}  // namespace quad_midbridge_mpcc
