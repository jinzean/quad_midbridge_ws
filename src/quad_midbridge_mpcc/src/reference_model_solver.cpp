#include "quad_midbridge_mpcc/reference_model_solver.hpp"

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

class ReferenceModelSolver final : public MpccSolverInterface
{
public:
  std::string name() const override
  {
    return "reference_model";
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

    out.ref.mode = input.intent->task_mode;
    out.ref.yaw = input.intent->yaw_pref;
    out.ref.yaw_rate = input.intent->yaw_rate_pref;

    if (!input.state.valid || input.intent->centerline_points.size() < 2) {
      out.ref.feasible = false;
      out.ref.violation_flags |= FLAG_EMPTY_PATH;
      out.debug.feasible = false;
      out.debug.status = input.state.valid ? "path_too_short" : "state_invalid";
      return out;
    }

    const PathFrame nearest = projectToPath(*input.intent, input.state.p);
    const double path_len = estimatePathLength(*input.intent);
    const double desired_speed = clamp(
      input.intent->speed_pref,
      0.0,
      input.intent->speed_max > 1e-3 ? input.intent->speed_max : input.params.max_speed);
    const double preview_s = clamp(nearest.s + std::max(0.0, desired_speed) * input.params.preview_time, 0.0, path_len);
    const PathFrame target = samplePathAtS(*input.intent, preview_s);

    if (!nearest.valid || !target.valid) {
      out.ref.feasible = false;
      out.ref.violation_flags |= FLAG_BAD_PROGRESS;
      out.debug.feasible = false;
      out.debug.status = "path_projection_failed";
      return out;
    }

    out.ref.position.x = target.position.x;
    out.ref.position.y = target.position.y;
    out.ref.position.z = target.position.z;

    Vec3 v_ref = mul(target.tangent, desired_speed);
    out.ref.velocity = toRosVector3(v_ref);

    Vec3 a_ref = mul(sub(v_ref, input.state.v), 1.0 / std::max(input.params.horizon_dt, 1e-3));
    const double a_norm = norm(a_ref);
    if (a_norm > input.params.max_accel) {
      a_ref = mul(a_ref, input.params.max_accel / a_norm);
    }
    out.ref.acceleration = toRosVector3(a_ref);

    Vec3 j_ref = mul(sub(a_ref, input.state.a), 1.0 / std::max(input.params.horizon_dt, 1e-3));
    const double j_norm = norm(j_ref);
    if (j_norm > input.params.max_jerk) {
      j_ref = mul(j_ref, input.params.max_jerk / j_norm);
    }
    out.ref.jerk = toRosVector3(j_ref);

    out.ref.progress = static_cast<float>(target.s);
    out.ref.contour_error = static_cast<float>(nearest.contour_error);
    out.ref.lag_error = static_cast<float>(nearest.lag_error);

    const Vec3 total_accel{a_ref.x, a_ref.y, a_ref.z + input.params.gravity};
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

    const double tangent_yaw = std::atan2(target.tangent.y, target.tangent.x);
    const double obs_priority = clamp(input.intent->observation_priority, 0.0, 1.0);
    const double blend = 1.0 - obs_priority;
    const double yaw_err = wrapAngle(tangent_yaw - input.intent->yaw_pref);
    out.ref.yaw = static_cast<float>(wrapAngle(input.intent->yaw_pref + blend * yaw_err));
    out.ref.yaw_rate = static_cast<float>(clamp(input.intent->yaw_rate_pref, -input.params.max_yaw_rate, input.params.max_yaw_rate));
    out.ref.feasible = true;

    out.debug.objective_value = static_cast<float>(
      input.params.weight_contour * nearest.contour_error * nearest.contour_error -
      input.params.weight_progress * desired_speed +
      input.params.weight_accel * a_norm * a_norm +
      input.params.weight_jerk * j_norm * j_norm);
    out.debug.contour_error = out.ref.contour_error;
    out.debug.lag_error = out.ref.lag_error;
    out.debug.progress_rate = static_cast<float>(desired_speed);
    out.debug.predicted_min_thrust = out.ref.thrust_nominal;
    out.debug.predicted_max_thrust = out.ref.thrust_nominal;
    out.debug.predicted_max_tilt = out.ref.tilt_nominal;
    out.debug.feasible = true;
    out.debug.status = "reference_model_backend";
    return out;
  }
};
}  // namespace

std::unique_ptr<MpccSolverInterface> createReferenceModelSolver()
{
  return std::make_unique<ReferenceModelSolver>();
}

}  // namespace quad_midbridge_mpcc
