#include "quad_midbridge_mpcc/acados_generated_solver.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "quad_midbridge_mpcc/acados_generated_adapter.hpp"
#include "quad_midbridge_mpcc/acados_problem_builder.hpp"
#include "quad_midbridge_mpcc/full_state_placeholder_solver.hpp"
#include "quad_midbridge_mpcc/mpcc_path_utils.hpp"

namespace quad_midbridge_mpcc
{
namespace
{
constexpr uint32_t FLAG_BACKEND_GENERATED_FALLBACK = 1u << 11;
constexpr uint32_t FLAG_BACKEND_GENERATED_OK = 1u << 12;

Vec3 unpackVec3(const std::array<double, AcadosModelLayout::kNx> & x, size_t offset)
{
  return {x[offset + 0], x[offset + 1], x[offset + 2]};
}

Vec3 unpackJerk(const std::array<double, AcadosModelLayout::kNu> & u)
{
  return {u[0], u[1], u[2]};
}

SolverOutput packGeneratedSolution(
  const SolverInput & input,
  const AcadosProblemData & problem,
  const AcadosGeneratedAdapterResult & solved)
{
  SolverOutput out;
  out.ref.header.stamp = input.stamp;
  out.ref.header.frame_id = "map";
  out.debug.header = out.ref.header;

  const Vec3 p = unpackVec3(solved.x1, 0);
  const Vec3 v = unpackVec3(solved.x1, 3);
  const Vec3 a = unpackVec3(solved.x1, 6);
  const Vec3 j = unpackJerk(solved.u0);
  const double yaw = solved.x1[9];
  const double s = solved.x1[10];
  const double yaw_rate = solved.u0[3];

  const auto & stage0 = problem.stages.front();
  const Vec3 path_point{stage0.path_point[0], stage0.path_point[1], stage0.path_point[2]};
  const Vec3 path_tangent{stage0.path_tangent[0], stage0.path_tangent[1], stage0.path_tangent[2]};
  const Vec3 err = sub(p, path_point);
  const double lag_error = dot(err, path_tangent);
  const Vec3 contour_vec = sub(err, mul(path_tangent, lag_error));
  const double contour_error = norm(contour_vec);

  out.ref.mode = input.intent ? input.intent->task_mode : 0u;
  out.ref.position.x = p.x;
  out.ref.position.y = p.y;
  out.ref.position.z = p.z;
  out.ref.velocity = toRosVector3(v);
  out.ref.acceleration = toRosVector3(a);
  out.ref.jerk = toRosVector3(j);
  out.ref.yaw = static_cast<float>(yaw);
  out.ref.yaw_rate = static_cast<float>(yaw_rate);
  out.ref.progress = static_cast<float>(s);
  out.ref.contour_error = static_cast<float>(contour_error);
  out.ref.lag_error = static_cast<float>(lag_error);

  const Vec3 total_accel{a.x, a.y, a.z + input.params.gravity};
  const double thrust = clamp(norm(total_accel), problem.nominal_bounds.thrust_min, problem.nominal_bounds.thrust_max);
  const double cos_tilt = clamp(total_accel.z / std::max(norm(total_accel), 1e-6), -1.0, 1.0);
  const double tilt = std::min(std::acos(cos_tilt), problem.nominal_bounds.tilt_max_rad);
  out.ref.thrust_nominal = static_cast<float>(thrust);
  out.ref.tilt_nominal = static_cast<float>(tilt);
  out.ref.violation_flags |= FLAG_BACKEND_GENERATED_OK;
  out.ref.feasible = true;

  out.debug.objective_value = static_cast<float>(solved.objective_value);
  out.debug.contour_error = out.ref.contour_error;
  out.debug.lag_error = out.ref.lag_error;
  out.debug.progress_rate = static_cast<float>(solved.u0[4]);
  out.debug.predicted_min_thrust = out.ref.thrust_nominal;
  out.debug.predicted_max_thrust = out.ref.thrust_nominal;
  out.debug.predicted_max_tilt = out.ref.tilt_nominal;
  out.debug.feasible = true;
  out.debug.status = solved.status;
  return out;
}

class AcadosGeneratedSolver final : public MpccSolverInterface
{
public:
  AcadosGeneratedSolver()
  : fallback_(createFullStatePlaceholderSolver())
  {
  }

  std::string name() const override
  {
    return "acados_generated";
  }

  SolverOutput solve(const SolverInput & input) override
  {
    const AcadosProblemData problem = buildAcadosProblem(input);
    if (problem.backend_status != "problem_ready") {
      SolverOutput failed;
      failed.ref.header.stamp = input.stamp;
      failed.debug.header = failed.ref.header;
      failed.ref.feasible = false;
      failed.debug.feasible = false;
      failed.debug.status = problem.backend_status;
      return failed;
    }

    const AcadosGeneratedAdapterResult solved = adapter_.solve(problem);
    if (solved.ok) {
      return packGeneratedSolution(input, problem, solved);
    }

    SolverOutput out = fallback_->solve(input);
    out.ref.violation_flags |= FLAG_BACKEND_GENERATED_FALLBACK;
    out.debug.status = std::string("acados_generated_fallback:") + solved.status;
    if (!adapter_.available()) {
      out.debug.status += std::string("|") + adapter_.availabilityReason();
    }
    out.debug.predicted_min_thrust = static_cast<float>(problem.nominal_bounds.thrust_min);
    out.debug.predicted_max_thrust = static_cast<float>(problem.nominal_bounds.thrust_max);
    out.debug.predicted_max_tilt = static_cast<float>(problem.nominal_bounds.tilt_max_rad);
    return out;
  }

private:
  AcadosGeneratedAdapter adapter_{};
  std::unique_ptr<MpccSolverInterface> fallback_;
};
}  // namespace

std::unique_ptr<MpccSolverInterface> createAcadosGeneratedSolver()
{
  return std::make_unique<AcadosGeneratedSolver>();
}

}  // namespace quad_midbridge_mpcc
