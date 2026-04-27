#include "quad_midbridge_mpcc/acados_skeleton_solver.hpp"

#include <memory>
#include <string>

#include "quad_midbridge_mpcc/acados_problem_builder.hpp"
#include "quad_midbridge_mpcc/full_state_placeholder_solver.hpp"

namespace quad_midbridge_mpcc
{
namespace
{
constexpr uint32_t FLAG_BACKEND_PLACEHOLDER = 1u << 10;

class AcadosSkeletonSolver final : public MpccSolverInterface
{
public:
  AcadosSkeletonSolver()
  : fallback_(createFullStatePlaceholderSolver())
  {
  }

  std::string name() const override
  {
    return "acados_skeleton";
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

    SolverOutput out = fallback_->solve(input);
    out.ref.violation_flags |= FLAG_BACKEND_PLACEHOLDER;
    out.debug.status = "acados_skeleton_problem_ready";
    out.debug.objective_value = static_cast<float>(
      problem.nominal_weights.contour * out.ref.contour_error * out.ref.contour_error +
      problem.nominal_weights.lag * out.ref.lag_error * out.ref.lag_error -
      problem.nominal_weights.progress * problem.desired_speed);
    out.debug.progress_rate = static_cast<float>(problem.desired_speed);
    out.debug.predicted_min_thrust = static_cast<float>(problem.nominal_bounds.thrust_min);
    out.debug.predicted_max_thrust = static_cast<float>(problem.nominal_bounds.thrust_max);
    out.debug.predicted_max_tilt = static_cast<float>(problem.nominal_bounds.tilt_max_rad);
    return out;
  }

private:
  std::unique_ptr<MpccSolverInterface> fallback_;
};
}  // namespace

std::unique_ptr<MpccSolverInterface> createAcadosSkeletonSolver()
{
  return std::make_unique<AcadosSkeletonSolver>();
}

}  // namespace quad_midbridge_mpcc
