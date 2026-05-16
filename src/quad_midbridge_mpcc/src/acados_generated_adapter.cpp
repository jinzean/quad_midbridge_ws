#include "quad_midbridge_mpcc/acados_generated_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace quad_midbridge_mpcc
{

namespace
{

bool finiteArray(const std::array<double, AcadosModelLayout::kNu> & a)
{
  for (double v : a) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

bool finiteArrayX(const std::array<double, AcadosModelLayout::kNx> & a)
{
  for (double v : a) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

bool generatedSolutionWithinBounds(
  const AcadosProblemData & problem,
  const std::array<double, AcadosModelLayout::kNu> & u0,
  const std::array<double, AcadosModelLayout::kNx> & x1,
  std::string & reason)
{
  if (!finiteArray(u0) || !finiteArrayX(x1)) {
    reason = "solution_non_finite";
    return false;
  }

  const auto & b = problem.nominal_bounds;
  const double jerk_norm = std::sqrt(u0[0] * u0[0] + u0[1] * u0[1] + u0[2] * u0[2]);
  const double yaw_rate = std::abs(u0[3]);
  const double s_dot = u0[4];

  const double jerk_limit = std::max(1.0e-6, b.jerk_max);
  const double yaw_rate_limit = std::max(1.0e-6, b.yaw_rate_max);
  const double s_dot_upper = std::max(1.0e-6, b.speed_max);
  const double s_dot_lower = b.allow_reverse_progress ? -s_dot_upper : -1.0e-9;

  if (jerk_norm > 1.25 * jerk_limit) {
    reason = "solution_jerk_out_of_bounds";
    return false;
  }

  if (yaw_rate > 1.25 * yaw_rate_limit) {
    reason = "solution_yaw_rate_out_of_bounds";
    return false;
  }

  if (s_dot < s_dot_lower || s_dot > 1.25 * s_dot_upper) {
    reason = "solution_progress_rate_out_of_bounds";
    return false;
  }

  const double vx = x1[3];
  const double vy = x1[4];
  const double vz = x1[5];
  const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
  if (speed > 1.25 * std::max(1.0e-6, b.speed_max)) {
    reason = "solution_speed_out_of_bounds";
    return false;
  }

  const double ax = x1[6];
  const double ay = x1[7];
  const double az = x1[8];
  const double accel = std::sqrt(ax * ax + ay * ay + az * az);
  if (accel > 1.25 * std::max(1.0e-6, b.accel_max)) {
    reason = "solution_accel_out_of_bounds";
    return false;
  }

  return true;
}

}  // namespace

bool AcadosGeneratedAdapter::available() const
{
  return bindings_.compiledIn();
}

AcadosGeneratedAdapter::~AcadosGeneratedAdapter()
{
  resetWorkspace();
}

std::string AcadosGeneratedAdapter::availabilityReason() const
{
  return bindings_.reason();
}

void AcadosGeneratedAdapter::resetWorkspace()
{
  bindings_.destroy(workspace_);
}

bool AcadosGeneratedAdapter::pushProblem(
  GeneratedSolverWorkspace & ws,
  const AcadosProblemData & problem,
  std::string & status) const
{
  if (!bindings_.setInitialState(ws, problem.x0)) {
    status = "set_initial_state_failed";
    return false;
  }

  for (std::size_t k = 0; k < problem.stages.size(); ++k) {
    const auto stage = buildGeneratedStageBuffers(problem.stages[k]);
    if (!bindings_.setStage(ws, static_cast<int>(k), stage)) {
      std::ostringstream oss;
      oss << "set_stage_failed:k=" << k;
      status = oss.str();
      return false;
    }
  }

  const auto term = buildGeneratedTerminalBuffers(problem.terminal);
  if (!bindings_.setTerminal(ws, term)) {
    status = "set_terminal_failed";
    return false;
  }

  if (!bindings_.setWarmStart(ws, problem.u_prev)) {
    status = "set_warm_start_failed";
    return false;
  }

  status = "problem_pushed";
  return true;
}

AcadosGeneratedAdapterResult AcadosGeneratedAdapter::solve(const AcadosProblemData & problem)
{
  AcadosGeneratedAdapterResult out;
  if (problem.backend_status != "problem_ready") {
    out.status = problem.backend_status;
    return out;
  }

  if (!bindings_.create(workspace_)) {
    out.status = std::string("create_failed:") + bindings_.reason();
    return out;
  }

  std::string push_status;
  if (!pushProblem(workspace_, problem, push_status)) {
    resetWorkspace();
    out.status = push_status;
    return out;
  }

  int solve_status = -999;
  if (!bindings_.solve(workspace_, solve_status)) {
    resetWorkspace();
    out.used_generated_code = true;
    out.status = std::string("solve_failed:status=") + std::to_string(solve_status);
    return out;
  }

  if (!bindings_.getFirstControl(workspace_, out.u0)) {
    resetWorkspace();
    out.used_generated_code = true;
    out.status = "extract_u0_failed";
    return out;
  }

  if (!bindings_.getNextState(workspace_, out.x1)) {
    resetWorkspace();
    out.used_generated_code = true;
    out.status = "extract_x1_failed";
    return out;
  }

  std::string validation_reason;
  if (!generatedSolutionWithinBounds(problem, out.u0, out.x1, validation_reason)) {
    out.used_generated_code = true;
    out.status = std::string("solution_rejected:") + validation_reason;
    return out;
  }

  double cost = 0.0;
  bindings_.getCost(workspace_, cost);

  out.ok = true;
  out.used_generated_code = true;
  out.status = "generated_code_solved";
  out.objective_value = cost;
  return out;
}

}  // namespace quad_midbridge_mpcc
