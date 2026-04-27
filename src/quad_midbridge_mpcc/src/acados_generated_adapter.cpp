#include "quad_midbridge_mpcc/acados_generated_adapter.hpp"

#include <sstream>

namespace quad_midbridge_mpcc
{

bool AcadosGeneratedAdapter::available() const
{
  return bindings_.compiledIn();
}

std::string AcadosGeneratedAdapter::availabilityReason() const
{
  return bindings_.reason();
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

AcadosGeneratedAdapterResult AcadosGeneratedAdapter::solve(const AcadosProblemData & problem) const
{
  AcadosGeneratedAdapterResult out;
  if (problem.backend_status != "problem_ready") {
    out.status = problem.backend_status;
    return out;
  }

  GeneratedSolverWorkspace ws;
  if (!bindings_.create(ws)) {
    out.status = std::string("create_failed:") + bindings_.reason();
    return out;
  }

  std::string push_status;
  if (!pushProblem(ws, problem, push_status)) {
    bindings_.destroy(ws);
    out.status = push_status;
    return out;
  }

  int solve_status = -999;
  if (!bindings_.solve(ws, solve_status)) {
    bindings_.destroy(ws);
    out.used_generated_code = true;
    out.status = std::string("solve_failed:status=") + std::to_string(solve_status);
    return out;
  }

  if (!bindings_.getFirstControl(ws, out.u0)) {
    bindings_.destroy(ws);
    out.used_generated_code = true;
    out.status = "extract_u0_failed";
    return out;
  }

  if (!bindings_.getNextState(ws, out.x1)) {
    bindings_.destroy(ws);
    out.used_generated_code = true;
    out.status = "extract_x1_failed";
    return out;
  }

  double cost = 0.0;
  bindings_.getCost(ws, cost);
  bindings_.destroy(ws);

  out.ok = true;
  out.used_generated_code = true;
  out.status = "generated_code_solved";
  out.objective_value = cost;
  return out;
}

}  // namespace quad_midbridge_mpcc
