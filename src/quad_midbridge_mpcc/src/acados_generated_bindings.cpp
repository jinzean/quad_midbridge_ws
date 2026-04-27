#include "quad_midbridge_mpcc/acados_generated_bindings.hpp"

#include <algorithm>
#include <cstring>

namespace quad_midbridge_mpcc
{
namespace
{
constexpr std::size_t kGeneratedParamDim = 16;

void fillCommonParamBlock(
  std::array<double, kGeneratedParamDim> & p,
  const std::array<double, 3> & path_point,
  const std::array<double, 3> & path_tangent,
  double s_ref,
  double yaw_ref,
  double speed_ref,
  double risk,
  const AcadosConstraintBounds & bounds)
{
  p.fill(0.0);
  p[0] = path_point[0];
  p[1] = path_point[1];
  p[2] = path_point[2];
  p[3] = path_tangent[0];
  p[4] = path_tangent[1];
  p[5] = path_tangent[2];
  p[6] = s_ref;
  p[7] = yaw_ref;
  p[8] = speed_ref;
  p[9] = risk;
  p[10] = bounds.corridor_half_width;
  p[11] = bounds.corridor_half_height;
  p[12] = bounds.thrust_min;
  p[13] = bounds.thrust_max;
  p[14] = bounds.tilt_max_rad;
  p[15] = bounds.yaw_rate_max;
}
}  // namespace

bool AcadosGeneratedBindings::compiledIn() const
{
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  return true;
#else
  return false;
#endif
}

std::string AcadosGeneratedBindings::reason() const
{
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  return "generated_bindings_compiled_but_symbol_mapping_pending";
#else
  return "generated_bindings_not_compiled";
#endif
}

bool AcadosGeneratedBindings::create(GeneratedSolverWorkspace & ws) const
{
  if (!compiledIn()) {
    return false;
  }
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  // TODO(jza): map to generated capsule creation and solver creation.
  // Expected sequence here:
  //   ws.capsule = <model>_acados_create_capsule();
  //   <model>_acados_create(ws.capsule);
  ws.created = false;
  ws.capsule = nullptr;
  return false;
#else
  (void)ws;
  return false;
#endif
}

void AcadosGeneratedBindings::destroy(GeneratedSolverWorkspace & ws) const
{
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  // TODO(jza): call generated solver free + free_capsule.
#endif
  ws.created = false;
  ws.capsule = nullptr;
}

bool AcadosGeneratedBindings::setInitialState(
  GeneratedSolverWorkspace & ws,
  const std::array<double, AcadosModelLayout::kNx> & x0) const
{
  (void)ws;
  (void)x0;
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  // TODO(jza): set lbx/ubx for stage 0 to x0.
#endif
  return compiledIn() && ws.created;
}

bool AcadosGeneratedBindings::setStage(
  GeneratedSolverWorkspace & ws,
  int stage_idx,
  const GeneratedStageBuffers & stage) const
{
  (void)ws;
  (void)stage_idx;
  (void)stage;
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  // TODO(jza): set yref, p, and generic bounds for this stage.
#endif
  return compiledIn() && ws.created;
}

bool AcadosGeneratedBindings::setTerminal(
  GeneratedSolverWorkspace & ws,
  const GeneratedTerminalBuffers & term) const
{
  (void)ws;
  (void)term;
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  // TODO(jza): set terminal yref_e / p_e.
#endif
  return compiledIn() && ws.created;
}

bool AcadosGeneratedBindings::setWarmStart(
  GeneratedSolverWorkspace & ws,
  const std::array<double, AcadosModelLayout::kNu> & u_prev) const
{
  (void)ws;
  (void)u_prev;
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  // TODO(jza): warm-start first control stage if your generated interface exposes it.
#endif
  return compiledIn() && ws.created;
}

bool AcadosGeneratedBindings::solve(GeneratedSolverWorkspace & ws, int & status_code) const
{
  (void)ws;
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  // TODO(jza): status_code = <model>_acados_solve(ws.capsule);
  status_code = -99;
  return false;
#else
  status_code = -1;
  return false;
#endif
}

bool AcadosGeneratedBindings::getFirstControl(
  GeneratedSolverWorkspace & ws,
  std::array<double, AcadosModelLayout::kNu> & u0) const
{
  (void)ws;
  u0.fill(0.0);
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  // TODO(jza): ocp_nlp_out_get(..., 0, "u", u0.data());
#endif
  return compiledIn() && ws.created;
}

bool AcadosGeneratedBindings::getNextState(
  GeneratedSolverWorkspace & ws,
  std::array<double, AcadosModelLayout::kNx> & x1) const
{
  (void)ws;
  x1.fill(0.0);
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  // TODO(jza): ocp_nlp_out_get(..., 1, "x", x1.data());
#endif
  return compiledIn() && ws.created;
}

bool AcadosGeneratedBindings::getCost(GeneratedSolverWorkspace & ws, double & cost) const
{
  (void)ws;
  cost = 0.0;
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  // TODO(jza): ocp_nlp_eval_cost(...) or out_get depending on generated API.
#endif
  return compiledIn() && ws.created;
}

GeneratedStageBuffers buildGeneratedStageBuffers(const AcadosStageData & stage)
{
  GeneratedStageBuffers out;
  out.x_ref = stage.x_ref;
  out.u_ref = stage.u_ref;
  fillCommonParamBlock(
    out.p,
    stage.path_point,
    stage.path_tangent,
    stage.s_ref,
    stage.yaw_ref,
    stage.speed_ref,
    stage.risk_level,
    stage.bounds);

  out.lh = {
    -stage.bounds.corridor_half_width,
    -stage.bounds.corridor_half_height,
    -stage.bounds.tilt_max_rad,
    stage.bounds.thrust_min,
    -stage.bounds.yaw_rate_max,
    stage.bounds.allow_reverse_progress ? -stage.bounds.speed_max : 0.0};

  out.uh = {
    stage.bounds.corridor_half_width,
    stage.bounds.corridor_half_height,
    stage.bounds.tilt_max_rad,
    stage.bounds.thrust_max,
    stage.bounds.yaw_rate_max,
    stage.bounds.speed_max};
  return out;
}

GeneratedTerminalBuffers buildGeneratedTerminalBuffers(const AcadosTerminalData & term)
{
  GeneratedTerminalBuffers out;
  out.x_ref = term.x_ref;
  fillCommonParamBlock(
    out.p,
    term.path_point,
    term.path_tangent,
    term.s_ref,
    term.yaw_ref,
    0.0,
    0.0,
    term.bounds);
  return out;
}

}  // namespace quad_midbridge_mpcc
