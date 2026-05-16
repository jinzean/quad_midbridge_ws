#include "quad_midbridge_mpcc/acados_generated_bindings.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_quad_midbridge_mpcc_ocp.h"
}
#endif

namespace quad_midbridge_mpcc
{
namespace
{
// Keep synchronized with tools/generate_mpcc_acados_solver.py.
constexpr int P_PATH_POINT = 0;
constexpr int P_PATH_TANGENT = 3;
constexpr int P_S_REF = 6;
constexpr int P_YAW_REF = 7;
constexpr int P_SPEED_REF = 8;
constexpr int P_RISK = 9;
constexpr int P_CORRIDOR_HALF_WIDTH = 10;
constexpr int P_CORRIDOR_HALF_HEIGHT = 11;
constexpr int P_THRUST_MIN = 12;
constexpr int P_THRUST_MAX = 13;
constexpr int P_TILT_MAX = 14;
constexpr int P_YAW_RATE_MAX = 15;
constexpr int P_W_CONTOUR = 16;
constexpr int P_W_LAG = 17;
constexpr int P_W_PROGRESS = 18;
constexpr int P_W_YAW = 19;
constexpr int P_W_SPEED = 20;
constexpr int P_W_ACCEL = 21;
constexpr int P_W_JERK = 22;
constexpr int P_W_DELTA_JERK = 23;
constexpr int P_W_DELTA_YAW_RATE = 24;
constexpr int P_W_TERMINAL_CONTOUR = 25;
constexpr int P_W_TERMINAL_LAG = 26;
constexpr int P_W_TERMINAL_YAW = 27;
constexpr int P_GRAVITY = 28;
constexpr int P_EPS = 29;

void fillCommonParamBlock(
  std::array<double, kGeneratedParamDim> & p,
  const std::array<double, 3> & path_point,
  const std::array<double, 3> & path_tangent,
  double s_ref,
  double yaw_ref,
  double speed_ref,
  double risk,
  double gravity,
  const AcadosCostWeights & weights,
  const AcadosConstraintBounds & bounds)
{
  p.fill(0.0);
  p[P_PATH_POINT + 0] = path_point[0];
  p[P_PATH_POINT + 1] = path_point[1];
  p[P_PATH_POINT + 2] = path_point[2];
  p[P_PATH_TANGENT + 0] = path_tangent[0];
  p[P_PATH_TANGENT + 1] = path_tangent[1];
  p[P_PATH_TANGENT + 2] = path_tangent[2];
  p[P_S_REF] = s_ref;
  p[P_YAW_REF] = yaw_ref;
  p[P_SPEED_REF] = speed_ref;
  p[P_RISK] = risk;
  p[P_CORRIDOR_HALF_WIDTH] = bounds.corridor_half_width;
  p[P_CORRIDOR_HALF_HEIGHT] = bounds.corridor_half_height;
  p[P_THRUST_MIN] = bounds.thrust_min;
  p[P_THRUST_MAX] = bounds.thrust_max;
  p[P_TILT_MAX] = bounds.tilt_max_rad;
  p[P_YAW_RATE_MAX] = bounds.yaw_rate_max;
  p[P_W_CONTOUR] = weights.contour;
  p[P_W_LAG] = weights.lag;
  p[P_W_PROGRESS] = weights.progress;
  p[P_W_YAW] = weights.yaw;
  p[P_W_SPEED] = weights.speed;
  p[P_W_ACCEL] = weights.accel;
  p[P_W_JERK] = weights.jerk;
  p[P_W_DELTA_JERK] = weights.delta_jerk;
  p[P_W_DELTA_YAW_RATE] = weights.delta_yaw_rate;
  p[P_W_TERMINAL_CONTOUR] = weights.terminal_contour;
  p[P_W_TERMINAL_LAG] = weights.terminal_lag;
  p[P_W_TERMINAL_YAW] = weights.terminal_yaw;
  p[P_GRAVITY] = std::isfinite(gravity) && gravity > 1.0e-6 ? gravity : 9.81;
  p[P_EPS] = 1.0e-6;
}

#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
using Capsule = quad_midbridge_mpcc_ocp_solver_capsule;

Capsule * capsule(GeneratedSolverWorkspace & ws)
{
  return static_cast<Capsule *>(ws.capsule);
}

#endif
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
  return "generated_bindings_compiled:quad_midbridge_mpcc_ocp";
#else
  return "generated_bindings_not_compiled";
#endif
}

bool AcadosGeneratedBindings::create(GeneratedSolverWorkspace & ws) const
{
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  if (ws.created) {
    return true;
  }
  Capsule * cap = quad_midbridge_mpcc_ocp_acados_create_capsule();
  if (cap == nullptr) {
    ws.capsule = nullptr;
    ws.created = false;
    return false;
  }
  const int status = quad_midbridge_mpcc_ocp_acados_create(cap);
  if (status != 0) {
    quad_midbridge_mpcc_ocp_acados_free_capsule(cap);
    ws.capsule = nullptr;
    ws.created = false;
    return false;
  }
  ws.capsule = cap;
  ws.created = true;
  return true;
#else
  (void)ws;
  return false;
#endif
}

void AcadosGeneratedBindings::destroy(GeneratedSolverWorkspace & ws) const
{
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  if (ws.created && ws.capsule != nullptr) {
    Capsule * cap = capsule(ws);
    quad_midbridge_mpcc_ocp_acados_free(cap);
    quad_midbridge_mpcc_ocp_acados_free_capsule(cap);
  }
#else
  (void)ws;
#endif
  ws.created = false;
  ws.capsule = nullptr;
}

bool AcadosGeneratedBindings::setInitialState(
  GeneratedSolverWorkspace & ws,
  const std::array<double, AcadosModelLayout::kNx> & x0) const
{
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  if (!ws.created || ws.capsule == nullptr) {
    return false;
  }
  Capsule * cap = capsule(ws);
  ocp_nlp_config * config = quad_midbridge_mpcc_ocp_acados_get_nlp_config(cap);
  ocp_nlp_dims * dims = quad_midbridge_mpcc_ocp_acados_get_nlp_dims(cap);
  ocp_nlp_in * in = quad_midbridge_mpcc_ocp_acados_get_nlp_in(cap);
  ocp_nlp_out * out = quad_midbridge_mpcc_ocp_acados_get_nlp_out(cap);

  auto * x0_mut = const_cast<double *>(x0.data());
  ocp_nlp_constraints_model_set(config, dims, in, out, 0, "lbx", x0_mut);
  ocp_nlp_constraints_model_set(config, dims, in, out, 0, "ubx", x0_mut);
  ocp_nlp_out_set(config, dims, out, in, 0, "x", x0_mut);
  return true;
#else
  (void)ws;
  (void)x0;
  return false;
#endif
}

bool AcadosGeneratedBindings::setStage(
  GeneratedSolverWorkspace & ws,
  int stage_idx,
  const GeneratedStageBuffers & stage) const
{
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  if (!ws.created || ws.capsule == nullptr || stage_idx < 0) {
    return false;
  }
  Capsule * cap = capsule(ws);
  ocp_nlp_config * config = quad_midbridge_mpcc_ocp_acados_get_nlp_config(cap);
  ocp_nlp_dims * dims = quad_midbridge_mpcc_ocp_acados_get_nlp_dims(cap);
  ocp_nlp_in * in = quad_midbridge_mpcc_ocp_acados_get_nlp_in(cap);
  ocp_nlp_out * out = quad_midbridge_mpcc_ocp_acados_get_nlp_out(cap);

  auto * p_mut = const_cast<double *>(stage.p.data());
  const int p_status = quad_midbridge_mpcc_ocp_acados_update_params(
    cap, stage_idx, p_mut, static_cast<int>(stage.p.size()));
  if (p_status != 0) {
    return false;
  }

  auto * lh_mut = const_cast<double *>(stage.lh.data());
  auto * uh_mut = const_cast<double *>(stage.uh.data());
  ocp_nlp_constraints_model_set(config, dims, in, out, stage_idx, "lh", lh_mut);
  ocp_nlp_constraints_model_set(config, dims, in, out, stage_idx, "uh", uh_mut);

  auto * lbu_mut = const_cast<double *>(stage.lbu.data());
  auto * ubu_mut = const_cast<double *>(stage.ubu.data());
  ocp_nlp_constraints_model_set(config, dims, in, out, stage_idx, "lbu", lbu_mut);
  ocp_nlp_constraints_model_set(config, dims, in, out, stage_idx, "ubu", ubu_mut);

  auto * x_ref_mut = const_cast<double *>(stage.x_ref.data());
  auto * u_ref_mut = const_cast<double *>(stage.u_ref.data());
  ocp_nlp_out_set(config, dims, out, in, stage_idx, "x", x_ref_mut);
  ocp_nlp_out_set(config, dims, out, in, stage_idx, "u", u_ref_mut);
  return true;
#else
  (void)ws;
  (void)stage_idx;
  (void)stage;
  return false;
#endif
}

bool AcadosGeneratedBindings::setTerminal(
  GeneratedSolverWorkspace & ws,
  const GeneratedTerminalBuffers & term) const
{
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  if (!ws.created || ws.capsule == nullptr) {
    return false;
  }
  Capsule * cap = capsule(ws);
  const int stage_idx = QUAD_MIDBRIDGE_MPCC_OCP_N;
  auto * p_mut = const_cast<double *>(term.p.data());
  const int p_status = quad_midbridge_mpcc_ocp_acados_update_params(
    cap, stage_idx, p_mut, static_cast<int>(term.p.size()));
  if (p_status != 0) {
    return false;
  }
  ocp_nlp_config * config = quad_midbridge_mpcc_ocp_acados_get_nlp_config(cap);
  ocp_nlp_dims * dims = quad_midbridge_mpcc_ocp_acados_get_nlp_dims(cap);
  ocp_nlp_in * in = quad_midbridge_mpcc_ocp_acados_get_nlp_in(cap);
  ocp_nlp_out * out = quad_midbridge_mpcc_ocp_acados_get_nlp_out(cap);
  auto * x_ref_mut = const_cast<double *>(term.x_ref.data());
  ocp_nlp_out_set(config, dims, out, in, stage_idx, "x", x_ref_mut);
  return true;
#else
  (void)ws;
  (void)term;
  return false;
#endif
}

bool AcadosGeneratedBindings::setWarmStart(
  GeneratedSolverWorkspace & ws,
  const std::array<double, AcadosModelLayout::kNu> & u_prev) const
{
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  if (!ws.created || ws.capsule == nullptr) {
    return false;
  }
  Capsule * cap = capsule(ws);
  ocp_nlp_config * config = quad_midbridge_mpcc_ocp_acados_get_nlp_config(cap);
  ocp_nlp_dims * dims = quad_midbridge_mpcc_ocp_acados_get_nlp_dims(cap);
  ocp_nlp_in * in = quad_midbridge_mpcc_ocp_acados_get_nlp_in(cap);
  ocp_nlp_out * out = quad_midbridge_mpcc_ocp_acados_get_nlp_out(cap);
  auto * u_prev_mut = const_cast<double *>(u_prev.data());
  ocp_nlp_out_set(config, dims, out, in, 0, "u", u_prev_mut);
  return true;
#else
  (void)ws;
  (void)u_prev;
  return false;
#endif
}

bool AcadosGeneratedBindings::solve(GeneratedSolverWorkspace & ws, int & status_code) const
{
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  if (!ws.created || ws.capsule == nullptr) {
    status_code = -100;
    return false;
  }
  status_code = quad_midbridge_mpcc_ocp_acados_solve(capsule(ws));
  return status_code == 0;
#else
  (void)ws;
  status_code = -1;
  return false;
#endif
}

bool AcadosGeneratedBindings::getFirstControl(
  GeneratedSolverWorkspace & ws,
  std::array<double, AcadosModelLayout::kNu> & u0) const
{
  u0.fill(0.0);
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  if (!ws.created || ws.capsule == nullptr) {
    return false;
  }
  Capsule * cap = capsule(ws);
  ocp_nlp_config * config = quad_midbridge_mpcc_ocp_acados_get_nlp_config(cap);
  ocp_nlp_dims * dims = quad_midbridge_mpcc_ocp_acados_get_nlp_dims(cap);
  ocp_nlp_out * out = quad_midbridge_mpcc_ocp_acados_get_nlp_out(cap);
  ocp_nlp_out_get(config, dims, out, 0, "u", u0.data());
  return true;
#else
  (void)ws;
  return false;
#endif
}

bool AcadosGeneratedBindings::getNextState(
  GeneratedSolverWorkspace & ws,
  std::array<double, AcadosModelLayout::kNx> & x1) const
{
  x1.fill(0.0);
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  if (!ws.created || ws.capsule == nullptr) {
    return false;
  }
  Capsule * cap = capsule(ws);
  ocp_nlp_config * config = quad_midbridge_mpcc_ocp_acados_get_nlp_config(cap);
  ocp_nlp_dims * dims = quad_midbridge_mpcc_ocp_acados_get_nlp_dims(cap);
  ocp_nlp_out * out = quad_midbridge_mpcc_ocp_acados_get_nlp_out(cap);
  ocp_nlp_out_get(config, dims, out, 1, "x", x1.data());
  return true;
#else
  (void)ws;
  return false;
#endif
}

bool AcadosGeneratedBindings::getCost(GeneratedSolverWorkspace & ws, double & cost) const
{
  cost = 0.0;
#ifdef QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED
  if (!ws.created || ws.capsule == nullptr) {
    return false;
  }
  // The generated API does not expose a stable cost getter across all acados
  // versions.  Keep this non-critical: objective_value in MpccDebug will be 0
  // unless you add a version-specific ocp_nlp_get("cost_value", ...) call here.
  return true;
#else
  (void)ws;
  return false;
#endif
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
    stage.gravity,
    stage.weights,
    stage.bounds);

  // Constraint order must match model.con_h_expr in generate_mpcc_acados_solver.py:
  // [contour_xy, vertical_err, tilt, thrust, psi_dot, s_dot, speed_norm, accel_norm, jerk_norm]
  const bool hold_progress = stage.bounds.terminal_hold || stage.speed_ref <= 1.0e-4;

  const double s_dot_lower =
    hold_progress ? 0.0 :
    (stage.bounds.allow_reverse_progress ? -stage.bounds.speed_max : 0.0);

  const double s_dot_upper =
    hold_progress ? 0.0 : stage.bounds.speed_max;

  out.lbu = {
    -stage.bounds.jerk_max,
    -stage.bounds.jerk_max,
    -stage.bounds.jerk_max,
    -stage.bounds.yaw_rate_max,
    s_dot_lower};
  out.ubu = {
    stage.bounds.jerk_max,
    stage.bounds.jerk_max,
    stage.bounds.jerk_max,
    stage.bounds.yaw_rate_max,
    s_dot_upper};

  out.lh = {
    0.0,
    -stage.bounds.corridor_half_height,
    0.0,
    stage.bounds.thrust_min,
    -stage.bounds.yaw_rate_max,
    s_dot_lower,
    0.0,
    0.0,
    0.0};

  out.uh = {
    stage.bounds.corridor_half_width,
    stage.bounds.corridor_half_height,
    stage.bounds.tilt_max_rad,
    stage.bounds.thrust_max,
    stage.bounds.yaw_rate_max,
    s_dot_upper,
    stage.bounds.speed_max,
    stage.bounds.accel_max,
    stage.bounds.jerk_max};
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
    term.gravity,
    term.weights,
    term.bounds);
  return out;
}

}  // namespace quad_midbridge_mpcc
