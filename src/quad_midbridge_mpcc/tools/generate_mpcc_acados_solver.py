#!/usr/bin/env python3
"""Generate the full acados MPCC solver for quad_midbridge_mpcc.

This script is intentionally self-contained.  Run it inside a Python environment
where acados_template and casadi are available.  The generated C solver uses the
same variable layout as the ROS2 package:

  x = [px, py, pz, vx, vy, vz, ax, ay, az, psi, s]
  u = [jx, jy, jz, psi_dot, s_dot]

The stage parameter vector p has length 32.  The first 16 entries are the
runtime geometric/execution parameters used by the ROS-side builder.  The next
entries carry cost weights so that the same generated solver can be retuned from
ROS parameters without regenerating code.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver


MODEL_NAME = "quad_midbridge_mpcc_ocp"
NX = 11
NU = 5
NP = 32
NH = 9


# Parameter layout.  Keep this synchronized with acados_generated_bindings.cpp.
P_PATH_POINT = slice(0, 3)
P_PATH_TANGENT = slice(3, 6)
P_S_REF = 6
P_YAW_REF = 7
P_SPEED_REF = 8
P_RISK = 9
P_CORRIDOR_HALF_WIDTH = 10
P_CORRIDOR_HALF_HEIGHT = 11
P_THRUST_MIN = 12
P_THRUST_MAX = 13
P_TILT_MAX = 14
P_YAW_RATE_MAX = 15
P_W_CONTOUR = 16
P_W_LAG = 17
P_W_PROGRESS = 18
P_W_YAW = 19
P_W_SPEED = 20
P_W_ACCEL = 21
P_W_JERK = 22
P_W_DELTA_JERK = 23  # reserved for future lifted-control formulation
P_W_DELTA_YAW_RATE = 24  # reserved
P_W_TERMINAL_CONTOUR = 25
P_W_TERMINAL_LAG = 26
P_W_TERMINAL_YAW = 27
P_GRAVITY = 28
P_EPS = 29
P_UNUSED_0 = 30
P_UNUSED_1 = 31


def safe_norm(v: ca.SX, eps: ca.SX) -> ca.SX:
    return ca.sqrt(ca.dot(v, v) + eps)


def wrap_quadratic_angle_error(angle: ca.SX) -> ca.SX:
    """Smooth-ish periodic angle penalty: 2 - 2*cos(e).

    This avoids the discontinuity of a hard wrap at +-pi while preserving a
    locally quadratic penalty around zero.
    """
    return 2.0 - 2.0 * ca.cos(angle)


def build_model() -> AcadosModel:
    x = ca.SX.sym("x", NX)
    u = ca.SX.sym("u", NU)
    xdot = ca.SX.sym("xdot", NX)
    p = ca.SX.sym("p", NP)

    px, py, pz = x[0], x[1], x[2]
    vx, vy, vz = x[3], x[4], x[5]
    ax, ay, az = x[6], x[7], x[8]
    psi, s_progress = x[9], x[10]
    jx, jy, jz, psi_dot, s_dot = u[0], u[1], u[2], u[3], u[4]

    f_expl = ca.vertcat(
        vx,
        vy,
        vz,
        ax,
        ay,
        az,
        jx,
        jy,
        jz,
        psi_dot,
        s_dot,
    )

    path_point = p[P_PATH_POINT]
    path_tangent_raw = p[P_PATH_TANGENT]
    eps = p[P_EPS] + 1e-9
    path_tangent = path_tangent_raw / safe_norm(path_tangent_raw, eps)
    yaw_ref = p[P_YAW_REF]
    speed_ref = p[P_SPEED_REF]
    gravity = p[P_GRAVITY]

    pos = ca.vertcat(px, py, pz)
    vel = ca.vertcat(vx, vy, vz)
    acc = ca.vertcat(ax, ay, az)
    jerk = ca.vertcat(jx, jy, jz)

    err = pos - path_point
    lag = ca.dot(err, path_tangent)
    contour_vec = err - lag * path_tangent
    contour_xy = safe_norm(contour_vec[0:2], eps)
    vertical_err = err[2]

    fd = ca.vertcat(-ax, -ay, gravity - az)
    thrust = safe_norm(fd, eps)
    horizontal_acc = safe_norm(ca.vertcat(ax, ay), eps)
    # Smooth tilt expression equivalent to atan(||a_xy|| / (g-a_z)) under normal flight.
    # This avoids the nonsmooth clamp/acos combination and gives a well-behaved derivative.
    tilt = ca.atan2(horizontal_acc, gravity - az)

    speed_norm = safe_norm(vel, eps)
    accel_norm = safe_norm(acc, eps)
    jerk_norm = safe_norm(jerk, eps)
    yaw_penalty = wrap_quadratic_angle_error(psi - yaw_ref)

    w_contour = p[P_W_CONTOUR]
    w_lag = p[P_W_LAG]
    w_progress = p[P_W_PROGRESS]
    w_yaw = p[P_W_YAW]
    w_speed = p[P_W_SPEED]
    w_accel = p[P_W_ACCEL]
    w_jerk = p[P_W_JERK]
    w_term_contour = p[P_W_TERMINAL_CONTOUR]
    w_term_lag = p[P_W_TERMINAL_LAG]
    w_term_yaw = p[P_W_TERMINAL_YAW]

    speed_err = speed_norm - speed_ref

    # Stage cost.  The negative progress term is linear in s_dot and therefore
    # directly encourages forward motion along the path parameter.
    stage_cost = (
        w_contour * ca.dot(contour_vec, contour_vec)
        + w_lag * lag * lag
        + w_yaw * yaw_penalty
        + w_speed * speed_err * speed_err
        + w_accel * ca.dot(acc, acc)
        + w_jerk * ca.dot(jerk, jerk)
        - w_progress * s_dot
        + 1e-4 * s_dot * s_dot
    )

    terminal_cost = (
        w_term_contour * ca.dot(contour_vec, contour_vec)
        + w_term_lag * lag * lag
        + w_term_yaw * yaw_penalty
        + 1e-4 * (s_progress - p[P_S_REF]) * (s_progress - p[P_S_REF])
    )

    # Generic nonlinear constraints.  The ordering must match
    # GeneratedStageBuffers::lh/uh in the C++ bindings.
    h = ca.vertcat(
        contour_xy,      # 0: lateral path corridor radius
        vertical_err,    # 1: vertical corridor around path z
        tilt,            # 2: flatness-implied tilt
        thrust,          # 3: flatness-implied mass-normalized thrust
        psi_dot,         # 4: yaw-rate
        s_dot,           # 5: path progress rate
        speed_norm,      # 6: translational speed
        accel_norm,      # 7: acceleration norm
        jerk_norm,       # 8: jerk norm
    )

    model = AcadosModel()
    model.name = MODEL_NAME
    model.x = x
    model.u = u
    model.xdot = xdot
    model.p = p
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl
    model.con_h_expr = h
    model.cost_expr_ext_cost = stage_cost
    model.cost_expr_ext_cost_e = terminal_cost
    return model


def build_ocp(horizon_steps: int, horizon_dt: float, export_dir: Path) -> AcadosOcp:
    ocp = AcadosOcp()
    ocp.model = build_model()
    ocp.dims.N = horizon_steps
    ocp.parameter_values = np.zeros(NP)
    ocp.parameter_values[P_GRAVITY] = 9.81
    ocp.parameter_values[P_EPS] = 1e-6

    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    # Initial-state equality constraint.  Runtime C++ overwrites lbx_0/ubx_0.
    ocp.constraints.idxbx_0 = np.arange(NX)
    ocp.constraints.lbx_0 = np.zeros(NX)
    ocp.constraints.ubx_0 = np.zeros(NX)

    # Default nonlinear h bounds.  Runtime C++ overwrites lh/uh per stage.
    ocp.constraints.lh = np.array([0.0, -0.5, 0.0, 4.0, -0.8, 0.0, 0.0, 0.0, 0.0])
    ocp.constraints.uh = np.array([1.0, 0.5, 0.65, 16.0, 0.8, 1.5, 1.5, 4.0, 8.0])

    # Direct input box constraints for:
    # u = [jx, jy, jz, psi_dot, s_dot]
    # These bounds are also overwritten per stage from C++.
    # Keeping psi_dot and s_dot as true control bounds avoids RTI nonlinear-constraint leakage.
    ocp.constraints.idxbu = np.array([0, 1, 2, 3, 4])
    ocp.constraints.lbu = np.array([-8.0, -8.0, -8.0, -0.8, 0.0])
    ocp.constraints.ubu = np.array([8.0, 8.0, 8.0, 0.8, 1.5])

    ocp.solver_options.tf = horizon_steps * horizon_dt
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    # External costs use an exact Hessian in acados.
    ocp.solver_options.hessian_approx = "EXACT"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.qp_solver_cond_N = min(horizon_steps, 5)
    ocp.solver_options.nlp_solver_max_iter = 1
    ocp.solver_options.print_level = 0
    ocp.code_export_directory = str(export_dir)
    return ocp


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--horizon-steps", type=int, default=20)
    parser.add_argument("--horizon-dt", type=float, default=0.05)
    parser.add_argument(
        "--export-dir",
        type=Path,
        default=Path("third_party/mpcc_acados_generated/c_generated_code"),
        help="Directory for acados generated C code.",
    )
    parser.add_argument("--build", action="store_true", help="Compile the generated C solver after export.")
    args = parser.parse_args()

    ocp = build_ocp(args.horizon_steps, args.horizon_dt, args.export_dir)
    solver = AcadosOcpSolver(ocp, json_file=str(args.export_dir / f"{MODEL_NAME}.json"), build=args.build)
    print(f"Generated {MODEL_NAME} with N={args.horizon_steps}, dt={args.horizon_dt}, build={args.build}")
    print(f"Export directory: {args.export_dir.resolve()}")
    # Keep solver alive until generation finishes.
    del solver


if __name__ == "__main__":
    main()
