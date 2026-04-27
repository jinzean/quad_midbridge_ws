# ACADOS generated solver integration notes

This package now separates three layers:

1. `buildAcadosProblem(...)`
   - converts ROS-side intent/state into solver-side stage data.
2. `AcadosGeneratedAdapter`
   - pushes `AcadosProblemData` into the generated solver.
3. `AcadosGeneratedBindings`
   - the only place where generated acados C symbols should appear.

## Expected generated-code layout

Recommended workspace layout:

```text
src/
  quad_midbridge_mpcc/
  third_party/
    mpcc_acados_generated/
      include/
      lib/
      <generated solver sources or shared libs>
```

## Build flags

Example build command:

```bash
colcon build \
  --packages-select quad_midbridge_mpcc \
  --cmake-args \
    -DQUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED=ON \
    -DQUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_INCLUDE_DIR=$PWD/src/third_party/mpcc_acados_generated/include \
    -DQUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIB_DIR=$PWD/src/third_party/mpcc_acados_generated/lib \
    -DQUAD_MIDBRIDGE_MPCC_ACADOS_GENERATED_LIBS="acados;hpipm;blasfeo;mpcc_solver"
```

Adjust the final library list to match your generated solver package.

## What to replace

Open `src/acados_generated_bindings.cpp` and replace the TODO regions with your generated model symbols.
Only this file should know the exact names of:

- `<model>_acados_create_capsule`
- `<model>_acados_create`
- `<model>_acados_solve`
- `<model>_acados_free`
- `<model>_acados_free_capsule`
- any `ocp_nlp_*` getter/setter accessors

## Mapping conventions in this codebase

State layout:

```text
x = [px, py, pz, vx, vy, vz, ax, ay, az, psi, s]
```

Control layout:

```text
u = [jx, jy, jz, psi_dot, s_dot]
```

Stage parameter vector `p[16]`:

```text
0:2   path point
3:5   path tangent
6     s_ref
7     yaw_ref
8     speed_ref
9     risk_level
10    corridor_half_width
11    corridor_half_height
12    thrust_min
13    thrust_max
14    tilt_max_rad
15    yaw_rate_max
```

Generic stage bounds `lh/uh` currently encode:

```text
0 contour lateral lower/upper
1 contour vertical lower/upper
2 tilt lower/upper
3 thrust lower/upper
4 yaw-rate lower/upper
5 progress-rate lower/upper
```

If your generated problem uses a different constraint ordering, only remap it inside `AcadosGeneratedBindings::setStage(...)`.

## Fallback behavior

If generated-code integration is missing or solve fails, `acados_generated_solver` falls back to `full_state_placeholder`.
That means you can enable the backend early without losing runtime continuity.
