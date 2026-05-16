# Patch notes

This version fixes the issues found during SITL testing.

1. `quad_midbridge_governor` is now a complete ROS2 package.
2. `attitude_offboard_node` no longer auto-arms by default, checks `valid`, stops publishing on timeout/invalid reference, and clamps thrust.
3. `flatness_mapper_node` now uses `/midbridge/governed_reference` by default and fixes the NED acceleration sign: negative `acceleration.z` increases thrust.
4. MPCC thrust/tilt debug estimates are aligned with the same NED convention.
5. Governor flag bits are moved to high bits to avoid collision with MPCC backend flags.
6. Governor no longer marks clamped references infeasible. Clamps are safety interventions, not execution failure.
7. A takeoff gate is added inside governor. Below the release height, it locks initial x/y and yaw and suppresses MPCC horizontal commands, performing only vertical takeoff. After release, it hands control back to MPCC.

Recommended first test: keep `auto_arm:=false`, use `auto_set_mode:=true`, publish local intent, inspect `/midbridge/attitude_thrust_reference`, then arm manually.

8. MPCC defaults are softened for the first SITL tests: larger preview time, lower max speed/yaw rate, and explicit vertical acceleration/jerk limits.

## 2026-04-28 safety/stability patch by ChatGPT

### Governor
- Fixed takeoff gate release bookkeeping: all release paths now call `markTakeoffReleased()` so `post_takeoff_blend_time_sec` actually starts from the release instant instead of being skipped by a zero timestamp.
- Ignored invalid `VehicleLocalPosition` samples instead of refreshing local-position timeout with bad PX4 data.
- Added `timeout_hover_feasible` parameter. With `hover_on_timeout: true` and `timeout_hover_feasible: true`, a raw-reference timeout now produces a valid level hover attitude command instead of an invalid reference that makes `attitude_offboard_node` stop publishing.
- Kept local-position timeout during takeoff gate conservative: if PX4 local position is missing/invalid, the governed reference is still forced invalid.

### Flatness mapper
- Added finite-input checks and an explicit invalid-neutral output path.
- Normalized output quaternion before publishing.
- Added a yaw-axis fallback for near-degenerate `b3 x xc` cases.
- Protected thrust scaling against zero/invalid scale values.

### Attitude offboard
- Added non-finite reference checks before publishing PX4 attitude setpoints.
- Normalized `q_d` before publishing.
- Reset warmup and auto-mode/auto-arm command latch on timeout or invalid reference so the node can recover cleanly after a reference dropout.

### MPCC node and placeholder solver
- Added PX4 local-position validity checks using `xy_valid`, `z_valid`, `v_xy_valid`, and `v_z_valid`.
- Added `state_timeout_sec` and `max_velocity_sample_dt` parameters.
- Suppressed stale-state solves by marking the solver input invalid when PX4 state times out.
- Reset path progress on every new local intent when the current state is valid, which prevents re-anchored local paths from inheriting stale progress.
- Fixed yaw blending in `full_state_placeholder_solver` to interpolate through wrapped angle error instead of linearly mixing absolute yaw angles.

### Aligned local intent
- Added validity checks before using PX4 local position.
- Added `local_pos_timeout_sec` parameter and protected timer period computation.

## 2026-04-28 Full MPCC/acados integration scaffold

This patch turns the previous `acados_generated` backend from a placeholder hook into a concrete full-MPCC integration path.

Added:

- `quad_midbridge_mpcc/tools/generate_mpcc_acados_solver.py`
  - Generates a complete acados OCP named `quad_midbridge_mpcc_ocp`.
  - State: `[p, v, a, psi, s]`, control: `[j, psi_dot, s_dot]`.
  - Stage cost: contour, lag, yaw, speed, acceleration, jerk, and progress terms.
  - Constraints: corridor, vertical corridor, flatness-implied tilt/thrust, yaw rate, progress rate, speed, acceleration, jerk.
- Real generated-code symbol mapping in `acados_generated_bindings.cpp` guarded by `QUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED`.
- Runtime parameter vector expanded from 16 to 32 entries so cost weights and gravity can be updated from ROS parameters without regenerating the solver.
- Nonlinear constraint vector expanded from 6 to 9 entries to include speed, acceleration and jerk norms inside the OCP.
- CMake options for both generated solver include/lib paths and acados include/lib paths.
- Updated `docs/ACADOS_GENERATED_INTEGRATION.md` with generation, build, runtime and debugging instructions.

Important:

- The generated solver horizon is fixed at generation time. Keep `horizon_steps` and `horizon_dt` synchronized with the generator arguments.
- Formal experiments should require `/midbridge/mpcc_debug.status == generated_code_solved`. Any status beginning with `acados_generated_fallback:` means the system is not using the full MPCC backend.

## 2026-05-07 secondary robustness patch

This patch tightens the PX4-compatible reference-governor chain after the stable hover/slow-forward tests.

### Governor fixes

- Removed the duplicated terminal-hold vertical bias term. Terminal hold now uses the same relative-height law as forward hold:
  `acc_z = 2.4 * err_z - 2.2 * local_vz - 0.35`.
- Added `refreshFlatnessDiagnostics()` so `thrust_nominal` and `tilt_nominal` always match the acceleration actually sent to the flatness mapper.
- Moved the final acceleration and flatness-envelope clamp after mode-dependent shaping. This ensures forward altitude hold and terminal hold cannot bypass the acceleration/thrust/tilt envelope.
- Changed terminal-hold XY correction from independent x/y clamps to vector saturation, preventing diagonal commands from exceeding the intended XY acceleration envelope.
- Kept forward and terminal height references relative to `takeoff_lock_z0`, preserving robustness against PX4 local-z origin changes.

### LocalIntent switching fix

- Added `scripted_local_intent_manager_node`, a single-node mission intent publisher with the stages:
  `hover -> forward -> decelerate -> final_hold`.
- Replaced the old `pkill -f aligned_local_intent_node` launch switching in `hover_then_forward_15m_robust.launch.py` with the new scripted manager.
- The manager latches stage anchors at transition time and ramps speed/progress down during deceleration before entering final hold. The default robust launch keeps the final-hold transition near the original 120 s point by using 45 s hover + 67 s forward + 8 s deceleration.

### Build/package fixes

- Added `geometry_msgs` as an explicit dependency of `quad_midbridge_governor`, because the LocalIntent publishers directly include `geometry_msgs/msg/point.hpp`.

### MPCC consistency fix

- The generated and fallback MPCC builders now clamp desired speed to the actual speed bound instead of allowing a hard minimum of 0.1 m/s. This avoids low-speed/risk-adjusted cases where desired speed could exceed the bound and trigger unnecessary generated-solver rejection.

## 2026-05-07 thrust adaptation patch

- Added online hover-thrust adaptation in `quad_midbridge_flatness_mapper`.
- The mapper now subscribes to `/fmu/out/vehicle_local_position` and uses low-speed tracking error to estimate a normalized hover thrust bias.
- The final thrust mapping becomes `collective_thrust = hover_thrust_norm_est * ||g e3 - a_ref|| / g` when adaptation is enabled.
- The estimate is published on `/midbridge/hover_thrust_estimate` for bag analysis.
- Adaptation is gated by reference speed, actual speed, lateral acceleration, vertical acceleration, and PX4 z-reset settling time.
- MPCC remains the 3D reference-generation kernel; thrust/mass compensation is implemented at the attitude-thrust mapping layer, where the normalized thrust is actually generated.

## 2026-05-07 flatness-mapper strengthening

1. Added `quad_midbridge_msgs/msg/FlatnessDebug.msg` and `/midbridge/flatness_debug`.
2. `flatness_mapper_node` now reports differential-flatness execution diagnostics: desired force norm, desired tilt, raw/clamped PX4 normalized thrust, hover-thrust estimate, local/desired z, adaptation gates, and thrust saturation.
3. The hover-thrust adaptation is kept in the flatness execution layer rather than in MPCC. MPCC remains the reference-generation kernel, while the mapper handles thrust-scale/payload/battery compensation at the point where acceleration is converted to PX4 attitude-thrust commands.
4. `finiteReference()` now checks all position components, not only `position.z`.
5. Robust launch files enable flatness debug publication by default.

## 2026-05-07 height-mode and adaptation-timeout patch

- Implemented the `height_mode` parameter in `governor_node`.
  - `takeoff_relative` preserves the current flat SITL behavior by holding height relative to `takeoff_lock_z0`.
  - `path_z` preserves the z component generated by the path/MPCC and adds only a bounded vertical correction.
  - `passthrough` leaves z shaping to the upstream reference and only keeps the generic acceleration/thrust/tilt envelope.
- Added `path_z_kp`, `path_z_kd`, `path_z_accel_correction_min`, and `path_z_accel_correction_max` parameters.
- Added explicit `height_mode: takeoff_relative` to the robust launch/config defaults so current hover/forward behavior is unchanged.
- Added `local_pos_timeout_sec` to `flatness_mapper_node` so online hover-thrust adaptation is disabled when PX4 local-position data becomes stale.
