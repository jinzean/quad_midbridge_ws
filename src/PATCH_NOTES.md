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
