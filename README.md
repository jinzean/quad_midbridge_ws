# quad_midbridge_ws

ROS 2/PX4 workspace for executable local-intent tracking with a quadrotor midbridge stack.

The current stack turns a high-level `LocalIntent` into PX4 attitude-thrust offboard setpoints:

```text
LocalIntent
  -> quad_midbridge_mpcc
  -> quad_midbridge_governor
  -> quad_midbridge_flatness_mapper
  -> quad_midbridge_attitude_offboard
  -> PX4
```

## Packages

- `quad_midbridge_msgs`: local intent, executable reference, MPCC debug, and flatness debug messages.
- `quad_midbridge_mpcc`: path-conditioned flat-output MPCC with generated acados backend support and fallback reference generation.
- `quad_midbridge_governor`: PX4-compatible reference governor, takeoff gate, terminal bridge, and scripted/circular local-intent publishers.
- `quad_midbridge_flatness_mapper`: differential-flatness mapping from governed references to attitude-thrust references, with hover-thrust adaptation diagnostics.
- `quad_midbridge_attitude_offboard`: PX4 offboard attitude setpoint streamer.

## Build

Fallback-only build:

```bash
cd ~/quad_midbridge_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select quad_midbridge_msgs quad_midbridge_mpcc quad_midbridge_governor \
  quad_midbridge_flatness_mapper quad_midbridge_attitude_offboard
```

Generated-acados build:

```bash
cd ~/quad_midbridge_ws
source /opt/ros/humble/setup.bash
export ACADOS_SOURCE_DIR=/home/zjnujza/acados
colcon build --symlink-install \
  --packages-select quad_midbridge_msgs quad_midbridge_mpcc quad_midbridge_governor \
  quad_midbridge_flatness_mapper quad_midbridge_attitude_offboard \
  --cmake-args -DQUAD_MIDBRIDGE_MPCC_WITH_ACADOS_GENERATED=ON
```

If generated code is absent, regenerate it from:

```bash
cd ~/quad_midbridge_ws/src/quad_midbridge_mpcc
python3 tools/generate_mpcc_acados_solver.py \
  --horizon-steps 20 \
  --horizon-dt 0.05 \
  --export-dir third_party/mpcc_acados_generated/c_generated_code \
  --build
```

## Circle Orbit Demo

Start PX4 SITL and MicroXRCEAgent first, then run:

```bash
cd ~/quad_midbridge_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ACADOS_SOURCE_DIR=/home/zjnujza/acados
export LD_LIBRARY_PATH=/home/zjnujza/acados/lib:$LD_LIBRARY_PATH
ros2 launch quad_midbridge_governor circle_orbit_robust.launch.py
```

Expected mission sequence:

```text
hover -> orbit -> decelerate -> final_hold
```

During orbit, `/midbridge/mpcc_debug.status` should mostly report:

```text
generated_code_solved
```

Fallback statuses are safe but should be counted in experiments.
