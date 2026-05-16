# RAL-oriented full MPCC development plan

## Core positioning

Do not present the work as `MPCC + flatness + PX4 integration`.  Present it as:

```text
Executable local coverage-intent tracking for PX4-compatible UAV flight.
```

The high-level module provides a local intent rather than a dynamically smooth trajectory:

```text
I = {centerline gamma(s), corridor, speed preference, yaw/view preference, risk, terminal flag}
```

The proposed middle layer solves a full MPCC over flat outputs:

```text
x = [p, v, a, psi, s]
u = [j, psi_dot, s_dot]
```

and then uses a reference governor plus flatness mapping to produce PX4 attitude-thrust commands.

## Required algorithmic claim

The algorithm should be described as:

1. Build a local path-conditioned OCP from the current UAV state and local coverage intent.
2. Penalize contour error, lag error, yaw/view error, speed error, acceleration and jerk.
3. Reward forward progress along path parameter `s`.
4. Enforce executable constraints induced by PX4 attitude-thrust execution:
   - speed norm
   - acceleration norm
   - jerk norm
   - yaw-rate limit
   - progress-rate limit
   - corridor bound
   - flatness-implied thrust bound
   - flatness-implied tilt bound
5. Send the first optimized flat-output reference through the governor and flatness mapper.

## Minimum experiments for RAL

Baselines:

1. PX4 position setpoint waypoint following.
2. Pure-pursuit or preview tracking without MPCC.
3. Raw full MPCC without governor.
4. Full MPCC + flatness without takeoff/blend/timeout protection.
5. Full method.

Metrics:

1. Coverage ratio or task completion rate.
2. Collision rate and out-of-bound rate.
3. Contour error and lag error.
4. Solver time and solve success rate.
5. Fallback count.  A valid full-MPCC experiment should have zero fallback after initialization.
6. Thrust saturation count.
7. Tilt saturation count.
8. Jerk violation count after governor.
9. PX4 offboard failsafe count.
10. Reference smoothness.

## Code milestones

### Milestone 1: Generated solver compiles

Expected debug status:

```text
generated_code_solved
```

Bad status:

```text
acados_generated_fallback:...
```

### Milestone 2: Static local-intent tracking

Use one straight path and one curved path.  Verify:

```text
contour_error decreases
progress increases
predicted thrust stays within bounds
predicted tilt stays within bounds
solver_time_ms is stable
```

### Milestone 3: Switching local intent

Enable reanchored or changing local-intent input.  Verify that the governor prevents jumps in:

```text
position
velocity
acceleration
jerk
yaw
thrust
tilt
```

### Milestone 4: Closed-loop coverage intent

Replace `aligned_local_intent_node` with a real local coverage planner or an intermediate scripted coverage-intent generator.  Report coverage-level metrics.

### Milestone 5: PX4 SITL plus real-flight validation

For real flight, keep conservative limits first:

```text
max_speed <= 0.5 m/s
max_accel <= 1.5 m/s^2
max_jerk <= 3.0 m/s^3
tilt_max_rad <= 0.25
```

Only increase limits after verifying thrust, tilt and offboard stability.
