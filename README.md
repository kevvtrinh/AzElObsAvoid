# Azimuth/Elevation Obstacle-Avoidance Planner

This branch contains one production planner:

```matlab
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
```

The planner generates a bounded deterministic set of geometric and timed
topology seeds, constructs continuous corridor-constrained quintic motion, and
independently validates the complete time-parameterized result. It returns the
earliest validated candidate from the finite set it attempted. This is not a
claim of global path completeness or global time optimality.

## Requirements

- MATLAB R2024b or a compatible release.
- Optimization Toolbox for the bounded affine/QP corridor solve.

## Quick start

```matlab
initialState = struct( ...
    "time_s", 0, ...
    "position_deg", [-5 0], ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);

goalState = struct( ...
    "time_s", 12, ...
    "position_deg", [5 0], ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);

limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2]);

options = planAzElMotion();
options.GoalTimeMode = "earliestArrival";

result = planAzElMotion([], initialState, goalState, limits, options);
```

Use `makeAzElObstacleData` for static or time-indexed polygons and
`makeMovingAzElObstacleData` for moving shapes. The constructors retain
original and protected geometry and apply each requested safety margin once.

## Public contract

`initialState` and `goalState` require:

- `time_s`;
- a one-by-two `position_deg` in `[azimuth elevation]` order.

Velocity and acceleration default to zero. A sampled moving goal additionally
uses increasing `targetTime_s`, matching `targetPosition_deg`, and an
optional `InterpolationMethod` of `"linear"` or `"pchip"`.

`limits` requires positive one-by-two values for:

- `maxVelocity_deg_s`;
- `maxAcceleration_deg_s2`;
- `maxJerk_deg_s3`.

Optional `azimuthInterval_deg` and `elevationInterval_deg` fields define
the workspace and default to `[-180 180]` and `[-90 90]` degrees.

Call `planAzElMotion()` for fully resolved defaults. Partial option structures
are accepted, and empty fields use defaults. The maintained
`MotionMethod="corridorQuintic"` value is a compatibility field, not a
dispatcher.

Expected no-path, work-limit, and dynamic-infeasibility outcomes return
`Success=false` with a machine-readable `TerminationReason`. Invalid public
contracts throw identified errors. Success and failure preserve the same result
field order.

## Moving-target interception

`planAzElMovingTargetIntercept` adapts sampled target motion to the same public
planner:

```matlab
interceptOptions = planAzElMovingTargetIntercept();
interceptOptions.InterceptMode = "earliest";
interceptOptions.PlannerOptions = planAzElMotion();

result = planAzElMovingTargetIntercept( ...
    obstacles, initialState, targetMotion, limits, interceptOptions);
```

Specified-time mode evaluates one fixed arrival. Earliest mode evaluates a
bounded chronological set of fixed-arrival trials and refines the first
observed feasible bracket.

## Results and diagnostics

Important result fields include:

- `Success`, `Message`, and `TerminationReason`;
- sampled position, velocity, acceleration, and jerk histories;
- the exact piecewise polynomial trajectory;
- generated seeds, per-seed summaries, and the selected seed;
- original inputs, resolved options, and deterministic seed;
- independent collision and kinematic validation;
- search diagnostics and exclusive stage timing.

`plotAzElMotion` consumes the returned result. It does not rerun planning and
can plot failed search diagnostics without a selected trajectory.

## Repository layout

```text
planAzElMotion.m                 public planner and orchestration
planAzElMovingTargetIntercept.m  moving-target adapter
validateAzElTrajectory.m         independent trajectory validator
queryAzElTimeObstacle.m          shared time-aware collision query
plotAzElMotion.m                 result and failure visualization

+azElInternal/
  +geometry/                     reusable polygon geometry
  +motion/                       quintic construction and retiming
  +obstacles/                    prepared obstacle histories
  +search/                       topology and dynamic-route helpers
  planner schema, options, timing, and certificates

examples/                        maintained runnable scenarios
tests/                           focused and integration tests
benchmarks/                      reproducible benchmark drivers
sandbox/                         interactive planner UI
```

## Verification

Run focused tests from the repository root:

```matlab
results = runtests("tests");
assert(all([results.Passed]));
```

Maintained examples support headless overrides such as
`PlotOutputs=false` and `FigureVisible="off"`. Benchmark history and
branch assessments record measured behavior and limitations; historical rows
are evidence, not a promise that every older implementation remains present.
