# MATLAB planning core

This repository contains one public planner:

```matlab
result = planner(obstacles, initialState, goalState, limits, options);
```

The planner prepares protected polygon geometry, constructs exact visibility
evidence, generates C3 motion with BMTP, and accepts a result only after the
public independent validator passes it. Expected no-path and infeasible outcomes
return `Success=false` with stable `Message` and `TerminationReason` fields.

MATLAB R2024b and Optimization Toolbox are the behavioral reference. Add only
the root and BMTP package parent to the path:

```matlab
addpath(pwd, fullfile(pwd, 'trajectory'));
```

## Pipeline

The production flow has no caller-selectable planner profile:

```text
validate request
  -> prepare original and protected geometry once
  -> build the applicable exact visibility proposal
  -> generate one BMTP motion for that proposal and clock
  -> continuously certify geometry, dynamics, and C3 continuity
  -> run the public independent validator
  -> return one stable result
```

Static visibility graphs exhaustively classify all endpoint and prepared
boundary-node pairs. Timed visibility uses one deterministic staging-node set
and one temporal search. A moving edge is checked over its complete time
interval: affine point motion against every affine convex obstacle cell is
reduced to quadratic half-space residuals, whose real roots partition all
possible contact intervals. Collision acceptance does not depend on sampling.

Fixed-arrival requests use one deterministic policy. The planner evaluates the
exact spatial guide appropriate to the request and constructs the timed guide
only when solver-level evidence shows that the spatial proposal cannot produce
a complete motion. A validator rejection terminates as a defect; it never
starts a retry or repair profile.

Earliest-arrival requests optimize arrival time. Static requests use the
variable-clock BMTP formulation. Moving requests test the physical direct
departure family and the same source-independent variable-clock timed profile;
chronological fixed-time trials are used only when neither supplies a certified
motion. `TemporalSearch.GlobalEarliestProven` remains false when discrete time
layers or a finite trial budget prevent a continuous-time global proof.

There are no route-class heuristics, Delaunay-first graphs, boundary-offset
retry schedules, connectivity-recovery passes, fixture-specific seeds, hidden
waypoints, or silent motion fallbacks.

## Inputs

`obstacles` may contain static polygons or canonical polygon histories created
with `obstacleAvoidance.obstacles.createObstacle`. Safety margins are applied
exactly once. Original and protected geometry remain distinct in the result.
Unsupported continuous deformation is reported explicitly rather than sampled
or relabeled as free space.

`initialState` and `goalState` contain:

- `time_s`
- `position_units`
- optional `velocity_units_s` and `acceleration_units_s2`, which default to
  `[0 0]`

A moving goal may instead supply `targetMotion.time_s`,
`targetMotion.position_units`, and `InterpolationMethod` (`linear` or `pchip`).
Target velocity and acceleration matching are explicit options.

`limits` contains:

- `xInterval_units`, `yInterval_units`
- `maxVelocity_units_s`
- `maxAcceleration_units_s2`
- `maxJerk_units_s3`

A scalar derivative limit is a combined magnitude and is conservatively split
equally between axes. A two-element vector is an explicit per-axis limit.

Public planner options are:

- `GoalTimeMode`: `fixedArrival` or `earliestArrival`
- `SampleTime_s`
- `ConstraintTolerance`
- `CollisionClearanceTolerance_units`
- `ArrivalTimeTolerance_s`
- `WrapX`, `WrapY`
- `MatchTargetVelocity`, `MatchTargetAcceleration`
- `TemporalResolution_s`
- `MaxArrivalTrials`

Unknown options issue one warning and do not change planner behavior. Periodic
coordinates are supported only for obstacle-free fixed-position goals.

## Outputs and validation

Successful results contain sampled position, velocity, acceleration, and jerk;
the authoritative piecewise polynomial; the selected route and visibility
evidence; prepared geometry; BMTP diagnostics; continuous plane certificates;
and the independent validation record.

Validate and plot without rerunning planning:

```matlab
validation = obstacleAvoidance.validateTrajectory(result);
handles = obstacleAvoidance.plotting.plotTrajectory(result, ...
    struct('ShowAnimation', false, 'FigureVisible', 'off'));
```

Position spans are quintic for fixed and timed clocks and degree eight for
static variable-clock BMTP. Position, velocity, acceleration, and jerk are
continuous at internal joins. Endpoint position, velocity, and acceleration
remain prescribed. No post-solve time stretching or tolerance weakening is
used to make a candidate pass.

## Examples

Add the examples directory, then call any maintained example with plots off:

```matlab
addpath(fullfile(pwd, 'examples'));
result = exampleMovingBarrierWait( ...
    struct('PlotOutputs', false, 'Verbose', false));
```

The maintained suite covers obstacle-free motion, static detours, concave and
dense geometry, expected no-path outcomes, moving obstacles, rotating obstacles,
moving targets, and large source histories. `exampleSpinningUAtStartAndGoal`
exercises rotating cavities at both endpoints. `exampleMovingObstacle220`
exercises 220-vertex moving geometry over a 230-second simulated horizon.

The MATLAB and browser sandboxes call the same public planner and validator:

```matlab
addpath(fullfile(pwd, 'sandbox'));
ui = obstacleAvoidanceSandbox();

addpath(fullfile(pwd, 'offlinesandbox'));
offlineSandbox.serveSandbox();
```

## Verification

Run the complete test suite and maintained example audit from the repository
root:

```matlab
addpath('trajectory', 'examples', 'tests');
assertSuccess(runtests('tests'));
summary = runExampleBenchmarks();
assert(all(summary.Valid));
```

The current decision-flow audit, hand reproductions, removed branches, and
identical-input quality/runtime measurements are recorded in
[`benchmarks/planner_decision_flow.md`](benchmarks/planner_decision_flow.md).
The concise completion record is [`plan.md`](plan.md). Dense-input and geometry
contracts are documented in the remaining focused benchmark reports and
[`obstacle_history_contract.md`](obstacle_history_contract.md).

Generated benchmark, profiling, plot, and scratch outputs are ignored and must
not be committed. The benchmark workbook and supplied MAT/CSV fixtures are
intentional reproducible test inputs.
