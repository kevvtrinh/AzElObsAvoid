# MATLAB planning core

This repository contains one public planner:

```matlab
result = planner(obstacles, initialState, goalState, limits, options);
```

The planner prepares protected polygon geometry, constructs exact visibility
evidence, generates C3 motion with BMTP, and accepts a result only after the
public independent validator passes it. Expected no-path and infeasible outcomes
return `Success=false` with stable `Message` and `TerminationReason` fields.

MATLAB R2024b and Optimization Toolbox are the behavioral reference. Parallel
Computing Toolbox is optional: large obstacle interval preparations use at most four
available background workers when the pool is idle. The serial path remains
the reference for geometry, certificates, and cache semantics. Add only the
root and BMTP package parent to the path:

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
exact initial- and arrival-snapshot visibility guides as bounded runtime
shortcuts, then constructs one time-expanded guide when neither shortcut
returns a complete motion. A shortcut failure is not evidence of request
infeasibility. Every stage is recorded in `result.Attempts`, including its
declared iteration limit, typed failure, selection, and child-attempt evidence.
A validator rejection terminates as a defect; it never starts another attempt.

Earliest-arrival requests pass through one capability-based coordinator. Static
fixed-position rest-to-rest requests use the exact spatial graph and
variable-clock BMTP. Dynamic fixed-position rest-to-rest requests first test the
physical direct-departure family, retain any validated motion as an incumbent,
then run one source-independent variable-clock timed challenger only below that
incumbent. Moving targets and non-rest endpoint requests skip unsupported
families and go directly to chronological fixed-clock trials. A typed
search/timing/proposal miss may admit the next method; validation, geometric
certification, reconstruction, numerical, and unknown failures are terminal.
A complete continuous curve that passes workspace, continuity, and geometry
checks but exceeds derivative limits is a clock-local timing miss, not a
weakened certificate.

By default, a validated dynamic incumbent is retained after the timed challenger
fails instead of launching an unbounded chronological search. The result reports
the unsearched interval and does not claim global optimality. Callers may spend
an explicit, bounded `IncumbentRefinementTrialLimit` to search earlier clocks
without changing the public validator or discarding the incumbent. Every route,
motion, and clock attempt is recorded in `result.Attempts`; chronological parent
attempts retain their fixed-arrival child evidence. `EarliestArrival` records
capabilities, the selected attempt, the incumbent, the bounded-search state,
and whether the necessary lower bound was actually attained.

There are no route-class pruning rules, Delaunay-first graphs, boundary-offset
repairs, connectivity-recovery passes, fixture-specific seeds, hidden
waypoints, fabricated direct seeds, or silent motion fallbacks. The two
snapshot shortcuts are deterministic, use exact exhaustive snapshot graphs,
and cannot weaken the independent acceptance gate.

BMTP checks that supplied coverage metadata is internally consistent before it
solves. That check is not treated as proof of coverage completeness: the public
validator reconstructs authoritative obstacle coverage from the original
request before any motion is accepted.

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
- `SpatialProbeIterationLimit`: BMTP iteration budget for each fixed-arrival
  snapshot shortcut (default `2`, maximum `35`)
- `MaxArrivalTrials`: maximum fixed-clock planner solves
- `MaxArrivalCandidates`: maximum regular-grid clocks screened; exact declared
  obstacle, target, and horizon boundaries inside that grid window are retained
- `IncumbentRefinementTrialLimit`: maximum chronological fixed-clock solves
  allowed below a validated dynamic incumbent after the timed challenger fails
  (default `0`; never exceeds `MaxArrivalTrials`)

Unknown options issue one warning and do not change planner behavior. A
wrapped axis is planned in the unwrapped frame inside the reach band of the
request: obstacles are represented by exact translated images at every period
offset that meets the band, a moving target is lifted by continuity from the
initial position, and every goal image inside the band is planned as a plain
request and accepted against the periodic request in the one acceptance gate
(earliest arrival first, or shortest motion for a fixed arrival). The
validator rebuilds the images, the lift, and the goal image from the supplied
request. Returned positions stay in the unwrapped frame.

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

The maintained decision-flow audit is recorded in
[`benchmarks/planner_decision_flow.md`](benchmarks/planner_decision_flow.md).
Dynamic-obstacle behavior and geometry contracts are documented in the focused
benchmark reports and
[`obstacle_history_contract.md`](obstacle_history_contract.md).

Generated benchmark, profiling, plot, and scratch outputs are ignored and must
not be committed. The benchmark workbook and supplied MAT/CSV fixtures are
intentional reproducible test inputs.
