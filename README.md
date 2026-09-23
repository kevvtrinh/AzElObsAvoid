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
the reference for geometry, proofs, and cache semantics. Add only the
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
  -> continuously prove geometry, dynamics, and C3 continuity
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

Earliest-arrival requests pass through one coordinator that runs the planning
methods in a fixed order. Static fixed-position rest-to-rest requests use the
exact spatial graph and variable-clock BMTP. Dynamic fixed-position rest-to-rest
requests first try the direct-departure family, keep any validated motion as the
best plan so far, then run one variable-clock timed search that may only beat
that plan. Moving targets and non-rest endpoint requests skip the families that
cannot handle their endpoint physics and go straight to arrival-time trials, each
a fixed-clock request on its own clock. A typed search, timing, or proposal miss
lets the next method run; validation, geometric proof, reconstruction,
numerical, and unknown failures end the sequence.
A complete continuous curve that passes workspace, continuity, and geometry
checks but exceeds derivative limits is a clock-local timing miss, not a
weakened proof.

By default, a validated best plan so far is kept when the timed search fails,
instead of launching an unbounded arrival-time search. The result reports the
unsearched interval and does not claim global optimality. Callers may spend an
explicit, bounded `BestSoFarRefinementTrialLimit` of arrival-time trials to look
for earlier clocks, without changing the public validator or discarding the plan
already found.

The timed search samples time in layers spaced by the horizon. When it succeeds
but waiting at the goal from the layer just before its goal window was not
clear, no layer sampled the times in between, and the goal may have cleared
anywhere there. The planner then tries the arrival-time grid in that one
interval, below the timed arrival, with the `MaxArrivalTrials` budget
(`BestSoFarRefinementTrialLimit` does not apply). It keeps the timed motion if
no trial passes or a trial fails; only an independent-validation rejection is
returned instead. The grid does not move with the horizon. Measured example: a
square that clears the goal at 16.3 s arrives at 17.000 s with either a 24 s or
a 30 s horizon (18.750 s at 30 s before this step).

This is not a general proof that arrival is independent of the horizon: the
timed layers, route, and interval ends still depend on it, and a 27 s horizon
arrives at 16.888 s.

Every route, motion, and clock attempt is recorded in
`result.Attempts`; an arrival-time parent attempt keeps its fixed-arrival child
evidence. `EarliestArrival` records the capabilities, the selected attempt, the
best plan so far, the bounded-search state, and whether the earliest possible
arrival was actually attained.

There are no route-class pruning rules, Delaunay-first graphs, boundary-offset
repairs, connectivity-recovery passes, fixture-specific seeds, hidden
waypoints, fabricated direct seeds, or silent motion fallbacks. The two
snapshot shortcuts are deterministic, use exact exhaustive snapshot graphs,
and cannot weaken the independent acceptance gate.

BMTP checks that supplied coverage metadata is internally consistent before it
solves. That check is not treated as proof of coverage completeness: the public
validator reconstructs supplied obstacle coverage from the original
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
- `WrapX`, `WrapY`: `"false"` (default), `"both"`, `"forward"` (the path
  may pass the upper interval end but not the lower), or `"backward"` (the
  lower end but not the upper). `true` and `false` still mean `"both"` and
  `"false"`
- `MatchTargetVelocity`, `MatchTargetAcceleration`
- `TemporalResolution_s`
- `SpatialProbeIterationLimit`: BMTP iteration budget for each fixed-arrival
  snapshot shortcut (default `2`, maximum `35`)
- `MaxArrivalTrials`: maximum fixed-clock planner solves
- `MaxArrivalCandidates`: maximum regular-grid clocks screened; exact declared
  obstacle, target, and horizon boundaries inside that grid window are retained
- `BestSoFarRefinementTrialLimit`: how many arrival-time trials may try to
  beat a validated best plan so far after the timed search fails
  (default `0`; never exceeds `MaxArrivalTrials`)

Unknown options issue one warning and do not change planner behavior. A
wrapped axis (azimuth 359 meets 0) is planned in plain unwrapped coordinates
inside the range the vehicle can reach in the time given. A `"forward"` range
stops at the lower interval end and a `"backward"` range at the upper end.
Every obstacle is copied across the ends that range covers: an x copy is
shifted by whole turns, and a y copy over an end is a pole copy, as for
elevation on a sphere: y is mirrored about that end and x turns by half the x
interval. On x [0 360], y [-90 90], the pole copy of (190, 89) is (10, 91), so
a slew from (10, 89) to (190, 89) can cross the pole in 2 degrees instead of
turning 180 in azimuth. The x interval width is treated as one full turn.
With `WrapY` on, x repeats every turn even when `WrapX` is `"false"`, as
azimuth does on a sphere: ordinary and pole copies are both placed by whole
turns, and `WrapX` only decides whether the motion may cross the x ends. For
example, an obstacle spanning x = 358..362 also has a copy at -2..2 on
[0 360]. A goal or target with no copy inside the speed-based planning range
returns `Success = false` and `TerminationReason = "timeWindowInfeasible"`,
after the same obstacle-interval and endpoint checks an unwrapped request
gets: an obstacle interval without a usable model, or an endpoint derivative
beyond its limit, is reported first.
With `WrapY` on, a moving target's goal y velocity and acceleration must be
zero or matched to the target: over a pole their sign depends on where the
target is met. To match one, leave that whole field out of the goal; a
supplied velocity with `MatchTargetVelocity` is rejected even if its y part
is zero. A moving obstacle whose vertex matching between samples is an
exact tie cannot be mirrored consistently; declare its matching with
`vertexCorrespondence = "sourceIndex"`. A moving target's path is
unwrapped so it never jumps at the seam. For fixed arrival, copies are listed
at the deadline; for earliest arrival, a moving target's whole path through
the horizon is considered. Each listed copy is planned as an ordinary
request and accepted against the wrapped request in the one acceptance gate
(earliest arrival first, or shortest motion for a fixed
arrival). The validator rebuilds the obstacle copies, the unwrapped target path,
and the goal copy from the supplied request. Returned positions stay in
unwrapped coordinates.

## Outputs and validation

Successful results contain sampled position, velocity, acceleration, and jerk;
the supplied piecewise polynomial; the selected route and visibility
evidence; prepared geometry; BMTP diagnostics; continuous plane proofs;
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
remain given. No post-solve time stretching or tolerance weakening is
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
