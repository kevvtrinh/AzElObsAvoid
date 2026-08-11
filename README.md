# Adaptive Azimuth/Elevation Obstacle Avoidance

A greenfield MATLAB planner for minimum-arrival-seeking pointing commands in
time-varying azimuth/elevation exclusion scenes.

The planner consumes canonical `azElData` polygons directly, chooses its own
coarse-to-fine spatial and temporal work, constructs continuous
position/velocity motion, and accepts a command only after a separate validator
checks continuous dynamics and obstacle clearance. A bounded search that finds
no certified answer returns `unknown`; it is never mislabeled infeasible.

## What is implemented

- Fixed and sampled moving complete-state goals.
- Static and time-varying polygons on independent obstacle time grids.
- Multiple disconnected regions separated by paired nonfinite rows.
- Explicit safety margins and temporal uncertainty padding.
- Wrapped azimuth with a continuous unwrapped command history.
- Exact initial and terminal position, velocity, and acceleration.
- C2 piecewise-quintic motion with positive velocity carry through safe turns.
- Near-bound direct rest-to-rest timing using symmetric constant-jerk S-curves.
- Explicit, physically stationary departure waits when timing requires them.
- Optional post-capture tracking of a moving goal.
- Deterministic, internally selected boundary and timing refinement.
- Exact polynomial-extrema checks for position, velocity, and acceleration.
- Recursive full-interval collision certificates against canonical geometry.
- Stable structured results for success, invalid input, proved infeasibility,
  and inconclusive searches.

## Requirements

The project is verified on MATLAB R2024b. The implementation uses MATLAB base
functionality and does not require callers to install a solver package or
construct private planner data.

## Quick start

```matlab
obstacle = makeAzElObstacleData( ...
    "central exclusion", [0; 30], ...
    [-5; 5; 5; -5], [-5; -5; 5; 5]);

initialState = struct( ...
    "time_s", 0, ...
    "position_deg", [-20, 0], ...
    "velocity_deg_s", [0, 0], ...
    "acceleration_deg_s2", [0, 0]);

goal = struct( ...
    "type", "fixed", ...
    "time_s", NaN, ...
    "position_deg", [20, 0], ...
    "velocity_deg_s", [0, 0], ...
    "acceleration_deg_s2", [0, 0]);

limits = struct( ...
    "azimuth_deg", [-60, 60], ...
    "elevation_deg", [-30, 30], ...
    "maxVelocity_deg_s", [12, 12], ...
    "maxAcceleration_deg_s2", [8, 8]);

missionOptions = struct( ...
    "safetyMargin_deg", 1, ...
    "deadline_s", 30);

request = struct( ...
    "obstacles", obstacle, ...
    "initialState", initialState, ...
    "goal", goal, ...
    "limits", limits, ...
    "options", missionOptions);

result = planAzElAvoidance(request);
assert(result.success, result.message);

evidence = validateAzElCommand( ...
    result.command, request, result.arrivalTime_s);
assert(evidence.isValid, evidence.message);
visualizeAzElPlan(request, result);
```

`planAzElAvoidance` is the one public planning entry point. Examples contain
mission inputs and independent evidence only; there are no caller-provided
routes, search modes, refinement settings, or scenario-specific solver hints.

## Canonical obstacle input

One obstacle is one scalar record:

```matlab
azElData = struct( ...
    "targetName", obstacleName, ...
    "time_s", time_s, ...
    "az_deg", {azimuthBoundaryByTime_deg}, ...
    "el_deg", {elevationBoundaryByTime_deg}, ...
    "status", statusByTime);
```

Use `makeAzElObstacleData` to construct it. Numeric boundary vectors describe a
static polygon repeated over the supplied time base. Cell boundaries describe
sampled moving polygons. Multiple obstacles may use different time arrays.

Between compatible samples, corresponding polygon vertices move linearly. If
region topology or vertex count changes, the planner conservatively holds the
union of both adjacent slices over that interval instead of inventing vertex
correspondence. Geometry is held at the first and last sample outside the
supplied time range. Temporal padding unions geometry at the query time and at
both uncertainty endpoints.

## Request contract

`request.initialState` is a complete state with a finite `time_s`.

A fixed goal has the same state fields and `type="fixed"`. Its `time_s` may be
`NaN` when arrival is free; a finite value becomes the deadline unless an
explicit earlier `options.deadline_s` is supplied.

A moving goal has `type="moving"`, an increasing `time_s` column, and N-by-2
position, velocity, and acceleration histories. Those complete states define a
coherent quintic goal trajectory between samples. Capture must occur within the
sample range. `trailingDuration_s` reserves enough goal history after capture.

Public options are mission facts only:

| Field | Default | Meaning |
| --- | ---: | --- |
| `safetyMargin_deg` | `0.5` | Required polygon clearance |
| `azimuthWrap` | `false` | Permit cyclic azimuth seam crossing |
| `temporalPadding_s` | `0` | Symmetric obstacle-time uncertainty |
| `deadline_s` | derived | Latest first arrival or capture |
| `trailingDuration_s` | `0` | Moving-goal tracking after capture |
| `planningWallTime_s` | `20` | One global planning budget |
| validation tolerances | documented in help | Independent evidence tolerances |

## Result and claims

Every exit returns the same top-level fields. On success, `result.command`
contains strictly increasing knot times and coherent wrapped position,
continuous unwrapped position, velocity, and acceleration histories. The
interpolation contract is quintic Hermite between knots.

`result.guarantee.feasibility` is `Validated feasible` only after every
independent gate passes. `result.guarantee.optimality` uses the vocabulary in
[`PLAN.md`](PLAN.md):

- obstacle-free direct motion can be `Minimum arrival under a stated model`;
- obstructed and time-varying results are conservatively labeled `Best found`;
- failure to find a command within finite work is `Unknown`.

The project does not claim global detour optimality. The independent
double-integrator lower bound and reported arrival gap make that limitation
visible.

## Run examples and tests

```matlab
addpath(pwd);
addpath(fullfile(pwd, "examples"));

example01Unobstructed(false);
example02StaticDetour(false);
example03TimedWallWait(false);
example04RlBranchArrivalOracles();
example05RlStaticDetour(false);
example06RlDynamicSafeIntervals(false);
example07RlWrappedSeamDetour(false);
example08RlNarrowPassage(false);

results = runtests("tests", "IncludeSubfolders", true);
assertSuccess(results);
```

The numbered examples all use the planner's ordinary internal defaults. Their
options describe only physical mission choices.

Examples 04-08 preserve only mission inputs and independent acceptance checks
from branch `all-dijkstra-rl-parameter-auto-tune`; no planner implementation,
learned artifact, route, or tuning value was imported. Example 04 enforces the
source branch's 1.03 arrival-ratio threshold on four analytic free-space
oracles. Examples 05-08 replay representative static, moving, wrapped, and
narrow obstacle scenes.

## Repository map

- [`planAzElAvoidance.m`](planAzElAvoidance.m): one public planning entry point.
- [`validateAzElCommand.m`](validateAzElCommand.m): independent authority.
- [`makeAzElObstacleData.m`](makeAzElObstacleData.m): fixed canonical builder.
- [`sampleAzElCommand.m`](sampleAzElCommand.m): coherent command evaluation.
- [`visualizeAzElPlan.m`](visualizeAzElPlan.m): post-validation display.
- [`private/`](private): adaptive work, geometry, timing, and motion internals.
- [`examples/`](examples): pure mission inputs and evidence.
- [`tests/`](tests): deterministic physical and source-level regressions.
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md): design and data flow.
- [`docs/VALIDATION.md`](docs/VALIDATION.md): certificate model and limits.
- [`HANDOFF.md`](HANDOFF.md): exact status, evidence, and next tasks.

The original greenfield requirements remain in [`PLAN.md`](PLAN.md),
[`STYLE.md`](STYLE.md), and [`AGENTS.md`](AGENTS.md).
