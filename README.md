# Obstacle-Avoidance Trajectory Planner

This branch provides one public obstacle-avoidance planner for trajectories in
the azimuth/elevation frame. It uses a third-order Hermite-Simpson (HS3)
transcription solved with `fmincon`. The `obstacleAvoidance` namespace owns
input, obstacles, geometry, search, planning, and plotting. The
dimension-neutral engine remains a separate normal `hs3/` product folder.

A successful result is accepted only after canonical independent validation.
Successful results have `SelectedMotionSource="hs3"` and echo
`PlannerMethod="hs3"` in their resolved options and search diagnostics.

The HS3 implementation descends from the `plan-325` snapshot at commit
`5a067112a9f880d015f52fb97538a99010871478`. Planner and engine ownership is
summarized below in this repository-level guide.

## Quick start

The zero-input call returns HS3 defaults:

```matlab
options = obstacleAvoidance.planTrajectory();

result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, options);
```

The explicit defaults request is equivalent:

```matlab
options = obstacleAvoidance.planTrajectory("hs3");
```

A partial options structure may omit `PlannerMethod` or set it to `"hs3"`:

```matlab
options = struct( ...
    "PlannerMethod", "hs3", ...
    "MaximumSeedCount", 3);
```

No other planner selector value is supported.

## Minimal fixed-goal example

```matlab
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    "protected rectangle", ...
    [0; 20], ...
    [-1; 1; 1; -1], ...
    [-2; -2; 2; 2], ...
    0.2);

initialState = struct( ...
    "time_s", 0, ...
    "position_deg", [-5 0]);

goalState = struct( ...
    "time_s", 12, ...
    "position_deg", [5 0]);

limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2]);

options = obstacleAvoidance.planTrajectory();
result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, options);
```

## Public planning contract

The public fixed-goal interface is:

```matlab
result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, options);
```

### Obstacles

Use `obstacleAvoidance.obstacles.createObstacle` for static or time-indexed polygon
histories and `obstacleAvoidance.obstacles.createMovingObstacle` for moving shapes.
Safety margins are applied by the package constructors exactly once, and
original and protected geometry remain separate.

`obstacles` may be an obstacle array, nested cells of obstacles, or `[]`.
Obstacle coordinates use degrees and history times use seconds.

### Initial and goal states

Each state requires:

- `time_s`: scalar time in seconds;
- `position_deg`: one-by-two `[azimuth elevation]` position in degrees.

Optional `velocity_deg_s` and `acceleration_deg_s2` fields default to zero.
A moving goal additionally supplies increasing `targetTime_s`, matching
`targetPosition_deg`, and its interpolation method.

### Limits

The required physical limits are:

- `maxVelocity_deg_s`;
- `maxAcceleration_deg_s2`;
- `maxJerk_deg_s3`.

Each is a positive one-by-two `[azimuth elevation]` limit. Optional
`azimuthInterval_deg` and `elevationInterval_deg` fields define the workspace;
their defaults are `[-180 180]` and `[-90 90]` degrees.

### Options

Call `obstacleAvoidance.planTrajectory()` or `obstacleAvoidance.planTrajectory("hs3")` to inspect the exact HS3
options. Partial override structures are accepted, and empty fields retain
their defaults.

HS3 supports fixed-arrival and earliest-arrival requests through
`GoalTimeMode`. It uses bounded deterministic topology proposals, so success
means that an independently validated motion was found by the configured
finite search. It is not a proof of global completeness or optimality.

## Moving-target interception

The public call forms are:

```matlab
result = obstacleAvoidance.planMovingTargetIntercept( ...
    initialState, targetMotion, limits, interceptOptions);

result = obstacleAvoidance.planMovingTargetIntercept( ...
    obstacles, initialState, targetMotion, limits, interceptOptions);
```

Configure HS3 inside `PlannerOptions`:

```matlab
interceptOptions = obstacleAvoidance.planMovingTargetIntercept();
interceptOptions.InterceptMode = "earliest";
interceptOptions.PlannerOptions = obstacleAvoidance.planTrajectory();
```

Earliest interception performs a bounded chronological sequence of
fixed-arrival trials and refines the first observed feasible bracket.
Specified-time interception performs one fixed-arrival trial.

## Results and diagnostics

Success and expected failure return the same stable result structure. Important
fields include:

- `Success`, `Message`, and `TerminationReason`;
- `time_s`, `position_deg`, `velocity_deg_s`, `acceleration_deg_s2`, and
  `jerk_deg_s3` on success;
- `Seeds`, `SeedSummaries`, and `SelectedSeedIndex`;
- `Validation` and complete input records;
- `SearchDiagnostics`, including graph and rejection evidence.

Wall-time accounting is reported under
`result.SearchDiagnostics.StageTiming`. The topology, seed-corridor
construction, motion solving, collision checking, final validation, and
unattributed contributions are exclusive and sum to the independently
measured total.

Expected no-path, work-limit, and dynamic-infeasibility outcomes return
`Success=false` with a recognized termination reason and retained diagnostics.
Invalid input contracts throw errors. Use
`obstacleAvoidance.validateTrajectory` for independent full-trajectory
validation and `obstacleAvoidance.plotting.plotTrajectory` to visualize a
returned motion or preserved failure diagnostics.

## HS3 behavior and limitations

- HS3 uses an actual third-order Hermite-Simpson finite-jerk transcription
  with `fmincon`.
- It preserves supported nonzero endpoint velocity and acceleration states.
- It exposes collocation, mesh, iteration, evaluation, collision
  relinearization, and cooperative planning-budget controls.
- Each route leg receives at least one segment when permitted, and configured
  bounded mesh-refinement passes may be attempted.
- One solver evaluation can finish after the cooperative deadline, so measured
  elapsed time may overrun `MaximumPlanningTime_s`. Set
  `DeterministicWorkBudget` to release that budget and keep only the
  machine-independent caps, so a result reproduces across machines.
- HS3 is a local nonlinear method. Conditioning warnings, local minima, or
  local failure can occur even when another proposal may be feasible.
- Stationary-obstacle constraints bound the trajectory continuously between
  constraint times. Moving and deforming obstacles retain ordered frozen-time
  associations, with independent adaptive validation authoritative between
  those times.
- Static scenes may stop after the first independently validated seed to
  protect the wall-time budget and can miss a faster unattempted topology. A
  motion pinned to the goal horizon never stops that search.
- Exact exhaustive static searches that reject the direct edge and find no
  route return failure without spending work on a topology-preserving motion
  solve. Reduced, truncated, and dynamic searches retain motion attempts.
- Arrival quality is mesh limited. A coarse `CollocationSegmentCount` reports a
  later arrival than the limits allow; a finer one arrives earlier at higher
  integrated jerk and materially more solve time.
- Timed topology proposals first preserve their causal event timing at the
  proposed arrival, then use bounded fixed-time feasibility bisection to seek
  an earlier independently validated arrival on the same topology.

## Repository layout

```text
+obstacleAvoidance/                 obstacle-avoidance product namespace
|-- planTrajectory.m                public HS3 planning entry point
|-- planMovingTargetIntercept.m     chronological intercept adapter
|-- validateTrajectory.m            public independent validation
|-- +input/                         request, endpoint, and option contracts
|-- +obstacles/                     construction, queries, and history
|-- +geometry/                      boundary and clearance primitives
|-- +search/                        topology, visibility, and corridors
|-- +planner/                       orchestration and HS3 adaptation
`-- +plotting/                      public result-driven plotting

hs3/                                dimension-neutral motion product
|-- solveTrajHS3.m                  public trajectory entry point
`-- +hs3Internal/                   neutral implementation ownership
    |-- +polynomial/                reconstruction, evaluation, and basis math
    |-- +constraints/               continuous and fixed constraint assembly
    |-- +solver/                    fixed/free-time optimization orchestration
    |-- defaultOptions.m            engine-wide option defaults
    `-- validate.m                  independent engine-result validation

examples/                           maintained deterministic scenarios
sandbox/                            persistent manual scene builder
tests/                              automated contracts and regressions
benchmarks/                         focused scaling evidence
benchmark.csv                       chronological measured example records
branch_assessment.md                strengths, weaknesses, and limitations
verification.md                     commands and historical evidence
```

Add the repository root for `obstacleAvoidance` and the neutral HS3 folder:

```matlab
repositoryRoot = pwd;
addpath( ...
    repositoryRoot, ...
    fullfile(repositoryRoot, "hs3"));
```

The generic HS3 engine is directly callable with dimension-neutral state and
limit records:

```matlab
trajectory = solveTrajHS3( ...
    initialState, terminalState, limits, options, pathConstraints);
```

### API migration

The folder refactor removes redundant compatibility facades. Update existing
callers as follows:

| Previous call | Current call |
| --- | --- |
| `planAzElMotion(...)` | `obstacleAvoidance.planTrajectory(...)` |
| `planAzElMovingTargetIntercept(...)` | `obstacleAvoidance.planMovingTargetIntercept(...)` |
| `validateAzElTrajectory(...)` | `obstacleAvoidance.validateTrajectory(...)` |
| `azElObstacles.makeAzElObstacleData(...)` | `obstacleAvoidance.obstacles.createObstacle(...)` |
| `azElObstacles.makeMovingAzElObstacleData(...)` | `obstacleAvoidance.obstacles.createMovingObstacle(...)` |
| `azElObstacles.combineAzElObstacles(...)` | `obstacleAvoidance.obstacles.combineObstacles(...)` |
| `azElObstacles.queryAzElTimeObstacle(...)` | `obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(...)` |
| `plotAzElMotion(...)` | `obstacleAvoidance.plotting.plotTrajectory(...)` |
| `hs3.solve(...)` | `solveTrajHS3(...)` |

## Maintained examples

Add the example folder alongside the two production paths:

```matlab
addpath(repositoryRoot, ...
    fullfile(repositoryRoot, "hs3"), ...
    fullfile(repositoryRoot, "examples"));
```

Run a headless example with HS3:

```matlab
result = exampleObstacleAvoidance(struct( ...
    "PlannerMethod", "hs3", ...
    "PlotOutputs", false, ...
    "FigureVisible", "off"));
```

The maintained examples cover static, moving, and deforming obstacles;
concave and geographic geometry; waiting; dense fields; moving targets;
azimuth wrapping; fixed and earliest arrival; and expected no-path
diagnostics. The persistent scene builder under `sandbox/` is a manual tool
outside the headless example matrix.

## Requirements

- MATLAB with `polyshape`, graph, table, string, and current graphics support.
- Optimization Toolbox for `fmincon`.
- A graphical MATLAB session for visible plots and the persistent scene builder.
  Planning, validation, and noninteractive examples can run headlessly.
- Geographic-outline examples may require their MATLAB geographic data and
  toolbox dependencies.

No network service, learned model, or external planner process is required.

## Verification and historical evidence

Exact measurements, source revisions, and known limitations remain in
`benchmark.csv`, `verification.md`, and `branch_assessment.md`. Some historical
records predate the HS3-only branch and therefore name implementations that
are no longer active. Treat those rows only as evidence for the recorded
revision; they do not describe the current public interface or prove current
correctness or performance.

Run the tracked MATLAB test tree with:

```matlab
results = runtests("tests", "IncludeSubfolders", true);
assertSuccess(results);
```

## Known limits

- The bounded deterministic proposal set is not complete.
- HS3 does not prove global minimum arrival or global optimality.
- Moving obstacles can require topology or timing proposals outside the
  configured finite portfolio.
- Azimuth wrapping with obstacles or moving goals remains unsupported.
- Local nonlinear solves can fail or encounter poor conditioning.
- Cooperative timeouts can overrun while an active callback or evaluation
  returns.

Keep unfavorable failures and runtime results visible. A successful example
demonstrates only the exercised case family, not universal feasibility.
