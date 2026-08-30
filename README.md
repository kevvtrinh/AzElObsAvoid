# Obstacle-Avoidance Trajectory Planner

This branch provides one public obstacle-avoidance planner for trajectories in
the azimuth/elevation frame. The `obstacleAvoidance` namespace owns inputs,
obstacles, geometry, topology search, candidate selection, validation, and
plotting. Dimension-neutral motion generation lives independently under
`trajectory/+bmtpEngine`.

A successful result is accepted only after canonical independent validation.
The planner converts static protected geometry to numeric convex exclusion
regions before calling BMTP; the engine never imports obstacle or planner
packages. Exact jerk switching, event-word integration, quintic offset splines,
polynomial evaluation, and the static-region SOCP/Bezier generator share that
one engine-owned representation.

## Quick start

Add both production parents. The zero-input call then returns obstacle-planner
defaults:

```matlab
addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));

options = obstacleAvoidance.planTrajectory();

result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, options);
```

A partial options structure can override only the controls it needs:

```matlab
options = struct( ...
    "MaximumSeedCount", 3);
```

Planner options select work and display policy; they do not expose internal
engine constants. Per-seed engine and independent-validation evidence is
retained in `result.SearchDiagnostics.SeedSummaries`.

`PerSeedWorkBudgetMultiplier` defaults to `3`. After an independently
validated topology-seed motion exists, each later BMTP solve receives a
deterministic work limit equal to this multiplier times the fastest validated
seed solve so far, with a one-second floor. A work-limited seed remains visible
in `SeedSummaries` with termination reason `seedWorkBudgetExhausted`.

`MaximumTimeLayerCount` defaults to `17` and bounds the timed visibility
search layers, including the initial and goal times. Lower values reduce
timed-search work but can omit obstacle-event times and lose feasible routes.

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

## Public planning requirement

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

Call `obstacleAvoidance.planTrajectory()` to inspect the exact planner options.
Partial override structures are accepted, and empty fields retain their
defaults.

Fixed-arrival and earliest-arrival requests use `GoalTimeMode`. The obstacle
planner uses bounded deterministic topology proposals when geometry is present,
so success
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

Configure obstacle planning inside `PlannerOptions`:

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
Invalid input requirements throw errors. Use
`obstacleAvoidance.validateTrajectory` for independent full-trajectory
validation and `obstacleAvoidance.plotting.plotTrajectory` to visualize a
returned motion or preserved failure diagnostics.

## Engine routing and limitations

- Obstacle-free rest-to-rest requests use the exact synchronized switching
  kernel. Fixed arrival stretches the same law without increasing derivatives.
- Static topology seeds are converted by the planner to convex numeric regions;
  BMTP alternates time-power and separating-plane SOCPs over composite Bezier
  curves. Plane witnesses certify only those supplied regions.
- Input-driven cavity, timed-opening, and fixed-clock lateral constructions
  remain planner-owned because they interpret obstacle geometry and timing.
  Their event words and quintic polynomials are generated by BMTP.
- The public validator remains authoritative for obstacle coverage, continuous
  collision freedom, workspace, endpoints, velocity, acceleration, and jerk.
- A validated candidate is not a general global-optimality proof. Exact or
  bounded arrival claims are returned only when a request-wide physical lower
  certificate applies; other results remain feasible incumbents.
- Work limits, exhausted topology search, unsupported dynamic families, and
  physical infeasibility remain visible as stable failure results.

## Repository layout

```text
+obstacleAvoidance/                 obstacle-avoidance product namespace
|-- planTrajectory.m                public obstacle-planning entry point
|-- planMovingTargetIntercept.m     chronological intercept adapter
|-- validateTrajectory.m            public independent validation
|-- +input/                         request, endpoint, and option requirements
|-- +obstacles/                     construction, queries, and history
|-- +geometry/                      boundary and clearance primitives
|-- +search/                        topology, visibility, and corridors
|-- +planner/                       engine routing and result assembly
|-- +validation/                    planner-domain continuous certificates
`-- +plotting/                      public result-driven plotting

trajectory/                         independent dimension-neutral motion
`-- +bmtpEngine/
    |-- solve.m                     numeric-region SOCP/Bezier generator
    |-- createDelayedMotion.m       dwell and event-time repartitioning
    |-- createDirectMotion.m        exact synchronized jerk switching
    |-- createMotionRecord.m        event-word integration and sampling
    |-- createOffsetSplineMotion.m  fixed-clock quintic composition
    |-- maximumRestToRestDistance.m exact scalar reachability bound
    `-- evaluatePolynomial.m        shared polynomial evaluator

examples/                           maintained deterministic scenarios
sandbox/                            persistent manual scene builder
tests/                              automated requirements and regressions
benchmarks/                         focused scaling evidence
benchmark.csv                       chronological measured example records
branch_assessment.md                strengths, weaknesses, and limitations
verification.md                     commands and historical evidence
```

Add the repository root and the trajectory package parent:

```matlab
repositoryRoot = pwd;
addpath( ...
    repositoryRoot, ...
    fullfile(repositoryRoot, "trajectory"));
```

The engine is directly callable for dimension-neutral rest-to-rest motion:

```matlab
motion = bmtpEngine.createDirectMotion( ...
    initialState, goalState, limits, options);
```

### API migration

Use the named product entry points in new code. Historical product-level names
map as follows; removed engine implementations have no forwarding shims:

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

## Maintained examples

Add the example folder alongside the two production paths:

```matlab
addpath(repositoryRoot, ...
    fullfile(repositoryRoot, "trajectory"), ...
    fullfile(repositoryRoot, "examples"));
```

Run a headless obstacle-avoidance example:

```matlab
result = exampleObstacleAvoidance(struct( ...
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
- Optimization Toolbox for `coneprog` in the static-region BMTP generator.
- A graphical MATLAB session for visible plots and the persistent scene builder.
  Planning, validation, and noninteractive examples can run headlessly.
- Geographic-outline examples may require their MATLAB geographic data and
  toolbox dependencies.

No network service, learned model, or external planner process is required.

## Verification and historical evidence

Exact measurements, source revisions, and known limitations remain in
`benchmark.csv`, `verification.md`, and `branch_assessment.md`. Some historical
records predate the separated-engine architecture and therefore name implementations that
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
- BMTP and the bounded candidate portfolio do not prove global minimum arrival
  or global optimality.
- Moving obstacles can require topology or timing proposals outside the
  configured finite portfolio.
- Azimuth wrapping with obstacles or moving goals remains unsupported.
- Local nonlinear solves can fail or encounter poor conditioning.
- Cooperative timeouts can overrun while an active callback or evaluation
  returns.

Keep unfavorable failures and runtime results visible. A successful example
demonstrates only the exercised case family, not universal feasibility.
