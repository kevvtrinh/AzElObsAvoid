# Obstacle-Avoidance Trajectory Planner

This branch provides one public obstacle-avoidance planner for trajectories in
the azimuth/elevation frame. The `obstacleAvoidance` namespace owns inputs,
obstacles, geometry, search, engine selection, validation, and plotting.
Dimension-neutral Ruckig-derived and Hermite-Simpson (HS3) engines live as
independent packages under `trajectory/`.

A successful result is accepted only after canonical independent validation.
The obstacle planner calls Ruckig directly for eligible obstacle-free fixed
targets. Obstacle-constrained and moving-target requests use topology search
and HS3. Neither trajectory engine contains Az/El or obstacle knowledge.

The HS3 implementation descends from the `plan-325` snapshot at commit
`5a067112a9f880d015f52fb97538a99010871478`. Planner and engine ownership is
summarized below in this repository-level guide.

## Quick start

The zero-input call returns obstacle-planner defaults:

```matlab
options = obstacleAvoidance.planTrajectory();

result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, options);
```

A partial options structure can override only the controls it needs:

```matlab
options = struct( ...
    "MaximumSeedCount", 3);
```

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

- Ruckig is selected only for an obstacle-free fixed-position target. It
  creates exact jerk-switching state-to-state motion without route search.
- A request outside the Ruckig switching family continues through the HS3 path.
  Physical infeasibility and independent-validation failures remain visible.
- Moving targets and every nonempty obstacle field use topology search and HS3.
- Routing and Ruckig-to-Az/El result translation are local to the obstacle
  planner; no neutral trajectory wrapper exists.

- HS3 uses an actual third-order Hermite-Simpson finite-jerk transcription
  with `fmincon`.
- It preserves supported nonzero endpoint velocity and acceleration states.
- It exposes collocation, mesh, iteration, evaluation, and collision
  relinearization controls.
- Each route leg receives at least one segment when permitted, and configured
  bounded mesh-refinement passes may be attempted.
- Candidate count, mesh passes, relinearizations, solver iterations, and
  function evaluations bound planner work independently of machine speed. The
  HS3 engine retains its own per-solve safety stop.
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
|-- planTrajectory.m                public obstacle-planning entry point
|-- planMovingTargetIntercept.m     chronological intercept adapter
|-- validateTrajectory.m            public independent validation
|-- +input/                         request, endpoint, and option requirements
|-- +obstacles/                     construction, queries, and history
|-- +geometry/                      boundary and clearance primitives
|-- +search/                        topology, visibility, and corridors
|-- +planner/                       engine routing and result assembly
`-- +plotting/                      public result-driven plotting

trajectory/                         independent dimension-neutral engines
|-- +ruckigEngine/                  exact jerk-switching implementation
|   |-- solve.m                     direct Ruckig-derived entry point
|   `-- +internal/                  Ruckig normalization and validation
|-- +hs3Engine/                     collocation implementation
|   |-- solve.m                     direct HS3 entry point
|   |-- +polynomial/                reconstruction and basis math
|   `-- +constraints/               continuous constraint assembly
`-- THIRD_PARTY_NOTICES.txt         source and publication notices

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

Both engines are directly callable with dimension-neutral state and limit
records:

```matlab
ruckigTrajectory = ruckigEngine.solve( ...
    initialState, terminalState, limits, ruckigOptions);

hs3Trajectory = hs3Engine.solve( ...
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
| `hs3.solve(...)` | `hs3Engine.solve(...)` |

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
- Optimization Toolbox for `fmincon`.
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
- HS3 does not prove global minimum arrival or global optimality.
- Moving obstacles can require topology or timing proposals outside the
  configured finite portfolio.
- Azimuth wrapping with obstacles or moving goals remains unsupported.
- Local nonlinear solves can fail or encounter poor conditioning.
- Cooperative timeouts can overrun while an active callback or evaluation
  returns.

Keep unfavorable failures and runtime results visible. A successful example
demonstrates only the exercised case family, not universal feasibility.
