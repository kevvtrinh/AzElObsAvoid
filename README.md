# Obstacle-Avoidance Trajectory Planner

This branch provides one public obstacle-avoidance planner for trajectories in
any two-dimensional x/y coordinate system. Use one consistent coordinate unit
for both axes; the planner does not convert units. The `obstacleAvoidance` namespace owns inputs,
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

The public planner takes `obstacles, initialState, goalState, limits, options`
in that order. Internal functions take only the inputs they use.

Add both production parents. The zero-input call then returns obstacle-planner
defaults:

```matlab
addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));

options = obstacleAvoidance.planTrajectory();

result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, options);
```

Add these folders explicitly; `genpath(repositoryRoot)` also includes old code
under `tmp` and can mix planner and engine versions. After switching revisions,
refresh the session from the repository root:

```matlab
addpath(pwd, fullfile(pwd, "trajectory"), fullfile(pwd, "examples"), "-begin");
clear functions
rehash
```

`which('bmtpEngine.createMotionRecord', '-all')` should resolve first to
`trajectory/+bmtpEngine/createMotionRecord.m` in the current checkout.

BMTP uses degree-eight Bezier curves and MATLAB `coneprog` for trajectory and
separating-plane conic programs. Optimization Toolbox is required; no external
MEX solver or Python runtime is needed. Ordinary regions use three spans per
segment, while timed and conservatively grouped regions use one. Solver counts
and timings remain in the returned diagnostics.

A partial options structure can override only the controls it needs:

```matlab
options = struct( ...
    "MaximumSeedCount", 5, ...
    "UnsupportedTimedTopologyPolicy", "fail");
```

Planner options control planning, while plot options control display.
`diagnosis.Attempts` summarizes the tried paths; `diagnosis.SolverDetails`
contains their detailed solver evidence.

`MaximumSeedCount` defaults to `2`. The planner compares at most those first
two seeds on the normal path. Values from `3` through `5` enable failure-only
recovery: later seeds are tried in order only when neither initial seed nor an
exact motion passed full validation, and recovery stops at its first validated
motion. Set the value to `2` to disable all later-seed recovery or `5` to allow
all three additional attempts.

BMTP is the obstacle-planning method; there is no method-selection option.
Standalone Ruckig utilities and the explicit `ruckigStopAtWaypoints` fallback
remain available; that fallback is disabled by default.

`GoalTimeMode` defaults to `"earliestArrival"`: prioritize arrival time and
break ties by shorter travel. Set `"fixedArrival"` to minimize travel at the
mission horizon. Jerk remains a hard limit.

For an eligible rest-to-rest request with one coordinate owning the physical
clock, the planner compares fixed-clock offset splines whose peaks come from
direct collision intervals. It then adjusts interior spline offsets to shorten
travel while preserving that clock and the governing coordinate. Each retained
adjustment must pass continuous public validation, including moving obstacles
and all kinematic limits. This bounded local refinement spends additional
planning time; it does not certify a globally shortest path. Diagnostics retain
the starting lobe reports and the final `TravelRefinement` offsets separately.

`MaximumTimeLayerCount` defaults to `17` and bounds timed BMTP motion segments
plus one. Timed visibility search retains every input-derived endpoint, source,
and midpoint time because discarding those times can lose feasible routes.

`MaximumWaitRefinementIterations` defaults to `16`. For an earliest-arrival
`directWait` seed that already passes independent validation, the planner first
validates zero wait and then bisects the measured infeasible/feasible wait
bracket. Every trial uses `obstacleAvoidance.validateTrajectory`; fixed-arrival
requests are not refined. Set the option to `0` to retain the accepted grid wait.

`UnsupportedTimedTopologyPolicy` defaults to `"fail"`. When the smooth timed
kernel cannot realize a multi-waypoint route, the original
`unsupportedTimedMultiWaypointRoute` failure remains visible. Set the policy to
`"ruckigStopAtWaypoints"` only when rest-to-rest composition at every interior
waypoint is acceptable and the normalized route has no more than two segments.
Longer routes fail with `ruckigWaypointSegmentLimitExceeded` before Ruckig is
run. The result reports the original failure, forced rest states, fallback
method, and fallback outcome.

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
    "position_units", [-5 0]);

goalState = struct( ...
    "time_s", 12, ...
    "position_units", [5 0]);

limits = struct( ...
    "maxVelocity_units_s", [2 2], ...
    "maxAcceleration_units_s2", [1 1], ...
    "maxJerk_units_s3", [2 2]);

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

[`obstacle_history_contract.md`](obstacle_history_contract.md) defines sample
activity, ring and hole semantics, linear corresponding-vertex interpolation,
the conservative swept fallback, and status metadata. Sampled rotation is not
interpreted as rigid arc motion, and unverified correspondence is enclosed
rather than interpolated.

`obstacles` may be an obstacle array, nested cells of obstacles, or `[]`.
Obstacle coordinates use coordinate units and history times use seconds.

### Initial and goal states

Each state requires:

- `time_s`: scalar time in seconds;
- `position_units`: one-by-two `[x y]` position in coordinate units.

Optional `velocity_units_s` and `acceleration_units_s2` fields default to zero.
A moving goal additionally supplies increasing `targetTime_s`, matching
`targetPosition_units`, and its interpolation method.

### Limits

The required physical limits are:

- `maxVelocity_units_s`;
- `maxAcceleration_units_s2`;
- `maxJerk_units_s3`.

All three must use the same form: positive finite scalars for combined
magnitudes, or two-element `[x y]` vectors for separate axis
limits. Mixing scalar and vector derivative limits raises
`planTrajectory:MixedLimitModes`.

A combined limit `L` is the hypotenuse and is allocated equally as
`[L/sqrt(2), L/sqrt(2)]` internally. For example:

```matlab
limits = struct();
limits.maxVelocity_units_s      = 2;
limits.maxAcceleration_units_s2 = 1;
limits.maxJerk_units_s3         = 2.5;
```

This fixes each axis's share; unused capacity on one axis is not transferred
to the other. `result.Inputs.limits` contains the resolved per-axis limits.
Moving-target interception and explicit `validateTrajectory` calls use the
same conversion, and reusing resolved limits does not divide them again.
MATLAB sandbox overrides and offline-sandbox JSON requests follow the same
rule; the sandbox controls display the resolved per-axis values.

Optional `xInterval_units` and `yInterval_units` fields remain
two-element workspace intervals; their defaults are `[-180 180]` and
`[-90 90]` coordinate units. Position intervals are never split or scaled.
The maintained examples use separate velocity and acceleration limits, so
their `MaxJerk_units_s3` override must also be a two-element vector.

### Options

Call `obstacleAvoidance.planTrajectory()` to inspect the exact planner options.
Partial override structures are accepted, and empty fields retain their
defaults.

`WrapX` and `WrapY` are independent logical options, both false by default.
An enabled axis is periodic with period `diff(limits.xInterval_units)` or
`diff(limits.yInterval_units)`. Any finite increasing interval is supported;
there is no fixed full-turn period. For example:

```matlab
limits.xInterval_units = [-5 5];     % x period: 10
limits.yInterval_units = [100 120];  % y period: 20
options.WrapX = true;
options.WrapY = true;
```

From `[4 119]`, a goal at `[-4 101]` becomes the nearest equivalent `[6 121]`.
An exact half-period tie selects the positive displacement.
Returned motion stays continuous in those unwrapped coordinates. Spatial plots
fold enabled axes into their intervals and split lines at either seam; a
separate continuous view preserves the actual motion. Disabled axes retain
their workspace bounds. Wrapping currently accepts only obstacle-free,
fixed-position goals; periodic obstacles and moving goals raise
`planTrajectory:UnsupportedWrappedGeometry`.

This API uses `position_units`, `velocity_units_s`, `acceleration_units_s2`,
and `jerk_units_s3`, with `[x y]` columns throughout. Rotation controls for
obstacle shapes still use degrees because those values are angles.

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

## Planning workflow

1. Standardize the request and prepare each obstacle history once.
2. Try a direct motion and a detour that preserves its duration.
3. If needed, build route-search geometry and find alternative paths.
4. Use those paths as initial guesses for the motion solver.
5. Check the complete motion, select a validated result, and return optional diagnosis.

Internal solvers use prepared obstacles directly. Public geometry queries and
`validateTrajectory` check and prepare caller-supplied data before using the
same geometry and validation implementation. Solver regions and conservative
moving-history enclosures are prepared once per request and reused across path
guesses. Safety margins are applied only
by obstacle construction, never again during preparation.

The path-guess stages have distinct jobs:

| Function | Job |
| --- | --- |
| `createEmptyPathGuess` | Define one empty record. |
| `createPathGuesses` | Add the direct guess and guesses from searched routes. |
| `createRoutePathGuesses` | Convert searched routes, including recovery routes. |
| `solvePathGuess` | Construct and validate motion for one guess. |
| `tryAdditionalPathGuesses` | Try later guesses only after the initial attempts fail. |

Some engineering terms used in the code:

| Term | Meaning |
| --- | --- |
| Path / route | Positions to pass through, without a motion schedule. |
| Trajectory / motion | Position, velocity, acceleration, and jerk over time. |
| Path guess / seed | An initial path supplied to the motion solver; it is not a feasible motion yet. |
| Obstacle envelope | An outline enclosing obstacles for search or conservative solving. |
| Corridor | An allowed region around a proposed path. |
| Visibility graph | Points connected where straight segments clear obstacles. |
| Route class | A different way around the obstacles. |
| Certificate | Mathematical evidence for a specific safety or constraint check. |
| Horizon | The time interval available for planning the motion. |

## Results and diagnostics

The first output contains the motion and everything needed to plot and independently
validate it. Request the second output when investigating a planning run:

```matlab
[result, diagnosis] = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, options);
obstacleAvoidance.plotting.plotTrajectory(result);
obstacleAvoidance.plotting.plotTrajectory(result, plotOptions, diagnosis);
```

`result` has the same fields on success and expected failure:

- `Success`, `Message`, `TerminationReason`: planning outcome.
- `Route_units`, `BestPartialRoute_units`: selected path or available failure path.
- `time_s`, `position_units`, `velocity_units_s`, `acceleration_units_s2`, `jerk_units_s3`: motion samples.
- `Inputs`, `Options`: normalized request, including original and protected obstacles.
- `Polynomial`, `PlaneCertificate`, `SeedCorridor`, `SeedCorridorBoundary_units`: exact motion and evidence used by independent validation.
- `Validation`, `ArrivalTime_s`, `TrajectoryDuration_s`, `ElapsedPlanningTime_s`.

`diagnosis` keeps the investigation data separate:

- `Routes`, `Attempts`, `SelectedAttemptIndex`: tried paths and their outcomes.
- `Search`, `SearchCoverage`: graph traces, complete counts, search limitations, and separate timed/spatial partial routes.
- `Timing`: exclusive stage times that add up to total planning time.
- `SolverDetails`, `VisibilityAttempts`: tables with `Attempt`, `Field`, and `Value` columns, including rejected edges and graph connectivity.
- `DirectMotion`, `PathRefinement`: field/value tables for the initial motion attempts.
- `Selection`: the actual ranking columns, values, and candidate order; jerk remains a hard constraint.
- Attempt counts and time to the first validated motion.

Candidate summaries use `ArrivalTime_s` and `TrajectoryDuration_s`, matching the
result. `Routes.ParameterBasis` identifies whether `tau` is normalized distance
or normalized time. `Routes.ObstacleEnvelope_units` stores the search obstacle
outline. `Validation.CertificateRejectionReason` explains a rejected timed
coverage certificate when validation proceeds to adaptive collision checks.

Detail tables use readable field paths instead of nested structures. For example,
filter `diagnosis.SolverDetails.Attempt == diagnosis.SelectedAttemptIndex` to inspect
the chosen attempt. The planner still collects search evidence during planning;
it assembles the optional detail tables only when the second output is requested.

`planMovingTargetIntercept` supports the same two outputs. Its result adds the
achieved intercept and terminal policies; its diagnosis adds `InterceptSearch`
and `InterceptOptions`.

Expected planning failures return `Success=false` and a reason. If a rejected
motion was constructed, its samples, polynomial, attempted arrival, and failed
`Validation` remain available for inspection; they do not indicate executable
motion. Otherwise those motion fields retain their documented empty values.
Invalid inputs
throw errors. `obstacleAvoidance.validateTrajectory(result)` checks the complete
motion independently, including between-sample collisions.

## Engine routing and limitations

- Obstacle-free rest-to-rest requests use the exact synchronized switching
  kernel. Fixed arrival stretches the same law without increasing derivatives.
- Static topology seeds are converted by the planner to convex numeric regions;
  BMTP alternates time-power and separating-plane SOCPs over composite Bezier
  curves. Plane witnesses certify only those supplied regions.
- For a moving-obstacle multi-waypoint seed, the planner first tests whether a
  stationary enclosure of every protected moving history permits a smooth
  spatial detour. If that fails and the guess has a time schedule, timed-cell
  BMTP constrains only the overlapping obstacle/motion intervals. Both attempts
  are validated against the original moving geometry.
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
|-- +search/                        visibility, route classes, and path guesses
|-- +planner/                       engine routing and result assembly
|-- +validation/                    planner-domain continuous certificates
`-- +plotting/                      public result-driven plotting

trajectory/                         independent dimension-neutral motion
`-- +bmtpEngine/
    |-- solve.m                     numeric-region SOCP/Bezier generator
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
| `planXYMotion(...)` | `obstacleAvoidance.planTrajectory(...)` |
| `planXYMovingTargetIntercept(...)` | `obstacleAvoidance.planMovingTargetIntercept(...)` |
| `validateXYTrajectory(...)` | `obstacleAvoidance.validateTrajectory(...)` |
| `xyObstacles.makeXYObstacleData(...)` | `obstacleAvoidance.obstacles.createObstacle(...)` |
| `xyObstacles.makeMovingXYObstacleData(...)` | `obstacleAvoidance.obstacles.createMovingObstacle(...)` |
| `xyObstacles.combineXYObstacles(...)` | `obstacleAvoidance.obstacles.combineObstacles(...)` |
| `xyObstacles.queryXYTimeObstacle(...)` | `obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(...)` |
| `plotXYMotion(...)` | `obstacleAvoidance.plotting.plotTrajectory(...)` |

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
fixed and earliest arrival; and expected no-path diagnostics. Dedicated tests
cover independent x/y wrapping. The persistent scene builder under `sandbox/` is a manual tool
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

The maintained documentation is this guide, [repository rules](AGENTS.md),
and the [obstacle history contract](obstacle_history_contract.md). The browser
sandbox has its own [usage guide](offlinesandbox/README.md).

Full recorded measurements remain in [benchmark.csv](benchmark.csv) and the CSVs
under [benchmarks](benchmarks/). See [planner decisions](branch_assessment.md)
for retained choices and [verification](verification.md) for the latest checked
scope. Historical rows describe their recorded revision, not current performance.

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
  configured finite portfolio. The stationary enclosure can close
  useful openings, and bounded timed-cell BMTP can fail on a feasible route.
  Unsupported guesses remain explicit in attempt diagnostics; failure does not
  establish physical infeasibility.
- Wrapping either axis with obstacles or moving goals remains unsupported.
- Local nonlinear solves can fail or encounter poor conditioning.
- Use Ctrl+C to interrupt MATLAB planning. The blocking HTTP server must be
  restarted after interruption.

Keep unfavorable failures and runtime results visible. A successful example
demonstrates only the exercised case family, not universal feasibility.

## Mathematical references

- Bhattacharya, S., Likhachev, M., and Kumar, V. (2012).
  [Search-Based Path Planning with Homotopy Class Constraints in 3D](https://doi.org/10.1609/aaai.v26i1.8435).
  The spatial search uses a bounded 2-D angle signature adapted from this
  approach. It is not the paper's 3-D construction or a continuous
  x/y/time homotopy certificate.
- Farouki, R. T. (2012).
  [The Bernstein Polynomial Basis: A Centennial Retrospective](https://doi.org/10.1016/j.cagd.2012.03.001).
  Bernstein convex-hull bounds certify complete polynomial intervals in the
  solver and independent validator.
- Perlin, K. (2002).
  [Improving Noise](https://doi.org/10.1145/566570.566636).
  Moving-obstacle scenarios use the quintic smoothstep
  `10*u^3 - 15*u^4 + 6*u^5` for zero endpoint velocity and acceleration.
- Historical HS3 implementation: Moreno-Martin, S., Ros, L., and Celaya, E.
  (2024), [Collocation Methods for Second and Higher Order Systems](https://doi.org/10.1007/s10514-023-10155-z).
  This reference explains the retired separated third-order collocation chain.
- Historical waypoint-state refinement: Koskela, P.,
  [rsruckig](https://github.com/petrikosk/rsruckig), MIT-licensed.
  The retired pass-through warm-start search adapted its local waypoint-state
  method in MATLAB; it did not embed or call the Rust implementation.

Retired HS3 implementation documents are kept in [the historical archive](docs/archive/hs3/README.md).
