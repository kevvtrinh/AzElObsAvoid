# Compact Plan 325 azimuth/elevation planner

This branch contains one production planner:

```matlab
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
```

The pipeline is:

```text
canonical original and protected obstacles
    -> bounded direct, sampled spatial, reduced, and timed seed proposals
    -> independently validated finite-jerk first motions when supported
    -> optional bounded separated HS3 improvement
    -> independent continuous validation
    -> deterministic candidate selection
```

The first-motion constructor uses one rest-to-rest quintic segment on each
geometric edge. It stops at each waypoint. This family supports a
fixed-position goal and a sampled moving goal at a fixed arrival time when
initial and terminal velocity and acceleration are zero.
The planner validates first motions in seed order. It stops this stage at the
first pass before it spends the bounded optional HS3 improvement budget.

The separated third-order Hermite-Simpson (HS3) solve is an optional
improvement stage. It is enabled by default. Its default improvement budget is
15 seconds through `MaximumHs3ImprovementTime_s`. HS3 is required when the
first-motion family does not support the endpoint state, moving goal, or timed
wait structure. A failed or worse HS3 solve does not replace a valid first
motion.

The result is the earliest independently validated candidate from the finite
seed and motion families that the planner attempted. A tie uses integrated
squared jerk and then the deterministic seed index. The result is not a
global time-optimality or path-completeness certificate.

## Requirements

- MATLAB R2024b or a compatible release.
- Optimization Toolbox for the HS3 stage and for requests that require HS3.

## Minimal use

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

## Repository layout

The stable public API stays at the repository root. Internal implementation is
split by responsibility so visibility search, obstacle interpolation, motion
generation, and certification are not mixed in one folder:

```text
planAzElMotion.m                 public planner
planAzElMovingTargetIntercept.m  moving-target convenience adapter
make* / normalize* / query*      public obstacle construction and queries
validateAzElTrajectory.m         independent trajectory validator
plotAzElMotion.m                 returned-result visualization

+azElInternal/
  +geometry/                     polygon and clearance primitives
  +obstacles/                    prepared dynamic obstacle histories
  +search/                       visibility, homology, and timed seeds
  +motion/                       quintic construction and retiming
  +validation/                   corridor and envelope certificates
  README.md                      internal dependency map
examples/                        maintained executable scenarios
sandbox/                         persistent interactive scene-building UI
tests/                           deterministic regression suites
benchmarks/                      reproducible performance investigations
```

Names inside `+azElInternal` intentionally omit repeated `AzEl` and planner
words where the containing package already supplies that context. For example,
the visibility entry point is `azElInternal.search.generateTopologySeeds` and
the candidate builder is `azElInternal.motion.solveCorridorQuintic`.

Use `makeAzElObstacleData` to construct protected static or sampled moving
polygons. It stores original and protected geometry. It applies the safety
margin exactly once. Use `makeMovingAzElObstacleData` when a callback creates
each moving or deforming source slice.

The planner prepares dynamic obstacle data once per planning call. This
preparation keeps one shape for every source slice. It also keeps
interval-specific interpolation data when adjacent slices have matching
topology. When topology changes, it keeps a conservative union for that
interval only. The planner does not replace a moving or deforming history with
one static shape. Planning queries, motion construction, HS3, plotting, and
independent validation reuse this immutable prepared data. Public results keep
the canonical obstacle format and do not expose the internal cache.

## Seed and geometry policy

The seed generator returns at most five deterministic seeds by default. It can
produce these proposal types:

- a direct seed;
- unreduced sampled or reduced spatial visibility seeds;
- original-geometry sampled time-layer seeds with motion and wait edges when
  their estimated query work is inside the bounded seed-search limit.

The spatial visibility search augments each node with an integer 2-D homology
signature. It keeps the shortest route for each discovered signature. One
interior representative defines each connected sampled obstacle region. Each
signature component is limited to one winding, the search uses at most 4,000
augmented states, and the public seed limit still controls returned routes.
The graph checks Delaunay candidate edges plus every start and goal connection.
This removes many long obstacle-node pairs before collision tests. The result
field `VisibilityCandidatePairCount` reports the number of tested pairs. This
sparse proposal is deterministic, but it is not a complete visibility graph.

Dense-history support envelopes and optional obstacle clusters are permitted
only for spatial seed proposals and their corridor certificates. Diagnostics
set `UsesReducedGeometry` when a seed uses this reduction. The dense-history
work gate suppresses sampled timed search and reports `timedQueryWorkLimit`.
This can miss a wait topology. When timed search runs, its edge samples use the
original protected history. HS3 and final validation always use that history.
Only final adaptive validation gives a continuous collision certificate. A
reduced region never replaces the obstacle data used to accept a trajectory.

`EstimatedDuration_s` is an initial guess for HS3. It is not a required lower
bound. HS3 can shorten or extend the motion when the goal-time policy and
physical constraints permit this change.

## Motion stages

The planner uses two motion stages:

1. It constructs deterministic stop-at-waypoint motions in seed order until
   one passes independent validation.
2. It runs bounded HS3 improvement when `EnableHs3Improvement` is true. It
   also runs HS3 when no valid first motion exists.

The first stage gives the planner a fast, finite-jerk result for a common
endpoint family. It does not make HS3 a fallback that can bypass validation.
Both stages return the same polynomial and sampled-history contract. Both
stages must pass `validateAzElTrajectory` before selection.

Use these main controls:

```matlab
options.EnableHs3Improvement = true;
options.MaximumHs3ImprovementTime_s = 15;
```

`MaximumHs3ImprovementTime_s` limits only optional HS3 work after a valid
motion exists. Required planning work uses deterministic seed, graph, solver
iteration, function-evaluation, collocation, and refinement bounds. The
planner has no whole-planner wall-clock cutoff. The result reports
`FirstValidatedMotionTime_s` and total elapsed planning time.

Workspace intervals belong to `limits`:

```matlab
limits.azimuthInterval_deg = [-180 180];
limits.elevationInterval_deg = [-90 90];
```

The planner supplies these values when they are omitted. Old workspace option
names and `MaximumPlanningTime_s` give actionable migration errors.

## Azimuth wrapping

Azimuth wrapping is currently supported only for obstacle-free requests with
a fixed-position goal. The planner rejects wrapping with any obstacle or with
a moving goal. This restriction prevents an incorrect collision or target
interpretation across the coordinate seam. Use an unwrapped obstacle and goal
coordinate system before you request these cases.

## Result contract

Every expected outcome returns the same main fields:

- `Success`, `Message`, and `TerminationReason`;
- resolved `Options` and `Inputs`;
- attempted `Seeds`, `SeedSummaries`, and `SelectedSeedIndex`;
- `SelectedMotionSource`, `ArrivalTime_s`, `TrajectoryDuration_s`, and
  `GoalHorizon_s`;
- sampled `time_s`, `position_deg`, `velocity_deg_s`,
  `acceleration_deg_s2`, and `jerk_deg_s3`;
- the exact segment `Polynomial` used for continuous validation;
- independent `Validation` and bounded `SearchDiagnostics`;
- elapsed time, first-valid time, and the deterministic seed field.

Expected planning failure does not throw. Invalid inputs and unsupported
contracts throw identified errors.

## General diagnostics

Set `options.Verbose = true` for concise planner diagnostics. The output
reports the topology-graph size, each attempted seed, each motion source, the
solver and validation outcome, arrival time, largest reported violation,
termination reason, and selected result.

`result.SearchDiagnostics` is available when verbose output is disabled. Use
`plotAzElMotion` to show original and protected obstacles, reduced seed-only
regions, accepted and rejected graph edges, explored nodes, the frontier, the
best partial route, and the selected motion. A failed result can therefore
produce a diagnostic plot without a selected trajectory.

The plotter uses the `main` branch visual conventions for original and
protected obstacles, candidate and selected routes, returned motion, moving
targets, and animation state.

## Maintained examples

The `examples` directory contains 14 main scenarios and four focused
verification scenarios. They cover obstacle-free motion, static and moving
obstacles, concave and geographic geometry, fixed and earliest moving-target
intercepts, waiting, obstacle-free azimuth wrapping, dense fields, and
expected failure.

For quick manual scene design, use
`exampleAzElInteractiveSandbox`. It lets you draw a forbidden path and
polygon obstacles in az/el space, then run the planner directly against
that geometry.

For a persistent two-tab workspace, add `sandbox` to the MATLAB path and call
`azElInteractiveSandbox`. Each tab guides the first three inputs in order:
click the start, click the goal (or first free-mode endpoint), and then draw
the first obstacle. **Add Obstacle** starts each additional obstacle stroke;
the interface does not require separate start or goal buttons.

Every example uses the same planner, validator, and plotter. A failed result
can show the retained visibility edges, rejected edges, explored states,
frontier data, and best partial seed without a second planner call.

Fixed-goal examples use earliest arrival. Moving-target examples state either
an earliest intercept or a specified intercept time as part of the request.

Run an example without figures as follows:

```matlab
addpath("examples");
result = exampleAzElPlanning(struct( ...
    "PlotOutputs", false, ...
    "FigureVisible", "off"));
```

## Verification

```matlab
results = runtests("tests/testHs3Planner.m");
assertSuccess(results);
```

Also run `tests/testBuildAzElStopWaypointMotion.m` and
`tests/testExampleContracts.m`. The focused tests cover the deterministic
first-motion family, the HS3 jerk chain, endpoint constraints, earliest
arrival, mesh refinement, seed diversity, unreduced and reduced seed policy,
moving geometry, waiting seeds, continuous collision and kinematic failures,
safety-margin provenance, wrapping restrictions, deterministic repetition,
the removed planner-timeout contract, moving targets, and stable no-path
diagnostics.

The current measured results are in `benchmark.csv`. The evidence-based branch
assessment is in `branch_assessment.md`. Update both records when planner
evidence changes.

## Known limits

- The finite seed set is not complete. It can miss a feasible topology.
- The homology signature classifies routes around sampled spatial regions. It
  does not classify continuous Az/El/time paths. Reduced or merged regions can
  also merge signature classes. Diagnostics report the representatives,
  discovered signatures, state count, and truncation state.
- The deterministic first-motion family requires zero endpoint velocity and
  acceleration. It supports a moving goal only at a fixed arrival time and
  stops at geometric waypoints.
- HS3 is a local nonlinear optimizer. It can fail or return a local solution.
- Spatial seed reduction can remove a useful narrow proposal. It cannot make
  an invalid motion pass because HS3 and validation use original protected
  obstacle histories.
- The moving-obstacle topology graph uses at most 17 time layers with bounded
  nodes and samples. It supplies proposals only.
- A seed-only convex region needs an independent containment and continuous
  corridor-clearance certificate before a first motion can use it.
- Adjacent obstacle slices with different topology use a conservative union
  inside the source interval. An unresolved validation interval fails.
- Concave obstacle corridors use fixed local boundary associations. Final
  validation checks all exact protected boundaries.
- Mesh refinement is optional. `MaximumMeshRefinementPasses` is zero by
  default. A refined result cannot replace a candidate when it is later or has
  a worse exact jerk cost.
- Azimuth wrapping with obstacles or moving goals is unsupported.
- No result proves global feasibility, completeness, or optimality.
