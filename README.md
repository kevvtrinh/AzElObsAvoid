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
geometric edge. It stops at each waypoint. This family is available for a
fixed-position goal with zero initial and terminal velocity and acceleration.
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

Use `makeAzElObstacleData` to construct protected static or sampled moving
polygons. It stores original and protected geometry. It applies the safety
margin exactly once. Use `makeMovingAzElObstacleData` when a callback creates
each moving or deforming source slice.

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
options.MaximumPlanningTime_s = 60;
```

The deadlines are cooperative. The planner checks them between bounded units
of work and inside adaptive validation. One unit can finish after its check.
The result reports `FirstValidatedMotionTime_s` and
`PlanningDeadlineOverrun_s`. A reported overrun is not hidden or changed to
zero.

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
- elapsed time, first-valid time, deadline overrun, and deterministic seed.

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
deadlines, moving targets, and stable no-path diagnostics.

The current measured results are in `benchmark.csv`. The evidence-based branch
assessment is in `branch_assessment.md`. Update both records when planner
evidence changes.

## Known limits

- The finite seed set is not complete. It can miss a feasible topology.
- The homology signature classifies routes around sampled spatial regions. It
  does not classify continuous Az/El/time paths. Reduced or merged regions can
  also merge signature classes. Diagnostics report the representatives,
  discovered signatures, state count, and truncation state.
- The deterministic first-motion family requires a fixed-position goal and
  zero endpoint velocity and acceleration. It stops at geometric waypoints.
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
- Cooperative deadlines can have a small reported overrun.
- Azimuth wrapping with obstacles or moving goals is unsupported.
- No result proves global feasibility, completeness, or optimality.
