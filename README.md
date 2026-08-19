# Compact HS3 azimuth/elevation planner

This branch contains one production planner:

```matlab
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
```

The pipeline is:

```text
canonical protected obstacles
    -> at most five deterministic visibility seeds by default
    -> separated third-order Hermite-Simpson optimization
    -> independent continuous validation
    -> earliest validated local solution
```

The result is the earliest independently validated local HS3 solution from
the finite seed set that was attempted. It is not a global time-optimality or
path-completeness certificate.

## Requirements

- MATLAB R2024b or a compatible release.
- Optimization Toolbox for `fmincon`.

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
polygons. It stores original and protected geometry and applies the safety
margin exactly once. Use `makeMovingAzElObstacleData` when a callback creates
each moving or deforming source slice.

## Result contract

Every expected outcome returns the same main fields:

- `Success`, `Message`, and `TerminationReason`;
- resolved `Options` and `Inputs`;
- attempted `Seeds`, `SeedSummaries`, and `SelectedSeedIndex`;
- sampled `time_s`, `position_deg`, `velocity_deg_s`,
  `acceleration_deg_s2`, and `jerk_deg_s3`;
- the exact segment `Polynomial` used for continuous validation;
- independent `Validation` and bounded `SearchDiagnostics`;
- elapsed time, deterministic seed, and the precise optimality statement.

Expected failure does not throw. Invalid inputs and unsupported contracts do
throw identified errors.

## General diagnostics

Set `options.Verbose = true` for concise planner diagnostics in any public
planner call. The output reports the topology-graph size, each attempted seed,
the optimizer and validation outcome, the arrival time, the largest reported
violation, the termination reason, and the selected result. Moving-obstacle
construction and inflation report periodic progress for large histories. The
nonlinear solver remains quiet, so its iteration table does not hide planner
events.

`result.SearchDiagnostics` remains available when verbose output is disabled.
Use `plotAzElMotion` to show original and protected obstacles, seed-only
envelopes, accepted and rejected graph edges, explored nodes, the frontier,
the best partial route, and the selected motion. A failed result can therefore
produce a useful diagnostic plot without a selected trajectory.

## Maintained examples

The `examples` directory contains 14 main scenarios and four focused
verification scenarios. They cover obstacle-free motion, static and moving
obstacles, concave and geographic geometry, fixed and earliest moving-target
intercepts, waiting, azimuth wrapping, dense fields, and expected failure.

Every example uses the same planner, validator, and plotter. A failed result
can show the retained visibility edges, rejected edges, explored states,
frontier data, and best partial seed without rerunning the planner.

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

The focused tests cover the analytic jerk chain, endpoint constraints,
earliest arrival, mesh refinement, topology diversity, translating and
deforming geometry, waiting seeds, between-node collision and kinematic
failures, safety-margin provenance, azimuth wrapping, deterministic
repetition, time-limit failure, moving-target adaptation, and stable no-path
diagnostics.

## Known limits

- HS3 is a local nonlinear optimizer. A finite seed set can miss a feasible
  topology.
- The spatial visibility graph uses protected boundary candidates. Moving
  obstacles add at most 17 time layers with straight motion and wait edges.
  This graph supplies initialization only. It does not certify dynamics or
  global optimality.
- One obstacle with a very dense swept history can use a conservative coarse
  directional support polygon for seed generation. The generator starts with
  16 directions and increases the count only when needed to keep the start or
  goal outside the polygon. It uses the exact convex hull if no coarse polygon
  preserves the request. This avoids a large polygon union without replacing
  the shape with a rectangle. Diagnostics and plots identify the envelope.
  HS3 and independent validation still use the exact protected history. The
  seed generator omits timed gap search because the envelope covers the
  complete swept history.
- `SeedClusterDistance_deg` is zero by default. A positive value replaces
  each connected group of at least three nearby swept regions with a
  conservative convex hull for seed generation. The plot shows these hulls.
  HS3 and independent validation still use every original protected obstacle.
  Cluster mode uses spatial routes around the swept hull and omits timed
  gap-seeking seeds. A large value can remove a useful narrow seed corridor.
- A seed-only convex envelope is converted to a continuous half-plane
  corridor. HS3 checks its complete segment polynomials. An independent
  certificate verifies envelope containment and continuous corridor clearance.
- Adjacent obstacle slices with different topology use a stationary
  conservative union inside the source interval. Source event times split HS3
  corridor checks. Continuous validation fails an interval when its motion
  bound cannot be resolved safely.
- Timed visibility seeds preserve causal lower-bound schedules. HS3 does not
  shorten a waiting or time-expanded seed below its estimated duration.
- Concave obstacle corridors use frozen local boundary-edge associations.
  Independent validation rejects contact with any other boundary part.
- Mesh refinement is supported for the selected candidate and is controlled by
  `MaximumMeshRefinementPasses`. Its default is zero to keep ordinary calls
  bounded. Set it to one or two when a denser final mesh is required.
