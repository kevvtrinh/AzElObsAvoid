# Supplied Vietnam boundary benchmark

The source CSV and implementation plan were committed as `3235d57` on
`build-core`, on top of `b35b4e9`. The CSV preserves all 842 pasted pairs and
their decimal text. It contains 280 rows at 2770 s and 281 rows at each of
2910 and 3000 s. The latter two include an exact repeated closing point.
The fixture removes only those two copies for computation, then interpolates
each corresponding vertex over the two anchor intervals. The planner input
has exactly 921 slices at 0.25 s spacing, each with 280 vertices.

## Reproduce the authoritative outcome

```matlab
addpath(pwd,fullfile(pwd,'examples'),fullfile(pwd,'trajectory'));
[obstacle,initialState,goalState,limits] = createVietnamBoundaryScenario();
prepared = obstacleAvoidance.obstacles.prepareObstacles( ...
    obstacle,[initialState.time_s,goalState.time_s]);
models = prepared.InternalPreparation.IntervalGeometryModel;
assert(all(models == "unsupportedContinuousDeformation"));

result = planner(prepared,initialState,goalState,limits, ...
    struct('GoalTimeMode','fixedArrival','WrapX',false));
assert(~result.Success);
assert(result.TerminationReason == "unsupportedObstacleInterpolation");
assert(isempty(result.time_s));
```

For plots, run `exampleVietnamBoundarySlew()`. This is a new supplied-data
fixture, separate from the historical eight-obstacle `exampleVietnamKeepoutSlew`.
It is also registered in `runExampleBenchmarks` without borrowing historical
reference metrics from a different case.

## Planner status and interpretation

This fixture is **not a successful trajectory benchmark**. Every one of its
920 intervals changes a concave polygon by a non-rigid deformation. Matching
vertex indices do not specify how the occupied interior moves continuously
between samples. The source geometry remains authoritative, so the preparer
classifies all intervals as `unsupportedContinuousDeformation` and the public
planner returns that stable failure.

No convex hull, endpoint swept hull, or other replacement geometry is used.
Historical claims of a 230 s validated motion, a 113.146007006023-unit path,
or an 8.9 s successful runtime depended on an unproved endpoint-hull
interpolation fallback and are not acceptance evidence for the current
geometry contract.

`tests/testVietnamBoundary.m` checks source fidelity, both interpolation
intervals, retained original geometry, all 920 interval classifications,
absence of a convex-hull substitution, and the public failure record.
Supporting this fixture as a planning case requires a new explicit continuous
occupancy model from the caller; it must not be inferred merely to pass this
case.
