# Azimuth-Elevation Motion Planner

This repository plans collision-free, time-parameterized motion in an
azimuth-elevation workspace. It supports static and moving polygon obstacles.
It returns a stable result for success and expected planning failure.

## Main Interface

Use the public planner with one or two outputs:

```matlab
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);

[result, diagnostics] = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
```

The first output contains the decision, selected route, trajectory, resolved
request, independent validation, and elapsed time. The second output contains
protected geometry, packed collision data, full search traces, candidate
optimization records, and timed-path certificate details.

Positions use `[azimuth elevation]` order in degrees. Time uses seconds.
Velocity, acceleration, and jerk use the units in their field names. Call
`planAzElMotion()` to get all default options.

The planning sequence is:

1. Validate and pack obstacle geometry.
2. Build snapshot visibility graphs.
3. Search a safe-interval state-time roadmap.
4. Optimize motion with HS-3 Hermite-Simpson direct collocation.
5. Independently validate collision, state, and motion constraints.

Expected no-path, time-limit, and iteration-limit outcomes use
`result.Success = false`. Invalid input contracts cause identified errors.

## Public Functions

- `planAzElMotion` is the maintained planner entry point.
- `planAzElMovingTargetIntercept` adds moving-target goal policies.
- `makeAzElObstacleData` and `makeMovingAzElObstacleData` create canonical
  obstacle records.
- `buildAzElTimeObstacleField` packs collision data.
- `queryAzElTimeObstacle` and `queryAzElTimedPathCollision` provide public
  collision checks.
- `validateAzElExampleResult` independently validates example results.
- `plotAzElMotion` and `animateAzElTimedSlopePath` consume planner data.

The `+azElInternal` package contains shared implementation rules. These files
are not separate planner interfaces. The deprecated
`buildAzElSpaceTimeVisibilityGraph` function forwards to the maintained
safe-interval roadmap for compatibility only.

## Repository Layout

- `+azElInternal/`: search, optimization, packed-data, and schema helpers.
- `examples/`: runnable scenarios with local inputs and independent checks.
- `tests/`: deterministic MATLAB unit tests.
- `benchmarks/`: example timing and baseline comparison tools.
- `citation.md`: source papers and stable links.

## Quick Headless Run

```matlab
addpath(pwd, fullfile(pwd, "examples"));
result = exampleAzElPlanning(struct( ...
    "FigureVisible", "off", ...
    "ShowAnimation", false, ...
    "PlotKinematics", false));
assert(result.Success && result.ExampleValidation.Passed);
```

Examples return one result. They request planner diagnostics internally for
independent validation and plots. Call `planAzElMotion` directly when full
diagnostics are required.

Run tests with:

```matlab
results = runtests("tests");
assert(all([results.Passed]));
```

Run maintained examples one process at a time. Record the result and failure
diagnosis before the next example starts. Do not run examples in parallel.

## Maintenance Rules

`AGENTS.md` defines the planner contract, MATLAB style, validation rules, and
required example reports. Do not add scenario-specific planner branches. A
planner change must implement an input-driven rule and must keep independent
validation active.

See `citation.md` for the SIPP, SIPP-IP, direct-collocation, and collision
methods used by this implementation.
