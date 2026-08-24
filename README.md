# Azimuth/Elevation Obstacle-Avoidance Planner

This branch contains two complete Az/El trajectory planners behind one public
interface:

| Planner | `PlannerMethod` | Default? | Motion strategy |
| --- | --- | :---: | --- |
| Corridor quintic | `"corridorQuintic"` | Yes | Bounded visibility and timed seeds followed by corridor-constrained continuous quintic motion. It uses no HS3 or nonlinear-programming solve. |
| HS3 | `"hs3"` | No | Bounded seeds, deterministic analytic first motions when applicable, and optional HS3 finite-jerk nonlinear optimization with `fmincon`. |

The methods are genuine alternatives. Both consume the same canonical
obstacle normalization and time-query layer, while selecting one runs only
that method's search, motion construction, validation, and result builder.
The dispatcher never runs both, silently substitutes one for the other, or
falls back after a selected method fails.

The corridor snapshot comes from `325-less-nlp` commit
`28526638886b69efdf6d697a942ad2c1207bcc04`. The HS3 snapshot comes from
`plan-325` commit `5a067112a9f880d015f52fb97538a99010871478`.
See the [method-package guide](+azElPlannerMethods/README.md), the
[corridor guide](+azElPlannerMethods/+corridor/README.md), and the
[HS3 guide](+azElPlannerMethods/+hs3/README.md) for ownership and unplugging
details.

## Quick start

### Default corridor planner

The zero-input defaults call and a planning call without options both select
`corridorQuintic`:

```matlab
options = planAzElMotion();

result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
```

The explicit form is equivalent:

```matlab
options = planAzElMotion("corridorQuintic");
```

### HS3 planner

Ask for the HS3 defaults, then pass them to the same planning function:

```matlab
options = planAzElMotion("hs3");

result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
```

A partial options structure can select either method:

```matlab
options = struct( ...
    "PlannerMethod", "hs3", ...
    "MaximumSeedCount", 3);
```

Accepted selector values are exactly `"corridorQuintic"` and `"hs3"`, with
case normalized by the public dispatcher. An invalid name is an error.

`PlannerMethod` is the public two-planner selector. The corridor snapshot also
retains `MotionMethod="corridorQuintic"` as a compatibility field. Changing
`MotionMethod` does not select HS3.

## Minimal fixed-goal example

```matlab
obstacles = makeAzElObstacleData( ...
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

options = planAzElMotion("corridorQuintic");
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
```

Replace the defaults call with `planAzElMotion("hs3")` to run the same
physical request through HS3.

## Public planning contract

The public fixed-goal interface is:

```matlab
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
```

### Obstacles

Use `makeAzElObstacleData` for static or time-indexed polygon histories and
`makeMovingAzElObstacleData` for moving shapes. Safety margins are applied by
the public obstacle constructors exactly once. Both planners retain original
and protected geometry separately.

`obstacles` may be an obstacle array, nested cells of obstacles, or `[]`.
Obstacle history coordinates use degrees and history times use seconds.

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

Call `planAzElMotion("corridorQuintic")` or `planAzElMotion("hs3")` to inspect
the exact options accepted by that method. Partial override structures are
accepted, and empty fields retain method defaults. Avoid copying
method-specific solver options into the other planner.

Both methods support fixed-arrival and earliest-arrival requests through
`GoalTimeMode`. They use bounded deterministic proposal sets, so a successful
result is an independently validated motion found by the selected finite
search—not a proof of global completeness or global time optimality.

## Moving-target interception

The public call forms are:

```matlab
result = planAzElMovingTargetIntercept( ...
    initialState, targetMotion, limits, interceptOptions);

result = planAzElMovingTargetIntercept( ...
    obstacles, initialState, targetMotion, limits, interceptOptions);
```

Select the planner inside `PlannerOptions`:

```matlab
interceptOptions = planAzElMovingTargetIntercept();
interceptOptions.InterceptMode = "earliest";
interceptOptions.PlannerOptions = planAzElMotion("hs3");
```

The method-local adapters deliberately preserve different source-branch
behavior:

- Corridor earliest interception performs a bounded chronological sequence of
  fixed-arrival trials and refines the first observed feasible bracket.
- HS3 earliest interception makes one moving-goal earliest-arrival planner
  call.
- Specified-time interception supplies the requested fixed arrival to the
  selected method.

The dispatcher does not blend these policies.

## Results and diagnostics

Success and expected failure return the same stable result structure. Important
fields include:

- `Success`, `Message`, and `TerminationReason`;
- `time_s`, `position_deg`, `velocity_deg_s`, `acceleration_deg_s2`, and
  `jerk_deg_s3` on success;
- `Seeds`, `SeedSummaries`, and `SelectedSeedIndex`;
- `Validation` and complete input records;
- `SearchDiagnostics`, including graph and rejection evidence.

Both methods expose the same exclusive wall-time accounting under
`result.SearchDiagnostics.StageTiming`:

- `TopologyElapsedTime_s`;
- `CorridorConstructionElapsedTime_s`;
- `MotionSolvingElapsedTime_s`;
- `CollisionCheckingElapsedTime_s`;
- `FinalValidationElapsedTime_s`;
- `UnattributedElapsedTime_s` and `TotalElapsedTime_s`.

The five named stages do not overlap. `UnattributedElapsedTime_s` retains
public-call work outside those stages, and all six contributions add to the
independently measured total.

The method that actually ran is echoed in both:

```matlab
result.Options.PlannerMethod
result.SearchDiagnostics.PlannerMethod
```

Moving-target results also restore it under:

```matlab
result.Intercept.Options.PlannerOptions.PlannerMethod
```

Expected no-path, work-limit, or dynamic-infeasibility outcomes return
`Success=false` with a recognized termination reason and retained diagnostic
data. Invalid input contracts throw errors. A selected method's planning
failure never causes the dispatcher to try the other method.

Use `validateAzElTrajectory` for independent complete timed-trajectory
validation and `plotAzElMotion` to visualize returned motion or preserved
failure diagnostics. Plotting does not rerun planning.

## What differs between the methods

### Corridor quintic

- Default public planner.
- Contains no HS3 path and does not call `fmincon`.
- Uses a finite visibility/timing seed portfolio.
- Builds corridor-constrained continuous quintic motion and an exact
  jerk-switching profile for applicable straight requests.
- Uses bounded linear or quadratic feasibility and timing work rather than a
  nonlinear program.
- Can still miss feasible topology or return a slower locally constructed
  motion; no completeness or global-optimum claim is made.

### HS3

- Must be selected explicitly.
- Preserves deterministic analytic stop-at-waypoint first motions when their
  certificates apply.
- Uses HS3 collocation and `fmincon` for bounded nonlinear improvement or
  recovery.
- Exposes method-specific collocation, mesh, iteration, evaluation, and HS3
  time-budget options.
- Is a local nonlinear method; conditioning warnings or local failure can
  occur even when another proposal might be feasible.

Keeping the implementations in separate folders is intentional. Shared
mathematical and search invariants live in `+azElInternal`; method packages
retain planner policy, solver behavior, certificates, and diagnostics whose
semantics differ.

## Repository layout

```text
planAzElMotion.m                    public fixed/moving-goal dispatcher
planAzElMovingTargetIntercept.m     public moving-target dispatcher

+azElPlannerMethods/
    README.md                       method selection and unplugging guide
    +corridor/
        README.md
        plan.m
        planMovingTargetIntercept.m
        +internal/
            +geometry/
            +obstacles/
            +search/
            +motion/
            +validation/

+azElInternal/                     method-independent planner invariants
    +hs3/
        README.md
        plan.m
        planMovingTargetIntercept.m
        +internal/
            +geometry/
            +obstacles/
            +search/
            +motion/
            +validation/

makeAzElObstacleData.m              protected polygon construction
makeMovingAzElObstacleData.m        moving polygon construction
inflateAzElObstacleData.m           explicit obstacle inflation helper
validateAzElTrajectory.m            public independent validation
plotAzElMotion.m                    result and diagnostics plotting

examples/                           maintained deterministic scenarios
sandbox/                            persistent manual scene builder
tests/                              automated contracts and regressions
benchmarks/                         focused scaling and method evidence
benchmark.csv                       chronological measured example records
branch_assessment.md                strengths, weaknesses, and limitations
verification.md                     commands and historical evidence
```

The package folders are backend implementation boundaries, not new application
entry points. Prefer the root public functions unless you are maintaining the
dispatcher itself.

## Maintained examples

Add the example folder once:

```matlab
addpath("examples");
```

Run a headless example with either method:

```matlab
result = exampleAzElPlanning(struct( ...
    "PlannerMethod", "corridorQuintic", ...
    "PlotOutputs", false, ...
    "FigureVisible", "off"));
```

The 18 noninteractive maintained examples are:

1. `exampleAlternatingSlalom`
2. `exampleAzElPlanning`
3. `exampleDenseConcaveAzElMotion`
4. `exampleFortyMovingCircleGrid`
5. `exampleFourAcceleratingCircles`
6. `exampleInterceptMovingTargetAtSetTime`
7. `exampleInterceptMovingTargetEarliest`
8. `exampleMovingBarrierWait`
9. `exampleMovingCircleNoAzimuthWrap`
10. `exampleMovingDeformingUSOutlineVisibility`
11. `exampleNoPathAzElMotion`
12. `exampleObstacleFreeAzElMotion`
13. `exampleOpeningUShapedAzElTimeSpace`
14. `exampleStraightTargetAlternatingOcclusion`
15. `exampleTargetExitsObstacle`
16. `exampleTwoOpposingUVisibilityGraph`
17. `exampleUShapedAzElTimeSpace`
18. `exampleUSOutlineExtremeVisibility`

They cover static, moving, and deforming obstacles; concave and geographic
geometry; waiting; dense fields; moving targets; azimuth wrapping; fixed and
earliest arrival; and expected no-path diagnostics.

`exampleAzElInteractiveSandbox` is a manual one-run example. The persistent
two-tab scene builder lives under `sandbox/`; see the
[sandbox guide](sandbox/README.md). Its guided input order is start, goal or
first endpoint, then obstacle drawing. Interactive tools are not part of the
headless maintained-example matrix.

## Requirements

- MATLAB with `polyshape`, graph, table, string, and current graphics support.
- Optimization Toolbox for the preserved optimization routines. HS3
  specifically requires `fmincon`; the corridor method does not use nonlinear
  programming.
- A graphical MATLAB session for visible plots and the interactive sandbox.
  Planning, validation, and noninteractive examples can run headlessly.
- Geographic-outline examples may require the MATLAB geographic data and
  toolbox support used by their private geometry helpers.

No network service, learned model, or external planner process is required.

## Verification status

Historical evidence remains in `benchmark.csv`, `verification.md`, and
`branch_assessment.md`:

- The Plan-325 HS3 source recorded a complete 18-example matrix with 17
  independently validated successes and the expected validated no-path result,
  plus a historical 59-test pass.
- The 325-less-NLP corridor source recorded the same 17-success/one-expected-
  failure example coverage and a historical 59-test pass. Its final comments
  and sandbox-only follow-up were explicitly not rerun in that session.

Those historical rows prove only the source branches at their recorded
revisions. Fresh combined-branch evidence now covers the new dispatchers,
package qualification, both selector paths, and physical unplugging:

- All 18 noninteractive maintained examples exactly matched the corridor
  source baseline at the gated categorical fields and stable metrics.
- The same 18 examples exactly matched the HS3 source baseline after applying
  only the source branch's recorded collocation and improvement-time settings.
- Each method produced 17 independently validated successes and the expected
  validated `noValidatedSeed` result, for 36 fresh example runs with zero gated
  differences.
- A corridor-only temporary copy ran and validated with the HS3 folder absent;
  an HS3-only copy did the same with the corridor folder absent.
- MATLAB Code Analyzer checked all 109 intended MATLAB files and reported zero
  messages. Two unrelated untracked report scripts were deliberately excluded.

Exact measurements and source tags are appended to `benchmark.csv` and
explained in `verification.md`; historical rows were not replaced.

The full tracked MATLAB test tree is run with:

```matlab
results = runtests("tests", "IncludeSubfolders", true);
assertSuccess(results);
```

The source-branch rows remain historical evidence. Fresh verification claims
for later combined-branch changes are recorded in `verification.md` and
`branch_assessment.md`; the visible interactive sandbox remains a manual tool
outside the automated matrix.

## Known limits

- Both planners use finite deterministic proposal sets and are not complete.
- Neither planner proves global minimum arrival or global optimality.
- A moving obstacle can require topology or timing proposals outside the
  configured finite portfolio.
- Azimuth wrapping with obstacles or moving goals remains unsupported.
- HS3 can encounter local minima, infeasible nonlinear iterates, or numerical
  conditioning warnings.
- Corridor motion can remain conservative or use small-clearance boundary
  routes even when a different smooth trajectory exists.
- Removing a method folder without updating both public dispatchers leaves an
  explicit missing-package error. There is no automatic discovery or fallback.

Keep unfavorable failures and runtime results visible. A successful example
demonstrates only the exercised case family, not universal feasibility.
