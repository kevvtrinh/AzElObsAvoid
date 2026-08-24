# Azimuth/Elevation Obstacle-Avoidance Planner

This branch exposes two fully separate Az/El planners behind one public
interface. The default compact corridor planner and the explicitly selected
Hermite-Simpson planner share neutral input, topology, obstacle, result, and
validation infrastructure, but neither method calls, warms, falls back to, or
merges results from the other:

| Planner | `PlannerMethod` | Default? | Motion strategy |
| --- | --- | :---: | --- |
| Compact corridor | `"corridorQuintic"` | Yes | Neutral bounded topology seeds followed by compact C3/C4 quintic motion and canonical independent validation. It uses no HS3 or nonlinear-programming solve. |
| Hermite-Simpson | `"hs3"` | No | Neutral bounded topology seeds followed by an actual third-order Hermite-Simpson transcription, `fmincon`, canonical independent validation, bounded collision relinearization, and optional mesh refinement. |

Both planners independently turn neutral topology proposals into motion and
accept only canonical independently validated trajectories. Selecting `hs3`
runs HS3 directly; successful HS3 results have
`SelectedMotionSource="hs3"` and contain no compact composition diagnostics.

The HS3 package currently owns 1,602 nonblank, noncomment production MATLAB
lines under its 2,000-line cap. That ownership count excludes genuinely neutral
shared dependencies, including request normalization, endpoint validation,
topology generation, obstacle queries, result construction, and canonical
trajectory validation. It does not include or depend on the corridor package.

The corridor package originated at `325-less-nlp` commit
`28526638886b69efdf6d697a942ad2c1207bcc04` and now contains the compact
replacement documented in its local guide. The HS3 snapshot comes from
`plan-325` commit `5a067112a9f880d015f52fb97538a99010871478`.
See the [method-package guide](+azElPlannerMethods/README.md), the
[corridor guide](+azElPlannerMethods/+corridor/README.md), and the
[HS3 guide](+azElPlannerMethods/+hs3/README.md) for current ownership and
dependency details.

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

### Standalone Hermite-Simpson planner

Ask for the HS3 defaults, then pass them to the same planning function:

```matlab
options = planAzElMotion("hs3");

result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
```

Those defaults configure actual HS3 work, including a 115-second cooperative
end-to-end planning budget. There is no improvement switch, compact warm start,
or compact fallback.

A partial options structure can select either method:

```matlab
options = struct( ...
    "PlannerMethod", "hs3", ...
    "MaximumSeedCount", 3);
```

Accepted selector values are exactly `"corridorQuintic"` and `"hs3"`, with
case normalized by the public dispatcher. An invalid name is an error.

`PlannerMethod` is the public two-method selector. The corridor snapshot also
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
physical request through the standalone Hermite-Simpson planner.

## Public planning contract

The public fixed-goal interface is:

```matlab
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
```

### Obstacles

Use `makeAzElObstacleData` for static or time-indexed polygon histories and
`makeMovingAzElObstacleData` for moving shapes. Safety margins are applied by
the public obstacle constructors exactly once. Both selections retain original
and protected geometry separately. The same `makeAzElObstacleData` owner also
normalizes a lone canonical obstacle record and rebuilds a canonical obstacle
container when passed a new absolute safety margin.

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
HS3-only solver options into the compact selection.

Both public selections support fixed-arrival and earliest-arrival requests through
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

One root adapter owns interception for both selections. Earliest interception
performs a bounded chronological sequence of fixed-arrival trials and refines
the first observed feasible bracket. Specified-time interception performs one
fixed-arrival trial. Every trial calls `planAzElMotion`, so `PlannerMethod`
changes the selected planner without changing target interpolation,
derivative matching, search order, or the `Intercept.Search` schema.

## Results and diagnostics

Success and expected failure return the same stable result structure. Important
fields include:

- `Success`, `Message`, and `TerminationReason`;
- `time_s`, `position_deg`, `velocity_deg_s`, `acceleration_deg_s2`, and
  `jerk_deg_s3` on success;
- `Seeds`, `SeedSummaries`, and `SelectedSeedIndex`;
- `Validation` and complete input records;
- `SearchDiagnostics`, including graph and rejection evidence.

Both selections expose the same exclusive wall-time accounting under
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

The selected public method is echoed in both:

```matlab
result.Options.PlannerMethod
result.SearchDiagnostics.PlannerMethod
```

`PlannerMethod="hs3"` runs only HS3 motion generation. A successful result has
`result.SelectedMotionSource="hs3"`; unsuccessful attempts remain visible in
`SeedSummaries` and `SearchDiagnostics` rather than composition fields.

Moving-target results also restore it under:

```matlab
result.Intercept.Options.PlannerOptions.PlannerMethod
```

Expected no-path, work-limit, or dynamic-infeasibility outcomes return
`Success=false` with a recognized termination reason and retained diagnostic
data. Invalid input contracts throw errors. Selecting corridor never invokes
HS3, and selecting HS3 never invokes corridor. Neither method silently falls
back to the other after failure.

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
- Runs an actual third-order Hermite-Simpson finite-jerk transcription with
  `fmincon`; it does not call or reuse compact motion.
- Preserves nonzero initial and terminal velocity and acceleration constraints.
- Exposes method-specific collocation, mesh, iteration, evaluation, and one
  cooperative end-to-end `MaximumPlanningTime_s` budget.
- Gives each route leg at least one segment when permitted, supports bounded
  collision relinearization, and can perform the configured bounded mesh
  refinement passes.
- Enforces time budgets cooperatively through setup checks and the `fmincon`
  output callback. One solver evaluation may finish after its deadline, so the
  measured elapsed time can overrun the requested budget.
- Is a local nonlinear method; conditioning warnings or local failure can
  occur even when another proposal might be feasible.

The package owns its standalone orchestrator, option resolver, affine HS3
sensitivity maps, exact fixed-time constraint matrices, hybrid earliest-time
constraint evaluation, jerk objective, solver, diagnostics schema, and
validation facade. Neutral search, seed-corridor construction, result
construction, and final validation live outside it. Its current 1,602
noncomment production lines remain below the 2,000-line cap.

## Repository layout

```text
planAzElMotion.m                    public fixed/moving-goal dispatcher
planAzElMovingTargetIntercept.m     shared chronological intercept adapter

+azElPlannerMethods/
    README.md                       selection and dependency guide
    +corridor/
        README.md
        plan.m
        +internal/
            +obstacles/
            +search/
            +motion/
    +hs3/
        README.md
        plan.m                      standalone HS3 orchestrator
        resolvePlannerOptions.m
        +internal/
            +motion/
                solveHs3.m
                hs3AffineSensitivity.m
                buildFixedHs3ConstraintMatrices.m
                evaluateHs3TrajectoryConstraints.m

+azElInternal/                     neutral planner invariants
    generateTopologySeeds.m
    normalizePlannerRequest.m
    validatePlannerEndpoints.m
    buildSeedCorridor.m
    certifySeedCorridor.m
    emptyPlannerResult.m

makeAzElObstacleData.m              construct, normalize, protect obstacles
makeMovingAzElObstacleData.m        moving polygon construction
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
headless maintained-example matrix. Both tabs can export a diagnosis-ready MAT
bundle containing the exact retained input, result, independent validation,
and reproduction commands.

For deterministic stress outside the maintained examples, add `benchmarks` to
the path and run `benchmarkRandomMovingPolygonStress`. Its default scenes use
three large 5-to-12-vertex obstacles that cross most of the Az/El workspace
while rotating 180 to 360 degrees. Every record preserves the exact inputs,
result, independent validation, and an input-external clear boundary witness.

## Requirements

- MATLAB with `polyshape`, graph, table, string, and current graphics support.
- Optimization Toolbox for the standalone HS3 method, which uses `fmincon`.
  The default compact path does not use nonlinear programming.
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
revisions. A pre-composition combined-branch run covered the former isolated
selector paths:

- All 18 noninteractive maintained examples exactly matched the corridor
  source baseline at the gated categorical fields and stable metrics.
- The same 18 examples exactly matched the HS3 source baseline after applying
  only the source branch's recorded collocation and improvement-time settings.
- Each method produced 17 independently validated successes and the expected
  validated `noValidatedSeed` result, for 36 fresh example runs with zero gated
  differences.
- Historical pre-composition copies demonstrated each former method snapshot
  in isolation. That evidence does not establish correctness or performance of
  the rewritten current standalone HS3 implementation.
- MATLAB Code Analyzer checked all 109 intended MATLAB files and reported zero
  messages. Two unrelated untracked report scripts were deliberately excluded.

That pre-composition evidence does not prove the current standalone HS3
orchestrator, exact-gradient implementation, timeout diagnostics, or current
dependency closure. Use the latest test and benchmark records for those
claims.

The current compact-corridor cutover supersedes the corridor motion evidence
above: its final source passed 133/133 tests, all 18 maintained example gates,
the 1/5/10/20-turn and 12-hairpin benchmarks, success and expected-failure
graphics smokes, and Code Analyzer across 99 intended MATLAB files. Every
successful maintained-example and scaling duration met or beat its frozen
legacy value. Exact current rows and known finite-search limitations are in
`verification.md` and `benchmark.csv`.

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

- Both public selections use finite deterministic proposal sets and are not complete.
- Neither planner proves global minimum arrival or global optimality.
- A moving obstacle can require topology or timing proposals outside the
  configured finite portfolio.
- Azimuth wrapping with obstacles or moving goals remains unsupported.
- HS3 can encounter local minima, infeasible nonlinear iterates, or numerical
  conditioning warnings.
- For static scenes HS3 stops after the first independently validated seed to
  protect the wall-time budget; it can therefore miss a faster or smoother
  unattempted topology.
- In earliest-arrival mode, timed `directWait` and time-expanded seeds are
  solved at their input-derived proposed arrival time. This preserves their
  timing law but can miss a faster HS3 solution on the same topology.
- The cooperative HS3 timeout may overrun while an active solver callback or
  function evaluation returns.
- Corridor motion can remain conservative or use small-clearance boundary
  routes even when a different smooth trajectory exists.
- The two methods share neutral infrastructure, but there is no automatic
  cross-method discovery, warm start, result merge, or silent fallback.

Keep unfavorable failures and runtime results visible. A successful example
demonstrates only the exercised case family, not universal feasibility.
