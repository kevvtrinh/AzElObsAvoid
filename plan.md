# Plan 325 Refactor Plan

## Objective

Refactor the existing `plan-325` branch without redesigning its planner.

Branch to modify:

`https://github.com/kevvtrinh/AzElObsAvoid/tree/plan-325`

Plotting reference:

`https://github.com/kevvtrinh/AzElObsAvoid/tree/main`

The purpose of this refactor is to improve consistency, diagnostics, API ownership, and plotting fidelity while preserving the current Plan 325 planning architecture and behavior.

Required changes:

1. Apply the plotting style and plotting behavior from `main` to `plan-325`.
2. Make verbose behavior consistent across every maintained example.
3. Move azimuth/elevation workspace limits from planner `options` into `limits`.
4. Remove `MaximumPlanningTime_s`.
5. Replace reliance on a whole-planner wall-clock cutoff with substantially better progress reporting through `Verbose`.
6. Preserve current planning algorithms, solver behavior, seed families, validation rules, and deterministic selection except where changes are required by this refactor.

The implementation must follow `AGENTS.md`.

# Non-Goals

This task is a refactor, not a new planning-method task.

Do not:

- replace the Plan 325 planner;
- add another planner;
- replace HS3;
- replace the deterministic first-motion stage;
- redesign seed generation;
- add SIPP;
- add scenario-specific planner logic;
- change safety-margin semantics;
- weaken collision or kinematic validation;
- change the result-selection policy;
- change example geometry merely to make a test pass;
- add a renamed replacement for `MaximumPlanningTime_s`.

Preserve the current Plan 325 pipeline:

```
canonical original/protected obstacles
    -> bounded deterministic seed proposals
    -> deterministic finite-jerk first motion when supported
    -> optional/general HS3 solve
    -> independent continuous validation
    -> deterministic candidate selection
```

# Code-Size Exception for Plotting

The existing Plan 325 size goals remain applicable to planner, solver, seed-generation, obstacle, validation, test, and example code.

However, **plotting and animation code are explicitly exempt from the existing code-base line limit when additional plotting code is needed to reproduce the plotting behavior of **`**main**`**.**

This exception exists specifically so plotting fidelity is not sacrificed merely to meet the old size target.

Allowed:

- increasing `plotAzElMotion.m`;
- increasing animation/plot helper code;
- adding plotting-only helpers when they materially improve organization;
- retaining diagnostic plotting data needed to reproduce `main`;
- restoring richer graph, obstacle, trajectory, kinematic, and animation visualization.

Not allowed:

- moving planner logic into plotting files to avoid the size limit;
- duplicating solver or geometry algorithms in plotting code;
- using the plotting exemption to justify unrelated code growth;
- changing planning results merely to make plots easier to reproduce.

At the end of the task, report line counts separately for:

1. core production code excluding plotting/animation;
2. plotting/animation code;
3. examples;
4. tests;
5. total maintained MATLAB code.

Plotting growth alone must not fail the Plan 325 size gate.

## Performance-Based Production Size Allowance

The production MATLAB target is 7,500 physical lines. Production can exceed
this target only when measured wall-time performance pays for the excess. Each
100 excess lines requires at least a 25 percent wall-time reduction. Apply the
requirement proportionally:

```text
required reduction = 0.25 * excess production lines / 100
```

Declare the representative affected benchmark set before evaluation. Use the
smallest reduction in that set. Do not use an average or a best case to hide a
regression. Use the same inputs, options, environment, and independent
validation. Correctness and arrival or route quality must stay within their
documented tolerances. Record all evidence in `verification.md`.

This allowance does not change the 900-line production-file limit, the
12,000-line maintained planner/test-tree limit excluding `examples/`, or any
correctness, generality, diagnostic, interface, and non-regression
requirement. Example files have no repository line cap, but their full line
count must still be reported and they must not be used to justify planner
growth.

# Phase 0 - Establish the Refactor Baseline

Before editing behavior:

1. Check out `plan-325`.
2. Read `AGENTS.md`.
3. Run the current focused test suite.
4. Run all maintained examples headlessly.
5. Record the current planner defaults returned by:

```
options = planAzElMotion();
```

1. Record all fields currently present in:
    - `options`;
    - normalized `limits`;
    - `result.SearchDiagnostics`;
    - `result.SeedSummaries`.
2. Capture representative plots from both:
    - `plan-325`;
    - `main`.

Use at least these scenario classes when comparing plots:

- obstacle-free;
- one static obstacle;
- multiple static obstacles;
- moving obstacle;
- moving target;
- failed/no-path result;
- a case with visibility-graph diagnostics.

The baseline must document existing behavior before changes.

Exit criterion:

- current tests and examples are recorded;
- representative `main` and `plan-325` figures are available for comparison;
- API migration locations are identified.

# Phase 1 - Restore the Plotting Style from `main`

## Source of truth

Treat the plotting behavior of `main` as the visual reference.

Do not blindly copy the entire main-branch planner or its old planning architecture.

Copy or adapt only plotting, animation, layout, formatting, and visualization behavior that is compatible with Plan 325's result diagnostics.

## Preserve visual conventions

The Plan 325 plotter should match `main` as closely as practical for the same type of result.

This includes, where applicable:

- figure titles;
- axes titles;
- figure naming;
- subplot/tiled-layout organization;
- grid visibility;
- box visibility;
- equal-axis behavior;
- azimuth/elevation labels;
- time-axis labels;
- kinematic plot layout;
- legend names;
- legend locations;
- line styles;
- marker styles;
- marker sizes;
- line widths;
- obstacle appearance;
- original-versus-protected obstacle appearance;
- selected-path appearance;
- candidate/seed route appearance;
- accepted visibility edges;
- rejected/blocked visibility edges;
- explored nodes;
- frontier nodes;
- best partial route;
- moving-target track;
- start marker;
- goal marker;
- animation appearance;
- diagnostic plotting for failure cases.

Do not replace descriptive main-branch titles with generic titles.

For example, preserve the concept of:

```
<example title> - <termination reason>
```

when that is the visual convention of the reference plot.

## Preserve plot scale and grid size

For equivalent examples, preserve the same workspace framing used by `main`.

The plotter should not independently choose dramatically different axes limits merely because Plan 325 has a different internal implementation.

Workspace limits should come from the canonical planner `limits` structure after the API migration described later.

Use:

```
limits.azimuthInterval_deg
limits.elevationInterval_deg
```

as the source for requested workspace bounds where appropriate.

Keep:

```
axis(..., "equal")
grid(..., "on")
box(..., "on")
```

where those are part of the main plotting convention.

Do not autoscale away the intended az/el workspace when the example explicitly defines one.

## Kinematic figures

Retain the main-style four-row layout:

```
position
velocity
acceleration
jerk
```

with shared time interpretation.

Plot azimuth and elevation consistently.

Show physical limit references where Plan 325 already has reliable access to those limits, but do not remove main-style layout or labeling to do so.

The plot title should remain the example/plot title, not an internal solver name.

## Failure diagnostics

A failed Plan 325 result must remain plottable.

Do not require `result.Success == true` to render useful workspace or search diagnostics.

For failed cases, show whatever Plan 325 retained, including when available:

- obstacles;
- direct route;
- sparse sampled visibility graph;
- accepted edges;
- blocked edges;
- explored nodes;
- frontier;
- best partial route;
- attempted seeds;
- moving target history.

A user should be able to inspect why a solve failed without rerunning the planner.

## Plotting data ownership

Do not make the plotter recompute the planner.

The plotter must consume diagnostics already stored in `result`.

If some information required to reproduce a useful main-style diagnostic is currently thrown away, extend `result.SearchDiagnostics` in the planner or seed generator to retain that data.

Do not perform a second graph search from `plotAzElMotion`.

## Plot verification

For each representative example, compare `main` and refactored `plan-325`.

Verification must explicitly inspect:

- title text;
- axes limits;
- grid;
- equal-axis behavior;
- obstacle rendering;
- selected motion rendering;
- graph rendering;
- start/goal rendering;
- moving-target rendering;
- kinematic layout;
- animation framing.

Pixel-perfect equality is not required because the planner internals differ.

Visual semantics and style should match.

Exit criterion:

- the Plan 325 plotting system visually follows `main`;
- failure diagnostics remain available;
- no planner algorithm was copied merely to support plotting.

# Phase 2 - Make Verbose Mode Consistent Across All Examples

## One planner control

Use one public planner control:

```
options.Verbose
```

Do not add:

```
VerboseSolver
VerboseGraph
VerboseExample
DisplayProgress
PrintProgress
```

or other overlapping public verbosity switches unless already required by a separate public contract.

Internal helper functions may receive the resolved verbose value, but there should be one public source of truth.

## Example contract

Every maintained example must expose the same override:

```
exampleOverrides.Verbose
```

The shared example option resolver must forward it into:

```
options.Verbose
```

Every example must behave the same way.

Do not have some examples hard-code:

```
options.Verbose = true;
```

while others leave it off.

Do not create example-specific `fprintf` logic that bypasses the planner's verbose system.

Recommended behavior:

- interactive example default: `Verbose = true`;
- tests: explicitly `Verbose = false`;
- benchmarks: explicitly `Verbose = false` unless verbose output itself is under test;
- zero-input planner default may remain `Verbose = false`.

The exact default may follow existing shared example conventions, but it must be uniform across all maintained examples.

## Example output responsibility

Examples may print one concise example-level summary after the planner returns.

Long-running progress output should come from the planner and its internal stages.

This keeps all examples consistent.

Exit criterion:

- every maintained example accepts the same `Verbose` override;
- changing `Verbose` produces consistent behavior across all examples;
- automated tests can run quietly.

# Phase 3 - Move Azimuth/Elevation Workspace Limits into `limits`

## Current fields to remove from `options`

Remove:

```
options.AzimuthInterval_deg
options.ElevationInterval_deg
```

These describe physical/planning workspace bounds and belong with the physical constraints, not algorithm options.

## New canonical fields

Use:

```
limits.azimuthInterval_deg
limits.elevationInterval_deg
```

The canonical public limits structure should therefore contain:

```
limits = struct( ...
    "azimuthInterval_deg", [-180 180], ...
    "elevationInterval_deg", [-90 90], ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2]);
```

Use lower-camel field naming because the existing physical limit fields use that convention.

## Required ownership change

After normalization, all planner stages must read the workspace from:

```
limits.azimuthInterval_deg
limits.elevationInterval_deg
```

This includes:

- endpoint validation;
- topology-seed generation;
- sparse Delaunay visibility candidates with complete start and goal access;
- time-layer search;
- HS3 bounds;
- independent trajectory validation;
- plotting;
- animation;
- moving-target wrapper;
- example metadata where applicable.

There must not be two active sources of truth.

## Defaults

The planner currently has default workspace intervals.

Preserve the same default numerical workspace:

```
[-180 180]
[-90 90]
```

but move ownership into limit normalization.

Because `limits` is already a required input, `normalizeLimits` should add these fields when omitted:

```
if ~isfield(limits, "azimuthInterval_deg") || isempty(...)
    limits.azimuthInterval_deg = [-180 180];
end

if ~isfield(limits, "elevationInterval_deg") || isempty(...)
    limits.elevationInterval_deg = [-90 90];
end
```

This avoids forcing every existing caller to specify global az/el limits while still making `limits` the canonical owner.

## Validation

Validate both as:

- numeric;
- real;
- finite;
- two elements;
- strictly increasing.

Normalize them to row vectors.

## Legacy-option handling

Do not silently ignore old fields.

If a caller passes:

```
options.AzimuthInterval_deg
```

or:

```
options.ElevationInterval_deg
```

raise an actionable migration error.

Example:

```
AzimuthInterval_deg has moved from planner options to
limits.azimuthInterval_deg.
```

Likewise for elevation.

This is preferable to an "unknown option ignored" warning because silently using the default workspace could change the requested problem.

## Repository migration

Update all maintained:

- examples;
- tests;
- benchmarks;
- README snippets;
- internal helper contracts;
- moving-target calls;
- validator calls;
- plot calls.

Search the entire repository for:

```
AzimuthInterval_deg
ElevationInterval_deg
```

At completion, those names may remain only in deliberate migration-error handling or historical documentation.

Exit criterion:

- `options` contains no active workspace intervals;
- all workspace consumers use `limits`;
- existing behavior is unchanged for numerically equivalent inputs.

# Phase 4 - Remove `MaximumPlanningTime_s`

## Remove the public option completely

Delete:

```
options.MaximumPlanningTime_s
```

This is not a rename.

Do not replace it with:

```
PlanningTimeout
PlannerTimeLimit
MaximumRuntime
WallClockLimit
```

or another whole-planner wall-clock cutoff.

## Remove all dependent behavior

Search the complete repository for:

```
MaximumPlanningTime_s
planningTimeLimit
PlanningDeadlineOverrun_s
InternalPlanningDeadline_s
InternalPlanningTimer
SeedPlanningTimeLimit_s
```

Then determine whether each use exists solely to implement the full planner wall-clock deadline.

Remove all such behavior.

This includes current logic that:

- gives seed generation a percentage of `MaximumPlanningTime_s`;
- stops seed processing when total elapsed planner time reaches the limit;
- skips seeds because the wall-clock limit expired;
- sets `planningTimeLimit` termination;
- propagates the overall planner deadline into HS3;
- propagates the overall planner deadline into validation;
- computes planner deadline overrun;
- reports deadline-specific search diagnostics;
- uses the planner time limit to decide whether a failure is `planningTimeLimit` or `noValidatedSeed`.

## Failure semantics after removal

If the finite deterministic seed/motion families complete without a valid trajectory, use the normal bounded-search failure semantics:

```
noValidatedSeed
```

or the existing more-specific non-timeout failure reason.

A failure must describe the actual algorithmic outcome, not a removed wall-clock deadline.

## Preserve elapsed-time diagnostics

Keep:

```
result.ElapsedPlanningTime_s
```

and useful per-stage elapsed durations.

Elapsed time becomes diagnostic only.

It must not change planner decisions.

`FirstValidatedMotionTime_s` may also remain because it is useful progress/performance information.

Remove `PlanningDeadlineOverrun_s` if it has no remaining meaning after the whole-planner deadline is deleted.

## Preserve deterministic work bounds

Removing the wall-clock cutoff must not make the algorithm structurally unbounded.

Continue to use deterministic or algorithmic limits such as:

```
MaximumSeedCount
MaximumNlpIterations
MaximumNlpFunctionEvaluations
MaximumMeshRefinementPasses
MaximumCollocationSegmentCount
```

and existing finite graph/state/sample bounds.

Do not increase them as part of this refactor unless required to preserve behavior after removing a time-based shortcut.

## `MaximumHs3ImprovementTime_s`

Do not automatically remove `MaximumHs3ImprovementTime_s` in this task.

It is a separate bounded optional-improvement policy.

However, decouple it from `MaximumPlanningTime_s`.

The optional HS3 improvement stage may continue to stop when its own configured improvement budget is exhausted.

Required behavior:

- if a valid first motion exists, optional HS3 may still be bounded by
`MaximumHs3ImprovementTime_s`;
- if HS3 is required because the first-motion family cannot represent the
request, do not incorrectly apply the old full-planner deadline;
- preserve existing deterministic solver iteration/function-evaluation bounds.

If current code has coupled the required-HS3 case to `MaximumPlanningTime_s`,
refactor that path carefully so removing the planner timeout does not make
required HS3 terminate immediately or inherit a stale deadline.

## Removed-field migration behavior

If a caller supplies:

```
MaximumPlanningTime_s
```

do not silently ignore it.

Raise a clear removal error:

```
MaximumPlanningTime_s has been removed.
Use Verbose=true to observe planner progress.
Planner work remains bounded by finite seed, graph, solver iteration,
function-evaluation, collocation, and refinement limits.
```

## Documentation

Remove claims that Plan 325 has a complete-planner 60-second deadline.

Update:

- `README.md`;
- `plan.md` or the active refactor document;
- `baseline.md`;
- `verification.md`;
- benchmark setup;
- comments;
- test names;
- result-contract descriptions.

Exit criterion:

- no whole-planner deadline controls execution;
- elapsed wall time is diagnostic only;
- no stale `planningTimeLimit` result is produced;
- bounded algorithmic work remains explicit.

# Phase 5 - Bolster Verbose Mode to Show Real Progress

The replacement for `MaximumPlanningTime_s` is not another timeout.

Instead, make long planner activity observable.

A user running:

```
options.Verbose = true;
```

should be able to tell what the planner is doing and whether progress is being made.

## Required progress stages

Verbose mode must report at least:

### Planner start

Report:

- goal time mode;
- obstacle count;
- workspace azimuth interval;
- workspace elevation interval;
- start time;
- goal horizon;
- maximum seed count;
- whether HS3 improvement is enabled.

Example:

```
[AzEl] Planning started.
[AzEl][setup] workspace az=[-10 10] deg, el=[-8 8] deg
[AzEl][setup] obstacles=3, seeds<=5, goalMode=earliestArrival
```

### Seed generation

Report:

- seed generation start;
- graph-building stage;
- node count;
- accepted/visible edge count;
- rejected edge count;
- expanded state count;
- number of candidate seeds;
- number retained;
- truncation/work-limit reason if a finite graph bound is reached;
- elapsed seed-generation time.

Example:

```
[AzEl][seeds] generating topology proposals...
[AzEl][seeds] nodes=84, visibleEdges=311, rejectedEdges=96, expanded=142
[AzEl][seeds] retained 4 seeds in 0.42 s
```

### First-motion attempts

For every attempted seed report:

- seed index/count;
- seed source/type;
- waypoint count or route length;
- whether first-motion construction is supported;
- whether independent validation passed;
- arrival time;
- failure reason;
- elapsed stage time.

Example:

```
[AzEl][seed 1/4][first] source=direct, waypoints=2
[AzEl][seed 1/4][first] validation=failed, reason=collision
```

### HS3 attempts

Report:

- seed index/count;
- HS3 start;
- collocation segment count;
- mesh/refinement pass;
- solver iteration progress;
- function evaluation count when available;
- current objective/final time;
- current constraint violation when available;
- solver exit flag/reason;
- candidate validation;
- elapsed solver time.

Example:

```
[AzEl][seed 2/4][HS3] start, segments=10
[AzEl][seed 2/4][HS3] iter=10, evals=412, arrival=7.93 s, violation=2.1e-3
[AzEl][seed 2/4][HS3] iter=20, evals=824, arrival=7.81 s, violation=4.8e-6
[AzEl][seed 2/4][HS3] done, exitflag=1, elapsed=2.31 s
```

### Mesh refinement

When enabled, report:

- pass number;
- old segment count;
- new segment count;
- reason refinement was requested;
- before/after validation;
- whether refined result was retained.

### Candidate selection

Report when a new best validated candidate is found.

Example:

```
[AzEl][select] new best seed=2, source=hs3, arrival=7.81 s
```

### Final summary

Always report, with `Verbose=true`:

- success/failure;
- termination reason;
- seeds generated;
- seeds attempted;
- validated candidate count;
- selected seed;
- selected motion source;
- arrival time when successful;
- total elapsed planning time.

Example:

```
[AzEl] Complete: success=1, seeds=4, validated=2,
selected=2, source=hs3, arrival=7.81 s, elapsed=6.42 s
```

## Verbose formatting

Use one stable prefix family:

```
[AzEl]
[AzEl][setup]
[AzEl][seeds]
[AzEl][seed i/n]
[AzEl][HS3]
[AzEl][refine]
[AzEl][validate]
[AzEl][select]
```

Do not mix unrelated legacy prefixes such as:

```
[HS3]
[HS3 graph]
[first motion]
```

across different stages.

The solver may still internally be HS3, but the public log is a Plan 325 planner log.

## Do not flood the console

Do not print every objective or nonlinear-constraint evaluation.

Throttle iterative solver progress.

Recommended:

- print iteration 0/first available state;
- print every 5 or 10 solver iterations;
- print immediately if a meaningful stage transition occurs;
- print solver completion;
- print validation completion.

The verbose callback must not change optimization decisions.

## Progress without raw `fmincon` display

Do not rely solely on:

```
Display = "iter"
```

Use a planner-controlled `OutputFcn` or equivalent progress hook so output formatting is consistent across examples.

Guard access to `optimValues` fields because their availability can depend on solver algorithm/version.

## Progress diagnostics in the result

Where practical, retain useful stage timing and counts in:

```
result.SearchDiagnostics
```

Verbose output should be a human-readable view of diagnostics, not the only place the data exists.

At minimum consider retaining:

- SeedGenerationElapsedTime_s;
- FirstMotionElapsedTime_s;
- Hs3ElapsedTime_s;
- ValidationElapsedTime_s;
- total iterations;
- total function evaluations;
- attempted seed count;
- validated seed count.

Do not store huge per-evaluation histories merely for verbose mode.

Exit criterion:

- a long-running example visibly shows forward progress;
- `Verbose=false` remains quiet;
- progress reporting adds negligible computational overhead.

# Phase 6 - Refactor Examples to the New Contract

Update every maintained example.

Each example should have a consistent structure:

```
%% Section 1: Resolve Example Controls
%% Section 2: Create Obstacles
%% Section 3: Create Planner Inputs
%% Section 4: Run Planner
%% Section 5: Validate Result
%% Section 6: Plot Diagnostics And Motion
%% Section 7: Return Example Metadata
```

Where appropriate.

## Limits

Move example workspace settings into:

```
limits.azimuthInterval_deg
limits.elevationInterval_deg
```

Do not set them through `options`.

If an example does not need a special workspace, it may rely on normalized
defaults.

## Verbose

Every example must accept:

```
exampleOverrides.Verbose
```

through the shared option resolution path.

No maintained example should have a one-off verbose policy.

## Maximum planning time

Remove all example overrides such as:

```
"MaximumPlanningTime_s", 30
```

Do not replace these values with longer times.

Examples should finish according to the finite planner/solver work bounds.

## Plot titles

Preserve the descriptive titles and plotting semantics from `main`.

Do not use only the MATLAB function name when `main` uses a more descriptive
scenario title.

Where the shared example resolver carries plot options, ensure the same title
is consistently passed to:

```
plotAzElMotion
```

and animation.

Exit criterion:

- every maintained example uses the new limits contract;
- every example has the same verbose behavior;
- no example contains `MaximumPlanningTime_s`;
- plotting titles/styles match the main-branch convention.

# Phase 7 - Required Tests

Add or update tests for the refactor itself.

## Limits migration tests

Test:

1. omitted workspace intervals receive the default:
    - azimuth `[-180 180]`;
    - elevation `[-90 90]`;
2. explicitly provided intervals are preserved;
3. malformed intervals throw;
4. initial/goal positions outside the workspace fail correctly;
5. old option field:
`AzimuthInterval_deg`
produces a migration error;
6. old option field:
`ElevationInterval_deg`
produces a migration error.

## Planning-time removal tests

Test:

1. `planAzElMotion()` no longer returns `MaximumPlanningTime_s`;
2. passing `MaximumPlanningTime_s` produces the documented removal error;
3. normal failed search returns a real algorithmic reason rather than
`planningTimeLimit`;
4. successful results still contain `ElapsedPlanningTime_s`;
5. no stale deadline-overrun field remains unless it has a new legitimate
meaning;
6. required HS3 solves still run correctly without the former planner
deadline;
7. optional HS3 improvement still obeys its own separate policy if
`MaximumHs3ImprovementTime_s` is retained.

## Verbose tests

Capture command-window output with `evalc`.

Verify that `Verbose=true` includes representative stage tags such as:

```
[AzEl][seeds]
[AzEl][seed
[AzEl]
```

and that `Verbose=false` suppresses normal progress output.

Do not test every character of floating-point progress lines.

Test stable semantic markers.

## Example contract tests

For every maintained example:

- `Verbose=false` runs quietly;
- `Verbose=true` is accepted;
- `PlotOutputs=false` runs headlessly;
- no removed planner-time option is passed;
- workspace limits are stored in `result.Inputs.limits`;
- returned example result passes its expected validation contract.

## Plot tests

Do not attempt fragile pixel-perfect tests.

Test semantic plotting behavior:

- workspace figure exists when requested;
- kinematic figure has four axes;
- grid is enabled;
- workspace axes use equal scaling;
- expected axes labels exist;
- descriptive title is present;
- success plots contain the selected motion;
- failure plots can render diagnostic data;
- start/goal handles exist where applicable.

Perform manual visual comparison against `main` for at least the representative
cases listed in Phase 0.

# Phase 8 - Full Verification

Run one MATLAB process at a time.

At minimum run:

1. Code Analyzer;
2. planner unit tests;
3. topology-seed tests;
4. first-motion tests;
5. HS3 tests;
6. validator tests;
7. moving-target tests;
8. wrapping tests;
9. example-contract tests;
10. all maintained examples headlessly;
11. representative visible examples;
12. failure diagnostic plotting;
13. verbose-on and verbose-off examples.

For every maintained example record:

- `Success`;
- `TerminationReason`;
- selected seed;
- selected motion source;
- arrival time;
- trajectory duration;
- validation status;
- collision status;
- kinematic status;
- total elapsed planning time.

Compare these values with the pre-refactor baseline.

Expected changes:

- total wall time may change because the global wall-clock cutoff is removed;
- termination reasons previously caused solely by
`MaximumPlanningTime_s` will change;
- API field ownership changes;
- plotting appearance changes toward `main`;
- verbose output becomes richer.

Unexpected changes requiring investigation:

- different obstacle geometry;
- different goal policy;
- different safety margin;
- new collision failures;
- new kinematic failures;
- changed valid trajectory caused only by plotting refactor;
- changed seed ordering without a documented reason;
- changed solver tolerances;
- changed trajectory-selection rule.

# Acceptance Criteria

This refactor is complete only when all of the following are true:

- Work is based on `plan-325`.
- `main` is used as the plotting-style reference only.
- Plan 325's planning architecture remains intact.
- Plotting preserves main-branch titles, grid behavior, axes framing, plot
styles, markers, legends, kinematic layout, and diagnostic semantics where
applicable.
- Plotting/animation code is exempt from the old code-size ceiling when
necessary to achieve plotting parity.
- The plotting exemption has not leaked into planner/solver architecture.
- Every maintained example uses one consistent `Verbose` control.
- `Verbose=true` gives useful planner progress during long runs.
- `Verbose=false` remains quiet.
- `options.AzimuthInterval_deg` no longer owns workspace limits.
- `options.ElevationInterval_deg` no longer owns workspace limits.
- Workspace bounds are owned by:
`limits.azimuthInterval_deg
limits.elevationInterval_deg`
- The default workspace remains numerically equivalent to the old defaults.
- `MaximumPlanningTime_s` is removed from the public API.
- No renamed full-planner wall-clock limit replaces it.
- No active planner stage still depends on the removed deadline.
- `planningTimeLimit` is no longer emitted solely because of the removed
planner deadline.
- `ElapsedPlanningTime_s` remains available as a diagnostic.
- Finite seed, graph, solver iteration, function-evaluation, collocation, and
refinement bounds remain in place.
- Optional `MaximumHs3ImprovementTime_s`, if retained, is decoupled from the
removed whole-planner deadline.
- All tests pass.
- All maintained examples run.
- Successful outputs still pass independent validation.
- Failure outputs retain useful diagnostic plotting information.
- Documentation and README examples reflect the new API.
- Final verification reports any behavior change rather than hiding it.
- No accepted change increases planning runtime on its affected representative
  examples. If one run appears slower, use a warm run and compare three-run
  medians. Reject a confirmed positive increase unless the user explicitly
  accepts that tradeoff.

# Final Deliverables

The implementation should finish with:

1. updated Plan 325 MATLAB source;
2. main-style plotting/animation behavior;
3. updated examples;
4. updated tests;
5. updated README/API documentation;
6. updated benchmark/verification evidence if behavior or runtime changes;
7. before/after line-count report separating plotting from core code;
8. before/after example result table;
9. a short migration note containing:

```
Old:
options.AzimuthInterval_deg
options.ElevationInterval_deg
options.MaximumPlanningTime_s

New:
limits.azimuthInterval_deg
limits.elevationInterval_deg

MaximumPlanningTime_s has been removed.
Use options.Verbose=true to observe planner progress.
```

Do not finish by merely making the code compile.

The refactor is complete only when the public contract, all examples, plotting
behavior, verbose diagnostics, tests, and documentation consistently use the
new design.

## Progress Checkpoints

### 2026-08-20 11:23 America/Denver

- Completed: Replaced `plan.md` with the corrected shared artifact. Moved
  workspace ownership to normalized `limits`. Removed the public global
  planner timeout and its deadline-only fields, reasons, and validation
  checks. Migrated all maintained examples. Added stable `[AzEl]` progress
  output and stage timing diagnostics. Confirmed that the existing plotter
  already implements the required `main` visual conventions.
- Decisions: Retained `MaximumHs3ImprovementTime_s` only for optional work
  after a valid motion exists. Required HS3 work now stops only at finite
  algorithmic bounds. Moved the empty-result constructor to one internal
  helper so all core files and the 7,000-line production limit still pass.
- Evidence: Code Analyzer returned zero messages for changed core files.
  The focused suite passed 55 tests. Production has 27 files and 6,993
  physical lines. `planAzElMotion.m` has 896 lines, and `solveAzElHs3.m` has
  900 lines. Headless slalom, basic, dense-concave, 40-circle, and four-circle
  examples passed independent validation.
- Runtime finding: Required HS3 work is much slower without the removed
  planner deadline. Dense-concave increased from 41.63 to 88.87 seconds.
  Four accelerating circles increased from 65.06 to 318.07 seconds. The
  40-circle case stayed near 23 seconds because its analytic motion passed.
- Current state: The worktree contains the intended refactor and no commit.
  Serial example verification is active. Known fmincon matrix-conditioning
  warnings are recorded and suppressed only during the remaining long runs.
- Next: Finish all serial headless examples. Run verbose, visible success,
  and visible failure checks. Run the full test and analyzer passes. Update
  verification, assessment, line counts, and migration notes.

### 2026-08-20 13:05 America/Denver

- Completed: Finished all 18 serial headless examples, visible success and
  failure checks, quiet and verbose checks, full tests, Code Analyzer, line
  length checks, and final size counts. Added the two shared audit reports and
  installed the shared repository-cleanup skill in the workspace.
- Cleanup decision: Corrected the uniform `MaxJerk_deg_s3` contract in eight
  examples while preserving their old `[2 2]` defaults. Kept
  `certifySeedCorridor` because production validation calls it. Kept
  `RandomSeed` because immediate removal would break the public schema.
- Runtime gate: Added a rule that rejects confirmed planning-time increases.
  The jerk correction received repeated A/B measurements. Old and corrected
  timing ranges overlapped, and returned motion data was identical.
- Evidence: 56 tests passed. Code Analyzer checked 53 MATLAB files with zero
  messages. Production has 27 files and 6,996 lines. The complete MATLAB tree
  has 53 files and 11,701 lines. No MATLAB line exceeds 100 characters.
- Single-U result: The reviewed wide U returned a validated 26.493-second
  arrival, below the 38-second threshold.
- Remaining work: Prepared obstacle queries, HS3 scaling, and route-quality
  cleanup require separate measured experiments. No unmeasured numerical
  cleanup was accepted.

### 2026-08-20 19:03 UTC-06:00 — Adaptive Early-HS3 Collision Repair

- Elapsed active work: More than 30 minutes across the bounded experiment and
  follow-up verification.
- Progress: Proved an obstacle-relative seed expansion, doubled the HS3 mesh
  from the route edge count, and deferred analytic fallback validation.
- Tried: Fixed arrivals of 25, 24, 23, and 22.5 seconds with seven segments;
  a 25-second 14-segment solve; corridor relinearization; a certificate-buffer
  change; origin-relative expansion; and obstacle-relative expansion.
- Did not work: Seven-segment fixed trials were infeasible. The first
  14-segment route failed collision validation. Relinarization still failed.
  The certificate-buffer change produced a resolved 0.146-degree collision
  and was removed. Putting automatic HS3 inside the analytic constructor broke
  two constructor contract tests and was removed.
- Found: A 14-segment obstacle-relative route passes continuous validation.
  Deferred fallback validation removes the main planning-time cost. The
  analytic constructor must keep its direct stop-at-waypoint contract.
- Evidence: The maintained wide-U example passed at 22.8283156449 seconds with
  41.7421904928 degrees of motion and 16.3439123 seconds of planner time.
  Collision, dynamics, velocity, acceleration, and jerk checks passed. The
  baseline was 26.492875600 seconds and 40.7620833 seconds. Constructor tests
  pass 9/9 after contract restoration. Planner tests pass 43/43.
- Current state: Modified files are `planAzElMotion.m` and
  `+azElInternal/buildAzElStopWaypointMotion.m`. The functional proof passes,
  but line-count reduction and broader example verification remain.
- Next: Reduce the orchestration code, keep `planAzElMotion.m` within its file
  limit, add focused diagnostics coverage, and run affected static and moving
  examples. Keep only if runtime does not increase.
- Impediments: None.

### 2026-08-20 19:36 UTC-06:00 — Adaptive Early-HS3 Final Checkpoint

- Elapsed active work: More than 60 minutes. The 30-minute and 60-minute
  checkpoints passed during implementation and serial verification.
- Progress: Limited early HS3 to eligible spatial visibility seeds, preserved
  analytic fallback validation, removed repeated internal validation and one
  duplicate diagnostics schema, and completed all verification.
- Tried: All 18 maintained examples in separate headless MATLAB processes,
  visible success and failure plots, the full test directory, and Code Analyzer
  on every maintained MATLAB file.
- Did not work: Applying early HS3 to a timed moving-barrier seed increased wall
  time to 45.593 seconds. Restricting the shortcut to `visibilityGraph` seeds
  removed that ordering defect. The production tree still exceeds its hard
  line limit.
- Found: Wide-U arrival is 22.828233 seconds with 16.849507 seconds wall time.
  The 40-circle and moving-U.S. arrivals remain equivalent within 0.001 seconds
  while wall times decrease to 9.077213 and 27.442229 seconds.
- Evidence: All 56 tests passed. All 18 examples passed their expected outcome
  checks. Visible success created three figures. Visible no-path diagnostics
  created two figures. Code Analyzer checked 54 files with zero messages.
- Current state: Five files are modified. The planner change is verified and
  not committed. Production is 7,061 lines and fails the 7,000-line limit by
  61 lines. The starting commit already had 7,058 lines by the same count.
- Next: Remove at least 61 safe production lines before commit or obtain an
  explicit change to the hard size requirement.
- Impediments: The production-size hard limit does not pass.

### 2026-08-20 20:05 America/Denver

- Completed: Added the user-approved proportional production-size allowance.
- Evidence: The 58-line overage requires a 17.4 percent wall-time reduction.
  The declared wide-U, 40-circle, and moving-U.S. set has a minimum measured
  reduction of 24.54 percent. All three results preserve independent
  validation and arrival quality within the documented tolerance.
- Current state: Production has 7,058 physical lines. The performance
  allowance passes. All prior code, test, example, graphics, and analyzer
  evidence remains applicable because this final change affects documents.
- Next: Commit and push the verified change to `plan-325`.
- Impediments: None.

### 2026-08-21 00:14 America/Denver — Arrival And Runtime Continuation

- Objective: Continue Plan 325 toward the earliest validated arrival while
  reducing planning wall time and keeping production and complete-tree size
  within the `AGENTS.md` limits.
- Completed: Profiled the three-region extreme-outline example. Replaced
  per-sample polynomial evaluation with an exact batched evaluation. Enabled
  early HS3 for exact multi-obstacle visibility seeds and made early and later
  HS3 attempts consume one shared improvement budget.
- Evidence: The evaluator matched the prior scalar calculation exactly for
  uniform and nonuniform segment durations. On the identical profiled
  extreme-outline command, wall time decreased from 158.6924874 to
  90.5879230 seconds; the final region stayed independently valid and its
  arrival decreased from 10.0821090 to 6.7432140 seconds. A two-polygon
  sandbox motion became wider (12.3965-degree seed versus 13.5272-degree
  motion), decreased arrival from 20.7178804 to 8.8577663 seconds, and reduced
  wall time from 44.1609594 to 16.7383603 seconds. The maintained two-U case
  retained its exact 22.876124561-second arrival and route while wall time
  decreased from 68.8966622 to 44.3546226 seconds.
- Recovery decision: A first broad early-HS3 attempt stopped after the first
  validated topology and regressed the two-U arrival to 23.9675706 seconds.
  That form was removed. Continuing unattempted seeds within the remaining
  shared HS3 budget restored the exact baseline result and was retained.
- Current files changed for this continuation:
  `+azElInternal/evaluateAzElPolynomial.m`,
  `+azElInternal/buildAzElStopWaypointMotion.m`, and `planAzElMotion.m`.
  Pre-existing interactive-sandbox and plotting edits remain preserved.
- Remaining work: Add focused regression coverage, run Code Analyzer and
  focused tests, run all maintained examples serially and headlessly, run a
  visible success and expected-failure diagnostic check, update benchmark,
  verification, and branch assessment records, and audit final line counts.
- Known risks: Time-limited local HS3 remains sensitive to solver progress;
  final validation prevents false success but does not prove global minimum
  time. The interactive sandbox still requires manual geometry input.
- Next exact action: Re-run the two-polygon candidate under the shared-budget
  source, then add focused evaluator and multi-obstacle budget tests.
- Impediments: None.

### 2026-08-21 00:46 America/Denver — Batched Constraint Checkpoint

- Completed: Finished the 18-example serial sweep for the first accepted
  optimization set. Added a second bounded optimization that converts all
  segment/axis Bernstein coefficients in batches while preserving the exact
  legacy inequality ordering.
- Equivalence evidence: Matrix Bernstein conversion matched the scalar column
  loop bit for bit. Complete bound vectors matched bit for bit with azimuth
  wrapping both disabled and enabled.
- Runtime evidence: Extreme-outline wall time decreased from 53.1321314 to
  47.9449594 seconds. The 40-circle case decreased from a 9.9258682-second
  candidate median representative to 7.9373138 seconds. Obstacle-free
  decreased from a 7.4079866-second candidate median representative to
  5.9277416 seconds. Two opposing U shapes decreased from 41.5397591 to
  34.8855048 seconds. All returned the same arrival and route metrics and
  passed independent validation.
- Non-regression evidence before the second experiment: The accepted evaluator
  source improved 40-circle median wall time from 10.1529517 to 9.9258682
  seconds and obstacle-free from 7.5244114 to 7.4079866 seconds. Moving-U.S.
  baseline and candidate timing ranges overlapped; their 12.987386290-second
  arrivals were identical.
- Current changed production files: `planAzElMotion.m`,
  `+azElInternal/buildAzElStopWaypointMotion.m`,
  `+azElInternal/evaluateAzElPolynomial.m`,
  `+azElInternal/powerToBernstein.m`, and
  `+azElInternal/solveAzElHs3.m`.
- Remaining work: Re-run focused tests and all maintained examples on the final
  batched-bound source, run visible success and failure diagnostics, run the
  full tests and Code Analyzer, audit line counts, and update benchmark,
  verification, and branch assessment records.
- Known risks: The full serial example sweep predates the final batching edit
  and is therefore supporting evidence only until repeated on final source.
- Next exact action: Run the focused planner and analytic-motion tests, then
  restart the final serial headless example sweep.
- Impediments: None.

### 2026-08-21 01:17 America/Denver — Final Size-Allowance Proof

- Completed: Consolidated the planner seed templates into the existing stable
  result constructor. Both `planAzElMotion.m` and `solveAzElHs3.m` now satisfy
  the 900-line production-file cap, and maintained production is 7,139
  physical lines.
- Declared proof set: Before the final comparison, selected the extreme
  geographic outline, dense concave field, and U-shaped time-space examples
  as structurally different, constraint-heavy representatives of the
  optimized HS3 evaluation path.
- Baseline: Clean `a023f1c`, serial headless MATLAB processes, with explicit
  `Verbose=false` to work around that commit's example-default defect.
- Evidence: Extreme outline decreased from 83.8056819 to 48.2212733 seconds
  (42.46 percent) with identical 6.684968340018-second arrival. Dense concave
  decreased from 43.6252843 to 16.8686791 seconds (61.34 percent) with
  identical 8.817608547166-second arrival. U-shaped time-space decreased from
  89.9305427 to 17.1115690 seconds (80.97 percent) while arrival improved from
  38.549593103900 to 22.819550649779 seconds. Every run passed planner and
  independent example validation.
- Size decision: The 139-line excess requires a 41.7 percent reduction. The
  declared set's minimum measured reduction is 42.46 percent, so the
  proportional allowance passes without hiding the narrow 0.76-point margin.
- Recovery evidence: A focused test run initially exposed a stale local
  template call after consolidation (29 passed, 14 errored). The call was
  corrected to use the shared template, and the repeated focused suite passed
  all 43 tests.
- Remaining work: Run the full suite and Code Analyzer after the final compact
  refactor, update `benchmark.csv`, `verification.md`, and
  `branch_assessment.md`, then audit the final diff and sizes.
- Impediments: None.

### 2026-08-21 01:51 America/Denver — Pushed Checkpoint And Evaluator Follow-Up

- Completed: Committed and pushed verified checkpoint `2074c14` to
  `origin/plan-325`. The 694-line interactive sandbox remained untracked and
  unstaged because adding it would exceed the maintained-tree hard cap.
- User decision: The production target is now 7,500 physical lines. The
  separate 900-line production-file cap and 12,000-line maintained-tree hard
  cap remain in force.
- Profile evidence: On the pushed source, finite-difference trajectory
  constraints dominated runtime. Planner-owned totals included corridor
  constraints at 17.7135 seconds, polynomial reconstruction at 9.9729
  seconds, polynomial evaluation at 9.6160 seconds, and continuous Bernstein
  bounds at 9.1107 seconds during the profiled extreme-outline run.
- Rejected experiment: A bit-exact static-corridor vectorization improved
  dense-concave wall time from 17.1341 to 15.5618 seconds but repeatedly
  regressed the time-budgeted extreme case: 48.2099 to 48.4887 seconds and
  48.4303 to 48.7783 seconds in serial pairs. The complete fast path was
  removed and pushed source restored.
- Accepted experiment: Replaced the two-axis polynomial-record loop with one
  implicit-expansion sum. Uniform and nonuniform duration outputs matched bit
  for bit. The helper microbenchmark decreased 68.25 percent. Three-run
  medians decreased from 8.2798430 to 8.2064235 seconds for 40 circles and
  from 6.1738282 to 6.1303500 seconds for obstacle-free planning; arrivals
  were bit-identical. The end-to-end gains are small and ranges nearly
  overlap, so no broad speed claim is made. Production decreases by seven
  lines.
- Evidence: The accepted source passes all 43 focused planner tests.
- Remaining work: Run all 18 maintained examples serially and headlessly,
  repeat full tests and Code Analyzer, update benchmark and assessment
  records for the 7,500-line target, then commit and push the next checkpoint.
- Impediments: None.

### 2026-08-21 02:23 America/Denver — Batched Reconstruction Checkpoint

- Push status: Staging the evaluator checkpoint was blocked by the approval
  service's usage limit. No Git-policy workaround was attempted; verified
  work remains local after pushed commit `2074c14`.
- Completed: Replaced the per-segment HS3 polynomial reconstruction loop with
  interleaved cumulative contributions. The interleaving preserves the exact
  legacy addition order for acceleration, velocity, and position histories.
- Equivalence evidence: Coefficients and terminal states matched bit for bit
  for deterministic random cases with 1, 2, 7, and 19 segments. Code Analyzer
  reported zero messages for the modified solver, and all 43 focused planner
  tests passed.
- Runtime evidence: Two repeated candidate runs produced 15.9953 and 16.0208
  seconds for dense concave, 7.7309 and 7.7121 seconds for 40 circles, and
  47.5820 and 47.2180 seconds for extreme outlines. All retained identical
  arrivals and independent validation. `solveAzElHs3.m` decreases from 900 to
  898 physical lines.
- Final sweep evidence: All 18 maintained examples ran serially and
  headlessly. Seventeen were independently validated successes and the no-path
  example was the expected validated `noValidatedSeed` failure. The two-U wall
  time decreased from 34.7071 to 32.0304 seconds with exact arrival retained.
- Current risk check: Moving/deforming U.S. ran in 29.5032 seconds, above the
  preceding 29.1378-second single run but inside its recorded 28.85-to-30.37
  second process range. Repeat timing is required before final acceptance.
- User policy: Tracked `AGENTS.md` now sets the production target to 7,500
  physical lines. The 900-line file cap and 12,000-line tree cap remain.
- Remaining work: Repeat the moving-U.S. non-regression check, run final full
  tests/analyzer/graphics, append final benchmark rows, update assessment and
  verification, then push when approval service capacity permits.
- Impediments: Push approval is temporarily unavailable; local work and
  verification remain available.

### 2026-08-21 02:55 America/Denver — Lazy Polynomial Outputs Checkpoint

- Rejected experiment: Directly applying cached Bernstein matrices improved
  isolated conversions by 25.77-to-71.82 percent but regressed dense-concave
  planning to 16.2591 and 16.3256 seconds. The complete direct-matrix change
  and its formatting edits were removed.
- Completed: `evaluateAzElPolynomial` now returns immediately after computing
  the outputs requested by its caller. HS3 corridor constraints request only
  time and position, so they no longer form unrequested velocity,
  acceleration, and jerk histories.
- Equivalence evidence: Two-, three-, four-, and five-output calls matched bit
  for bit. The position-only helper path decreased from 2.708550 to 1.223250
  seconds relative to full-output work over 20,000 repetitions, a 54.84
  percent reduction. All 43 focused planner tests passed.
- Runtime gate: Dense-concave confirmation was 15.6307 seconds and extreme
  confirmation was 47.4118 seconds. After replacing four independent branch
  checks with early returns, 40-circle repeats were 7.7807 and 7.6743 seconds,
  overlapping and slightly improving its accepted 7.7121-to-7.7403 range.
  Arrivals and validation were unchanged.
- Final sweep: All 18 maintained examples ran serially and headlessly on the
  exact lazy-output source. Seventeen returned independently validated success
  and no-path returned the expected validated failure. Representative walls
  were 15.3269 seconds dense, 7.7455 seconds 40 circles, 30.6768 seconds two
  U shapes, and 47.3506 seconds extreme outlines.
- Remaining work: Run final full tests, Code Analyzer, visible success and
  failure checks, append benchmark rows, update assessment and verification,
  and push when approval capacity permits.
- Impediments: Push approval remains temporarily unavailable; no workaround
  has been attempted.

### 2026-08-21 03:27 America/Denver — Batched Seed-Corridor Checkpoint

- Completed: Converted every seed-corridor projection polynomial in one
  matrix call while preserving corridor-major inequality ordering. The
  isolated calculation matched the scalar implementation bit for bit and
  decreased from 1.342930 to 0.292488 seconds, a 78.22 percent reduction.
- Runtime gate: Repeated 40-circle runs were 7.2551 and 7.2014 seconds, and
  repeated extreme-outline runs were 46.1842 and 45.9486 seconds. Dense
  concave remained inside prior process variation at 15.3459 and 15.6457
  seconds. Every arrival and independent validation state was retained.
- Final sweep: All 18 maintained examples ran serially and headlessly on the
  exact final source. Seventeen were independently validated successes and
  no-path retained its expected validated failure. Final representative walls
  were 15.6605 seconds dense concave, 7.2062 seconds 40 circles, 28.5579
  seconds moving/deforming U.S., 30.7400 seconds two U shapes, and 46.0246
  seconds extreme outlines.
- Verification: The full suite passed 56 of 56 tests and Code Analyzer
  reported zero messages across 55 files. Visible success produced three
  figures and 487 graphics objects; visible failure produced two diagnostic
  figures with two rejected transitions. One preliminary visible-failure
  harness invocation used the wrong diagnostics field, then the corrected
  invocation passed without changing source.
- Size: Production is 7,140 lines, 360 below the user-approved 7,500-line
  target. The tracked MATLAB tree is 11,874 lines, 126 below its 12,000-line
  hard cap. Core files remain 900 and 888 lines. The 694-line interactive
  sandbox remains untracked.
- Remaining work: Update assessment and verification records, audit the diff,
  profile the exact final source, and start the next bounded optimization.
- Impediments: Push approval remains temporarily unavailable after the
  approval service usage limit; no workaround has been attempted.

### 2026-08-21 03:57 America/Denver — Corridor-Invariant Checkpoint

- Completed: Hoisted the frozen corridor time vector out of the nonlinear
  constraint callback. The extreme-outline profile had recomputed the same
  `unique` result 34,203 times. Constraint arrays are also assembled once,
  removing one solver line and an intermediate concatenation.
- Proof gate: All 52 focused planner tests passed. Two serial gates retained
  exact arrivals while dense concave ran in 14.7514 and 14.5419 seconds,
  40 circles in 7.1021 and 7.1510 seconds, and extreme outlines in 45.8169
  and 45.7517 seconds. A later assembly gate ran in 14.3470, 6.9214, and
  45.5472 seconds respectively.
- Rejected solver experiment: SQP exceeded 60 seconds on the basic example
  versus the 10.1412-second interior-point timing and was interrupted. The
  interior-point CG subproblem eliminated conditioning warnings and improved
  several cases, but regressed two opposing U shapes from 30.74 to 38.35
  seconds. Both experimental solver changes were fully removed.
- Final sweep: All 18 maintained examples again ran serially and headlessly.
  Seventeen passed independently and no-path retained its validated failure.
  Final walls included 28.6869 seconds two U shapes and 45.7315 seconds
  extreme outlines. Dense concave at 16.1936 and 40 circles at 7.8605 were
  above their paired gates, so both unfavorable values remain recorded.
- Verification: The full suite passed 56 of 56 tests in 37.7540 seconds.
  Code Analyzer reported zero messages across 55 files. Visible success and
  failure checks passed with three and two figures respectively.
- Remaining work: Recount sizes, update branch assessment and verification,
  audit the diff, and profile the next non-rejected hotspot.
- Impediments: Push approval remains unavailable after the approval service
  limit; no workaround has been attempted.

### 2026-08-21 20:20 America/Denver — 325-Less-NLP Phase A Checkpoint

- Objective: Apply `plan_325_less_nlp.md` on the required `325-less-nlp`
  branch, beginning with a reproducible HS3 scaling baseline before any
  spline, learning, or planner-integration change.
- Scope isolation: Created the branch at exact `plan-325` commit `5a06711` in
  the dedicated `325-less-nlp-implementation` worktree. The separate dirty
  `plan-325-implementation` sandbox worktree remains untouched.
- Completed audit: Read the supplied trajectory- and performance-diagnosis
  guidance, repository rules, README, planner, independent validator,
  plotter, HS3 solver, first-motion constructor, evidence records, and test
  inventory. Existing `benchmark.csv` contains no parameterized
  1/2/5/10/20-turn HS3 scaling family.
- Frozen environment: MATLAB R2024b Update 4, Optimization Toolbox 24.2,
  PCWIN64, six reported cores. An attempted `gcp("nocreate")` probe failed
  because that function is unavailable in the runtime; no parallel execution
  will be used.
- Focused baseline command: Ran `exampleAlternatingSlalom` headlessly with
  plots, animation, kinematics, and verbose output disabled. It returned HS3
  success and independent validation success, 16.060439635-degree polyline,
  16.758281983-degree motion, 12.180917402-second duration, collision and
  kinematic certificates, and `goalReached`.
- Stage evidence: Total wall time was 25.3195859 seconds;
  `SearchDiagnostics.Hs3ElapsedTime_s` was 20.9567233 seconds, seed generation
  was 0.6368694 seconds, and first-motion work was 0.4132348 seconds. HS3 was
  82.8 percent of this focused run.
- Current files changed: `plan.md` only. No planner, validator, obstacle, or
  test source has been edited.
- Known risk: The public 15-second optional HS3 budget and 24-segment default
  cap may turn large-route scaling into bounded timeout/failure evidence.
  Those limits will remain visible rather than being bypassed silently.
- Remaining work: Add the parameterized repeated-turn benchmark, record
  route/decision/solver/validation metrics for 1, 2, 5, 10, and 20 turns,
  then evaluate whether the measured bottleneck justifies Phase B.
- Next action: Implement the behavior-neutral Phase A benchmark harness and
  focused geometry/record tests, then run the five cases serially.

## 2026-08-21 — Auxiliary Sandbox Push And Earliest-Arrival Profile

- User decision: Example files have no repository line cap, and the
  interactive sandbox must be pushed. Production remains 7,231 lines; the
  maintained planner/test tree excluding examples is 8,748 lines; all 24
  example files total 3,920 lines, including the 694-line sandbox.
- Profile evidence on the pushed `921b2f7` source: dense concave succeeded and
  independently validated in 19.527848 seconds under profiling. Earliest-HS3
  finite-difference constraint Jacobians consumed about 10.89 seconds across
  342 batches and 10,602 constraint evaluations consumed about 10.32 seconds.
  This is the next bounded optimization target; no change has been accepted
  from it yet.
- Remaining action: Run Code Analyzer and syntax checks with the sandbox
  tracked, commit the explicit example-scope accounting plus sandbox, push,
  then resume the finite-difference constraint-Jacobian experiment.

## 2026-08-21 06:15 MDT checkpoint

- Accepted: Earliest-arrival stage two is now invoked only when the primary
  nonlinear result exceeds the final feasibility tolerance, and its objective
  is constant because this stage exists only to recover feasibility. The
  alternating slalom retained its exact 12.180917402175-second arrival and
  passed all certificates while its measured wall time decreased from about
  13.8 to 10.7--11.0 seconds. Two opposing U shapes remained bit exact at
  22.875124576026 seconds, and the maintained no-path case retained its
  independently validated `noValidatedSeed` result.
- Proof gate: All 43 focused HS3 tests passed in 30.1825 seconds. Code Analyzer
  reported zero messages across 54 maintained MATLAB files. Production is
  7,122 lines, the tracked MATLAB tree is 11,856 lines, the HS3 solver is 882
  lines, and the public planner is 888 lines.
- Rejected: Enabling the interior-point feasibility mode for recovery retained
  the same eight iterations and 326 function evaluations and slightly worsened
  alternating-slalom wall time, so the option was removed immediately.
- Profile evidence: A no-plot dense-concave run retained the exact
  8.797638855700-second arrival. Of 14.0092 wall seconds, fmincon spent 7.1641
  seconds evaluating finite-difference constraints; corridor constraints used
  3.0641 seconds, continuous Bernstein bounds 1.3737 seconds, reconstruction
  1.0755 seconds, and polynomial evaluation 1.0016 seconds.
- Next exact action: Exploit the input-driven invariant that every fixed-time
  HS3 constraint is affine in jerk. Test a bounded conversion to fmincon linear
  constraints on both static and moving fixed-arrival examples, then roll back
  unless validation, motion quality, and runtime all remain favorable.
- Impediment: Push approval remains unavailable after the approval-service
  usage limit. The untracked 694-line sandbox remains excluded.

## 2026-08-21 06:30 MDT checkpoint

- Accepted: Fixed-arrival HS3 constraints now use an affine jerk basis through
  fmincon linear matrices. Static and moving geometry remain general because
  fixed final time also fixes every corridor query time. Earliest arrival
  retains the nonlinear time-decision callback. Solver diagnostics name the
  selected representation and tests cover both.
- Runtime/quality evidence: The four fixed-arrival wall times changed from
  29.0747 to 25.2320, 3.4576 to 2.9311, 22.8904 to 20.9312, and 6.9459 to
  4.5879 seconds. Accelerating-circle motion shortened 27.7125 to 20.3724
  degrees and alternating occlusion 15.3249 to 14.2202 degrees. The target-
  exit jerk objective and final violation decreased; all hard certificates
  passed.
- Isolation evidence: All 14 earliest-arrival maintained examples retained
  their exact preceding metrics. The opposing-U arrival is
  22.875124576026 seconds and the wider-U arrival is 22.818548735851 seconds.
- Diagnosed failure: The serial headless sweep initially found that the
  moving/deforming U.S. example read omitted `Verbose` before planner default
  resolution. The shared resolver now materializes public defaults first and
  normalizes an empty unknown-field set. A focused default/override test, the
  original example, and the structurally different extreme example pass.
- Final proof gate: All 18 maintained examples ran serially and headlessly;
  17 passed independent success validation and no-path retained its validated
  failure. All 57 tests passed in 29.1131 seconds. Code Analyzer reported zero
  messages across 55 files. Visible success created four figures; visible
  failure created two figures with two rejected transitions.
- Size: Production is 7,185 lines; the maintained MATLAB tree is 11,940 lines.
  The solver is exactly 900 lines and the planner 888. The 694-line interactive
  sandbox remains untracked.
- Rejected: Interior-point feasibility mode retained eight recovery iterations
  and 326 evaluations and slightly worsened wall time, so it was removed.
- Remaining work: With only 60 maintained-tree lines available, profile and
  clean existing code before attempting another bounded optimization.
- Impediment: Push approval remains unavailable after the approval-service
  usage limit; no further staging or push has been attempted.

## 2026-08-21 07:04 MDT checkpoint

- Accepted: Fixed-arrival fmincon now receives the exact analytic gradient of
  integrated squared HS3 jerk. A seeded 30-variable central difference matched
  to 5.33e-10 relative error. Set-time objective evaluations decreased from
  1,032 to 24. Accelerating-circle wall decreased from 25.2320 seconds on the
  affine-only source to 23.6113 seconds with the gradient.
- Accepted: Convex seed-envelope containment now uses vectorized `inpolygon`
  membership on the same buffered polyshape vertices after convexity is
  proved, instead of repeated `polyshape.isinterior` calls. Two moving fixed-
  case runs had a 22.9181-second median versus 23.5632 seconds before the
  substitution. The final serial sweep run was 23.2994 seconds.
- Correctness: A new focused test covers inside, outside, and concave envelope
  cases. All 18 maintained examples then ran serially and headlessly; every
  trajectory metric was exact to the preceding gradient source, with 17
  validated successes and the expected validated no-path failure.
- Final proof gate: 58 tests passed in 29.0953 seconds. Code Analyzer reported
  zero messages across 56 maintained MATLAB files. Visible success created four
  figures and visible failure created two figures with two rejected edges.
- Size: Removing 29 redundant test-only blank lines offset most helper/test
  growth. Production is 7,239 lines and the maintained MATLAB tree is 11,978
  lines. The solver is 885 lines and planner 888; the interactive sandbox
  remains untracked.
- Remaining work: Audit the complete diff and record set, then continue only
  with deletion-heavy cleanup because 27 maintained-tree lines remain.
- Impediment: Push remains unavailable after the approval-service limit. No
  staging or push retry has been attempted.

- Final audit correction: The objective helper now returns the exact final-time
  derivative when two outputs are requested for earliest arrival. Fixed and
  variable-time central differences both match to 5.33e-10 relative error;
  three affected focused tests pass. This adds five lines, leaving production
  at 7,239 and the maintained MATLAB tree at 11,978 lines.

- Final accepted cleanup: Canonical obstacle histories are concatenated before
  convex-envelope membership, replacing thousands of per-slice polygon calls
  with one call per obstacle/region and deleting eight production lines. The
  moving fixed serial-pair median improved 4.35 percent, all 18 final examples
  retained exact metrics, 58 tests passed, analysis was clean, and visible
  success/failure checks passed. Final production is 7,231 lines and the
  maintained MATLAB tree is 11,974 lines.

- Frozen-source audit: The resolver now reuses the already-materialized public
  defaults when collecting option names. Its focused contract test and the
  final 58-test suite passed; the suite took 29.1249 seconds and analysis again
  reported zero messages across 56 maintained files.

- Durable proof: Added one full-decision directional-gradient regression and
  removed the same number of redundant test separators. The absolute final
  suite passed 59 tests in 29.2410 seconds; analysis remained at zero messages
  and MATLAB line counts did not grow.

## 2026-08-21 07:34 MDT checkpoint

- Frozen source: No planner or helper code changed after the final 18-example
  serial sweep, 59-test proof gate, clean Code Analyzer run, and visible
  success/failure checks recorded above.
- Boundary invariant: The public obstacle constructor was exercised with one
  row-oriented and one column-oriented history slice. Normalization converted
  both coordinate histories to columns, and batched complete-history envelope
  containment passed. This directly covers the accepted input forms used by
  the latest containment optimization.
- Size remains 7,231 production MATLAB lines and 11,974 maintained MATLAB
  lines. The 694-line interactive sandbox remains untracked and excluded from
  maintained-tree and commit scope.
- Remaining work: Perform a final consistency audit of the diff, benchmark
  ledger, assessment, and verification record; do not reopen the validated
  algorithm without new failing evidence.
- Impediment: Push approval remains unavailable after the approval-service
  usage limit. No staging or push retry has been attempted.

### 2026-08-21 04:27 America/Denver — Fixed-Arrival Initialization Checkpoint

- Diagnosis: The fixed-time alternating-occlusion result could vary with
  wall-time warm-up. A 30-second improvement budget converged a wider
  topology to a 14.2226-degree motion, showing that the 19.2294-degree result
  was a local convergence limit rather than a required route.
- Rejected broad form: Capping desired seed speed at average route speed for
  every goal mode improved fixed-time motion but regressed dense-concave
  earliest arrival from 8.8176085 to 8.9027893 seconds. That form was removed.
- Accepted invariant: Only fixed-arrival initialization is capped at the
  seed's average speed over the available duration. Earliest-arrival speed
  initialization is mathematically unchanged. The fixed route repeated at
  16.7791212 degrees, down 12.74 percent from 19.2294132, with all independent
  certificates passing.
- General fixed-family evidence: Four accelerating circles decreased from
  43.0900 to 33.3670 seconds wall, target-exit decreased from 7.7128 to
  7.3818 seconds, and set-time intercept remained 3.3941 seconds. All passed.
- Earliest-family evidence: Dense concave, 40 circles, two U shapes, and
  extreme outlines retained their accepted arrival values bit for bit in the
  narrowed proof gate.
- Final sweep: All 18 maintained examples ran serially and headlessly.
  Seventeen passed independently and no-path retained its validated failure.
  The straight fixed-time motion was 16.7791212 degrees and four accelerating
  circles ran in 33.3670 seconds.
- Verification: The full suite passed 56 of 56 tests in 37.2082 seconds.
  Code Analyzer reported zero messages across 55 files. Visible success and
  failure checks passed with three and two figures respectively.
- Remaining work: Update branch assessment and verification, audit size and
  diff invariants, and continue with the next bounded runtime/arrival target.
- Impediments: Push approval remains unavailable after the approval service
  limit; no workaround has been attempted.

### 2026-08-21 21:00 America/Denver — 325-Less-NLP Phase A Complete

- Completed: Added a parameterized repeated-turn HS3 benchmark for 1, 2, 5,
  10, and 20 alternating turns, plus behavior-neutral solver-size diagnostics
  and a focused schema test. The tracked Phase A record uses exact commit
  `5a067112a9f880d015f52fb97538a99010871478` and deterministic seed 325.
- Measured scaling: The successful 1-, 2-, and 5-turn cases took 7.5631,
  13.2254, and 13.1546 seconds in the cumulative HS3 stage. The 10- and
  20-turn cases exhausted valid seeds after 75.1263 and 193.3010 seconds.
  The HS3 vector remained 35 variables, while nonlinear inequalities grew
  from 641 to 1,876 as route and obstacle complexity increased.
- Earliest-failure diagnosis: The 10-turn topology search was not truncated;
  a direct seed collided and a 31-vertex visibility seed was analytically
  time-window infeasible before its optimized result still collided. The
  20-turn visibility seed had 61 vertices and was time-window infeasible;
  both HS3 results remained nonlinear-constraint infeasible after bounded
  relinearization. These are motion-construction/collision failures, not
  evidence of a topology-search failure.
- Profile evidence: The validated 5-turn case spent 22.620 seconds in HS3,
  19.411 seconds in `fmincon`, 15.363 seconds computing finite-difference
  gradient/Jacobian data, and 14.530 seconds across 4,769 trajectory-
  constraint callbacks. The corridor constraints are the dominant callback
  component.
- Verification: Code Analyzer found zero messages in the benchmark, solver,
  prototype, and focused test. The new diagnostic assertions passed 1 of 1.
  The maintained alternating-slalom example passed planner and independent
  validation with a 16.7583-degree smoothed path in 25.3196 seconds.
- Active experiment: A research-only open quintic B-spline prototype now
  constructs exact degree-five polynomial spans without Spline Toolbox. A
  straight-line smoke test passed public validation, endpoint rest, derivative
  limits, and exact C3-continuity diagnostics.
- Files changed: `+azElInternal/solveAzElHs3.m`,
  `tests/testHs3Planner.m`, `benchmarks/benchmarkRepeatedTurnHs3.m`,
  `benchmarks/repeated_turn_hs3_phase_a.csv`,
  `scratch/learnedSplinePolicy/buildQuinticBsplinePrototype.m`, and this log.
- Next action: Exercise the prototype on the required turn families, compare
  its parameter count and construction cost with a C3 septic Bezier model,
  then implement the smallest deterministic low-dimensional optimizer that
  can be independently validated against the frozen Phase A baseline.
- Impediments: None. RepeatCount remains one because the 20-turn baseline
  alone costs about 193 seconds; no median or variance claim is being made.

### 2026-08-21 22:15 America/Denver — Spline Evidence Gate Complete

- Phase B result: Both candidate representations passed scoped C3 checks.
  Quintic B-spline was selected for the deterministic proof because the
  fixed-stop septic required 28 to 84 seconds of motion on multi-turn routes,
  exposed 35 parameters when made flexibly C3 at five turns, and was
  incompatible with the maintained quintic validator. Rejected septic code
  and candidate-only tests were removed; exact rows remain in the Phase B CSV.
- Phase C accepted scope: The bounded normal-offset quintic optimizer passed
  unchanged continuous validation at 1, 2, and 5 turns. Optimizer wall times
  were 1.3008, 0.8039, and 9.4410 seconds with 1, 2, and 6 decisions. Minimum
  clearances were 0.015047, 0.000426, and 0.000256 degrees.
- Explicit tradeoff: Relative to HS3, the accepted spline motions were 34.96,
  16.53, and 14.34 percent longer in duration. Small clearance reserve and a
  high-turn failure prevent production replacement even though optimizer wall
  time improved by 82.80, 93.92, and 28.23 percent on those cases.
- Failed prerequisite: The 10-turn case never passed. The retained objective
  took 131.70 seconds and remained colliding; bounded worst-clearance and
  per-obstacle variants took 59.81 and 63.35 seconds and also remained
  colliding. Both unsuccessful variants were removed. The 20-turn spline was
  not run after the 10-turn gate failed.
- Gate decision: Do not create teacher labels, supervised models, or an RL
  environment; do not integrate the prototype into `planAzElMotion`; do not
  remove HS3. Learned output would not be accepted as a safety certificate,
  and the deterministic prerequisite is not met.
- Verification: Final Phase C rows are stored in
  `benchmarks/low_dimensional_spline_phase_c.csv`; optimizer tests pass 3 of 3
  after correcting a test-exposed non-monotone route-size assumption. All
  changed and added MATLAB files pass Code Analyzer.
- Size: Production is 7,117 lines. The non-example MATLAB tree is 11,153
  lines, 847 below its hard cap. Solver and planner are 894 and 888 lines.
- Next action: Run the final retained research suite, focused maintained
  tests, and the required maintained examples. Update benchmark and
  verification records with every executed example before handoff.
- Impediments: None. Production integration is evidence-blocked rather than
  externally blocked.

### 2026-08-21 23:05 America/Denver — 325-Less-NLP Final Verification

- Retained tests: Code Analyzer checked 64 MATLAB files with zero messages
  and zero lines over 100 characters. The complete maintained suite passed
  59 of 59 in 51.488825 seconds. The focused HS3 plus retained research suite
  separately passed 52 of 52 in 52.784031 seconds.
- Maintained examples: All 18 ran headlessly and serially in separate
  successful MATLAB processes. Seventeen returned independently validated
  success; the no-path example returned independently validated
  `noValidatedSeed`. Every successful row retained collision and kinematic
  certificates. Exact metrics and walls were appended to `benchmark.csv` and
  `verification.md`.
- Graphics: Visible obstacle-free success created three figures. Hidden
  no-path diagnostics created two figures and retained one expanded state and
  two rejected transitions.
- Environment incidents: An initial rapid PowerShell launcher produced 18
  MATLAB startup failures before any example code executed. Two reporting
  wrappers later used stale field names after execution; both affected cases
  were rerun successfully. Four accelerating circles also emitted extensive
  near-singular interior-point warnings despite passing validation.
- Final scope: Phase A diagnostics and benchmarks, Phase B evidence, and the
  bounded Phase C research prototype are retained. Supervised learning, RL,
  production integration, and HS3 removal remain skipped by the failed
  10-turn evidence gate.
- Size: Production remains 7,117 lines. The non-example MATLAB tree remains
  11,153 lines. No file or repository size limit is exceeded.
- Next action: Audit CSV schemas, Git diff/whitespace, status, and final
  file accounting, then hand off without committing or pushing.
- Impediments: None.

### 2026-08-21 04:50 America/Denver — Goal-Mode Solver Checkpoint

- Rejected arrival experiment: Increasing shape-relative seed expansion from
  2 to 5 percent retained extreme arrival and cut its wall time to 37.9509
  seconds, but regressed wide-U arrival from 22.8196 to 24.3270 seconds. The
  2-percent value was restored completely.
- Accepted solver invariant: Earliest arrival retains interior-point
  factorization. Fixed arrival uses the conjugate-gradient subproblem because
  its average-speed initialization consistently converges the fixed-time jerk
  tie-break faster. The solver remains one entry point and adds no option.
- Fixed proof gate: Straight motion repeated at 15.3248805 degrees and four
  accelerating circles repeated at 29.2401 and 29.3633 seconds. Target-exit
  ran in 6.7637 seconds and every fixed result passed independent validation.
- Earliest proof gate: Dense concave and two opposing U shapes retained exact
  accepted arrivals with factorization. The earlier global-CG two-U regression
  is therefore excluded by the goal-mode invariant rather than hidden.
- Final sweep: All 18 maintained examples ran serially and headlessly.
  Seventeen passed independently and no-path retained its validated failure.
  Straight motion was 15.3248805 degrees, 20.30 percent below the earlier
  19.2294132-degree result. Four accelerating circles ran in 29.1694 seconds.
- Verification: The full suite passed 56 of 56 tests in 38.0220 seconds.
  Code Analyzer reported zero messages across 55 files. Visible success and
  failure checks passed with three and two figures respectively.
- Remaining work: Update assessment and verification, audit final invariants,
  and profile the next general earliest-arrival or runtime opportunity.
- Impediments: Push approval remains unavailable after the approval service
  limit; no workaround has been attempted.

### 2026-08-21 05:10 America/Denver — Geometry-Conditioned Solver Checkpoint

- Accepted invariant: Fixed arrival and untimed earliest-arrival seeds with
  one exact obstacle or reduced geometry use the interior-point CG
  subproblem. Timed earliest-arrival seeds and multiple exact obstacles retain
  factorization. The rule uses only supplied timing, obstacle count, and seed
  provenance and adds no option or scenario-specific logic.
- Arrival evidence: Dense concave improved from 8.817608547166 to
  8.798638844754 seconds. Basic improved by 0.000000166454 seconds. Other
  duration changes were below the documented one-millisecond comparison
  tolerance and are not claimed as meaningful improvements.
- Runtime evidence: Basic ran in 9.2429 seconds, dense concave in 13.0904,
  40 circles in 6.2417, and the three-region extreme example in 40.8815.
  Exact multi-obstacle alternating slalom and opposing U shapes retained
  bit-identical arrivals under factorization.
- Rejected broad form: Allowing CG on timed seeds increased moving-barrier
  wall time from the preceding 24.8248-second sweep to 28.2319 seconds. Timed
  seeds were restored to factorization. The accepted motion then returned
  bit for bit, while the final 28.1907-second wall remains unfavorable process
  variation rather than an improvement claim.
- Final sweep: All 18 maintained examples ran serially and headlessly.
  Seventeen passed independent success validation; no-path retained its
  independently validated `noValidatedSeed` result with NaN motion metrics.
- Verification: The focused HS3 suite passed 43 of 43 tests. The full suite
  passed 56 of 56 tests in 34.6245 seconds. Code Analyzer found zero messages
  across 55 MATLAB files. Visible success and failure produced three and two
  figures respectively; the failure retained two rejected transitions.
- Size: Production remains 7,140 lines, 360 below the user-approved 7,500-line
  target. The tracked MATLAB tree remains 11,874 lines, 126 below the 12,000-
  line hard cap. Core files remain 900 and 888 lines. The 694-line interactive
  sandbox remains untracked.
- Remaining work: Audit the final records and diff, then profile a next
  numerical-conditioning or arrival-time hotspot under a fresh bounded proof.
- Impediments: Push approval remains unavailable after the approval service
  limit; no workaround has been attempted.

### 2026-08-21 05:45 America/Denver — Feasibility-Recovery Checkpoint

- Accepted arrival policy: An earliest-arrival HS3 primary result that already
  meets `ConstraintTolerance` is retained directly. The second nonlinear solve
  is now reserved for recovering an infeasible primary residual instead of
  routinely trading up to one millisecond of arrival for lower jerk. Fixed-
  arrival jerk minimization is unchanged.
- Recovery proof: Removing the second solve globally made alternating slalom
  return `noValidatedSeed`. Its primary equality residuals were
  0.00331588996721 and 0.00000257574762. The accepted residual gate restores
  independently valid alternating-slalom success while feasible primary
  solutions keep the fast path.
- Arrival and runtime: Selected primary HS3 results arrive about one
  millisecond earlier. Wide-U wall decreased from 17.0979 to 7.7078 seconds,
  40 circles from 6.2417 to 4.4305, opposing U shapes from 28.2519 to 21.4478,
  and extreme outlines from 40.8815 to 36.7603. Dense ran in 12.3737 seconds
  versus 13.0904.
- Explicit tradeoff: Minimum-time primary solutions can be wider and longer.
  Dense motion increased 4.86 percent, 40 circles 2.57 percent, wide U 1.18
  percent, and opposing U shapes 0.28 percent. All hard jerk, acceleration,
  velocity, collision, endpoint, and dynamics certificates passed.
- Rejected experiments: A `1e-5` arrival tolerance made opposing-U take
  34.3097 seconds. Limited-memory BFGS took 13.5924 seconds on dense. PCG
  tolerances 0.01 and 0.2 did not change iteration counts. Feasible-guess
  `TypicalX` improved an extreme pair only 0.35 percent. Nargout-sized
  evaluator allocation was 2.76 percent slower. Every form was removed.
- Final sweep: All 18 maintained examples ran serially and headlessly.
  Seventeen passed independent success validation; no-path retained its
  independently validated `noValidatedSeed` failure.
- Verification: The focused HS3 suite passed 43 of 43 tests in 30.8299
  seconds. The full suite passed 56 of 56 in 33.8326 seconds. Code Analyzer
  found zero messages across 55 MATLAB files. Visible success and failure
  produced three and two figures; failure retained two rejected transitions.
- Size: Production decreased from 7,140 to 7,123 lines. The tracked MATLAB
  tree decreased from 11,874 to 11,857 lines. The solver is 883 lines and the
  planner is 888. The 694-line interactive sandbox remains untracked.
- Cleanup: Reused the primary or recovery constraint arrays for final
  diagnostics instead of evaluating the selected decision again. Basic and
  alternating results were bit exact and all 43 focused tests passed. The
  repository-cleanup skill also consolidated 506 superseded verification
  lines while preserving all CSV history and checkpoint evidence.
- Remaining work: Audit final records and source diff, then continue profiling
  from the accepted minimum-time source under another bounded experiment.
- Impediments: Push approval remains unavailable after the approval service
  limit; no workaround has been attempted.
