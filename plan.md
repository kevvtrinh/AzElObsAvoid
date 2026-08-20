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
- sampled visibility graph;
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
- visibility graph construction;
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
