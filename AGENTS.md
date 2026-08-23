# AGENTS.md

## Purpose

These instructions apply to every file in this repository unless a more
specific `AGENTS.md` exists in a subdirectory.

Build general-purpose planning software. Prefer reusable algorithms, stable
interfaces, and uniform examples over scenario-specific code. A change is not
complete merely because it makes the current examples pass.

## Priority Order

When requirements compete, use this order:

1. Correctness and physical feasibility.
2. General behavior across unseen scenarios.
3. Clear failure diagnostics and reproducibility.
4. A small, stable public interface.
5. Maintainability and readability.
6. Runtime and memory efficiency.
7. Visual polish.

Never trade a higher-priority item for a lower-priority one without documenting
the reason and the observed consequence.

Do not trick the user into thinking a requirement passed by hiding a limit,
violation, fallback, substitution, or unfavorable result inside the code.

## Generality Rules

- Do not put example names, obstacle names, map shapes, or expected routes into
  planner logic.
- Do not encode an example's event sequence, route transition, opening time,
  or expected decision as a production-planner heuristic, even when the code
  uses neutral names. Generality is determined by behavior, not identifiers.
- A planner change motivated by one example must state the input-driven
  invariant it implements and must be tested on a structurally different case.
  If no general algorithm has been implemented, report that limitation instead
  of presenting an example-specific heuristic as general, adaptive, optimal,
  or complete.
- Do not pretend that passing the motivating example proves general behavior.
  Report exactly which scenario families and failure modes were exercised.
- Do not add hidden waypoints, preferred detour directions, guided corridors,
  hard-coded seeds, or scenario-specific branches to make a fixture pass.
- Do not trick the user into thinking a requirement passed by hiding a limit,
  violation, fallback, substitution, or unfavorable result inside the code.
- Do not infer behavior from function names or filenames. Behavior must follow
  only from explicit inputs.
- Keep obstacle construction, planning, validation, and visualization separate.
- Keep one public planner entry point when the supported behaviors can be
  expressed through inputs and options. Put shared implementation details in
  private/local helpers rather than creating competing planners.
- Add an option only when it represents a meaningful user choice. Do not expose
  internal constants without a demonstrated need.
- New templates and documentation must use neutral names such as `obstacles`,
  `initialState`, `goalState`, `limits`, `options`, and `result`. Avoid embedding
  the repository name or a particular mission/scenario name.
- Example code may define a specific scenario, but reusable helpers and public
  interfaces must remain scenario-independent.

## Required Planner Contract

The public planner should accept these conceptual inputs in this order:

```matlab
result = planner(obstacles, initialState, goalState, limits, options);
```

The exact function name is repository-defined, but the roles must remain clear:

- `obstacles`: validated static or time-varying obstacle data.
- `initialState`: initial time, position, and supported motion state.
- `goalState`: goal time or time policy, position, and supported terminal state.
- `limits`: physical and workspace limits, with units in field names.
- `options`: algorithm and display choices that do not change input meaning.
- `result`: a stable success-or-failure record containing the trajectory and
  diagnostics needed for independent validation and plotting.

Use a zero-input call for argument-independent default options. Resolve defaults
in one place. Partial option structures are valid; omitted or empty fields use
defaults. Warn once about unknown option fields and ignore them, and always echo
the resolved options in the returned record.

### Success and failure behavior

Expected planning outcomes must return a result rather than terminate the
example:

```matlab
result.Success = false;
result.Message = "No feasible path found.";
result.TerminationReason = "openSetExhausted";
```

Expected outcomes include an exhausted search, unreachable goal, time-budget
limit, iteration limit, and dynamically infeasible routes. Reserve errors for
invalid inputs, violated API contracts, corrupt internal state, or unsupported
configurations.

The result schema must remain stable on success and failure. Fields without a
value should contain a documented empty value rather than disappear.

Do not trick the user into thinking a requirement passed by hiding a limit,
violation, fallback, substitution, or unfavorable result inside the code.

### Required result information

At minimum, return or preserve:

- success flag, message, and machine-readable termination reason;
- resolved inputs, limits, and options;
- original obstacle geometry and planning/inflated geometry when they differ;
- selected geometric route and time-parameterized trajectory on success;
- position, velocity, and acceleration histories when those states are modeled;
- collision and constraint-validation summaries;
- candidate-route or search statistics;
- elapsed planning time and deterministic random seed when randomness is used;
- search diagnostics sufficient to reconstruct a failure plot.

Do not hide a violated constraint by clipping the returned trajectory. Report
the violation and fix the generating algorithm.

## Search Diagnostics Are a First-Class Output

Every search-based planner must collect diagnostics while it searches, not try
to reconstruct them after failure. Use a stable structure such as
`result.SearchDiagnostics` containing the applicable subset of:

- sampled or discretized workspace bounds and resolution;
- time horizon and time resolution;
- generated, expanded, reopened, and rejected states;
- accepted edges or motion primitives;
- collision-rejected and dynamics-rejected transitions;
- frontier/open-set states at termination;
- closed/visited states;
- parent relationships and cost-to-come values;
- heuristic values when a heuristic search is used;
- start and goal states;
- best partial state and its distance or cost to the goal;
- iteration count, elapsed time, and termination reason.

Large traces may be downsampled for display, but retain counts for the complete
search and document the downsampling rule. Diagnostic collection must not
change planner decisions.

Provide one reusable search-diagnostic plotting function. It must be able to
show, as applicable:

- original and safety-adjusted obstacles;
- start and goal;
- explored states/nodes;
- accepted search edges or motion primitives;
- rejected transitions using visually distinct categories;
- final frontier and best partial route;
- workspace and time bounds.

The plot title or annotation must include the termination reason and key counts.
Failure visualization is required even when no trajectory exists.

## Uniform Example Template

All runnable examples must follow the same top-level sequence and section
ordering. Use this structure:

```matlab
function result = exampleScenario(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleScenario()
%   result = exampleScenario(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate one scenario through the maintained public planner.
%   - State the general behavior tested and expected success or failure.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Display and runtime overrides documented by field.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable planner result plus validation and example metadata.
%**************************************************************************
% UNITS
%   - Position is degrees, time is seconds, velocity is degrees per second,
%     and acceleration is degrees per second squared.
%**************************************************************************

%% Section 1: Resolve Example Controls
% Merge display/runtime overrides with documented defaults.

%% Section 2: Create Obstacles
% Build only canonical obstacle data through public constructors.

%% Section 3: Create Planner Inputs
% Define initialState, goalState, limits, and options explicitly.

%% Section 4: Run Planner
result = planner(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result
validation = validatePlannerResult( ...
    result, obstacles, initialState, goalState, limits, options);
result.Validation = validation;

%% Section 6: Plot Diagnostics And Motion
if result.Success
    % Plot the workspace, selected path, animation, and kinematics.
else
    % Plot the propagated search space and failure diagnostics.
end

%% Section 7: Return Example Metadata
% Store scenario geometry and validation results needed for inspection.
end
```

Every example must visibly and locally define:

- obstacle geometry and obstacle time history;
- initial state;
- goal state or goal-time policy;
- limits;
- planner options;
- example display/runtime controls.

Do not bury these inputs in unrelated helpers. Fold one-call state and default
constructors into the visible progression. Retain a geometry helper only when
its vertex-level detail would obscure the scenario flow, and explain that
reason beside the call.

Examples must call public production functions. Do not duplicate collision
checking, motion profiling, or search logic inside an example.

### Example options

Every example should accept one optional scalar override structure. Calling the
example with no input or `[]` must use defaults successfully. At minimum,
provide consistent controls for:

- figure visibility;
- whether animation is shown;
- whether kinematics are plotted;
- animation frame stride or playback speed;
- deterministic random seed when applicable.

Normalize text options once and accept the documented MATLAB text forms. Test
the default, an explicit visible setting, and an explicit headless setting.

### Example validation

Examples are executable demonstrations and must validate their own results.
On success, check at least:

- the reported success state and termination reason;
- finite, strictly nondecreasing trajectory time;
- initial and terminal state agreement within documented tolerances;
- workspace-bound compliance;
- velocity and acceleration limits, plus jerk if modeled;
- collision freedom over the full time-parameterized trajectory;
- safety-margin policy;
- coordinate-wrap policy when applicable.

Validation must use public collision/constraint functions or an independent
check. It must not simply trust a planner-owned `Success` flag.

On failure, validate that:

- the termination reason is recognized;
- the message is actionable;
- search counts are internally consistent;
- diagnostic arrays have compatible sizes and finite values where required;
- the search-diagnostic plot can be created without a selected path.

Assertions should explain the violated quantity, observed value, limit, and
tolerance.

### Example visualization

Successful examples must produce, when enabled:

1. workspace/search-space view with obstacles and selected route;
2. motion animation against the time-varying obstacles;
3. position, velocity, and acceleration plots with applicable limits.

Failed examples must produce, when enabled:

1. workspace/search-space view;
2. propagated nodes/states and accepted edges/primitives;
3. rejected transitions by reason when available;
4. frontier or best partial route;
5. a concise failure annotation.

Visualization code must consume the returned result. It must not rerun the
planner or use scenario-specific knowledge.

## Motion and Collision Correctness

Do not trick the user into thinking a requirement passed by hiding a limit,
violation, fallback, substitution, or unfavorable result inside the code.

- Plan and validate the complete time-parameterized motion, not only geometric
  waypoints.
- Preserve continuity required by the selected motion model.
- Check constraints between samples when interpolation could exceed a limit.
- Check collisions along every edge or primitive and throughout obstacle time,
  using a resolution justified by geometry and dynamics.
- Treat original geometry and safety-adjusted geometry as different data with
  explicit provenance. Apply a safety margin exactly once.
- Keep units in variable and field names, for example `_deg`, `_s`, `_deg_s`,
  and `_deg_s2`.
- Use tolerances intentionally. Name them, validate them, and report them in
  diagnostics.
- Do not declare success unless the final returned trajectory passes the same
  public validation used by examples/tests.

## MATLAB Coding Style and Headers

These rules apply to every MATLAB source file in the repository. MATLAB is the
source language and behavioral reference. Octave may be used for numerical
smoke tests, but do not rewrite correct MATLAB solely to satisfy an Octave
incompatibility. Record runtime limitations separately from code failures.

### Function-oriented design

- Keep the code function-oriented. Packed obstacle data, states, limits,
  options, planner results, and diagnostics are structures. Do not introduce
  classes or handle wrappers for them.
- Keep one public function per file, and make the filename match that function
  exactly.
- Keep the main algorithm inline and in execution order. Retain a local helper
  only when several call sites share a nontrivial invariant or when inlining it
  would obscure the main algorithm.
- Put local functions after the main executable sections under a final numbered
  `Local Functions` section.
- Hoist frequently read packed arrays into local variables before inner loops
  instead of repeatedly dereferencing structure fields.

### Mandatory function header

Every function, public or local, begins immediately after its complete function
declaration with:

```matlab
%% Section 0: Header & Readme
```

The help block must list every supported call form under `SYNTAX`, followed by
the four required blocks in this exact order:

1. `PURPOSE`
2. `INPUTS`
3. `OUTPUTS`
4. `UNITS`

Separate the blocks with `%` followed by 74 asterisks:

```matlab
%**************************************************************************
```

Use this canonical public-function header:

```matlab
function result = functionName(requiredInput, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   defaults = functionName()
%   result = functionName(requiredInput)
%   result = functionName(requiredInput, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Describe one behavior-oriented responsibility.
%**************************************************************************
% INPUTS
%   - requiredInput (type and shape)
%       Describe meaning, required fields, ordering, and constraints.
%   - optionOverrides (scalar struct, optional; default struct())
%       Describe each accepted field and its default.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Describe the stable success-or-failure schema and important fields.
%**************************************************************************
% UNITS
%   - State every physical unit and array ordering used by the interface.
%**************************************************************************
```

Header requirements:

- List only call forms the function actually implements. Include a zero-input
  defaults call only when it exists.
- Use the actual function and argument names from the declaration.
- `PURPOSE` entries use `%   -` bullets and describe responsibilities rather
  than implementation history.
- Arguments use `%   - name (type and shape)`. Continue their descriptions on
  the next indented line. Nest structure fields one level deeper.
- State whether an input is optional, its default, accepted empty behavior, and
  important cross-field constraints.
- Describe coordinate ordering and history orientation, for example N-by-2
  `[azimuth elevation]`, whenever ambiguity is possible.
- Put a caveat beside the input, field, or output it governs instead of adding a
  detached notes section.
- Document failure behavior in `OUTPUTS`, including whether invalid input throws
  and expected planning failure returns `Success = false`.
- A short local helper still receives the `Section 0` contract, but its body
  needs no additional executable sections when it reads as one idea.
- Function-based local test cases are the one exception: a descriptive test
  function name and a concise purpose comment are sufficient.

### Executable sections

Use numbered, title-cased sections in execution order. A typical planning
function uses:

```matlab
%% Section 1: Validate Inputs & Apply Defaults
%% Section 2: Build The Planning Representation
%% Section 3: Generate Candidate Routes
%% Section 4: Retime & Validate Candidates
%% Section 5: Assemble The Output
%% Section 6: Local Functions
```

Name the actual algorithm or operation when a more specific title is useful.
Do not use unnumbered top-level `%%` cells. Do not place `%%` sections inside a
loop or conditional. For a visible internal stage within a long section, use:

```matlab
% --- Broad-Phase Reject Box ---------------------------------------
```

Section titles describe the work performed, not vague phases such as `Process`
or `Miscellaneous`. Renumber sections whenever their execution order changes.

### Naming and units

- Use descriptive lower-camel-case names for functions and local variables.
  Preserve the established `AzEl` acronym casing in names such as
  `planAzElMotion` and `azElData`.
- Spell words out: use `maximumVertices`, not `maxVerts`, and
  `neighborElevationIndex`, not `nbrElIdx`.
- Use singular names for one record and plural names for collections. End
  indices with `Index`, counts with `Count`, and graphics handles with `Handle`
  or `Handles`.
- Boolean names read as assertions or controls, such as `isUniformTime`,
  `edgeHasLength`, `hasOriginalObstacleField`, `showAnimation`, and
  `allowAzimuthWrapping`.
- Constants and tolerances receive descriptive names and unit suffixes. Do not
  leave unexplained magic numbers in algorithmic code.

Physical quantities carry these suffixes:

| Suffix | Meaning |
| --- | --- |
| `_deg` | degrees |
| `_rad` | radians |
| `_s` | seconds |
| `_deg_s` | degrees per second |
| `_rad_s` | radians per second |
| `_deg_s2` | degrees per second squared |
| `_deg_s3` | degrees per second cubed |
| `_deg2` | square degrees |
| `_1_deg` | per degree |

Extend the same readable pattern for other compound units, such as `_deg2_s5`
for integrated squared jerk. Dimensionless quantities have no suffix.

The canonical packed container name is `obstacleField`, with plural record
members such as `obstacleField.Obstacles`. Existing public fields are API
contracts and keep their current spelling. In particular:

- planner status/control fields remain Pascal case, for example `Success`,
  `Message`, `TerminationReason`, `Options`, `Validation`, and
  `SearchDiagnostics`;
- scientific payload fields remain descriptive and unit-bearing, for example
  `time_s`, `position_deg`, `velocity_deg_s`, and `acceleration_deg_s2`;
- established packed-record fields and current option names are compatibility
  exceptions, not patterns to justify an unrelated new naming scheme.

Do not rename a public field solely to improve style. Apply the migration rules
below when a rename has functional value.

### Options structures

- A public function with argument-independent defaults supports a zero-argument
  call that returns a fully populated options structure.
- When defaults depend on required input, support an explicit defaults request
  rather than inventing dummy inputs, for example:

  ```matlab
  options = planner(limits, "defaults");
  ```

- One defaults structure or local defaults function is the only source of
  truth. Do not repeat the same default in validation, examples, and plotting.
- Optional override inputs accept both omission and `[]` consistently.
- Partial structures are valid. Omitted or empty fields receive defaults.
- Unknown fields warn once per call and are ignored. The warning lists every
  ignored field and states that no behavior was changed by it.
- Resolve defaults, normalize representation, and validate values before the
  main computation.
- Echo the fully resolved options in the result or returned display record.
- Normalize scalar text once to MATLAB strings. Validate logical controls as
  scalar logical or binary numeric values before converting them.

### Validation, errors, and warnings

Validate conditions whose failure would otherwise be silent or appear far
downstream, including:

- required fields and scalar-structure contracts;
- numeric type, shape, finiteness, and orientation;
- strictly increasing time bases;
- workspace and algorithm-dependent domains;
- array sizes and cross-field relationships;
- bounds or caps that could silently discard input data.

Use `validateattributes` for numeric arguments and explicit identified errors
for structural and cross-field failures. Normalize row/column orientation at a
public boundary rather than making every inner loop accept both forms. Do not
repeat expensive validation already performed naturally by the main packing or
search loop. When ragged, empty, or nonfinite input is deliberately tolerated,
document the exact behavior at the point of use.

Errors and warnings use the emitting function plus a Pascal-case problem name:

```matlab
error("buildAzElTimeObstacleField:BoundaryCountMismatch", ...)
warning("planAzElMotion:UnknownOptions", ...)
```

Messages are actionable. Name the affected input or field, expected type or
shape, unit when relevant, and observed value or count when useful. Errors are
reserved for invalid contracts, unsupported configurations, and corrupt
internal state. Expected planning outcomes return the stable failure schema.

Warn when requested behavior, samples, or geometry are reduced, dropped, or
ignored and the returned value alone would not reveal it. Accumulate counts
inside loops and warn once per obstacle or result. State the consequence, not
merely the event, and distinguish an expected tradeoff from an implementation
defect.

### Comments

- Comments explain why, the invariant being protected, or the consequence of a
  choice. Do not narrate code that already states what it does.
- Explain every non-obvious tolerance, magic constant, deliberate asymmetry,
  and approximation at the point of use.
- When a helper exists to centralize an invariant, state what would diverge if
  callers duplicated the logic.
- Use complete sentences for explanatory block comments.
- Keep `%#ok<...>` suppressions local. Explain intentional growth or another
  non-obvious suppression beside the affected block.

### Return schemas and structure arrays

- Every exit path returns the same public fields in the same order.
- Construct success, failure, and empty results from one stable template helper.
  Use documented empty arrays, `NaN`, `false`, or empty records where a value is
  unavailable; do not omit a field on failure.
- Preallocate structure arrays from the same empty-record template used by the
  final output.
- When final size is unknown and the result may be large, grow storage
  geometrically and trim once. Small diagnostic growth may use `%#ok<AGROW>`
  only when the simplicity is worth the bounded cost and the reason is stated.
- Use scalar structures to group states, limits, options, results, and graphics
  records. Keep field order stable for readable inspection and diffs.
- Collect search traces and counts during the search. Plotters must not
  reconstruct a missing trace or rerun planning.

### Layout and formatting

- Target about 78 characters per line. Treat 100 characters as a hard limit
  except for an unbreakable identifier or URL.
- Indent block bodies four spaces. Do not use tabs or trailing whitespace.
- Put one statement on each line and use spaces after commas and around binary
  operators.
- Continue long expressions with `...` and indent continuations four spaces
  beyond the owning statement. Align related function arguments and structure
  field/value pairs consistently.
- Do not end a line with an assignment or comparison operator followed only by
  `...`. Put the first meaningful term on that line or name an intermediate
  assertion so the operation remains visible when scanning vertically.
- Replace long compound conditions with named intermediate assertions when that
  improves debugging and readability.
- In a multiline `struct(...)`, normally put one field/value pair on each line.
  A small one- or two-field structure may remain on one line when readable.
- Use explicit empty shapes, such as `zeros(0, 2)`, when column meaning matters.
- Use double-quoted strings for semantic text, field/property names,
  identifiers, and enumerations. Use character vectors only where a MATLAB API
  requires them, such as `validateattributes` lists or a legacy format string.
- Do not use `cellfun` or `arrayfun`. Prefer a vectorized operation when it
  remains readable; otherwise use an explicit loop with a diagnostic index name
  such as `sampleIndex`, `obstacleIndex`, `regionIndex`, or `routeIndex`.
- Avoid broad formatting or unrelated refactors in a focused change.

### Renaming and compatibility

- A public rename keeps the previous spelling as a compatibility alias for one
  release unless the task explicitly authorizes a breaking change.
- Mark every compatibility shim with the word `deprecated` so its eventual
  cleanup is one repository search.
- Centralize forwarding aliases instead of maintaining two implementations.
- Collision queries and readers may accept an old format tag during migration,
  but writers publish only the current format.
- Automated replacement does not understand compatibility branches. Review
  every changed `isfield`, format-tag check, fallback, and alias by hand.
- Update examples, tests, headers, and returned-field documentation in the same
  change as a public rename.

### Visualization coding style

- Visualization consumes the same packed obstacle field and returned planner
  result used by collision validation so displayed and checked geometry cannot
  diverge.
- Pass axes handles explicitly to plotting functions; do not rely on `gca`.
- Use `figureHandle`, `axesHandle`, and plural `Handles` names consistently, and
  return handles required for inspection or later updates.
- Put units in every axis label. Use `DisplayName` for legend entries. Set
  `hold`, `grid`, `box`, and spatial scaling explicitly.
- Visually distinguish original and safety-adjusted geometry, selected and
  candidate routes, and accepted and rejected search elements.
- Kinematic plots include applicable limit lines. Failure plots annotate the
  termination reason and key search counts.
- Respect `FigureVisible`, animation toggles, frame stride, and pause/playback
  controls. Hidden figures never sleep merely to simulate animation.
- Plotting and animation functions do not rerun the planner or use knowledge of
  a named example.

### Examples, benchmarks, and tests

Do not trick the user into thinking a requirement passed by hiding a limit,
violation, fallback, substitution, or unfavorable result inside the code.

- Numbered examples, if numbering is used, follow one visible progression:
  construct canonical obstacles, define the request, run the maintained
  planner, independently validate the result, then animate and report.
- Randomized generators preserve seeded draw order during readability changes.
  Never regenerate a case merely because planning failed.
- Benchmarks report the seed or input scale and the evidence needed to reproduce
  the result.
- Public test entry points use the complete `Section 0` header contract.
  Individual local test cases remain ordinary function-based tests whose
  descriptive names serve as their headers.

### Required example-run reporting

Whenever one or more maintained example files are executed, report every
executed example directly in the chat. Do not require the user to open a
generated report or local artifact to see the result. For each example and
motion-constraint mode, include:

- example name and whether the jerk constraint was enabled;
- planner success and independent example-validation status;
- selected polyline length in degrees;
- smoothed-path length in degrees;
- minimum motion duration in seconds;
- collision-free status and applicable kinematic/certificate status.

If a requested metric is unavailable because planning failed, print the
documented empty value and the termination reason. State the exact baseline
when making a performance comparison.

## Testing and Verification

Do not run every maintained example before a planned code change. Inspect the
available baseline evidence, make the code change, and run the required
example verification after the change. Before the change, run only the
focused cases that are necessary to identify a specific failure.

Before considering a change complete:

1. Run syntax/static checks available in the environment.
2. Run focused unit tests for modified helpers.
3. Run every example headlessly with plots and animation disabled.
   Run only one example process at a time. Finish and record the result,
   output, runtime, and failure diagnosis for that example before starting
   the next example. Do not run examples as one combined suite or in
   parallel.
4. Run at least one visible example when graphics are available.
5. Exercise both a successful plan and an expected no-path result.
6. Verify the failure case produces a search-space diagnostic figure.
7. Test static and moving obstacles when either path is affected.
8. Test each supported motion profile when motion logic changes.
9. Test default options and user overrides separately.
10. Report commands run, runtime used, passes, failures, and untested items.

Use deterministic tests. If a planner is randomized, set and return the seed.
Do not weaken assertions, enlarge tolerances, reduce obstacle geometry, or alter
expected results merely to obtain a passing run.

## Branch Assessment and Benchmark Records

Every agent must inspect these root records before a commit or push. Every
agent that changes planner behavior, example behavior, validation, diagnostics,
or runtime must maintain them:

- `branch_assessment.md` states the largest current strength and weaknesses.
  Base each statement on measured evidence. Update it when a change affects
  correctness, coverage, diagnostics, maintainability, size, or runtime. Keep
  unfavorable limits visible. Do not convert a local result into a global
  optimality or completeness claim.
- `benchmark.csv` contains one row for each executed maintained example and
  motion-constraint mode. Record the run date, source commit, branch, example,
  goal-time mode, jerk state, planner and independent-validation states,
  polyline and smoothed lengths, motion duration, collision and certificate
  states, wall time, termination reason, and concise notes.

Before a commit or push, update both records when the current work changed the
evidence that they describe. If no benchmark was executed, do not invent or
copy a new result. Preserve the last measured row and state that the metric was
not rerun in the chat. Append a new row when the source commit, option mode, or
measured result changes. Do not delete or replace an unfavorable historical
row to make a comparison look better. Correct a factual CSV error in place and
explain the correction in the commit message or verification report.

Use `NaN` for an unavailable numeric benchmark value. Use an empty note only
when no clarification is required. Keep the CSV header stable. Add a column
only when it represents a general result that later runs can populate.

## Change Discipline

### Performance-Based Production Size Allowance

- The production MATLAB target is 7,500 physical lines. A measured runtime
  improvement can permit a proportional overage.
- Each 100 physical production lines above the target requires at least a
  25 percent wall-time reduction. Apply the rule proportionally:
  `required reduction = 0.25 * excess lines / 100`.
- Declare the representative affected benchmark set before evaluation. Use
  the smallest wall-time reduction in that set. Do not use the average or the
  best result to hide a regression.
- Use the same inputs, options, environment, and independent validation for
  the baseline and changed runs. Each benchmark must keep correctness and
  arrival or route quality within its documented tolerance.
- Record the line count, formula, baseline, changed result, and minimum
  measured reduction in `verification.md` before a commit or push.
- This allowance does not change the 900-line production-file limit, the
  12,000-line maintained planner/test-tree limit excluding `examples/`, or any
  correctness, generality, diagnostic, interface, and non-regression
  requirement. Example files have no repository line cap, but their full line
  count must still be reported and they must not be used to justify planner
  growth.

- Inspect existing interfaces and call sites before editing.
- Before completing, committing, or pushing a change, inspect the per-file
  diff statistics. If an existing source file or script has more than 50 added
  lines in the current change, explain that growth directly in the chat before
  claiming completion.
- For every file above the 50-added-line threshold, report its additions,
  deletions, and net growth; the responsibilities added; why existing code was
  not sufficient; the alternatives considered; why the selected design and
  file location were chosen; and the checks that exercised the new code.
- Count headers, comments, UI plumbing, compatibility code, and helper
  functions in the disclosure. Do not dismiss them as boilerplate or hide them
  by reporting only net line growth.
- If a working-tree diff combines several user requests, separate the line
  growth by request when reliable history permits it. Otherwise state that the
  exact separation is unavailable and explain the cumulative diff honestly.
- If the explanation does not justify the size or reveals duplicated or
  weakly related responsibilities, simplify or refactor the change before
  completing it. Moving the same code into another file is not a size
  reduction and must not be presented as one.
- Preserve backward compatibility unless the task explicitly authorizes an API
  change. If an API must change, update all examples, tests, and documentation
  together.
- When a branch is explicitly dedicated to a new planner or replacement
  feature, treat that feature as the branch's production implementation, not
  as a sidecar demonstration. Before the branch is complete, migrate every
  maintained runnable example and applicable wrapper to use the new feature.
- On an experimental or replacement branch, a maintained example counts as
  passing only when its returned diagnostics prove that it executed the
  branch's experimental production method. A legacy HS3/NLP solve, hidden
  fallback, or result substituted from the legacy planner does not satisfy the
  example gate; treat that example as unmigrated even if independent
  validation passes.
- On a feature-specific replacement branch, remove superseded implementation
  code, scripts, options, tests, benchmarks, and adapters once the new feature
  no longer depends on them. Do not retain parallel legacy execution paths for
  compatibility unless the user explicitly requests coexistence. Preserve
  shared infrastructure that the replacement genuinely uses.
- Verify feature-specific branch completeness with a repository-wide call-site
  and dependency audit. A standalone example is not evidence that the other
  examples use the feature. If migration or legacy removal is incomplete,
  report the branch as incomplete rather than claiming the feature is fully
  applied.
- Preserve user changes and avoid unrelated edits.
- Keep generated files, temporary output, and runtime artifacts out of source
  control.
- Do not claim verification that was not run.
- When blocked by an unavailable MATLAB feature, distinguish an environment
  limitation from a code failure.

## Completion Checklist

A planner or example change is complete only when all applicable answers are
yes:

- Is the implementation general rather than tailored to named examples?
- Does the example follow the uniform obstacle/input/planner/validate/plot flow?
- Are defaults and headless overrides tested?
- Is the full timed trajectory independently validated?
- Does expected failure return a stable result instead of losing diagnostics?
- Can failure propagation be plotted without rerunning the search?
- Are animation and kinematic plots driven by the returned result?
- On a feature-specific branch, do all maintained examples and applicable
  wrappers execute the feature's production path?
- On a feature-specific replacement branch, were superseded implementation
  paths removed after confirming that no required dependency remains?
- Are safety margins and units explicit and applied exactly once?
- Were success and failure paths both exercised?
- Are verification limits and untested cases stated honestly?
