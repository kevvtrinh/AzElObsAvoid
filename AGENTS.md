# AGENTS.md

Build general-purpose MATLAB planning software. These instructions apply
throughout this repository unless a more specific `AGENTS.md` applies.
Use judgment on implementation and workflow; keep changes focused on the task.

## Priorities

- Prioritize correctness and physical feasibility, then generality, diagnostics,
  stable interfaces, maintainability, performance, and visual polish.
- Base planner decisions on explicit inputs. Do not hard-code example routes,
  event sequences, preferred detours, hidden waypoints, or fixture-specific seeds.
  A fix motivated by one example needs an input-driven invariant and a test on a
  structurally different case.
- Report failures, limitations, fallbacks, and unfavorable results plainly.
  Distinguish measured evidence from inference and hypotheses. Do not claim
  completeness, optimality, or generality beyond the evidence.

## Planner Contract

- Keep one public planner entry point with these conceptual inputs in order:
  `result = planner(obstacles, initialState, goalState, limits, options)`.
  Keep obstacle construction, planning, validation, and visualization separate.
- Resolve defaults in one place. Support a zero-input call for independent
  defaults; accept partial options and omitted or empty overrides. Warn once
  about unknown fields, ignore them, and return the resolved options.
- Expected failures return a stable result with `Success`, `Message`, and
  `TerminationReason`. Reserve errors for invalid inputs, unsupported
  configurations, or corrupt internal state. Use documented empty values for
  unavailable fields; preserve the same result schema on success and failure.
- Preserve resolved inputs, limits, options, original and planning geometry,
  selected route, timed trajectory and modeled motion histories, validation,
  search statistics, elapsed time, and random seed when applicable.
- Collect search diagnostics during search, including applicable state/edge
  traces, rejection reasons, frontier, parents/costs, bounds, and best partial
  route. Retain complete counts and disclose trace downsampling. Failures must
  be plottable from the result without rerunning the planner.

## Motion and Validation

- Validate the complete returned motion independently, including between-sample
  collisions and constraint extrema, continuity, endpoint states, workspace,
  moving obstacles, and applicable velocity, acceleration, and jerk limits.
  Success requires passing the public validator used by examples and tests.
- Keep original and safety-adjusted geometry distinct; apply margins exactly
  once. Make units, tolerances, interpolation, and coordinate-wrap policies
  explicit. Sampled clearance is not proof of continuous safety.
- Do not obtain a passing result or speedup by clipping trajectories, weakening
  validation or tolerances, reducing protected geometry, or hiding a fallback.
  Distinguish proven infeasibility, exhausted search budgets, unresolved
  certification, and invalid motion. Do not assume time feasibility is monotone
  for moving obstacles.

## MATLAB Conventions

- MATLAB is the behavioral reference. Report Octave or unavailable-runtime
  limitations separately from code failures.
- Use functions and structures, with one public function per matching file.
  Keep execution order readable; use helpers for shared invariants or clarity.
- Follow surrounding style: descriptive lower-camel-case names, physical unit
  suffixes (`_deg`, `_s`, `_deg_s`, `_deg_s2`, `_deg_s3`), and explicit shapes.
  Preserve established public field names and compatibility unless a breaking
  change is authorized. Update affected callers, tests, and docs together.
- Public function help starts with `%% Section 0: Header & Readme` and covers
  `SYNTAX`, `PURPOSE`, `INPUTS`, `OUTPUTS`, and `UNITS`. Use numbered executable
  sections. Local helpers and test cases need only concise purpose comments.
- Validate and normalize public inputs before computation. Explain non-obvious
  approximations and tolerances; prefer clear loops or vectorization over
  `cellfun`/`arrayfun`. Avoid unrelated formatting and speculative abstractions.

## Examples and Verification

- Keep examples uniform and readable: controls -> obstacles -> explicit states,
  limits, and options -> public planner -> independent validation -> plots.
  Return the planner result unchanged; keep example metrics and handles local.
- Support default/empty overrides and consistent visibility, animation,
  kinematics, playback, and seed controls. Plots consume returned results and
  the geometry used for validation. Show failure diagnostics without a path;
  successful displays include the route, motion, and kinematics with limits.
- Choose verification proportional to the change. Use focused regressions;
  broaden to affected scenario families for planner changes, including success,
  expected failure, static/moving obstacles, and motion modes as applicable.
  Exercise defaults, headless controls, and graphics when those paths change.
  Documentation-only changes do not require planner runs.
- Keep tests deterministic; do not regenerate failed random cases or weaken
  assertions to pass. Report what ran, failed, or remains untested.
- For every executed maintained example and jerk mode, report in chat: planner
  and independent-validation status, polyline and smoothed lengths (deg), motion
  duration (s), collision/kinematic/certificate status, and termination reason.
  Use `NaN` for unavailable numeric metrics.

## Investigation and Change Discipline

- Inspect existing interfaces, working changes, and baseline evidence first.
  Preserve user work and keep generated artifacts out of source control.
- Diagnose the earliest broken stage before changing behavior. Measure suspected
  bottlenecks; source inspection alone does not establish runtime cost.
- For algorithm or performance experiments, define acceptance and revert criteria
  before measuring. Compare identical inputs, options, and environments; warm
  up and repeat timings enough to assess noise. Report runtime alongside motion
  quality and correctness. Retain complexity only for a demonstrated benefit.
- Keep `branch_assessment.md` current when findings change and append actual
  example runs to `benchmark.csv` using its existing schema. Preserve unfavorable
  history; correct disproven claims where recorded. Never invent rerun results.
- A requested replacement must reach all applicable production callers and
  examples. Verify the executed method; disclose incomplete migration and
  remove superseded paths once no longer needed, unless coexistence is requested.
- Review the final diff and state the outcome and material verification limits.
