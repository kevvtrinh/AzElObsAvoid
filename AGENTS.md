# AGENTS.md

Build a small, general-purpose MATLAB planning core. This branch is a blank
slate: keep only obstacle preparation, an exact visibility graph, BMTP motion
generation, independent validation, and plots of those returned core results.

## Contract

- Keep one public planner entry point:
  `result = planner(obstacles, initialState, goalState, limits, options)`.
- Keep obstacle preparation, planning, independent validation, and plotting
  conceptually separate. Return expected no-path or infeasible outcomes through
  stable `Success`, `Message`, and `TerminationReason` fields.
- Success requires a complete motion that passes the public independent
  validator. Apply obstacle margins exactly once and keep original and protected
  geometry distinct. Never weaken validation, tolerances, or geometry to pass.

## Algorithm

- Build the visibility graph exhaustively from exact prepared geometry. Do not
  add route heuristics, preferred detours, hidden waypoints, route-class pruning,
  retry schedules, fixture-specific seeds, or silent fallbacks.
- Use BMTP for motion generation. Keep any future heuristic out until identical,
  deterministic benchmarks demonstrate a necessary benefit without a correctness
  or motion-quality regression.
- Keep examples deterministic and input-driven. A change motivated by one case
  needs a structurally different regression case.

## MATLAB Style

- Public function help starts with `%% Section 0: Header & Readme` and includes
  `SYNTAX`, `PURPOSE`, `INPUTS`, `OUTPUTS`, and `UNITS`. Use numbered executable
  sections, descriptive lower-camel-case names, and physical-unit suffixes.
- Prefer readable logic inline in the public function that owns it. Create a
  helper only when logic is genuinely shared or moving it out materially improves
  clarity; avoid sprawling layers of tiny functions.
- Keep the main function body flush left and local helper bodies indented four
  spaces. Validate public inputs before computation and explain only non-obvious
  geometry, tolerances, or approximations.

## Verification

- Use MATLAB as the behavioral reference. Test direct motion, obstacle detours,
  expected no-path outcomes, invalid inputs, and independent validation.
- Measure runtime and correctness on identical inputs before retaining an
  algorithm change. Report limitations and unfavorable results plainly.
- Review the final diff and keep generated artifacts out of source control.
- Before finishing, remove temporary files and directories created during the
  task, including unneeded scratch, profiling, and benchmark outputs. Preserve
  user-created inputs and any reusable evidence that is intentionally retained
  and documented.
