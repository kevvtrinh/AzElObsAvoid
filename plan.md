# Quintic solver quality and runtime recovery

## Outcome

Completed a general, input-driven recovery of compact corridor-quintic timing
quality and runtime. No scenario names, hidden waypoints, geometry changes,
limit relaxation, validation changes, or HS3 fallback were introduced.

## Root causes and retained design

- Refined exact-C3 routes doubled every interior knot and crossed a dense-QP
  dimension cliff. Keep exact C3 through 48 decisions and use the existing
  continuous-C4 exact representation above that bound.
- Route subdivision incorrectly replaced geometric span weights with equal
  weights. Always derive timing weights from the actual refined edge lengths.
- Six earliest-arrival trials left wide feasible/infeasible brackets. Use 14
  bounded trials for exact earliest-arrival searches; fixed arrival uses one.

Rejected experiments included a larger C3 map, pure length-proportional
timing, and axis-demand weighting. They either retained the defect or worsened
path length, jerk, or wall time.

## Final evidence

- `az_el_sandbox_goal_20260824_174716.mat`: identical selected trajectory at
  37.845175 s; wall time 77.9230141 -> 9.2077994 s (88.18% reduction).
- `173vs131.mat`: compact arrival 173.25 -> 136.042437744 s; HS3 is
  131.642423799 s. Compact remains 4.400014 s later but has the shorter
  smoothed path and lower integrated jerk-squared.
- Structurally distinct 12-hairpin route: valid 138.455023011 s arrival,
  8.2178 s wall time, 96 C4 decisions, and 0.02-degree clearance.
- Maintained compact examples: 18/18 contracts passed, comprising 17 valid
  successes and the expected valid no-path result.
- Graphics: visible success created 3 figures/6 axes; expected failure created
  2 figures/2 axes from returned diagnostics.
- Automated suite: 144/144 passed in 643.151742 s wall time.
- Task-only detached push gate: 142/142 passed in 709.815981 s wall time;
  unrelated dirty source and tests were absent.
- Code Analyzer: zero messages in the changed production owner and focused
  regression test. `git diff --check`: no whitespace errors.

## Files owned by this task

- `+azElPlannerMethods/+corridor/+internal/+motion/solveCompactC3.m`
- `tests/testCorridorPlannerDynamicTiming.m`
- `verification.md`, `branch_assessment.md`, `benchmark.csv`, and `plan.md`

The worktree retains unrelated preexisting user edits. Temporary task scripts,
result MAT files, and the detached baseline worktree are removed at handoff.
No commit or push was requested.
