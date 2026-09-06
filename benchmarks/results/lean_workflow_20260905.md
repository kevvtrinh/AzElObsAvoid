# Lean planner workflow — 2026-09-05

Baseline: commit `4344795` plus the existing working changes and the new
result/diagnosis split. This was not a clean-commit comparison. The exact
preparation baseline source and input/result records were saved under ignored
`tmp/result-diagnosis-refactor/` before changing preparation ownership.

## Retention criteria

Remove repeated obstacle preparation from inner solve/query loops. Preserve
sampled motion, polynomial coefficients, selected/partial routes, arrival and
duration, termination reasons, and every independent validation field except
elapsed timing. Keep public stale-cache checks and full collision geometry.
Do not change solver algorithms, search budgets, margins, or tolerances.

Four deterministic cases were warmed up, timed twice, and separately profiled
in MATLAB R2024b. No randomness or display work was used. The primary metric
was preparation call count, not a general runtime claim.

| Case | Preparation calls before / after | Median wall time before / after (s) | Outcome |
| --- | ---: | ---: | --- |
| Direct, no obstacles | 3 / 1 | 0.250301 / 0.255162 | Valid motion |
| Static box detour | 5575 / 1 | 1.81801 / 1.33100 | Valid motion |
| Moving barrier with wait | 961 / 1 | 0.476463 / 0.381705 | Valid motion |
| Full-height wall | 39 / 1 | 0.290346 / 0.287972 | Expected no-path |

Exact comparisons passed for all retained motion fields and all independent
validation fields except elapsed times. Two repeats do not establish a general
speedup. The demonstrated benefit is one initial preparation per original scene
in these cases. Derived static projections are prepared separately when built.

## Interface and ownership changes

- `planTrajectory` and the intercept adapter return `[result, diagnosis]`.
  The first output retains motion, plotting inputs, and validation evidence.
  Detailed search/solver evidence is optional and is no longer duplicated
  inside the result. Solver detail tables use field paths instead of nesting.
- Public validation and geometry queries prepare caller data. Internal solvers
  call the same validation/geometry implementation with prepared histories.
  Public mutation/stale-cache regressions still pass.
- Internal stages no longer accept unused arguments. Direct and searched path
  guesses are assembled once after the optional route search.
- `prepareObstacles`, `createRouteSearchGeometry`, and `createSolverGoalState`
  replace misleading names; callers and descriptions were updated.
- Unused per-obstacle scene summaries, repeated internal result fields, and
  zero-only retired solver counters were removed.

## Verification

All 142 distinct MATLAB tests passed across the full inventory and focused
reruns. The full non-example run initially passed 138/139; its fixture hash
guard included changed output/validation calls. The input sections were checked
against HEAD and were identical. The full guarded hashes were updated after
review; the guard's scope was preserved. An unchanged geographic helper's
decorative comment edits were removed to preserve its exact fixture hash.
The remaining three example-based tests and final output/timing checks passed.
All 24 Node browser tests passed, including the separate diagnosis projection.

All 18 maintained examples ran with their default finite jerk limits: 17
returned independently valid motion; `exampleNoPath` returned the expected
failure. Metrics are appended to `benchmark.csv`. The geographic sequence's
reported motion is its final Philippines case; its wall time includes all
three regions. One opening-example helper initially lacked its diagnosis
argument; the failed execution is recorded, and the corrected example passed.

MATLAB hidden-graphics tests covered result-only plots, failure diagnosis,
coordinate wrapping, and sandbox controls. Native browser interaction was not
manually exercised; the actual page functions were exercised through Node.
PDF source references and the export consumer were updated; PDFs were not
regenerated. Changes remain uncommitted.
