# Quintic-only branch cutover

## Objective

Create `quintic-only` from committed `325-full-suite` tip `9439a43`,
remove the superseded planner completely, eliminate the redundant
`azElPlannerMethods.corridor.plan` hop, preserve the maintained
corridor-quintic behavior, verify it, and commit the result.

## Current design

- `planAzElMotion` is the sole public planner and directly owns input/default
  resolution, endpoint rejection, topology generation, candidate orchestration,
  and timing finalization.
- Quintic motion, obstacle, search, and timing helpers live under
  `+azElInternal`.
- `MotionMethod="corridorQuintic"` remains a compatibility value but does not
  dispatch behavior.
- Historical benchmark and assessment evidence is retained; no removed planner
  implementation, adapter, option owner, dedicated test, or live documentation
  link remains.

## Completed work

- Created isolated Git branch/worktree `quintic-only` from `325-full-suite`.
- Removed the complete `+azElPlannerMethods` tree, three dedicated planner
  test files, and the standalone scaling CSV.
- Moved the maintained helper implementation to `+azElInternal` and migrated
  all callers, examples, benchmarks, tests, sandbox controls, and docs.
- Simplified moving-target and obstacle-query adapters to the one public
  planner.
- Updated `README.md`, `short_file_rationale.md`,
  `branch_assessment.md`, `verification.md`, and `benchmark.csv`.

## Evidence

- Code Analyzer: zero findings across 81 MATLAB files.
- Tests: 77/77 passed in 216.3257 seconds.
- Maintained examples: 18/18 independent contracts passed; 17 collision-free,
  kinematically certified successes and one expected validated
  `noValidatedSeed` failure.
- Graphics: visible success produced three figures; expected failure produced
  two diagnostic figures and retained 15 rejected transitions.
- Active dependency and filesystem audits: zero removed-planner and zero
  `azElPlannerMethods` implementation matches.
- Detailed metrics and the one pre-execution MATLAB startup failure are retained
  in `verification.md` and `benchmark.csv`.

## Current scope and limitations

Production contains 44 MATLAB files, 7,874 physical lines, and 5,726
nonblank/noncomment lines under the established exclusion rule. The refactor
preserves the finite candidate search; it does not establish completeness,
global optimality, or a uniform runtime improvement.

The original root `sandbox` checkout and dirty `325-full-suite` worktree
remain untouched. All task changes are isolated in `quintic-only`.

## Final outcome

The explicit task paths are staged. Rename-aware review reports 759 additions,
4,378 deletions, and 11 retained helper moves. The staged diff check, branch
provenance, resulting-path audit, and direct-call audit pass. This checkpoint
is ready for the requested `quintic-only` commit.
