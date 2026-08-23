# Active compact instrumentation checkpoint

## Objective

Add consistent topology, corridor-construction, motion-solving,
collision-checking, and final-validation timings to both planners; consolidate
fixed-duration affine construction; preserve planner behavior and independently
validated results.

## Completed

- Rejected the 1,706-line instrumentation prototype and removed its public HS3
  attempt ledger, solver micro-timers, activity trees, duplicate counters, and
  setup/selection stages.
- Consolidated the timing helpers and canonical obstacle/query infrastructure;
  production now has 494 fewer physical lines than `27070ac` with the same
  76-file count.
- Both planners now publish the same seven-field record: the five requested
  stages plus `UnattributedElapsedTime_s` and `TotalElapsedTime_s`.
- Validators retain only collision and total elapsed scalars internally. Existing
  `SeedSummaries` remain the candidate-level diagnostic record.
- Consolidated fixed-duration affine construction across the direct, Compact C3,
  exact, and dynamic-repair paths. Retained the generalized coordinate count,
  zero-decision route, and positive-span fallback fixes.
- Focused affine/timing/motion tests passed 23/23 on the final source. The final
  maintained test suite passed 127/127 in 174.7697065 seconds.
- MATLAB Code Analyzer reported zero findings across 93 maintained production
  and test files. `git diff --check` is clean apart from line-ending warnings.
- Audited `+azElInternal`: all six files are still used by root public utilities,
  plotting, or maintained examples, so deletion would break active callers.
- Both methods passed all 18 maintained examples in fresh processes; each had
  17 successes and one expected validated `noValidatedSeed` result.
- The visible corridor graphics smoke passed with two rendered figures.
- The 48-process alternating frozen A/B preserved every contract, selected seed,
  and physical result within `1e-6`; the largest numeric difference was
  `1.3056e-13`.
- Timing medians were mixed. Corridor no-path planner time was an unfavorable
  +14.443%; it remains recorded as a regression signal.
- Current evidence and the 48 raw A/B rows are recorded in `verification.md`,
  `branch_assessment.md`, and `benchmark.csv`.

## Handoff

- Removed the task-owned baseline snapshot, archive, harnesses, and temporary
  A/B files after preserving their results in repository evidence.
- Final static, CSV-integrity, whitespace, and Git-diff checks passed. The
  unrelated untracked `docs/` directory is preserved.
- No files are staged. Do not commit or push without a new explicit request.

## Current risks and limits

- The complete change is 494 production lines smaller than the frozen baseline.
- The maintained tree is 20,348 physical lines and remains above the former
  12,000-line cap; the branch was already above that cap at `27070ac`.
- Pre-existing topology generators remain above 900 lines; this change does not
  claim to resolve those unrelated size violations.
- `+azElInternal` is a public-shell implementation package, not a dead planner
  backend. Renaming its ownership is possible but is a separate refactor and
  would not reduce code by itself.
