# HS3-only branch plan

## Objective

Remove the superseded corridor-quintic planner from `HS3-planner` and leave one
maintained public planner path backed by HS3.

## Completed

- Removed the complete `+azElPlannerMethods/+corridor` implementation.
- Removed corridor/quintic-only tests, benchmarks, walkthrough code, and frozen
  spline benchmark artifacts.
- Reduced `planAzElMotion` to an HS3-only dispatcher and stable HS3 result
  schema.
- Migrated examples, moving-target planning, sandbox controls, maintained
  tests, benchmark drivers, and active documentation to HS3-only ownership.
- Audited active source and documentation for removed planner symbols.
- Passed Code Analyzer on all 83 remaining MATLAB files.
- Passed 67/67 focused HS3 and maintained-contract tests.
- Passed the complete post-cutover suite, 75/75 tests.

## Verification limitation

The fresh serial example matrix was attempted after the focused suite. Each
fresh MATLAB process failed during startup, before any example code ran, with
`System Error: File system inconsistency`. A later single MATLAB process did
run the complete test suite successfully. Historical benchmark evidence remains
preserved; no example result was copied or invented for the failed executions.

## Sandbox export follow-up

- Added `sandboxState.ReadState().ExportBundle(filePath, modeName)` so exports
  can bypass the UI save dialog.
- Verify that the saved MAT file is nonempty and contains `diagnosisBundle`.
- Surface UI export errors in a modal dialog.
- Focused export verification passed 4/4 with zero Code Analyzer messages.
- Allow a scene and its current controls to be exported before planning, with
  explicit `PlanningState = "notRun"` and no fabricated planner result.
- Focused pre-run/completed/failed export verification passed 5/5 with zero
  Code Analyzer messages.
