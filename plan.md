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
