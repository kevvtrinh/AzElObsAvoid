# Plan: Compare Conic Solvers

## Decision

Compare conic backends on the BMTP trajectory and maximum-margin plane
problems. Retain a replacement only when it improves warmed end-to-end planner
time by at least 10% without changing independent trajectory validity, arrival,
motion length, diagnostics, or expected failures.

## Constraints

- Work on a new `compare-conic-solvers` branch from `bmtp-cleanup-codex`.
- Put repository-owned solver code in a dedicated MATLAB `+conicSolver`
  package.
- Keep all repository-owned benchmark and adapter code in MATLAB.
- Evaluate `coneprog`, MOSEK, Clarabel, ECOS, and public license-free MATLAB
  alternatives discovered during the work.
- Treat an unavailable dependency or license as an unavailable result, not a
  benchmark failure to hide.
- Do not expose a public planner option solely for this experiment.
- Keep `obstacleAvoidance.validateTrajectory` authoritative.

## Checklist

1. Record the exact source commit, dirty state, MATLAB release, toolboxes,
   hardware context, and initial solver availability.
2. Freeze a focused warmed `coneprog` planner baseline.
3. Create one stable conic-problem/result interface in `+conicSolver`.
4. Route both BMTP trajectory SOCPs and maximum-margin plane SOCPs through that
   interface without changing their mathematical problems.
5. Add focused tests for defaults, partial overrides, unknown fields, invalid
   problems, successful solves, and stable solver failures.
6. Benchmark trajectory-shaped and plane-shaped problems separately.
7. Warm each solver before recording repeated median, minimum, and maximum
   time.
8. Record availability, validity, termination reason, objective, constraint
   residual, problem scale, dependency, and notes in one table.
9. For `coneprog`, try `auto`, `prodchol`, `normal`, and `schur` linear solvers.
10. Run a focused public-planner comparison for any isolated winner.
11. Run one structurally different planner case only after the focused 10%
    gate passes.
12. Require unchanged collision, workspace, endpoint, velocity, acceleration,
    jerk, plane-certificate, and public result validation.
13. Keep the incumbent `coneprog` backend if no replacement clears the gate.
14. Remove unavailable, losing, or unverified adapters after the decision.
15. Update `branch_assessment.md`, `benchmark.csv`, and verification evidence
    only for the retained outcome.

## Comparison Table

Use one table with these columns:

```text
Backend, Configuration, ProblemFamily, Available, Valid,
TerminationReason, FirstRun_s, RepeatedMedian_s, Minimum_s, Maximum_s,
ObjectiveValue, MaximumConstraintViolation, VariableCount,
InequalityCount, EqualityCount, ConeCount, Dependency, License,
SourceRepository, Notes
```

Keep invalid, unavailable, timeout, and license-failure rows. Do not average
them away.

## Acceptance

Accept a replacement only when:

- every successful trajectory passes the unchanged public validator;
- expected failures retain accurate termination reasons and diagnostics;
- focused warmed end-to-end median time improves by at least 10%;
- a structurally different case confirms the benefit;
- no representative case has an unacceptable arrival, motion-quality, or
  worst-runtime regression; and
- added dependencies and production code are justified by the measured gain.

Otherwise keep `coneprog`, remove the losing adapters, rerun the frozen
baseline, and preserve the negative result in the branch assessment and
benchmark table.

## Completed Decision

The retained backend is `coneprog` with its default `auto` linear solver.
The isolated `normal` setting did not complete the focused planner gate, SCS
made the validated focused planner case about 23 times slower, and ECOS did
not produce a validated trajectory. MOSEK was unavailable because neither the
dependency nor a license was present. Clarabel is Apache-2.0 but has no
official MATLAB interface. All losing candidate adapters were removed.
