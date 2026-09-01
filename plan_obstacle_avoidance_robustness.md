# Plan: Obstacle-Avoidance Robustness

## Source And Scope

The supplied shared-chat snapshot exposes `plan_coneprog.md`; it does not
contain an artifact named `obstacle avoidance robustness.md`. This repository
already owns `benchmarkRandomMovingPolygonStress`, a deterministic public-
planner corpus with exact inputs, independently validated results, and an
analytic clear boundary witness. This plan uses that corpus rather than
inventing or regenerating scenarios.

## Baseline

- revision: `81a94be2224ef6dd0183848772f5fbdbce81b85f` on
  `compare-conic-solvers`;
- initial dirty state: clean;
- MATLAB: R2024b Update 4;
- fixed random seeds: `1001:1012`;
- public planner: `obstacleAvoidance.planTrajectory` with resolved defaults
  plus `GoalTimeMode = "earliestArrival"`;
- motivating historical failure: seed 1011 returned `noValidatedSeed` even
  though its analytic boundary witness had 2.40165847462 degrees clearance;
- fresh focused result: seed 1011 succeeded, independently validated, and
  arrived at 17.9833348954 seconds in 26.526273 seconds wall time.

## Fixed Gate

Retain a planner change only if all of the following hold:

- every changed success passes `obstacleAvoidance.validateTrajectory`;
- no clear-witness corpus case changes from valid success to failure;
- a fixed failing seed, if one exists, becomes an independently validated
  success without changing its inputs or random seed;
- the fix implements an input-driven invariant at the earliest broken stage;
- a structurally different moving-obstacle case passes the same invariant;
- the expected `exampleNoPath` result remains a validated failure with a
  diagnostic figure;
- established maintained-example physical metrics remain within their existing
  tolerances;
- static checks, focused tests, the full suite, all maintained headless
  examples, and one visible success pass.

Any candidate is removed if it needs scenario recognition, a hidden waypoint,
a preferred detour direction, weaker collision or kinematic validation, a
larger tolerance, regenerated geometry, or an unreported fallback. A fully
passing baseline is evidence that no defect was reproduced, not permission to
alter planner behavior speculatively.

## Workflow

1. Run seed 1011 alone and preserve its request, resolved options, result,
   search diagnostics, wall time, arrival, and witness clearance.
2. Run seeds `1001:1012` serially and retain every success and failure record.
3. If a case fails despite its analytic witness, classify the earliest broken
   stage from returned diagnostics before editing.
4. State the violated general invariant and one structurally different check.
5. Evaluate one minimal candidate at a time under the fixed gate.
6. Remove a failed candidate completely and record the negative result.
7. Run the required repository-wide verification only after the focused and
   structural gates pass.
8. Update this plan, `branch_assessment.md`, `benchmark.csv`, and
   `verification.md` with measured results; do not invent benchmark rows.

## Status

Completed without a planner change. The historical seed-1011 failure is not
present at the pushed baseline, and all twelve fixed moving-polygon cases
returned `goalReached` with independent validation passed. Corpus wall time
was 509.2953517 seconds; minimum, median, and maximum per-case times were
21.0016595, 34.32322055, and 82.4723368 seconds. Seed 1011 arrived at the same
17.9833348954 seconds in its focused and corpus runs.

No failing clear-witness case remained to diagnose, so steps 3 through 7 were
not triggered. Production and tests remain unchanged. `benchmark.csv` was not
modified because its stable schema is explicitly one row per maintained
example and motion-constraint mode; inventing an example-shaped row for this
stress benchmark would violate that record.
