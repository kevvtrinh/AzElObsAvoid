# Plan 325 corridor-only completion checkpoint

## Objective

Complete `325-less-nlp` as a general deterministic corridor-quintic planner
without restoring HS3/NLP. Preserve independent continuous collision and
kinematic validation, stable diagnostics, useful expected failure, maintained
examples, and honest limits. Improve arrival and runtime through input-driven
geometry and control theory rather than scenario-specific tuning.

## Current Design

1. Generate deterministic topology from obstacle, state, limit, and option
   inputs; expand visibility only under the existing bounded-work policy.
2. Build a protected-clear route and retry once with the full route if safe
   compression makes the affine corridor infeasible.
3. Construct C3 corridor quintics whose complete spans satisfy Bernstein
   half-space constraints.
4. Allocate span time with measured velocity, acceleration, and jerk demand.
5. For small static earliest-arrival systems with zero endpoint derivatives,
   minimize a common continuous derivative scale with bounded convex bisection and
   exact polynomial-extrema exchange. Compare unit and damped first responses
   once, retain the shorter validated response, then use bounded secant
   feedback; retain the fixed controller elsewhere.
6. For sufficiently resolved dynamic routes, use bounded minimum-jerk
   quadratic updates inside time-local protected safe-side barriers. Backtrack
   every update and retain it only after full independent validation.
7. For one validated compact topology with three through ten vertices, search
   an eight-span C3 minimum-jerk spline with fixed-duration convex QPs. Batch
   exact point-to-edge projection for stationary histories; moving histories
   retain the 256-vertex work cap. Probe near the physical duration lower
   bound: feasibility selects lower-bound bisection, while infeasibility selects
   the established high-to-low continuation, within eight total trials. Admit
   a zero-length seed only after bounded hold recovery; retain only a strictly
   shorter independently valid result.
8. For direct rest-to-rest fixed-arrival motion, uniformly stretch the exact
   jerk-switching S-curve; derivative bounds decrease by the corresponding
   first/second/third powers and infeasible short requests still fail validation.
9. Independently validate original protected geometry, continuous collision,
   workspace, endpoints, and exact polynomial kinematic extrema.
10. Return one stable success/failure schema with method provenance; HS3-only
   compatibility fields have been removed.

The exact-exchange eligibility is input-driven and capped at 100
decision-span work units. Production does not branch on example names,
obstacle names, expected routes, or hidden waypoints.

## Current Evidence — 2026-08-22

- U acceptance gate: `22.640860106984 s`, beating frozen main; final
  fresh-process wall `5.5963246 s`; independent collision and kinematic
  validation passed with `0.000021707112 deg` clearance.
- 12-wall hairpin: exact `164.828287993152 s` motion, `9.4377117 s`
  candidate was superseded by `8.0691334 s` candidate and `9.3474237 s`
  total wall; `0.02 deg` clearance, corridor
  certificate and independent validation passed.
- Alternating slalom retains `10.855664258356 s` with `4.4654387 s` wall.
  Opposing U now reaches `22.160945761398 s` versus frozen main
  `22.875124576026 s`, with `8.5208632 s` wall and independently validated
  `0.0143650783404 deg` clearance.
- A bounded 200-work exact-exchange experiment did not improve slalom and
  increased wall time. It was fully reverted.
- The final 18-example matrix ran headlessly and serially in fresh processes:
  17 independently valid successes and one independently valid expected
  no-path result. Current production contains no HS3 execution or HS3-only
  result fields.
- The compact C3 controller improved forty moving circles from
  `105.698249573983 s` to `62.477739862636 s`, beating the frozen
  `64.555779916429 s` reference. Its fresh-process wall was `16.3138462 s`
  and protected clearance was `0.001843121779 deg`.
- The same controller improved the moving-circle case from
  `13.619220197654 s` to `8.751228736151 s`, beating the frozen benchmark;
  fresh-process wall was `6.6309076 s`. Independent continuous validation
  passed with `0.001266077441 deg` clearance.
- Extending the identical eligibility rule to static geometry improved basic
  planning to `7.649656344043 s` and dense concavity to
  `8.703796600286 s`; both now beat their frozen main rows. Their walls were
  `5.2756101 s` and `5.4895786 s`.
- Reshaping a recovered direct-wait seed improved moving barrier to
  `10.371387562474 s`, beating main. Uniformly scaled exact S-curves improved
  earliest moving-target intercept to `6.111534301758 s`, also beating main,
  while every maintained fixed-arrival regression passed.
- Batched exact stationary projection now lets the compact controller process
  full dense geometry. Extreme outline reaches `6.222166624146 s` versus
  frozen main `6.684968340018 s`, with `29.0349372 s` wall and independently
  validated `0.00709851977447 deg` clearance.
- The final 18-row fresh-process wall sum is `167.0127367 s`, a `33.74%`
  reduction from the `252.0683835 s` optimized-main matrix and an `18.27%`
  reduction from the prior `204.3536736 s` corridor-only matrix. All 17
  success durations meet or beat the frozen optimized-main row; fixed-arrival
  equality differs by at most `3.0e-13 s` roundoff.
- Profiling attributed `60.4469 s` of a `63.6347 s` dense run to two dense
  interior-point QPs. LP feasibility plus active-set QP reduced the final
  isolated wall to `3.7330015 s` with bit-identical `12.140801107795 s`
  duration. The complete 18-example wall sum decreased `28.64%`, though
  per-example timing noise prevents a uniform speedup claim.
- The moving/deforming short seed is still available at `63.084805 deg`, but
  its fixed-point retime alone reached only `17.850546 s`. The retained
  minimum-jerk safe-side controller now reaches a fresh-process,
  independently valid `12.873502939647 s`, beating the frozen
  `12.986386910606 s` path-time reference by `0.112884 s`. Its no-plot wall
  is unfavorable at `59.3688435 s`, so no runtime speedup is claimed.
- The current complete suite passes 56/56 in `28.7732939 s`; Code Analyzer
  reports zero messages across 66 nonscratch MATLAB files.
- Visible success produced three figures and 529 graphics objects. Expected
  no-path produced two diagnostic figures, 343 objects, and two rejected
  transitions.
- Core production is 7,498 literal physical lines excluding 565
  plotting lines and meets the user-authorized conditional ceiling.
  Maintained MATLAB excluding examples/scratch is 10,296 lines and passes
  the 12,000-line cap. Every production file is at most 881 lines.
- No commit or push has been requested. The interactive sandbox remains
  untested.

Full commands, the 18-row matrix, and historical evidence are retained in
`verification.md`, `branch_assessment.md`, and `benchmark.csv`.

## Residual Opportunities And Caveats

1. The requested acceptance gates are met. Further work may reduce the
   moving/deforming U.S. wall outlier without losing its independently
   validated `0.112883970959 s` path-time improvement.
2. Wall speedup is aggregate, not uniform. Forty moving circles,
   moving/deforming U.S., and target-exits-obstacle remain slower than optimized
   main in isolated wall time and are recorded explicitly.
3. Rapid consecutive MATLAB launches can still trigger a MathWorks Service Host
   file-system error. A controlled host restart plus ten-second settle produced
   the complete 18-row fresh-process proof; this is an environment workaround,
   not planner behavior.
4. Keep core production at or below the conditional 7,500-line ceiling and
   preserve the current benchmark evidence if additional optimization is tried.

## Completion Gates

- every maintained noninteractive example uses `corridorQuintic` with zero
  HS3 attempts;
- successes pass independent continuous collision and kinematic validation;
- expected failure returns a stable result and useful plots without rerunning;
- static/moving, earliest/fixed, wrapping, endpoint derivatives, and jerk
  limits remain covered;
- retained retiming changes have verified benefit without a confirmed
  correctness or runtime regression elsewhere;
- production, per-file, and maintained-tree size limits pass;
- no legacy execution path or scenario-specific behavior remains;
- limitations are stated without global optimality or completeness claims.

## Rejected Directions

Do not repeat without a new mechanism and declared proof gate:

- scalar clearance weights or ranking-only tuning;
- uniform route reduction through protected geometry;
- unconstrained offset coordinate search;
- sampled derivative penalties or geometric jerk QPs that failed to improve
  the certified result;
- universal unit-gain or error-magnitude gain schedules: unit gain improved U
  but regressed planning/slalom, while a two-band schedule entered a worse U
  controller basin. The switched geometry solve needs a local Jacobian or
  trust-region acceptance mechanism, not another scalar gain formula;
- broad topology expansion outside bounded work;
- an exact-exchange cap above 100 based only on hope of wider applicability;
- fixed nonuniform C3 knot allocation from velocity/acceleration/jerk demand:
  opposing U stayed at `27.5872457615 s` and wall rose to `29.1646229 s`;
- moving compact C3 after the existing timing controller: it validated at
  `26.6497104564 s` but still missed main and raised wall to `27.9170964 s`;
- restoration of HS3/NLP as a hidden fallback.
