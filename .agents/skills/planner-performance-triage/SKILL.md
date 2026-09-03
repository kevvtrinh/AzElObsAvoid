---
name: planner-performance-triage
description: Triage Az/El planner runs that stall, consume excessive time, or return suspicious noValidatedSeed failures using measured repo-specific cost centres, invariants, and regression sentinels. Use planner-failure-diagnosis instead for general stage-by-stage root-cause isolation.
---

# Planner Performance Triage

Use this companion only after runtime or a known false-negative signature points
to a concrete planner path. For unexplained failures, start with
`planner-failure-diagnosis`; do not duplicate its stage-tracing procedure here.

## Bound And Attribute The Run

Do not wait indefinitely. `planTrajectory` calls `CancellationCheckFcn` at
stage boundaries. Bound a run with a deadline closure:

```matlab
runTimer = tic;
options.CancellationCheckFcn = @() toc(runTimer) >= deadline_s;
```

Catch `planTrajectory:UserCancelled`. A callback cannot interrupt one active
solver or vectorized geometry call, so record `dbstack` and callback times when
an atomic cost must be localized. Attribute work from returned `StageTiming`,
`SearchDiagnostics`, and per-seed `SolverDiagnostics`; do not infer the hot
stage from total wall time or create a persistent diagnostic wrapper.

## Check Known Cost Centres In This Order

| Cost centre | Measured signature | Consequence |
| --- | --- | --- |
| Timed visibility edges | Cost scales with input-derived layers, reached nodes, candidate target layers, and 13 collision samples per edge. | Inspect `MotionEdgeCount`, `WaitEdgeCount`, and temporal state counts before blaming motion solving. |
| BMTP motion solving | Cost scales with optimizer spans, active curve-region pairs, outer iterations, and trajectory/plane SOCP counts. | A 35-iteration run can retain a valid incumbent; inspect `TrialWasCollisionFree` and `RetainedBestTrialDuration_s` before calling the seed a failure. |
| `StageTiming` attribution | Exact constructors and timed motion stages can call `validateTrajectory` internally. | Keep nested validation out of `MotionSolvingElapsedTime_s`; do not count the same wall interval twice. |

## Audit `noValidatedSeed` As A Bounded Result

`noValidatedSeed` means only that no attempted motion passed independent
validation. It is not proof that no physical path exists. Locate the earliest
bounded decision that prevented a useful candidate:

1. Confirm that direct, timed, and spatial seeds were retained.
   `EstimatedDuration_s` is advisory: it must not discard a spatial seed or
   shorten the motion horizon.
2. For timed search, retain every input-derived layer. The componentwise
   velocity bound may exclude physically impossible early arrivals, but a
   rest-to-rest acceleration estimate is invalid at through-moving nodes.
3. Do not discard later arrivals merely because the first velocity-feasible
   traversal collides. An earlier arrival dominates a later one only within a
   target-layer interval joined by verified clear stationary waits.
4. A cheap conservative representation may run first, but its failure must not
   be final. Dense-envelope timed search, coarse timed cells, grouped static
   regions, and deferred multi-winding routes all have existing recovery paths;
   reuse their prior work instead of rebuilding the stage.
5. Before the first collision-free static BMTP iterate, an infeasible
   trajectory SOCP may expand only to the finite warm duration. Final dilation
   and validation must still enforce the requested horizon.

Finite route-class, visibility, timed-cell, timed-edge, polynomial, segment,
and solver budgets remain explicit completeness limits. Change one only with a
counterexample, a structurally different sentinel, and a measured runtime
bound; never relabel it as a proof of infeasibility.

## Stop On Regression Sentinel Movement

Do not judge or bless a change that moves any sentinel. Stop and report it.

| Sentinel | Required value |
| --- | --- |
| `Rogue Examples/failed.mat` | success and independent validation; polyline 143.92829584254 deg, smooth length 145.143797542061 deg, arrival 71.2828117654205 s |
| `exampleTwoOpposingUVisibilityGraph` | arrival 21.6333333333333 s |
| `exampleStaticUShapedObstacle` | polyline 34.9425880404659 deg, smooth length 39.1412774270613 deg, arrival 20.7865397074203 s |
| README quick-start (`exampleObstacleAvoidance`) | polyline 11.1521195190242 deg, smooth length 11.4116854105306 deg, arrival 7.52917416639509 s |
| `exampleMovingBarrierWait` | arrival 10.0903015136719 s |
| `exampleMovingCircleNoAzimuthWrap` | arrival 8.5 s |
| `exampleMovingDeformingUSOutlineVisibility` | arrival 7.91666666666667 s |
| Full suite | Zero failed or incomplete tests, including the exact saved bundle |
| `exampleNoPath` | Independently validated expected failure; do not manufacture success |

Treat `exampleNoPath` as the negative control for every recovery. Whenever a
maintained example runs, report every required metric in the chat as required
by `AGENTS.md`; an artifact or benchmark row is not a substitute.

## Use Cross-Branch Evidence Carefully

`benchmark.csv` records other revisions. A row is a record for its revision
only, not reproducible evidence. Read deleted precedents without checkout:

```text
git show HS3-planner:<path>
```

The `HS3-planner` reachability-frontier
`timeExpandedVisibilitySearch` is a useful historical precedent because it
expands only reached nodes and stops on `reachable(layerIndex, 2)` for earliest
arrival. Its `+search/boundedTimeLayers.m` selector is a negative precedent:
thinning supplied obstacle times can erase a brief feasible opening and must
not authorize `noValidatedSeed`.

Testing only the first velocity-feasible arrival layer is also a negative
precedent. A later traversal can be collision-free even when the source cannot
wait. Discard later arrivals only after a clear earlier arrival and clear
target waits establish state-and-cost dominance.

## Never Do These Things

- Never weaken `obstacleAvoidance.validateTrajectory`, a tolerance, or an
  acceptance criterion. It is the sole authority.
- Never add a scenario detector. Orthogonal-cavity and timed-opening detectors
  were removed because they encoded rectilinear benchmark structure rather
  than a general planning invariant.
- Never restore the HS3 engine tree. `tests/testArchitectureBoundaries.m`
  enforces `testDeletedHs3EngineTreeRemainsAbsent`.
- Never let `trajectory/+bmtpEngine` reference `obstacleAvoidance.*`.
- Never add a wrapper, option, fallback, or diagnostic field when an existing
  owner can express the invariant directly.
- Never retain neutral cleanup or speculative optimization. Revert it when the
  declared correctness, quality, size, or runtime benefit is not demonstrated.

Prefer deletion and one-source-of-truth changes. Run focused tests after each
coherent edit and the full suite plus maintained example matrix only at major
behavioral milestones.
