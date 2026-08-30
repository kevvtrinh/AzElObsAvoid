---
name: planner-performance-triage
description: Triage Az/El planner runs that stall, consume excessive time, or return suspicious noValidatedSeed failures using measured repo-specific cost centres, invariants, and regression sentinels. Use planner-failure-diagnosis instead for general stage-by-stage root-cause isolation.
---

# Planner Performance Triage

Use this companion after performance or a known false-negative signature points
to a concrete planner path. For unexplained failures and general stage tracing,
use `planner-failure-diagnosis`; do not restate or bypass its methodology.

## Bound A Run And Locate The Atomic Cost

Do not wait indefinitely. `planTrajectory` calls `CancellationCheckFcn` at
stage boundaries. Bound a run with a deadline closure:

```matlab
runTimer = tic;
options.CancellationCheckFcn = @() toc(runTimer) >= deadline_s;
```

Catch `planTrajectory:UserCancelled`. To attribute wall time, call `dbstack`
inside the callback and record the stack frame and elapsed time at every
checkpoint. Differences between consecutive timestamps belong to the work
between those checkpoints. The largest gap identifies the atomic call that
cancellation cannot preempt. Rebuild this as a local diagnostic callback; do
not depend on a session scratchpad file.

Cancellation cannot interrupt MATLAB while it is inside one atomic solver or
vectorized geometry call.

## Check Known Cost Centres In This Order

| Cost centre | Measured signature | Consequence |
| --- | --- | --- |
| `+search/timeExpandedVisibilitySearch.m` edge generation | Moving-obstacle scene: 41 layers, 23,040 `edgeIsClear` calls, about 299,520 point-time collision samples, 91% of wall time, and no finish within 180 s. Cost is layers x nodes x targets x 13 samples; `edgeIsClear` always uses 13 samples regardless of edge length. | Inspect topology-search work before blaming motion solving. |
| Per-seed motion solving | Before `26050af`, one losing seed used 55.271 s of a 58.17 s README quick-start plan. | `PerSeedWorkBudgetMultiplier` bounds work only after a seed passes independent validation. It never arms when nothing validates. |
| `StageTiming` attribution | Fast-path constructors call `validateTrajectory` internally. | Their reported validation time is moved out of `MotionSolvingElapsedTime_s`. A new constructor that omits validation timing silently charges it to motion solving again. |

Prefer a work bound that does not depend on prior success. An incumbent-armed
bound does nothing on the hard scenes that need it most.

## Recognize The Critical False Negative

`TerminationReason = "noValidatedSeed"` does not establish infeasibility.
Inspect every `SeedSummaries(k).TerminationReason` first.

If every seed reports `unsupportedTimedTopology` with solve times near
0.001--0.002 s, the planner discarded the topology search. With dynamic
obstacles, `supportsStaticHorizon` is false, so seeds route to
`createTimedSeedCandidate`, which accepts only `directWait` seeds.

Measured on one moving-obstacle bundle with the identical request and
`WaypointWarmStartMode = "none"` in both legs. The warm-start path was not
involved, so this signature is not evidence for restoring deleted warm-start
code:

| Method | Result | Seed evidence |
| --- | --- | --- |
| `TrajectoryMethod = "bmtp"` | `success=0`, `noValidatedSeed` | All seeds `unsupportedTimedTopology`, about 0.001 s each. |
| `TrajectoryMethod = "ruckigWaypoint"` | `success=1`, `validation=1`, arrival 107.632292801 s, length 227.751816227 deg | Winning seeds solved in 0.027 s and 0.083 s. |

The complete run took about 144 s while motion solving took 0.027--0.083 s.
Treat essentially all cost in this signature as topology search, not solver
cost.

## Stop On Regression Sentinel Movement

Do not judge or bless a change that moves any sentinel. Stop and report it.

| Sentinel | Required value |
| --- | --- |
| `exampleTwoOpposingUVisibilityGraph` | arrival 21.6333333333333 s (`649/30`, certified physical floor) |
| `exampleStaticUShapedObstacle` | duration 20.7124477849715 s; smoothed path 40.2550285014326 deg (selected polyline 34.9425880404659 deg) |
| README quick-start (`exampleObstacleAvoidance`) | arrival 7.5745417663213 s; length 11.411861 deg. Older records quote `7.574542`; that is a rounded value, so compare at 1e-6, not tighter. |
| `exampleMovingBarrierWait` | arrival 10.5 s |
| `exampleMovingCircleNoAzimuthWrap` | arrival 8.5 s |
| `exampleMovingDeformingUSOutlineVisibility` | arrival 7.91666666666667 s |
| Full suite | 84/84 |
| `exampleNoPath` | Must remain a failure. |

Treat `exampleNoPath` as the negative control for every fallback. Making a
genuinely infeasible request succeed is worse than the bug being fixed.

## Attribute A Winning Construction Correctly

Do not read the winning construction from
`result.Seeds(result.SelectedSeedIndex).Source`. When a fast path such as the
cavity portfolio wins, `planCorridorQuintic` passes the original topology seed
to `finishFastPath`, so the source still reads `visibilityGraph`. A census
built that way reported zero cavity wins for code whose removal measurably
regressed `exampleStaticUShapedObstacle` by 0.069 s. Attribute from
`SearchDiagnostics` instead, and treat "wins no maintained example" as a
hypothesis to test by removal-and-measure, never as grounds for deletion.

## Use Cross-Branch Evidence Carefully

`benchmark.csv` records other revisions. A row is a record for its revision
only, not reproducible evidence. Read deleted precedents without checkout:

```text
git show HS3-planner:<path>
```

Useful `HS3-planner` precedents are `+search/boundedTimeLayers.m` for bounded
layer selection and its reachability-frontier
`timeExpandedVisibilitySearch`, which expands only reached nodes and stops on
`reachable(layerIndex, 2)` for earliest arrival.

## Never Do These Things

- Never weaken `obstacleAvoidance.validateTrajectory`, a tolerance, or an
  acceptance criterion. It is the sole authority.
- Never add another scenario detector. The branch already has about 2,523
  lines: `createFixedClockLateralExcursion` 882,
  `createOrthogonalCavityMotion` 489,
  `createTimedOrthogonalOpeningMotion` 271, plus three certifiers.
  `detectCavity` requires axis-parallel polygon edges; it is a rectilinear
  detector, not a general method.
- Never restore the HS3 engine tree. `tests/testArchitectureBoundaries.m`
  enforces `testDeletedHs3EngineTreeRemainsAbsent`.
- Never let `trajectory/+bmtpEngine` reference `obstacleAvoidance.*`.
- Never present an incumbent-dependent work bound as protection for scenes
  where no candidate validates.

## Consolidate Known Divergence When In Scope

- `roundoffReserve_deg` appears in four places with two unequal formulas:
  `2^20 * eps(scale)` and `2^20 * eps * scale`.
  `createOrthogonalCavityMotion.m` uses both; authoritative
  `validateTrajectory.m` uses the second. A constructor margin can therefore
  be narrower than the verifier requires.
- The predicate "every obstacle is static and active over the horizon" is
  implemented five times with divergent semantics. One copy lacks an
  `~isempty` guard and errors on empty `time_s`; another safely returns false.
- The seed struct literal appears four times. `certificateTemplate` and
  `candidateTemplate` each exist as local functions in two files.

Consolidate only when the task owns that invariant. Preserve behavior and use
the authoritative validator's semantics; do not mix cleanup into a focused
performance change without measured benefit.
