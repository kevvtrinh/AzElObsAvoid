# Planner decision-flow ledger

This file describes the live `build-core` working tree. It is a coverage
ledger, not a claim that the remaining fallback tree is acceptable.

## Intended core

```text
normalize one public request
  -> prepare exact protected geometry once
  -> reject invalid, unsupported, or proven-infeasible inputs
  -> construct the exhaustive visibility graph appropriate to the clock
  -> solve one BMTP motion on that route and clock
  -> continuously certify the polynomial
  -> assemble once
  -> run the public independent validator
```

Static geometry is the zero-motion specialization. Fixed arrival fixes the
clock; earliest arrival optimizes it. Those are necessary mathematical
distinctions, not alternative planner policies.

## Public entry decisions

| Decision | Current outcome | Fixture | Judgment |
|---|---|---|---|
| Omitted public inputs | Deterministic default static detour | `testPlanningCore/testDetourAndTampering` | Necessary input handling. |
| Nonfinite state or invalid time order | Public error | `testPlanningCore/testInvalidInputs` | Necessary input handling. Other validation errors remain to be enumerated below. |
| Position goal versus sampled moving target | Target is evaluated on the selected physical clock | `testPlannerDecisionFlow/testFixedMovingTargetMatchesPchipDerivatives`, `testEarliestMovingTargetUsesChronologicalClock` | Necessary semantics. |
| Explicit versus matched target velocity/acceleration | PCHIP derivatives become terminal state | `testPlannerDecisionFlow/testFixedMovingTargetMatchesPchipDerivatives` | Necessary semantics; now covered. |
| Periodic coordinate request | Nearest image, obstacle-free fixed goal only | `testPlannerDecisionFlow/testPeriodicWrapUsesNearestImage`, `testPeriodicRequestWithObstacleIsRejected` | Necessary exposed semantics; unsupported combinations are explicit. |
| Unsupported continuous obstacle correspondence | Stable `unsupportedObstacleInterpolation` result | `testBoundedCorrespondence/testUnsupportedGeometryReturnsStablePlannerOutcome` | Necessary honest rejection. Never replace it with an endpoint convex hull. |
| Initial endpoint occupied | `endpointBlocked` | `testPlannerDecisionFlow/testInitialEndpointBlocked` | Necessary exact rejection. |
| Terminal reachable set covered by one obstacle | `terminalReachabilityBlocked` | `testPlannerDecisionFlow/testTerminalReachabilityBlocked` | Necessary sufficient certificate. Failure to prove it must continue planning. |
| Endpoint derivative exceeds a declared limit | `dynamicEndpointInfeasible` | `testPlannerDecisionFlow/testEndpointDerivativeLimit` | Necessary rejection. |
| Endpoint outside workspace | `endpointOutsideWorkspace` | `testPlannerDecisionFlow/testEndpointOutsideWorkspace` | Necessary rejection. |
| Fixed clock below the kinematic lower bound | `timeWindowInfeasible` | `testPlannerDecisionFlow/testTimeWindowInfeasible` | Necessary rejection. |

## Active success paths

| Path | Branch signature and fixture | Judgment |
|---|---|---|
| Fixed direct quintic | `initialSpatialSnapshot`, `minimumJerkQuintic`; `testPlanningCore/testDirect` | A proven minimum-jerk shortcut when its complete motion certifies. The rest/non-rest formulas return the same polynomial class but the attempted unification was 1.17% slower and was rejected. |
| Earliest direct C3 chord | `initialSpatialSnapshot`, `c3JerkLimitedChord`, `analyticC3Clock`; `testPlannerDecisionFlow/testEarliestStaticDirectUsesAnalyticClock` | A directly certifiable C3 construction. Its smoothed duration is not a proof of globally earliest kinematic arrival. |
| Static fixed detour | exhaustive spatial graph and `bmtpStaticDegree5`; `testPlanningCore/testDetourAndTampering` | Necessary fixed-clock specialization. |
| Static earliest detour | exhaustive spatial graph and `bmtpStaticDegree8`; `testStaticActivePairBmtp/testSeparatedSlalomBarriers` | The clock is semantically necessary. The separate active-pair implementation remains a numerical split to consolidate only after an equivalent common formulation is measured. |
| Fixed time-expanded detour | `timeExpandedVisibilityGraph`, `bmtpTimeCellsDegree8`; `testFixedTimedVisibility/testSavedDetourUsesPrescribedDeadline` | Necessary absolute-time geometry; fixed-clock and variable-clock implementations can share more machinery. |
| Dense dynamic earliest detour | `timeExpandedVisibilityGraph`, `bmtpTimeCellsDegree5`; `testTimeScopedPlanes/testSavedMovingDetourEarliestArrival` and `exampleMovingCircleNoWrap` | Necessary absolute-time geometry. Current sampled graph and uniformly scaled clock are not a final general formulation. |
| Analytic delayed direct chord | `c3DepartureSchedule`; `exampleMovingBarrierWait`, `exampleOpeningUShapedObstacle`, and arrival-search regressions | Exact for this single route, but currently reached as a fallback and selected with an invalid initial-snapshot bound. It belongs as an exact timed edge, not a competing planner. |
| Moving-target chronological fixed clocks | `TemporalSearch`, `FixedArrivalTrialTime_s`; `testPlannerDecisionFlow/testEarliestMovingTargetUsesChronologicalClock` | Target position changes with time, so clock search is necessary. The recursive full planner invocation is implementation duplication. |

## Remaining policy splits that block completion

These branches are active and must not be described as consolidated:

1. `tryTimedArrival` changes algorithms when source history has fewer than 16
   intervals. Geometry sampling density is not a physical distinction.
2. Earliest dynamic planning currently tries timed visibility, then an analytic
   delayed chord, then a chronological fixed-arrival schedule. This is a
   fallback ladder rather than one declared objective.
3. A delayed chord can skip chronological trials using initial spatial route
   length divided by velocity. That is not a valid lower bound for a moving
   scene; an initially disconnected scene produces `Inf` even when it opens.
4. Initial goal occupancy or a disconnected initial dynamic snapshot silently
   replaces the graph route with a direct chord seed. The corresponding fixtures
   are `testInitiallyOccupiedFutureGoalUsesTemporalSeed` and
   `testDisconnectedDynamicSnapshotUsesTemporalSeed`.
5. Timed proposal construction uses selected snapshot unions, an offset retry,
   a work-budget node cap, Delaunay-first pairs, and sampled edge checks. These
   are proposal heuristics, not an exhaustive exact visibility graph.
6. Timed search commits to the first goal window. Its selected free window is
   solved once from the wait guide; the provably unreachable second wait-guide
   retry has been removed.
7. Static active-pair, variable-clock timed, and fixed-clock all-pair BMTP use
   different degree, mesh, slack, plane-update, and refinement policies. Clock
   treatment differs physically; the remaining solver-policy differences need
   equivalence benchmarks before unification.

## Manual sparse-clock diagnosis

Removing the 16-interval gate without fixing the clock is not a solution.

| Case | Current valid result | Timed proposal | Exact failure |
|---|---:|---:|---|
| Moving barrier | arrival `10.1400889188`, length `10` | waits at 0, 1.5, 3; reaches goal at 10.5 | Uniform shrink reaches 9.7976450662 and collides in segment 7 with cells 1 and 2; signed gaps are -0.1504971383 and -0.0700964673. |
| Opening U | arrival `11.6133888606`, length `10` | waits to 3.4995; reaches goal at 15 | Uniform shrink reaches 9.3155260457 and collides in segments 4-6 with cell 5; gaps are -0.3016665328, -1.2648249097, and -0.8164084429. |

The timed guide is collision-free on its supplied clock. The failure occurs
because the solver scales every absolute wait and motion interval by one common
factor. Preserving the sampled wait is also incorrect: the barrier graph rounds
the safe departure to 3 seconds while the exact analytic departure is
2.5400888956 seconds, causing arrival 10.6000000174 and violating the 1% quality
gate. Prepending a fixed wait to the opening-U suffix also failed C3 because the
optimized suffix begins with nonzero jerk.

The required general formulation must therefore retain absolute obstacle event
times, optimize wait and motion durations, and impose zero velocity,
acceleration, and jerk at every stationary join. A sampled-clock rescale, frozen
wait prefix, retry schedule, or restored axis lock does not solve this.

## Historical kinematic-clock assessment

Commits `bfde52f` and `987e594` implemented the remembered kinematic-clock
guides. They are not suitable for restoration:

- they locked the independent-axis bang-bang lower-bound profile;
- they required monotone progress and excluded reversals and waits;
- nonlinear phases used conservative spatial enclosures;
- the bound profile was generally incompatible with required C3 jerk
  continuity; and
- `b4e80c5` removed its active use, then `e6ad57b` deleted the dead helpers.

Only the sound lower bound and alignment to exact obstacle events should carry
forward.

## Retained consolidations

- One `finalizeCandidate` copies a candidate, evaluates the actual target at
  arrival, runs independent validation, and assigns `invalidMotion` once.
- One canonical `createTimeCells` coverage record feeds timed BMTP; the duplicate
  wrapper and duplicate exact-cell recertification were removed.
- Unused monotone visibility parameters, unreachable earliest retry code,
  unused fixed-control plumbing, and an uncalled timed-line solver were removed.
- Timed travel refinement now has one reachable contract: an earliest-arrival
  variable-clock motion is length-polished at its already selected clock.
  Unreachable fixed-request handling, full-state compatibility, scalar-time
  expansion, certificate-event hooks, and unused timed retry diagnostics were
  removed.
- The fixed all-pair alternating solver now declares its fixed-clock contract;
  its unreachable non-fixed policy and the unused non-fixed lexicographic
  length branch were removed.
- Decision-flow tests now assert the branch signatures instead of only checking
  `Success`.
- Timed proposal acceptance uses one BMTP call. The former second wait-guide
  retry was unreachable because the same selected-window condition had already
  selected that guide before the first solve.

## Acceptance evidence and next gate

The first consolidated batch preserved success, termination reason, arrival,
and motion length exactly on 12 representative cases, including the saved Rogue
request. Forty-eight focused tests passed before commit `d93f006`; the expanded
working-tree branch suite also passes.

Completion still requires:

1. replace the absolute-time fallback ladder with one event-aware timed route
   and BMTP formulation;
2. cover every remaining active failure/retention branch or classify it as a
   defensive unreachable postcondition;
3. run two warmups and seven counterbalanced measurements against the frozen
   baseline, with fresh and repeated-request timing separated; and
4. require unchanged success/termination/validation and at most 1% absolute
   change in positive arrival, trajectory length, and runtime metrics.

The event-aware independent-duration sandbox in
`sandbox/eventAwareTimingTrial/` is negative evidence, not a production
replacement. Its four raw proposals all failed independent endpoint/C3
validation (two also exceeded acceleration bounds), so none was adopted.
