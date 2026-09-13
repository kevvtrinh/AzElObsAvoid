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
| Dense dynamic earliest detour | `timeExpandedVisibilityGraph`, `bmtpTimeCellsDegree5`; `testTimeScopedPlanes/testSavedMovingDetourEarliestArrival` | Necessary absolute-time geometry. Current sampled graph and uniformly scaled clock are not a final general formulation. |
| Analytic delayed direct chord | `c3DepartureSchedule`; `exampleMovingBarrierWait`, `exampleOpeningUShapedObstacle`, and arrival-search regressions | Exact for this single route, but currently reached as a fallback and selected with an invalid initial-snapshot bound. It belongs as an exact timed edge, not a competing planner. |
| Chronological fixed clocks | `TemporalSearch`, `FixedArrivalTrialTime_s`; `testPlannerDecisionFlow/testEarliestMovingTargetUsesChronologicalClock` and `exampleMovingCircleNoWrap` | Target position changes with time and the sparse fixed-goal timed clock can miss a faster detour. The recursive full planner invocation is implementation duplication, not the desired common solve. |

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
- Public decision fixtures now also cover nonfinite time, scalar-limit
  normalization, empty known-option defaults, noninteger arrival trials,
  WrapY and dual-axis wrap, target-driven periodic rejection, equal explicit
  and matched target derivatives, acceleration-only matching, and outward
  boundary acceleration.
- Exact-row fixtures compare every materialized plane-row residual with the
  omission oracle, prove an omitted conflicting row is loaded through another
  solve, and prove a retained row cannot hide a different violated pair.
- Timed proposal acceptance uses one BMTP call. The former second wait-guide
  retry was unreachable because the same selected-window condition had already
  selected that guide before the first solve.
- A lossless complete-polynomial edge adapter now carries absolute segment
  times, Bernstein controls, normalized power coefficients, and authoritative
  physical position/velocity/acceleration/jerk endpoint states into
  `createWarmStart`. Existing route callers are unchanged. Split/rejoin trials
  on barrier, opening-U, and moving-circle preserve the complete motion within
  `7.39e-14`, retain shared jets within `4.07e-12`, change arrival and length
  only at roundoff, and pass fresh exact certificates and public validation.
  This establishes the required graph-to-BMTP data contract; it does not yet
  make route search discover those reachable jet labels.

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
The follow-up degree-eight Hermite formulation makes endpoints and C3 exact by
construction and rebuilds absolute-time coverage before validation. A timed
graph barrier seed remains valid but stays near 10.5 seconds versus the
10.1400889188-second reference, outside the one-percent arrival gate; the
opening-U timing step retains only its unchanged 15-second feasible seed versus
the 11.6133888606-second reference. It is therefore also not productionized.

The continuous-edge follow-up moves the exact delayed-chord calculation into an
all-pairs graph and joins edges at zero position derivatives through jerk. It
recovers independently valid barrier and opening-U motions at 10.1400889 and
11.6133887 seconds, respectively. That does not generalize: on the moving-circle
fixture the graph checks 4,753 edges in 97.984 seconds and returns a valid
11.1193767-second motion versus the existing 9-second BMTP result. Requiring a
complete stop at each visibility vertex loses 23.55% arrival quality, so this
formulation is rejected rather than added as another route policy.

The invalid initial-snapshot skip also cannot simply be deleted. Directly
challenging the moving-barrier incumbent with the existing chronological search
tries six fixed clocks from 7.5 through 10 seconds, finds no certified motion,
retains 10.1400889 seconds, and adds 156.079 seconds. The opening-U replay was
stopped after several minutes. A replacement must jointly preserve through-knot
velocity, acceleration, and jerk and optimize independent absolute segment
times; repeated full planner calls fail the runtime gate.

Removing only the sparse-history gate is also insufficient. A production-exact
copy without that one predicate returns a valid moving-circle timed motion at
9.4416530 seconds, while the chronological production path reaches 9 seconds.
The degree-eight shared-knot, independent-duration sandbox improves the same
timed seed to a valid 9.4015543 seconds but stalls there even with a larger local
SQP budget. These are 4.91% and 4.46% arrival regressions, respectively. The gate
cannot be removed until the common joint timing/control solve reaches the same
physical solution rather than merely exposing the current shared-clock basin.

A dense-grid sandbox adds every declared temporal-resolution sample to the timed
graph. It finds a fully validated 8.6915415-second moving-circle motion, proving
the production 9-second chronological result is grid-limited rather than globally
earliest. The same change fails the two direct-wait regressions: the barrier
warm start turns unequal waits and motion into fixed ratios, inflates to
56.6853 seconds, and cannot initialize two exact pairs; opening-U expands to
245 layers, spends 17.094 seconds, and also returns `timedMotionInfeasible`.
Denser sampling therefore moves one answer while preserving the shared-clock
defect and violating the runtime gate. It is not productionized.

The remembered `987e594` kinematic-clock implementation passes its own six
clock-guide tests and the complete historical 50-test suite, but it does not
solve the current problem. The current Rogue request returns
`timeWindowInfeasible` after 20.74 seconds, and the historical opening-U example
returns `noOptimizedFeasibleIterate`. The historical moving-barrier example also
warns that it did not select its direct waiting seed. Restoring that branch
would therefore replace current exact successes with known failures.

The complete-motion handoff sandbox isolates the solver interface from route
generation. It passes the exact delayed-chord control net, every physical
segment duration, and its normalized power polynomial into the current
time-scoped solver instead of letting `createWarmStart` reconstruct a linearly
interpolated route. On the moving barrier the complete seed validates at
10.1400888979 seconds and length 10, while the current timed optimizer loses an
exact clock pair and returns no motion. On opening-U the complete seed validates
at 11.6133886803 seconds and length 10; the optimizer returns valid but slower
20.4441770- and 14.9580-second motions before colliding at 9.8678 seconds.
Selecting the already certified full-motion incumbent preserves both references
within numerical roundoff and passes the public independent validator, with
0.513 and 3.905 seconds spent in the existing optimizer. A common solver must
therefore accept and retain a complete physical motion. This does not prove that
the direct route dominates every detour, so it is not a production early exit.

The dense timed graph's early wait routes were then checked continuously by
solving the exact quadratic half-space inequalities for a linear trajectory
against every affine convex obstacle cell. The sampled 9-second barrier route
actually intersects the protected barrier over `[6.084999991, 6.193750013]` s;
the sampled 10-second opening-U route intersects the closing gate over
`[6.899999984, 6.999]` s. The 8.5-second moving-circle guide is continuously
clear. Thus the graph has two independent defects: sampled collision checks
admit false edges in the wait cases, and its velocity-only transition bound
admits motions that cannot satisfy acceleration and jerk.

Replacing sampling with the exact edge check and applying the exact endpoint
state lower bound to the start-to-goal transition moves the first graph clocks
to 10.5 seconds for the barrier and 12 seconds for opening-U. The exact-edge
incumbents at 10.1400889 and 11.6133887 seconds then win without the invalid
initial-snapshot length bound. A naive all-edge implementation is not retainable:
opening-U takes 25.561 seconds on the dense 245-layer graph (192.177 seconds
without the sampled rejection prefilter), versus 0.2147 seconds for production.
The input-derived sparse graph takes 0.411 and 0.645 seconds on the wait cases,
but its 104 fixed swept-envelope nodes make moving-circle search take 16.093
seconds. The next graph needs lazy exact edge certification and moving vertex
tracks; neither denser layers nor a fixed swept-envelope node union meets the
runtime gate.

The selected guide is not uniformly a collision-free object in current
production. The fixed-arrival spinning-U guide's second edge intersects the
protected rotating obstacle from 15.9221 to 15.9690 seconds; the fixed-clock
BMTP bends it into a different, independently valid motion. The current Rogue
guide is continuously clear and its final 61.3966865-second motion also passes
public validation. Therefore a lazy exact check cannot simply reject the whole
planning attempt when a guide edge fails. It must add the failed space-time pair
to route constraint generation and allow BMTP/graph reconstruction to find a
certified corridor. Final-motion validation remains authoritative.

A lazy exact graph prototype now uses the 13-point test only as a rejection
prefilter, continuously certifies the selected route, and adds the midpoint of
each proven collision interval as a constraint-generation witness. Barrier and
opening-U each require one witness; their final guides are continuously clear at
10.5 and 12 seconds. Moving-circle needs no witness and remains clear at 8.5
seconds. This removes false accepted edges without checking the full graph, but
dense opening-U still costs 21.371 seconds because the fixed swept-envelope node
set is rebuilt over 245 layers. Truncating search at the certified incumbent is
sound and reduces opening-U to 0.359 seconds, but barrier still needs two
witnesses and 2.512 seconds before proving no graph route beats 10.1400889.

Exact per-layer spatial graphs establish the next structural reduction. On the
moving circle, all 37 moving snapshots build in 0.687 seconds, each with 50
nodes and a median of 90 visible edges; the frozen union graph takes 6.114
seconds with 82 nodes. Across topology-changing workspace intersections the
visible node count is ragged: barrier varies from 2 to 6 nodes and opening-U
from 16 to 24. A general timed DAG therefore must carry prepared vertex
identities and mark unavailable states, not infer identity from snapshot graph
row numbers. It must also create stationary wait anchors lazily; merely moving
every visibility vertex would lose valid waits.

The follow-up ragged moving-boundary DAG confirms that moving nodes alone are
not the missing state. Barrier obtains a continuously clear 9-second linear
guide only by requesting an impossible rest-to-rest suffix; opening-U matches
the incumbent arrival with position knots that have no compatible C3 handoff;
moving-circle obtains a clear 8.5-second linear guide whose optimized candidate
fails the public validator. Sound forward/backward jerk/acceleration endpoint
bounds remove these false labels, but also remove the known valid routes because
the graph edge still means constant-speed interpolation.

The representation gate is now positive on three structurally distinct
motions. Splitting the highest-motion polynomial span and storing each edge's
Bernstein controls, physical duration, and shared position/velocity/
acceleration/jerk endpoint states reconstructs barrier, opening-U, and
moving-circle with maximum trajectory residuals `7.39e-14`, `7.39e-14`, and
`1.33e-15`, respectively. All rebuilt plane certificates and public validators
pass; arrival and length changes are numerical zero. The graph must therefore
search polynomial edges with reachable-jet labels. Protected-boundary vertices
are guide features, not pinned physical waypoints. Adding further scalar bounds
to `(position,time)` labels cannot solve the remaining consolidation.
