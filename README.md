# MATLAB planning core

Development baseline: `8cd15bb`. The Vietnam slew milestone passes its
benchmarks. The full example-suite goal is **not complete**.

```matlab
addpath(pwd, fullfile(pwd, 'trajectory'));
result = planner(obstacles, initialState, goalState, limits, options);
validation = obstacleAvoidance.validateTrajectory(result);
obstacleAvoidance.plotting.plotTrajectory(result);
```

Calling `planner()` runs a fixed-arrival static detour. MATLAB R2024b and
Optimization Toolbox are the behavioral reference.

The core separates protected obstacle preparation, exact implicit visibility
search, BMTP optimization, independent polynomial validation, and plotting.
Obstacle margins are applied once to retained originals. Concave static
regions are triangulated and adjacent faces are merged only when their union
is convex. This removes internal diagonals without changing occupied geometry.
Identical static source boundaries share one vectorized occupancy query over
their original activity interval. Visibility search operates
on the occupied union and evaluates graph edges as A* expands nodes, preserving
the Euclidean shortest-route objective. BMTP uses nonuniform segment durations
and retains continuous position, velocity, and acceleration. Jerk may jump at
joins, with both one-sided values subject to the same jerk bounds. Fixed-time
dynamic motion uses convex regions with absolute activity intervals. Spline
knots align with source events, and one elastic convex problem minimizes
control-polygon length at the actual requested clock. Supporting axes provide
separating planes analytically. Acceptance requires independent reconstruction
of source geometry, active pairs, clearance, endpoints, and polynomial limits.

Goals may supply `goalState.targetMotion` with sampled `time_s`,
N-by-2 `position_units`, and `InterpolationMethod` (`linear` or `pchip`). The
validator evaluates that source again at actual arrival. A visible direct
fixed-time request uses the exact minimum-jerk quintic when its motion and
collision limits certify, avoiding an optimization call and preserving the
straight path. Exact de Casteljau restriction checks each curve only during
each source cell's active interval; the validator independently reconstructs
those overlaps from the physical clock and source history. The representation
retains every source interval without requiring an optimizer span per sample
for the direct solution.

Current limitations: earliest target interception certifies motion at its
kinematic lower bound; later meeting times are not yet searched if obstacles
prevent certification there. Dynamic earliest requests retain full source-cell
coverage, but initialization uses an initial-time visibility route,
so the planner is not complete for dynamic topology changes or goals blocked
only at the initial time. The full static suite still needs improvement.
Historical example interfaces are being migrated to the single `planner`
entry point. No universal trajectory optimality or runtime guarantee is claimed.
Static and dynamic fixed-arrival requests now share the same elastic
length-minimizing convex formulation; scene motion does not choose the objective.
Earliest-arrival requests use four degree-eight subspans per visibility edge.
One convex solve minimizes the time-power objective; a second minimizes
control-polygon length while preserving that returned time value. Finite stalled
iterates remain proposals, and convergence is reported only when both stages
converge. Polynomial export shares physical position through acceleration at joins,
then rebuilds derivative bounds and collision certificates for the corrected
curve. Validation limits and tolerances remain unchanged.

## Verification

```matlab
addpath('tests');
assertSuccess(runtests('tests'));
checkBenchmarkTimingContract();
summary = runExampleBenchmarks({'exampleVietnamKeepoutSlew'}, 5);
```

The runner preserves example inputs and workbook references, reports failures,
uses median end-to-end example wall time, and counts all production `.m` lines
(including comments/blanks) in the root entry point and production packages.
Per-run metrics and summary reports remain ignored by Git. Scenario-validation
warnings count as benchmark failures, and expected no-path outcomes require
the explicit no-route termination reason. The geographic sequence contains
three internal requests; it still requires per-request capture before claiming
full benchmark coverage from the historical final-result row.

The workbook's route-length column measures the historical selected seed,
which can be a blocked direct chord. For example, target-exit's seed length
20.1357890335 equals `norm([12.1,1.2]-[-8,0])`, and alternating occlusion's
13.3416640641 equals `norm([-1,0]-[-14,3])`; both examples explicitly require
the direct path to be blocked. The old solver bends the returned motion while
retaining that diagnostic seed. Current `Route_units` is the exact protected
visibility route. The runner reports both route lengths and their comparison,
but uses the executable motion length for the physical path-quality gate.

## Bounded-jerk timing contract

`exampleObstacleFree` requests x displacement 4, velocity bound 2, acceleration
bound 1, and jerk bound 2. The exact minimum rest-to-rest duration is

```
T = A/J + sqrt((A/J)^2 + 4D/A) = 4.531128874149275 seconds.
```

Peak velocity is 1.765564437, below the velocity bound. Equality requires the
bangbang jerk schedule `+J, 0, -J, 0, +J`, with instantaneous changes. The workbook
reference equals this bound within floating-point precision. The user approved
bounded jerk jumps. Optimization and independent validation now require C2
motion and retain the original jerk bounds, collision margins, and tolerances.
Direct earliest motion uses an exact scalar jerk-limited profile on the endpoint
chord, elevated into the shared Bezier representation and independently
certified. Obstacle-free motion has no exemption from validation.

## Measured development results

Vietnam, unchanged physical inputs and workbook references, five-run median:

| Metric | New core | Reference |
| --- | --- | --- |
| Arrival (s) | 30.000000 | 30.000000 |
| Motion length | 17.1447365583 | 17.3053746209 |
| Seed route length | 17.1499224098 | 17.2979913158 |
| End-to-end example wall time (s) | 2.939 | 21.2103192 |
| Production physical lines, including comments/blanks | 3,586 | Limit: fewer than 7,000 |

All five motions independently validate. There are 240 active source cells,
30 optimizer spans, 60 exported spans, and 480 final clearance pairs. The run
uses one trajectory SOCP and no plane SOCPs. The finite solver iterate reports
stalled numerical convergence (`-7`); it is accepted only after all independent
physical certificates pass. This is a valid feasible result, not a claim of
proven global optimality. Thirteen tests pass, including source/certificate
tampering, fixed-time endpoints, nonzero absolute time origin, and core cases.
Integrating speed on a separate 1 ms grid gives length 17.1447473836, also
below the reference; the benchmark improvement does not depend on 50 ms output
sampling underestimating the curve length.

On 30 identical randomized convex scenes, lazy visibility search matched the
exhaustive baseline route lengths within 1e-8. Total measured graph time fell
from 1.74648 to 0.383798 seconds (4.55x). This is evidence for these inputs,
not a universal runtime or trajectory optimality guarantee.

Earlier static diagnostic measurements before the Vietnam refinement (three-run
medians; these are development evidence, not current full-suite certification):

| Example | Valid outcome | Duration / reference (s) | Length / reference | Wall / reference (s) |
| --- | --- | --- | --- | --- |
| Obstacle free | Yes | 6.11010 / 4.53113 | 4.47234 / 4.47214 | 0.13947 / 0.10130 |
| No path | Expected failure | n.a. | n.a. | 0.09666 / 0.56736 |
| Static U | No | n.a. / 20.87255 | n.a. / 38.67808 | 1.96226 / 6.49741 |
| Opposing Us | Yes | 23.51905 / 22.10063 | 24.72464 / 24.20576 | 0.57687 / 2.09183 |
| Alternating slalom | Yes | 10.72341 / 10.55009 | 17.30310 / 16.03475 | 0.25309 / 7.48968 |

Fixed-time moving-target interception now also passes all references: five-run
median wall time 0.05589 s versus 0.1950635 s, arrival 12 s, and motion length
9.5389405468 (equal to the endpoint distance and reference). It uses no SOCP.
Four target tests pass, including altered source data and invalid histories;
the prior thirteen core/Vietnam tests also pass. A three-run Vietnam regression
retains its exact motion length and 30 s arrival, with median wall time 3.397 s.

The four accelerating circles also pass unchanged: arrival 22 s, exact motion
and route length 20, and five-run median wall time 4.030 s versus 7.1524886 s.
All 880 source intervals remain certified, with two exported polynomial spans
and no SOCP. Four additional tests cover polynomial restriction, full source
coverage, forged activity, and interior source changes. That milestone contained
3,696 physical production lines. A profile of the prior formulation attributed
12.44 s to `coneprog` within a 23.13 s run that failed validation; the direct
formulation removes that solve, rather than relaxing any physical constraints.
Profiled time is diagnostic only; the benchmark medians have profiling off.

The static target-exit example now passes physical benchmarks: arrival 24 s,
motion length 20.5043115855 versus 20.6851467568, and five-run median wall time
2.114 s versus 4.6280002 s. Alternating occlusion also passes after exact convex
merging and static occupancy batching: its five-run median wall time fell from
4.113 s to 2.035 s, below 2.4666327 s. Arrival remains 20.8695652174 s and motion
length 13.5563779512 beats 13.6104156607. The scene has 18 convex regions instead
of 48. Tests check occupied-area equality, disjoint interiors, convexity, holes,
disconnected components, exact source equality, activity endpoints, and first
blocking-obstacle order. The current core has 3,765 physical production lines.

With merged convex regions, static U now returns valid motion in a three-run
median of 0.316 s, but duration 24.010 s and length 43.576 still exceed references
20.873 s and 38.678. The earliest-arrival formulation still needs improvement.

Vietnam, fixed-time interception, accelerating circles, target exit, alternating
occlusion, and the expected no-path case have demonstrated the physical benchmark metrics.
The full suite remains unfinished. No scenario-specific fallback was added.

The preparation milestone's combined regression: all 25 tests pass and all five runs independently
validate for each of the six demonstrated examples (no-path returns the expected
explicit failure). Profiling is disabled; medians include full example execution:

| Example | Arrival / reference (s) | Motion length / reference | Wall / reference (s) |
| --- | --- | --- | --- |
| Vietnam slew | 30 / 30 | 17.144737 / 17.305375 | 2.936 / 21.210 |
| Accelerating circles | 22 / 22 | 20 / 20 | 4.339 / 7.152 |
| Fixed-time target | 12 / 12 | 9.538941 / 9.538941 | 0.01380 / 0.19506 |
| Target exit | 24 / 24 | 20.504312 / 20.685147 | 1.934 / 4.628 |
| Alternating occlusion | 20.869565 / 20.869565 | 13.556378 / 13.610416 | 1.854 / 2.467 |
| No path | Expected no route | n.a. | 0.01026 / 0.56736 |

The subsequent earliest-arrival milestone adds opposing Us: arrival
21.8551 s versus 22.1006 s, motion length 24.1924 versus 24.2058, and five-run
median wall time 0.8174 s versus 2.0918 s. The core contains 3,816 physical
production lines, and 27 tests pass. The short-span reconstruction test checks
actual physical derivative continuity after correcting the curve; it does not
relax the independent validator's 1e-8 threshold.

Remaining earliest-arrival limitations are measured explicitly. Static U now
has valid motion of length 37.458, below 38.678, but arrival 23.300 s exceeds
20.873 s. Slalom remains above both targets (11.171 s and 16.569, versus
10.550 s and 16.035). Obstacle-free motion improves to straight length
4.472135955 and duration 4.65699 s; its exact historical duration remains
a limitation of that earlier C3 implementation. The approved C2 contract and
exact jerk-limited primitive resolve the obstacle-free timing limitation.

Trials of extra alternating iterations, a shared bangbang progress clock for
knot initialization, and joint nonlinear time/control optimization were
discarded. They missed quality or runtime targets, or failed validation. No
nonlinear optimizer or scenario-specific repair was retained in production.

The bounded-jerk milestone permits jerk jumps while retaining continuous
position, velocity, and acceleration and every original magnitude bound.
Thirty existing tests plus four new bounded-jerk tests pass. Five-run benchmark
medians (3,877 physical production lines) all meet the eight listed references:

| Example | Arrival (s) | Motion length | Wall (s) |
| --- | --- | --- | --- |
| Obstacle free | 4.531128874 | 4.472135955 | 0.02036 |
| Vietnam slew | 30 | 17.1441 | 2.9191 |
| Accelerating circles | 22 | 20 | 4.3479 |
| Fixed-time target | 12 | 9.538940547 | 0.01055 |
| Target exit | 24 | 20.5043 | 1.9423 |
| Alternating occlusion | 20.869565217 | 13.5564 | 1.7238 |
| No path | Expected no route | n.a. | 0.00955 |
| Opposing Us | 21.8192 | 24.1877 | 0.7422 |

The direct primitive covers triangular acceleration, acceleration saturation,
and velocity cruise with unequal axis limits and nonzero absolute start times.
Independent tests accept bounded jerk jumps, reject excessive jerk, and reject
acceleration discontinuities even when derivative arrays and histories agree.
Source-history coverage is also required for dynamic earliest motion.

Earliest sampled-target interception now uses the single public planner. It
partitions linear or pchip target histories at source and reachability events,
then intersects cubic polynomial inequalities for both axes. This enumerates
meeting windows even when feasibility disappears before the horizon. A target
outside the entire reachable set returns `targetUnreachable`; an uncertified
motion at its lower bound returns `earliestInterceptUncertified` without claiming
that later interception is impossible. The public validator reevaluates the
original target at the actual returned arrival, including endpoint occupancy.

Five-run earliest-target results: arrival 6.111111111 s, equal reference motion
length 7.308890240, and median wall time 0.01559 s versus 0.1488863 s. The core
contains 3,988 physical production lines. Fixed-time target and obstacle-free
regressions also pass all benchmark gates. Additional tests cover disconnected
meeting windows, unreachable targets, source tampering, shifted clocks, pchip
motion, and all bounded-jerk timing regimes.

A trial mapping route vertices onto a single chord's jerk-phase clock was
removed. It passed slalom quality (10.500004 s and 16.020866 length) but missed
dense-concave references (8.5000038 s and 12.774610 versus 8.5 and 12.761105),
regressed static U to 27.743 s and 41.962 length, and made opposing-U optimization
slow enough to terminate. No case-dependent mesh fallback was retained.
