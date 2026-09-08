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

At the interception milestone, all 39 tests pass and nine of the eighteen
historical examples have demonstrated all physical benchmark gates. The goal
remains incomplete. A subsequent single-run audit confirmed unresolved static
U, dense-concave, slalom, translating-circle, rotating-field, barrier-wait, and
opening-U cases. The deforming-outline example failed certification and took
114.977 s versus its 30.805 s reference. The audit was interrupted during the
large static geographic sequence; its three internal requests still need
separate capture and measurement. This audit is not a repeated-runtime result.

A second phase-mesh trial exactly preserved old controls and knot timing while
adding analytic jerk switches by subdivision. Opposing-U time improved to
21.6337 s, but median runtime increased to 3.595 s, above 2.092 s. Dense-concave,
slalom, and static-U timing still missed their references. This refinement was
also removed; the original detour mesh remains in production.

Visibility containment queries now run once per expanded A* node, retaining
every edge-contact partition and the same MATLAB `inpolygon` decision. A
60-second profile of the deforming-outline request attributed 48.587 s to
visibility search, with 219,280 segment tests and 117,640 containment calls.
Batched search took 20.917 s on the saved identical scene; its entire graph
record, including accepted/rejected edges, weights, expansion count, and route,
was exactly equal to the saved pre-change result (`isequaln`).

The full deforming-outline example now has a three-run median of 36.376 s.
The earlier single-run audit was 114.977 s; its historical runtime reference
is 30.805 s, and motion certification still fails. This is a measured search
speedup, not a passing example. Vietnam's three-run regression still passes at
30 s arrival and 17.1441 length, with median wall time 2.9615 s. All 40 tests
pass, including analytic route lengths for holes, disconnected/touching
components, collinear contact, and a concave U. The core contains 3,996 physical
production lines. No ring-correspondence optimization was retained: the profile
identified graph containment work as the main cost.

Time cells retain affine endpoint vertices for source intervals whose vertex
correspondence has been verified. Constant cells retain equal endpoints, and
uncertain topology still uses the authoritative conservative interval union.
Separating planes can therefore translate with an obstacle. The obstacle-side
certificate bounds the exact quadratic Bernstein product of affine vertices
and an affine normal. The independent validator reconstructs endpoint geometry
from the original source and checks every active physical-time overlap.

All 44 tests pass. New tests cover a safe translating separation whose spatial
sweep overlaps the curve, clipping a source interval, interior plane violations,
and forged or omitted endpoint coverage. Three-run medians remain below
references for Vietnam (2.998 s, arrival 30 s, motion 17.1441) and accelerating
circles (3.913 s, arrival 22 s, motion 20). Motion bounds and safety margins
are unchanged. Unknown-clock optimization still uses the conservative spatial
projection; fixed-clock optimization uses the retained cell motion.

The kinematic-bound guide first solves the independent-axis jerk-limited
profiles. It projects authoritative space-time cells onto the limiting axis's
clock, searches the resulting exact monotone visibility graph, and optimizes
the remaining axis on the common phase/event mesh. Constant-velocity phases
use convex space-time intersections; nonlinear phases retain conservative
spatial enclosures. The returned trajectory still passes the original full
independent validator. Prescribed analytic controls are eliminated exactly
from the conic system to avoid redundant axis constraints.

All 49 correctness tests pass, including coordinate exchange, absolute clock
shifts, and rejection of zero-time edges that would falsely escape a cavity.
The core contains 4,355 physical production lines, including comments and
blank lines. Three-run results add four passing benchmark examples:

| Example | Arrival (s) | Motion length | Median wall (s) | Reference wall (s) |
| --- | --- | --- | --- | --- |
| Dense concave | 8.5000000002 | 12.756125 | 1.570 | 3.835 |
| Moving circle | 8.5000000004 | 12.448756 | 0.359 | 2.097 |
| Alternating slalom | 10.5000000006 | 16.020389 | 0.661 | 7.490 |
| Rotating field | 9.0416666670 | 20.4559 | 0.414 | 2.380 |
| Opposing Us | 21.6333333333 | 24.0563 | 0.883 | 2.092 |
| Vietnam slew | 30 | 17.1441 | 3.418 | 21.210 |

Opposing Us now reaches its kinematic time bound as well. The full historical
goal remains incomplete. Alternating occlusion still passes physical quality,
but its latest three-run median is 2.718 s against 2.467 s. Interleaved runs
of the committed and proposed conic steps produce identical motion and similar
warm runtimes around 2.5 s; this runtime gate needs more margin. Thus the
current evidence supports twelve of eighteen examples meeting all gates,
with occlusion, static U, both waiting cases, and both large geographic
examples outstanding. Eliminating endpoint equalities was tried and removed:
it did not resolve occlusion runtime and missed the circle's arrival tolerance.

The planner now tests the analytic/kinematic-bound motion before constructing
the initial spatial visibility graph. A successful clock projection is the
returned graph; an analytic chord reports `SearchKind="analyticMotion"` with
empty search trace arrays. Failed bound attempts proceed to spatial search
without repeating the bound solve. This also permits an initially occupied
goal that clears before the actual arrival. Fixed-arrival planning is unchanged.

Prescribed cubic axis coefficients now remain analytic through subdivision
and export. This avoids manufacturing higher-degree roundoff on very short
cruise spans. Both original controls and the complete exported curve retain
the existing derivative-bound, collision, and independent validation checks.
Trials of generic endpoint reconstruction and derivative-bound changes were
discarded; no tolerances were relaxed.

All 50 tests pass. At 4,452 production lines, thirteen of eighteen historical
examples meet every gate. The deforming US outline now passes with arrival
7.916666667972 s (reference 7.916666666667), motion length 40.238008060418
(reference 40.248219224097), and three-run median wall time 17.130697 s
(reference 30.8049645). Before moving the bound solve upfront, the first valid
run took 42.86 s. Its tiny export-related timing excess was also corrected.
Vietnam's repeated median is 3.0494 s with unchanged 30 s arrival and 17.1441
motion length. The latest occlusion median remains above its runtime gate:
2.588 s versus 2.467 s. Static U, both waiting cases, and the static geographic
sequence also remain outstanding.

The geographic sequence exposes its three unmodified planner results as an
optional third output. The benchmark runner independently checks every
subcase and writes their individual metrics to an ignored subcase CSV;
the final Philippines result alone cannot establish sequence success.

A subsequent geographic audit was interrupted after more than five minutes
without returning results. A bounded profile attributed 42.01 s to three
conic solves, versus 4.11 s to visibility search. The unchanged geographic
inputs were captured separately for reproducible per-region diagnosis.
Hawaii has 278 convex cells and 13 clock spans; Croatia has 340 cells and
8 spans; the Philippines has 2,193 cells and 20 spans. The shortest geographic
clock spans are about 5--6 milliseconds.

Removing optimization pairs already separated along the prescribed axis was
tried with physical derivative row scaling. The small clock tests passed
after scaling, but the geographic sequence still did not return within several
minutes. That trial was removed. Arc-length timing and uniform-time spatial
mesh trials were also removed: four spans per edge yielded static-U arrival
20.8921 s and length 38.7071, narrowly missing both references; three yielded
21.2814 s and 40.0102, and five became too slow to retain. These experiments
do not change the thirteen-case milestone or its committed production core.

Departure scheduling now uses the same analytic jerk-limited progress clock
and authoritative convex time cells. Intersecting each cell with the chord
produces a convex polygon in path progress and absolute time. Along each
polygon edge, forbidden departure times are a cubic function of local phase
time; endpoints and real derivative roots give its extrema. The union of
these intervals determines the first available departure without a time grid.
The initial position must remain free throughout the wait. Returned motion
includes the stationary interval, retains analytic cubic coefficients, and
passes the unchanged public independent validator.

Stationary intervals within an otherwise changing history retain their exact
nonconvex decomposition. This shortcut requires verified vertex correspondence
and zero motion. Unknown correspondence still retains the entire authoritative
interval union, even when that conservative representation reports zero speed.
The regression suite explicitly checks this distinction.

The scheduled chord is compared with the speed bound for traversing the initial
spatial guide; when that guide remains competitive, its optimized candidate is
also compared. This searches a delayed direct-motion family, and does not claim
global optimality among every possible moving detour. The waiting examples now
check stationary spans and actual gap crossings rather than obsolete seed names.

All 56 correctness tests pass. At 4,644 physical production lines, fifteen of
eighteen examples now meet every historical gate in three-run measurements:

| Example | Arrival (s) | Motion length | Median wall (s) | Reference wall (s) |
| --- | --- | --- | --- | --- |
| Moving barrier wait | 10.090088895723 | 10 | 0.048625 | 1.107174 |
| Opening U | 11.584333447453 | 10 | 0.157334 | 1.769550 |
| Vietnam slew | 30 | 17.144137073137 | 2.827637 | 21.210319 |
| Deforming US outline | 7.916666667972 | 40.238008060418 | 16.75 | 30.804965 |

The two waiting references arrive at 10.090301513672 and 13.617522354126 s,
respectively, both with motion length 10. Separate fresh-process three-run
medians for the waiting examples were 0.269 and 0.300 s, also below their gates.
The remaining failures are alternating-occlusion runtime, static-U quality,
and the three-region geographic sequence. No claim of complete goal success
is made.

Sparse separating-plane constraints are now assembled in one triplet operation,
and control-pair enumeration uses the exact lower-triangular index order.
These algebraically equivalent changes reduce the core to 4,635 physical
production lines. All 56 tests still pass; original and triplet-assembled
occlusion runs returned bit-identical polynomial records. Interleaved warm
comparisons saved about 0.12 s, but a fresh five-run median of 2.586333 s still
misses the 2.466633 s occlusion runtime gate. This is a code-size and assembly
improvement, not an additional passing benchmark.

Supporting-direction selection now accounts for the exact Bernstein product
used by certification. It retains the original hull-clearance ranking, but
when any available direction can certify the product, directions that cannot
certify it are excluded. Existing successful selections remain unchanged.
A regression covers a curve whose original hull overlaps an obstacle while
the certified product hull has a separating direction that the old ranking
missed. All 57 tests pass, and all fifteen achieved examples again pass three-run
benchmark checks. The core contains 4,643 physical production lines. Occlusion
remains above its runtime reference at 2.517 s.

A separate motion-knot/collision-partition prototype was tested and removed.
It used exact de Casteljau constraint restrictions and direct Taylor export,
with one or two spans per physical phase. The one-span version missed several
path-length gates; the two-span version still missed slalom length. Hawaii
retained collision slack up to 0.115 after 35 iterations (about 61 s) at the
kinematic-bound clock. Letting that clock vary took about 122 s and failed to
return a certified motion. These failures apply to the tested formulation;
they do not establish global infeasibility. No failed mesh, clock, or coordinate
translation trial remains in production.

Visibility now prepares edge vectors, bounds, and parallel tolerances once per
scene, then batches the unchanged intersection predicates with bounded temporary
storage. It still classifies every open interval between contacts. Complete
visibility records were identical to the previous implementation on occlusion
and all three geographic scenes. Geographic graph times changed from
3.258/0.040/1.770 s to 2.278/0.034/1.210 s for Hawaii/Croatia/Philippines.

Final certification tries the preceding span's separating direction, recomputes
its supports on the current physical interval, and runs the full Bernstein
inequalities before accepting it. A failed proposal triggers the existing
supporting-axis search. Every pair remains covered by the unchanged independent
validator. The new regression checks reuse, reversal to the opposite side of an
obstacle, and rejection of an interior point.

The core has 4,669 physical production lines. All 58 correctness tests pass.
Occlusion's five-run median is 2.391 s against its 2.466633 s reference, with
unchanged arrival 20.8695652173913 s and motion length 13.5563597604046.
The prior five-run baseline in this comparison was 2.676 s. The static-U and
geographic motion benchmarks remain outstanding; faster visibility alone does
not solve their optimization failures.
All sixteen achieved examples also pass a separate three-run sweep: occlusion
has a 2.382127 s median and Vietnam has a 2.738645 s median. Vietnam retains
arrival 30 s and motion length 17.144137073137 against references 30 s,
17.305374620919, and 21.210319 s wall time.

Two further optimization experiments were rejected without production changes.
Exact dual clock sensitivities agreed with finite differences, but optimizing
static-U span ratios took 25.993 s and arrived at 20.914458 s, still missing the
20.872548 s reference. Integrated jerk formulations did not improve the original
conic step: separate span states failed validation in 4.701 s, while globally
integrated jerk controls took 43.572 s and failed endpoint validation. These
results do not justify adding either formulation to the planning core.
