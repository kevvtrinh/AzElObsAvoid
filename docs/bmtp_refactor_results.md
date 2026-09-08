# BMTP refactor verified results — September 8, 2026

The retained implementation combines bounded exact predicates, static geometry reuse,
analytic departure scheduling, eligible physical-clock corridors, adaptive motion
length, one planner entry point, and a shared numerical motion core. The full MATLAB
R2024b suite passes **222/222 tests**. All **60** final frozen-request comparisons
pass against pinned cleanup `c04f3b280725edd30824947f18751afe07a7a0d9`.

## Matched planner runtime

Both revisions use the same 20 frozen physical requests and options, three untimed
warmups requesting two outputs, and three timed calls per case on the same machine
and MATLAB release. The table gives medians, not profiler times. Each geographic
case is captured separately. These measurements exclude example setup and the
interception wrapper's time search: the frozen interception request is its selected
fixed-time trial. They are not end-to-end interception speedups. Small millisecond
timings are particularly noisy. No claim of universal speedup follows from this suite.

| Example | Before (s) | Now (s) | Speedup | Runtime change |
| --- | ---: | ---: | ---: | ---: |
| exampleAlternatingSlalom | 2.063982 | 0.310758 | 6.64x | -84.9% |
| exampleVietnamKeepoutSlew | 10.305056 | 9.497585 | 1.09x | -7.8% |
| exampleDenseConcaveObstacle | 1.655341 | 0.337029 | 4.91x | -79.6% |
| exampleFourAcceleratingCircles | 1.052097 | 0.823317 | 1.28x | -21.7% |
| exampleInterceptMovingTargetAtSetTime | 0.010234 | 0.005716 | 1.79x | -44.1% |
| exampleInterceptMovingTargetEarliest | 0.010383 | 0.007380 | 1.41x | -28.9% |
| exampleMovingBarrierWait | 0.240027 | 0.113239 | 2.12x | -52.8% |
| exampleMovingCircleNoWrap | 0.966975 | 0.700081 | 1.38x | -27.6% |
| exampleMovingDeformingUSOutlineVisibility | 3.165194 | 2.951616 | 1.07x | -6.7% |
| exampleMovingRotatingObstacleField | 1.232469 | 0.923029 | 1.34x | -25.1% |
| exampleNoPath | 0.209288 | 0.181021 | 1.16x | -13.5% |
| exampleObstacleFree | 0.009326 | 0.004712 | 1.98x | -49.5% |
| exampleOpeningUShapedObstacle | 0.842497 | 0.376014 | 2.24x | -55.4% |
| exampleStraightTargetAlternatingOcclusion | 1.090653 | 1.032126 | 1.06x | -5.4% |
| exampleTargetExitsObstacle | 2.612883 | 2.590662 | 1.01x | -0.9% |
| exampleTwoOpposingUVisibilityGraph | 1.160726 | 1.163136 | 1.00x | +0.2% |
| exampleStaticUShapedObstacle | 4.308094 | 4.242801 | 1.02x | -1.5% |
| GeographyHawaii | 2.698869 | 1.152189 | 2.34x | -57.3% |
| GeographyCroatia | 1.221629 | 1.110506 | 1.10x | -9.1% |
| GeographyPhilippines | 5.637475 | 4.186914 | 1.35x | -25.7% |

The sum of case medians falls from **40.493 s to 31.710 s**
(21.7% less runtime; 1.28x). This gives each example one
run and is not a measured wall-clock suite run or an application workload forecast.
Opposing U is effectively unchanged (+0.2%); static U and target exit improve only
slightly. All individual measurements, including slower samples, remain in
`benchmark.csv`. Raw logs/captures are retained in ignored `scratch/`.

## Returned motion and independent checks

| Example | Duration (s) | Route (units) | Motion arc (units) | Success / independent validation / collision / kinematics / certificate | Reason |
| --- | ---: | ---: | ---: | --- | --- |
| exampleAlternatingSlalom | 10.5000000001 | 16.0193197983 | 16.0200251124 | pass / pass / pass / pass / pass | goalReached |
| exampleVietnamKeepoutSlew | 30 | 17.2979913158 | 17.3054420696 | pass / pass / pass / pass / pass | goalReached |
| exampleDenseConcaveObstacle | 8.50000000007 | 12.7007215595 | 12.7611343658 | pass / pass / pass / pass / pass | goalReached |
| exampleFourAcceleratingCircles | 22 | 20 | 20 | pass / pass / pass / pass / pass | goalReached |
| exampleInterceptMovingTargetAtSetTime | 12 | 9.53894054682 | 9.53894054682 | pass / pass / pass / pass / pass | goalReached |
| exampleInterceptMovingTargetEarliest | 6.11111111111 | 7.30889024019 | 7.30889024019 | pass / pass / pass / pass / pass | goalReached |
| exampleMovingBarrierWait | 10.0910888956 | 10 | 10 | pass / pass / pass / pass / pass | goalReached |
| exampleMovingCircleNoWrap | 8.5 | 12.4537884602 | 12.4538380036 | pass / pass / pass / pass / pass | goalReached |
| exampleMovingDeformingUSOutlineVisibility | 7.91666666667 | 40.2482192241 | 40.2482682085 | pass / pass / pass / pass / pass | goalReached |
| exampleMovingRotatingObstacleField | 9.04166666667 | 20.7162087908 | 20.7162516635 | pass / pass / pass / pass / pass | goalReached |
| exampleNoPath | NaN | NaN | NaN | false / false / false / NaN / NaN | noValidatedSeed |
| exampleObstacleFree | 4.53112887415 | 4.472135955 | 4.472135955 | pass / pass / pass / pass / pass | goalReached |
| exampleOpeningUShapedObstacle | 11.5853334462 | 10 | 10 | pass / pass / pass / pass / pass | goalReached |
| exampleStraightTargetAlternatingOcclusion | 20.8695652174 | 13.3416640641 | 13.6104194209 | pass / pass / pass / pass / pass | goalReached |
| exampleTargetExitsObstacle | 24 | 20.1357890335 | 20.6851514237 | pass / pass / pass / pass / pass | goalReached |
| exampleTwoOpposingUVisibilityGraph | 22.1006280522 | 24.035784715 | 24.205785351 | pass / pass / pass / pass / pass | goalReached |
| exampleStaticUShapedObstacle | 20.8725483491 | 34.9425880405 | 38.6784456458 | pass / pass / pass / pass / pass | goalReached |
| GeographyHawaii | 4.31423580472 | 12.6470414151 | 12.6895074597 | pass / pass / pass / pass / pass | goalReached |
| GeographyCroatia | 3.27509132513 | 7.27825414719 | 7.51903810119 | pass / pass / pass / pass / pass | goalReached |
| GeographyPhilippines | 5.82760502419 | 22.0706469075 | 23.2586701098 | pass / pass / pass / pass / pass | goalReached |

All 19 successful cases pass fresh public validation, continuous collision and
physical-limit checks, and applicable certificates on each repetition. The no-path
fixture retains `noValidatedSeed`, which is exhausted search rather than a global
infeasibility proof. Arrival and independently integrated arc satisfy the unchanged
quality gates. Relative to cleanup, slalom arrives about 0.050094 s earlier with a
0.014749-unit shorter arc; opening U arrives 2.032189 s earlier; Hawaii's arc shortens
by 0.23039 units. The moving barrier arrives 0.000787382 s later, within the existing
0.001 s acceptance tolerance. Other differences are numerical noise or zero.
The final shared-core cleanup changes no arrivals or arcs from its preceding capture.

## Production size and displaced implementations

Counts include all production MATLAB packages and the root planner, excluding tests,
examples, sandbox interfaces, and reports. Code means a nonblank line whose first
nonspace character is not `%`; this is separate from physical line count.

| Subsystem | Before files / physical / code | Now files / physical / code | Now comments / blanks |
| --- | ---: | ---: | ---: |
| `+obstacleAvoidance` | 3 / 640 / 419 | 1 / 58 / 29 | 27 / 2 |
| `+obstacleAvoidance/+geometry` | 5 / 660 / 450 | 5 / 643 / 465 | 138 / 40 |
| `+obstacleAvoidance/+input` | 9 / 545 / 243 | 9 / 483 / 243 | 197 / 43 |
| `+obstacleAvoidance/+obstacles` | 12 / 1411 / 938 | 12 / 1379 / 968 | 325 / 86 |
| `+obstacleAvoidance/+planner` | 19 / 2906 / 1974 | 20 / 3028 / 2179 | 657 / 192 |
| `+obstacleAvoidance/+plotting` | 2 / 505 / 400 | 2 / 493 / 400 | 65 / 28 |
| `+obstacleAvoidance/+search` | 13 / 1749 / 1166 | 13 / 1685 / 1176 | 409 / 100 |
| `+obstacleAvoidance/+validation` | 5 / 1229 / 948 | 4 / 1041 / 851 | 145 / 45 |
| `planner.m` | 0 / 0 / 0 | 1 / 279 / 185 | 55 / 39 |
| `trajectory` | 1 / 46 / 13 | 0 / 0 / 0 | 0 / 0 |
| `trajectory/+bmtpEngine` | 22 / 2660 / 1679 | 26 / 3052 / 2216 | 653 / 183 |
| `trajectory/+motionCore` | 0 / 0 / 0 | 4 / 312 / 230 | 65 / 17 |
| `trajectory/+ruckigEngine` | 17 / 3177 / 2286 | 14 / 2635 / 2007 | 447 / 181 |
| **Total** | **108 / 15528 / 10516** | **111 / 15088 / 10949** | **3183 / 956** |

Physical production is 440 lines (2.8%) below the 15,528-line baseline. Executable
lines are still **433 above baseline** (4.1%); the new formulations and broader
coverage have not produced an executable-line reduction against the original branch.
The first API consolidation checkpoint had 11,231 code lines: sharing the numerical
core removes **282 actual code lines**, beyond its separate comment cleanup.
This is a substantive deletion but not an emptycore-sized rewrite. Standalone Ruckig
capabilities remain retained while the user's scope choice is pending.

Removed production implementations include duplicate jerk-event integration/export
in BMTP, scalar Ruckig, general Ruckig, and acceleration-only Ruckig; duplicate
polynomial evaluators and format conversion at the waypoint boundary; duplicate
rest-to-rest seven-phase timing formulas; duplicate scalar range checkers; the
single-caller coefficient wrapper; the earlier duplicate complete-motion exporter;
public planner/interception forwarding layers; and recursive diagnosis-table
flattening. Optional diagnostics preserve the complete raw evidence instead.
Independent polynomial basis reconstruction remains in the validator so generator
conversion errors are not accepted through the same conversion code.

## Shared-core verification and unfavorable evidence

The first focused run exposed an overly broad symbol replacement that incorrectly
renamed `evaluatePolynomialConstraints`; it was corrected before retention. The
focused suite then passed 61 tests, followed by two complete 222-test runs around
the final removal of duplicate endpoint work and one-use coefficient calls.
Reference tests cover scalar laws under both timing policies, nonzero endpoints,
multiple axes, polynomial degrees 3/5/8, event boundaries, acceleration switching,
and 1,200 range comparisons across degree and scale. Public feature tests retain
later interception, explicit derivatives, wrapping, holes, changing geometry,
forged/stale preparation rejection, and explicitly selected waypoint behavior.

A matched rerun of the initially slower shared-core cases was performed once.
Dense concave and specified-time interception improved in that rerun; Croatia was
almost unchanged. Earliest interception was 0.285 ms slower and obstacle-free was
1.761 ms slower. A nonzero two-axis engine case was 0.102 ms slower. Those results
are preserved, not replaced by favorable trials. Subsequent profiling identified
duplicate linear endpoint work and a redundant one-use coefficient call. Removing
them led to the final capture above. The final earliest-interception microtiming is
still variable and slower than the preceding shared-core capture, though faster
than pinned cleanup. Shared numerical ownership is retained for measured executable
deletion and exact behavior preservation; it is not claimed to accelerate every call.

Five migrated interception examples were also run end to end with independent
validation, and one returned target motion was plotted with kinematics. Full example
runtimes (including setup/search) were 2.656137 s for accelerating circles,
0.067132 s for specified interception, 0.087062 s for earliest interception,
1.943845 s for alternating occlusion, and 2.827999 s for target exit. These are single
functional runs, not matched before/after speed measurements. All five passed every
applicable check and returned `goalReached`; their complete metrics are in the CSV.

## Eligibility and remaining limitations

- Exact batched predicates retain source geometry, margins, and roundoff treatment.
  Search still has the original bounded graph construction, retries, and budgets;
  it is not a newly complete global visibility planner.
- Static geometry is reused within a request. Moving geometry remains authoritative
  at its modeled times; public validation rebuilds preparation independently.
- Analytic departure and physical clocks are conditional on the documented endpoint,
  timing, motion, and route assumptions. Nonzero endpoints, fixed arrivals, moving
  geometry, limiting-axis ties, and reversing guides retain broader methods as needed.
- Failed clock attempts add overhead on some fallback cases. Earlier matched opposing-U
  and Philippines regressions remain explicit in the corridor report. A necessary
  reachability and linear-feasibility precheck limits that work; it cannot eliminate it.
- Speed quadrature replaces the proxy objective in eligible new convex problems.
  Final ranking uses adaptive arc length. The broader legacy refinement still uses
  its control-polygon surrogate internally; not every optimization objective changed.
- Emptycore's cubic reversal method was evaluated on its actual reversing case and
  rejected as a speed replacement. Wholesale emptycore substitution also lost a
  supported interception and worsened Hawaii quality. Broader capability was retained.
- No geometry, validator tolerances, physical limits, or example requests were relaxed.

Stage evidence: [baseline and reference](bmtp_refactor_reference_comparison.md),
[feature gates](bmtp_refactor_feature_gates.md), [containment](bmtp_refactor_containment.md),
[departure](bmtp_refactor_departure.md), [corridors](bmtp_refactor_corridors.md),
[length](bmtp_refactor_length.md), [visibility](bmtp_refactor_visibility.md), and
[API/shared-core consolidation](bmtp_refactor_consolidation.md).
