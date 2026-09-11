# Fixed-arrival time-expanded visibility

This change is based on `build-core` commit `59a8935`. The public planner now
accepts `FixedArrivalSearch='timeExpanded'` together
with `GoalTimeMode='fixedArrival'`. The default remains `FixedArrivalSearch='spatial'`.
The timed option targets the prescribed final graph layer, then solves BMTP
at that exact deadline. It does not call earliest arrival and pad its result.

## Implementation

The existing graph search already distinguishes earliest and fixed arrival.
This change connects its fixed mode to the public dispatcher and the shared
BMTP handoff. The existing sampled edge checks, node generation, time layers,
obstacle cells, solver limits, and independent validator are retained.

Fixed requests remove only interior points of constant-position runs from the
timed seed. Their first and last times remain, so the continuous piecewise-linear
guide is identical. This avoids turning hundreds of wait samples into hundreds
of BMTP spans. BMTP jointly optimizes the full trajectory with continuous
position, velocity, acceleration, and jerk; a wait is not an instantaneous stop.

The supported request has a fixed-position goal and zero endpoint velocity and
acceleration. Unlike the earliest-arrival automatic dispatcher, the explicitly
requested fixed method also accepts sparse histories. Unsupported requests,
graph exhaustion, failed BMTP motion, and failed certification have explicit
termination reasons. There is no silent switch to spatial planning when the
user selects this option. Search time and retained route times are exposed in
`VisibilityGraph.TimedSearch.ElapsedTime_s` and `VisibilityGraph.RouteTime_s`.

## Runtime and motion comparison

The final counterbalanced benchmark uses one warmup and two measured runs per
method. Both timed runs pass independent validation in both cases. All three
spatial moving-detour attempts (warmup and measured) return the same failure.
Unprofiled timed-search stages take 44.285/44.151 s for Vietnam, versus
0.565/0.532 s for the saved detour. The much denser Vietnam time grid explains
most of the end-to-end difference.

| Case | Method | Measured times (s) | Median (s) | Result |
| --- | --- | --- | ---: | --- |
| Vietnam | Spatial | 7.083557, 6.763672 | 6.923615 | Valid, length 113.146007006023 |
| Vietnam | Timed | 52.865466, 52.580814 | 52.723140 | Valid, length 113.200668397872 |
| Saved moving detour | Spatial | 39.165264, 39.382486 | 39.273875 | No optimized feasible iterate |
| Saved moving detour | Timed | 5.149291, 5.009621 | 5.079456 | Valid, length 229.706603150283 |

Vietnam's timed method is 7.61 times slower and its path is 0.0483% longer.
The moving-detour timed method produces validated motion in 5.08 s, while the
spatial method spends 39.27 s before failing. These are feasibility outcomes
at the same deadline, not a speedup between equivalent successful motions.

### Exploratory runs

Both modes use identical physical inputs and deadlines, MATLAB R2024b, and no
plots. Every successful motion passes the public independent validator.
These initial one-shot timings include public planning and its internal
validation; extra independent validation is outside the timer.

| Case | Deadline (s) | Spatial runtime (s) | Timed runtime (s) | Spatial result | Timed result |
| --- | ---: | ---: | ---: | --- | --- |
| Supplied Vietnam, 921 slices / 280 vertices | 3000 | 9.067820 | 54.528844 | Valid, length 113.146007006023 | Valid, length 113.200668397872 |
| Saved moving detour | 180 | 41.328897 | 5.655161 | `noOptimizedFeasibleIterate` | Valid, length 229.706603150283 |

The moving-detour result is a feasibility benefit at the fixed deadline; the
failed spatial solve has no valid path length to compare. Vietnam is a runtime
and path-length regression with the timed option, so it does not justify
changing the default. Both methods retain the 230-second Vietnam duration.

## Why Vietnam is expensive

A focused search profile takes 57.24 s with profiling enabled: 1,841 layers,
23 nodes, 28,226 expanded states, and 366,038 edge-check calls. The existing
parent tie policy continues through the prescribed horizon. It returns 1,522
route points before redundant wait knots are removed. Profiling timings are
attribution evidence, not directly comparable to unprofiled wall times.

## Rejected experiment

An exact spatial lower-bound exit stopped fixed search when it found a route
as short as the endpoint distance and verified graph waits to the deadline.
Vietnam then expanded one state and completed the attempt in 15.80 s, but
BMTP failed to construct feasible motion from that different timing seed.
The graph's length bound does not certify acceleration/jerk feasibility or
the optimizer's ability to recover a trajectory. The shortcut is rejected;
the final implementation preserves the original search and tie policy.

## Reproduce and verify

**All 50 MATLAB tests passed** (0 failures, 0 incomplete), including the five
new fixed-arrival tests and all existing earliest-arrival/default regressions.

```matlab
addpath('benchmarks');
[runs,results] = benchmarkFixedArrivalSearch(2);
```

The benchmark warms each method, counterbalances measured method order, and
returns per-run timings, validity, arrival, path length, and complete results.
Input loading, warmup, and extra independent validation are outside measured
planner times. It writes no files. The default repetition count is three.
Generated evidence is kept in ignored `scratch/fixed_timed/`.

Tests cover prescribed arrival, continuous validated motion, explicit unsupported
states, invalid option values, graph exhaustion without fallback, and the saved
detour at 180 s. Existing earliest-arrival and default spatial regressions remain
part of the full suite. The timed graph remains a discrete proposal, not a proof
of continuous-time completeness or global motion optimality.
