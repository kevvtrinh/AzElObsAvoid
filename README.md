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
regions are covered by exact polygon triangulation. Visibility search operates
on the occupied union and evaluates graph edges as A* expands nodes, preserving
the Euclidean shortest-route objective. BMTP uses nonuniform segment durations
and retains continuous position, velocity, acceleration, and jerk. Fixed-time
dynamic motion uses convex regions with absolute activity intervals. Spline
knots align with source events, and one elastic convex problem minimizes
control-polygon length at the actual requested clock. Supporting axes provide
separating planes analytically. Acceptance requires independent reconstruction
of source geometry, active pairs, clearance, endpoints, and polynomial limits.

Current limitations: dynamic earliest-arrival motion and target interception
are not yet implemented. Initialization uses an initial-time visibility route,
so the planner is not complete for dynamic topology changes or goals blocked
only at the initial time. The full static suite still needs improvement.
Historical example interfaces are being migrated to the single `planner`
entry point. No universal trajectory optimality or runtime guarantee is claimed.

## Verification

```matlab
addpath('tests');
assertSuccess(runtests({'tests/testPlanningCore.m','tests/testVietnamSlew.m'}));
checkBenchmarkTimingContract();
summary = runExampleBenchmarks({'exampleVietnamKeepoutSlew'}, 5);
```

The runner preserves example inputs and workbook references, reports failures,
uses median end-to-end example wall time, and counts all production `.m` lines
(including comments/blanks) in the root entry point and production packages.
Generated reports remain ignored by Git. The geographic sequence contains
three internal requests; it still requires per-request capture before claiming
full benchmark coverage from the historical final-result row.

## Timing contract conflict

`exampleObstacleFree` requests x displacement 4, velocity bound 2, acceleration
bound 1, and jerk bound 2. The exact minimum rest-to-rest duration is

```
T = A/J + sqrt((A/J)^2 + 4D/A) = 4.531128874149275 seconds.
```

Peak velocity is 1.765564437, below the velocity bound. Equality requires the
bangbang jerk schedule `+J, 0, -J, 0, +J`, with instantaneous changes. The workbook
reference equals this bound within floating-point precision. The empty-core
validator requires jerk continuity, so no continuous-jerk trajectory can attain
that exact bound. Resolving the benchmark goal therefore requires an explicit
choice between revising this timing reference and permitting bounded jerk jumps.
No validation tolerance has been increased to conceal the conflict.

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

Vietnam and the expected no-path case have demonstrated all reference metrics.
The full suite remains unfinished. No scenario-specific fallback was added.
