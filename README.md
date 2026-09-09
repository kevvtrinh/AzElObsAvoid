# MATLAB planning core

The `build-core` branch starts at `bmtp-emptycore` commit `26c343b` and extends
the shared BMTP equations to prescribed position, velocity, and acceleration
at both endpoints. Reaching the terminal state ends the segment; stopping is
optional. See [BUILD_CORE.md](BUILD_CORE.md) for the implementation and measured
audit at commit `bdb3a65`. The subsequent C3 quintic change and its
verification are documented in [C3_QUINTIC.md](C3_QUINTIC.md).
The subsequent runtime investigation and controlled comparisons are in
[RUNTIME_OPTIMIZATION.md](RUNTIME_OPTIMIZATION.md).

```matlab
addpath(pwd, fullfile(pwd, 'trajectory'));
result = planner(obstacles, initialState, goalState, limits, options);
validation = obstacleAvoidance.validateTrajectory(result);
obstacleAvoidance.plotting.plotTrajectory(result);
```

`planner` is the single public planning entry point. Calling `planner()` runs
a fixed-arrival static detour. MATLAB and Optimization Toolbox are required.
Expected no-path and infeasible outcomes use stable `Success`, `Message`, and
`TerminationReason` fields.

The plotter is ported from `bmtp-cleanup-codex` (`c04f3b2`). It provides
workspace/visibility, four kinematic panels, animation, GIF export, and paired
wrapped/continuous views using this branch's retained results. Query its display
defaults with `obstacleAvoidance.plotting.plotTrajectory()`, or select views:

```matlab
handles = obstacleAvoidance.plotting.plotTrajectory(result, ...
    struct('ShowAnimation', false, 'FigureVisible', 'off'));
```

Passing Cartesian axes as the second argument keeps the existing single-view
call. The core `Axes`, `Trajectory`, and geometry handles remain available.
Moving targets use `targetMotion`; obstacle margins are not reapplied.
Legacy seed-path, swept-surface, and snapshot-count controls remain accepted
for example compatibility but have no effect because the core does not return
those diagnostic histories.

## Endpoint states, limits, and optional capabilities

Omitted or empty `velocity_units_s` and `acceleration_units_s2` default to
`[0 0]`; supplied values are preserved. Fixed-arrival BMTP constrains the first
and last span using their own physical durations. Shared joins are C3 and
do not impose stops at route-guide vertices. Motion is never
stretched after solving. Static earliest arrival integrates the full initial
state, optimizes physical jerk and phase times, and repairs at those exact
durations. This local optimization does not prove a global earliest arrival.
The default arrival mode remains `fixedArrival`.

```matlab
initial = struct('time_s',0,'position_units',[-4 0], ...
    'velocity_units_s',[-0.2 0.1],'acceleration_units_s2',[0.04 -0.02]);
goal = struct('time_s',12,'position_units',[4 1], ...
    'velocity_units_s',[0.3 -0.1],'acceleration_units_s2',[-0.03 0.02]);
result = planner([],initial,goal);
```

Velocity, acceleration, and jerk limits must all use the same scalar/vector
form. A scalar `L` is a combined magnitude, resolved once to
`[L/sqrt(2) L/sqrt(2)]`. A two-element vector is an explicit per-axis bound.
Equal allocation is conservative and cannot transfer unused capacity between
axes. Supply `[L L]` to retain the former scalar-broadcast physical case.
`SuppliedLimits` retains the supplied input, `RequestedLimits` contains resolved
physical limits, and `Limits` contains the solver workspace and resolved
physical limits. Internal calls pass resolved vectors. Maintained examples
already use explicit vectors and retain their physical inputs.

A goal may supply `targetMotion.time_s`, `targetMotion.position_units`, and
`InterpolationMethod` (`linear` or `pchip`). Set `MatchTargetVelocity` and/or
`MatchTargetAcceleration` in options to match derivatives of that same
interpolant at the actual meeting time. Both default false, preserving explicit
goal derivatives. Conflicting explicit/matched derivatives are rejected.
Linear corners have undefined derivatives and are rejected when matching is
requested. PCHIP uses the right-hand piece at interior knots and the left-hand
piece at the final knot, including one-sided acceleration. Source history and
matching policies remain available for independent validation.

`WrapX` and `WrapY` enable periodic coordinates for obstacle-free,
fixed-position goals. The corresponding workspace width defines the period;
the nearest equivalent endpoint is selected, with positive half-period ties.
This coordinate policy is not a global timing guarantee. The solver uses a
finite velocity-reachable unwrapped interval on enabled axes and preserves
bounds on disabled axes. Returned polynomials and samples remain continuous
and unwrapped; plots wrap coordinates and break lines at display seams.
Periodic obstacle and moving-target requests are explicitly unsupported.

Rest-to-rest dynamic chords can include a stationary wait joined with zero
jerk. A positive triangular smoothing kernel turns the former bang-bang
profile into continuous quadratic jerk and quintic position, increasing its
duration. Other earliest dynamic and target requests use chronological
fixed-arrival trials, with `TemporalResolution_s` (default 0.5 s) and
`MaxArrivalTrials` (default 100). Source/activity boundaries are included.
`TemporalSearch` reports trials, budget, unsearched intervals, and the absence
of a global earliest proof. `arrivalSearchExhausted` means no tested time was
certified; it does not prove physical infeasibility.

## Planning method

Obstacle preparation, planning, independent validation, and plotting remain
separate. Original and protected geometry are retained; obstacle margins are
applied once. Concave regions are triangulated, and adjacent faces merge only
when their exact union is convex. Visibility search uses the protected occupied
union, including holes and disconnected components. A* evaluates exact graph
edges as needed, with a Euclidean distance lower bound.

Returned position spans use degree-five polynomials, with continuous position,
velocity, acceleration, and jerk at every internal join. Endpoint position,
velocity, and acceleration remain prescribed; endpoint jerk is free unless
joining a stationary wait. No new snap limit is imposed.

Direct rest motion uses a minimum-jerk quintic with duration determined by its
exact velocity, acceleration, and jerk peaks. This optimizes duration within
that family, not over all C3 splines. Static monotone detours reuse exact source
facet corridors with a smoothed quintic locked coordinate and integrated quadratic jerk
in the free coordinate. Other static detours jointly optimize quadratic jerk
and phase durations, followed by a conic repair. Fixed-arrival motion enforces
C3 joins in the shared Bernstein equations. Timed obstacles retain their exact
affine cells and absolute activity intervals. No example identity selects a
production method.

Export preserves the physical clock. A global continuity projection stays in
the quintic spline space, and every corrected motion is checked again. Exact
subdivision exposes collision clearance without changing the curve. Historical
C2 timing bounds generally cannot be attained with continuous jerk; the
workbook references remain unchanged so regressions stay visible.

Finite solver iterates are proposals. Success requires the public independent
validator to reconstruct source geometry and the final motion and check
endpoints, physical derivatives, workspace, clearance, and every applicable
source pair. Static plane checks are batched with the same scalar Bernstein
products, roundoff reserve, normal bound, and clearance inequalities. Comparison
against the scalar verifier found identical decisions and signed gaps on all
48,312 geographic pairs. Randomized, near-clearance, and single-region tests
also check equivalence. No validation tolerance or protected geometry was relaxed.

## Reproduce the complete audit

```matlab
addpath('trajectory', 'examples', 'tests');
assertSuccess(runtests('tests'));
checkBenchmarkTimingContract();
summary = runExampleBenchmarks([], 3);
disp(summary(:, {'Case','Valid','MeetsQuality'}));
disp(summary(:, {'Case','WallTime_s','ReferenceWallTime_s','MeetsAll'}));
```

Inspect validity and historical quality/runtime gates separately. C3 results
and limitations are recorded in C3_QUINTIC.md. The unchanged references are in
`benchmarks/bmtp_emptycore_benchmark.xlsx`.
The runner times the entire example with plotting disabled and profiling off,
uses identical inputs, reports all runs, and requires independent validity.
Runtime gates use the three-run median, not a cold-start guarantee. Physical
production lines include the root entry point and all `.m` files in both
production packages; tests, examples, and scratch experiments are excluded.
Generated CSV/MAT/log artifacts remain outside source control.

The geographic example executes Hawaii, Croatia, and the Philippines in order.
Every regional result is captured and independently validated. Its historical
arrival and length row describes the final Philippines result; its runtime
covers the complete three-region example. All nine regional results in the
historical C2 three-run audit passed. The no-path example passes only with the expected
explicit no-route outcome and no returned motion.

The workbook's seed-route length may describe a blocked direct chord, while its
returned motion detours around obstacles. The audit therefore compares
executable polynomial motion length for path quality. It also reports the seed
length separately. `MotionLength_units` now uses adaptive integration of speed
on each polynomial span, independent of output sampling. This strengthens the
comparison against historical lengths measured from sampled output. A coarse
sampling regression verifies that the reported arc length remains unchanged.

The table below preserves the **historical `26c343b` audit / workbook reference**
values. It is not a measurement of this branch; see BUILD_CORE.md for the new
baseline and changed-branch comparison.

| Example | Arrival (s) | Motion length | Full example runtime (s) |
| --- | ---: | ---: | ---: |
| `exampleAlternatingSlalom` | 10.500000 / 10.550094 | 16.020012 / 16.034754 | 0.240 / 7.490 |
| `exampleDenseConcaveObstacle` | 8.500000 / 8.500000 | 12.755775 / 12.761105 | 0.495 / 3.835 |
| `exampleFourAcceleratingCircles` | 22.000000 / 22.000000 | 20.000000 / 20.000000 | 3.895 / 7.152 |
| `exampleInterceptMovingTargetAtSetTime` | 12.000000 / 12.000000 | 9.538941 / 9.538941 | 0.011 / 0.195 |
| `exampleInterceptMovingTargetEarliest` | 6.111111 / 6.111111 | 7.308890 / 7.308890 | 0.017 / 0.149 |
| `exampleMovingBarrierWait` | 10.090089 / 10.090302 | 10.000000 / 10.000000 | 0.055 / 1.107 |
| `exampleMovingCircleNoWrap` | 8.500000 / 8.500000 | 12.448798 / 12.453788 | 0.271 / 2.097 |
| `exampleMovingDeformingUSOutlineVisibility` | 7.916667 / 7.916667 | 40.238053 / 40.248219 | 15.087 / 30.805 |
| `exampleMovingRotatingObstacleField` | 9.041667 / 9.041667 | 20.455931 / 20.716209 | 0.359 / 2.380 |
| `exampleNoPath` | expected no path | expected no path | 0.059 / 0.567 |
| `exampleObstacleFree` | 4.531129 / 4.531129 | 4.472136 / 4.472136 | 0.016 / 0.101 |
| `exampleOpeningUShapedObstacle` | 11.584333 / 13.617522 | 10.000000 / 10.000000 | 0.142 / 1.770 |
| `exampleStaticUShapedObstacle` | 20.762801 / 20.872548 | 37.779253 / 38.678082 | 5.289 / 6.497 |
| `exampleStraightTargetAlternatingOcclusion` | 20.869565 / 20.869565 | 13.556360 / 13.610416 | 1.970 / 2.467 |
| `exampleTargetExitsObstacle` | 24.000000 / 24.000000 | 20.504272 / 20.685147 | 0.987 / 4.628 |
| `exampleTwoOpposingUVisibilityGraph` | 21.633333 / 22.100628 | 24.048035 / 24.205764 | 0.366 / 2.092 |
| `exampleUSOutlineExtremeVisibility` | 5.204940 / 5.827605 | 18.801408 / 23.257993 | 19.186 / 24.871 |
| `exampleVietnamKeepoutSlew` | 30.000000 / 30.000000 | 17.144141 / 17.305375 | 2.497 / 21.210 |

The tests cover direct motion, detours, no path, infeasibility, invalid inputs,
axis reversal and exchange, coordinate and clock translations, holes, exact
visibility, moving/deforming cells, jerk bounds, continuity, and tampering with
source geometry, activity intervals, certificates, endpoints, and histories.

## Scope and limitations

Passing this suite is a measured result, not a universal optimality or runtime
guarantee. Static optimization uses a visibility-selected topology and a finite
polynomial family. Earliest dynamic planning can miss routes that require
topology changes after the initial snapshot. Later dynamic/target meeting times are searched only at declared trial times;
feasible times may be disconnected and open gaps remain unsearched. Nonlinear
optimization and fixed-clock conic failures do not prove physical infeasibility.
These cases return stable failure outcomes without weakening validation.
