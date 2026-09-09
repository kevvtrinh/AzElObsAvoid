# C3 quintic motion

The timing table below records the original C3 implementation. Subsequent
runtime changes and current measurements are in [RUNTIME_OPTIMIZATION.md](RUNTIME_OPTIMIZATION.md).

The previous full-state implementation was committed first as `bdb3a65` on
`build-core`, descended from `26c343b`. This subsequent change is uncommitted.
The pre-existing user edit to AGENTS.md is preserved and excluded from that commit.

## Contract and implementation

Every returned position span has six ascending coefficients (degree five).
Position, velocity, acceleration, and jerk are continuous at every internal join.
Endpoint position, velocity, and acceleration retain their public semantics;
jerk is free at the outer endpoints. Stationary waits join with zero jerk.
The public independent validator rejects C2-only curves even when their
endpoints, sampled histories, and derivative equations are otherwise correct.

Fixed-time BMTP now imposes four continuity orders on quintic Bernstein spans.
Uniform static time spans avoid amplifying roundoff on short polygon edges.
Moving geometry keeps its source-event partition. Static variable-time motion
integrates quadratic jerk with shared endpoint jerk, analytic Jacobians, and an
adjoint Hessian, then performs a fixed-clock conic repair. Rest-state repair
allows a 1e-5 relative time reserve within the horizon to avoid solving length
at a degenerate conic time bound; nonzero-state repair retains its exact clock. Monotone static corridors keep
all original source facets and enforce their exact clock-event restrictions.
A positive triangular convolution smooths the old bang-bang chord into quintic
motion for corridors and waiting schedules. Its width is one tenth of the
shortest original phase; it adds twice that width to the chord duration and
preserves derivative bounds and total displacement mathematically.

A quintic has too few coefficients to prescribe p/v/a/j at both ends independently.
The exporter instead projects the whole spline onto shared C3 equations while
preserving outer p/v/a. Corrected curves are rechecked against the original
geometry and unchanged continuous physical limits. Prescribed integrated
powers retain their low-degree algebra. The conic geometry constraints include
an additional numerical reserve of ten constraint-tolerance units. Exact selective subdivision may resolve a
missing Bernstein separation certificate without changing the curve.
No geometry, margin, clearance requirement, or validation tolerance is relaxed.

## Timing and limitations

Continuous jerk changes the feasible motion class. Historical bang-bang C2
minimum-time references generally cannot be attained by C3 motion. Direct
rest-to-rest requests use the minimum-jerk quintic family and exact peak bounds;
this is not a global earliest-time proof. Dynamic/target searches report their
finite chronological trials and unsearched intervals. Departure scheduling
optimizes the delay of its selected smoothed chord, not all possible motions.
Nonlinear detour optimization remains local and may stop at its work limit.
Coordinate exchange can change which local detour is returned.
C2 axis minimum times remain valid necessary bounds for rest states because
allowing jerk jumps enlarges the feasible set; they reject impossible trials
without claiming an attainable C3 optimum. Chronological trials reuse only
source-checked prepared geometry. The last visibility graph is cached only
when every scene, endpoint, limit, and option input is exactly equal.

The unchanged benchmark workbook continues to expose historical duration,
length, and runtime misses. BUILD_CORE.md records the earlier C2 audit, not
measurements of this change. Generated test logs and CSVs stay ignored.

## Performance diagnosis

A focused MATLAB profile of a geographic trial at 8 seconds attributed
24.75 of 26.41 seconds to visibility construction (profiling enabled).
Repeated chronological trials had identical spatial graph inputs. The exact
input cache removes that repeated work, and necessary axis-time bounds skip
physically impossible trials before search.

In the waiting scheduler, 16 source cells triggered 23,940 progress inversions.
Most queries were already known phase endpoints. Handling those endpoints
directly and using Horner evaluation for the remaining 48-step bisections
reduced the same unprofiled 16-cell calculation from 2.893 to 0.470 seconds.
The returned controls, times, and powers agreed within 1e-10. The corresponding
profiled timings were 10.014 and 1.070 seconds; profile timings are not used as
end-to-end runtime gates. The complete 108-cell departure calculation took
4.273 seconds with profiling enabled and correctly reported no usable delay.

## Verification

MATLAB R2024b with Optimization Toolbox is the behavioral reference.
The complete regression suite and focused rechecks cover **87 passing tests**.
Coverage includes direct and detouring motion, nonzero endpoint derivatives,
invalid inputs, expected no-path outcomes, moving targets and obstacles,
wrapping, stationary waiting, and independent rejection of jerk jumps,
incorrect degrees, weakened certificates, and changed source geometry.

**All 18 unchanged examples passed their independent scenario validation.**
The geographic sequence includes all three regional results, each validated.
Against the unchanged historical workbook, 10/18 meet the quality gates and
7/18 meet all gates. The expected no-path case is counted as a valid outcome.
These counts do not conceal the slower runtimes or later arrivals below.

The C2 column is the saved three-run median at `bdb3a65`; C3 is one complete
profile-disabled run on identical physical inputs, not a three-run timing
certification. Source geometry, examples, and workbook references are unchanged.
Production physical lines are 5,999, versus 5,894 at the committed baseline.
Final diff whitespace checks passed; generated artifacts remain ignored.

| Example | C2 duration (s) | C3 duration (s) | C2 length | C3 length | C2 wall (s) | C3 wall (s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `exampleAlternatingSlalom` | 10.5 | 10.6 | 16.02 | 16.0386 | 0.305078 | 1.06664 |
| `exampleDenseConcaveObstacle` | 8.5 | 8.6 | 12.7558 | 12.7805 | 0.729602 | 1.43166 |
| `exampleFourAcceleratingCircles` | 22 | 22 | 20 | 20 | 4.04701 | 4.69362 |
| `exampleInterceptMovingTargetAtSetTime` | 12 | 12 | 9.53894 | 9.53894 | 0.0176601 | 0.0450168 |
| `exampleInterceptMovingTargetEarliest` | 6.11111 | 6.5 | 7.30889 | 7.38694 | 0.0204488 | 0.15 |
| `exampleMovingBarrierWait` | 10.0901 | 10.1401 | 10 | 10 | 0.0632704 | 0.127268 |
| `exampleMovingCircleNoWrap` | 8.5 | 9.5 | 12.4488 | 14.4241 | 0.292501 | 0.476467 |
| `exampleMovingDeformingUSOutlineVisibility` | 7.91667 | 8.5 | 40.2381 | 40.2983 | 17.015 | 49.5966 |
| `exampleMovingRotatingObstacleField` | 9.04167 | 10.5 | 20.4559 | 20.7935 | 0.417621 | 1.00594 |
| `exampleNoPath` | — | — | — | — | 0.0565714 | 0.0389205 |
| `exampleObstacleFree` | 4.53113 | 4.93242 | 4.47214 | 4.47214 | 0.0182826 | 0.112452 |
| `exampleOpeningUShapedObstacle` | 11.5843 | 11.6143 | 10 | 10 | 0.144265 | 0.263696 |
| `exampleStaticUShapedObstacle` | 20.7628 | 20.7734 | 37.7793 | 38.3494 | 6.23768 | 43.4851 |
| `exampleStraightTargetAlternatingOcclusion` | 20.8696 | 20.8696 | 13.5564 | 13.558 | 2.48606 | 4.06088 |
| `exampleTargetExitsObstacle` | 24 | 24 | 20.5043 | 20.5057 | 1.29657 | 2.11997 |
| `exampleTwoOpposingUVisibilityGraph` | 21.6333 | 21.6933 | 24.048 | 24.0761 | 0.447678 | 0.807964 |
| `exampleUSOutlineExtremeVisibility` | 5.20494 | 5.25494 | 18.8014 | 18.8124 | 22.9985 | 43.4384 |
| `exampleVietnamKeepoutSlew` | 30 | 30 | 17.1441 | 17.1413 | 2.92833 | 8.28542 |

For example, the moving U.S. case took 49.60 s versus 17.02 s at the C2
baseline, and the static U case took 43.49 s versus 6.24 s. Those regressions
remain; this change does not claim to preserve the former runtime targets.
The direct obstacle-free motion takes 4.9324 s instead of 4.5311 s in the
chosen quintic family. Nonlinear optimization and chronological sampling
also affect arrival and path quality independently of the continuity change.

Reproduce the checks with `runtests('tests')` and
`runExampleBenchmarks([], 1)` after adding `trajectory`, `examples`, and `tests`
to the MATLAB path. Local evidence is in `scratch/c3_test_summary.csv`,
`scratch/c3_final_postprofile_tests.mat`, `scratch/c3_final_performance.log`, and
`scratch/c3_benchmark_summary.csv`.
