# MATLAB planning core

Starting from `8cd15bb`, all **18 historical examples** now meet or beat their
arrival, executable motion-length, and three-run median runtime references.
The core contains **5,462 physical production lines**, including comments and
blank lines, below the 7,000-line limit. All **64 MATLAB tests** pass.
The complete audit was run on September 8, 2026, with MATLAB R2024b.

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

## Planning method

Obstacle preparation, planning, independent validation, and plotting remain
separate. Original and protected geometry are retained; obstacle margins are
applied once. Concave regions are triangulated, and adjacent faces merge only
when their exact union is convex. Visibility search uses the protected occupied
union, including holes and disconnected components. A* evaluates exact graph
edges as needed, with a Euclidean distance lower bound.

Earliest motion begins with the analytic jerk-limited direct chord and the
independent-axis timing bound. For a static monotone detour, the prepared
geometry is searched once. A sweep constructs the exact affine obstacle
boundaries facing the visibility guide. The limiting coordinate follows its
analytic clock; the free coordinate integrates quadratic Bernstein jerk through
shared position, velocity, and acceleration. Restricting these polynomials to
the source-facet intervals gives linear corridor constraints. One convex solve
minimizes length at the timing bound. When more time is required, convex time
powers determine a common dilation, followed by length optimization. Positive
Gauss-Legendre weights provide a convex quadrature of the actual speed for
this objective, avoiding the excess length of a control-polygon approximation.

This upfront representation removes repeated geometry projection and avoids
one optimization constraint block per motion-span/source-cell pair. For the
Philippines input, static geometry reuse reduces the clock graph from 1,983 to
630 nodes and the motion mesh from 20 to 8 spans. The corridor still uses all
2,193 exact convex source regions; no coastline is simplified.

Static routes that require reversing the limiting coordinate use integrated
constant-jerk cubic phases. A conic solve initializes their clock, joint
optimization varies jerk and phase times with analytic derivatives, and a final
conic solve optimizes time and length at those phase ratios. Timed moving
obstacles retain affine convex cells with their original absolute activity
intervals. The fixed-clock formulation minimizes length under those cells;
analytic departure scheduling also handles applicable earliest dynamic chords.
No example names or geographic identities select production behavior.

Every returned motion is C2: position, velocity, and acceleration are continuous.
Bounded jerk jumps are allowed, as requested. Exact jerk-limited timing reaches
the obstacle-free reference of 4.531128874149275 s; enforcing continuous jerk
would prevent attaining that bound. Curves are exported in the shared
degree-eight polynomial format, preserving analytic low-degree powers.

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
assert(all(summary.MeetsAll));
```

The unchanged references are in `benchmarks/bmtp_emptycore_benchmark.xlsx`.
The runner times the entire example with plotting disabled and profiling off,
uses identical inputs, reports all runs, and requires independent validity.
Runtime gates use the three-run median, not a cold-start guarantee. Physical
production lines include the root entry point and all `.m` files in both
production packages; tests, examples, and scratch experiments are excluded.
Generated CSV/MAT/log artifacts remain outside source control.

The geographic example executes Hawaii, Croatia, and the Philippines in order.
Every regional result is captured and independently validated. Its historical
arrival and length row describes the final Philippines result; its runtime
covers the complete three-region example. All nine regional results in this
three-run audit passed. The no-path example passes only with the expected
explicit no-route outcome and no returned motion.

The workbook's seed-route length may describe a blocked direct chord, while its
returned motion detours around obstacles. The audit therefore compares
executable polynomial motion length for path quality. It also reports the seed
length separately. `MotionLength_units` now uses adaptive integration of speed
on each polynomial span, independent of output sampling. This strengthens the
comparison against historical lengths measured from sampled output. A coarse
sampling regression verifies that the reported arc length remains unchanged.

All values below are **current / reference**. Every row passes all gates.

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
topology changes after the initial snapshot. Earliest moving-target planning
certifies the kinematic interception bound; later meeting times are not searched
when obstacles prevent certification there. These cases return stable failure
outcomes rather than weakening independent validation.
