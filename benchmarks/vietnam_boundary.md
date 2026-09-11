# Supplied Vietnam boundary benchmark

The source CSV and implementation plan were committed as `3235d57` on
`build-core`, on top of `b35b4e9`. The CSV preserves all 842 pasted pairs and
their decimal text. It contains 280 rows at 2770 s and 281 rows at each of
2910 and 3000 s. The latter two include an exact repeated closing point.
The fixture removes only those two copies for computation, then interpolates
each corresponding vertex over the two anchor intervals. The planner input
has exactly 921 slices at 0.25 s spacing, each with 280 vertices.

## Reproduce

```matlab
addpath(pwd,fullfile(pwd,'examples'),fullfile(pwd,'trajectory'));
[obstacles,initialState,goalState,limits] = createVietnamBoundaryScenario();
options = struct('GoalTimeMode','fixedArrival');
planner(obstacles,initialState,goalState,limits,options); % warmup
elapsed_s = zeros(3,1);
for repeatIndex = 1:3
    timer = tic;
    result = planner(obstacles,initialState,goalState,limits,options);
    elapsed_s(repeatIndex) = toc(timer);
    assert(result.Success && obstacleAvoidance.validateTrajectory(result).Passed);
end
median(elapsed_s)
```

For plots, run `exampleVietnamBoundarySlew()`. This is a new supplied-data
fixture, separate from the historical eight-obstacle `exampleVietnamKeepoutSlew`.
It is also registered in `runExampleBenchmarks` without borrowing historical
reference metrics from a different case.

## Measurements

MATLAB R2024b, same machine and identical inputs, profiling and plotting off;
one warmup and three measured planner calls per version in separate MATLAB
processes. The timer includes public planning and its independent validation.
CSV loading/interpolation and an additional independent validation are outside
the timer. The baseline and candidate groups were sequential, not interleaved.

| Metric | Baseline `b35b4e9` core | Area-bound shortcut |
| --- | ---: | ---: |
| Run 1 wall time (s) | 8.942224 | 7.187836 |
| Run 2 wall time (s) | 8.681386 | 7.037435 |
| Run 3 wall time (s) | 8.864717 | 6.883449 |
| Median wall time (s) | 8.864717 | 7.037435 |
| Absolute arrival (s) | 3000 | 3000 |
| Motion duration (s) | 230 | 230 |
| Motion length (degrees in az/el coordinates) | 113.146007006023 | 113.146007006023 |
| Independent validation | Passed, all 3 | Passed, all 3 |

Median runtime improves **20.61%**. Complete prepared-obstacle records,
visibility route arrays, and polynomial records compare exactly between
versions with `isequaln`.

## Why the change helps

The initial profile took 14.26 s. `prepareObstacles` used 6.31 s across its
nested calls; interval shape comparison used 4.14 s, including 3,680 polygon
subtractions taking 3.81 s. These are overlapping inclusive times and must not
be added. Independent validation deliberately rebuilds preparation from source.

If polygon A has greater area than B, the area of A minus B is at least their
area difference. `compareShapes` now uses this lower bound to skip a Boolean
subtraction that cannot establish containment. It retains an extra numerical
tolerance reserve and uses the original subtraction for close cases. The
existing containment tolerance, source geometry, interpolation policy, and
independent validator are unchanged. This adds only **7 production physical
lines**, of which **4 are nonblank, non-comment-only lines**.

A candidate preparation-only profile uses 920 subtractions per source rebuild,
half the previous 1,840. Preparation takes 2.15 s in that profile, including
1.09 s in interval comparison. Final runtime comparisons above have profiling
disabled.

## Motion and geometry limits

The exact visibility graph has 26 nodes. BMTP jointly optimizes eight quintic
spans; the returned polynomial has 16 spans after endpoint processing. The
speed at internal joins ranges from 0.002057676163 to 1.986444180693 degrees/s.
No internal join is forced to rest. Position, velocity, acceleration, and jerk
are continuous, with maximum residual 2.9843e-13. Existing shared C3 equations
already supplied this behavior; no waypoint-stop or waiting rule was added.

The 921 input polygons interpolate the supplied coordinates, but the current
preparer classifies all 920 concave-deformation intervals as
`conservativeEndpointConvexHull`. These explicit interval envelopes remain
authoritative for this planner run. Equal vertex counts do not establish the
actual unsupplied physical motion. Supporting continuously deforming concave
boundaries with a different occupancy contract would be a separate change.

This run honors the chat's fixed-arrival request. The public
`GoalTimeMode='earliestArrival'` override remains available and is eligible
for the existing dense-history timed proposal because the fixture has a
fixed-position goal, zero endpoint derivatives, and more than 16 intervals.
Earliest-arrival runtime and optimality are not established by this benchmark.
The fixed-clock conic solve returns a finite exit -7 iterate; as in the base
commit, complete physical and independent validation establish feasibility,
not solver convergence or global optimality.

## Verification

**All 45 MATLAB regression tests passed**, including direct motion, obstacle
detours, no-path outcomes, invalid inputs, independent rejection of altered
motion/certificates, and the new Vietnam tests.

Source tests cover counts, decimal anchor values, both interpolation intervals,
and retained original geometry. The full fixture regression verifies successful
fixed arrival, path length, exhaustive graph, nonzero internal speeds, C3 joins,
and the reported interval models. A structurally different containment regression
compares the shortcut against the original Boolean classification for growing,
shrinking, shifted, disconnected, and holed polygons in both orders.

Generated profile tables, full polynomial comparisons, timing runs, and MATLAB
test results stay in ignored `scratch/vietnam/`.
