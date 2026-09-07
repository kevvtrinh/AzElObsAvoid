# Timed route handoff correction — 2026-09-06

## Decision

Retain the shared timing-handoff correction. The unmodified
`Rogue Examples/inefficientroute.mat` request now uses the early timed route
already found by search instead of the whole-history spatial detour.
Arrival falls from 123.870022798 to 112.500000000 s (9.179%); sampled motion
length falls from 291.311403232 to 228.212822569 units (21.660%).
The required gate was at least 5% earlier arrival, shorter motion, and unchanged
independent safety/kinematic/endpoint checks.

No independently timed-axis shortcut, scenario waypoint, new planner option,
validation relaxation, or safety-margin reduction was added. The three production
files add 39 net physical lines including comments. The new helper replaces the
old local time-grid function; both search and timed construction use it.

## Root cause and mechanism

The saved search already found a bend near (-43.0572, -15.5395) units at 27 s.
The adapter treated its 108 s velocity-only estimate as a fixed arrival, although
the exact rest-to-rest axis lower bound is 108.490297925 s. After that failure it
tried only the 180 s deadline, reinterpreting the interior knot's normalized time
as 45 s. The obstacle had moved and grown; that solve also failed. The remaining
validated candidate used a static convex enclosure of the complete obstacle
history, producing the unnecessary dive to -89.151995302 units y.

The repaired adapter retains the same input-derived time layers as route search,
adds the physical lower bound and advisory estimate, and excludes only times
below an applicable fixed-endpoint physical bound. Moving-target endpoints do
not use that fixed-endpoint bound. Each attempted arrival preserves the absolute
times of interior seed knots instead of stretching the whole seed. For an
earlier trial, out-of-horizon knots are omitted from the *warm start only*;
the optimized motion still has to reach the requested goal and pass validation.
Every failed arrival leaves later layers available; no monotonic-feasibility
assumption or bisection pruning is used. Fixed-arrival requests retain their
prescribed deadline.

On the supplied request the 108.4903 s basis-constrained construction still
fails, then 112.5 s passes. The new path reaches a minimum y of
-24.549682328 units. Public validation passes continuous collision resolution,
workspace, velocity, acceleration, jerk, endpoint, and continuity checks.
Reported protected-geometry clearance is 0.040321429 units, versus 0.894364201 units
for the saved detour; the original 0.2 units obstacle margin is unchanged.
Both paths use the adaptive independent collision check, not an accepted
separating-plane certificate.

A separate diagnostic witness lets y finish at 28.707424739 s while
x finishes at 108.490297925 s. It passes full validation and demonstrates
that an earlier turn is physically possible. That witness is not a production
candidate or an asserted output of this repair.

## Measurement

MATLAB R2024b Update 4, 24.2.0.2833386. Baseline is
231e02b68f81c4d5e760582fe4b8667e82011827 plus the pre-existing working changes,
including the bumpy-road jerk guard. Production source was frozen under ignored
`tmp/inefficientroute-baseline` before this experiment. Candidate uses the same
working source plus this correction. Both use identical bundle inputs, default
finite jerk limits, headless display controls, and independent validation.
The target-exits-obstacle case matches the maintained test's clearance 1e-4
and MaximumSeedCount=2 overrides.

Separate MATLAB sessions ran one warm-up and three measured calls per case,
80 actual calls in total. This is **8 of the 18 maintained examples**, plus two
saved-bundle replays; the example inventory was not reduced. The other ten
examples were not rerun for this change: exampleAlternatingSlalom,
exampleDenseConcaveObstacle, exampleInterceptMovingTargetAtSetTime,
exampleInterceptMovingTargetEarliest, exampleMovingBarrierWait,
exampleMovingDeformingUSOutlineVisibility, exampleObstacleFree,
exampleStraightTargetAlternatingOcclusion, exampleTwoOpposingUVisibilityGraph,
and exampleUSOutlineExtremeVisibility. The MATLAB unit-test suite is a separate
verification set and is not a substitute for that complete example matrix.

Wall time includes bundle/example setup and the public
planner call (including an example's own validation), but excludes the extra
independent validation performed by the measurement runner. All raw calls,
including warm-ups and expected failures, are appended to [benchmark.csv](../../benchmark.csv).
No process startup failures are represented as planner measurements.

| Case | Baseline median [min, max] s | Candidate median [min, max] s | Median change | Arrival before / after s | Motion length before / after units |
| --- | --- | --- | --- | --- | --- |
| inefficientroute | 15.072 [14.213, 16.109] | 13.809 [13.608, 15.098] | -8.4% | 123.870 / 112.500 | 291.311 / 228.213 |
| bumpyroad | 2.179 [2.039, 2.323] | 1.906 [1.789, 1.992] | -12.5% | 66.217 / 66.217 | 135.483 / 135.483 |
| exampleStaticUShapedObstacle | 6.093 [5.861, 8.332] | 6.944 [6.615, 7.063] | 14.0% | 20.850 / 20.850 | 39.384 / 39.384 |
| exampleMovingCircleNoWrap | 0.791 [0.758, 0.849] | 0.831 [0.769, 0.881] | 5.1% | 8.500 / 8.500 | 12.482 / 12.482 |
| exampleOpeningUShapedObstacle | 1.295 [1.204, 1.314] | 1.509 [1.385, 1.551] | 16.5% | 13.618 / 13.618 | 10.000 / 10.000 |
| exampleFourAcceleratingCircles | 3.269 [3.102, 3.498] | 3.762 [3.726, 4.404] | 15.1% | 22.000 / 22.000 | 20.000 / 20.000 |
| exampleNoPath | 0.331 [0.330, 0.506] | 0.332 [0.315, 0.578] | 0.2% | NaN / NaN | NaN / NaN |
| exampleObstacleAvoidance | 0.864 [0.851, 0.937] | 0.914 [0.911, 0.937] | 5.8% | 7.565 / 7.565 | 11.441 / 11.441 |
| exampleMovingRotatingObstacleField | 1.165 [1.125, 1.219] | 1.386 [1.385, 1.411] | 19.0% | 9.042 / 9.042 | 20.716 / 20.716 |
| exampleTargetExitsObstacle | 3.404 [3.241, 3.607] | 3.604 [3.388, 3.727] | 5.9% | 24.000 / 24.000 | 21.932 / 21.932 |

All nine other cases retain the same recorded lengths, durations, selected
sources, and validation outcomes. This is not a claim that every polynomial
coefficient was compared. The supplied case's median wall time falls 8.38%,
but unchanged execution paths also vary, from -12.50% to +18.97%. Therefore
no general planning-speedup claim is made. The improvement being retained is
motion quality and preservation of temporal search choices.

## Executed example and replay checks

Every row ran in finite-jerk mode in both versions. Numeric results were stable
over all four calls. 1 means pass/true; the no-path row deliberately returns no
motion, so its numeric path metrics are NaN and its motion checks are false.
The combined certificate column requires continuous collision resolution,
kinematics, endpoint/continuity, and timing; the plane column separately reports
whether an optional separating-plane certificate was accepted.

| Case | Goal mode | Planner / independent validation | Polyline units | Smoothed units | Duration s | Collision / kinematics / combined checks | Plane certificate | Termination |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| inefficientroute | earliestArrival | 1 / 1 | 228.352 | 228.213 | 112.500 | 1 / 1 / 1 | 0 | goalReached |
| bumpyroad | earliestArrival | 1 / 1 | 135.483 | 135.483 | 66.217 | 1 / 1 / 1 | 0 | goalReached |
| exampleStaticUShapedObstacle | earliestArrival | 1 / 1 | 34.943 | 39.384 | 20.850 | 1 / 1 / 1 | 1 | goalReached |
| exampleMovingCircleNoWrap | earliestArrival | 1 / 1 | 12.482 | 12.482 | 8.500 | 1 / 1 / 1 | 0 | goalReached |
| exampleOpeningUShapedObstacle | earliestArrival | 1 / 1 | 10.000 | 10.000 | 13.618 | 1 / 1 / 1 | 0 | goalReached |
| exampleFourAcceleratingCircles | fixedArrival | 1 / 1 | 20.000 | 20.000 | 22.000 | 1 / 1 / 1 | 0 | goalReached |
| exampleNoPath | earliestArrival | 0 / 0 | NaN | NaN | NaN | 0 / 0 / 0 | 0 | noValidatedSeed |
| exampleObstacleAvoidance | earliestArrival | 1 / 1 | 11.152 | 11.441 | 7.565 | 1 / 1 / 1 | 1 | goalReached |
| exampleMovingRotatingObstacleField | earliestArrival | 1 / 1 | 20.716 | 20.716 | 9.042 | 1 / 1 / 1 | 0 | goalReached |
| exampleTargetExitsObstacle | fixedArrival | 1 / 1 | 21.743 | 21.932 | 24.000 | 1 / 1 / 1 | 1 | goalReached |

## Tests and limits

The exact supplied bundle was added as a regression before production changes;
the baseline failed its arrival, length, and selected-timed-source assertions.
The repaired artifact regression passes, including preservation of the original
27 s interior waypoint for every attempted arrival. A short-event/nonzero-start
time-grid test also passes. The existing four moving-circle-plus-static-U tests
pass, explicitly exercising the timed owner as well as public planning.

The full suite ran 158 tests: 154 passed, three could not load user-deleted
fixtures (failed.mat, pathtoolong.mat, pathtoolong2.mat), and the new midpoint
test initially expected a decimal literal bitwise equal to a computed midpoint.
That test expectation was corrected to calculate from the stored endpoints;
the subsequent focused run passed both new tests. Thus 155 distinct tests have
passing evidence; the three missing-fixture tests remain unverified. No fixture
was restored, regenerated, or removed by this task. All four changed/new MATLAB
files have zero Code Analyzer messages.

This is still a discrete arrival search with local trajectory optimization,
not a global continuous-time optimum or a complete motion planner. Dense or
unsuccessful timed problems can require more solves because intermediate
arrival layers are no longer discarded. Such worst-case scaling was not
benchmarked here. The existing static-projection dispatch/fallback policy is
unchanged. No claim is made that this correction eliminates every source of
conservative behavior. Existing dirty changes remain uncommitted.
