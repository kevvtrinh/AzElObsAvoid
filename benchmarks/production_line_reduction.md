# Production line reduction - 2026-09-10

The changes from `ac0d44b` remove **807 production physical lines (6.9%)**
and fix false no-motion outcomes in fixed-clock BMTP refinement. The existing
benchmark's motion metrics and gate decisions are retained.
Of the net reduction, **165 are nonblank, non-comment-only lines**; the other
642 are help, mechanical comments, and blank-line reductions. Required public
help content and explanations of geometry and tolerances remain.

| Production scope | Before | After | Removed |
| --- | ---: | ---: | ---: |
| `+obstacleAvoidance` | 4,153 | 3,824 | 329 |
| `trajectory/+bmtpEngine` | 4,496 | 4,018 | 478 |
| `trajectory/+ruckigEngine` | 2,635 | 2,635 | 0 |
| `planner.m` | 429 | 429 | 0 |
| **Total physical lines** | **11,713** | **10,906** | **807** |
| **Total nonblank, non-comment-only lines** | **8,808** | **8,643** | **165** |

The two targeted packages shrink by 9.3% in physical lines. Production file
count increases from 82 to 85 because three duplicated constraint builders
now have shared implementations. Tests, examples, benchmark scripts, reports,
and scratch files are excluded from both line counts. The code-line metric
counts MATLAB continuation lines and lines containing executable code plus
an inline comment; it is not a statement count.

## Reduction and failure fix

- Share workspace bounds, endpoint equations, C3 joins, and derivative rows
  between the general and timed BMTP solvers, preserving their ordering and
  numerical arithmetic.
- Share exact Bernstein separating-plane products and the two time-power
  cones. The objectives, cones, bounds, solver tolerances, and iteration
  budgets are unchanged.
- Use one traversal for rebuilding and reverifying alternating-solver planes,
  retaining physical time restrictions, diagnostics, and early exits.
- Remove unused sample/interval bounding boxes, per-ring bounds, and an
  unused query flag from obstacle preparation. Preparation version 6 rebuilds
  older derived caches; source equality and requested-window coverage still
  govern reuse. Exact edges, geometry, and margins are retained.
- Check and extend each obstacle in one pass, remove a duplicate empty-input
  branch, inline one-owner plane initialization, and remove multiplication of
  physical durations by the constant one.
- Condense help and remove comments that merely repeat loop statements.

The timed solver used to discard all nonpositive solver exits, including
finite fixed-clock iterates that were physically valid. Nine inspected
single-span cases returned MATLAB exit flag -7: primal feasibility residuals
were approximately 1e-15, but dual optimality convergence stalled. Every one
of those motions passed the unchanged public independent validator when
retained for inspection.

`solveTimedTrajectoryStep` now retains finite fixed-clock -7 iterates as
proposals, matching the existing general BMTP policy. `refineTimedTravel`
considers those proposals through its existing collision discovery and
objective checks. Final continuous certification and public independent
validation still gate planner success. The original exit flag is preserved;
`OptimizationConverged` and travel-refinement diagnostics explicitly report
that optimality was not established. Earliest-arrival exit handling is
unchanged, and empty or nonfinite outputs are still rejected.

This fixes the false no-motion result; it does not claim MATLAB has proved
optimality for a stalled solve. Ruckig, `planner.m`, the public independent
validator and its range checker, and the historical workbook are
byte-for-byte unchanged. No validation, geometry, or tolerance was weakened.

## Behavioral evidence

**All 42 tests pass.** The original suite had 39 tests. New regressions cover:

- Eight formerly rejected feasible single-span timed motions across degrees
  5 and 8 and fixed durations of 5, 6, 8, and 10 seconds. Each returned motion
  passes the public independent validator, while preserving solver status.
- An analytically infeasible single-span clock that must return no motion.
- A nonuniform three-span clock with nonzero endpoint velocity and
  acceleration, including C3 jerk checks and row-vector duration ratios.

Existing regressions also cover direct motion, moving and static obstacle
detours, new collision-plane discovery during travel refinement, explicit
no-path outcomes, invalid inputs, margin/cache behavior, and independent
validation.

Across three repetitions of all 19 maintained examples, all 57 complete
example records and 63 subcase records have exactly equal CSV values for
every field except measured times. Validity, termination reason, duration,
executable motion length, and reported route length are unchanged. This
includes the expected explicit no-path result and all three geographic
regions. Each run is independently validated.

Historical quality decisions remain 10/19, including the additional dense
moving case that has no historical reference. `MeetsAll` remains false for
all rows: the retained benchmark also requires fewer than 7,000 production
physical lines. Existing historical quality misses remain visible. The
workbook, runner, thresholds, and timing-contract check are unchanged.

Direct MATLAB comparisons capture original and final conic solver inputs.
Objectives, inequalities, equalities, bounds, and every cone's A/b/d/gamma
fields match exactly across nine cases: relaxed clocks, nonzero endpoint
states, unequal span times, partial plane intervals, the intrinsic-variation
transformation, timed earliest/fixed arrival, a single span, and row-vector
duration ratios.

## Runtime observations

MATLAB R2024b, identical default example inputs, separate fresh sessions,
three repetitions, and six computational threads in both versions. Timers
include example construction, planning, and example-level validation, with
plotting and profiling disabled. Additional validation and serialization are
outside the timers. Baseline examples ran before the baseline tests; final
examples ran after the final tests. Medians reduce cold-start sensitivity
but do not establish identical warmup state or a universal speed guarantee.

The sum of per-example median times is **112.0060 s before and
108.2280 s after (3.4% lower)**. This is a sum of medians,
not the median of whole-suite wall times. 4 examples are slower by
0.3-7.4%; the measured increases remain in the table.

| Example | Before median (s) | After median (s) | Change |
| --- | ---: | ---: | ---: |
| `exampleAlternatingSlalom` | 1.1331 | 0.9488 | -16.3% |
| `exampleDenseConcaveObstacle` | 1.4543 | 1.2204 | -16.1% |
| `exampleFourAcceleratingCircles` | 2.6081 | 2.3550 | -9.7% |
| `exampleInterceptMovingTargetAtSetTime` | 0.0321 | 0.0227 | -29.3% |
| `exampleInterceptMovingTargetEarliest` | 0.1889 | 0.1300 | -31.2% |
| `exampleMovingBarrierWait` | 14.0971 | 13.2015 | -6.4% |
| `exampleMovingCircleNoWrap` | 0.4332 | 0.3878 | -10.5% |
| `exampleMovingDeformingUSOutlineVisibility` | 15.4412 | 13.7565 | -10.9% |
| `exampleMovingRotatingObstacleField` | 0.3365 | 0.3155 | -6.2% |
| `exampleNoPath` | 0.0442 | 0.0374 | -15.3% |
| `exampleObstacleFree` | 0.0354 | 0.0340 | -3.9% |
| `exampleOpeningUShapedObstacle` | 18.2529 | 17.9720 | -1.5% |
| `exampleStaticUShapedObstacle` | 7.9322 | 8.5191 | +7.4% |
| `exampleStraightTargetAlternatingOcclusion` | 0.7040 | 0.7064 | +0.3% |
| `exampleTargetExitsObstacle` | 0.7012 | 0.6887 | -1.8% |
| `exampleTwoOpposingUVisibilityGraph` | 1.2490 | 1.2670 | +1.4% |
| `exampleUSOutlineExtremeVisibility` | 30.4775 | 29.8656 | -2.0% |
| `exampleVietnamKeepoutSlew` | 5.7782 | 5.9060 | +2.2% |
| `exampleMovingObstacle220` | 11.1071 | 10.8937 | -1.9% |

Three samples do not reliably distinguish small regressions from runtime
variation. A separate dense-moving profile of the shared formulation was
excluded from the benchmark times.

## Reproduction and local evidence

```matlab
addpath('trajectory','examples','tests');
assertSuccess(runtests('tests'));
checkBenchmarkTimingContract();
summary = runExampleBenchmarks([],3);
```

Run those commands against the original commit and this checkout with the
same MATLAB settings. The ignored `scratch/line-reduction/` directory keeps
the original source snapshot, baseline and final (`fixed_*`) CSV/MAT results,
exact conic comparisons, stalled-iterate diagnostics, line-count audit,
analyzer results, and separate profile. No generated benchmark outputs are
included in source control.
