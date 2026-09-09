> Historical audit of the C2 implementation committed as `bdb3a65`.
> See C3_QUINTIC.md for the later C3 quintic change.

# build-core implementation and audit

Implemented on `build-core`, starting at `26c343b9ab7704d0cd5a3d8f72af058e99c936b1`
of `bmtp-emptycore`. MATLAB R2024b measurements were made on September 8, 2026.
The existing working-tree edit to AGENTS.md was preserved.

The shared BMTP fixed-clock equations now prescribe full initial and terminal
position, velocity, and acceleration. Endpoint Bernstein controls use each
boundary span's own physical duration. Warm starts, conic constraints, final
reconstruction, polynomial export, and independent validation all preserve
those states. Interior joins remain C2 without imposed waypoint stops.
Nonzero-state motion cannot use post-solve time dilation. An over-limit
analytical proposal reaches the shared optimizer instead of being accepted.

Static earliest arrival integrates the initial full state as an affine offset
in the existing cubic jerk basis. Initialization and final repair solve at
actual fixed durations; joint phase-time optimization retains its analytical
gradients and Hessian with the full terminal-state residual. Rest-only chord,
clock, corridor, reachability, and delayed-departure methods remain eligible
only for exact rest states. Numerical convergence is reported separately from
independently checked feasibility.

Endpoint checks now use start occupancy at the start time and fixed terminal
occupancy at the terminal time, plus workspace, derivative, and necessary
travel-time bounds. A future goal occupied only in the initial snapshot uses
an explicitly unsearched direct guide with authoritative affine obstacle cells;
it is not represented as an exact obstacle-free visibility graph.

Target velocity/acceleration matching differentiates the declared linear or
PCHIP interpolant at the actual arrival time. Matching defaults off, preserves
explicit derivatives, and rejects conflicts. Scalar limits consistently mean
combined magnitudes allocated as `[L/sqrt(2), L/sqrt(2)]`; vectors remain per-axis.
Supplied and resolved limits are retained. Periodic x/y coordinates support
obstacle-free fixed-position goals with continuous unwrapped motion and wrapped
plots. Earliest dynamic/target fallback searches chronological fixed-time
trials with explicit resolution, budget, source boundaries, and unsearched gaps.
Every accepted trial passes the public independent validator.

All applicable production callers were migrated. The trajectory-step interface
accepts full states directly. The example option resolver forwards matching
and temporal controls; four target examples now call the public planner without
obsolete interception-option records. Existing examples already used explicit
per-axis limits, so their physical cases and the reference workbook were retained.
A test exercises matching through the maintained specified-time target example.

The validator additionally checks terminal metadata, arrival/duration metadata,
sample-time coverage, supplied/resolved limit consistency, periodic endpoint
selection and workspace policy, and matched target derivatives against source
history. Geometry, obstacle margins, and validation tolerances were not relaxed.

## Behavioral and benchmark evidence

The destination baseline passed 64 tests. The changed branch passed the complete
83-test suite and the benchmark timing-contract check. Additional acceleration-only,
velocity-only, and mixed endpoint combinations were added to the existing full-state
test and checked in the final focused run. Tests cover negative derivatives,
initial motion away from the goal, unequal limits, coordinate/axis/time transforms,
static and moving detours, more than two guide segments, target derivative matching,
periodic plots and disabled-axis bounds, exhausted time-search budgets, expected
no-path/infeasible outcomes, and source/output tampering. Existing continuous
between-sample collision and source/activity tampering tests remain passing.

All 18 maintained examples were run three times on both the preserved baseline
and changed source, including every geographic subcase. Every expected outcome
and independent validation passed. Arrival and executable polynomial motion
length are unchanged from the fresh baseline at the runner's full recorded
precision. No reference workbook measurements were changed.

The final run passes 17/18 historical all-gates checks. The alternating-occlusion
runtime is 2.4860634 s versus a 2.4666327 s historical limit (+0.79%); the fresh
baseline also missed that limit at 2.4810943 s. An earlier changed-branch run
passed all 18 gates, including 2.4580578 s for this case. Both results are retained;
this audit does not claim an unconditional runtime improvement.

| Example | Arrival (s), unchanged | Motion length, unchanged | Fresh baseline median (s) | Final median (s) |
| --- | ---: | ---: | ---: | ---: |
| `exampleAlternatingSlalom` | 10.500000 | 16.020012 | 0.561886 | 0.305078 |
| `exampleDenseConcaveObstacle` | 8.500000 | 12.755775 | 0.764288 | 0.729602 |
| `exampleFourAcceleratingCircles` | 22.000000 | 20.000000 | 4.486181 | 4.047008 |
| `exampleInterceptMovingTargetAtSetTime` | 12.000000 | 9.538941 | 0.031020 | 0.017660 |
| `exampleInterceptMovingTargetEarliest` | 6.111111 | 7.308890 | 0.048781 | 0.020449 |
| `exampleMovingBarrierWait` | 10.090089 | 10.000000 | 0.117823 | 0.063270 |
| `exampleMovingCircleNoWrap` | 8.500000 | 12.448798 | 0.443628 | 0.292501 |
| `exampleMovingDeformingUSOutlineVisibility` | 7.916667 | 40.238053 | 19.797807 | 17.015034 |
| `exampleMovingRotatingObstacleField` | 9.041667 | 20.455931 | 0.444452 | 0.417621 |
| `exampleNoPath` | expected no path | — | 0.104751 | 0.056571 |
| `exampleObstacleFree` | 4.531129 | 4.472136 | 0.019446 | 0.018283 |
| `exampleOpeningUShapedObstacle` | 11.584333 | 10.000000 | 0.171397 | 0.144265 |
| `exampleStaticUShapedObstacle` | 20.762801 | 37.779253 | 6.413035 | 6.237680 |
| `exampleStraightTargetAlternatingOcclusion` | 20.869565 | 13.556360 | 2.481094 | 2.486063 |
| `exampleTargetExitsObstacle` | 24.000000 | 20.504272 | 1.166462 | 1.296570 |
| `exampleTwoOpposingUVisibilityGraph` | 21.633333 | 24.048035 | 0.387663 | 0.447678 |
| `exampleUSOutlineExtremeVisibility` | 5.204940 | 18.801408 | 24.386355 | 22.998475 |
| `exampleVietnamKeepoutSlew` | 30.000000 | 17.144141 | 2.880222 | 2.928327 |

Paired follow-up measurements alternated baseline/current source with one
warm-up per measurement, three measured repetitions each, and profiling off.
They retained identical arrival and motion length and passed validation.

| Case | Paired baseline (s) | Paired current (s) | Change |
| --- | ---: | ---: | ---: |
| `exampleTargetExitsObstacle` | 1.167476 | 1.189133 | +1.86% |
| `exampleOpeningUShapedObstacle` | 0.302110 | 0.312981 | +3.60% |
| `exampleTwoOpposingUVisibilityGraph` | 0.546585 | 0.576347 | +5.44% |
| `exampleStraightTargetAlternatingOcclusion` | 2.528984 | 2.474145 | -2.17% |

The repeatable modest slowdowns are retained and reported. A separate profiled
TargetExitsObstacle call attributed 0.017085 s of self time to the new endpoint
preflight; occupancy queries used 0.047895 s across six calls, and two public
validator calls used 0.105208 s of self time. Profiling was not used for the
benchmark medians. This evidence supports retaining the new physical checks
with an explicit runtime cost, rather than claiming all cases became faster.

## Size and dependencies

Physical lines include comments and blank lines. Production includes planner.m
and every MATLAB file in all production packages; tests/examples are counted
separately. No code was moved to excluded folders to reduce these counts.

| Scope | Before | After | Added | Removed | Net |
| --- | ---: | ---: | ---: | ---: | ---: |
| Production | 5,462 | 5,894 | 485 | 53 | +432 |
| Tests | 1,422 | 1,672 | 251 | 1 | +250 |
| Examples | 2,758 | 2,757 | 15 | 16 | -1 |

Production function declarations increase from 115 to 119: the shared physical
endpoint-control helper, endpoint preflight checks, chronological arrival search,
and a local wrapped-display helper. No production functions were removed.
Four options were added: MatchTargetVelocity, MatchTargetAcceleration,
TemporalResolution_s, and MaxArrivalTrials. Existing WrapX/WrapY controls are now
implemented. MATLAB and Optimization Toolbox remain the only requirements;
no Ruckig or other engine/dependency was introduced, and the donor branch was not merged.

Removed/replaced code includes rest-only endpoint equations and reconstruction
assignments, hard-coded zero terminal derivatives, the position-only trajectory-step
interface, and obsolete example interception records. Generalizing the existing
Bernstein and integrated-jerk representations supplied the core capability;
there is no second motion pipeline or collection of case-specific recovery methods.

## Remaining limits and reproduction

The spatial guide and finite polynomial family are not a complete topology
search. Static earliest optimization is local, not a global earliest proof.
Chronological time trials leave open intervals unsearched; disconnected feasible
times can be missed, especially after budget exhaustion. No solver failure is
promoted to proof of physical infeasibility. Tight 3.0/3.3 s exploratory requests
returned `noOptimizedFeasibleIterate`; that unfavorable evidence is preserved.
The inherited distinct-position input restriction remains.

Equal scalar-limit allocation is conservative. Periodic obstacles and targets
are unsupported, and nearest equivalent coordinates do not imply globally
minimum time. Linear target corners reject derivative matching; PCHIP uses
right-hand pieces at interior knots and the left-hand piece at the final knot,
including one-sided acceleration. Optional richer occupancy-query details were
not added. Independent validation remains mandatory for success.

Run `assertSuccess(runtests('tests'))`, `checkBenchmarkTimingContract()`, and
`runExampleBenchmarks([],3)` after adding the root, trajectory, examples, and
tests folders to the MATLAB path. Benchmark CSVs, logs, profiles, and the source
snapshot are ignored local artifacts under scratch/ and benchmarks/. Evidence:
`build_core_baseline_tests.log`, `build_core_baseline_examples.log`,
`build_core_first_summary.csv`, `build_core_final_audit.log`, and
`build_core_runtime_check.log` in scratch/. The final diff was reviewed and
`git diff --check` passed.
