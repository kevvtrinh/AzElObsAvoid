# Core benchmark - 2026-09-11

This report records the final cleanup and certification-reuse change on branch
`build-core`. MATLAB was the behavioral reference. Every successful result was
checked again with `obstacleAvoidance.validateTrajectory`; the expected no-path
case was required to return no motion and `noVisibilityRoute`.

## Maintained examples

`runExampleBenchmarks([],3)` executed all 20 maintained examples three times:
60 top-level runs and 66 recorded subcase runs. All 20 examples passed their
expected outcome on every repetition. Times below are whole-example medians.

| Example | Valid | Duration (s) | Length | Wall time (s) | Historical quality | Historical runtime |
|---|---:|---:|---:|---:|---:|---:|
| Alternating slalom | yes | 10.5256 | 16.0349 | 1.9132 | no | yes |
| Dense concave obstacle | yes | 8.5254 | 13.4130 | 1.1422 | no | yes |
| Four accelerating circles | yes | 22.0000 | 20.0000 | 2.3015 | yes | yes |
| Intercept moving target at set time | yes | 12.0000 | 9.5389 | 0.0305 | yes | yes |
| Intercept moving target earliest | yes | 6.5000 | 7.3869 | 0.1501 | no | no |
| Moving barrier wait | yes | 10.1401 | 10.0000 | 12.9770 | no | no |
| Moving circle, no wrap | yes | 9.0000 | 12.0910 | 0.4119 | no | yes |
| Moving/deforming US outline | yes | 8.5000 | 40.2396 | 13.0160 | no | yes |
| Moving rotating obstacle field | yes | 9.5000 | 20.6687 | 0.3164 | no | yes |
| No path | yes | - | - | 0.0384 | yes | yes |
| Obstacle free | yes | 4.6311 | 4.4721 | 0.0313 | no | yes |
| Opening U-shaped obstacle | yes | 11.6143 | 10.0000 | 17.6380 | yes | no |
| Static U-shaped obstacle | yes | 20.8452 | 39.3457 | 7.3856 | no | no |
| Straight target, alternating occlusion | yes | 20.8696 | 13.5668 | 0.6910 | yes | yes |
| Target exits obstacle | yes | 24.0000 | 20.5146 | 0.6839 | yes | yes |
| Two opposing U obstacles | yes | 22.1004 | 24.1407 | 0.7894 | yes | yes |
| US outline extreme visibility | yes | 5.2642 | 18.8320 | 42.8925 | yes | no |
| Vietnam keepout slew | yes | 30.0000 | 17.1401 | 5.9029 | yes | yes |
| Moving obstacle 220 | yes | 230.0000 | 121.5030 | 11.3970 | yes | yes |
| Vietnam boundary slew | yes | 230.0000 | 113.1460 | 9.1040 | yes | yes |

Aggregate results:

- independently valid expected outcomes: 20/20;
- historical quality gates: 11/20;
- historical runtime gates: 15/20;
- both historical quality and runtime: 9/20;
- sum of per-example median wall times: 128.8134 s;
- median of per-example medians: 1.5277 s;
- maximum median: 42.8925 s;
- final production physical lines, including comments and blanks: 7,431
  (down from 10,355 before dead-code removal).

The historical workbook predates the current C3 formulation. Its gates are
reported as comparisons, not substituted for present independent validity.
Historical seed-route length is also reported by the CSV harness but is not part
of the combined gate: a seed can be a blocked direct chord, and the no-path case
has no finite route length.

## Randomized azimuth corpus

`benchmarkRandomAzimuth(1:80,"spatial",...)` ran 80 deterministic random
wide-azimuth requests twice: once with a moving obstacle and once with the same
moving obstacle plus a static obstacle. All 160 planner results succeeded and
passed independent validation.

| Variant | Valid | Runs | Median (s) | Mean (s) | Maximum (s) |
|---|---:|---:|---:|---:|---:|
| Moving only | 80 | 80 | 0.1299 | 0.1335 | 0.8872 |
| Moving plus static | 80 | 80 | 0.1816 | 0.2032 | 0.7182 |
| **Combined** | **160** | **160** | **0.1511** | **0.1684** | **0.8872** |

The fixed seed is part of `createRandomAzimuthScenario`, so the cases are
reproducible and paired. They span both azimuth directions and at least 80
degrees of azimuth.

## Optimization decision

Profiling the slowest example showed live work in active-pair BMTP, especially
trajectory SOCPs, maximum-margin plane solves, and continuous final
certification. The retained optimization passes the already prepared and
certified travel-refinement motion directly to the finalization stage. This is
safe because the certificate belongs to that exact post-projection,
post-subdivision, possibly time-dilated motion. No request, geometry, margin,
tolerance, controls, or clock changes between the two stages, and the public
independent validator still runs.

Before this handoff, the post-cleanup 20-example run had a 130.5452 s sum of
medians and a 46.2960 s maximum median. The final run measured 128.8134 s and
42.8925 s respectively. The hardest example therefore improved about 7.4%
without changing its 5.264175 s duration or 18.8320395-unit path length. The
alternating slalom also retained its 10.525609 s duration and 16.0349346-unit
path length; its median was effectively unchanged.

One smaller-looking formulation was explicitly rejected. Omitting otherwise
unused length variables and cones from time-only SOCPs reduced work on the
hardest example but changed the numerical active-pair sequence. Alternating
slalom became slower and its path grew from 16.0349346 to 16.0350643; the hard
outline path grew from 18.8320395 to 18.8379451. Fewer variables alone did not
preserve the optimizer's selected motion, so that experiment was reverted.

## Cleanup and compatibility

Seventeen unreachable MATLAB files were removed: the standalone Ruckig package,
an orphan clock-guide path, an unused reachability warm start, and an unused
target-time helper. These were real former algorithms, not empty wrappers. Their
deletion is justified by the current one-entry-point BMTP contract and absence
of repository callers; BMTP is not claimed to reproduce the standalone Ruckig
API. Git history retains those implementations.

`PathLengthTimeAllowance_s` was also removed. It formerly controlled extra-time
length refinement and jerk regularization, but those consumers had already been
removed. Supplying it now produces the existing unknown-option warning, and it
no longer appears in resolved options. The permanently false
`LowerBoundAttempt` diagnostic was removed for the same reason.

Verification after these changes: 57/57 MATLAB tests passed. Generated MAT,
CSV, profiler, and scratch outputs were excluded from source control and removed
after their aggregate results were recorded here.

## Continuous improvement: skip dominated arrival trials

Profiling the two waiting examples exposed a second, independent bottleneck.
`exampleMovingBarrierWait` made 176 trajectory SOCP calls across six failed
fixed-arrival trials after already finding a certified 10.140089-second delayed
chord. `exampleOpeningUShapedObstacle` made 246 trajectory SOCP calls across
eight failed trials after already finding its certified 11.614334-second motion.
Both searches ultimately returned the original incumbent unchanged.

The planner now constructs the exact initial visibility graph before launching
those trials. Let `L` be its shortest spatial route length and let
`norm(maxVelocity_units_s)` be an optimistic upper bound on Euclidean speed.
If there is no initial route, or `L/norm(maxVelocity_units_s)` cannot beat the
certified delayed chord, the fixed-arrival trials are skipped. This is a
necessary velocity-only bound for that exact initial route, not a claim of
global time optimality in changing geometry. If the route can beat the
incumbent, chronological search remains active.

The moving-circle regression is the structurally different control: its initial
route can beat the wait incumbent, so it still searches and returns the earlier
9-second detour.

| Example | Before (s) | After (s) | Duration unchanged | Length unchanged |
|---|---:|---:|---:|---:|
| Moving barrier wait | 12.9770 | 0.1008 | yes | yes |
| Opening U-shaped obstacle | 17.6380 | 0.1325 | yes | yes |
| Moving circle control | 0.4119 | 0.4713 | yes | yes |

The complete 20-example, three-repetition rerun remained 20/20 independently
valid. The sum of example medians fell from 128.8134 to 96.4458 seconds (25.1%),
the median of medians fell from 1.5277 to 0.7496 seconds, and historical runtime
passes increased from 15/20 to 17/20. The maximum median was 41.9489 seconds.
All 58 MATLAB tests passed after adding disconnected and finite-route-bound
regressions.

## Continuous improvement: transient implied-plane removal

The remaining dense-static profile showed many separating planes whose
trajectory inequalities overlap even though their maximum-margin update inputs
are distinct. A lifted one-source implication proof now removes only transient
arrival rows: one nonnegative scale must reconstruct both endpoint normals, and
offset dominance includes the unchanged arrival reserve and a conservative
workspace bound on floating residual. The complete upstream plane/tag set is
unchanged. Consolidation stops after the first feasible iterate, so every later
feasible-improvement and final-length solve receives the full corridor.

The cheap one-source diagnostic found 1,002 removable plane blocks among 4,173
geographic arrival-plane appearances in 1.29 seconds. A broader two-source
diagnostic found 1,627 but required 15.44 seconds and was not retained. Static U
had zero removable blocks among 349 arrival-plane appearances.

The identical 20-example, three-repetition benchmark remained 20/20 valid:

| Measure | Before | After | Change |
|---|---:|---:|---:|
| Sum of median wall times | 96.4458 s | 91.9332 s | -4.7% |
| Median of median wall times | 0.7496 s | 0.7650 s | +2.1% |
| Maximum median wall time | 41.9489 s | 36.5875 s | -12.8% |
| Historical quality passes | 11/20 | 11/20 | unchanged |
| Historical runtime passes | 17/20 | 17/20 | unchanged |
| Both historical gates | 10/20 | 10/20 | unchanged |

Every maintained duration and motion length matched the retained baseline
except the Philippines subcase. Its duration changed from 5.26417518 to
5.26417412 seconds and its length from 18.83203950 to 18.84936528 units
(+0.092%). This small path trade is recorded rather than hidden; runtime was the
stated priority. The other two geographic motions were unchanged.

The deterministic random-azimuth corpus also remained 160/160 successful and
independently valid. Combined median, mean, and maximum wall times were 0.1534,
0.1698, and 0.8590 seconds. The moving-only and moving-plus-static medians were
0.1342 and 0.1813 seconds respectively. The final MATLAB suite passed 60/60,
including two focused regressions for the lifted implication proof.
