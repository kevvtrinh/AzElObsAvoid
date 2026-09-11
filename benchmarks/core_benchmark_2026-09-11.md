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
