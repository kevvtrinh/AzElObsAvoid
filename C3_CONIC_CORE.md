# C3 runtime and arrival refinement

## Runtime milestone: local conic state equations

The static quintic solver previously integrated every earlier jerk variable into
every later position, velocity, and acceleration constraint. Profile
initialization and final conic repair now use explicit knot position, velocity,
and acceleration, and share the jerk
value at each phase join. Every phase depends only on its starting state and
three quadratic-jerk controls; local equality rows connect its terminal state
to the next knot. This is the same integrated quintic feasible set and C3
continuity, with a sparse constraint matrix.

The change applies to the general static quintic solver, including ordinary
planning's final repair, profile-assisted initialization/repair, and non-rest
boundary states. Ordinary initialization keeps its previous equations so its
numerical solution does not perturb the subsequent nonconvex timing search.
It does not inspect obstacle names, scenario IDs, or example filenames. In this
first milestone, conic objectives, nonlinear timing budgets, physical limits,
obstacle geometry, clearances, validation tolerances, and public independent
validation are unchanged. The variable count increases: the measured 14-phase initialization
has 210 variables instead of 158. Reduced coupling, rather than fewer
variables, explains the speedup.

## Measurement

Baseline is commit `81a95c4`. Comparisons use MATLAB R2024b Update 4 with
Optimization Toolbox and six computational threads. Inputs and the profile
bank are identical. Eight held-out profile cases have one warmup followed by
three measured repetitions per version, with version order counterbalanced.
Timing covers the whole planner call, including input/library checking,
preparation, initialization trials, optimization, construction, and validation.
Additional independent checks and serialization are outside the timer.

A separate profile of case 1 measured 4.93 seconds overall, 2.42 seconds in five
conic solves, and 1.46 seconds in nonlinear timing optimization. These are
attribution measurements; reported runtime comparisons have profiling disabled.

All 24 measured candidate profile runs passed independent validation and used
the library without fallback. Median planner runtime falls 16.3–35.9%. Arrival
changes range from 0.009513 seconds earlier to 0.000790 seconds later. Numerical
changes in the equivalent conic problem perturb the later local nonlinear
optimization; the results are not bitwise identical. The largest path increase
is 0.3461%; the largest normalized jerk-variation increase is 0.3064%.

The exact table is in the ignored evidence file
`scratch/c3_followup/local_paired_summary.md`; all motions and trials are in
`local_paired.mat` and `local_paired.json` in that directory.

Eight additional seeded scenes vary proportions, rotation, translation,
anisotropic limits, boundary derivatives, and obstacle topology. The two
profile-assisted cases improve by 24.5% and 26.0% in the measured pair. Two
ordinary cavities improve by 4.2% and 8.6%, and the non-rest detour improves by
1.8%. Ordinary arrival times are unchanged, with path changes below 0.0001%.
Unaffected analytic/corridor cases keep identical motions. This exploratory set
has one warmup and one measured pair, not the three-repetition precision of the
main benchmark. The failed L-shaped case is 0.0416 seconds slower in the
measured pair; no performance gain is claimed for that case.

The L-shaped scene fails continuous plane certification in both versions. It
remains a reported limitation; it is not relabeled as successful or removed
from the evidence. Inputs are reproducible from seed 41073 in
`scratch/c3_followup/compareGeneral.m` and are saved in `general_inputs.mat`.

## Verification

All 147 MATLAB tests passed in the shared production checkout, including the
other task's moving-obstacle work. This covers direct motion, static and moving
detours, expected no-path outcomes, invalid inputs, independent validation,
profile caps/fallback, and the unchanged path-quality gates. A new non-rest
detour test exercises rotation, anisotropic limits, a nonzero initial time,
nonzero endpoint velocity/acceleration, and swapped axes. The exact result is
`scratch/c3_followup/production_milestone_tests.mat`; its execution log is
`jerk_and_production_tests.log` in that directory.

Generated artifacts stay ignored. This milestone commits only the conic source,
the new non-rest test, and this report; the other task's files remain separate.

## Experiments not retained

Applying local states to ordinary initialization too reduced runtime, but
changed the later nonlinear local optimum. One generated cavity arrived
0.038375 seconds later, and `testQuinticClock` exceeded its existing path cap:
39.801451 versus a required value below 39.739582 units. That version passed
136 of 137 tests and was rejected. Preserving ordinary initialization restores
its timing and passes the unchanged focused quality checks.

Removing unused length variables and avoiding construction of their unused
cones did not establish a clear runtime improvement. It also perturbed local
optimization, so it was not retained. Uniformly compressing the final clock
cannot provide useful gains here: exact polynomial extrema show at most
0.7 microseconds of unused duration in the measured profile outputs. Their
velocity or acceleration limits are already active.

Neither this formulation nor the profile library proves globally earliest
arrival. Generated measurements, MAT files, and experiment copies stay ignored.

## Arrival milestone: allocate less duration to final repair

Profile `repair` mode now assigns one third of the configured
`PathLengthTimeAllowance_s` to final length/variation repair and two thirds to
bounded timing regularization. Previously each received half. The total
allowance is unchanged; zero still removes both allowances. With the default
0.49-second allowance, final repair adds at most 0.163333 seconds instead of
0.245 seconds, before the existing numerical time reserve and horizon cap.
Ordinary planning and profile `warmStart` mode keep their previous allocation.

This is an explicit arrival/path/smoothness trade, not an equivalent-objective
speedup. Less final duration generally leaves less freedom for shortening and
smoothing the curve. The physical feasible set, constraint tolerances,
obstacle margins, and independent validator remain unchanged. The same rule
applies to every eligible profile request, independent of obstacle coordinates,
names, example IDs, or training membership.

The table compares both retained changes against original commit `81a95c4`.
It uses the same eight inputs and bank, one warmup and three measured repetitions
per version, and counterbalanced order. All 24 measured candidate outputs
passed independent validation and accepted a profile without fallback.

| Case | Baseline median (s) | Final median (s) | Runtime reduction | Arrival change (s) | Path change | Jerk variation change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 3.6529 | 2.9134 | 20.2% | -0.081633 | +0.4786% | -0.9859% |
| 2 | 3.2230 | 2.7682 | 14.1% | -0.080663 | +0.5104% | +3.4017% |
| 3 | 2.8057 | 2.2892 | 18.4% | -0.115748 | +0.3773% | +5.0937% |
| 4 | 2.2473 | 1.6995 | 24.4% | -0.080844 | +0.4533% | +10.3885% |
| 5 | 2.1618 | 1.4534 | 32.8% | -0.080809 | +0.3494% | -1.2278% |
| 6 | 3.9293 | 2.6837 | 31.7% | -0.082242 | +0.3946% | -1.0208% |
| 7 | 1.7416 | 1.3526 | 22.3% | -0.081645 | +0.7290% | -3.3631% |
| 8 | 1.6737 | 1.1423 | 31.7% | -0.081775 | +0.5338% | +3.7975% |

Timing evidence and complete returned motions are in the ignored
`scratch/c3_followup/third_paired.mat`, `third_paired.json`, and
`third_paired_summary.md`. These gains are measured on this workload, not a
guarantee for arbitrary inputs. In particular, profile matching and fallback
can cost more than ordinary planning when a bank is unsuitable.

Against the original ordinary-solver outputs on those same inputs, the final
profiles remain 0.118–0.356 seconds later, within the existing 0.49-second
comparison cap. Seven paths are shorter; one is 0.1928% longer, within the
existing 0.5% comparison cap. Jerk variation is 18.1–43.5% lower. These recorded
ordinary motions and the final profile motions were independently revalidated;
the exact comparisons are in `third_reference_quality.json`. Those empirical
comparison caps are not claims of globally optimal arrival or path length.

A fresh generated set uses seed 90017 with varied proportions, rotation,
translation, anisotropic limits, margins, obstacle topology, nonzero initial
time, and non-rest boundary derivatives. Each version has one warmup and one
measured run per scene. All eight final motions pass independent validation.
The first three accept a profile and arrive 0.0804–0.0823 seconds earlier:

| Scene | Baseline (s) | Final (s) | Runtime reduction | Arrival change (s) | Path change | Jerk variation change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 2.8447 | 3.1813 | -11.8% | -0.080434 | -0.0195% | +18.7228% |
| 2 | 3.2820 | 2.8789 | 12.3% | -0.081680 | +0.2037% | +1.5692% |
| 3 | 2.4577 | 1.7959 | 26.9% | -0.082329 | +0.6388% | +5.6848% |
| 4 | 11.3057 | 9.2260 | 18.4% | 0 | <0.0001% | +0.0162% |
| 5 | 58.5381 | 59.6305 | -1.9% | 0 | <0.0001% | -0.0009% |
| 6 | 1.7296 | 1.7221 | 0.4% | 0 | 0 | 0 |
| 7 | 0.8717 | 0.8315 | 4.6% | 0 | <0.0001% | -0.0095% |
| 8 | 1.5525 | 1.6196 | -4.3% | 0 | 0 | 0 |

Scene 1 is slower and has more jerk variation; the improvement is not uniform.
Scenes 4–8 use the ordinary solver and retain their arrival times. Scene 5 emits
near-singular matrix warnings and takes about a minute in both versions before
returning a valid motion. This numerical limitation was not suppressed or fixed
by relaxing constraints. Small timing differences in this single-pair set,
especially on unchanged paths, should not be treated as established gains.
Evidence, returned motions, and seed/input data are in `third_fresh.mat`,
`third_fresh.json`, `third_fresh_inputs.mat`, and `third_comparison.log` under
`scratch/c3_followup`.

Additional rejected arrival experiments included half-span Bernstein bounds
for velocity, acceleration, or jerk, and increasing the profile timing iteration
budget. These produced inconsistent arrival gains, slower cases, or larger path
and jerk-variation regressions. Exact half-span jerk bounds saved 0.101 seconds
in one case but arrived 0.032 seconds later and ran 5.1% slower in another.
They are not in the retained source. Removing final extra duration altogether
saved about 0.24 seconds but lengthened some paths by about 4%. A quarter-share
saved about 0.12 seconds with path growth up to 1.24%; the one-third share retains
more final repair freedom. No benchmark-specific branches were introduced.

## Final verification

All 148 distinct MATLAB tests pass on the shared production checkout. The full
suite passed all 147 existing tests. The new allowance test initially used 0.9
seconds, outside the public `[0,0.5)` input range; after correcting that fixture
to 0.3 seconds, its focused rerun passed. No production source changed between
these runs. The test exercises both zero and a custom allowance, requires an
accepted profile and independent validation, and checks that timing penalty
plus final repair allowance stays within the configured total.

The original suite results are in `production_final_tests.mat` and
`production_final_suite.log`; the focused result is in `allowance_regression.mat`
and `allowance_regression.log`. `verified_final_tests.mat` combines the unchanged
147 passing results with the corrected test's passing result. All evidence is
under the ignored `scratch/c3_followup` directory.

The final source exactly matches the measured `third` candidate. This milestone
contains only the allocation change, the allowance regression, this report, and
the allocation paragraph in `C3_PROFILE_LIBRARY.md`. The other task's moving
planner changes and its separate documentation edits remain uncommitted here.
