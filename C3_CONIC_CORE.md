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

This formulation does not prove globally earliest arrival. Generated
measurements, MAT files, and experiment copies stay ignored.
