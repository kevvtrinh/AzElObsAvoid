# Plan 325 branch assessment

The current judgment is the **Retry-exhausted boundary-support assessment** at
the end of this file. Earlier sections remain as historical evidence.

## Current corridor-only assessment — compact C3 duration controller

This section supersedes the older HS3-era judgment below for the current
uncommitted `325-less-nlp` worktree. Production contains no HS3/NLP execution.
HS3-only result and benchmark compatibility fields are now removed. The
retained dynamic controller minimizes a sampled, limit-normalized integrated
jerk quadratic while enforcing exchanged exact derivative bounds and
time-local protected safe-side constraints. It backtracks and independently
validates every retained update; eligibility depends on route dimension, not
scenario identity.

One additional bounded controller now handles compact static or dynamic topology. It
fits an eight-span doubled-knot C3 quintic to the shortest validated eligible
seed and solves fixed-duration convex minimum-jerk problems. An input-derived
probe near the physical lower bound switches between lower-bound bisection and
the established high-to-low continuation. Exact point-to-edge projection is
batched for histories proven stationary; moving dense geometry retains the
256-vertex cap. Eligibility remains input-driven: earliest arrival, three
through ten vertices, and zero-length seeds only after successful hold recovery.
It retains only a strictly shorter independently validated proposal.

This improved forty moving circles to `62.477739862636 s` versus the frozen
`64.555779916429 s` reference, with `15.3927512 s` fresh-process wall and
`0.001843121779 deg` clearance. Moving circle improved to
`8.751228736151 s`, with `6.9248772 s` wall and
`0.001266077441 deg` clearance. Both passed independent continuous collision
and exact kinematic validation. Batched stationary projection now makes dense
geographic geometry tractable: extreme outline reaches `6.222166624146 s`
versus frozen main `6.683971648809 s`, with `27.9310043 s` final wall and
`0.00709851977447 deg` independently validated clearance.

The same controller improved static basic planning to `7.649656344043 s`
and dense concavity to `8.690573182986 s`, beating both frozen main rows with
fresh walls of `4.0561583 s` and `3.9323402 s`. Ten-vertex eligibility improved
slalom to `10.855664258356 s`; feasibility-switched continuation preserves U at
`22.640860106984 s` with `5.5963246 s` wall. Opposing U now reaches
`22.160945761398 s` versus frozen main `22.875124576026 s`, with `8.5208632 s`
wall and `0.0143650783404 deg` clearance. Reshaping a recovered hold improved
moving barrier to `10.371387562474 s`. A uniformly scaled analytic jerk-switching S-curve
improved earliest intercept to `6.111534301758 s` without fixed-time regression.

The final 18-example fresh-process wall sum is `167.0127367 s`, `33.74%`
below the frozen optimized-main `252.0683835 s` matrix and `18.27%` below the
prior complete corridor-only matrix. All 17 success durations meet or beat the
frozen optimized-main row, and the expected no-path contract passed. This is
not a uniform wall-speed claim: forty moving circles, moving/deforming U.S.,
and target-exits-obstacle remain slower than optimized main in isolated wall
time.

The moving/deforming U.S. example now reaches `12.873502939647 s`, beating
the frozen `12.986386910606 s` reference by `0.112884 s`. It is independently
collision- and kinematic-valid with `0.001768850899 deg` clearance. The
final fresh-process no-plot wall is `53.0425551 s`, materially slower than the
optimized-main `26.8349249 s` row, so this is a path-time result rather
than a runtime improvement. Core production is 7,498 literal lines
excluding 565 plotting lines, meeting the user-authorized conditional ceiling;
maintained MATLAB excluding examples/scratch is 10,296 lines. The largest
production file is 881 lines.

The current full suite passed 56/56 in `28.7732939 s`, and Code Analyzer found
zero messages across 66 nonscratch MATLAB files. Visible success created three
figures and 529 graphics objects; expected no path created two diagnostic
figures with two rejected transitions. A 12-wall hairpin passed independent
validation and its corridor certificate in `9.3474237 s` total wall. These
tests establish the exercised matrix, not global optimality or completeness.

The prior exact-retimer assessment follows for historical context.

## Previous corridor-only assessment — exact derivative retimer
The retained small-system retimer derives affine velocity, acceleration, and
jerk constraints from the spline, finds exact continuous polynomial extrema,
and uses bounded convex exchange plus a secant feedback gain. It is input-driven:
the exact exchange is eligible only for static, earliest-arrival,
zero-endpoint-derivative systems with at most 100 decision-span work units.
Larger systems retain the validated span-demand controller.

The motivating U case now passes both declared gates: `23.746859860594 s` is
4.07 percent above the frozen main-branch `22.818548735851 s` reference,
and the final fresh-process headless wall was `5.9768360 s`. It passed
independent collision and exact kinematic
validation. The 12-wall hairpin remained corridor-certified and independently
valid at `164.828287993153 s` duration, `0.02 deg` clearance, and
`7.1631349 s` total wall. Opposing U improved from the prior corridor-only
`31.9439273474 s` to `28.5759222913 s`. Alternating slalom retained
`13.2008531355 s` exactly.

All 18 maintained examples ran headlessly and serially in fresh MATLAB
processes: 17 independently valid successes and one independently valid
expected no-path result. Relative to the preceding corridor-only matrix,
durations were preserved or improved. Relative to the frozen main/HS3 matrix,
the branch still trails materially on dense concavity, 40 moving circles,
moving/deforming U.S. geometry, opposing U, and extreme-outline cases. The
exact retimer therefore solves the focused U/hairpin acceptance problem but
does not yet meet or beat every historical NLP duration. No global optimality
or completeness claim is made.

The full suite passes 54/54, and Code Analyzer reports zero messages across 65
maintained MATLAB files. A visible success created four figures and 643
graphics objects; the expected no-path result created two diagnostic figures
and retained two rejected transitions. Consolidating repeated point-to-polygon
projection and one-use planner helpers reduced core production before the
retained trust step; current core is 7,200 physical lines excluding 565
plotting lines, 200 above the 7,000-line target. Maintained MATLAB excluding
examples/scratch is 9,901 lines and remains below
12,000. The production-size target and the historical all-example duration
comparison remain completion blockers. LP feasibility plus active-set QP
reduced dense-concavity wall from `63.7774602 s` to `3.7330015 s` with
bit-identical duration. The complete isolated example wall sum decreased
28.64 percent, but individual wall variation prevents a uniform speed claim.

A bounded control-law experiment confirmed that the gain is not a
scenario-independent scalar optimum. Unit log-time gain improved U to
`23.746859860594 s` but regressed planning and slalom. A continuous
error-scheduled gain improved U to `23.752030563984 s` but regressed planning
by `0.001269743990 s`; a two-band variant entered a worse `24.277638906889 s`
U basin. All variants were removed, and U recovered to
`23.801121658982 s`. The corridor QP changes geometry between timing trials,
so further controller work requires a local geometry-response Jacobian or a
trust-region acceptance test rather than scalar gain tuning.

The retained controller now performs that trust-region acceptance once: for
the first exact-exchange response it evaluates the unit and established
damped steps, retains the shorter validated response, and then resumes secant
feedback. The full 18-example fresh-process matrix preserved every prior
duration except U, which improved to `23.746859860594 s`; U wall was
`5.9768360 s`. This is bounded response selection, not a scenario branch.

## Evidence scope

This assessment covers pushed commit `2074c14` plus the current loop-free,
lazy-output evaluator, batched reconstruction, and batched seed-corridor
worktree, plus the corridor-invariant hoist, fixed-arrival speed-aware
initialization, geometry-conditioned solver choice, and constraint-feasibility
recovery. It also covers affine fixed-arrival constraints, the exact jerk
gradient, and batched complete-history envelope containment. Earlier user
changes remain preserved in the checkpoint.

- All 18 maintained examples ran serially and headlessly on the final
  combined source. Seventeen returned independently validated
  success and the no-path example returned its expected validated failure.
- The final full suite passed 59 of 59 tests. Code Analyzer checked all 56
  MATLAB files present in the worktree and reported zero messages.
- A visible success created four figures and 643 graphics objects. A visible
  expected failure created two diagnostic figures with 341 graphics objects
  and retained two rejected transitions.
- Maintained production has 30 files and 7,231 physical lines. The maintained
  planner/test tree excluding `examples/` has 33 files and 8,748 physical
  lines. The 24 example files total 3,920 lines, including the 694-line
  interactive sandbox; examples have no repository line cap and are excluded
  from planner-growth accounting.

## Current judgment

Plan 325 remains a compact, physically validated planner with materially
better runtime and earlier arrival on the affected static-visibility cases.
The current size policy keeps the 7,500-line production target and requires a
minimum 25 percent wall-time reduction for every 100 production lines above
that target; example files are uncapped but cannot justify planner growth.
The most important change is input-driven: an exact multi-obstacle visibility
seed may now receive the same early HS3 opportunity as the prior single- or
reduced-obstacle cases. The analytic stop-at-waypoint motion remains the
validated fallback, and all early and later HS3 work shares one bounded time
budget.

Earliest-arrival HS3 now preserves a constraint-feasible primary minimum-time
solution instead of always spending a second nonlinear solve to trade up to
one millisecond of arrival for lower integrated jerk. The second solve remains
available only to recover a primary result whose nonlinear residual exceeds
the configured constraint tolerance. Independent validation remains the final
success gate.

The planner does not claim global optimality or search completeness. It returns
the fastest independently validated candidate found within deterministic
proposal limits and bounded local optimization work.

## Largest strengths

### 1. The wall-hugging failure class now receives an early smooth-motion test

The old eligibility condition suppressed early HS3 whenever more than one
exact obstacle was present. In the two-polygon interactive case, this allowed
the analytic boundary-following fallback to win before a wider smooth arc was
tested. The general fix depends only on seed provenance and obstacle count; it
contains no scenario names, route directions, hidden waypoints, or fixture
geometry.

The sandbox case decreased arrival from about 20.718 to 8.859 seconds while
selecting a wider smooth route and passing independent validation. The final
maintained two-opposing-U case selected HS3 at 22.875124576026 seconds. Its
24.370904895056-degree smooth motion is wider than the
23.853720883753-degree visibility seed and ran in 21.7702201 seconds. The
diagnostic rerun selected seed 1 from `directVisibilityEdge`; all five seeds
received HS3 attempts, four validated, and each analytic first motion remained
available as fallback.

### 2. Polynomial and continuous-bound evaluation are batched exactly

Polynomial histories are evaluated by sample batches instead of repeated
per-sample helper calls. Bernstein conversion accepts multiple polynomial
columns, HS3 converts segment/axis bounds in batches, and all seed-corridor
projections share one matrix conversion while reconstructing the legacy
inequality ordering exactly.

The frozen corridor time vector is computed once per HS3 setup instead of in
every finite-difference callback. The pre-change extreme profile recomputed
that invariant 34,203 times. Constraint arrays now use one final concatenation.
The primary or recovery constraint arrays are also reused for final diagnostics
instead of evaluating the selected decision a second time.

Uniform- and nonuniform-duration polynomial checks were bit-for-bit equal to
the scalar calculation. Matrix Bernstein conversion and complete bound vectors
were also bit-for-bit equal, including azimuth-wrapping mode. The optimization
therefore changes evaluation cost, not constraint meaning or tolerance.

### 3. Arrival and runtime improved without weakening validation

On the latest feasibility-recovery 18-example sweep:

- extreme outlines reached 6.683971648809 seconds in 35.1960021 seconds wall;
- dense concave reached 8.797638855700 seconds and ran in 12.1487701 seconds;
- the wide U reached 22.818548735851 seconds in 7.6498733 seconds wall;
- 40 moving circles reached 64.555779916429 seconds and ran in 4.1723092
  seconds wall;
- moving/deforming U.S. reached 12.986386910606 seconds and ran in
  26.8349249 seconds wall;
- two opposing U shapes reached 22.875124576026 seconds and ran in
  21.7702201 seconds wall;
- obstacle-free planning reached 4.612405963436 seconds and ran in
  3.1649118 seconds wall.

Every successful motion passed collision and applicable kinematic certificate
checks. The expected no-path result remained a stable failure with diagnostics.
Every selected HS3 primary solution is about one millisecond earlier than the
preceding jerk-relaxed result. The gain is small in absolute time but exact in
policy: the planner no longer deliberately gives it back. Runtime reductions
are material on wide U, 40 circles, opposing U shapes, and extreme outlines.

### 4. Runtime evidence is reproducible and production passes directly

The user explicitly raised the production target from 7,000 to 7,500 physical
lines. Production is now 7,231 lines, so it passes directly by 269 lines; the
previous proportional allowance is no longer required.

The earlier declared A/B evidence against clean `a023f1c` remains useful:

| Example | Baseline wall (s) | Candidate wall (s) | Reduction | Arrival result |
| --- | ---: | ---: | ---: | --- |
| Extreme outlines | 83.8056819 | 48.2212733 | 42.46% | identical |
| Dense concave | 43.6252843 | 16.8686791 | 61.34% | identical |
| U-shaped time-space | 89.9305427 | 17.1115690 | 80.97% | improved |

The final loop-free evaluator also removes seven production lines. Its helper
microbenchmark improved 68.25 percent with bit-identical values. Paired
three-run medians improved 40-circle wall time from 8.2798430 to 8.2064235
seconds and obstacle-free wall time from 6.1738282 to 6.1303500 seconds. Those
end-to-end gains are below one percent and are not presented as a broad speed
guarantee.

The later reconstruction batch preserves coefficient and terminal-state bits
for 1, 2, 7, and 19 segments. Repeated dense-concave runs were 15.9953 and
16.0208 seconds, 40-circle runs were 7.7309 and 7.7121 seconds, and extreme
runs were 47.5820 and 47.2180 seconds. The final two-U sweep improved from
34.7071 to 32.0304 seconds with exact arrival retained.

The final lazy-output evaluator preserves every requested output bit for two-
through five-output calls. Its position-only helper path is 54.84 percent
faster. Final dense-concave and extreme walls are 15.3269 and 47.3506 seconds;
40-circle repeated timing overlaps the reconstruction range. The final two-U
wall decreases further to 30.6768 seconds.

The seed-corridor batch matched the scalar inequality vector bit for bit and
reduced its isolated helper time by 78.22 percent. Repeated 40-circle runs
were 7.2551 and 7.2014 seconds, and repeated extreme runs were 46.1842 and
45.9486 seconds. The final sweep retained all arrivals and validations while
recording 7.2062 seconds for 40 circles and 46.0246 seconds for extreme
outlines. Dense concave remained inside prior process variation at 15.6605
seconds, so no uniform per-scenario speedup claim is made.

Hoisting the frozen corridor times retained exact arrivals in two serial
gates. Dense concave ran in 14.7514 and 14.5419 seconds, 40 circles in 7.1021
and 7.1510 seconds, and extreme outlines in 45.8169 and 45.7517 seconds. The
single final sweep was slower for dense concave and 40 circles at 16.1936 and
7.8605 seconds. Those unfavorable values remain visible, and the evidence is
treated as process variation rather than a uniform speed claim.

Fixed-arrival HS3 now caps desired seed speed at the route's average speed
over the available duration instead of always using the axis-limited maximum.
The alternating-occlusion motion decreased from 19.229413227596 to
15.324880519000 degrees after the fixed-time CG follow-up, a 20.30 percent
reduction. Four accelerating circles decreased from 43.0900 to 29.1694
seconds wall in the preceding sweep.

CG is now also used for untimed earliest-arrival seeds with one exact obstacle
or reduced geometry. Timed seeds and multiple exact obstacles retain
factorization because those constraint systems regressed under broader CG.
This latest form improved dense-concave arrival by 0.018969702412 seconds and
reduced the recorded extreme-outline wall from 45.7787 to 40.8815 seconds.
The moving-barrier CG trial regressed wall time from 24.8248 to 28.2319
seconds, so timed seeds were removed from that form before the final sweep.

## Main weaknesses

### 1. Search and optimality remain bounded

Spatial and timed proposals use finite samples and deterministic caps. HS3 is
local and time bounded. A missed topology or poor local basin can still prevent
the globally fastest motion. Final validation prevents false success but does
not prove global infeasibility or global minimum arrival.

### 2. Conditioning warnings remain

The 40-circle and four-accelerating-circle cases emit large streams of MATLAB
matrix-conditioning warnings. Returned motions pass independent polynomial,
collision, endpoint, and kinematic validation, but variable scaling remains
the best next numerical target.

### 3. Minimum-time priority produces wider motions

Skipping the optional jerk-relaxation solve when the primary constraints are
already feasible increases sampled motion length on some cases: dense concave
from 12.808111324296 to 13.431299536656 degrees, 40 circles from
122.955082873284 to 126.114009817632 degrees, and wide U from
42.754420388989 to 43.259235381251 degrees. These motions arrive one
millisecond earlier, run materially faster, and remain within hard jerk and
other physical limits. They are minimum-time local solutions, not minimum-
jerk solutions within a one-millisecond arrival band.

`exampleStraightTargetAlternatingOcclusion` now has a 15.324880519000-degree
motion for a 13.341664064126-degree polyline, down from 19.229413227596. It is
valid and meets its fixed arrival, but the remaining excess length and
wall-time-sensitive topology convergence still merit a general improvement.

### 4. Repository size has very little maintained-tree headroom

Production is 269 lines below the user-approved 7,500-line target. The
maintained planner/test tree excluding examples is 3,252 lines below the
12,000-line cap. The HS3 solver is 885 lines. Example files are uncapped and
their 3,920-line total is reported separately.

### 5. Fixed-arrival constraints now avoid nonlinear finite differences

For fixed arrival, final time and obstacle query times are constants, making
the complete HS3 constraint vector affine in jerk. The planner now builds the
affine basis once and uses fmincon linear constraints. Four serial examples
retained independent collision and kinematic certificates while wall times
changed from 29.0747 to 25.2320, 3.4576 to 2.9311, 22.8904 to 20.9312, and
6.9459 to 4.5879 seconds. The accelerating-circle motion shortened from
27.7125 to 20.3724 degrees and alternating occlusion from 15.3249 to 14.2202
degrees. Target-exit motion changed by 0.0000233 degrees while its jerk
objective and final violation both decreased.

Earliest arrival retains the nonlinear time-decision representation. All 14
earliest-arrival maintained examples were bit exact to the preceding accepted
sweep, including the 22.875124576026-second opposing-U result and
22.818548735851-second wider-U result.

The fixed-time jerk objective now supplies its exact analytic gradient.
Central differences matched to 5.33e-10 relative error, and set-time fmincon
objective evaluations decreased from 1,032 to 24. The largest serial wall
improvement was accelerating circles, 25.2320 to 23.6113 seconds. Smaller
cases are dominated by startup and showed changes from -0.9 to 1.3 percent;
no broad runtime claim is made from those individual timings.

Convex buffered seed-envelope membership now uses vectorized polygon queries
instead of repeated polyshape method dispatch after convexity is proved. The
accelerating-circle serial-pair median improved 2.7 percent, from 23.5632 to
22.9181 seconds, with bit-exact motion. The moving/deforming U.S. serial sweep
improved about 2.0 percent. Every maintained trajectory metric remained exact
and all final 59 tests passed.

Complete canonical histories are concatenated before the polygon query. This
deleted eight production lines and improved the accelerating-circle serial-
pair median another 4.35 percent, from 22.9181 to 21.9214 seconds, while a
mixed inside/outside history regression proves the complete-history rule is
retained. Relative to the pre-affine 29.0747-second sweep, the final
22.5888-second run is 22.3 percent faster, but only this scenario family is
claimed.

The final profile attributes the gain to the intended mechanism: envelope
polygon calls decreased from 6,452 to 292 and helper time from 3.0510 to
1.9240 profiled seconds. Profiler wall time is excluded from serial benchmark
comparisons.

Across all 18 rows, summed wall time decreased 7.13 percent and the median case
decreased 4.11 percent relative to the feasibility-recovery sweep. Opposing-U
was the only slower row at +1.50 percent, with bit-exact motion; this is treated
as an unfavorable timing observation rather than a guaranteed regression or a
hidden exception.

### 6. Shared example defaults are now actually resolved

The final headless sweep initially exposed that two U.S.-outline examples
could read `Verbose` before the public planner filled omitted defaults. The
shared resolver now begins with the sole public default structure, then applies
scenario and user overrides. A dedicated test covers default and explicit
values. Moving/deforming and extreme-outline headless examples both pass.

## Rejected experiments and recovery

- Stopping after the first early multi-obstacle HS3 result regressed the
  two-opposing-U arrival from 22.8761246 to 23.9675706 seconds. That form was
  removed; remaining unattempted exact topologies now share the residual HS3
  budget.
- Continuing early HS3 too broadly increased the 40-circle wall time from its
  roughly 9-second reference to 24.217 seconds. Continuation was narrowed to
  multiple exact obstacles; the accepted seed-batch run was 7.2061601
  seconds and the latest feasibility-recovery sweep was 4.4305056 seconds.
- A bit-exact vectorized static-corridor fast path improved dense-concave wall
  time from 17.1341 to 15.5618 seconds but repeatedly regressed the extreme
  case in serial pairs: 48.2099 to 48.4887 seconds and 48.4303 to 48.7783
  seconds. The complete fast path was removed.
- Direct cached Bernstein matrices improved isolated conversion calls by up to
  71.82 percent but repeatedly increased dense-concave planning to 16.2591 and
  16.3256 seconds. The complete change was removed.
- SQP did not finish the basic example inside a 60-second proof window versus
  the 10.1412-second interior-point baseline. Interior-point CG removed the
  recurring conditioning warning and improved several cases, but regressed
  two opposing U shapes from 30.74 to 38.35 seconds. SQP and global CG were
  removed. The later retained CG use is restricted to fixed arrival or
  untimed earliest-arrival seeds with one exact obstacle or reduced geometry.
  A timed moving-barrier CG trial also regressed wall time from 24.8248 to
  28.2319 seconds; timed seeds therefore retain factorization.
- Applying average-speed initialization to earliest-arrival cases regressed
  dense concave from 8.8176085 to 8.9027893 seconds. The broad form was
  removed; the retained condition applies only to fixed arrival.
- Tightening `ArrivalTimeTolerance_s` recovered 0.99 milliseconds but made the
  exact opposing-U second solve take 34.31 seconds. Removing the second solve
  globally made alternating slalom return `noValidatedSeed`. Both broad forms
  were removed. The accepted form skips the jerk-relaxation solve only when
  the primary nonlinear residual already meets `ConstraintTolerance` and
  retains the second solve as infeasibility recovery.
- Limited-memory BFGS increased dense-concave wall time from 13.09 to 13.59
  seconds. PCG tolerances 0.01 and 0.2 gave 13.23 and 13.15 seconds without
  changing iteration counts. Feasible-guess `TypicalX` scaling improved an
  extreme serial pair by only 0.35 percent. All three unverified forms were
  removed.
- Nargout-sized allocation in the polynomial evaluator preserved requested
  outputs but took 1.1225 seconds versus 1.0924 seconds for 20,000 calls. It
  was removed.
- Increasing shape-relative clearance expansion from 2 to 5 percent reduced
  extreme-outline runtime but regressed wide-U arrival from 22.8196 to
  24.3270 seconds. The production value remains 2 percent.
- A template-consolidation edit initially left one stale local constructor
  call. The focused suite exposed 14 errors. The call was fixed and all 43
  focused tests then passed before the full 56-test run.

Unfavorable evidence remains part of the assessment rather than being replaced
by the accepted measurements.

## Recommended next work

1. Reduce maintained-tree size before adding further production machinery;
   only 26 lines remain below the hard cap.
2. Improve the alternating-occlusion route length through a general
   through-velocity or topology-ranking improvement.
3. Keep the user-authorized interactive sandbox labeled auxiliary and separate
   from maintained planner accounting.
4. Keep runtime changes under the same serial A/B recovery rule and avoid
   growing the 885- and 888-line core files.

## Final claim

Plan 325 now tests wider smooth motion earlier for exact multi-obstacle
visibility routes, evaluates polynomial constraints faster, and avoids
nonlinear finite differences for fixed-arrival affine constraints. It supports
static, moving, and deforming obstacles, target interception, timed waits,
finite jerk, and stable no-path diagnostics.

It is a bounded, independently validated planner—not a complete or globally
optimal solver.

## 325-less-nlp evidence-gated spline assessment — 2026-08-21

This isolated worktree starts at exact `plan-325` commit
`5a067112a9f880d015f52fb97538a99010871478`. Production planner selection is
unchanged: HS3 remains the maintained motion constructor.

### Largest measured strength

The research-only bounded quintic B-spline constructor produces independently
validated repeated-turn motions with much smaller decision records and lower
motion-construction wall time on the accepted 1-, 2-, and 5-turn scope:

| Turns | HS3 stage (s) | Spline optimizer (s) | Reduction | Spline decisions | Exact validation |
| ---: | ---: | ---: | ---: | ---: | :---: |
| 1 | 7.5631 | 1.3008 | 82.80% | 1 | pass |
| 2 | 13.2254 | 0.8039 | 93.92% | 2 | pass |
| 5 | 13.1546 | 9.4410 | 28.23% | 6 | pass |

This speed result is not a production acceptance result. The spline motions
are 34.96, 16.53, and 14.34 percent longer in duration than the corresponding
HS3 motions. Minimum continuous clearance falls to 0.015047, 0.000426, and
0.000256 degrees. The last two values are positive under the maintained
validator but leave little numerical or modeling reserve.

### Largest measured weaknesses

1. The 10-turn spline gate failed. The retained mean-penalty formulation took
   131.70 seconds and remained in collision. A worst-clearance retry took
   59.81 seconds and remained at -0.14462 degrees sampled clearance; a
   per-obstacle retry took 63.35 seconds and remained at -0.10607 degrees.
   Both retry objectives were removed.
2. The 20-turn spline case was not run because the prerequisite 10-turn gate
   had already failed. No high-turn completeness or scaling claim is made.
3. The selected quintic B-spline does not interpolate interior route vertices.
   It therefore depends on exact post-construction collision validation rather
   than inheriting geometric-route clearance.
4. The rejected fixed-stop septic Bezier interpolated all route vertices but
   required 28 to 84 seconds of motion on the multi-turn representation cases
   and did not satisfy the maintained quintic polynomial schema. Its code was
   removed; the measured comparison remains in the Phase B CSV.
5. No supervised imitation or reinforcement-learning phase was started. The
   deterministic prerequisite failed, and learned output would not constitute
   a safety certificate in any case.
6. The final `exampleFourAcceleratingCircles` run passed every independent
   certificate but emitted extensive near-singular interior-point warnings.
   This numerical-conditioning weakness remains visible and was not relabeled
   as a harmless success condition.
7. A bounded interior-route interpolation experiment was rejected. It reduced
   10-turn wall time from 90.31 to 55.77 seconds but worsened continuous
   clearance from -0.011725 to -0.544570 degrees because its derivative demand
   forced route reduction from 14 to 8 vertices. All candidate code and tests
   were removed, and the recovered baseline reproduced the same route count,
   decision count, evaluation count, motion duration, clearance, and
   termination reason. This rules out interpolation alone; it does not rule
   out a corridor-constrained representation with independent time handling.
8. Five isolated option experiments showed that retaining route detail is
   necessary but not sufficient. `TimingReserveFraction=1.0` retained 24
   vertices and produced the first independently validated 10-turn spline,
   but its clearance was only 0.000144580 degrees and wall time was 140.74
   seconds. Reducing duration weight by 100 times and increasing collision
   penalty by 10 times reproduced the default decision exactly. A finer
   coordinate step and wider normal-offset bound worsened clearance to
   -0.393003 and -0.353569 degrees. These results reject further scalar option
   tuning as the next step; they do not establish that the retained 24-vertex
   result has a material safety reserve.
9. A feasibility-first, seed-corridor-constrained quintic candidate failed its
   focused one-turn proof: it produced no validated motion, corridor
   certificate, or finite clearance. The candidate was removed before a
   10-turn run, and exact recovery reproduced the frozen default deterministic
   result. This rejects the tested combination of hard Bernstein corridor
   ranking and the existing normal-offset coordinate search; it does not reject
   corridor methods paired with a different decision representation or solver.
10. A worst-clearance-first ranking candidate also failed. It improved over
    the retained optimizer's selected sampled collision but still returned an
    invalid 10-turn motion at -0.075959 degrees clearance and took 100.53
    seconds. Exact recovery restored the deterministic baseline. Together with
    the earlier worst-clearance and per-obstacle trials, this is evidence that
    objective reformulation alone is not the missing high-turn mechanism.
11. The earliest high-turn spline defect is now localized to route reduction.
    The original visibility route is sampled-clear, while uniform arc-length
    reduction creates thousands of occupied edge samples before smoothing.
    A 5-turn profile also assigns about 80 percent of optimizer time to scalar
    sampled-clearance queries. The next justified work is a topology-preserving
    reducer with behavior-equivalent static batching, not another objective,
    tolerance, seed, or validation change.
12. Behavior-equivalent batching removed the sampled-clearance runtime
    bottleneck for the frozen static cases and retained exact 5-turn and
    10-turn deterministic outputs. Protected-route reduction then produced
    independently valid 10-turn splines in 5.22 to 8.45 seconds, proving the
    topology defect was repairable, but their continuous clearances were only
    0.000912 and 0.000155 degrees. Continuing the coordinate search with a
    hard 0.02-degree acceptance reserve reached only 0.000786 degrees; a
    worst-deficit objective regressed to 0.000343 degrees and 107.35 seconds.
    Every reducer and reserve-search edit was recovered. The remaining defect
    is lack of explicit full-span corridor enforcement in the noninterpolating
    quintic construction, not measured evidence for more scalar tuning.
13. A retained affine corridor-constrained research prototype now supplies the
    missing full-span enforcement. With an explicit 22-vertex representation,
    it passed the frozen 10-turn independent validator and existing corridor
    certificate at 0.0200000000000009 degrees clearance, 60.2576877911092
    seconds motion, and 4.0949002 seconds method wall time. The same code passed
    a single rectangle, a moving-history envelope, adaptive protected-route
    growth, and a stable unclear-source-route failure. A new 12-wall alternating
    end maze required repeated near-180-degree turns. The sparse visibility
    graph was disconnected, so an exhaustive graph was used only because its
    estimated work fit the existing budget. Compression grew from 22 to 26
    vertices, proved infeasible, and recovered once to the complete 50-vertex
    route. The resulting C3-continuous motion independently validated and
    certified at 0.02 degrees clearance, 369.337421187 seconds arrival, and
    16.0761214 seconds wall time. This is the branch's strongest deterministic
    hairpin evidence, but it is not global minimum-arrival or production
    replacement evidence.
14. The same recovery does not satisfy every high-turn arrival policy. The
    20-turn repeated-barrier case now reaches a certified full 61-vertex spline
    at 0.02 degrees clearance, but its 122.474368665-second arrival exceeds the
    115.5-second horizon and correctly returns `trajectoryValidationFailed`.
    The feasibility problem is repaired; the remaining failure is timing
    quality under that deadline.

### Frozen HS3 scaling diagnosis

The HS3 decision vector remains 35 variables from 1 through 20 turns, while
nonlinear inequalities grow from 641 to 1,876. The final Phase A runs validate
at 1, 2, and 5 turns and fail at 10 and 20 turns after 75.13 and 193.30 seconds
in the cumulative HS3 stage. The 10-turn topology search is not truncated; its
31-vertex visibility seed is analytically time-window infeasible and its
optimized trajectory still collides. The 20-turn 61-vertex seed is also
time-window infeasible and both bounded HS3 results remain constraint
infeasible. These are motion-construction failures, not demonstrated topology
failures.

### Current scope and size

The replacement-branch spline code remains under `scratch/learnedSplinePolicy`
but counts as production under the repository change-discipline rule. Current
production MATLAB is 9,206 physical lines versus the 7,000 target; the complete
MATLAB tree is 16,483 lines versus the 12,000 cap. `HEAD` was already oversized
at 8,389 production and 14,690 complete-tree lines, and the current work worsens
both totals. The largest production files remain individually compliant:
`solveAzElHs3.m` is 894 lines, while `generateAzElTopologySeeds.m` and
`planAzElMotion.m` are each 888 lines. Branch-wide size compliance is not
established and no performance allowance can waive the 12,000-line cap.

Eighteen noninteractive maintained examples were rerun serially after the
topology change. Seventeen returned independently valid, collision-free,
kinematically certified motions; the expected no-path example returned
`noValidatedSeed`, passed its example-level failure validation, and created two
hidden diagnostic figures. `exampleAzElInteractiveSandbox` remains unexecuted
because it requires live mouse input. Several HS3 examples retained extensive
near-singular interior-point warnings. Keep HS3 until the new method passes the
remaining integration, size, and compatibility gates, and do not describe the
hairpin proof as globally optimal or production-ready.

## Corridor-only replacement assessment — 2026-08-22

The replacement is now integrated into the public planner. `planAzElMotion`
accepts only `MotionMethod="corridorQuintic"`; the dormant HS3 solver,
stop-at-waypoint constructor, NLP options, direct legacy tests, superseded
prototype scripts, and HS3 scaling benchmark were removed. Zero-valued HS3
diagnostic fields remain so callers can prove that no NLP attempt occurred.

The retained algorithm is input-driven. A disconnected visibility graph uses
exhaustive visibility only inside the existing work budget, then finishes a
bounded obstacle-offset ladder to give the recovered topology continuous-motion
reserve. Initially connected graphs retain their base offset. Static and dynamic
routes may use bounded exact-geometry densification, and every candidate is
checked against the original protected geometry, complete timed collisions,
workspace, endpoint states, and exact polynomial kinematic extrema. No example
name, obstacle name, expected route, hidden waypoint, or scenario branch is
consulted.

All 18 noninteractive maintained examples passed a final isolated-process gate.
Seventeen returned independently valid `corridorQuintic` motions; the declared
no-path case returned `noValidatedSeed` and passed its failure contract. Every
case reported zero HS3 attempts. The target-exit example reaches its exact
24-second fixed arrival. The 10-hairpin guard remains independently valid and
corridor-certified at 137.287 seconds and 0.02 degrees clearance; prior
20-hairpin evidence remains valid at 274.993 seconds and 0.02 degrees clearance.
These are bounded deterministic results, not completeness or global-optimum
certificates.

The final automated suite passes 53/53. A visible slalom run created three valid
figures, and the expected no-path run created two diagnostic figures without
rerunning planning. Core production excluding plotting is 6,954 physical MATLAB
lines, plotting is 565 lines, examples total 3,910 lines, and the maintained
tree excluding examples/scratch is 9,709 lines. The 7,000 production target,
900-line per-file limit, and 12,000 maintained-tree cap pass directly without a
performance allowance. The interactive sandbox remains unexecuted because it
requires live mouse input.

## Span-demand controller assessment — 2026-08-22

The 180-trial span coordinate search is replaced by a bounded proportional
controller using per-span velocity, acceleration, and jerk time demand. On the
single U it reduces timing wall time from 32.890 to 5.991 seconds while changing
arrival from 24.740511444152 to 24.973219952131 seconds. That is a verified
5.49-times speedup with a 0.94-percent arrival penalty; the result remains 9.44
percent slower than the frozen 22.818548735851-second HS3 reference. It does not
prove reproduction of HS3's exact geometric path or global minimum arrival.

All 53 automated tests and all 18 isolated maintained examples pass with zero
HS3 time. The selected single-U motion uses 11 controller trials and saturates
the velocity and acceleration limits without exceeding velocity, acceleration,
or jerk certificates. Alternating slalom also improves in arrival. The dense
concavity case remains a visible performance concern at 49.84 seconds, but its
selected motion uses no controller trials; investigate exact concave-corridor
construction separately rather than attributing that cost to retiming.

The current recount is 7,171 core production lines, 565 plotting lines, and
9,928 maintained lines excluding examples/scratch. The largest production file
is 900 lines and the 12,000-line maintained-tree cap passes, but core production
is 171 lines above the 7,000-line target. Because dense concavity has a confirmed
runtime regression outside controller work, the controller speedup cannot
justify a performance-based size allowance. The branch is not merge-ready until
at least 171 production lines are removed without weakening supported behavior.

## Batched affine corridor runtime assessment — 2026-08-22

The retained controller now terminates on the first non-improving certified
duration, preserving the selected U motion while reducing its 11 trials to 6.
Protected-route sampling batches identical edge samples by start vertex, and
zero-endpoint-derivative affine corridor systems build one exact polynomial
basis map instead of one validated trajectory per decision column. Nonzero
endpoint derivatives retain the established path; empty corridors build no
unneeded Jacobian.

The 12-wall hairpin candidate median decreased from `17.2736906` to
`8.3679933 s` (`51.56%`) across three independently valid runs; total-wall
median is `9.7478098 s`. Motion duration (`164.828287993221 s`), 0.02-degree
clearance, collision freedom, and the full-span certificate were unchanged.
The final U run also stayed below ten seconds at `8.3241566 s`, with zero HS3
time and unchanged independently valid `24.973219952159 s` motion.

Arrival quality remains the principal blocker: U is `9.44%` slower than the
frozen `22.818548735851 s` main-branch reference and therefore fails the
required five-percent limit (`23.959476172644 s`). Convex geometric-jerk and
Bernstein derivative-minimax objectives were valid but substantially slower in
motion time and were removed. A less-conservative direct time-parameterization
method is still required before comparing every maintained example against the
main-branch arrival table. The automated suite passes 53/53; the complete
maintained-example matrix was not rerun after this runtime-only change.
Current size is 7,267 core production lines, 565 plotting lines, and 10,024
maintained lines excluding examples/scratch. The hard maintained-tree cap
passes, but the 267-line core overage remains a merge-readiness blocker.

## Dynamic seed-slot coverage assessment — 2026-08-22

The planner now reserves bounded seed capacity for an extended temporal route
only when that route is actually nonempty and distinct. This preserves the
existing direct-wait and spatial-before-extended ordering while converting one
of eight fixed-seed moving-circle failures into an independently valid motion.
All 18 maintained examples retain their frozen motion durations and the full
57-test suite passes.

The largest remaining weakness is dynamic route scaling. A 42-vertex moving
maze requires the original full timed topology to retain success; two faster
22-vertex pruning variants lost that solution. Similar high-dimensional
dynamic routes can therefore remain expensive, and the random moving-circle
probe still succeeds in only three of eight feasible fields. No completeness
or general runtime improvement is claimed.

## Shallow collision-residual feedback assessment — 2026-08-22

The dynamic retimer now applies signed-clearance feedback only to a failed
candidate whose penetration is no deeper than `0.005 deg`. The gain is derived
from each active barrier row's control-point sensitivity and the current trust
radius; a minimum-norm bounded QP supplies the correction. Negative signed
clearance uses the outward interior gradient, and every accepted step must
strictly improve independent clearance while retaining kinematic validity.

This recovers one additional fixed moving-circle field, improving the final
deterministic sweep from `3/8` to `4/8`. Its selected motion has
`0.00570897255047 deg` independent clearance. An initially unbounded recovery
was rejected: it changed the extreme-outline benchmark from
`6.22216662414646 s` to `8.39529809634767 s`. The retained local-residual bound
restores the exact benchmark path and excludes all three deeper residuals seen
in that sequence. All 18 maintained examples and all 58 tests pass. The four
remaining circle fields still return explicit `noValidatedSeed`; topology and
large-residual collision recovery remain known limitations.

## Retry-exhausted boundary-support assessment — 2026-08-22

The largest current strength is deterministic coverage with preserved
benchmark quality. A visibility graph that remains disconnected after the
existing third offset/exhaustive retry now tests the four input workspace
corners as bounded support nodes and preserves one spatial seed opportunity.
The fixed moving-circle sweep improves from 4/8 to 8/8 independently validated
solutions. All four formerly failing cases use a selected spatial visibility
seed with positive continuous clearance; the unchanged workspace-spanning wall
still returns the expected diagnosable `noValidatedSeed` result.

The rule is deliberately inactive on connected retry-zero graphs. An earlier
broader seed replacement regressed the maintained moving-circle duration from
`8.75122873615098 s` to `8.77956166926098 s`; the retained retry-three gate
restores it exactly. The final fresh-process 18-example matrix has 17 validated
successes and one validated expected failure, with every successful path metric
exact to pushed evidence. The 59-test suite passes and Code Analyzer reports no
messages.

The largest remaining weakness is runtime and the absence of completeness.
The final eight-case sweep takes `232.089424 s`; boundary routes often use a
workspace corner and all eight clearances are positive but as small as
`0.000355731152304 deg`. The fresh example wall sum is an unfavorable
`205.6452420 s`, with `61.1979723 s` for the moving/deforming outline and
`39.7610225 s` for the extreme outline. This is a focused coverage improvement,
not a global completeness, optimality, or runtime claim. Production remains at
the exact 7,500-line conditional ceiling, leaving no growth margin.

## Repository module assessment — 2026-08-22

The largest maintainability strength is now explicit module ownership. Public
entry points remain at the root, while internal geometry, obstacle
preparation, visibility/timed search, continuous motion, and certification are
separate MATLAB subpackages with concise names and documented dependency
boundaries. The refactor removes 18 executable lines despite adding two shared
geometry helpers. It also stops each corridor candidate from discarding and
rebuilding the planner's immutable prepared-obstacle cache.

Behavioral evidence is unchanged: all 59 tests pass; 17 maintained examples
remain independently valid; the expected no-path example retains stable
diagnostics; and every recorded route and duration exactly matches the prior
boundary-support evidence. The flat-package refactor therefore has regression
evidence across static obstacles, moving/deforming obstacles, moving targets,
waiting, wrapping, dense fields, expected failure, and physical certificates.

The largest weaknesses remain algorithmic rather than organizational. The
finite seed portfolio is not complete, small-clearance boundary routes remain,
and the fresh example wall sum is an unfavorable `222.7331866 s`. The inert
public `RandomSeed` compatibility field remains intentionally retained rather
than removed by an internal cleanup. Physical core size is 7,773 lines because
the user authorized helpful comments above the ceiling; executable core size
is 6,450 lines, 18 below the pre-refactor executable count.

## Readability assessment — 2026-08-23

The strongest current readability property is a consistent two-level comment
contract: the primary function owns the complete Section 0 readme, while every
local function starts with a direct explanation and relies on nearby comments
for loops, decisions, state transitions, and failure paths. No local Section 0
or duplicate local `PURPOSE` block remains. The largest planner and search
files also explain why bounded fallbacks and candidate transitions occur, not
only what each condition tests.

The short-file audit found no unused production MATLAB file below 100 code
lines. The four single-caller helpers remain separate because merging them
would hide a stable result schema or add a distinct algorithm to an existing
486-, 847-, or 898-line orchestrator. This improves navigation without claiming
that file count alone proves good modularity.

The latest pass was not regression-tested because the user explicitly disabled
tests for the session. Structural text audits and `git diff --check` passed;
the earlier planner, example, and Code Analyzer evidence remains historical
evidence from before the final comments-only edits.

## Persistent sandbox assessment — 2026-08-23

The branch now includes a persistent manual scene-building UI under `sandbox/`.
Its strongest usability property is guided input: the first scene advances
from start to goal or first endpoint and then to the first obstacle without
separate mode buttons. Both tabs provide an explicit Add Obstacle action for
later strokes. Reset clears retained state and all axes children, including
hidden freehand traces.

The controls use shared columns and three labeled groups instead of repeating
axis sublabels for every value. The plot reserves its outer rectangle so
azimuth ticks and labels do not overlap the action row. The principal weakness
is size: the supplied UI is a 1,714-line standalone sandbox and is intentionally
kept outside production and maintained examples. It exercises public planner
and validator interfaces but adds no planner correctness evidence. Per the
user's instruction, only structural checks and `git diff --check` were run;
no MATLAB, Code Analyzer, example, or regression execution is claimed.

## Combined method-suite assessment — 2026-08-23

The branch now exposes two genuine planner choices without blending their
internals. `corridorQuintic` preserves the no-NLP `325-less-nlp` engine and is
the backward-compatible default. `hs3` preserves the `plan-325` analytic/HS3
engine and must be selected explicitly. Each package owns obstacle preparation,
search, motion construction, validation, result construction, and its original
moving-target adapter. The public dispatcher runs one package only and records
the choice in result options and diagnostics.

The strongest evidence is source-baseline reproduction rather than a claim
that the methods are equivalent. Fresh processes produced 18/18 exact gated
matches for corridor and 18/18 for HS3. Each set contains 17 independently
validated successes and the expected validated `noValidatedSeed` failure.
Corridor's fresh wall sum was `163.9501049 s`; HS3's was `238.5162172 s`.
Wall time was retained but not equality-gated. Both directions of physical
folder removal also passed an obstacle-free maintained example with independent
validation and the correct surviving method echo.

The main maintainability cost is intentional duplication: similar low-level
helpers exist inside both method packages so a change to one cannot silently
change the other. This trades repository size for removability and baseline
fidelity. Obsolete planner copies at the root were not kept—22 unreachable
search, motion, validation, and result-builder files were deleted. The remaining
root internals have real plotting, constructor, or option-handling callers.

Repository-wide static evidence is clean. MATLAB Code Analyzer checked 109
intended MATLAB files with zero messages. Text audits found all 368 loops
directly explained, primary-only Section 0 headers, no local PURPOSE boilerplate,
and no unused production MATLAB file. Canonical comparison also found no
executable drift from either selected source snapshot after approved namespace
and entry-point renames.

The methods retain different option sets and different earliest moving-target
policies. Users must not assume that switching only the method produces the
same arrival interpretation or trajectory. Neither finite proposal portfolio
is complete, neither result proves global optimality, and HS3 retains local NLP
conditioning/failure risk. The full automated regression suite was not run
because the user prohibited tests in this session; the 36 fresh example runs
are direct combined-branch evidence but are not represented as an unrun test
result.
