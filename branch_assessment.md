# Plan 325 branch assessment

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
