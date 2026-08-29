# Plan 325 branch assessment

The current planner judgment is **independent Ruckig-derived and HS3 trajectory
engines, with obstacle-aware routing owned by the Az/El planner and no neutral
trajectory wrapper — 2026-08-27**.
Earlier sections remain as historical evidence and may name implementations
that are no longer present.

## HS3 optimizer complexity split — 2026-08-28

The shared HS3 optimizer now delegates its numerical-problem contract to one
local validation owner. MATLAB's McCabe complexity for `hs3Engine.optimize`
decreased from 43 to 21, while the extracted validation function measures 23;
the file maximum therefore decreased 46.5%, from 43 to 23. Solver selection,
recovery, termination classification, tolerances, and returned diagnostics are
unchanged. Code Analyzer reported zero findings, and 99 focused standalone
kernel, planner, polynomial, and nonuniform-mesh tests passed. The deforming
obstacle test retained its known near-singular `fmincon` warning stream. No
runtime or trajectory-quality improvement is claimed for this structural
refactor.

## Branch hygiene cleanup — 2026-08-28

Generated `output/`, scratch `tmp/`, and local `.claude/` worktree state are
now excluded from version-control status. No artifacts were deleted, and the
untracked `Rogue Examples/` directory remains visible because it is not a
recognized reproducible build or benchmark output. This cleanup changes no
planner behavior. MATLAB Code Analyzer reported zero findings for the four
new hybrid classification, activity-mesh, pass-through, and mesh-resolution
owners before the hygiene-only change.

## Optional non-stopping Ruckig-to-HS3 warm start — 2026-08-28

The planner now exposes an opt-in `WaypointWarmStartMode="passThrough"` for
static earliest-arrival detours. A native MATLAB adaptation of the
MIT-licensed `rsruckig` waypoint search estimates shared nonzero interior
velocity and acceleration, chains exact jerk-limited state-to-state sections,
then supplies the sampled profile to a fixed-time HS3 repair and a 14-segment
free-time HS3 polish. Ordinary HS3 still runs, and only independently validated
candidates participate in the existing arrival-time ranking. The default mode
remains `"none"`.

On `exampleStaticUShapedObstacle`, the retained one-sweep configuration passed
independent collision and kinematic validation at 21.9798565134 seconds and
41.2461615078 degrees, versus the default 22.6308876389 seconds and
41.5363500661 degrees. The measured integrated run used 16.139615 seconds wall
time, so no runtime improvement is claimed against the earlier 13.717231-second
default record. The best experimental four-sweep combination remains recorded
separately at 21.4038773205 seconds and 41.056539875 degrees, but its full
34.3518164-second pipeline is too slow to use as the maintained configuration.

The structurally different `exampleTwoOpposingUVisibilityGraph` demonstrates
the selection guard. Default HS3 and the opt-in comparison both returned the
same independently validated 21.9090835611-second, 24.4189853364-degree
candidate. The opt-in run took 9.375114 seconds versus 5.977472 seconds for the
default, so the hybrid adds approximately 3.40 seconds when it does not win.
These two cases do not establish global time or path optimality.

Focused evidence currently includes 4/4 planner-option tests, 10/10 Ruckig
engine tests, 1/1 non-stopping waypoint test, and 64/64 full HS3 planner tests.
The source credit and adaptation boundary are documented in `citation.md`.

## Bounded interactive sandbox planning — 2026-08-27

The sandbox now owns a smaller, explicitly documented HS3 work portfolio while
the production planner defaults remain unchanged. On a screenshot-matched
one-polygon moving-obstacle request, the production-default baseline required
123.0002 seconds; 119.3994 seconds, or 97.1%, was motion solving. The bounded
sandbox configuration returned an independently validated, collision-free and
kinematically valid motion in 4.3057 seconds, a 28.57x wall-time reduction.

The responsiveness gain has a visible quality cost: moving-case earliest
arrival changed from 40.7616 to 51.7410 seconds, a 26.93% later result. The
matched stationary case improved from 14.0421 to 5.2767 seconds (2.66x), with
arrival changing from 41.7830 to 43.9204 seconds (5.12% later); both runs were
independently valid. The sandbox result is an interactive preview, not a global
or production-quality optimum. Callers may replace
`sandboxOverrides.PlannerOptions` when finer arrival quality is worth the
additional nonlinear work. These two cases do not establish broad scaling.

Successful, independently validated Goal Mode runs now automatically launch
the maintained result-driven animation dashboard. It advances the returned
motion, time-varying obstacles, elapsed path, and four kinematic histories from
the same planner result; it does not rerun planning. Hidden sessions skip the
animation and pause entirely. `AnimateOnRun`, `AnimationFrameStride`, and
`AnimationPause_s` remain sandbox-level display controls.

## Independent engines and obstacle-owned routing — 2026-08-27

The largest architectural strength is now enforceable ownership. The
`trajectory/` root contains no MATLAB entry-point files: `ruckigEngine.solve`
owns exact jerk-switching motion, while `hs3Engine.solve` owns direct
collocation and its polynomial and constraint packages. Ruckig contains no HS3,
optimizer, obstacle, or planner dependency. HS3 contains no Ruckig switching
functions or Az/El dependency. Obstacle avoidance alone sees geometry and calls
Ruckig only for an empty obstacle field with a fixed-position target. Every
moving target or nonempty obstacle field continues through topology search and
HS3. Unsupported Ruckig switching families continue to HS3, while identified
physical infeasibility and independent-validation failures remain visible.

The former neutral dispatcher and its request, validation, conversion, and
internal-helper files were removed. Each engine now resolves its own options,
normalizes its own input, and validates its own output. The obstacle planner's
Ruckig-to-Az/El polynomial translation is a local function at the geometry-aware
boundary, followed by canonical obstacle-planner validation. Direct engine
results no longer contain selection or fallback provenance fields.

Measured warm overhead supports the removal. On the same moderate two-axis
request, direct Ruckig measured 1.8168 ms median over 30 repetitions. The public
obstacle planner measured 7.0647 ms median, a 5.2479 ms addition for Az/El input
normalization, endpoint checks, result translation, and a second independent
continuous validation. Before removal, the neutral dispatcher itself added
roughly 0.4--0.6 ms through duplicate normalization. The remaining 5.25 ms is
real safety work and is not presented as zero-cost routing. It is material for
millisecond-scale callers, so callers that need only dimension-neutral motion
should invoke an engine directly.

The full repository suite passed 144/144 in 55.099274 test seconds. All 17
maintained examples then ran serially in fresh headless MATLAB processes. The
16 successful motions passed their example-specific checks and independent
continuous collision, position, velocity, acceleration, jerk, and dynamics
validation. `exampleNoPath` returned the expected `noValidatedSeed` with
consistent failure diagnostics. A visible Ruckig-routed success created three
figures and six axes; a visible expected failure created two diagnostic figures
and two axes.

Obstacle-constrained route, motion-length, and duration results match the
preceding exact-switching evidence. The obstacle-free example improved from
4.5458984375 to 4.53112887415 seconds while retaining the 4.472135955-degree
straight path. This local result does not prove global optimality. The largest
runtime weakness remains the moving/deforming U.S. outline example at
166.070652 seconds in this run, consistent with the recent 178--184 second
range. The largest maintainability weakness also remains total implementation
size and the complexity of the nonlinear obstacle-constrained path.

## Exact dimension-neutral third-order switching — 2026-08-27

The public HS3 engine now solves symmetric third-order position limits with a
pure-MATLAB adaptation of the Ruckig v0.19.4 switching equations. Each axis is
solved independently for its certified minimum time, faster axes are
synchronized to the limiting duration, and equal-duration candidates are
ranked by dense spatial length before integrated jerk. Eligible rest-to-rest
multi-axis moves retain the certified scalar-progress path so a direct motion
is exactly straight. Every constructed polynomial still passes the existing
continuous HS3 constraint evaluator and independent public validation. Path
constraints, asymmetric derivative bounds, and unsupported numerical families
retain the existing general optimizer. The engine consumes only arbitrary-
dimension states, derivative bounds, time policy, and optional path
constraints; it has no knowledge of obstacles, azimuth/elevation, targets, or
planner examples. The MIT attribution is retained in
`hs3/THIRD_PARTY_NOTICES.txt`.

Against exact baseline `e3596a3`, the final serial 21-case shared reference run
matched 19 of the 20 published success arrival values within 0.00005 seconds;
the largest absolute difference was 0.00004514 seconds. All 21 wall-time gates
passed. Summed measured solver wall time decreased from 32.068521 to 1.1144582
seconds (96.52%), while the 19 matched successful arrivals improved by a
combined 2.0775845 seconds. The retained reference script, input table,
benchmark harness, and final measurements make the comparison reproducible.

The unmatched published-success row is not treated as a solver failure to be
hidden. Its initial velocity is exactly at the lower bound while its initial
acceleration points farther outward. Any finite continuous jerk therefore
violates the velocity bound immediately. HS3 now returns
`kinematicallyInfeasibleBoundaryState` in 0.0037 seconds rather than claiming
the published 31.9573-second motion is feasible. The other published failure
is rejected as `fixedTimeBelowMinimum` in 0.027 seconds.

Minimum arrival remains the primary objective and spatial length is the
secondary objective at that duration. Relative to the slower baseline, 16 of
19 successful paths were equal or shorter and three were longer while reaching
the endpoint earlier. The largest reduction was 16.7155 coordinate units for
the reordered case. The mixed case-2/fixed-case-8 path was 21.7015 units at
7.91695 seconds versus 21.3863 at 8.39679 seconds; its direct distance is
21.2843. This is a real time-versus-length tradeoff, and the switching-family
tie-break does not establish global shortestness for non-rest endpoint states.

The focused standalone HS3 suite passed 18/18. The complete repository suite
passed 132/132 in 68.757 seconds. All 17 maintained examples then ran serially
in fresh headless MATLAB processes: 16 successes passed independent collision
and continuous position, velocity, acceleration, jerk, and dynamics checks;
the expected `noValidatedSeed` result passed its failure-diagnostic checks. A
visible success created three figures, and a visible expected failure created
two diagnostic figures with retained search data. Every maintained example
kept its preceding selected route, smoothed-path length, and duration.

The exact solver adds substantial implementation size: production is 13,674
physical lines and production plus tests is 18,250, above the repository's
7,500- and 12,000-line targets. The largest production file remains 899 lines.
This size cost is the main maintainability weakness. Explicit `parfor` was not
added because each accepted axis solve is normally only a few milliseconds and
pool startup would dominate these cases; the axis decomposition remains ready
for a caller-owned parallel strategy if much higher dimensions justify it.

## Certified multi-axis direct progress — 2026-08-27

The dimension-neutral public HS3 engine now reduces an eligible obstacle-free,
rest-to-rest direct motion to one monotone scalar progress polynomial, then
lifts the validated jerk controls back to every input dimension. Effective
velocity, acceleration, and jerk limits are the intersection of the
displacement-normalized limits from every active axis. Fixed-time requests use
the reduction only when the direct line is feasible. Earliest-arrival requests
additionally require one axis, including ties, to own all three normalized
derivative limits; that axis supplies a lower bound on every unrestricted
multi-axis motion, so the reduction cannot sacrifice arrival time. Asymmetric
bounds, path constraints, non-rest endpoints, mixed limiting axes, and
one-dimensional inputs retain the preceding general solve.

Against exact baseline `e72957c`, three structurally different 3-D/4-D
earliest-arrival cases kept arrival within 0.0000386 seconds of the same-mesh
convex fixed-time feasibility boundary. Their spatial excess above the direct
endpoint distance decreased from 0.00364763382, 0.00495067325, and
0.00239304950 to magnitudes below `6.82e-9`. A three-axis moving-target
fixed-time sweep decreased spatial excess from 0.000188808074 to floating-point
zero while preserving every trial time. The warmed three-case median free-time
wall time decreased from 1.6038312 to 0.5261287 seconds (67.20%). The first
balanced solve remained dominated by cold Optimization Toolbox startup at
3.1921851 seconds. The moving-target sweep changed from 0.2210358 to
0.2851910 seconds; this single 0.064-second unfavorable observation is retained
and no moving-sweep speedup is claimed.

The focused standalone suite passed 17/17. The complete suite passed 131/131
in 79.596835 seconds. Code Analyzer reported zero findings in the changed
engine file. All 17 maintained examples ran serially in fresh MATLAB processes:
16 successes passed independent continuous validation and the expected
`noValidatedSeed` result passed its failure-diagnostic checks. Every maintained
path and arrival metric matched `e72957c`. A visible success created three
figures with six axes; a visible expected failure created two diagnostic
figures with two axes.

This is an exact shortest-path and minimum-time result only for the declared
direct-progress gate, not a global optimality claim for constrained paths or
mixed axis bottlenecks. Explicit parallel workers were not added: the accepted
case is now one small scalar solve, for which pool startup and serialization
would dominate; MATLAB remains free to use native threaded linear algebra.
The change adds 147 production lines and 82 test lines. Production is 11,823
lines, so the 4,323-line overage requires
`0.25 * 4323 / 100 = 10.8075`, or a 1080.75% wall-time reduction, under the
repository formula. That literal size gate remains impossible and unsatisfied.
Production plus tests is 16,359 lines, also above its 12,000-line target.

## Unified spatial and timed seed equivalence — 2026-08-27

The route search now owns one sampled seed-equivalence helper instead of
separate spatial and timed implementations. Spatial routes reuse the complete
seed's arc-length parameterization, while timed routes additionally preserve
the existing relative duration comparison. Position tolerances, seed order,
reachability decisions, and diagnostics are unchanged.

Exact A/B checks against `5cf2d87` compared complete seeds and diagnostics for
a static concave U and a translating barrier. Both were `isequaln`. Eight
focused duration, waiting, homology, cleanup, reachability, clustering, and
dense-envelope tests passed 8/8. The complete suite passed 128/128 in
80.343082 seconds, and Code Analyzer reported zero findings in the changed
file. All 17 maintained examples then ran serially in fresh headless MATLAB
processes: 16 successes and the expected `noValidatedSeed` failure all passed
independent validation with unchanged path and arrival metrics. Their wall-time
sum was 349.9932267 seconds. A visible success created three figures with six
axes; the visible failure created two diagnostic figures with two axes.

The matched route-search timings do not establish a speedup. Static-U median
time changed from 0.0536149 to 0.0545641 seconds (-1.77%), while moving-barrier
median time changed from 0.6353544 to 0.6119082 seconds (+3.69%). Both are
below the 5% runtime gate, so the change is retained only for removing one
duplicate implementation, three executable lines, and six physical lines.
The main route owner remains McCabe 27 and its visibility helper remains 18;
the two old equivalence helpers totaled complexity 7, versus 5 for the shared
helper. `createRouteCandidates.m` is now 885 lines. Production is 11,676 lines
and production plus tests is 16,130. The 4,176-line production overage still
requires an impossible 1044% wall-time reduction under the literal formula.

## Route-candidate file-cap cleanup — 2026-08-27

`createRouteCandidates.m` decreased from 924 to 891 physical lines by removing
33 duplicate narrative-comment or blank lines. Mechanical diff inspection
confirms that no executable line changed. Production is now 11,682 lines, and
the largest production file is `solveRouteCandidate.m` at 899 lines, so the
hard per-file limit is currently satisfied.

This is a file-cap and readability cleanup, not a cyclomatic-complexity claim.
McCabe complexity remains 27 for the main route-candidate owner and 18 for its
visibility-graph helper. Production plus tests remains 16,136 lines, and the
4,182-line production overage still requires an impossible 1045.5% wall-time
reduction under the literal formula. Four MATLAB R2024b verification launches
failed before code execution with a host file-system inconsistency while
existing processes were preserved, so focused post-cleanup tests remain
unexecuted.

## Prepared dynamic boundary-edge queries — 2026-08-27

Dynamic non-support corridor constraints now interpolate only the selected
canonical boundary edge. Preparation caches finite ring bounds and the
topology-change union ring bounds, so each solver callback avoids rebuilding
every boundary edge. Support constraints, fixed obstacles, final collision
validation, and the spatial visibility search are unchanged. Exact source-time
selection, query-time closure, multi-ring edge order, and conservative topology
unions remain shared invariants with the complete geometry path. Old or partial
caches and finite endpoints whose interpolation delta overflows fall back to
that complete path.

At exact baseline `750e9c7`, three moving/deforming U.S. runs had wall times of
179.5093499, 179.4877722, and 180.0643631 seconds. The initial exact fast-path
candidate measured 148.7280267, 147.6009931, and 148.5100961 seconds, a 17.27%
median reduction. The later safeguarded worktree's 165.7597307-second run was
collected in a different session without a contemporaneous baseline, so the
former 7.66% comparison does not establish the final implementation's causal
gain. Every recorded run retained the exact 40-degree selected route,
43.0751355347-degree smoothed motion, 7.96286899667-second duration, solver
evidence, and validation certificates. Structurally different opening-U,
accelerating-circle, moving-circle, and static-outline controls retained their
exact path and arrival metrics.

A post-push experiment moved defensive cache validation outside the selected-
edge hot loop. Against exact parent `8059595`, counterbalanced parent wall times
were 167.9463006 and 165.2065477 seconds and candidate times were 165.7019183
and 163.1769446 seconds. The 1.28% median wall reduction and 1.08% median
reported-planner reduction missed the declared 5% retention gate; the code was
reverted. This localizes repeated public-query validation as a real but minor
cost rather than the remaining dominant bottleneck.

Sparse nonlinear Jacobians were also tested only at the neutral HS3 `fmincon`
boundary. The counterbalanced Opening-U median improved 2.29%, from
43.48570245 to 42.4909808 seconds, while solver iterations, objective,
residuals, route, arrival, and validation remained exact. Because the result
missed the declared 5% wall-time gate and required production growth, it was
reverted. Sparse conversion is therefore not the next retained optimization at
the current 43-variable problem scale.

Limited-memory BFGS was also rejected: one identical Opening-U run increased
wall time to 69.5511834 seconds without changing the 19-iteration solution.

The complete suite passed 128/128. All 17 maintained examples ran serially in
fresh headless processes: 16 independently validated successes and the
independently validated expected `noValidatedSeed` failure. Every successful
result passed collision and continuous kinematic checks. Code Analyzer reported
zero findings across all 87 maintained MATLAB files. Focused obstacle and HS3
tests passed 21/21, including exact fast-path versus complete-boundary values
and gradients for a deforming edge. A visible successful run created three
figures with six axes; a visible expected failure created two diagnostic
figures with two axes.

The retained diff adds 205 net production lines and 173 net test lines. The
production tree is 11,715 physical lines, so the 4,215-line overage requires
`0.25 * 4215 / 100 = 10.5375`, or a 1053.75% wall-time reduction under the
repository formula. That literal size gate is unsatisfied. Production plus
tests is 16,169 lines, also above the independent 12,000-line limit. The size
increase is a real maintainability cost. The benefit is localized to dynamic
selected-edge constraints; support constraints and solver factorization remain
dominant in other cases, and this optimization does not establish global path
or arrival-time optimality.

## Convex arrival search for certified static direct routes — 2026-08-27

A static, geometry-certified direct seed with rest-to-rest endpoint velocity
and acceleration now uses the existing fixed-time linear HS3 formulation and
a bounded feasibility bisection for earliest arrival. The trial count is
derived from the initial time bracket and `ArrivalTimeTolerance_s`, so the
reported arrival resolves the public tolerance rather than an unrelated
internal cap. Empty obstacle fields retain their existing branch. Detours,
changing obstacles, moving goals, non-direct routes, and non-rest endpoint
states retain the free-time or timed-topology behavior they had before.

Against exact commit `db58000`, a diagonal 94.4183046713-degree direct case
with irrelevant static geometry kept the exact shortest spatial length.
Arrival changed by only 0.0000915641 seconds, within the 0.001-second public
tolerance, while the warmed five-run median decreased from 1.5234749 to
0.8456878 seconds (44.5%); every candidate run was faster than its baseline
counterpart. A structurally different 20-degree axis-aligned case improved
arrival from 12.8082788069 to 12.6129150391 seconds and decreased its warmed
five-run median from 0.73225 to 0.6630497 seconds (9.45%). The exact fixed-time
formulation removed the latter case's 7.45e-8-degree spatial drift.

The complete suite passed 126/126. All 17 maintained examples ran serially in
fresh headless processes: 16 independently validated successes and the
independently validated expected `noValidatedSeed` failure. Every successful
result passed collision and continuous kinematic checks, and all selected
routes, smoothed lengths, and durations matched `db58000`. Code Analyzer
reported zero findings across all 86 maintained MATLAB files, and the six
example-requirement tests passed on a clean repository-only path. A visible
obstacle-free run created three figures with six axes, and a visible expected
failure created two diagnostic figures with two axes. The moving/deforming
U.S. case retained its 40-degree route, 43.0751355347-degree motion, and
7.96286899667-second duration at 179.1714178 seconds.

The retained diff adds 24 net production lines and 31 net test lines.
`plan.m` is 789 physical lines. The production tree is 11,510 physical lines
and production plus tests is 15,791, still above the repository's 7,500- and
12,000-line targets. The planner remains a bounded candidate search: this
convex timing result applies only after a direct spatial route has been
certified and does not establish global path or arrival-time optimality. The
moving/deforming case remains the largest measured runtime weakness.

## Certified monotonic direct-line progress — 2026-08-27

Ordered, collinear, geometry-certified seeds now receive a continuous
nonnegative-progress constraint only when both endpoint velocity and
acceleration are zero within floating-point tolerance. The route tangent is
projected onto every segment's complete velocity polynomial, converted to
Bernstein form, and every coefficient is constrained nonnegative. This makes
the returned motion advance or wait along the certified direct line rather
than reverse along it. Detours, unordered collinear seeds, moving targets,
nonparallel endpoint derivatives, and every non-rest endpoint request retain
the original unrestricted behavior.

The Bernstein coefficient condition is a conservative sufficient certificate,
not a necessary characterization of every nonnegative polynomial. Restricting
activation to certified rest-to-rest lines bounds that conservatism, but the
planner remains a bounded candidate search and this change does not establish
global path or arrival-time optimality. Focused tests verify value/Jacobian row
parity, reject an interior backward velocity, preserve partial-layout rows, and
retain required reversals for both backward initial velocity and a short move
with large positive endpoint speeds.

Against exact commit `36b854f`, Opening U retained its 10-degree selected
route and 11.8560791016-second arrival while motion length decreased from
10.0912159691 to exactly 10 degrees. The final fresh retention run took
39.0340602 seconds versus the 38.5046175-second three-run baseline median;
this 1.38% difference is treated as run noise rather than a speedup. Every
other maintained example retained exact selected-route, smoothed-motion, and
duration metrics from the preceding matrix. The longest moving/deforming U.S.
case retained its 40-degree route, 43.0751355347-degree motion, and
7.96286899667-second duration at 179.707168 seconds.

The complete suite passed 125/125. All 17 maintained examples ran serially in
fresh headless processes: 16 independently validated successes and the
independently validated expected `noValidatedSeed` failure. Every successful
result passed collision and continuous kinematic checks. A visible
obstacle-free run created three figures with six axes, and a visible expected
failure created two diagnostic figures with two axes.

The retained diff adds 79 net production lines and 81 net test lines.
`solveRouteCandidate.m` is 899 physical lines, below the 900-line changed-file
cap, but existing `createRouteCandidates.m` remains 924 lines. The production
tree is 11,486 physical lines and production plus tests is 15,736, still above
the repository's 7,500- and 12,000-line targets. That pre-existing size debt
remains a maintainability weakness; the moving/deforming case remains the
largest runtime weakness.

## Batched occupancy and deferred query allocation — 2026-08-27

Multi-ring occupancy now passes every point at one obstacle/time group through
the existing vectorized signed-clearance query. This reuses one boundary-edge
traversal while preserving the same interior, boundary-tolerance, and first-
blocker policies. `shapeAtTime` now constructs the stable empty geometry and
empty `polyshape` only on inactive returns; active cached and interpolated
queries no longer allocate placeholders that are immediately overwritten.
Moving, deforming, topology-changing, single-slice, and out-of-range activity
semantics are unchanged.

Against exact commit `2cc0988`, an identical instrumented Opening-U run kept
the exact 10-degree seed, 10.0912159691-degree trajectory, and
11.8560791016-second duration. Occupancy-query time decreased from 1.63920621
to 1.24773731 seconds (23.9%), its occupancy-only block from 1.16577671 to
0.913307407 seconds (21.7%), multi-ring occupancy from 0.627571103 to
0.546502604 seconds (12.9%), and 8,075 `shapeAtTime` calls from 0.880268904 to
0.580085704 seconds (34.1%). Profiled wall time decreased from 41.1939427 to
40.2238949 seconds (2.36%). The proprietary augmented-matrix factorization
still dominates the optimizer and was not changed.

The final serial moving/deforming U.S. example retained exactly its 40-degree
seed, 43.0751355347-degree motion, and 7.96286899667-second duration while wall
time decreased from the preceding 182.935442-second run to 178.568018 seconds
(2.39%). Opening-U decreased from 38.9261867 to 38.4144474 seconds and the
extreme U.S. outline from 39.5964486 to 38.7741662 seconds, with exact path and
arrival parity. These are individual fresh-process observations rather than a
claim of uniform speedup.

The complete suite passed 122/122. All 17 maintained examples ran serially in
fresh headless processes: 16 independently validated successes and the
independently validated expected `noValidatedSeed` failure. Every successful
result passed collision, continuous kinematic, and collision-resolution
certificates without path-length or arrival-time regression. A visible failure
run created two diagnostic figures with two axes. Ten focused obstacle tests
cover batched-versus-pointwise multi-ring decisions and every allocation-
sensitive return type, including inactive, interpolated, single-slice, and
topology-changing queries.

The retained production change adds two net lines; the focused tests add 86
lines of boundary-policy and return-type evidence. It removes repeated work
but introduces no cache lifetime, new option, tolerance, or planner heuristic.
The moving/deforming example at 178.568018 seconds remains the largest runtime
weakness.

## Time-invariant obstacle geometry cache — 2026-08-27

Prepared obstacles now retain one complete shape and geometry record only when
their input history proves exact time invariance: either one source slice, or
matching topology with zero corresponding-vertex speed across every interval.
Out-of-range activity rules and source-slice indices are still evaluated per
query. Moving, deforming, and topology-changing histories keep the original
interpolation path.

Against exact commit `81f2f8b`, 20,000 varying-time static geometry queries
decreased from 0.9724214 to 0.4832129 seconds (50.3%) with the same checksum.
Static-U profiler time across 8,317 `shapeAtTime` calls decreased from
0.7005765 to 0.4038666 seconds. A warmed, counterbalanced Static-U comparison
decreased median wall time from 10.56790325 to 10.1591988 seconds (3.87%).
Success, validation, selected route, sampled motion, duration, and the complete
polynomial were exactly equal.

The complete suite passed 119/119. All 17 maintained examples ran serially in
fresh headless processes: 16 independently validated successes and the
independently validated expected no-path result. Every successful result passed
collision and kinematic certificates with no path-length or arrival-time drift.
The moving-circle control retained its exact trajectory and reported the
time-invariant flag false. A visible Static-U run created three figures; the
expected no-path run created two diagnostic figures with two axes.

The cache adds 29 net production lines and retains one additional shape plus
one geometry record per exactly static obstacle; focused tests add 12 net
lines. This bounded memory cost is retained for the measured repeated-query
and planner-level gains. The 183.495223-second moving/deforming U.S. example
remains the largest runtime weakness and receives no shortcut.

## Dimension-neutral polynomial map caches — 2026-08-27

Two one-entry caches now reuse HS3 polynomial structure that is invariant
across optimizer callbacks. `createAffineSensitivityModel` caches mesh
incidence and prior-segment integration ordering by segment count.
`createSubintervalBernsteinMap` caches one complete interval restriction map
by exact segment count, coefficient count, and normalized interval endpoints.
Both remain dimension-neutral; obstacle, Az/El, and planner concepts are absent.
Keys are published only with complete values, and one-entry replacement bounds
persistent memory.

On identical inputs, 600 affine-map calls decreased from 0.6946311 to
0.4367565 seconds (37.1%), and 10,000 repeated subinterval-map calls decreased
from 1.5852113 to 0.1228665 seconds (92.2%). A warmed, counterbalanced
Static-U comparison against exact commit `e1db1ed` decreased median wall time
from 10.4040084 to 10.35779345 seconds (0.44%). Success, validation, selected
route, sampled motion, duration, and the complete polynomial were exactly
equal. The modest end-to-end gain is reported as measured; fmincon factorization
still dominates total wall time.

The complete unit suite passed 119/119. All 17 maintained examples then ran
serially in fresh headless MATLAB processes: 16 independently validated
successes and the independently validated expected `noValidatedSeed` result.
Every successful trajectory passed collision and kinematic certificates, with
no path-length or arrival-time drift from the preceding `9ba28f4` matrix. The
moving/deforming U.S. case remained the largest weakness at 183.354475 seconds.
A visible obstacle-free run created three figures, and the expected no-path run
created two diagnostic figures with two axes.

The implementation adds 30 net production lines across the two neutral
polynomial helpers and seven net focused-test lines. That size is retained for
the measured repeated-call reductions and exact-output evidence, not as a claim
of a material solver-wide acceleration.

## Independent-validation complexity refactor — 2026-08-27

`obstacleAvoidance.validateTrajectory` now delegates sampled-history checks,
endpoint agreement, complete collision certification, and safety-margin
provenance to focused local helpers. The primary function's measured
cyclomatic complexity decreased from 62 to 19. Complexity for the complete
file decreased from 85 to 77 rather than merely moving intact into helpers;
the largest new helper is 14 and the largest retained helper is 18. The public
format, issue ordering, pass/fail decisions, exceptions, and collision evidence
are unchanged.

The retention gate compared normalized success and deliberately perturbed
failure records exactly and passed all nine focused collision, dynamics, and
timing tests. The complete unit suite passed 119/119 before the user-requested
removal of `exampleFortyMovingCircleGrid`; the reduced maintained inventory is
17 examples. All 17 ran serially headlessly: 16 independently validated
successes and the independently validated expected `noValidatedSeed` failure.
A default-options visible obstacle-free run also passed and created the normal
plots. Every successful run passed collision and kinematic certificates.

Two fresh metrics differ from the older `0302439` benchmark: the
moving/deforming U.S. path is 43.0751355347 degrees at 7.96286899667 seconds,
and the target-exit path is 20.6764423274 degrees at its fixed 24-second
arrival. Clean detached runs at exact pre-refactor commit `9ba28f4` reproduced
both values exactly, proving that this refactor introduced neither path-length
nor arrival-time drift. The older target-exit difference came from replacing
the example's seeded random target history with deterministic evenly spaced
steps; it is an input change, not a same-input planner regression.

The largest remaining production-function complexities are 85 in planner
orchestration, 53 in trajectory plotting, and 49 in polynomial trajectory
validation. The moving/deforming U.S. example took 184.596685 seconds in this
run, so runtime remains a major weakness. The removed 40-circle grid had still
not completed after roughly six minutes; historical measurements remain in
`benchmark.csv`, but it is no longer a maintained runnable example.

## Interactive polygon motion editing — 2026-08-27

The sandbox now creates polygon obstacles from explicit left-clicked vertices.
A right-click closes and retains the polygon after at least three non-collinear
vertices are present. The Set Motion action selects a retained polygon by an
interior click and previews an arrow from its centroid to a second click. Each
polygon stores its own motion vector and one of four profiles: immediate
constant velocity, acceleration from zero velocity, trapezoidal
accelerate-cruise-decelerate motion, or one oscillating out-and-back cycle.

Motion profiles are sampled into the same time-varying obstacle format used by
the production planner. Focused tests verify the midpoint and endpoint position
of every profile and confirm that the constant-velocity history produces the
expected query-time polygon centroid. A visible UI smoke test confirmed one
rendered sandbox figure, four profile choices, and the Set Motion action.
Code Analyzer reports zero findings for the four changed MATLAB files.

The vector defines total displacement over the active planning horizon. The
oscillating choice completes one full cycle during that horizon. The sandbox
does not yet expose a separate speed, acceleration time, cycle count, or phase
control. Existing freehand line data remains readable, but new obstacle input
through the UI is polygon-vertex based.

## High-level public namespace and normal example results — 2026-08-27

Production code is now exposed through the high-level +obstacleAvoidance
package and the separate normal hs3 product. The public planner calls are
obstacleAvoidance.planTrajectory,
obstacleAvoidance.planMovingTargetIntercept, and
obstacleAvoidance.validateTrajectory. Input, obstacle, geometry, search,
planner, and plotting ownership is visible in matching subpackages. The
dimension-neutral HS3 engine still has one public solveTrajHS3 entry and is
source-checked against Az/El domain dependencies.

Construction vocabulary now consistently uses create; query, combine,
convert, evaluate, solve, and validate remain distinct behavioral verbs.
HS3 polynomial helpers state their mathematical result, including
createTrajectoryPolynomial, evaluateTrajectoryPolynomial,
createAffineSensitivityModel, createSubintervalBernsteinMap, and
evaluateIntegratedSquaredJerk. Architecture tests reject the removed nested
product and legacy packages, require unique production basenames, and verify
that numerical optimization stays in HS3.

Examples no longer append names, controls, geometry, metrics, plot handles, or
sequence summaries to planner results. The deleted example-metrics generator
is replaced by external benchmark calculation, so every example returns the
normal public planner format. Independent validation and plotting remain local.
The moving-barrier and opening-U examples locally suppress only MATLAB's
nearlySingularMatrix and singularMatrix warnings during their expected
wait-seed solves, restore the caller's warning state, and still warn if
independent validation fails.

The plotting API now provides a synchronized spatial/position/velocity/
acceleration/jerk dashboard, fixed derivative limits, and optional GIF export.
A focused run produced a 95,507-byte two-frame GIF and four live kinematic
axes. The complete unit suite passes 117/117 and Code Analyzer reports zero
findings. All 18 maintained examples ran serially in fresh MATLAB processes:
17 independently validated successes and the independently validated expected
noValidatedSeed failure. A visible success created three figures, and the
expected failure created both workspace and time-expanded diagnostic figures.

No planner algorithm changed in this namespace/output refactor. Representative
trajectory metrics match the preceding route-cleanup evidence, including the
20-degree center-line accelerating-circle motion and 24.035784715-degree
two-opposing-U seed. The largest measured example wall time remains
55.227758 seconds for the extreme U.S. outline, so runtime remains the largest
current weakness and no performance improvement is claimed.

## Documentation and interactive-example cleanup — 2026-08-26

Eight redundant subfolder `README.md` files and the manual
`exampleAzElInteractiveSandbox` entry were removed. The repository-level
`README.md` remains the single documentation entry and no longer links to the
deleted guides or advertises the removed example. The example-requirement test no
longer carries a sandbox-specific exclusion; all 18 maintained examples are
now covered uniformly by the file-level metadata check.

This cleanup removes nine files without changing planner, obstacle, validation,
plotting, or HS3 behavior. The focused example-requirement suite passes 6/6 and
Code Analyzer reports zero findings for the modified test. No maintained
example was executed, so `benchmark.csv` correctly retains its last measured
rows without a fabricated cleanup measurement. Historical verification text
that records when the interactive example existed remains unchanged.

## Same-homology spatial route cleanup — 2026-08-26

The spatial visibility product now removes avoidable consecutive route edges
after homology-augmented search and before HS3. A replacement is retained only
when the direct segment is visible in the same protected swept geometry, the
existing winding signature matches the searched class, and the route becomes
strictly shorter beyond a scale-aware numerical tolerance. The original route
is the fallback. Direct and timed seeds are not rewritten, visibility search
still discovers the topology, and the dimension-neutral `hs3/` engine remains
unchanged and Az/El-agnostic.

The primary six-rectangle gate reduced the aggregate length of four homology
routes by 0.0065005233 degrees with two accepted shortcuts. A structurally
different four-circle case accepted eight shortcuts, rejected 172 visible
candidates for changing homology, and reduced aggregate route length by
86.1926109665 degrees. Independent tests recomputed every retained signature,
route length, and protected-geometry clearance. An existing two-rectangle case
accepted no shortcut and retained its original lengths, demonstrating the
no-benefit fallback.

The strongest maintained-example benefit is the two-opposing-U selected seed,
which decreased from 24.5077116377 to 24.0357847150 degrees while the final
motion remained independently collision-free and kinematically certified.
Alternating slalom also decreased from 16.0604396350 to 16.0193197983 degrees.
All 18 maintained examples completed serially: 17 validated successes and the
validated expected no-path result. The focused planner suite passed 59/59 and
the complete suite passed 114/114. Search diagnostics expose candidate,
rejection, acceptance, and length-reduction counts.

This cleanup proves neither global shortestness nor homotopy completeness; it
only reaches a deterministic direct-shortcut fixed point within each route's
searched homology signature. The most unfavorable fresh maintained wall time
was 61.034329 seconds for the extreme outline, above its preceding 56.218460
seconds, so no runtime benefit is claimed. Existing singular-solver warning
floods and the moving/deforming-outline runtime remain current weaknesses.

## Certified direct-path collinearity — 2026-08-26

The Az/El adapter now preserves the Euclidean path for a certified direct
fixed-position request when endpoint velocity and acceleration are parallel to
that path and one axis governs every finite normalized velocity, acceleration,
and jerk limit. The common-bottleneck condition prevents a spatial preference
from tightening the earliest-arrival bound in mixed-axis cases. Obstacle-free
requests are certified directly; obstacle-present requests require the spatial
visibility certificate, while collinear timed-search seeds retain their
time-aware provenance. Moving targets and incompatible endpoint derivatives
remain unrestricted. The dimension-neutral `hs3/` product is unchanged.

The saved `Rogue Examples/Bend2.mat` request is the primary measured gate. At
baseline `69cef57`, its straight 94.4183046111-degree seed became a
95.3959651184-degree motion with 2.47268757136 degrees of line deviation. The
retained implementation returns 94.4183046111 degrees with
2.91322521662e-13-degree deviation. Arrival is unchanged at
39.5285834641 seconds, and independent collision, velocity, acceleration,
jerk, dynamics, and endpoint validation all pass. The measured planner time
increased from the saved 0.8398472 seconds to 3.0007202 seconds because the
fixed-time active-set QPs carry additional normal-jerk equations. This is an
explicit correctness/shortest-path tradeoff, not a runtime improvement.

A structurally different diagonal request with irrelevant protected geometry
also attained its exact Euclidean lower bound after the visibility graph
certified the direct edge. A nonparallel endpoint-velocity request remained
unrestricted and valid. Exact directional-gradient coverage verifies the
reduced terminal equations and normal-jerk rows. Code Analyzer reports zero
findings for all six changed production/test files; the affected suites pass
64/64 and the complete repository suite passes 112/112 in 62.332678 seconds.

All 18 maintained examples ran serially in fresh MATLAB processes: 17 returned
independently validated success and the expected no-path case returned an
independently validated `noValidatedSeed` failure. Every successful result
passed collision and kinematic certificates. The obstacle-free example and
four-accelerating-circle demo both retained exact Euclidean motion length. A
visible success created three figures, and the visible expected failure
created two diagnostic figures. Existing near-singular `fmincon` warning
floods remain visible in moving-barrier and opening-U; they are not caused or
hidden by this change.

## Two-product normal-folder architecture — 2026-08-26

Production source now has two top-level ownership roots: `planAzElMotion/` and
`hs3/`. The repository root contains no MATLAB functions or package folders.
The Az/El product exposes three unqualified entry points for planning,
interception, and independent validation; its obstacle and plotting APIs are
owned directly by `azElObstacles.*` and `azElPlotting.plotMotion`. This removes
the redundant plotting facade and four loose obstacle files.

The dimension-neutral engine is now a normal `hs3/` folder with one public
`solveTrajHS3.m` entry and a single `+hs3Internal/` implementation package.
Az/El constraint adaptation still delegates optimization, polynomial
reconstruction, evaluation, sensitivity, and Bernstein operations to that
neutral engine. Architecture tests continue to reject numerical solver calls
from the Az/El product and Az/El domain language from every HS3 source file.

This is an intentional API migration: callers must add both product folders,
use qualified obstacle and plotting functions, and replace `hs3.solve` with
`solveTrajHS3`. No compatibility wrappers remain. The migration table is in
`README.md`.

Code Analyzer reports zero findings across both production products and the
architecture test. Focused architecture, obstacle, and standalone-HS3 tests
pass 29/29; example-requirement tests pass 6/6; and the complete suite passes
108/108 in 63.545976 seconds. All 18 maintained examples ran serially and
completed in 185.535980 seconds: 17 independently validated successes and the
expected independently validated `noValidatedSeed` failure. Every successful
motion passed collision and kinematic certificates. A visible success created
three figures, and the hidden expected failure created two diagnostic figures.

The example matrix used one MATLAB process because repeated fresh launches
intermittently failed in the MathWorks launcher before repository code ran.
Consequently its wall times are verification evidence, not a runtime
comparison against earlier fresh-process matrices. Existing `fmincon`
conditioning warnings remain visible in the moving-barrier and opening-U
cases; independent validation passed both.

## Fixed-arrival geometric lower-bound proof — 2026-08-26

Dynamic fixed-arrival requests now evaluate retained topology seeds in
increasing geometric length and stop only when an independently validated
motion attains the Euclidean start-goal distance within numerical tolerance.
This is an input-derived global spatial lower bound, not a scenario-specific
route heuristic. If the bound is not attained, bounded multi-seed length-first
selection remains active. Earliest-arrival behavior and frozen `+hs3` code are
unchanged.

The four-accelerating-circle demo now uses a 42-second conceptual smooth
rest-to-rest obstacle profile but supplies only the 0--22-second planning
history relevant to its fixed arrival. It independently validates the exact
20-degree center-line motion at 22 seconds with collision and every derivative
certificate passing. One of three available seeds is attempted because the
first valid seed attains the geometric lower bound.

Before retention, the same demo supplied 421 history slices and took
62.021637 seconds with plots disabled. Exclusive planner timing attributed
34.9519 seconds to corridor construction, 21.6022 seconds to collision
checking, and only 2.1221 seconds to HS3 motion solving. `polyshape.union`
accounted for 48.2364 profiled seconds. Clipping post-arrival history alone
reduced wall time to 26.148349 seconds; lower-bound seed ordering and stopping
reduced the final serial wall time to 5.634472 seconds, an 11.01x speedup.

Fresh verification passes 106/106 tests and all 18 maintained examples. The
matrix has 17 independently validated successes and the expected validated
`noValidatedSeed` result. The final matrix used 297.286362 seconds. The
earliest-arrival moving-circle case returned a valid 8.707031-second motion,
0.0609997 seconds later than the preceding record; because the retained rule
is fixed-arrival-only, this is reported as existing wall-budget variability,
not hidden as an improvement.

## Fixed-arrival length-first candidate quality — 2026-08-26

Fixed-arrival planning now retains the shortest independently validated
sampled motion within each HS3 seed and selects the shortest across every
validated seed completed within the existing global work budget. Integrated
squared jerk is only a path-length tie-breaker. Earliest-arrival ranking and
refinement are unchanged. The dimension-neutral `+hs3` engine remains frozen;
this policy uses only its returned motions and independent Az/El validation.

Against clean commit `855a569`, the focused four-accelerating-circle case
kept its exact 22-second arrival and shortened from 27.8702009821 to
25.9348981999 degrees, a 1.9353027822-degree or 6.943986% improvement.
The fixed alternating-occlusion intercept shortened from 14.2200520815 to
13.6782719080 degrees (3.809973%), and target-exits-obstacle shortened from
21.3509241121 to 20.3320561588 degrees (4.772009%). The straight specified-time
intercept remained at its geometric lower bound of 9.53894054682 degrees.
Every fixed arrival time, collision certificate, and kinematic certificate
remained unchanged.

The apparent 20-degree center-line route in the four-circle case is not
physically feasible at 22 seconds. Protected geometry blocks the complete
center line from 7.3664844164 through 12.6335155836 seconds. Early clearance
needs at least 8.075 seconds even without acceleration. After reopening, an
optimistic state already at the left protected boundary with maximum forward
speed still fails the HS3 limits by 0.0880882; the jerk-limited cruise-and-stop
lower bound is about 9.558 seconds versus 9.366 seconds available.

Fresh verification contains 18/18 maintained outcomes in 254.1345943 seconds:
17 independently validated, collision-free successes with passing kinematic
certificates and the expected validated `noValidatedSeed` result. The focused
planner suite passes 54/54. Fixed static multi-route coverage now asserts that
all retained seeds are attempted and the minimum validated motion length is
selected. The tradeoff is bounded extra work for static fixed-arrival requests:
instead of stopping after the first valid seed, planning may evaluate up to
`MaximumSeedCount` candidates within `MaximumPlanningTime_s`.

## Flat architecture and frozen HS3 boundary — 2026-08-26

Production code now has six one-level Az/El packages: input, obstacles,
geometry, search, planner, and plotting. The separate root `+hs3` package is
unchanged from commit `ad3139c`. The former nested `+azElInternal` and
`+azElPlannerMethods` trees and their forwarding or duplicate implementations
are removed. Root public entry points remain stable.

The solver dependency is now explicit and tested. `+azElPlanner` translates
unit-bearing Az/El state and corridor records into dimension-neutral HS3
records. Optimization, polynomial reconstruction, and polynomial evaluation
then run through `hs3.optimize`, `hs3.reconstructPolynomial`, and
`hs3.evaluatePolynomial`. No Az/El package calls `fmincon`, `quadprog`, or
`optimoptions`; those calls occur only in frozen `+hs3/optimize.m`. HS3 MATLAB
source contains no Az/El, obstacle, visibility, topology, corridor, plotting,
or planner dependency.

Fresh evidence is 104/104 automated tests in 52.560 seconds, including seven
new architecture-boundary tests, and zero Code Analyzer findings across the
production and test MATLAB files. All 18 maintained examples ran serially in
fresh headless processes in 241.822 seconds: 17 independently validated,
collision-free successes with kinematic certificates and the expected
independently validated `noValidatedSeed` failure. A visible success created
three figures; the failure check created two diagnostic figures and retained
its search grid. Exact rows are in `benchmark.csv` under
`ad3139c+flat-architecture-worktree`.

The largest remaining weakness is numerical conditioning inside the frozen
HS3 nonlinear solve: the moving-barrier and opening-U runs emitted repeated
near-singular or singular working-precision warnings even though their final
trajectories passed independent collision and kinematic validation. This
architecture change does not claim a solver-quality or runtime improvement.

## Standalone dimension-neutral HS3 extraction — 2026-08-26

The HS3 polynomial and optimization engine is now available through the root
`hs3.solve(initialState, terminalState, limits, options, pathConstraints)`
interface. It supports arbitrary state dimension, fixed and earliest arrival,
continuous Bernstein position/velocity/acceleration/jerk bounds, and affine
point or single-segment interval path constraints. Azimuth/elevation obstacle,
visibility, topology, seed, moving-target, plotting, validation, and planner
assembly concerns remain outside the engine. Existing production planning
delegates the shared numerical solve and leaf mathematics to the new package;
deprecated internal aliases preserve existing callers.

Extraction evidence is frozen against commit `4827e47` and seed 325. The
three-run scaling matrix passed all 15 candidate cases with independent
validation and zero behavioral or numeric mismatches. Candidate versus
baseline median planner times were 1.5928771 versus 1.6185557 seconds for one
turn, 38.0545350 versus 37.9652044 for five turns, 49.1296070 versus
49.9628830 for ten turns, 476.9026500 versus 476.8622450 for twenty turns,
and 154.1580630 versus 154.6614689 for the 12-wall hairpin. The median
five-case total was 721.395402 seconds versus 723.185449 seconds, within the
required five-percent aggregate gate. This proves extraction parity only; it
does not claim a new planning algorithm, broader completeness, or uniform
speedup.

The exact isolated commit passes 101/101 repository tests, including the 18
standalone-kernel, 6 affine-sensitivity, 4 option-ownership, and 52 production
planner tests. Code Analyzer reports zero findings across all 100 MATLAB files.
All 18 maintained examples were then run serially in fresh headless MATLAB
processes: 17 returned independently validated success with collision and
kinematic certificates, and the expected no-path example returned an
independently validated `noValidatedSeed` failure. A visible success created
three figures, and the visible expected failure created two search-diagnostic
figures. The exact per-example metrics and wall times are retained in
`benchmark.csv` under source `725c91d`.

## Severe-static fixed-time quality search — 2026-08-26

The retained improvement addresses a severe static-route discretization local
minimum without encoding example identity. On the first quality decision only,
an earliest-arrival spatial candidate whose relative sampled-motion inflation
exceeds `2.5 / segmentCount` is re-solved on the configured maximum mesh as a
fixed-arrival feasibility problem. Existing timed bisection then shortens the
horizon while retaining the original topology seed. Dynamic obstacles, timed
topologies, fixed-arrival requests, later mesh passes, and less-inflated static
routes keep their previous behavior. No public option or result field was
added.

Wide U now independently validates at 22.6308876389 seconds, with a
34.9425880405-degree selected polyline, 41.5363500661-degree sampled motion,
64 segments, and one mesh pass. Relative to the preceding
`7661321+slack-quality-worktree` row, arrival improves by 0.4075710153 seconds
and final wall time improves from 19.117491 to 17.688485 seconds. The sampled
motion is 0.3257153577 degrees longer, so this is an arrival improvement rather
than a uniform path-quality claim. The remaining like-for-like 325 gap is
0.7981520967 seconds; wall time remains 11.9222537 seconds slower and sampled
motion remains 1.8321728042 degrees longer than 325.

The final `7661321+fixed-quality-worktree` matrix contains 18 fresh serial
rows: 17 independently validated successes and the independently validated
expected `noValidatedSeed` result. Two opposing U remains exactly
21.9090824092 seconds, forty moving circles remains 61.2011842765 seconds,
extreme U.S. remains 6.3679977362 seconds, and the moving/deforming U.S. remains
8.75061035156 seconds. Tests pass 82/82 in 50.675781 seconds. Code Analyzer
reports zero messages across 84 MATLAB files. Visible success and expected
failure each produce two figures. The two rogue horizon replays remain
independently valid at 88.2939404925 and 88.2939359679 seconds, a
4.525-microsecond difference. The HS3 package contains exactly 2,000 nonblank,
noncomment MATLAB lines, at the hard cap.

Rejected broader variants remain visible. Applying the fixed-time search to
all static candidates increased two-opposing-U wall time from about 14 to
20 seconds and only reached 22.996 seconds for wide U. An
interior-point-convex fixed solve restored feasibility where active-set could
not, but individual probes took 80--87 seconds and a shorter target exceeded
12 minutes. A 64-segment extreme-U.S. probe regressed arrival by
0.00194985 seconds, and 80 segments regressed further. These variants were not
retained.

Remaining arrival gaps against the same 325 matrix are forty moving circles
+0.8393466878 seconds, wide U +0.7981520967 seconds, and extreme U.S.
+0.3610623598 seconds. Two opposing U is 0.2018852770 seconds earlier than
325. Moving/deforming U.S. is 0.3892445513 seconds earlier, but its final wall
time is 56.253798 seconds versus 18.2106663 seconds on 325, a
38.0431317-second runtime regression. No global optimality, completeness, or
uniform runtime claim is made.

## Derivative-slack continuation quality pass — 2026-08-26

The retained improvement addresses a time-discretization local minimum without
encoding obstacle or example identity. After one valid same-mesh
relinearization, an earliest-arrival spatial candidate receives one 2x
continuation-seeded mesh pass only when every acceleration and jerk peak is
below 75% of its applicable limit. The slack is input-derived evidence that
the coarse motion is velocity-dominated and that a finer time mesh can improve
arrival without asking for a new topology. Existing length-inflation passes
continue to start from their original topology seed; this distinction prevents
the forty-moving-circle and extreme-U.S. regressions observed in rejected
probes.

Two opposing U obstacles now independently validate at 21.9090824092 seconds,
with a 24.5077116377-degree selected polyline, 24.4201122273-degree sampled
motion, 20 segments, and one mesh pass. Two focused repeats and the final
matrix reproduced the identical arrival. This is 0.9660319268 seconds earlier
than the preceding `7661321+dynamic-quality-worktree` row and 0.2018852770
seconds earlier than the like-for-like
`da52da8+quintic-root-recovery-worktree` 325 row. The final wall time is
13.972852 seconds, versus 4.6082376 seconds in the preceding worktree and
8.5152389 seconds on 325, so the arrival gain is not presented as a runtime
gain.

The final `7661321+slack-quality-worktree` matrix contains 18 fresh serial
rows: 17 independently validated successes and the independently validated
expected `noValidatedSeed` result. All other arrival and path metrics match
the preceding matrix. Tests pass 82/82 in 50.245827 seconds. Code Analyzer
reports zero messages across 84 MATLAB files. Visible success produces three
figures; expected failure produces two diagnostic figures. The two rogue
horizon replays validate at 88.2939404925 and 88.2939359679 seconds, a
4.525-microsecond difference. The HS3 package contains exactly 2,000 nonblank,
noncomment MATLAB lines, at the hard cap.

Remaining arrival gaps against the same 325 matrix are wide U
+1.2057231120 seconds, forty moving circles +0.8393466878 seconds, and extreme
U.S. +0.3610623598 seconds. Moving/deforming U.S. remains 0.3892445513 seconds
earlier but its final measured wall time is 55.917228 seconds versus
18.2106663 seconds on 325. No global optimality, completeness, or uniform
runtime claim is made.

## Dynamic spatial quality pass — 2026-08-26

The largest remaining arrival gap was localized to dynamic spatial mesh
resolution rather than topology. Forty moving circles already used the same
110.807922148-degree polyline as the 325 comparator, but the 10-segment HS3
motion inflated to 126.23121736 degrees and arrived at 64.5557730468 seconds.
Starting every seed at a finer mesh improved arrival but repaid the failed
direct seed: 20 segments reached 61.2011842765 seconds in 23.9281868 seconds
wall, and 30 reached 60.1588345587 seconds in 31.5052284 seconds wall.

The retained rule instead refines only an already validated earliest-arrival
spatial candidate whose sampled motion is inflated by more than one coarse
mesh interval. Changing-obstacle spatial routes receive one 2x pass;
fixed-arrival cases and causal timed topologies are excluded. The authoritative
default run reaches the same 20-segment, 61.2011842765-second motion in
18.6109941 seconds wall. Relative to the preceding worktree row, arrival
improves by 3.3545887703 seconds for 2.1703734 seconds additional wall. The
like-for-like `da52da8+quintic-root-recovery-worktree` 325 gap falls from
4.1939354581 to 0.8393466878 seconds. The 30-segment probe is 0.20300303
seconds earlier than 325 but costs 12.8942343 seconds more wall than the
retained 20-segment result, so it was rejected as the default.

Structurally different controls preserve scope. Moving circle has only 6.4%
motion inflation, stays at 10 segments with zero quality passes, and retains
its 8.64603156476-second arrival. The moving/deforming U.S. result remains a
`timeExpandedVisibilityGraph` seed at 8.75061035156 seconds with zero mesh
passes. Fixed-arrival accelerating circles retains its 22-second motion.
Single dense translating ellipses and circles with timed-search suppression
measured 5.2%, 8.7%, and 6.7% inflation and correctly did not trigger the pass.

The final `7661321+dynamic-quality-worktree` matrix contains 18 fresh serial
rows: 17 independently validated successes and the independently validated
expected `noValidatedSeed` result. Tests pass 82/82 in 50.3745157 seconds.
Code Analyzer reports zero messages across 84 MATLAB files. Visible success
produces three figures and 526 objects; expected failure produces two figures
and 341 objects with 15 rejected transitions and 9 retained rejected edges.
The HS3 package contains 1,999 nonblank, noncomment MATLAB lines, one below the
hard cap.

Remaining arrival gaps against the same 325 matrix are now wide U
+1.205723112 seconds, forty moving circles +0.8393466878 seconds, two opposing
U obstacles +0.7641466498 seconds, and extreme U.S. +0.36106235982 seconds.
Moving/deforming U.S. remains 0.3892445513 seconds earlier but 31.5077561
seconds slower wall. No global optimality, completeness, or uniform runtime
claim is made.

## Ordered-boundary and route-quality Pareto — 2026-08-26

The largest new quality gain is shortest-route-first static proposal ordering.
The extreme U.S. sequence previously attempted a 22.3733117302-degree,
five-point route first because waypoint count inflated the ordering score; the
shorter 22.2394635087-degree, seven-point route was never attempted. Per-seed
work is already budgeted, so earliest-arrival ordering now uses geometric
length alone. The final three-region example selects the shorter Philippines
route and independently validates at 6.3679977362 seconds with a
24.6064786878-degree sampled motion and 20 segments. This improves the prior
worktree arrival by 1.85831341561 seconds and reduces the like-for-like
`da52da8+quintic-root-recovery-worktree` gap on `325-full-suite` to
0.36106235982 seconds. Its authoritative final wall time is 64.0234264 seconds,
4.5955795 seconds slower than the preceding worktree row and far slower than
the 9.8415693-second 325 row, so this is not presented as a uniform runtime
gain.

Static quality refinement remains one bounded pass. Motions inflated by more
than one coarse mesh interval receive 2x segments; only severe inflation above
2.5 intervals receives 3x. The wide-U coarse motion measures 2.586 intervals
of inflation and therefore preserves its 30-segment, 23.0384586542-second
validated result. Extreme U stays at 20 segments. The rogue horizon pair also
uses 20 segments: the 180-second request validates at 88.2939404925 seconds in
6.9218305 seconds wall, and the 360-second request validates at
88.2939359679 seconds in 7.5901807 seconds wall, a 4.52-microsecond horizon
difference. This deliberately gives back about 1.144 seconds versus the
30-segment rogue result while cutting refinement work; it remains 2.725 seconds
earlier than the coarse result and fully resolves the original horizon failure.

Moving-obstacle corridor construction no longer rebuilds a `polyshape` merely
to recover orientation and convexity from a canonical ordered single-region
boundary. `shapeAtTime` reports those properties directly from the ordered
vertices; multi-region and degenerate boundaries retain the existing
`polyshape` fallback. A focused convex, concave, and multi-region regression
freezes equivalence. Forty moving circles keeps the identical
64.5557730468-second motion while wall time falls from 20.6046246 to
16.4406207 seconds. Moving circle falls from 10.3724330 to 9.4004092 seconds,
four accelerating circles from 28.6209839 to 25.5803952 seconds, and the
moving/deforming U.S. example from 52.9070181 to 49.4045848 seconds, with all
physical metrics and certificates unchanged.

The authoritative final matrix under
`7661321+geometry-fastpath-worktree` contains 18 serial fresh-process rows:
17 independently validated successes and the independently validated expected
`noValidatedSeed` failure. The repository suite passes 82/82 in
49.6429588 seconds. Code Analyzer reports zero messages across 84 MATLAB files.
Visible success produces three figures and 526 objects; expected failure
produces two figures and 341 objects with 15 rejected transitions and 9
retained rejected edges. The HS3 package contains 1,998 nonblank, noncomment
MATLAB lines, two below its hard cap.

Remaining arrival gaps against the like-for-like 325 matrix are explicit:
forty moving circles +4.1939354581 seconds, wide U +1.205723112 seconds, two
opposing U obstacles +0.7641466498 seconds, and extreme U.S. visibility
+0.36106235982 seconds. Moving/deforming U.S. remains 0.3892445513 seconds
earlier than its 325 row but 31.1939185 seconds slower wall. No global
optimality, completeness, or uniform runtime claim is made.

## Static quality and time-expanded retiming — 2026-08-26

The largest current arrival gain is on the maintained moving/deforming U.S.
case. The causal time-expanded topology previously validated at
30.1605224609 seconds. One bounded alternative now removes zero-length waits,
distributes that same input-derived topology by arc length, solves it at a
physical-duration target, and retains it only after the ordinary independent
validator passes. The final serial run selects that time-expanded seed at
8.75061035156 seconds, with a 41.5785140688-degree polyline, a
40.7424283094-degree sampled motion, passing collision and kinematic
certificates, and 52.9070181 seconds wall. This is 21.4099121093 seconds
earlier than the immediately preceding worktree result. It is about
0.3892445513 seconds earlier than the like-for-like
`da52da8+quintic-root-recovery-worktree` row on `325-full-suite`, while wall
time is 34.6963518 seconds slower. The earlier commit-only comparator predates
the extreme 25-slice deformation, so comparisons here use the later recorded
branch matrix rather than infer parity from that older source state.

The alternative retiming is limited by seed semantics, not an example name.
Only `timeExpandedVisibilityGraph` seeds are eligible. A `directWait` seed's
repeated point is its causal law and remains on the absolute-time bisection
path. An intermediate broad implementation exposed this distinction:
opening-U selected a physically valid direct-visibility motion that failed the
example's required waiting demonstration. After the semantic restriction,
opening-U reproducibly returns the direct-wait seed at 11.8560791016 seconds
in 13.0011819 seconds wall, and moving barrier returns the direct-wait seed at
10.2314453125 seconds in 12.4091501 seconds wall. Both independently validate.

Static spatial candidates now receive at most one 3x mesh-quality pass when
their sampled motion-length inflation exceeds one coarse mesh interval. The
saved rogue detour improves from the horizon-invariant 91.0189-second result
to 87.1503426168 seconds at a 180-second horizon and 87.1503401418 seconds at
360 seconds. Both select seed 2, use 30 segments, and pass independent
collision and kinematic validation; the horizon difference is about
2.5 microseconds. The historical saved 86.5088536619-second, 40-segment
trajectory also passes today's validator, so the remaining roughly
0.6415-second local-quality gap stays visible. A nonfinite free-time duration
probe is now mapped to a finite bound, and a nonfinite arrival objective is
given a finite rejection value, which recovers feasible high-resolution
iterates without accepting them unless independent validation passes.

The authoritative post-fix matrix ran all 18 maintained examples serially in
fresh MATLAB processes. Seventeen are independently validated successes and
the expected no-path case is an independently validated `noValidatedSeed`
failure. Exact rows are appended to `benchmark.csv` under
`7661321+timed-retiming-worktree`. The complete repository test suite passes
81/81 in 52.4952677 seconds. Code Analyzer reports zero messages across all 84
MATLAB files, and `git diff --check` reports no whitespace errors beyond
line-ending notices. A visible success creates three figures and 526 graphics
objects. The expected failure creates two diagnostic figures, reports 15
rejected transitions, and retains 9 rejected edges for plotting. The HS3
package contains exactly 2,000 nonblank, noncomment MATLAB lines, so it passes
but has no remaining size headroom.

Unfavorable arrival gaps remain against the
`da52da8+quintic-root-recovery-worktree` matrix on `325-full-suite`: forty
moving circles is 4.1939354581 seconds later, extreme U.S. visibility is
2.21937577543 seconds later, the wide U is 1.205723112 seconds later, and two
opposing U obstacles is 0.7641466498 seconds later. Those rows are not hidden
or reclassified. The moving/deforming U.S. example still spends roughly
19 seconds in repeated protected-polygon construction (`polybuffer` dominates)
and remains about 53 seconds wall even after the arrival repair. No global
optimality, completeness, or uniform runtime claim is made.

## Timed-arrival and exhaustive-failure repair — 2026-08-26

The largest newly measured correctness gain is horizon invariance for a saved
static detour. A prior edit reused the independent-axis velocity lower bound as
the HS3 warm-start duration. On identical geometry this made the 180-second
request lose its best topology and arrive at 98.7494014538 seconds, while the
360-second request arrived at 88.29393436 seconds. Reachability and incumbent
pruning now use only the physical per-axis bound; solver initialization uses a
conservative route-length estimate clamped to the available horizon. Fresh
serial replays select the same seed and arrive at 91.0188996291 and
91.0189002025 seconds respectively, a 5.734e-7-second numerical difference.
Both pass independent collision and kinematic validation. This repair is
input-driven and has a structurally different tall-detour regression; it does
not claim global optimality. The consistent 91.019-second result is still
2.724965 seconds later than the pre-repair 360-second worktree run and
4.510050 seconds later than the saved historical result, so arrival quality on
this topology remains an explicit weakness.

All four supplied rogue bundles now replay as independently validated
successes. The unobstructed and concurrent-axis cases retain arrivals of
57.5394882088 and 57.5394875671 seconds. The full repository suite passes
79/79 in 51.045611 seconds, and Code Analyzer reports zero messages across all
84 MATLAB files. Fresh maintained static, moving, and expected-no-path controls
preserve their prior physical metrics. The complete 18-example matrix remains
untested in this worktree, so no uniform suite-runtime claim is made.

The current worktree preserves timed obstacle events while searching earlier
fixed-time HS3 feasibility with exact QPs. Opening-U improved from 15 seconds
and 57.61 seconds wall to 11.8560791016 seconds and 12.9626115 seconds wall.
Moving barrier improved from 10.5 seconds and 25.18 seconds wall to
10.2314453125 seconds and 12.6557865 seconds wall. Both pass independent
collision and kinematic validation. Against commit `67bc087` on
`325-full-suite`, opening-U remains 0.122795 seconds later and 1.20 times
slower, while moving barrier remains 0.016544 seconds later and 1.75 times
slower. Those unfavorable gaps remain open.

An exact, exhaustive, untruncated static visibility failure now skips HS3
instead of refining an impossible direct topology. The maintained no-path
example retains `noValidatedSeed`, complete search diagnostics, and independent
failure validation while improving from 53.8261095 seconds to 1.3769188
seconds wall. Reduced or dynamic graphs do not use this certificate.

The earlier full repository suite passed 78/78 in 44.1178 seconds. A visible successful
example created three figures and 527 graphics objects; the expected no-path
case created two diagnostic figures with 15 rejected transitions. Basic static
planning remains 7.57952069664 seconds and moving circle preserves its earlier
8.64603156476 seconds arrival, but moving-circle wall time is 10.7064233
seconds versus the edited pre-repair 9.0140762 seconds. A complete 18-example
final-source matrix has not been rerun because the user stopped that comparison
after the focused regressions were identified. The HS3 package contains 1,927
nonblank, noncomment MATLAB lines, below its 2,000-line ownership cap.

## Sandbox export recovery — 2026-08-25

Diagnosis export no longer depends exclusively on the save-dialog callback.
Every live public sandbox snapshot now exposes
`ExportBundle(filePath, modeName)`, which reads the current guidata-backed
state and writes the same diagnosis bundle to an explicit path. The writer
verifies a nonempty file and the required `diagnosisBundle` MAT variable before
reporting success. UI export errors are surfaced in a modal dialog rather than
being visible only in the tab log.

The focused export suite passed 4/4, including a real no-dialog file write from
the public sandbox state, and Code Analyzer reported zero messages in the two
production files and focused test. Two visible user runs then isolated a
cross-version UI defect missed by hidden tests: the `uiputfile` filter cell used
MATLAB strings where that API requires character arrays. The filter, dialog,
`save`, `whos -file`, `version`, and `datetime` compatibility boundaries now
use character arguments. Export failures also report their identifier and
earliest source line. A final visible rerun remains required.

Pre-run export is now an explicit supported state. Export becomes available
once a tab contains scene data; the bundle captures current controls and
canonicalized geometry without invoking HS3. Complete Goal Mode scenes also
include replayable planner inputs. The bundle reports `PlanningState =
"notRun"`, `HasPlannerResult = false`, and an empty result rather than
fabricating success, failure, or validation evidence.

The expanded focused export suite passed 5/5 with zero Code Analyzer messages,
including a public pre-run Goal Mode export that wrote, reloaded, and checked
the exact requested endpoints and HS3 options.

## HS3-only production cutover — 2026-08-25

The branch now has one production planner implementation and one public
selection: HS3. The corridor-quintic package, its private motion/search code,
method-specific tests and benchmarks, and superseded spline benchmark artifacts
are removed. Maintained examples, the moving-target wrapper, sandbox controls,
test requirements, benchmark drivers, and active documentation now resolve HS3
only. The production MATLAB surface decreased from 55 files and 10,346
physical lines at `67bc087` to 44 files and 7,821 physical lines: 11 files and
2,525 lines removed.

The strongest current evidence is the complete post-cutover suite: 75/75 tests
passed in 366.849286 seconds. The focused HS3 verification also passed 67/67 in
388.369877 seconds, including option ownership, affine sensitivity, planner
behavior, stage timing, and maintained example requirements. Code Analyzer
reported zero messages across all 83 remaining MATLAB files, and a direct HS3
request returned a successful independently validated five-second trajectory
with `SelectedMotionSource = "hs3"`.

The main remaining weakness is numerical conditioning. Moving-barrier and
moving-target coverage still emits many near-singular `fmincon` warnings even
though the returned trajectories pass independent validation. The required
fresh serial example matrix, visible graphics smoke, and expected-failure
figure check could not be rerun because their fresh MATLAB processes failed
before user code with `System Error: File system inconsistency`. No new
benchmark rows are recorded from those failed startups. Therefore the cutover
does not claim a fresh full-matrix runtime or graphics result.

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
frozen optimized-main row, and the expected no-path requirement passed. This is
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
   and did not satisfy the maintained quintic polynomial format. Its code was
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
no-path case returned `noValidatedSeed` and passed its failure requirement. Every
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
requirement: the primary function owns the complete Section 0 readme, while every
local function starts with a direct explanation and relies on nearby comments
for loops, decisions, state transitions, and failure paths. No local Section 0
or duplicate local `PURPOSE` block remains. The largest planner and search
files also explain why bounded fallbacks and candidate transitions occur, not
only what each condition tests.

The short-file audit found no unused production MATLAB file below 100 code
lines. The four single-caller helpers remain separate because merging them
would hide a stable result format or add a distinct algorithm to an existing
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

## Compact instrumentation assessment — 2026-08-23

The current recommendation is to retain the compact instrumentation. Both
methods now expose the same stable seven-field `StageTiming` record:
`TopologyElapsedTime_s`, `CorridorConstructionElapsedTime_s`,
`MotionSolvingElapsedTime_s`, `CollisionCheckingElapsedTime_s`,
`FinalValidationElapsedTime_s`, `UnattributedElapsedTime_s`, and
`TotalElapsedTime_s`. The five named stages are exclusive, unattributed time
reconciles them to the independently measured total, and attempted candidate
work is accumulated before selection so failed or discarded work remains
visible instead of being overwritten.

The fixed-duration constraint construction is also materially healthier. One
shared affine builder now serves the direct, Compact C3, exact-traversal, and
dynamic-repair motion paths, removing divergent copies of the same
coefficient-to-state mapping. The public timing format and this shared builder
are the useful core of the change; the prototype solver microtimers, activity
trees, and public HS3 attempt ledger should remain removed.

Both planners passed all 18 maintained examples in fresh processes: 17
independently validated successes and the expected validated `noValidatedSeed`
failure for each method. The alternating three-repetition A/B used 48 fresh
serial MATLAB processes against frozen `27070ac`. Status, validation, selected
seed, and physical requirements matched; the largest physical numeric difference
was only `1.3056223108405e-13 deg` in corridor U-case clearance, far below the
`1e-6` audit threshold.

The final source also passed all 127 automated tests with no failure or
incomplete result, and MATLAB Code Analyzer reported zero findings across all
93 maintained production and test files.

Positive percentages below are slower candidate medians; negative values are
faster. The mixed directions are evidence against claiming a uniform speedup.

| Method | Representative case | Direct wall | Harness wall | Planner median (baseline -> candidate) | Planner change |
| --- | --- | ---: | ---: | ---: | ---: |
| HS3 | Obstacle free | +1.308% | -9.959% | 3.8779488 -> 3.6864421 s | -4.938% |
| HS3 | U shaped | +2.650% | +0.111% | 7.7324331 -> 7.9896984 s | +3.327% |
| HS3 | Four accelerating circles | -0.198% | -3.193% | 23.5276799 -> 22.7331264 s | -3.377% |
| HS3 | Expected no path | -4.035% | -4.995% | 27.3161664 -> 25.9819868 s | -4.884% |
| Corridor | Obstacle free | -5.652% | -0.090% | 1.7478295 -> 1.8684094 s | +6.899% |
| Corridor | U shaped | +0.848% | -1.667% | 7.5230662 -> 7.5110127 s | -0.160% |
| Corridor | Four accelerating circles | -3.841% | -6.203% | 5.8348097 -> 5.5424184 s | -5.011% |
| Corridor | Expected no path | +2.899% | +5.907% | 1.1005869 -> 1.2595498 s | **+14.443%** |

The final compact recount, after consolidating timing and canonical obstacle
infrastructure, is 76 production MATLAB files and 15,140 physical lines versus
76 files and 15,634 lines at `27070ac`: 494 fewer lines with no file-count
growth. The maintained non-example, non-scratch tree is 84 files and 18,609
lines versus 84 files and 19,057 lines: 448 fewer lines. The former 12,000-line
maintained-tree cap is still exceeded.

`+azElInternal` is retained as the neutral shared obstacle layer. Both planners
now use its canonical boundary traversal and signed clearance, while root
utilities also use its option, preparation, and interpolation requirements. No
method calls its sibling package.

One known HS3 limitation remains pre-existing: time spent in a failed embedded
HS3 attempt does not reduce the later global improvement allowance. The new
timing counts that discarded work, but changing the allowance would alter
planner behavior and is outside this behavior-preserving checkpoint. Static
comparison confirmed the same budget behavior in frozen `27070ac`.

Immediately after the A/B run, timer boundaries were tightened and one unused
internal output was removed. That follow-up only reclassified already measured
elapsed work and reduced internal plumbing; it changed no topology, motion,
selection, collision, or validation algorithm. On balance, retain this compact
version and its honest unfavorable evidence, without restoring the prototype
microtimers or attempt ledger.

## Corridor helper consolidation checkpoint — 2026-08-23

The corridor fixed-goal runtime closure now contains 36 MATLAB files and 7,366
physical lines, down from 42 files and 7,713 lines at task baseline
`3280bb0`. Five corridor-local helpers that were executable copies of the
neutral `azElInternal` boundary, obstacle-preparation, interpolation, option,
and logical-normalization helpers were removed. The closed-form straight
jerk-profile implementation was moved into its sole caller,
`buildQuinticSpline`, without changing its equations or returned motion
format.

Repository production, counted as MATLAB outside examples, tests, benchmarks,
and sandbox, is now 70 files and 14,787 physical lines versus 76 files and
15,133 lines at the task baseline. Production plus tests is 78 files and
18,256 lines versus 84 files and 18,602 lines. These counts use PowerShell
`Get-Content` physical rows consistently on both sides; earlier sections retain
their historical counting records rather than being rewritten.

The focused behavior gate passed 52/52 before and 52/52 after consolidation.
It covers shared obstacle infrastructure and corridor static, moving,
fixed/earliest-arrival, wrapping, expected-failure, trajectory, and independent
validation behavior. MATLAB Code Analyzer reported zero messages across all 14
changed MATLAB callers. A static call audit found all 36 runtime files
reachable from `azElPlannerMethods.corridor.plan`, with no reference to a
deleted helper. `git diff --check` also passed.

This checkpoint improves ownership and deployment size; it is not a runtime or
trajectory-quality claim. The complete maintained-example, visible-graphics,
and full 127-test matrices were not rerun. No benchmark row was appended
because no maintained example benchmark was executed.

## Ungrouped corridor and stronger U.S. deformation assessment — 2026-08-23

Retain the corridor collision broad phase. In the maintained 40-moving-circle
case with seed clustering disabled, median planner time improved from 14.5690
to 7.0040 seconds (-51.925%) and collision time improved from 8.2384 to 0.5109
seconds (-93.799%). All three fresh-process pairs preserved the exact selected
route, sampled motion, 62.4777398626363-second duration, minimum clearance,
success, collision state, and kinematic certificate. The default example's
2-degree cluster request creates zero groups, so this is genuinely ungrouped
evidence rather than a reduced-geometry substitute.

The mechanism reads each incoming obstacle's prepared complete-history bounds
and never assumes fixed speed, rigidity, size, route, or example identity.
Near-path obstacles still receive exact time-slice polygon queries and adaptive
certification. The full 132-test suite, all 18 corridor examples, the expected
no-path result, and a visible three-figure smoke passed.

The moving/deforming U.S. example now supplies visibly stronger growth and
rotation. Protected extents grow by 16.805% azimuth and 22.920% elevation, and
the new motion remains independently valid. This is an intentional input
requirement change, so its new route and duration are not presented as a runtime
speedup against the easier former geometry.

The principal remaining runtime weakness in the 40-circle candidate is
corridor construction, roughly 4.6 seconds of a 7.0-second median planner run.
No claim is made that history boxes help when supplied bounds overlap most of
the path, or that absent evolution may be guessed. Production must receive
updated histories or caller-supplied uncertainty bounds and replan fail-safely.

Production grew by 35 MATLAB lines and remains at 14,013 lines, above the
7,000-line target. The smallest paired planner reduction (50.067%) exceeds the
10.5% task-growth allowance for those 35 lines, but does not erase the
pre-existing overall size excess. Raw A/B and final example rows are retained
in `benchmark.csv`.

## Dynamics-timescale HS3 mesh start — 2026-08-26

The largest newly measured arrival improvement is on long multi-leg detours
whose default HS3 segment duration exceeds a complete acceleration/deceleration
cycle derived from the supplied velocity and acceleration limits. Starting
those untimed detours at twice the base mesh reduced the maintained 40-moving-
circle arrival from 61.2011842765 to 58.6189853057 seconds (-4.219%). This also
beats the final `325-full-suite` corridor baseline of 60.3618375887 seconds by
1.7428522830 seconds. The smoothed path shortened from 125.185941203 to
123.380530717 degrees and independent collision and kinematic validation pass.

The structurally different preserved rogue horizon pair improved from
88.2939404925/88.2939359679 seconds to 86.5467293065/86.5467226767 seconds.
Both results independently validate, and their 6.630-microsecond difference
preserves the repaired horizon-invariance behavior. The mechanism uses only
route point count, estimated duration, public collocation count, physical
velocity/acceleration limits, and timed-seed provenance; it does not inspect an
example, obstacle name, horizon, expected route, or stored outcome.

The cost is visible. A fresh same-session 40-circle control took 18.873958
seconds before the change and the exact-current rerun took 24.875451 seconds,
an increase of 6.001493 seconds (+31.798%). A blanket 20-segment start was
rejected: the static U arrival regressed from 22.6308876389 to 22.6623174130
seconds, and the short moving-circle runtime rose to 16.976155 seconds. The
retained dynamics-timescale gate leaves those cases unchanged at
22.6308876389 and 8.64603156476 seconds respectively.

Fresh evidence is 18/18 maintained outcomes (17 independently validated
successes plus the expected validated no-path result), 82/82 automated tests,
zero Code Analyzer findings in the changed planner, a visible three-figure
success, and a two-figure expected-failure diagnostic. HS3 remains exactly
2,000 noncomment production lines. The moving/deforming U.S. example still
takes 49.394024 seconds, with obstacle protection remaining the dominant
whole-pipeline bottleneck; attempted polygon simplification, batching, shape-
property skipping, and interval-union caching did not earn retention.

## Shared helper consolidation checkpoint — 2026-08-23

Both planner methods now use `+azElInternal` as the single owner of option
merging, logical normalization, fixed/moving-goal evaluation, dynamic-obstacle
preparation, shape-at-time interpolation, polynomial evaluation, and
power-to-Bernstein conversion. Ten behavior-equivalent private MATLAB
implementations were removed: three corridor helpers and seven HS3 helpers.
Planner-specific search, motion construction, certification, result
construction, and independent validation remain isolated, and neither method
calls its sibling package.

Against task baseline `a51f6e9`, production MATLAB decreased from 70 files and
14,822 physical lines to 62 files and 14,256 lines: eight fewer files and 566
fewer lines. Production plus tests decreased from 78 files and 18,291 lines to
70 files and 17,725 lines. The focused pre-change baseline passed 33/33. After
consolidation, the complete shared-obstacle, HS3, and corridor unit files
plus the fixed-duration polynomial tests passed 101/101. MATLAB Code Analyzer
reported zero messages across all 20
modified MATLAB files, the stale-reference and cross-method audits found zero
matches, and `git diff --check` passed with line-ending conversion warnings
only.

This is an ownership and deployment-size improvement, not a runtime or
trajectory-quality claim. Planner constants, decisions, candidate order,
certificates, collision policy, and returned formats were not changed. The
maintained examples, visible graphics, and complete repository suite were not
rerun, and no benchmark row was added because no maintained example benchmark
was executed.

## Shared validation and timed-search ownership checkpoint — 2026-08-23

The strongest measured result is a 510-line reduction in non-comment MATLAB
across production and tests while preserving all 132 automated requirements and
both 18-example method matrices. Production owns 340 fewer non-comment lines;
tests own 170 fewer. The corridor and HS3 packages still retain their distinct
planner, graph-construction, solver, range-certificate, and diagnostic policy.

Shared ownership now covers seed-corridor Bernstein inequalities, common
polynomial format/dynamics/history validation, and time-expanded visibility
search. Corridor continues to certify exact stationary-point polynomial extrema
and HS3 continues to use conservative Bernstein bounds through an explicit
callback. Twenty-four common per-method test requirements and seven fixture
builders have one implementation with thin method-visible wrappers.

Fresh evidence is 132/132 tests, zero Code Analyzer messages across 102 intended
MATLAB files, 18/18 corridor examples, and 18/18 HS3 examples. Each matrix has
17 independently validated successes plus the expected validated no-path
failure. A visible success created three figures, and the expected failure
created two hidden diagnostic figures with no selected trajectory.

The largest remaining weakness is HS3 numerical conditioning: maintained runs
still emit extensive near-singular or singular `fmincon` warnings even when the
returned motion passes independent validation. The full-suite wall time was
245.517 seconds versus a 174.860-second pre-change baseline; one run does not
establish causality, so no runtime improvement is claimed. Method-specific
envelope, clustering, graph construction, result formats, and collision
certificates remain separate where their requirements differ.

## Compact corridor cutover — 2026-08-24

The largest current strength is that one 1,023-noncomment-line compact
obstacle-path implementation now owns the corridor method. It supports static,
moving, deforming, timed-hold, and nonzero endpoint-derivative requests; the
small direct analytic quintic remains separate for static straight motion.
Four superseded legacy motion files containing 1,011 noncomment lines at
committed `8111d0f` were removed after a zero-caller audit.

The final evidence is 133/133 automated tests, zero Code Analyzer messages
across 99 intended MATLAB files, 18/18 maintained-example outcomes, and five of
five repeated-turn/hairpin benchmarks. The example matrix contains 17
independently validated successes plus the expected validated no-path result;
every successful arrival met or beat its frozen legacy duration. A distinct
nonzero-velocity/acceleration detour also passed with endpoint errors no larger
than `1.37e-12` in the applicable derivative units. Success and expected-
failure graphics both rendered from returned diagnostics.

The most important limitation is still finite search. Neither bounded topology
generation nor bounded duration exchange supplies completeness or a global
optimality certificate. The moving/deforming U.S. case remains the slowest
recorded example at 21.32539 s wall time, and the results do not establish a
uniform runtime speedup. The next HS3 reduction must therefore compare every
arrival and wall time against this committed compact baseline rather than rely
on aggregate averages or fallback claims.

## Standalone Hermite-Simpson restoration — 2026-08-24

The branch again contains two genuinely separate motion methods. Compact owns
its corridor-constrained quintic implementation. HS3 now owns a standalone
third-order Hermite-Simpson transcription and returns only HS3-generated motion
on success. Static search geometry and request normalization are neutral shared
infrastructure; no compact planner result, warm start, fallback, or merged
acceptance path crosses the HS3 boundary.

The implementation is within its size and runtime gates: 1,602 noncomment HS3
production lines against a 2,000-line cap, and an independently validated
12-hairpin run in 93.339104 seconds total against the 120-second requirement.
All 18 maintained example outcomes and all 137 automated tests passed. The
public 1/5/10/20-turn benchmark also passed independent validation at every
scale.

This is not yet method-quality parity. HS3 beats the frozen compact arrival on
5 turns, 10 turns, and the 12-hairpin case, but trails by 0.006849 seconds on
one turn and by 46.202735 seconds on 20 turns. Static scenes intentionally stop
at the first independently valid topology, and timed seeds use their
input-derived arrival during the local solve. Those bounded choices make the
runtime gate practical but can leave a better topology or timing local optimum
unexplored. HS3 therefore remains a finite, locally optimized planner without a
global optimality or completeness certificate.

## Extreme deforming U.S. scenario — 2026-08-24

The moving/deforming U.S. example now supplies a materially harder obstacle
history: 8%-to-135% growth, interior deformation, a full 180-degree rotation,
disappearance after 240 seconds, and a separate moving starburst sun along the
bottom. Geometry assertions are part of `ExampleValidation`, so a valid path
alone cannot conceal loss of any requested stage. Compact and standalone HS3
both pass independent and scenario validation.

The main unfavorable result is cost: the new compact example took 40.659661 s
wall time and produced a 9.14130766846 s motion, while HS3 took 108.428247 s
and produced a 75 s motion. This scenario therefore strengthens dynamic and
topology-change coverage but does not support a speed or arrival-quality claim
for HS3.

## Diagnosis export and cross-frame polygon stress — 2026-08-24

The persistent sandbox can now export a versioned diagnosis bundle after any
successful or failed run. Focused tests prove that the saved request and result
round-trip, reproduce, preserve an expected endpoint failure, and remain
available from both tabs without serializing graphics state.

A new deterministic benchmark stresses large multi-vertex obstacles that
translate across the frame and rotate by at least 180 degrees. Compact passed
11/12 tested seeds; standalone HS3 passed all seven exercised seeds, including
compact's seed-1011 failure. The failing compact scene has a conservatively
clear boundary witness and is physically feasible under the identical public
requirement.

The isolated weakness is exact workspace feasibility during compact motion
construction. Sampled QP bounds allow a minimum-jerk spline placed on the
workspace boundary to undershoot between samples by as much as 0.000642
degrees. The independent continuous validator correctly rejects it. This
checkpoint exposes and preserves the regression but does not conceal it with a
larger tolerance, expanded workspace, scenario special case, or HS3 fallback.

## Unified obstacle construction ownership — 2026-08-24

Fresh construction, imported canonical normalization, and absolute safety-
margin reconstruction now have one public owner in `makeAzElObstacleData`.
The separate normalizer and inflater files are removed, all maintained callers
use the unified call forms, and the idempotent original-to-protected geometry
invariant remains covered by both planner requirement suites.

This consolidation improves ownership but increases source size: the former
three files contained 533 physical / 339 noncomment lines, while the unified
owner contains 576 / 464. With the combiner call-site change, production grows
by 44 physical / 126 noncomment lines. That unfavorable size cost is retained
explicitly rather than presented as cleanup savings. The compensating evidence
is one implementation boundary, two fewer public files, zero remaining callers
of the removed names, 140/140 tests, and 18/18 maintained outcomes for each
separate planner method.

## Corridor-quintic regression recovery — 2026-08-24

The largest newly measured strength is that compact motion construction now
scales its exact spline representation from route complexity and assigns time
from actual refined geometry. On the first exported Rogue bundle this preserves
the selected motion bit-for-bit while cutting wall time from 77.9230 to 9.2078 s.
On `173vs131`, it removes 37.2076 s of a 41.6076 s arrival regression and also
beats HS3 on smoothed path length and integrated jerk-squared. The mechanism is
not scenario keyed: a structurally distinct 12-hairpin route and all 18
maintained compact examples pass independent validation, and the full suite is
144/144.

The principal remaining weakness is local arrival quality. Compact still
arrives 4.400014 s (3.34%) after standalone HS3 on `173vs131`, so this evidence
does not establish uniform superiority or global optimality. Bounded topology
enumeration and duration exchange remain finite, and existing HS3 moving-target
coverage still produces near-singular `fmincon` warnings even when independent
validation passes.

## Direct long-detour mesh refinement — 2026-08-26

The strongest current HS3 quality result is retained while the redundant
mid-resolution transcription is avoided. Long untimed multi-leg routes whose
base segment time exceeds a complete input-derived acceleration cycle make one
quality refinement directly from 10 to 40 segments. Forty moving circles
retain the 58.6189853057-second arrival that beats the final `325-full-suite`
row by 1.7428522830 seconds; final wall time is 22.727164 seconds rather than
24.875451 seconds for the pushed 20-to-40 flow. Both supplied 180/360-second
rogue horizons independently validate at 86.5467293065/86.5467226767 seconds,
only 6.630 microseconds apart.

This is a bounded local-quality policy, not an optimality or completeness
claim. The neutral-circle control improves arrival from 80.2105179472 to
78.7444420156 seconds but costs 7.139318 rather than 4.140629 seconds wall.
The rejected 30-segment point is faster on the 40-circle case at 19.113692
seconds but arrives 1.5398492530 seconds later than the retained result. The
moving/deforming U.S. example still takes 49.573514 seconds, and known
near-singular `fmincon` warnings remain visible on timed moving-obstacle cases.
Production size does not grow: the HS3 package remains at its exact 2,000-line
cap, with 82/82 tests and all 18 maintained outcomes passing.

## Deforming-outline runtime localization — 2026-08-26

The largest current stage-level weakness remains the moving/deforming U.S.
example. Profiling attributes 20.669104 of 51.619861 seconds to scenario
construction and 26.1176 seconds to planning; exact polygon buffering alone
costs 16.556995 seconds. A classification micro-optimization was rejected
because its 57% isolated gain produced contradictory end-to-end measurements
of 52.626923 seconds profiled and 49.326170 seconds fresh. No production code
or geometry change was retained. Exact translated-shape reuse already covers
the sun, while nonlinear deformation and varying scale prevent exact U.S.
buffer reuse. This bottleneck remains measured and visible rather than being
hidden through reduced geometry or a coarser obstacle history.

## Obstacle-free bounded arrival search — 2026-08-26

The largest current strength is that a single compact HS3 implementation now
uses its existing convex fixed-time feasibility search for obstacle-free
earliest-arrival requests. It starts that bracket at twice the configured mesh
and remains capped by `MaximumCollocationSegmentCount`; nonempty obstacles,
timed seeds, fixed-arrival requests, public options, and result fields are
unchanged. The maintained obstacle-free example improves from 4.60777936881 to
4.5458984375 seconds while wall time improves from 3.882538 to 2.956477
seconds. A structurally different direct request independently validates at
5.70751953125 seconds on 20 segments. The implementation removes the former
segment-count helper while inlining the input-driven bound, so the nine-file
HS3 package remains exactly 2,000 nonblank, noncomment lines.

Against the isolated `67bc087` 325-full-suite baseline, current obstacle-free
arrival remains 0.01476956335 seconds later but wall time is 1.631599 seconds
lower. Wide U remains 0.7981520967 seconds later than its isolated 325 result;
80 segments recovered only 0.0511143 seconds while raising wall time from
13.019699 to 18.815125 seconds, so that broader mesh increase was rejected.
The rogue 180- and 360-second inputs remain independently valid at
86.5467293065 and 86.5467226767 seconds, a 6.630-microsecond difference.

All 18 maintained outcomes pass in fresh serial processes, the warnings-enabled
suite passes 83/83, and Code Analyzer reports zero findings across 84 MATLAB
files. The dominant weakness remains the 49--51-second moving/deforming U.S.
case, split between exact scenario construction and planning. No global
optimality, completeness, or uniform runtime claim is made. Further generic
mesh tuning is not justified by the measured arrival/runtime tradeoff; the
next improvement should target a newly localized invariant rather than extend
this search.

## Shared helpers and HS3 internal subpackages — 2026-08-26

The current worktree removes four duplicated local invariants and organizes
the neutral HS3 implementation into `polynomial`, `constraints`, and `solver`
subpackages. Dependency inspection shows solver code depending on constraints
and polynomial mechanics, constraint code depending on polynomial mechanics,
and no higher-layer dependency from the polynomial package. HS3 remains free
of Az/El domain terminology, and the Az/El adapter uses the qualified neutral
engine functions rather than owning numerical solver calls.

Focused fresh-process evidence covers a moving circle, an opening U, and a
static U. All three planner results and independent example validations pass,
including collision and jerk/kinematic certificates. Their polyline lengths,
smoothed lengths, and durations exactly match the preceding
`de372d5+spatial-route-cleanup-worktree` records. Wall times were 13.708932,
14.261098, and 15.192708 seconds respectively; they are single runs and do not
support a performance claim. The opening-U case retains its known repeated
ill-conditioned `fmincon` warning flood despite producing a validated result.

This is only a focused architecture smoke test. The remaining maintained
examples, expected failure visualization, visible graphics, complete unit
suite, and Code Analyzer have not yet been rerun against this worktree.

## Prepared constraint-layout reuse — 2026-08-27

The largest newly measured runtime strength is that one HS3 solve now prepares
its immutable corridor interval maps, constraint-row offsets, and safeguarded
final-time event locations once. Constraint callbacks continue to reevaluate
the polynomial and every moving-geometry value; only topology that cannot change
during that solve is reused. The neutral HS3 sensitivity builder also skips its
sampled position and velocity maps when the obstacle adapter needs coefficient
Jacobians only.

On the identical Opening-U input, a matched MATLAB profile improved from
41.5774063 to 40.2039387 seconds, or 3.30%. Time attributed to
`evaluateTrajectoryConstraints` fell from 5.45811353 to 4.13240582 seconds
(24.3%), `createConstraintMatrices` fell from 2.31449291 to 1.83661551
seconds (20.6%), and corridor constraint evaluation fell from 1.25470921 to
0.641739103 seconds (48.8%). The selected 10-degree polyline,
10.0912159691-degree smoothed path, and 11.8560791016-second duration are
unchanged. A direct dynamic-geometry test proves prepared and unprepared value
and gradient outputs are exactly equal.

Every maintained example ran serially in a fresh MATLAB process. All 16
expected successes and the expected `noValidatedSeed` result passed independent
validation with unchanged path and arrival metrics. The moving/deforming U.S.
case first measured an unfavorable 200.066640 seconds, then repeated at
182.935442 seconds against the 183.4952231-second baseline with identical
trajectory metrics; the first result is retained here as visible runtime noise,
not hidden. The complete suite passes 120/120. A visible success produced three
figures and six axes, while the expected failure produced two diagnostic figures
and two axes.

The dominant weakness remains nonlinear solver factorization: it consumes
22.1272051 of the 40.2039387 profiled Opening-U seconds. Timed-seed conjugate
gradient was not retried because repository evidence already records a
24.8248-to-28.2319-second Moving-Barrier regression. The retained refactor adds
layout plumbing and compatibility fallbacks, so its source-size and branching
cost are real. It is retained for measured callback reduction, not as an
optimality, completeness, or uniform wall-time claim.

## Rogue sandbox route-quality correction — 2026-08-27

The saved `az_el_sandbox_goal_20260826_192542.mat` case exposed two distinct
failures. A two-seed sandbox budget was completely consumed by the direct and
direct-wait candidates, so the visibility search could not return a spatial
detour. The earliest-arrival nonlinear solve then returned a valid but visibly
looping 112.432758778-degree motion for an 87.6756595117-degree direct seed.

The sandbox now permits three seeds, and timed-candidate reservation leaves one
slot for spatial visibility search whenever the budget can represent all three
roles. This is an input-driven portfolio invariant rather than a waypoint or
scenario-specific preference. If spatial search finds nothing, the timed seed
can still reuse the slot. A validated earliest-arrival motion whose length
exceeds its seed by more than five percent receives one bounded fixed-arrival
shape cleanup. The cleanup is retained only when it independently validates,
keeps the same arrival, and strictly shortens the motion.

On the saved case, the corrected run selected a 90.1324376889-degree
`visibilityGraph` seed. The initial motion was 99.8503182971 degrees and the
accepted cleanup returned 90.3025879374 degrees at 50.9444849555 seconds. This
is 19.683% shorter and 4.9035 seconds earlier than the saved valid loop. It does
not establish global shortest-path or global earliest-arrival optimality.

The focused HS3 and sandbox suites pass 72/72. Structurally different moving
and static maintained examples also passed independent validation:
`exampleMovingCircleNoAzimuthWrap` returned 12.7171175863 degrees in
8.64603261241 seconds, and `exampleObstacleAvoidance` returned
11.4464747617 degrees in 7.57952069664 seconds. Two earlier moving-example
invocations completed planning but their reporting commands referenced invalid
display/validation fields; those harness errors are not counted as test passes.

## Sandbox obstacle-constructor panel — 2026-08-27

The sandbox now places a titled `Add` panel at the far left of each mode. Its
Polygon, Circle, Hand Drawn, and Square controls all feed the existing line or
polygon obstacle representation, so planning, safety-margin application,
motion assignment, validation, and plotting remain centralized. Circle uses a
center and edge click, Square uses one corner and an equal-length opposite
direction, Polygon retains vertex clicks plus right-click completion, and Hand
Drawn uses press-drag-release. Endpoint placement no longer forces Polygon
mode, allowing the user to choose the constructor explicitly.

The focused sandbox suite passes 8/8 and Code Analyzer reports zero findings.
Interactive mouse geometry was covered by the existing callback paths and UI
contract checks; an automated pixel-level layout test was not added.

## Named trajectory entry points — 2026-08-27

The trajectory root now exposes `planTrajHs3` and `planTrajRuckig` as the two
maintained public motion-planning names. Engine packages retain ownership of
their numerical implementations, polynomial utilities, constraints, and
validation. Their existing `solve` functions remain deprecated compatibility
paths for one release, avoiding a breaking namespace migration.

The obstacle planner now routes eligible direct motion through
`planTrajRuckig`. HS3 obstacle-corridor planning still calls engine-owned
optimization and polynomial primitives because it constructs a specialized
constrained problem rather than invoking the generic state-to-state entry.
Architecture tests pass 14/14, Ruckig tests pass 8/8, and a named HS3
fixed-time solve passes independent engine validation. This is an interface
clarification, not a planner-quality or runtime change.

The sandbox Run animation now advances twenty samples per frame with a
0.001-second inter-frame pause by default, replacing five samples and a
0.01-second pause. User overrides remain supported, and hidden figures still
do not sleep for animation.

## Completed multi-seed diagnostic mode — 2026-08-28

`CollectAllSeedCandidates=true` now disables normal arrival-bound and
first-success pruning, retains each attempted candidate's completed timed path,
and automatically enables the labeled seed-path overlay. Labels report seed
source, geometric seed length, final motion length, arrival time, and
independent validation status. Ordinary planning retains its existing pruning,
result selection, and runtime behavior.

The diagnostic mode gives each seed a wall budget of `30 + 15 * obstacleCount`
seconds. A two-second internal reserve leaves time for independent continuous
validation and result assembly. On the two-obstacle gate, the three 60-second
budgets measured 58.297863, 5.806901, and 45.916104 seconds. All three paths
were retained; two validated and one remained explicitly
`optimizerInfeasible`. The selected overall trajectory independently validated.

For `exampleMovingCircleNoAzimuthWrap`, the normal three-seed comparison proves
the direct seed was selected correctly: its 8.64603261240521-second arrival is
0.00000076226416 seconds earlier than the visibility candidate, its motion is
12.7171175863 versus 12.9249981707 degrees, and its integrated squared jerk is
6.84138672765 versus 7.38459816076 deg^2/s^5. A five-seed diagnostic also
exercised the upper visibility route; it arrived at 8.61481291141 seconds with
a 13.5662813685-degree final motion, while the direct candidate arrived at
8.50652835070 seconds. This establishes ranking only among attempted validated
candidates, not global optimality.

## Nonuniform HS3 mesh design gate — 2026-08-28

HS3 now supports explicit normalized segment breakpoints throughout exact
state propagation, integrated-jerk derivatives, affine fixed-time constraints,
obstacle corridors, reconstruction, and independent validation. Omitting the
option preserves the legacy uniform mesh. This general capability is retained
to make segment-placement experiments certificate-safe; it is not evidence
that a particular adaptive policy is beneficial.

The Single-U sweep separated segment count from placement. Uniform 19 segments
reached 21.6190328443 seconds. Ruckig-signature placement with the same count
reached 21.4852160195 seconds, but remained 0.0813386990 seconds slower than
the validated uniform-20 reference of 21.4038773205 seconds. Signature-derived
14 segments reached 21.8372833147 seconds, while jerk-fit coarsening of the
20-segment solution to 14 reached 21.8814530191 seconds. Thus 20 is the
smallest tested mesh to reproduce 21.4038773205 seconds, not a proven minimum.
Neither Ruckig section savings nor low jerk-fit error is an absolute segment-
redundancy certificate.

The adaptive 14-segment policy was removed from the pass-through pipeline. It
improved Single U relative to uniform 14, but increased wall time and did not
improve the selected Two-U or dense-concave result. The retained candidate is
one Ruckig sweep followed by uniform fixed-time repair and uniform 14-segment
free-time polish. Fresh serial static gates independently passed: Single U
21.9798547914 seconds, Two U 21.9090835611 seconds, and dense concave
8.60599186623 seconds.

## Adaptive static hybrid records — 2026-08-28

The largest current strength is an input-driven static hybrid that improves
the three declared record cases in both arrival and warm serial wall time.
Coupled/reversing waypoint chains retain one refined non-stopping Ruckig
profile; scalar chains first compare a zero-sweep timing probe with an actual
valid coarse HS3 motion; two-point routes refine only endpoint intervals when
HS3 acceleration/jerk activity dominates a velocity-limited interior. Scalar
visibility seeds containing a sub-one-percent geometry edge bypass Ruckig and
reuse the valid coarse HS3 polynomial on one 20-segment mesh. No example name,
obstacle shape, expected route, or preferred detour is present in those rules.

| Static case | Previous arrival / wall (s) | Retained arrival / warm wall (s) |
| --- | ---: | ---: |
| Single U | 21.4038773205 / 34.3518164 | 21.2540286325 / 10.4296843 |
| Two opposing U | 21.7309195016 / 67.482064 | 21.7254621235 / 20.2258638 |
| Dense concave | 8.5551003021 / about 13.5 | 8.55509702317 / 9.9209119 |

The Single-U cold process remained variable at 17.7--19.4 seconds, while an
immediately repeated serial run was 10.4296843 seconds and planner-attributed
time was 9.5504081 seconds. This warm/cold distinction is visible and the
10-second result is not presented as a guaranteed fresh-process bound.

Four deterministic random static fields remained collision-free and
kinematically valid. Seed 101 retained a 0.735224-second arrival improvement
for 5.847557 seconds of additional wall work. Seeds 303 and 404 skipped the
expensive repair/polish after their measured timing potential was below 0.5
seconds; seed 303 matched the ordinary 15.7631244134-second result, and seed
404 matched the ordinary 16.0903836179-second result. A failed fixed-time
repair is no longer used merely because it returned samples; only canonical
validation can promote it to the next warm start.

The principal weakness is that the three declared records do not imply every
historical row from unrelated older branches has been beaten. A broader static
audit matched current ordinary quality on alternating slalom and the extreme
U.S. outline, but returned 12.1510445858 and 6.3679977502 seconds versus older
`325-full-suite` records of 10.7822098011 and 6.00693537638 seconds. The hybrid
therefore remains optional (`WaypointWarmStartMode="none"` is still the
default) until that cross-branch gap and cold-start runtime variability are
resolved. The complete focused verification passed 96/96 tests (82 planner,
mesh, and signature tests plus 14 option/Ruckig tests); modified production
files reported zero Code Analyzer findings. The deforming-obstacle planner
test passed but emitted extensive near-singular `fmincon` warnings, which
remain an unfavorable conditioning diagnostic.

## Certified hybrid milestone update — 2026-08-28

The strongest new result is the certified alternating-slalom continuation:
10.7625-second arrival, 16.0904389848-degree motion, and 4.7953407-second
planner wall, all independently valid. The trigger is an axis-aligned,
monotone, near-direct route with repeated lateral reversals; it does not encode
an example name or obstacle shape. A broader near-direct trigger was rejected
because it degraded Two U to 25.92 seconds and accepted an incomplete ladder.

The first runtime milestone generalizes across three predeclared random static
fields. Every selected hybrid result was an actual HS3 attempt that was
optimizer-feasible and independently validated. Relative to ordinary HS3,
hybrid wall deltas were +6.3116919, +4.1164493, and -2.3575010 seconds. Seed
101 improved arrival by 0.7352241443 seconds; seeds 303 and 404 matched ordinary
arrival and path exactly. These cases establish bounded behavior for two- and
three-obstacle random rectangles, not global behavior for arbitrary geometry.

The principal remaining weaknesses are cold-runtime variability and code size.
Single U reproduced the exact 21.2540286325-second solution after cleanup, but
fresh wall was 19.8996114 seconds, so a sub-ten-second cold guarantee has not
been achieved. The worktree adds 2,123 and removes 131 production physical
lines versus `f0d12f8` (1,992 net; 1,795 nonblank, non-comment added lines).
That is a material cost even though the default mode remains `"none"`. The
three protected wins currently rely on distinct mechanisms, so deleting the
pass-through solver, activity mesh, or certified continuation would remove a
measured result rather than neutral experimental scaffolding.

## Final bounded runtime gate — 2026-08-28

The moving/deforming runtime investigation retained no new production shortcut.
Profiling localized most nonlinear time below `fmincon` constraint evaluation,
but an exact time-query trace showed only a 9.02% theoretical cache hit rate.
The failed free-time recovery stage used 0.308 seconds, so neither caching nor
recovery suppression could explain the material wall time. Starting directly
at 20 segments was rejected after escalating to 40 segments and taking 192.03
seconds; this proves the invalid 10-segment result is useful basin information.

A 50-iteration 10-segment pass followed by the normal 20-segment solve remained
independently valid in the complete planner. It used 56.10 seconds of planner
time versus a 61.20-second clean median, but changed the smooth length from
43.0722665 to 43.1321712 degrees. The apparent 8.3% saving is smaller than the
observed run variance and accompanied by a 0.139% path regression, so the cap
was removed. The retained deforming result remains 7.9628679 seconds arrival
and about 43.0723 degrees smooth length; no stronger runtime claim is made.

Post-removal verification passed 93/93 focused tests in 137.44 seconds. A
visible obstacle-free run produced four figures with frame stride 20, and the
expected no-path run returned `noValidatedSeed` with two diagnostic figures.
Fresh Single-U and Two-U runs remained independently valid at 21.2540287320
and 21.7254621235 seconds. The deforming test still emits the known extensive
near-singular `fmincon` warning stream.

The post-consolidation production diff versus `f0d12f8` is 2,132 added and
133 removed physical lines, or 1,999 net; 1,807 added lines are nonblank and
non-comment. The cleanup removed duplicate validation already owned by the
public planner and two derivable internal diagnostic fields. It did not remove
the reachable activity/nonuniform behavior used by the Two-U endpoint
refinement. This clears the literal two-thousand-net objective by one line,
but remains a material implementation-size cost rather than a small change.

After that consolidation, all 17 maintained examples ran serially with finite
jerk limits: 16 independently validated successes and the expected validated
`noValidatedSeed` failure. The moving/deforming result remained exactly
7.96286792066 seconds arrival and 43.0722665096 degrees smooth length. The
Single-U and Two-U results remained 21.2540287320 and 21.7254621235 seconds.
A visible obstacle-free smoke created four figures at frame stride 20, and the
no-path diagnostic smoke created two figures.

The three deterministic random A/B gates also remain valid HS3 answers.
Hybrid overheads versus ordinary HS3 were +9.2573743 seconds for seed 101,
+5.1015524 seconds for seed 303, and -3.0835252 seconds for seed 404. Seed 101
improved arrival by 0.7352241443 seconds; the other two matched ordinary arrival.
The final focused suite passed 93/93 in 115.924289 seconds. Only the duplicate
final suite suppressed the already-recorded `MATLAB:nearlySingularMatrix`
console flood; the preceding unsuppressed 93/93 run remains the diagnostic
evidence for that conditioning weakness.

The final fresh-process Single-U A/B provides one scenario where the retained
hybrid improves all three requested metrics while returning actual validated
HS3 output. Pass-through used 21.2540287320 seconds arrival,
40.5204361036 degrees motion, and 28.7366170 seconds wall. Ordinary HS3 used
22.6308871020 seconds arrival, 41.5367249083 degrees motion, and 40.8732228
seconds wall. The hybrid was therefore 1.3768583700 seconds earlier,
1.0162888047 degrees shorter, and 12.1366058 seconds faster. This is a bounded
Single-U result, not a global-optimality claim.

## Four-File Hybrid Cleanup

The behavior-preserving cleanup after `2958303` reduced the four new hybrid
implementation files from 815 to 758 physical lines and from 629 to 574
nonblank, non-comment lines. The pass-through waypoint solver now shares one
candidate-probe loop across vector and scalar searches, constructs invariant
probe groups outside the sweep, and assembles section histories during the
existing final solve pass. The mesh and classification helpers received only
small representation simplifications; their separate invariant ownership was
retained.

The complete production diff versus `main` is now 2,075 added and 133 removed
physical lines, or 1,942 net. The cleanup therefore removes 57 production
lines from the committed hybrid without shifting them into another file.

Static analysis reported zero messages on all four files. The four focused
test files passed 14/14, and the complete MATLAB test tree passed 164/164 in
84.9595026 aggregate test seconds. All 17 maintained examples then ran
serially with finite jerk limits: 16 independently validated successes and
the expected validated `noValidatedSeed` failure. Single-U, Two-U, and the
moving/deforming case remained exactly 21.2540287320, 21.7254621235, and
7.96286792066 seconds, respectively. Their smooth lengths remained exactly
40.5204361036, 24.4031321261, and 43.0722665096 degrees. A default visible
obstacle-free run created four figures, and the no-path diagnostic run created
two figures with three expanded states. No runtime improvement is claimed from
this structural cleanup because fresh-process wall time remains noisy.
