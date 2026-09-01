# Novel replacement branch assessment

## Detailed plane-reuse trace retirement - 2026-09-01

The tenth `bmtp-cleanup-codex` milestone removes diagnostic-only state around
automatic plane reuse. `PlaneReuseIterationHistory`,
`PlaneReuseControlDifference_deg`, and
`PlaneReuseDurationDifference_s` no longer appear in solver diagnostics. Their
pending control-net and duration snapshots never influenced a solver input,
continuation, convergence decision, retained candidate, or certificate.

The behavior-bearing mechanism remains intact. `PlaneReuseApplied` and
`PlaneReuseCount` still summarize use; the arrival-tolerance condition, stable
tagged-pair requirement, plane-preserving `continue`, collision histories,
retained-best evidence, and convergence diagnostics are unchanged. The focused
contract still requires reuse, convergence, initial collision evidence, and the
exact minimum collision-free retained duration.

Target Exits at `1e-4 deg` clearance and Extreme US Outline both reused once at
iteration 7 and finished at iteration 8. After removing only runtime fields and
the three retired arrays, their complete baseline/candidate results matched
recursively with maximum numeric difference zero. Target Exits retained 24
seconds and 21.9416287312 degrees; Extreme retained 5.81065318159 seconds and
23.3457566443 degrees. Code Analyzer reported zero findings, focused tests
passed 49/49, and the full suite passed 123/123 in 88.5057376 seconds wall time.

All 17 maintained examples retained their prior metrics: 16 independently
validated successes plus the expected validated `noValidatedSeed`. Visible and
failure-figure gates passed. The milestone removes exactly 17 engine lines and
adds no production replacement. The ten-milestone branch is now 213 non-test
MATLAB lines smaller than `5c0a6c9`.

## External BMTP restart retirement - 2026-09-01

The ninth `bmtp-cleanup-codex` milestone removes externally supplied restart
state from the BMTP engine. Neither maintained planner adapter consumed this
state: static and timed planning both called the engine with seven inputs and
two outputs. The core now has one seed-derived initialization path and no
restart validation, alternate initial-best branch, template allocation, or
restart export.

`planTrajBmtp` remains as a one-release compatibility facade. Ordinary
seven-input/two-output use is unchanged. A former eighth input or requested
third output warns once with `planTrajBmtp:DeprecatedRestart`; supplied state
is ignored and the returned restart record is empty. This is an intentional
compatibility loss for direct external callers: the measured test restart cut
one repeated direct solve from 1.0651848 to 0.2434276 seconds. No maintained
planner path received that benefit, and the old warm and cold fixture motions
were exactly identical.

Saved static degree-16, true timed-cell degree-7, and direct cold results
matched recursively with maximum numeric difference zero after removing only
runtime fields. The deprecated direct call also reproduced the old warm
trajectory exactly. Focused engine tests passed 12/12, the broader focused gate
passed 37/37, Code Analyzer reported zero findings, and the full suite passed
123/123 in 88.4909933 seconds wall time. All 17 examples retained their prior
metrics: 16 independently validated successes and the expected validated
`noValidatedSeed`. Visible and failure-figure gates passed.

The engine removes 44 net production lines; the migration facade adds four,
for a net 40-line non-test MATLAB reduction. The nine-milestone branch is now
196 non-test MATLAB lines smaller than `5c0a6c9`. Plane reuse, three-rate travel
refinement, timed cells, conservative grouping, specialized input-driven
constructors, certificates, validation, and failure diagnostics remain because
their maintained result ownership has not been replaced.

## Internal BMTP segmentation ownership - 2026-09-01

The eighth `bmtp-cleanup-codex` milestone removes the last public option used
only to size BMTP's conic construction. `CollocationSegmentCount` is no longer
resolved or echoed. Static warm routes and timed-cell routes retain the former
default effective cap of 20 spans. Legacy input warns once, is ignored, and
cannot retune segmentation. The timed helper also drops its now-unused options
argument.

Saved static-U and true timed-cell BMTP results matched recursively at `1e-9`
after removing only runtime evidence and the retired field. The timed fixture
retained `bmtpTimedCell`, seven optimizer spans, seven timed cells, 35 seconds,
36.6949453597 degrees, full coverage, planes, validation, and certificates.
The existing dense 30-edge engine fixture still resamples to exactly 20 spans.
A legacy value of 2 warned, disappeared, and matched automatic static-U output
recursively.

Focused tests passed 56/56. Sandbox diagnosis took 10.1182416 seconds versus
10.3521119 before the change; route economy took 14.8954734 versus
14.6741929 seconds, ordinary run noise. All 17 maintained examples then ran
serially with exact prior trajectory metrics: 16 validated successes plus the
expected independently checked `noValidatedSeed`. Visible and failure plotting
passed, Code Analyzer reported zero findings, and the full suite passed 123/123
in 88.5229176 seconds wall time.

The required migration shim makes this milestone two net non-test MATLAB lines
larger even though the public interface and two consumer paths are smaller.
Under the established accounting, the eight-milestone branch remains 156 lines
smaller than `5c0a6c9`. Public defaults now contain 14 meaningful fields, and
the audited public surface has no remaining solver-construction-only control.

## Internal trajectory-solver cap ownership - 2026-09-01

The seventh `bmtp-cleanup-codex` milestone removes the obsolete public
`MaximumNlpIterations` field without deleting its active safeguard. BMTP now
owns one fixed trajectory `coneprog` iteration cap of 300, equal to the former
public default. Legacy direct-planner input warns once, is ignored, and is not
returned. The sandbox and examples no longer present private solver tuning as
a request-level choice.

The former unsupported-topology integration fixture was not independent of
this option: `MaximumNlpIterations=1` manufactured a solver failure. With the
real cap it instead returned `goalReached` after about 66 seconds. The revised
fixture uses a physically infeasible eight-second fixed-arrival deadline, so
the default policy genuinely refuses fallback and the explicit policy genuinely
attempts it. The separate Ruckig unit test continues to own the two-segment
limit. Revised policy tests pass 2/2 in 2.9632806 seconds.

The measured tradeoff is unfavorable but bounded in one deliberately low-cap
test: timed BMTP rose from 9.9854775 to 15.0755105 seconds while retaining a
validated smooth result. Sandbox route economy remained effectively unchanged
at 14.6741929 versus 14.7555260 seconds. The combined focused gate passed
40/40; all changed MATLAB files had zero Code Analyzer findings; and the full
suite passed 121/121 in 88.5048981 seconds wall time.

All 17 maintained examples ran serially in fresh MATLAB processes. Sixteen
succeeded with independent validation, collision freedom, and kinematic
compliance; `exampleNoPath` retained its expected independently checked
`noValidatedSeed`. Every trajectory metric matched the preceding committed
milestone. Visible success created two figures without warnings and hidden
failure created one diagnostic figure containing the reason. The manual-data
exporter passed in 5.7923965 seconds. The milestone removes six net non-test
MATLAB lines under the established branch accounting, leaving the branch 158
lines smaller than `5c0a6c9`. The remaining active implementation option is
`CollocationSegmentCount`, which requires its own bounded experiment.

## Automatic plane-reuse ownership - 2026-09-01

The sixth `bmtp-cleanup-codex` milestone keeps BMTP separating-plane reuse and
its diagnostics while removing two public implementation controls:
`EnablePlaneReuse` and `PlaneReuseImprovementTolerance_s`. Reuse is now an
internal continuation invariant: the retained-best duration improvement must
be within `ArrivalTimeTolerance_s`, and the tagged path--region pair set must
be unchanged. Direct legacy fields warn once, are ignored, and cannot disable
or retune the mechanism.

This removes two false user choices and one duplicated tolerance relationship,
but the one-release migration shim costs three net production MATLAB lines.
The six-milestone branch total is therefore 152 production lines smaller than
`5c0a6c9`. The benefit is interface and ownership reduction, not a runtime or
physical-line claim.

Three pre-edit results were saved and compared recursively after removing only
elapsed-time evidence and the two retired option fields. The tight-clearance
Target Exits case remained 24 seconds and 21.9416287311844 degrees with reuse
count 1, 60 plane SOCPs, and 8 trajectory SOCPs. The structurally different
timed alternating-occlusion case remained 20.8695652173913 seconds and
13.571326600194 degrees, with per-seed reuse counts `[0 1 1 0 1]`. Static U,
which did not activate reuse, also remained exact at 20.7124477860115 seconds
and 40.2550285040009 degrees. A legacy `false` plus custom tolerance reproduced
the automatic Target Exits result exactly.

Broad verification passed 120/120 tests in 81.7413358 seconds wall time and
Code Analyzer reported zero findings. All 17 maintained examples ran in fresh
serial processes: 16 independently validated successes plus the expected
validated `noValidatedSeed`. Visible success and hidden failure plotting both
passed. The manual-data exporter ran successfully and now records plane reuse
as `automatic`.

The remaining public-surface candidates are active solver controls, not dead
fields. `MaximumNlpIterations` owns the `coneprog` iteration cap despite its
obsolete name, while `CollocationSegmentCount` bounds static/timed BMTP route
segmentation. Evaluate each independently; do not remove or retune either
without exact-result evidence.

## Dead planner verbosity option removal - 2026-09-01

The fifth `bmtp-cleanup-codex` milestone removes the public planner `Verbose`
field. A complete read audit found that the field was resolved, validated,
echoed, and forwarded but never read by planning, search, motion generation,
validation, or plotting. The live obstacle-construction verbosity controls
remain separate. The sandbox also retains its top-level verbosity checkbox,
which now owns console capture outside the planner instead of injecting dead
planner state. Direct legacy planner input warns once, is ignored, and is not
returned.

This is an interface reduction rather than a physical-line reduction. The
one-release compatibility shim and its ownership plumbing cost eight net
production MATLAB lines, so the branch total changes from 163 to 155 removed lines
across five milestones. That unfavorable line-count delta is explicit; the
benefit is one fewer false planner capability and clearer logging ownership.

Default and legacy-`Verbose` obstacle-free runs matched exactly after removing
only measured runtime fields. All 17 maintained examples then ran in separate
fresh MATLAB processes: 16 successes independently passed collision and
kinematic validation, and `exampleNoPath` retained the expected validated
`noValidatedSeed`. The complete suite passed 118/118 in 81.436094 seconds wall
time, Code Analyzer reported zero findings, the visible success created two
figures without warnings, and the hidden failure created its diagnostic figure.

The remaining option audit found no other unread default. At that milestone,
collocation, solver-iteration, and plane-reuse controls were all still active;
the later automatic plane-reuse milestone internalized only the two reuse
fields under exact-result gates.

## Dormant seed-clustering removal - 2026-09-01

The fourth `bmtp-cleanup-codex` milestone removes optional conservative hull
clustering from topology-seed generation. `SeedClusterDistance_deg` previously
defaulted to zero and no current maintained example, test, benchmark, or
sandbox enabled it. The 85-line `clusterSeedShape.m` helper is deleted and
route candidates always use the unclustered protected swept geometry. A
one-release option shim warns that a supplied legacy distance is deprecated
and ignored. The existing `SearchDiagnostics.Grid.SeedCluster` record remains
with its exact default values and source-region count so default diagnostic
schema and plotting consumers do not change.

The measurable maintainability benefit is a net reduction of 83 production
MATLAB lines: 100 removed and 17 added after the compatibility and diagnostic
cost. The branch has now removed 163 production lines across four committed
milestones while preserving route generation, continuous BMTP, separating-
plane reuse, static and time-varying obstacle handling, time policies, motion
constraints, validation, certificates, failure diagnostics, and the public
result/restart contracts.

The strongest correctness evidence is recursive comparison of every current
maintained example against frozen commit `11582e3`. All 17 results match at
`1e-9` outside the intentionally removed option and runtime fields. This
includes seed ordering, visibility graph counts, coverage flags, the retained
zero-valued cluster diagnostic, route and trajectory histories, validation,
certificates, and termination. Sixteen examples succeeded and independently
validated; the expected no-path example retained its validated
`noValidatedSeed` failure. The extreme outline retained 5.81065318159 seconds
arrival and 23.3457566443 degrees of motion, with 67.5731971 seconds wall time
versus 67.4136091 seconds at baseline.

A structurally different three-region fixture proved the removed behavior was
actually exercised. At distance zero the frozen baseline used 26 nodes and 46
visibility edges. At one degree it formed one conservative group and reduced
the graph to 10 nodes and 16 edges, while both returned the same validated
8.08716891419-degree motion at 6.5 seconds. The candidate legacy replay warned
once, used the unclustered 26-node/46-edge graph, and matched the zero-distance
baseline recursively. Its 4.3206596-second wall time was close to the
4.2952183-second clustered run; this small fixture does not establish a global
runtime ratio for fragmented fields.

Broad verification passed 117/117 tests. A hidden no-path run retained the
termination reason and search counts in its figure title, and a visible
obstacle-free run created two visible figures. MATLAB Code Analyzer reported
zero findings in the three changed production files, documentation no longer
claims the helper exists, benchmark rows record every executed example, and
`git diff --check` passed.

The largest remaining option-surface issue is not simple dead state.
`CollocationSegmentCount` actively caps BMTP warm-route and timed segmentation,
and `MaximumNlpIterations` actively sets the trajectory `coneprog` iteration
limit despite its outdated name. They should be evaluated for internal
ownership or clearer naming in independent bounded changes, not deleted as
unused. The next option audit should trace every remaining default and rank
truly unread fields ahead of active solver controls.

## Wall-clock seed cutoff removal - 2026-09-01

The third `bmtp-cleanup-codex` milestone removes the planner's
machine-load-dependent per-seed wall-clock cutoff. Every admitted seed now
runs to the existing deterministic BMTP iteration and cone-program limits, or
to the existing explicit cancellation boundary. The public
`PerSeedWorkBudgetMultiplier` is recognized for one release, warned as
deprecated and ignored, then stripped before ordinary option resolution. The
private `MaximumSolverTime_s`, `WorkLimitReached`, and
`seedWorkBudgetExhausted` paths are gone. `MaximumSeedCount`, the 35 BMTP
outer-iteration bound, nonlinear and cone-program limits, validation,
diagnostics, and the tested public restart API remain.

The measurable maintainability benefit is a net reduction of 40 production
MATLAB lines: 57 removed and 17 added, including the compatibility shim and
example-boundary forwarding. Together with the first two milestones, the
branch has removed 80 production lines while retaining the continuous BMTP
solver, separating-plane reuse, static and time-varying obstacle support,
arrival policies, motion limits, validation, certificates, failure
diagnostics, and public result contract.

The strongest correctness evidence is recursive comparison against frozen
commit `dd7a674` at `1e-9`. Fixed-arrival alternating occlusion, earliest and
balanced obstacle avoidance, the extreme outline, moving/deforming geometry,
and expected no-path results matched their completed-seed baselines outside
the declared option and diagnostic removal. Alternating occlusion now
deterministically completes seed 5 and selects its 13.5713266002-degree motion
instead of sometimes discarding it and selecting the 13.5986641387-degree
motion; arrival remains 20.8695652174 seconds. Earliest arrival remains
7.57454176632 seconds with 11.4118613877 degrees of motion. Balanced arrival
remains 7.54855735896 seconds, 11.2161345431 degrees of motion, and
18.764691902 degrees of declared composite cost.

Broad verification passed 115/115 tests. All 17 maintained examples ran in
separate serial headless processes: 16 planner/example-validation successes
and the expected validated `noValidatedSeed` result. Every successful motion
passed collision and kinematic checks. A hidden no-path run created one
diagnostic figure titled with `noValidatedSeed`, one seed, one expanded state,
and two rejected transitions. A visible obstacle-free run created two visible
figures. MATLAB Code Analyzer reported zero findings in all four changed
production files, and `git diff --check` passed.

The explicit unfavorable tradeoff is runtime. The extreme-outline default
wall time increased from the prior cutoff run's 42.1929796 seconds to
67.4136091 seconds in the full sweep. A controlled completed-seed comparison
was much closer: 72.2275885 seconds before the edit and 72.80709 seconds after
it, with exact non-runtime results. This is accepted because the user
prioritized a smaller deterministic core over early runtime and because the
old cutoff could discard a better valid result. Runtime ratios remain
case-specific; this milestone does not claim a universal slowdown bound.

The next highest-confidence cleanup candidate is dormant seed-region
clustering. Its default is zero, no maintained example or test enables it,
and a separate bounded experiment could remove approximately 90-100
production lines while requiring exact default-result equality. Plane reuse
itself remains explicitly retained: its completed removal experiment worsened
motion length and approximately doubled runtime.

## Dormant waypoint warm-start option removal - 2026-09-01

The second `bmtp-cleanup-codex` milestone removes the planner-option surface
for an implementation that is not present in the repository. No production
planner or trajectory engine consumed `WaypointWarmStartMode`,
`RequestedWaypointWarmStartMode`, or `IsWaypointWarmStartAvailable`; their
only behavior was validation, probing for the absent `ruckigWarmStart.m`, and
echoing fallback state. Current defaults and results no longer contain those
fields. A one-release migration shim recognizes all three legacy names, emits
one explicit deprecation warning, and strips them before ordinary option
resolution. The example option boundary forwards legacy names to that single
warning/strip owner instead of misclassifying them as unknown example fields.

The measurable maintainability benefit is a net reduction of 16 production
lines in `resolvePlannerOptions.m` and 10 production lines overall after the
six-line example-boundary compatibility cost. The tested public BMTP
eight-input/three-output restart interface is unchanged. Together with the
first milestone, the branch has removed 40 production lines while preserving
the active trajectory engine, route families, plane reuse, validation, and
diagnostics.

The strongest result-retention evidence is two controlled comparisons against
the exact accepted baseline commit `93a28e6`. Four paired obstacle-free runs
with a legacy option replay matched at `1e-9` for all non-runtime result data
outside the three intentionally removed option fields. Warmed medians were
0.0777916 s baseline and 0.0835580 s candidate, a 7.413% difference within the
declared 10% noise allowance. A structurally different fixed-arrival,
alternating-occlusion comparison used the existing
`PerSeedWorkBudgetMultiplier=100` diagnostic to remove wall-clock cutoff
variability. Both sides selected seed 5 and returned exactly
27.950433436 deg polyline, 13.5713266002 deg smoothed motion, and
20.8695652174 s duration; recursive comparison found no non-runtime result
difference, and wall time changed from 24.8525406 s to 25.1338125 s (+1.132%).

Broad verification passed 113/113 tests. All 17 maintained examples ran in
separate serial headless processes: 16 planner/example-validation successes
and the expected validated `noValidatedSeed` result. A hidden no-path run
created one diagnostic figure with the reason and search counts; a visible
obstacle-free run created two visible figures. MATLAB Code Analyzer reported
zero findings in both changed production files, and `git diff --check` passed.

The largest observed weakness is the existing wall-clock per-seed work budget.
One default alternating-occlusion run allowed seed 5 to finish and selected a
27.950433436 deg conservative seed whose final motion was 13.5713266002 deg;
other baseline and candidate runs stopped that seed at
`seedWorkBudgetExhausted` and selected the 13.3416640641 deg direct seed with
13.5986641387 deg final motion. Arrival and validity were identical, and the
alternate final motion was shorter rather than worse, but default selected
seed identity is timing-sensitive. The controlled high-budget comparison
shows this cleanup did not create the difference; deterministic work budgeting
remains a separate core-maintainability candidate.

## Self-contained BMTP SOCP construction - 2026-09-01

The first `bmtp-cleanup-codex` milestone removes the immutable trajectory-SOCP
template threaded through the BMTP alternation and travel-refinement loops.
Each trajectory solve now constructs its own equality rows, derivative rows,
bounds, time/travel cones, and active separating-plane rows in execution order.
The final-sized sparse inequality matrix is allocated once, so the prior base
matrix cache plus later enlargement/copy path is gone. Public planner inputs,
options, result fields, diagnostics, solver arguments, and selection policies
are unchanged. The edit is confined to `trajectory/+bmtpEngine/solve.m` and
reduces it from 1,380 to 1,350 physical lines and from 1,235 to 1,205
nonblank/noncomment lines.

The largest measured strength is exact result retention under a controlled
baseline/candidate comparison. The frozen baseline revision was
`5c0a6c97bf68e9db03ace5281bda2e0f84243a8c`. Four paired runs of
`exampleTargetExitsObstacle`, including one warmup and three timed repetitions
per side, had identical success, independent validation, termination, selected
seed/source, certificate decisions, non-runtime diagnostics, and sampled time,
position, velocity, acceleration, and jerk histories. The maximum sampled
numerical difference was zero against a `1e-9` gate. Both sides returned a
24 s motion, 21.7425467317 deg selected polyline, and 21.9416287312 deg
smoothed path.

The explicit unfavorable tradeoff is runtime. The warmed median increased from
10.7475989 s to 11.7573298 s, or 9.395%, on that repeated-SOCP case. This is
inside the predeclared 25% limit and was retained because eliminating hidden
cache state makes the optimization kernel smaller and self-contained. It is
one measured fixed-arrival case, not a general runtime ratio.

Verification covered the structurally different static degree-16 and timed
degree-7 BMTP paths, moving and deforming obstacles, fixed and earliest arrival,
successful and expected no-path outcomes, and graphics diagnostics. The full
test tree passed 111/111 with zero failed or incomplete tests. All 17 maintained
examples ran in separate serial headless processes: 16 planner/example-
validation successes and the expected validated `noValidatedSeed` result. A
hidden failure plot included the reason and search counts, a visible
obstacle-free run created two visible figures, Code Analyzer reported zero
findings for both baseline and candidate `solve.m`, and `git diff --check`
passed.

The largest remaining weakness is that this is intentionally only the first
cleanup milestone. The repository still contains plane reuse, the three-rate
travel-refinement portfolio, specialized exact-clock/timed-opening/cavity
constructors, and an externally visible restart surface. Several are measured
load-bearing for arrival or path length and cannot be deleted honestly until a
general mechanism reproduces their results. This milestone establishes neither
planner completeness nor global optimality; the next bounded experiment is to
evaluate plane-reuse removal independently.

## HTML bundle replay and velocity-authored obstacle motion - 2026-08-31

The HTML sandbox can now load an
`obstacleAvoidanceSandboxDiagnosis-v2` MAT file in live mode and run its
canonical request through the current planner. Replay reconstructs the initial
state, goal, limits, resolved options, original obstacle keyframes, and safety
margins; it does not reuse the result stored in the bundle. The reproduced
result becomes the current downloadable diagnosis bundle, so replay remains a
complete inspect-run-save workflow rather than a display-only import.

Moving-obstacle authoring now follows an explicit Set Motion interaction. The
selected polygon's arrow is a velocity vector in deg/s: its component values
and Euclidean magnitude are displayed, and its length in planning coordinates
equals that magnitude. Constant, zero-start, trapezoidal, and out-and-back
velocity laws are integrated into the 21 position keyframes supplied to the
planner. In particular, zero-start treats the arrow as final velocity, while
trapezoidal and out-and-back treat it as peak velocity. This replaces the old
and easily misread total-displacement arrow.

The measured strength is end-to-end reproduction without a second planner
interface. A raw MAT upload through `/run-bundle` returned a fresh
`goalReached` result in 1.498398 server seconds; its independent validation
passed. A moving-obstacle unit replay preserved both keyframe times and the
final original polygon slice. The complete test tree passed 111/111 in
123.685910 seconds, and Code Analyzer reported no findings in the changed
MATLAB files.

The largest current limitation is that MAT replay requires the loopback MATLAB
server and is capped at 128 MiB. The in-app browser policy blocked opening the
local `file://` page, so no browser-driven visual claim is made; production
JavaScript syntax, the extracted velocity-profile integration, HTML wiring,
MATLAB replay, and the live HTTP endpoint were verified independently. Two
fresh attempts to rerun the maintained example matrix were blocked before the
first example by MATLAB's already-recorded host startup error, `System Error:
File system inconsistency`. Planner sources were not changed by this UI and
transport work; the immediately preceding 17-example matrix remains the exact
planner baseline.

## Balanced travel-time planning and bounded waypoint fallback - 2026-08-31

The planner now separates hard feasibility from preference. `GoalTimeMode`
defaults to `balancedArrival`, whose explicit
`MinimumTravelSavingsRate_deg_s=1` policy selects a later validated motion only
when it saves more than one degree per second of delay. Jerk remains a hard
validated limit and is not a selection cost. Equal-cost candidates prefer the
earlier arrival, then greater mean normalized peak velocity, acceleration, and
jerk utilization. Every seed summary reports its degree-valued tradeoff cost
and utilization, and search diagnostics state the formula and retain the
secondary conic portfolio's trial rates, durations, lengths, and costs.

The largest measured strength is that one input-driven policy now corrects
three different failure mechanisms without scenario branches. The supplied
static shrimp already had a 175.168391-degree upper-boundary seed, but the
time-only BMTP kernel expanded it to 192.556229 degrees and 56.293 degrees of
elevation. A two-stage solve now first establishes a collision-free homotopy,
then minimizes a convex Bezier control-edge travel bound under retained and
newly discovered separating planes. A bounded three-rate portfolio rejects
dominated local scalarizations. The retained shrimp motion is 175.780063
degrees at 82.389498 s, never exceeds 39.288753 degrees elevation, and passes
independent collision, velocity, acceleration, and jerk validation.

For moving obstacles, balanced timed search retains the shortest ancestry at
the mission horizon but removes terminal goal dwell before motion realization.
That supplies a moving-aware later/shorter candidate while the spatial search
supplies the faster end of the comparison. On the supplied non-ideal case the
planner compares a validated 144 s / 228.491135-degree timed motion (cost
372.491135 degrees) with a validated 119.473594 s / 260.029509-degree detour
(cost 379.503102 degrees) and selects the former. This is a measured incumbent
comparison, not a completeness or global-Pareto-optimality claim.

The fixed-clock excursion now validates and compares its progress-polynomial
and one-sided families by actual travel at the identical physical clock. The
supplied sine case falls from 153.472521 to 146.976783 degrees without changing
its 70.344251 s duration or any constraint. Candidate selection no longer uses
integrated squared jerk.

Ruckig waypoint composition now has a visible two-segment hard limit. A route
with more segments returns `ruckigWaypointSegmentLimitExceeded` before Ruckig
runs. The supplied six-segment hidden-fallback request still succeeds through
velocity-carried BMTP at 104.261457 s and 233.911502 degrees; it is not
misreported as no-path and no interior state is forced to rest. The maintained
alternating-occlusion example was moved from explicit Ruckig to BMTP because
its route genuinely exceeds that public limit.

The main unfavorable tradeoff is runtime and boundedness. The shrimp balanced
solve took 32.54 s versus the saved pre-change planner result's 13.08 s; this
comparison includes different MATLAB sessions and is not a controlled speed
ratio. The three-rate refinement is deliberately bounded, and timed search is
still bounded by its node, layer, cell, and seed caps. A returned solution is
independently valid, but failure does not prove that no continuous trajectory
exists and success does not prove the complete Pareto frontier was found.

Code Analyzer reported zero findings on all changed MATLAB files. The full
suite initially passed 107/108 tests; the sole failure was a certificate fixture
that had unintentionally inherited the new default. After declaring its
intended `earliestArrival` policy, the focused certificate suite passed 3/3,
and the final complete suite passed 108/108 in 111.388 s. All 17 maintained
examples ran in separate headless MATLAB processes: 16 independently validated
successes and the expected `exampleNoPath` failure. A hidden no-path run created
one diagnostic figure, and a visible obstacle-avoidance run created one visible
validated figure.

## Sandbox route-economy coverage - 2026-08-31

Sandbox-scale route tests now measure accumulated two-axis travel and
meaningful lateral velocity reversals for a static circle, an irregular
concave static outline, and the same irregular outline moving across the
direct route. Each case also requires independent collision, velocity,
acceleration, and jerk validation. The static-circle guard compares the
returned motion with the exact tangent-and-arc geometric lower bound; the
irregular cases use direct endpoint distance as a conservative lower bound.

Before clearance-boundary refinement, the centered protected circle returned
a validated 7.333333-second fixed-clock motion of 16.822181 degrees. Refining
the coarse failing/passing amplitude bracket with authoritative continuous
validation retains the same arrival time and reduces travel to 16.700092
degrees. The protected-radius tangent-and-arc lower bound is about 16.638
degrees. The retained motion is therefore within one percent of that geometric
lower bound and contains one lateral reversal. A visibility-route alternative has a
16.636942-degree geometric seed, but its smooth realization is 17.216282
degrees and takes 8.169085 seconds. A waypoint-stop realization preserves the
16.636942-degree geometry but takes 15.675572 seconds and stops at every
interior point. Those alternatives were rejected because they increase time,
joint cycling, or both.

The route-economy checks limit regressions; they do not prove global
minimum-wear motion. The planner still prioritizes earliest validated arrival,
then path length and integrated squared jerk. Mechanical wear also depends on
loads, backlash, lubrication, and controller behavior that are not modeled.
The refined boundary requires additional full validation calls. The centered
circle planning call took 3.123552 seconds in the retained focused run; an
identically instrumented pre-change runtime was not recorded, so no runtime
ratio is claimed.

This file records the authoritative state of `novel-rep` and a concise ledger
of approaches already tried. Superseded benchmark matrices remain in
`benchmark.csv`; verification details remain in `verification.md`. Historical
work is retained here only when it records a mechanism, outcome, or warning
that should influence future planner work.

## Current state: smooth timed multi-waypoint BMTP - 2026-08-31

The planner now has a general smooth path for time-expanded multi-waypoint
seeds. It partitions each moving obstacle history into physical-time cells,
uses the convex hull of protected endpoint and midpoint geometry as a
conservative cell superset, and applies each cell only to overlapping equal-
duration Bezier spans. Static concave geometry remains exactly decomposed and
active for the complete motion. Interior search points guide the warm route;
they are not constrained to zero velocity or acceleration.

The primary frozen baseline at `fe076fe` was a moving circle plus a static
concave U, with two admitted seeds and the default fail policy. It returned
`unsupportedTimedMultiWaypointRoute` after 4.790300 s. With the retained
changes, that exact request returned an independently validated smooth motion
at 14.634958917 s with 39.676450938 deg sampled length; its minimum speed at
the four interior timed-seed points was 3.644660375 deg/s.

A stronger version held the circle in the detour for the first 10 s, which
made both static and swept projections fail. The time-cell solver then selected
the time-expanded seed and returned an independently certified 35 s motion:
45.574198839 deg selected polyline, 45.517670812 deg sampled smooth length,
and 1.469097516 deg/s minimum speed at interior timed-seed points. Its complete
91-pair time-cell certificate reconstructed every static region and moving
cell from the original protected obstacles before verifying the degree-one
Bernstein planes. A structurally different four-span translating-polygon
engine test also verifies that a region is enforced only on its overlapping
time spans.

The contact-linearization correction is also retained. A solved separator for
a visibility seed that merely touches protected geometry may initialize the
next alternating trajectory step even when it does not yet have the required
positive gap. It is never an acceptance certificate: final Bernstein plane
verification and public trajectory validation remain mandatory. This removed
a measured `3.3e-8 deg` numerical-contact dead end without relaxing collision
clearance or any public tolerance.

The largest current weakness is completeness and conservatism. Time cells use
convex supersets, the solver currently maps the search-layer clock to at most
the existing BMTP span budget, and earliest-arrival work is bounded to the
search estimate plus the request horizon. A failure is therefore evidence that
these bounded representations found no validated trajectory, not proof that
no dynamic path exists or that a returned arrival is globally optimal.

Final verification passed Code Analyzer on all six changed MATLAB files and
101/101 tests. All 17 maintained examples ran in separate serial headless
MATLAB processes: 16 independently validated successes and the expected
validated `exampleNoPath` failure. The visible moving-circle example created
two figures and passed. Existing static-U and moving-barrier sentinels retained
20.7124477860115 s and 10.0903015136719 s durations, respectively.

## Current state: stagnation stop trade - 2026-08-31

The earlier statement that the stagnation stop was dead on the default path is
wrong. The stop worked and was the more accurate of the two measured methods
on `exampleUSOutlineExtremeVisibility`.

| setting | arrival (s) | length (deg) | plane solves | wall (s) |
| --- | ---: | ---: | ---: | ---: |
| neither | 5.794009507455 | 23.354756039381 | 776 | 88.39 |
| stagnation stop | 5.794009507455 | 23.354756039381 | 468 | 64.73 |
| plane reuse | 5.810653181589 | 23.345756644341 | 160 | 52.42 |

The stagnation stop reached the exact original arrival and was 27% faster.
Plane reuse was 41% faster and arrived 0.016643674134 s later. Kevin chose
plane reuse, accepted that arrival-time cost, and chose to retain one method
rather than two.

At baseline `9d18840`, counted production lines were 11,618, the full suite
passed 92 of 92 tests, and all 17 reference examples verified. Earlier
stagnation-stop numbers below are historical records and do not supersede this
current-state decision.

## BMTP unchanged-plane reuse, initially gated off - 2026-08-30

`EnablePlaneReuse=false` preserves the existing plane reset and re-derivation
path. The four supplied default sentinels were bit-identical to their required
arrivals and sampled motion lengths: Target Exits
`24 / 22.554006042022394`, Obstacle Avoidance
`7.574541766321258 / 11.411861387735195`, Static U
`20.712447786011488 / 40.255028504000862`, and Two Opposing U
`21.633333333333336 / 24.096812127187516` (seconds / degrees). All passed
independent validation. The clean MATLAB suite passed 96/96, the changed files
were Code Analyzer clean, and `exampleNoPath` retained `noValidatedSeed`.

The enabled gate requires both a retained-best improvement no greater than
`PlaneReuseImprovementTolerance_s` and an unchanged tagged-pair set. It skips
only the reset/re-derivation and performs the next trajectory solve. In the
`1e-4 deg` Target case that repeated SOCP was bit-identical on this machine:
the maximum control-net and duration differences were both zero, then the
existing convergence test fired. Baseline / stagnation-only / reuse-only
respectively used `42 / 18 / 15` outer and trajectory SOCPs and `391 / 141 /
93` plane SOCPs. Their warmed minimum / median walls were
`53.0606007 / 53.5620057`, `23.8239231 / 24.4742484`, and
`19.5354281 / 19.7202448 s`; all retained the `24 s` arrival and
`22.555163889326948 deg` path. Reuse therefore achieved more plane-SOCP and
wall reduction than the existing stagnation stop in the measured wandering
case; they are not redundant there.

At default clearance, Target Exits similarly changed from `17 / 121` outer /
plane SOCPs to `12 / 73`, with a `20.8999410 s` baseline median and
`16.3771825 s` reuse median, with no result movement. Obstacle Avoidance did
not trigger the gate (`7 / 13` outer / plane SOCPs in both modes); Static U and
Two Opposing U did no BMTP conic work in either mode. The read-only Rogue
bundle was also a gate-null at both tested horizons: 180 s stayed at
`22 / 576`, `100.664824112242897 s`, and `221.885353904752918 deg`; 360 s
stayed at `20 / 507`, `100.675947361398343 s`, and
`220.666927423424511 deg`. Its warmed medians changed only within ordinary
wall variance (`25.0848222` to `24.8300924 s` at 180 and `23.0830120` to
`23.4646274 s` at 360). The supplied bundle succeeds at 180 s on this branch;
the measured 180 result agrees with the supplied known-good arrival and length
despite the brief's historical failure description.

The option remains off because this is a measured diagnostic/runtime tradeoff,
not a proof that every future alternating problem has deterministic SOCP
repeats. The retained reset remains load-bearing whenever the incumbent is
still improving or tagged pairs change.

### Reverted, then restored - decision record

This mechanism was reverted once (`438c0be`) and then restored. The revert
applied a kill criterion worded as "Regime C regresses in any way", against a
warmed 360 s median moving from 23.0830120 to 23.4646274 s. That criterion was
too strict for wall time and the rejection was wrong:

- The gate does not trigger at all on the Rogue bundle. Counts, arrival,
  length, and validation were identical at both horizons.
- The only added work on that path is one small logical-array copy and one
  `isequal` per outer iteration - microseconds across 20 iterations, not the
  0.38 s the median moved.
- The two horizons moved in opposite directions: 180 s improved from
  25.0848222 to 24.8300924 s while 360 s worsened. Opposite-signed movement on
  a gate-inert path is variance, not effect. The section above had already
  described it as ordinary wall variance.

Wall-clock variance on this machine has exceeded 30% between sessions, larger
than the difference that triggered the rejection. Behavioural invariance, not
wall time, is the criterion that should gate a change on a path the option
never touches.


## Historical: BMTP retained-best stagnation stop, off by default - 2026-08-30

The optional retained-best stop is inert at its default. Four supplied
sentinels kept their recorded arrivals and smoothed lengths: Target Exits
`24 / 22.554006042022394`, Obstacle Avoidance
`7.574541766321258 / 11.411861387735195`, Static U
`20.712447786011488 / 40.255028504000862`, and Two Opposing U
`21.633333333333336 / 24.096812127187516` (seconds / degrees). All four
passed independent validation.

With `EnableStagnationStop=true`, `StagnationIterationLimit=5`, and the
resolved arrival tolerance, the diagnostic `1e-4 deg` Target case reduced
from 42 outer iterations and 433 conic calls to 18 and 159 without moving its
returned `24 s` arrival or `22.555163889326948 deg` smoothed path. The default
Target case also triggered, reducing 17 / 138 to 15 / 120 while retaining its
exact returned result. The other three production sentinels executed no BMTP
conic calls, so the option was a mechanical null there.

After one discarded warm-up, the `1e-4 deg` raw off walls were
`57.1787115 / 56.4846390 / 55.8231577 s`; enabled walls were
`24.9188578 / 26.2214737 / 25.3317567 s`. Thus the observed min / median
changed from `55.8231577 / 56.4846390 s` to `24.9188578 / 25.3317567 s`.
This is a diagnostic-clearance benefit, not a claim about every production
scene or a reason to enable the option by default.

The adverse horizon sentinel is also a null: at 180 s, both modes returned
the independently validated `100.664824112242897 s` motion with 22 / 598
outer / conic work and no stagnation trigger. The 360 s control stayed at
`100.675947361398343 s`, also with no trigger. This does not establish a
production-wide performance win. The documented default remains off; enable
only as an explicit diagnostic/runtime tradeoff because the retained
best-before-stop can be worse than a later oscillating iterate even when the
returned example result does not move.

## BMTP immutable SOCP cache retained - 2026-08-30

The measured reconstruction bottleneck was reduced without moving an answer.
The maintained Target Exits warm median changed from `15.7403272` to
`14.1946827 s`, and its `1e-4 deg` diagnostic changed from `41.7338001` to
`35.3340565 s`. Corresponding `solveTrajectorySocp` profiler self time fell
from `2.882206656` to `0.394152606 s` at default clearance and from
`7.664537344` to `0.448020402 s` at `1e-4 deg`.

The cache is deliberately narrow: it reuses only immutable per-seed
trajectory-SOCP structure while rebuilding plane rows and horizon bounds in
their original order. A temporary legacy-versus-cache oracle found exact
trajectory `coneprog` arguments across all 17 trajectory calls in the
138-conic-call default Target sequence, all 42 trajectory calls in the
433-call slow sequence, and a horizon expansion. All 17 maintained examples
had exactly zero arrival, selected-polyline, and smoothed-path movement against
the archived `747f46c` baseline; aggregate solver structure was also exact.
Focused tests passed 31/31 and the staged gate passed 94/94.

The original instability remains visible. At `1e-4 deg`, seed 2 still
oscillates to the 35-iteration cap; the cache reduces reconstruction cost but
does not alter that stopping path. Maintained Straight Target remains a
zero-BMTP Ruckig control. Its explicit BMTP diagnostic is wall-budgeted, so
outer and conic call counts can vary with wall timing even when its answer is
unchanged.

The branch also remains oversized. Counted production is now 12,015 lines,
40 above the 11,975 baseline and 4,515 above the literal 7,500 target.
`solve.m` is 1,065 physical and 936 noncomment lines, above its 900-line
target. The runtime and exactness evidence supports retaining the cache, but
it is not a size-compliance result and does not resolve the slow-clearance
alternation mechanism.

## BMTP conic runtime localization - 2026-08-30

Fresh warmed, repeated profiling corrects two runtime attributions. Maintained
Straight Target is a Ruckig waypoint case: its three-run wall was
`5.2936647 s` minimum and `5.4580266 s` median with zero BMTP and zero
`coneprog` calls. Its 5.5--6 second runtime must not be used as evidence about
the conic solver. An explicitly labeled BMTP override measured `20.9824146 s`
minimum and `21.2219000 s` median.

Maintained Target Exits at the default `1e-7 deg` clearance measured
`17.6197381 s` minimum and `17.6795313 s` median. The otherwise identical
`1e-4 deg` diagnostic measured `48.2423766 s` minimum and `49.1185155 s`
median. The slow regime was reproduced in every recorded repetition.

The adverse mechanism is now measured. Both modes reached the seed-2 first
collision-free iterate at outer iteration 2 and used no horizon expansion or
caller restart. Default seed 2 converged in 5 iterations; `1e-4` stayed
collision-free but its objective oscillated until the 35-iteration cap. The
total therefore changed from 17 outer iterations and 138 conic calls to 42 and
433. All Target Exits calls exited `+1`. Individual seed-2 trajectory-call
medians changed only from `0.6687750` to `0.7168136 s`; repeated calls, not a
single pathological solve, dominate the near-threefold wall swing.

MATLAB profiler self time identifies immutable trajectory-SOCP reconstruction
as the largest engine-owned cost outside `coneprog`: `2.882206656 s` over 17
calls at default clearance and `7.664537344 s` over 42 calls at `1e-4`.
The next bounded experiment may reuse only those immutable per-seed matrices,
bounds, and cones. Plane-dependent rows and horizon limits must remain fresh;
all solver tolerances, acceptance rules, and outputs remain frozen. A movement
over `1e-9` in any maintained arrival or path length is an immediate revert.

## Ruckig-to-BMTP warm-start experiment stopped at step 2 — 2026-08-30

The approach failed its predeclared collision-survival gate and is not wired
into the planner. Across five maintained static cases, eight Ruckig route
motions first passed the unchanged public endpoint, dynamics, and collision
validation. All eight converted successfully to the exact degree-16 BMTP span
count, but only one converted control net passed the complete degree-one
Bernstein plane check: `1/8`, or 12.5%. The declared kill threshold was fewer
than one-third of at least six validated conversions.

| Case | Converted seeds | Maximum conversion error (deg) | Certified |
| --- | --- | ---: | ---: |
| `exampleObstacleAvoidance` | 2, 3 | 1.74313715049989e-5 each | 0/2 |
| `exampleStaticUShapedObstacle` | 2, 3 | 7.85168079719939e-5 each | 0/2 |
| `exampleTargetExitsObstacle` | 2, 3 | 1.34309047285578e-5; 5.68171630073286e-5 | 1/2 |
| `exampleAlternatingSlalom` | 2 | 9.50061068454221e-6 | 0/1 |
| `exampleDenseConcaveObstacle` | 2 | 9.59122759523563e-6 | 0/1 |

The census used an extended earliest-arrival horizon only for the intermediate
Ruckig source, then independently validated that complete source motion before
conversion. Direct seeds that failed collision validation were excluded from
the denominator. A preliminary constant-plane-only result of 0/8 was discarded
because it did not implement the specified degree-one validator; the 1/8 result
above uses the same conic plane form and direct Bernstein-product inequalities
as BMTP.

No planner option or caller was added, so no arrival, route, or cold BMTP
behavior changed. In accordance with the kill criterion, outer iterations to
first feasibility and repeated warm/cold wall times were not measured. Step 2
adds 323 counted production lines, taking the experimental total to 11,975;
this dead-end code has no measured performance allowance.

## Ruckig-to-BMTP warm-start experiment, step 1 — 2026-08-30

The standalone equal-duration converter is implemented but remains unreachable
from the planner. It fits each requested Bernstein span at Chebyshev-Lobatto
times and reports independently sampled position error; it does not claim that
the converted curve is collision-free or suitable for BMTP yet.

On one exact six-phase Ruckig rest-to-rest profile, degree 7 with 64 uniform
spans reproduced spans containing no interior jerk switch to
`4.96506830649455e-15 deg`; the maximum over all 64 spans was
`4.97014121608466e-9 deg`. With two uniform spans, each containing two
interior switches, the measured maximum was `0.000534264339868523 deg` at
degree 7 and `3.52475369970282e-5 deg` at degree 16. These are sampled errors
for one fixture, not general bounds.

The final step-1 edit passed the fast sentinel gate, the two converter tests,
and all nine architecture-boundary tests; Code Analyzer reported no findings
in the three changed MATLAB files. The production audit moved from the exact
branch baseline of 11,524 to 11,652 nonblank, noncomment lines: the unwired
converter adds 128 counted lines. Retention therefore remains conditional on
the later collision-survival and first-feasibility iteration gates.

## Static earliest-arrival horizon monotonicity — 2026-08-30

The frozen `180bad360good` request disproved the suspected seed-admission
failure. At both 180 s and 360 s the search generated and admitted the same two
seeds: a 153.358411534181 deg direct seed estimated at 76.6792057670906 s and a
281.401707597662 deg visibility seed estimated at 140.700853798831 s. Before
the correction, the visibility seed ended as `noOptimizedFeasibleIterate` at
180 s but reached 100.675947361398 s at 360 s.

The earliest divergence was inside the static BMTP biconvex solve. Six
horizon-bounded trajectory SOCPs remained colliding at durations through
179.203882993999 s, and the seventh became infeasible. The 360 s solve used the
same plane sequence to find a temporary 191.541694821082 s collision-free
iterate, then descended to the roughly 100.676 s validated motion. Thus the
request horizon incorrectly bounded a feasibility iterate even though final
horizon enforcement already existed in BMTP and public validation.

The retained correction activates only when the horizon-bounded SOCP reports
infeasibility before any collision-free iterate. It doubles the intermediate
horizon, capped by the finite 454.754593605252 s kinematic duration of the
seed-warm control net, and restores the request horizon immediately after the
first collision-free iterate. Work remains bounded by the existing 35 outer
iterations, per-SOCP iteration cap, and any active per-seed solver-time budget.
No seed gate, public validator, tolerance, obstacle geometry, or scenario rule
changed.

In the final single-process gate, the 180 s request succeeded with independent
validation at 100.664824112243 s in 19.2650543 s wall. The unchanged 360 s
request succeeded at 100.675947361398 s in 16.2218415 s wall. All 17 maintained
examples passed serial headless validation; `exampleNoPath` remained the
expected `noValidatedSeed` failure; visible success and hidden failure plots
were created. Code Analyzer reported zero findings, the complete suite passed
84/84 in 22.2649327 s, and production remained exactly 11,524 counted lines.

## Dynamic-scene findings — 2026-08-30

These supersede two claims in commit `4138f26`'s message, which were wrong.

### Correction to `4138f26`

That message states the rogue bundle "still takes about 177.8 s wall" and that
edge-query batching "is not attempted here". Both are false. The 177.8 s figure
was a bad measurement taken under process contention. Two independent
measurements of the same commit give **12.9 s and 14.5 s**, and the committed
reachability-frontier search already groups edge queries by layer pair:
`edgeIsClear` calls fell from 23,040 to 82, and `queryObstacleOccupancyAtTime`
from 23,093 to 222. The commit is pushed, so the record is corrected here
rather than by rewriting history.

### Sandbox bundle `az_el_sandbox_goal_20260829_212652`

Two moving obstacles, 180 s horizon, about 203 degrees of azimuth travel at
2 deg/s. Before `4138f26` this request never returned: cancelled at 180 s,
150 s, and 120 s. It now succeeds in about 12.9 s with independent validation,
arrival 107.632292801 s and length 227.751816227 deg. `HS3-planner` solves the
same request at 155.205670334 s, so this branch arrives 47.573378 s earlier.

The failure had two causes. The time-expanded search built the complete edge
list for every free node at every layer with no bound. Separately, with dynamic
obstacles `supportsStaticHorizon` is false, so every seed routed to
`createTimedSeedCandidate`, which accepts only `directWait` seeds and returned
`unsupportedTimedTopology` in about one millisecond — discarding the entire
topology search. A `noValidatedSeed` result with that per-seed signature is a
false negative, not evidence of infeasibility.

### The orthogonal-cavity path is load-bearing

A removal experiment deleted `createOrthogonalCavityMotion`,
`certifyOrthogonalCavityLowerBound`, `evaluateArrivalCertificatePortfolio`, and
their planner wiring, then measured the sentinels.
`exampleStaticUShapedObstacle` regressed from 20.7124477849715 s to
20.7814828183771 s, so the deletion was reverted in full. Those roughly 880
lines earn their keep and are not size-reduction candidates.

That experiment also invalidated an attribution method worth recording. Reading
the winning construction from `result.Seeds(SelectedSeedIndex).Source`
misreports cavity wins: when the cavity portfolio wins, `planCorridorQuintic`
passes the original topology seed to `finishFastPath`, so the source still
reads `visibilityGraph`. A census built that way reported zero cavity wins for
code that measurably changes the result. Attribute constructions from
`SearchDiagnostics`, not from the seed source.

### Roundoff reserve consistency correction

Unifying the plane-certificate `roundoffReserve_deg` formulas moved
`exampleStaticUShapedObstacle` from 20.7124477849715 s to
20.7124477860115 s, a measured +1.04e-9 s change. The constructor's
`eps(coordinateScale_deg)` form was the outlier; the shared helper now uses the
more conservative validator-owned `eps * coordinateScale_deg` formula so the
constructor and authoritative certificate check reason in the same reserve.

### Closed moving-barrier arrival gap

At `d0f00e1+worktree`, `exampleMovingBarrierWait` arrives at
10.0903015136719 s on the unchanged 10 deg path, improving both the prior
10.5 s branch result and `HS3-planner`'s recorded 10.2314453125 s result. The
accepted `directWait` seed now validates a zero-wait lower trial and bisects
the measured infeasible/feasible bracket through the unchanged public
validator. Sixteen deterministic trials refined the wait from 3 s to
2.59030151367188 s; the final measured infeasible lower wait was
2.5902099609375 s. This is a bounded refinement of one validated direct-wait
construction, not a request-wide minimum-arrival proof.

## Current verdict — 2026-08-29

The branch is a working research milestone, not yet the requested beta. It has
one public planner, separated BMTP and Ruckig trajectory engines, stable failure
diagnostics, and independent continuous validation. The last complete matrix
had 16 validated successes and one expected validated failure, but the branch
still misses the combined size, runtime, arrival, and path-record gates. That
matrix does not prove general completeness or global optimality.

- Current integration and cooperative-cancellation suite: 84/84 tests passed
  after the reachability-frontier timed-search port. The rogue bundle now
  terminates as `noValidatedSeed` in 149.484 seconds under its 180-second poll.
- The sandbox now has a Stop action that remains enabled during synchronous
  planning, polls the time-expanded and homology searches plus planner-stage
  boundaries, restores idle state, and enables a replayable pre-run export.
  Cancellation cannot preempt MATLAB inside one atomic solver or vectorized
  geometry call; it takes effect at the next safe checkpoint.
- Production size audit rule: 11,524 nonblank, noncomment lines across 72
  files at HEAD. That measured size is now the ceiling, replacing the
  earlier 4,999 target. The audit counts only `+obstacleAvoidance` and
  `trajectory`; `tests/`, `examples/`, `benchmarks/`, and `sandbox/` are
  outside the counted roots, so adding coverage costs nothing against it.
- Strongest result: Two opposing U reaches the exact `649/30 s` physical
  arrival floor with a 24.0968121271875 deg path and 3.0005152 s full wall.
- Straight Target now explicitly selects exact Ruckig waypoint composition. It
  retains the 20.8695652173913 s clock and passes every public check with a
  20.7720160748 deg path, 5.8749177 s planner time, and 10.1040635 s wall.
  Target Exits remains at the `944a738` BMTP result pending a fresh Ruckig gate.
- In that matrix, Obstacle Avoidance is about 46.57 ms late. Extreme is
  1.179 ms late, inside the user's 2.07 ms allowance but still above the
  literal historical record.
- Straight Target and same-input Target Exits remain longer than their
  historical records; the restored Ruckig route stops at its intermediate
  vertices and is not a claim of locally time-optimal waypoint motion.

## Current invariant boundary

- `obstacleAvoidance.planTrajectory` remains the sole public fixed-goal
  planner; obstacle construction, planning, validation, and frozen plotting
  remain separate.
- The public validator independently owns endpoint, time, workspace,
  derivative, collision, safety-margin, and certificate acceptance.
- Expected no-path and bounded-work outcomes retain stable results and search
  diagnostics. Obstacle grouping remains admissible only with reconstructed
  exact-region coverage.
- Every counted production file has a production or contract-test caller. The
  sub-5,000 target therefore needs a behavior-preserving algorithm cutover,
  not dead-file deletion. No transitive compact implementation has yet proved
  current diagnostics, continuous validation, and plane-certificate parity.

## Strongest current evidence

The Two-U request requires 20 degrees of elevation travel from rest to rest
under `1 deg/s`, `0.75 deg/s^2`, and `2.5 deg/s^3`. The independent scalar
switching lower bound is exactly `649/30 s`. A retained degree-15 progress
polynomial attains it and passes every public continuous check with
0.00618966852407 deg protected clearance. This proves globally minimum arrival
for that request, not globally minimum path length.

## Current blockers and next bounded gate

1. Do not grow past the 11,524-line ceiling. Reduction is welcome but is no
   longer a release gate: the one measured attempt, deleting the
   orthogonal-cavity path, regressed `exampleStaticUShapedObstacle` and was
   reverted, so remaining size is not obviously recoverable without losing
   capability. `trajectory/+ruckigEngine` (2,083 counted lines) is shared
   with other projects and `+obstacleAvoidance/+plotting` (425) is frozen,
   leaving about 7,138 counted lines in scope for any future reduction.
2. Recover pass-through path quality without relabeling the local state-to-state
   Ruckig engine as a waypoint-optimal solver.
3. Close the Obstacle Avoidance arrival gap and both path-record gaps without
   scenario branches or relaxed tolerances.
4. Reduce Extreme planner and scenario-construction wall while preserving
   exact protected geometry and the accepted arrival tolerance.
5. Rerun all maintained examples sequentially before another release claim.

The current fixed-arrival gate retains 20.8695652173913 s but still must reduce
Straight Target from 20.7720160748 deg and 10.1040635 s wall to the comparable
13.678271907957 deg and 2.0964864 s records. Target Exits must retain 24 s,
reach at most 20.6764423274 deg, and beat the 5.167399 s same-input wall record.
The older 20.2803317257 deg and 1.9774286 s rows used a different randomized
target endpoint and are not comparable to the maintained deterministic fixture.

Headless profiling at `4ed7f46` localized the Straight Target loss to motion,
not topology: BMTP spent 70.6255 of 75.4950 planner seconds and
selected a longer seed even though a shorter valid seed was available. Its
output uses 48 degree-16 certified spans, while the same-input historical path
record used ten quintic spans. The Target Exits profile spent 22.1507 of
31.3204 planner seconds in motion. Comparisons must include like-for-like
planner work and pass the unchanged public validator.

## Experiment ledger

### 2026-08-21 — early low-dimensional spline replacement

- **Bounded quintic B-spline:** one-, two-, and five-turn cases constructed
  faster than HS3 but arrived 14–35% later. Ten turns remained colliding after
  mean, worst-clearance, and per-obstacle objectives; 20 turns was not run.
- **Fixed-stop septic Bezier:** interpolated vertices but required 28–84 s
  motion and violated the maintained polynomial format; removed.
- **Interior-route interpolation:** reduced 10-turn wall but worsened
  clearance because route reduction discarded topology; removed.
- **Scalar option sweeps:** timing reserve retained route detail but took
  140.74 s with only 0.000145 deg clearance. Duration weight, collision
  penalty, step size, and offset-bound changes did not repair the mechanism.
- **Feasibility-first hard corridor and worst-clearance ranking:** both failed
  focused gates and were removed.
- **Topology-preserving batching/reduction:** produced valid 10-turn splines
  in 5.22–8.45 s but with less than 0.001 deg clearance.
- **Affine corridor prototype:** reached certified 0.02 deg clearance on a
  10-turn route and a 12-wall maze. The 20-turn motion exceeded its horizon,
  so no production replacement was established.
### 2026-08-22 — corridor-only replacement development

- **Corridor-only cutover:** removed dormant HS3/NLP paths and established an
  input-driven visibility/corridor planner. It passed maintained cases but did
  not meet global arrival or size requirements.
- **Span-demand controller:** replaced 180 timing trials with bounded
  per-span feedback. It was faster on Single U with a measured arrival
  penalty; later superseded.
- **Batched affine corridor work:** reduced repeated timing and constraint
  construction. Broader timing variants regressed records and were removed.
- **Dynamic seed-slot reservation:** recovered one moving-circle field while
  preserving deterministic order; broader timed-route replacement failed.
- **Shallow collision-residual feedback:** recovered narrow penetrations.
  Applying it to deeper residuals regressed Extreme to 8.395298096 s, so the
  broad form was rejected.
- **Retry-exhausted boundary support:** workspace-corner nodes recovered the
  fixed moving-circle sweep. Earlier activation regressed a maintained result.

### 2026-08-23 to 2026-08-25 — broad phase and legacy cutovers

- **Ungrouped collision broad phase:** materially accelerated 40 circles with
  exact trajectory parity.
- **Legacy cutovers:** **Combined method suite**, **Compact corridor cutover**,
  **Standalone Hermite-Simpson sequence**, and **HS3-only production cutover**
  were exercised. A cross-frame stress case exposed a compact-corridor seed
  collision; standalone HS3 had weak conditioning, runtime, and record
  quality; the HS3-only cutover removed 2,525 lines but kept those weaknesses.
- **Legacy recovery:** **Corridor regression recovery**, **Compact C3 duration
  controller**, and **Exact derivative retimer** repaired dense-route or
  selected U cases, but not every maintained arrival record.

### 2026-08-26 — quality, timing, and architecture experiments

- **Mesh and refinement family:** **Dynamics-timescale mesh start**, **Direct
  long-detour refinement**, **Severe-static fixed-time quality solve**,
  **Derivative-slack continuation**, and **Dynamic spatial quality** improved
  selected cases. Broader static, all-seed, or continuation use regressed
  arrival, runtime, or quality.
- **Route, ranking, and retiming family:** **Obstacle-free bounded arrival**,
  **Same-homology shortcutting**, **Certified direct collinearity**,
  **Fixed-arrival geometric lower bound**, **Fixed-arrival length-first
  ranking**, **Shortest-route-first ordering**, **Time-expanded retiming**, and
  **Timed-arrival repair** were tried. Benefits were local; one final matrix
  was slower and one timed case remained late.
- **Deforming-outline localization:** polygon buffering and nonlinear
  constraints were dominant; a classification micro-optimization was removed.
- **Two-product and flat architectures:** both package layouts were tried and
  superseded; they did not change the algorithmic limits.

### 2026-08-27 — performance and invariant consolidation

- **Direct motion and moving geometry:** **Exact third-order switching and
  scalar progress**, **Prepared dynamic boundary queries**,
  **Convex direct-route arrival search**, and **Monotonic direct-line progress**
  improved eligible direct cases, but one moving-target timing observation was
  adverse.
- **Invariant batching and caching:** **Batched occupancy and deferred
  allocation**, **Static geometry and polynomial-map caches**, and
  **Prepared constraint-layout reuse** produced measured local wins. A timed
  moving-barrier CG variant regressed.
- **Bounded sandbox planning** made one moving result 26.93% later.
  **Unified seed equivalence** reduced ownership but missed its runtime gate.

### 2026-08-28 — hybrid replacement experiments

- **Direct Ruckig before topology**, **Optional pass-through warm start**, and
  **Nonuniform mesh** each gave a local benefit or retained engine capability
  but no general runtime or selection win; their broad policies were removed.
- **Adaptive static hybrid** and **Alternating-slalom hybrid** improved selected
  cases, including a 10.7625 s validated slalom, but variability and a broader
  near-direct trigger prevented a general replacement.
- **Moving/deforming runtime gate:** query reuse was 9.02%; skipping the coarse
  basin took 192.03 s, and a short iteration cap regressed path quality.

### 2026-08-29 — BMTP replacement branch

- **Restored exact Ruckig engine and explicit route method:** restored the
  independent state-to-state switching engine and facade, added the general
  `TrajectoryMethod="ruckigWaypoint"` route adapter, and wired Straight Target
  through it without a BMTP fallback. The engine tests passed 10/10, the full
  suite passed 78/78, and a structurally different static-box detour passed
  earliest and fixed arrival. Straight Target passed at the exact fixed clock,
  but its stop-at-waypoint path and wall remain above the historical records.
- **Exact waypoint fallback:** composed exact jerk-limited stops along a failed
  spatial seed and recovered a failed multi-edge request. Swept-envelope HS3
  was rejected.
- **BMTP cutover:** replaced legacy production with a separated Bezier/plane
  engine, exact direct/event kernels, and complete region certificates.
- **Exact-clock progress polynomial:** attained the Two-U global arrival lower
  bound with sub-10-second wall and truthful route provenance.
- **Full fixed-corridor Bezier QP:** rejected. Static U produced
  30.047333018 s and about 45.6554866 deg versus retained 20.712447785 s and
  40.2550285 deg. Missing certificate ownership was diagnosed, but quality
  already failed, so experiment code and artifacts were removed.
- **Fixed-clock null-space family:** a 109-variable form beat the dense path
  target and validated but took 35.14 s. A seven-coefficient form retained
  quality, then passed Straight Target with a fully plane-certified
  13.582258304 deg path at the exact 20.869565217 s clock in 8.03538 helper
  seconds. Target Exits remained valid but took 22.1424908 helper seconds and
  produced 21.811076622 deg versus the comparable 20.676442327 deg record.
  Shared preparation and direct validation were excluded from that timer, so
  no general or end-to-end speed claim was retained.
- **Rest-state safe-interval search:** rejected as a universal replacement.
  On Alternating Slalom, its 8-vertex visibility route was only
  16.0193197983 deg, but exact rest-to-rest jerk primitives summed to
  24.6732769008 s versus the retained 10.5 s fly-through motion. Stopping at
  every graph vertex cannot meet the maintained arrival records.
- **Open-quintic route smoother:** rejected at its first frozen gate. It made
  a fully validated Straight Target trajectory in 3.0661778 s candidate scope
  at the exact 20.869565217 s clock, but its 13.7395585901 deg path missed the
  13.678271908 deg record. Target Exits was not run after that gate failed.
- **Historical eight-span C3 sampled-barrier QP:** rejected at its first
  frozen gate. Straight Target remained fully valid at the exact
  20.869565217 s clock, but its 13.7165279811 deg path missed the
  13.678271908 deg record and projected planner scope was 5.0271313 s versus
  the 2.0964864 s wall record. Target Exits was not run, and all experiment
  code and artifacts were removed. A later same-input reconstruction confirmed
  that length-first selection cannot improve it because seed 2 is already its
  shortest validated motion. Increasing the compact basis through 9, 10, 12,
  and 14 spans preserved every public check but only reached 13.7164337975 deg;
  warm walls were 4.4719357, 3.4983968, 3.8779253, and 5.1402292 s. The quality
  gap is therefore representation-level, not a candidate-ranking defect.
- **Fixed-clock velocity-energy C3 QP:** rejected after the Straight Target
  gate. Six relinearized QPs passed continuous motion validation and beat the
  path record at 13.6049323647 deg on the exact 20.869565217 s clock, but took
  3.5579439 s in candidate scope and did not provide the required current
  plane-certificate parity. A one-QP form took 2.7812950 s and regressed path
  length to 14.1539862749 deg. Both missed the 2.0964864 s wall record; Target
  Exits was not run, and the experiment code was removed.
- **One-shot eight-span Bernstein velocity-energy QP:** rejected at the
  Straight Target runtime gate. The 447-line experiment eliminated 96 control
  scalars to 28 QP variables and reached the common exporter, but the focused
  end-to-end example did not finish within 30 measured seconds versus the
  2.0964864 s record. It was stopped before a valid path or certificate result
  was available; Target Exits was not run, and the hook and helper were removed.
- **Fixed-clock QP witness reuse and active-set solve:** rejected at the
  Straight Target gate. Directly retained constant support planes certified all
  288 output span-region pairs with zero analytic or conic fallbacks in
  0.0554667 s. Switching the same 28-variable QP from interior point
  (7.2986105 s) to MATLAB's active-set algorithm reduced successful seed solves
  to 0.1599676 s, 0.0308900 s, and 0.0235469 s. All five seeds were attempted,
  and the selected motion passed the exact 20.8695652173913 s clock, public
  collision, kinematic, and plane-certificate checks. Its 14.2707707658 deg
  path and 10.0545878 s full wall still missed the 13.678271908 deg and
  2.0964864 s records; topology alone took 2.3570636 s. Target Exits was not
  run, and all probe code and instrumentation were removed.
- **Continuous Bernstein safe-corridor QP:** rejected at its first frozen gate.
  Its 451-line candidate passed the public collision, kinematic, exact-clock,
  and seed-corridor checks on Straight Target, but produced a
  14.1505333646 deg path in 90.488413 s candidate scope versus the
  13.678271908 deg and 2.0964864 s records. Target Exits was not run, and all
  experiment code and artifacts were removed.

## Cross-cutting rejected solver and micro-optimization trials

- First-valid stopping, broad early continuation, average-speed starts,
  tighter arrival tolerance, globally skipped recovery, and larger clearance
  expansion each regressed a maintained arrival, failure, or runtime case.
- Vectorized static corridors, Bernstein and other cache variants, `TypicalX`,
  sparse Jacobians, repeated-query removal, and nargout-sized polynomial
  allocation won only microbenchmarks or missed their end-to-end gates.
- Broad SQP, conjugate-gradient, limited-memory BFGS, PCG, Parallel Computing
  Toolbox, and `parfor` variants timed out, regressed, or established no sound
  end-to-end benefit.

## Per-Seed Work Budget Verification — 2026-08-29

The README fixed-goal protected-rectangle request was rerun on
`422f887+worktree` with `PerSeedWorkBudgetMultiplier=3`. The retained selected
motion is seed 3 with a 7.574541766-second arrival and 11.411861388-degree
motion length. Final wall time was 14.060 seconds. The losing fourth BMTP seed
ended after 4.857 seconds with `seedWorkBudgetExhausted`, rather than an
independent-validation failure. Exclusive final stage timing was 0.4830 seconds
topology, 12.0845 seconds motion solving, 0.3230 seconds collision checking,
0.3292 seconds final validation, and 0.7899 seconds unattributed, totaling
14.0096 seconds. This is one measured static request, not a general runtime or
optimality claim. The full MATLAB suite passed 84/84; the timed and cavity
certificate coverage is direct, while the geometric lower-bound pruning proposal
was deliberately not implemented because a topology seed is not a mandatory
optimized vertex chain.

## Explicit timed-topology policy and conservative moving BMTP — 2026-08-31

The largest current strength is that a changing obstacle no longer causes an
undocumented switch to rest-to-rest waypoint composition. The public
`UnsupportedTimedTopologyPolicy` defaults to `"fail"`; an intentionally
work-limited static-U-plus-mover request returned the earliest accurate
`unsupportedTimedMultiWaypointRoute` reason with zero fallback attempts. The
same request succeeded at 31.4265 s only when
`"ruckigStopAtWaypoints"` was explicit, and diagnostics retained the original
failure plus every forced zero-velocity and zero-acceleration interior state.

Candidate-specific relevance removes the old request-wide static-kernel veto.
A static U with a distant moving obstacle succeeded through static BMTP at
20.8454 s and 39.5987 deg under the default fail policy after full continuous
validation against both obstacles. A structurally different translating
rectangle used the conservative protected-history convex-hull projection and
produced one globally smooth 20 s, 10.7117850149 deg BMTP motion with
0.0657896049 deg minimum clearance against the original moving geometry.

The largest current weakness remains genuine time dependence. The retained
projection is a conservative static swept-history superset; it cannot exploit
an obstacle opening later, couple separating planes to physical-time cells, or
guarantee a wait-plus-detour solution. Those cases still return
`unsupportedTimedMultiWaypointRoute` unless the explicitly labeled
stop-at-waypoints recovery is enabled. This branch therefore demonstrates one
moving-detour family, not a general dynamic BMTP completeness or optimality
result.

Final verification on `f383ae4+worktree` passed 98/98 tests in 69.6771 s.
All 17 maintained examples ran in separate serial headless MATLAB processes:
16 independently validated successes and the expected validated
`exampleNoPath` failure. A visible obstacle-free run also passed. The static-U
sentinel remained 20.712447786 s and the moving-barrier direct-wait sentinel
remained 10.0903015137 s.

## Balanced Selection And One-Sided Exact-Clock Economy — 2026-08-31

The largest current strength is that route choice now represents the stated
gimbal-wear trade rather than using jerk as a preference or choosing arrival
time lexicographically. The default balanced objective is actual motion travel
plus `MinimumTravelSavingsRate_deg_s` times elapsed time; jerk remains a hard
constraint, and normalized kinematic utilization is only a deterministic
tie-break. Static exact-clock detours now enumerate asymmetric, one-sided
progress polynomials whose peak locations are derived from direct-path
collision progress. Every proposal retains the clock-owning coordinate's
physical-limit motion and is accepted only after the unchanged continuous
validator passes.

On the motivating `newheart` bundle, the prior alternating fixed-clock motion
was 201.070948503 deg at the 100.970425693 s physical time floor. The retained
`oneSidedBeta_1_4` motion is 199.268051966 deg at the identical clock, has no
sign reversal relative to the direct chord, and passes continuous collision,
workspace, velocity, acceleration, and jerk checks. This removes
1.802896537 deg of travel. A structurally different near-start rectangle also
selected a one-sided exact-clock basis: 20.493950992 deg versus
20.585690610 deg for the validated alternating family.

The same change improved, rather than traded against, the existing rogue
sentinels: `sinetraj` reached 146.928879089 deg at 70.344250998 s, and balanced
`shrimp` reached 175.703912280 deg at 81.455142283 s. Balanced `non-ideal`
remained 228.491135293 deg at 144 s, while `hiddenruckigfallback` remained a
validated velocity-carried BMTP result with no silent Ruckig substitution.

The largest current weakness is bounded family coverage and runtime. The
one-sided portfolio applies only to static, two-axis, rest-to-rest requests
with one straight-progress coordinate owning the physical clock. It does not
prove globally shortest travel, and alternating/multi-obstacle homotopies may
still require topology BMTP. `newheart` used 154 full validation calls and
65.7788 planner seconds versus the saved 32.7045-second prior run. Runtime is
therefore an explicit regression on that rogue case, retained because
correctness and 1.8029 deg less gimbal travel have higher repository priority.

Final verification passed 110/110 tests in 126.782359 s. All 17 maintained
examples ran serially and headlessly: 16 continuously validated successes and
the expected `noValidatedSeed` result without an example-validation warning.
A visible obstacle-free smoke passed and created two figures. A fresh repeat
of the hidden failure-figure smoke was blocked after the example pass by
MATLAB's environment-level `System Error: File system inconsistency`; the same
worktree's earlier failure-plot check had already created the diagnostic figure,
and no plotting source changed in this final algorithm step.
