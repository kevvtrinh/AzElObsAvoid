# Plan 325 verification

## Preserve plans across finite timing disagreement - 2026-09-01

Item 17 changes `obstacleAvoidance.planner.stageTiming` reconciliation only.
The record now includes `TimingAccountingValid` and
`TimingAccountingResidual_s`. The residual is the independently measured total
elapsed time minus the sum of the five exclusive named stages. Residuals at
least negative clock tolerance are valid; larger finite over-attribution sets
the flag false. `UnattributedElapsedTime_s` remains the nonnegative part of the
residual, and `TotalElapsedTime_s` remains the independent wall measurement.

The three-input finalizer always writes the diagnostic record and elapsed wall
time back to the supplied planner result. A deliberate ten-second exclusive
stage finalized immediately retained `Success=true`, returned a negative
residual, and set the flag false. The two-input form likewise reconciled a
two-second exclusive stage against a one-second total to residual -1 and zero
unattributed time without throwing.

Input corruption still throws. Tests verify named `InvalidTimingValue` errors
for negative and `NaN` exclusive stages and `InvalidTotalElapsedTime` for an
infinite total. Production also validates every derived nonnegative elapsed
field, the logical flag, the finite signed residual, scalar structure shape,
and exact field order before reconciliation.

Code Analyzer returned zero findings for `stageTiming.m` and
`testPlannerStageTiming.m`. The focused suite passed 6/6: collision activity,
minimum-step unresolved evidence, success/failure format parity, motion-solver
reconciliation, finite over-attribution reporting, and corrupt-input rejection.
The summed test duration was approximately 2.109028 seconds. This change has no
trajectory or performance claim; its acceptance evidence is preservation of a
valid result with explicit unfavorable timing evidence.

## Isolate one-seed solving and bound early exit - 2026-09-01

Item 16 extracts the complete per-seed state machine into `solveOneSeed`. The
fallback order is unchanged: configured Ruckig or static BMTP; otherwise swept
static projection, timed BMTP if the swept result fails, compact direct-wait
construction, and only then the explicitly enabled stop-at-waypoint fallback.
The helper returns the candidate, independent validation, stable summary, and
accumulated stage timing. The main loop owns cancellation, candidate storage,
first-success timing, and final selection. Code Analyzer identified the old
`isBetterSummary` function as unused after extraction, so it was removed.

Wait refinement now runs when `GoalTimeMode` is `balancedArrival` because the
public objective is motion length plus the declared arrival-time exchange rate.
For a fixed direct path, avoidable wait can only worsen that objective. The
forced two-window regression runs both earliest and balanced policies; both
pass independent validation, perform local refinement, and select window one.

The seed loop records a physical arrival lower bound from the obstacle-free
direct motion. It stops only for earliest-arrival policy when a seed passes the
independent validator and its arrival differs from that bound by no more than
`ArrivalTimeTolerance_s`. Direct fast-path tests retain an explicit
`notApplicableExactPath` record; the later-wait regression retains
`lowerBoundNotReached`. No maintained case triggered the positive guard, so no
runtime saving is claimed from early exit.

Exact adjacent suite comparisons were:

- `testPlannerContract`: item-15 baseline 14/14 in 34.5795205 seconds;
  item-16 candidate 14/14 in 34.0742936 seconds (-1.461 percent).
- `testTimedBmtpPlanning`: baseline 3/3 in 12.2135944 seconds; candidate
  3/3 in 12.3922855 seconds (+1.463 percent).
- `testSandboxRouteEconomy`: baseline 3/3 in 10.4367040 seconds; candidate
  3/3 in 10.0221966 seconds (-3.972 percent).

After final diagnostic assertions, planner contract again passed 14/14 in
34.103085 seconds and obstacle-history robustness passed 8/8 in 2.884767
seconds. Code Analyzer returned zero findings for the planner and both changed
tests. These are single-process suite observations and establish decision
neutrality, not a general performance change.

The parallel evaluation was kept separate. `license('test',
'Distrib_Computing_Toolbox')` returned one, but `ver('parallel')` returned no
installed product and both `parpool` and the pool runtime were absent; calling
`gcp` reported that Parallel Computing Toolbox was required. No unmeasured
`parfor` branch or nested parallelism was retained.

## Refine every event-aware feasibility window - 2026-09-01

Item 15 replaces the single global fail/pass bisection in moving-target
intercept and direct-wait refinement. A shared chronological grid retains a
16-interval uniform background, every caller-supplied source event, and one
midpoint between consecutive events. Each path evaluates the complete grid,
records every observed false-to-true window opening, refines each opening
locally to the declared arrival tolerance and iteration bound, and chooses the
earliest result that passed the independent public validator. Diagnostics now
retain event/trial times, trial outcomes, window brackets, the selected window,
and the horizon-projection ownership key.

The intercept regression uses a stationary target and a blocker with clear
windows at source times 2.90--3.08 and 6.50--8.00 seconds. The exact item-14
baseline returned 6.582983337402 seconds; the candidate returned
2.931640625000 seconds and recorded at least two windows. The forced direct-wait
regression uses a full-workspace barrier with two clear intervals, so no lateral
route can replace timing. Baseline refinement returned 5.021850585938 seconds
of wait; the candidate returned 2.022460937500 seconds, recorded two windows,
36 coarse trials, 16 refinements, and selected window one.

The focused warm medians quantify the correctness cost: intercept rose from
0.4793112 to 0.7438613 seconds (55.185 percent) and direct wait from 0.4451262
to 0.6665108 seconds (49.738 percent). The retained implementation prepares the
immutable obstacle collection once before intercept trials, preserves the
cache through canonical normalization only when every record carries it, and
relies on `prepareDynamic`'s exact source snapshot to reject stale data.
Internal preparation is stripped before assembling public result inputs.
Horizon-dependent geometry is not shared: intercept diagnostics key it by
`trialGoalTime_s`, and direct-wait diagnostics by `candidateTimeRange_s`.

Code Analyzer returned zero findings for the four changed production files,
the new event-grid helper, and both changed tests. MATLAB results were:

- `testObstacleHistoryRobustness`: 8/8 passed in 4.695680 seconds.
- `testObstacleInfrastructure`: 10/10 passed in 1.301944 seconds.
- `testPlannerContract`: 14/14 passed in 33.869245 seconds.
- `testTimedBmtpPlanning`: 3/3 passed in 12.702431 seconds.
- `testSandboxRouteEconomy`: 3/3 passed in 10.675562 seconds.
- `testPlannerOptions`: 5/5 passed in 0.513904 seconds.

The maintained `exampleObstacleAvoidance` was run headlessly four times per
side with jerk enabled. Baseline and candidate both passed planning and
independent validation with 11.152119519024-degree selected polyline,
11.411861387735-degree smoothed motion, 7.574541766321-second duration, and
collision/kinematic/certificate status 1/1/1. Adjacent warm-plus-three medians
were 3.4247017 seconds baseline and 3.2834362 seconds candidate (4.125 percent
faster). This is neutral evidence that ordinary static planning did not regress
in the sample, not a general end-to-end speedup claim.

## Reduce spatial-homology overhead - 2026-09-01

Item 14 replaces string state keys with an injective numeric encoding. Each
winding component is an `int8` digit in `[-1, 1]`, shifted to base-3 digit
`[0, 2]`; the node index occupies the least-significant block. Key powers and
the complete node/signature product are checked against `uint64` capacity
before the first dictionary insertion. A 40-component/two-node regression
proves overflow returns the identified `StateKeyCapacityExceeded` error rather
than wrapping or colliding.

Cleanup alternatives now call `shortestpathtree` exactly twice—once from the
start and once from the goal—and reuse their path cells for every waypoint.
A chain regression verifies the cleaned route, zero-width signature matrix,
`uint64Base3` evidence, retained `linearScan`, two tree builds, and accepted
shortcut. Code Analyzer reported zero findings. Spatial-homology tests passed
2/2, planner contract 14/14, and route economy 3/3.

A deterministic 50-node bent chain exercised 4,900 state encodes and cleanup
on 50 repetitions. The exact parent implementation took 1.814420 profiled
seconds total, 0.051524 seconds in formatted `stateKey`, and 0.392447 seconds in
cleanup alternatives. The retained candidate took 1.481489 seconds total,
0.005690 seconds in `numericStateKey`, and 0.137429 seconds in cleanup, reductions
of 18.349, 88.956, and 64.981 percent respectively. Route count, state count,
expansions, cleanup acceptance, and returned geometry were unchanged.

A deterministic binary heap with exact cost/state-index tie ordering was then
benchmarked against the retained linear scan. It returned the same 50 states
and 49 expansions but took 1.770479 profiled seconds, 19.505 percent more than
the 1.481489-second linear candidate. The heap was removed. The final maintained
`exampleObstacleAvoidance` warm-plus-three median was 3.3334726 seconds versus
item 12's 3.2084006 seconds, a 3.898 percent increase with exact
11.4118613877-degree motion and 7.5745417663-second arrival. Retention is based
on the affected homology workload, not an end-to-end speedup claim.

## Reject time-expanded move-then-wait candidate - 2026-09-01

Item 13 was implemented and removed under the bounded-change gate. Its layer
selector reserved request endpoints and authoritative obstacle source events
before midpoint and uniform candidates and reported when hard events alone
exceeded the layer cap. Search transitions used `uint32` parent/transition
indices, vectorized duration and target-layer calculations, and preallocated
the full explored-node trace. A moving edge arrived after the existing
input-derived minimum transition duration, then retained a duplicate destination
knot through the target layer rather than stretching motion across the gap.

Three new focused tests passed: event priority under ordinary truncation,
explicit hard-event overflow, and a one-second move followed by a nine-second
wait reconstructed at times `[0, 1, 10]`. Code Analyzer reported zero findings.
The adversarial history suite stayed 6/7 with only item 15 red; timed BMTP
passed 3/3, planner contract 14/14, and route economy 3/3.

The first complete candidate made the timed-BMTP test setup take about 180
seconds. Its profile attributed 154.334851 seconds to 59 occupancy queries,
including 374,872 `shapeAtTime`, 374,942 preparation, and 56,926 exact
point-clearance calls. A secondary general batching experiment evaluated
static shapes, rigid translations in the source frame, and conservative
interval enclosures without per-time shape construction; unsupported
deformation retained the exact fallback. It passed 10/10 obstacle equivalence
tests and reduced the timed suite to 30.1980109 seconds.

The exact immediate item-12 `ab5c0d9` suite still completed in 20.4758520
seconds. Its profile used 269 conic-wrapper solves, while the explicit
arrival/wait candidate used 680 because the extra timed knots enlarged the
continuous formulation. The final candidate was 47.482 percent slower despite
valid output. Reducing the 13 motion plus 12 nonduplicate dwell samples or
hiding the additional solves would weaken the declared comparison, so all
item-13 production and test code was removed. Content hashes of the four
affected tracked files match `ab5c0d9` exactly; only this negative record is
retained.

## Restore occupancy bounding-box acceleration - 2026-09-01

Item 12 adds exact-time bounds to the lightweight `shapeAtTime` geometry record
and removes occupancy's convex request-horizon construction. Occupancy-only and
first-blocker calls use a tolerance-expanded box and skip points already
blocked by an earlier obstacle. Minimum-clearance diagnostics keep processing
a point unless a positive point-to-box distance is already no better than the
best exact signed clearance. Thus an overlapping box or negative current
clearance is never pruned unsafely, and caller-order first-blocker behavior is
unchanged.

Code Analyzer reported zero findings in both production files and the modified
test. Obstacle infrastructure passed 10/10, including one-, two-, and
three-output equivalence, multi-ring batched/pointwise agreement, boundary
policies, and exact diagnostic clearance. Planner contract passed 14/14 and
timed BMTP 3/3. The adversarial suite remained 6/7, with only item 15's known
first-window failure at 6.5829833374 seconds.

An exact no-warmup profile at immediate parent `2f7f91f` and a candidate
warmup/profile both returned the same successful, independently validated
17.9853686689-second seed-1011 result. Decision work was identical: 259
retained collision intervals, zero unresolved, 57 occupancy calls, 167 route
states, 268 expansions, 4,852 `coneprog` calls, one projection, one static
representation, and one convex decomposition. `polyshape` calls fell from 711
to 423, a 40.506 percent exact reduction. Lightweight `shapeAtTime` calls rose
from 5,639 to 15,488 because box queries now avoid shape construction.

The candidate profiled wall was 100.4388029 seconds and the cold parent profile
was 99.2463007 seconds; those differently warmed profiler runs are not a valid
speed comparison. A separate candidate warm-plus-three static median was
3.2084006 seconds versus item 11's 3.0803439 seconds, a 4.157 percent increase,
with exact 11.4118613877-degree motion and 7.5745417663-second arrival. Item 12
is retained for the material exact construction reduction, not a runtime claim.

## Add a certified continuous-bound fast path - 2026-09-01

Item 11 first converts each ascending-power scalar polynomial to equal-degree
Bernstein controls. A hull wholly within the allowed range proves the interval;
otherwise the checker recursively bisects through de Casteljau construction.
A child whose complete hull lies beyond one side proves a violation. Ambiguous
children fall back to endpoints and every numerically real stationary root.
The fallback therefore remains authoritative where subdivision does not decide,
and a single outside Bernstein coefficient is not used as an exact rejection.

Two permanent public-validator regressions use `p(t) = a*t*(1-t)`. With
`a = 4`, a Bernstein control lies above the one-degree limit while the actual
curve only touches it; validation passes. With `a = 5`, the same polynomial
family truly reaches 1.25 degrees; continuous position validation fails. Both
tests pass, as do planner stage/collision 5/5, planner contract 14/14, and timed
BMTP 3/3. The adversarial suite remains at its expected pre-item-15 state of
6/7, selecting 6.5829833374 seconds in the still-red first-window test. Code
Analyzer reported zero findings in both changed files.

An exact detached item-10 `d40422f` profile of
`exampleObstacleAvoidance` performed 600 `withinBounds` calls and 537 `roots`
calls, with 0.391988 seconds attributed to `validateTrajectory`. Item 11
performed the same 600 checks with zero `roots` calls and 0.226719 seconds in
validation, a 42.162 percent profile-local decrease. Its warm-plus-three
end-to-end median was 3.0803439 seconds versus item 10's 3.3586421 seconds,
an 8.287 percent observation. The physical result remained exactly
11.4118613877 degrees of sampled motion and 7.5745417663 seconds arrival. The
exact 537-to-zero root-call reduction, not a broad runtime claim, satisfies the
retention gate.

## Strengthen continuous collision certification - 2026-09-01

Item 10 replaces the global workspace velocity norm and whole-request obstacle
boxes used by the adaptive certificate. Velocity power coefficients are
converted once per polynomial segment to Bernstein controls, and stable de
Casteljau subdivision bounds each exact adaptive interval. Prepared sample
bounds, interval bounds, interpolation deltas, and speed bounds provide an
obstacle-specific swept box for the same time slice. Midpoint clearance plus
the combined path/obstacle Lipschitz bound remains a proof; dense sampling was
not substituted, and an interval that reaches the minimum timestep without a
proof still fails closed.

`Validation.CollisionDiagnostics` is stable on every exit. It records the proof
method and termination reason; broad-phase reject counts; maximum path and
obstacle speed bounds; the configured minimum timestep; and, when unresolved,
the last interval, limiting obstacle index, both speed bounds, required
certifiable clearance, and observed midpoint clearance. A deliberate near-miss
with a one-second minimum step failed closed and reported interval `[0, 1]`, a
2 deg/s path bound, zero obstacle speed, and required clearance greater than
the observed clearance.

Code Analyzer reported zero findings. Planner stage/collision tests passed 5/5,
obstacle infrastructure 10/10, planner contract 14/14, and timed BMTP 3/3.
The adversarial suite remained at its scheduled 6/7 state: the only failure is
item 15's first-intercept-window case, which selected 6.5829833374 seconds
instead of a time below 3.09 seconds.

The final adjacent warm-plus-three seed-1011 comparison used exact item-8
commit `1aaff35` as baseline. Baseline measured times were 86.5679274,
84.7575470, and 86.0659709 seconds (median 86.0659709). Candidate times were
86.2290776, 86.6045835, and 85.9689313 seconds (median 86.2290776), a 0.190
percent increase treated as neutral. Both returned the same independently
validated 17.9853686689-second arrival and zero unresolved intervals. Retained
adaptive intervals fell exactly from 331 to 259, or 21.752 percent, which is
the material work-reduction evidence for retention.

A structurally different adjacent static gate retained the exact
11.4118613877-degree motion and 7.5745417663-second arrival while its median
changed from 3.5125306 to 3.3586421 seconds. The maintained moving
`exampleTargetExitsObstacle` also passed planning and independent validation:
20.1357890335-degree polyline, 20.6100682085-degree motion, 24-second duration,
collision, kinematic, and applicable-certificate checks all true, and
26.8137175 seconds wall. No end-to-end speedup is claimed.

## Reject direct cached-edge clearance - 2026-09-01

Item 9 evaluated a direct signed-clearance implementation over the cached
boundary edges created by item 8. Five focused equivalence cases covered holes,
disconnected components, NaN-separated rings, boundary points, orientation,
exact sample reuse, verified interpolation, and conservative fallback. The
candidate passed 5/5 equivalence tests, obstacle infrastructure 10/10, planner
stage timing 4/4, planner contract 14/14, timed BMTP 3/3, and the adversarial
robustness suite's established 6/7 state. The sole red robustness case remained
the item-15 disjoint intercept-window test, selecting 6.668424 seconds rather
than the required first window below 3.09 seconds.

Repeated timings rejected the candidate. A strict warmup followed by three
seed-1011 runs produced an item-8 median of 81.4626468 seconds and a candidate
median of 81.3327973 seconds, a 0.159 percent decrease. Of 4,953 clearance
queries, only 144 used direct cached edges while 4,809 still required
`polyshape`. A structurally different static maintained example retained exact
physical output but regressed from a 2.9158384-second baseline median to
3.1493434 seconds, or 8.008 percent. The candidate added 188 net production
lines. All item-9 production and test code was removed; content hashes of the
affected tracked files match commit `1aaff35` exactly. Only this negative
benchmark record is retained.

## Own one prepared obstacle collection per request - 2026-09-01

Item 8 gives every prepared obstacle a schema version and an exact snapshot of
all canonical public source fields. A current snapshot reuses preparation; any
change to time, protected or original boundaries, safety margin, status, or
name invalidates and rebuilds the complete collection. The cache now contains
exact sample shapes, sample and interval bounds, ordered sample and fallback
edges, per-ring bounds, interpolation deltas, and speed bounds. Static
projection no longer reconstructs canonical obstacles or discards their valid
request-owned preparation.

The new mutation regression shifts every protected sample of an already
prepared rectangle by 5 degrees. `shapeAtTime`, occupancy, static projection,
and direct re-preparation all use the shifted geometry rather than the stale
sample shape. Obstacle infrastructure passed 10/10, static projection 2/2,
planner contract 14/14, and timed BMTP 3/3. Code Analyzer reported zero
findings for the five modified production/test files.

The identical seed-1011 warmup passed in 128.5012 seconds; the profiled repeat
passed planning and independent validation in 137.6644170 seconds. Arrival
remained 17.9853686689 seconds, with 167 route states, 268 expansions, 331
retained collision intervals, zero unresolved intervals, 57 occupancy calls,
5,871 `shapeAtTime` calls, and 4,852 `coneprog` calls. Preserving preparation
reduced `polyshape` constructions from item 7's 855 to 741 (13.333 percent).
The 0.588 percent profiled wall decrease is not an end-to-end speedup claim;
the exact construction reduction is the retention evidence.

## Reduce invariant planner calls - 2026-09-01

Item 7 now creates a request-invariant static planning projection and its BMTP
convex representation before the seed loop. Static requests likewise create
their BMTP representation once. `StaticProjectionCreationCount` and
`StaticRepresentationCreationCount` are stable search-diagnostic fields, and
the static and moving BMTP tests require the applicable count to equal one.
Candidate selection already copied each candidate's retained independent
validation, so no duplicate selection-time validation was present to remove.

Fixed-clock amplitude refinement now stops on the independently validated
side of the bracket at a documented 0.001-degree physical resolution, a larger
caller-supplied collision tolerance when applicable, or twelve bisections.
The target, achieved reserve, iteration count, and cap remain visible in the
returned diagnostics. No collision tolerance, obstacle geometry, seed budget,
or validator requirement was weakened.

The identical warmed/profiled seed-1011 gate compared against item 6. Planner
and independent validation remained successful; arrival stayed
17.9853686689 seconds, with 167 route states, 268 expansions, 331 retained
collision intervals, zero unresolved intervals, and 4,852 `coneprog` calls.
Profiled wall time decreased from 143.0640531 to 138.4790277 seconds, only
3.205 percent, so no end-to-end speedup is claimed. Exact work did decrease:
`polyshape` construction fell from 1,377 to 855 (37.908 percent),
`shapeAtTime` calls fell from 5,988 to 5,871, and the repeated projection and
BMTP decomposition fell from four constructions to one.

Code Analyzer reported zero findings for the five modified/added planner
files. Planner-contract tests passed 14/14, timed-BMTP tests passed 3/3, and
route-economy tests passed 3/3. These cover a static request, a structurally
different moving-obstacle request, and the fixed-clock path-quality bounds.
The subsequent static-projection audit exposed that the internal adapter's
established canonical-obstacle call form had been narrowed to the reusable
representation. A forwarding compatibility path now constructs one
representation for that standalone call, while public planner requests still
pass the request-owned representation. Static-projection tests pass 2/2.
The profiler remains roughly five times slower and uses over twice as many
`coneprog` calls as the pre-correctness item-2 baseline; that unfavorable cost
remains visible for later items rather than being attributed to this modest
call reduction.

## Restrict geometry to the request horizon - 2026-09-01

Item 6 added one source-derived `queryHorizonGeometry` record and routed static
planning projections, dense fallback envelopes, seed-corridor replay,
occupancy broad-phase bounds, and continuous-validator broad-phase bounds
through it. The record includes exact request endpoints, stored samples inside
the closed horizon, and every conservative fallback interval intersecting the
horizon. Samples and intervals wholly outside the request are omitted.

The prior red projection case now passes: a `[0, 1]` request over a history
with remote samples at -10 and 10 seconds retains the expected azimuth span
`[-1, 2]` instead of `[-101, 101]`. `checkcode` reported zero findings for all
seven modified or added production files. Static projection passed 2/2,
obstacle infrastructure 9/9, stage timing 4/4, and the public planner contract
14/14. The adversarial suite now passes 6/7; only the disjoint intercept window
scheduled for item 15 remains red.

The maintained `exampleTargetExitsObstacle` ran headlessly with jerk enabled.
Planning and independent validation passed with `goalReached`, a
20.1357890335-degree selected polyline, 20.6100682085-degree sampled motion,
24-second duration, collision freedom, full kinematic compliance, and an
applicable independent certificate. Wall time was 35.1896642 seconds. No
runtime comparison is claimed from this correctness-focused run.

## Conservative obstacle interpolation - 2026-09-01

Item 5 replaced size-and-finite-mask correspondence inference with a bounded
single-ring alignment and proof. Cyclic shifts and orientation reversal are
enumerated deterministically. Translation is verified directly; other aligned
single rings must remain strictly convex over the complete linear interpolation
interval. Equivalent multi-ring samples remain static. Every unsupported or
topology-changing interval now uses the convex hull of all finite endpoint
vertices, occupied for the whole interval, rather than a separated endpoint
union.

The two applicable red regressions now pass. Equivalent reordered rectangles
return their unchanged occupied set with zero speed, and the mismatched
topology translation occupies `[0, 0]` in its between-sample gap. The complete
`testObstacleInfrastructure` suite passed 9/9, static projection passed 2/2,
timed BMTP passed 3/3, and unsupported-timed-topology policy passed 2/2.
`checkcode` reported zero findings for both modified production files. The
seven-case adversarial suite now passes 5/7; only the deliberately separate
request-horizon and disjoint-intercept tests remain red for items 6 and 15.

The maintained `exampleMovingCircleNoAzimuthWrap` ran headlessly with jerk
enabled. Planning and independent validation passed with `goalReached`, a
12.4795774865-degree selected polyline, a 12.4795774865-degree sampled motion,
8.5-second duration, collision freedom, and velocity, acceleration, and jerk
limits all passing. No seed- or plane-certificate fast path applied, so the
independent continuous collision check was authoritative. Wall time was
4.4307681 seconds. This focused run is a correctness check, not a runtime
comparison.

## Adversarial obstacle-history baseline - 2026-09-01

Before changing moving-obstacle semantics, the new deterministic
`testObstacleHistoryRobustness` suite ran against commit `b6239cb`. `checkcode`
reported zero findings. Three cases already passed: sampled rotation followed
the existing linear corresponding-vertex model, a 0.1-second source event was
retained as a timed-search layer, and `status` remained conservative metadata
rather than silently deactivating geometry.

Four cases failed and preserve the exact defects that later groups must fix.
Cyclically shifted and reversed representations of one rectangle fabricated
up to 8 square degrees of shape difference and a 4.47213595499958-degree/s
speed. A topology-changing translation left the point `[0, 0]` clear in the
between-sample swept gap with 2 degrees reported clearance. A `[0, 1]` request
projection incorrectly included stored samples at -10 and 10 seconds, growing
the expected azimuth span `[-1, 2]` to `[-101, 101]`. Finally, chronological
intercept sampling skipped a feasible 2.89-to-3.09-second window and selected
the later disjoint window at 6.668424011230469 seconds. These are expected
red tests, not accepted planner outcomes.

## Obstacle-performance profiler baseline - 2026-09-01

The fixed pre-change commit was
`0a55ef7e1500a9d64b37187ce97c7218d682b8b4` on
`obstacle-performance-robustness`. That commit changed only plans and
instructions from the previously measured planner, so the production MATLAB
code was identical to the 12/12 independently validated moving-polygon corpus
reported below. MATLAB R2024b Update 4 and Optimization Toolbox 24.2 ran on
64-bit Windows 11 10.0.26200 with an AMD Ryzen 5 3600, six physical cores,
and twelve logical processors.

The deterministic seed-1011 request was warmed and then profiled in the same
MATLAB session. The warmup passed public planning and independent validation
in 26.5167718 seconds. The profiled repeat also returned `goalReached` and
passed independent validation in 27.6499923 seconds. The profile attributed
16.210907148 seconds to 2,272 `coneprog` calls, 3.815109682 seconds to 55
occupancy-query calls, 2.451599353 seconds to 7,714 point-to-polygon clearance
calls, and 1.140148324 seconds to 7,969 `shapeAtTime` calls. It counted 8,087
top-level `polyshape` constructions and four full independent validations;
49 zero-input validation-template requests were reported separately.

`benchmarkObstaclePlannerProfile` now reproduces those profiler counts and
also reports returned collision work and route-search state counts. A
post-change diagnostic run preserved the same physical outcome and the same
function-call counts: 2,272 `coneprog`, 7,969 `shapeAtTime`, 8,087
`polyshape`, 55 occupancy queries, four full validations, 105 homology states,
and 284 expanded search states. The retained candidate summaries reported 209
adaptive collision intervals and zero unresolved intervals. Its profiled wall
time was 42.7303887 seconds; this variable profiler time is retained as an
unfavorable measurement and is not used as a performance comparison.

The diagnostic-only implementation passed `checkcode` with zero findings for
all four modified/added production and benchmark files. All four
`testPlannerStageTiming` cases and all fourteen `testPlannerContract` cases
passed. The collision test now proves the interval counter advances when an
adaptive midpoint detects a collision, while the existing success/failure
shape checks prove the new field is stable on every result path.

## Moving-obstacle robustness audit - 2026-09-01

The fixed baseline was clean commit
`81a94be2224ef6dd0183848772f5fbdbce81b85f` on
`compare-conic-solvers`, MATLAB R2024b Update 4. The supplied shared-chat
snapshot contained only `plan_coneprog.md`, so the audit used the repository's
existing `benchmarkRandomMovingPolygonStress` corpus instead of inventing a
missing artifact or regenerating an unfavorable seed.

The first two launch attempts were setup failures and did not execute planner
code: the benchmark folder, then the `trajectory` package root, were absent
from the MATLAB path. The corrected command added only the repository root,
`trajectory`, and `benchmarks` before calling:

```matlab
benchmarkRandomMovingPolygonStress(1011, struct("PrintProgress", true))
benchmarkRandomMovingPolygonStress(1001:1012, ...
    struct("PrintProgress", true))
```

Focused seed 1011 passed public planning and independent validation, returned
`goalReached`, preserved its 2.40165847462-degree analytic witness, and arrived
at 17.9833348954 seconds in 26.526273 seconds wall time. The complete serial
corpus then produced:

| Seed | Success / validation | Arrival (s) | Witness (deg) | Wall (s) |
| ---: | :---: | ---: | ---: | ---: |
| 1001 | 1 / 1 | 17.7894304357 | 2.69544318398 | 24.0943898 |
| 1002 | 1 / 1 | 17.9179911782 | 2.71859797993 | 68.6336954 |
| 1003 | 1 / 1 | 18.1897334792 | 2.76428444127 | 40.7002512 |
| 1004 | 1 / 1 | 17.6869100774 | 3.45094899920 | 21.0016595 |
| 1005 | 1 / 1 | 18.6733536574 | 2.82510707443 | 29.0789434 |
| 1006 | 1 / 1 | 17.3773712877 | 3.35991651014 | 73.0848992 |
| 1007 | 1 / 1 | 18.5580002700 | 2.69072852617 | 82.4723368 |
| 1008 | 1 / 1 | 18.5048278151 | 2.74471951247 | 54.7600947 |
| 1009 | 1 / 1 | 18.7423396702 | 2.63077659636 | 36.1416153 |
| 1010 | 1 / 1 | 18.4718184308 | 2.51526924210 | 32.5048258 |
| 1011 | 1 / 1 | 17.9833348954 | 2.40165847462 | 23.4149016 |
| 1012 | 1 / 1 | 18.0660421140 | 2.80039171894 | 23.4077390 |

All twelve cases returned `goalReached`. Total corpus wall time was
509.2953517 seconds; per-case minimum, median, and maximum were 21.0016595,
34.32322055, and 82.4723368 seconds. No planner or test file changed, so the
already-recorded 114/114 suite, 17 serial headless examples, visible success,
and no-path diagnostic at the same commit remain the applicable production
verification. `benchmark.csv` remains unchanged because this stress corpus is
not a maintained example/motion-mode row under its stable schema.

## Dormant seed-clustering removal - 2026-09-01

The authoritative baseline was commit
`11582e39998de997c1e453c4e5f798d4a3d4961f`; its complete 17-example result
artifacts were already recorded under `tmp/bmtp-seed-budget-full`. The
candidate was branch `bmtp-cleanup-codex`. Every new MATLAB invocation used a
fresh private `MATLAB_PREFDIR`, `TEMP`, and `TMP`, and maintained examples ran
one process at a time.

Read-only caller tracing found that `SeedClusterDistance_deg` defaulted to zero
and no current maintained example, test, benchmark, or sandbox supplied it.
Zero caused `clusterSeedShape` to return the protected swept geometry
unchanged. Historical evidence had one maintained nonzero use, but its swept
input contained one source region and therefore formed zero cluster groups.
The accepted edit:

- deletes the 85-line `+obstacleAvoidance/+search/clusterSeedShape.m` helper;
- removes `SeedClusterDistance_deg` from defaults and validation;
- warns once that a legacy value is deprecated and ignored, strips it before
  unknown-option handling, and forwards it through the example boundary;
- always constructs visibility and homology proposals from the unclustered
  protected swept geometry;
- retains `SearchDiagnostics.Grid.SeedCluster` for one release with distance
  zero, the true source-region count, zero groups, zero clustered regions, and
  an empty boundary;
- removes the obsolete helper entry from current architecture and technical
  documentation.

Production MATLAB changes were 17 additions and 100 deletions, a net reduction
of 83 lines, exceeding the declared 75-line gate. Together with prior
milestones, the branch production reduction is 163 lines. `git diff --check`
passed; stale helper references are absent outside historical records and the
one-release option shim/tests. MATLAB Code Analyzer reported zero findings in
`resolvePlannerOptions.m`, `createRouteCandidates.m`, and
`resolveExampleOptions.m`.

Focused verification passed 48/48 option, example, contract, architecture,
static-projection, timed-BMTP, and unsupported-topology tests. Legacy partial
and fully resolved options warn with
`planTrajectory:DeprecatedSeedClusterDistance`, are not also unknown, do not
echo in resolved options, and cannot restore clustering. The maintained
obstacle-avoidance test additionally protects the retained diagnostic record.

A new neutral three-region fixture established both sides of the behavior:

| Call | Groups | Nodes | Edges | Polyline/motion (deg) | Arrival (s) | Wall (s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Frozen distance 0 | 0 | 26 | 46 | 8.08716891419 | 6.5 | 4.2970193 |
| Frozen distance 1 deg | 1 | 10 | 16 | 8.08716891419 | 6.5 | 4.2952183 |
| Candidate legacy 1 deg | 0 | 26 | 46 | 8.08716891419 | 6.5 | 4.3206596 |

The candidate legacy replay matched the frozen zero-distance planner result
recursively at `1e-9`, including unclustered graph counts, route, trajectory,
validation, termination, and zero-valued diagnostic fields. The nonzero
baseline proves the removed code was capable of changing the graph rather than
being unreachable. The candidate uses more graph nodes for legacy nonzero
requests, as intended, but this fixture showed no material wall-time or result
cost. No global fragmented-field runtime claim is made.

All 17 maintained candidate examples then ran serially and headlessly with
jerk enabled. Every result matched the corresponding frozen `11582e3` artifact
recursively at `1e-9`, excluding only runtime and the removed option. Sixteen
returned planner and independent validation success; `exampleNoPath` returned
the expected independently validated `noValidatedSeed`. Every successful
motion remained collision-free and kinematically valid, with applicable
certificates passing. Exact per-example lengths, durations, wall times and
certificate applicability are appended to `benchmark.csv`.

The hidden failure-plot gate created one invisible figure titled
`Azimuth/elevation motion plan | noValidatedSeed | seeds 1 | expanded 1 |
rejected 2`. The visible obstacle-free gate created two figures with
`Visible="on"`. The complete test tree passed 117/117 with zero failed or
incomplete, 79.0522037 seconds aggregate test duration, and 85.7283342 seconds
wall time.

Three fixture/setup errors produced no adverse planner finding and were
corrected without weakening a gate. The first fixture attempted a nonexistent
zero-input obstacle constructor; the second queried the cluster record from a
planner fast-path diagnostic rather than directly from route generation; and
the first candidate comparison lost the external comparator path when the
runner restored MATLAB's default path. The corrected fixture explicitly
constructs all obstacles, invokes the maintained route generator on the
resolved request for graph evidence, and compares already-saved results in a
fresh process. No required gate remains untested.

## Wall-clock seed cutoff removal - 2026-09-01

The authoritative baseline was commit
`dd7a67441086c651c818adbd65c110cb72c752fa` in detached worktree
`tmp/bmtp-seed-budget-baseline`; the candidate was branch
`bmtp-cleanup-codex`. Every MATLAB command used a fresh private
`MATLAB_PREFDIR`, `TEMP`, and `TMP`, and every maintained example ran in its
own process. The predeclared gate allowed a documented runtime increase but
required no representative result-quality regression.

The accepted edit removes the complete timing-dependent seed cutoff:

- `PerSeedWorkBudgetMultiplier` is no longer a planner default or echoed
  result option. A one-release shim warns once that a supplied legacy value is
  deprecated and ignored, then strips it before unknown-option handling.
- `planCorridorQuintic` no longer computes an incumbent-derived wall budget,
  injects it into a seed solve, or translates a solver stop into
  `seedWorkBudgetExhausted`.
- `bmtpEngine.solve` no longer accepts or polls `MaximumSolverTime_s`, and no
  longer publishes `WorkLimitReached`.
- The example option boundary forwards the legacy public name to the central
  resolver. Deterministic iteration/conic limits, `MaximumSeedCount`, explicit
  cancellation, validation, diagnostics, and the public BMTP restart surface
  are unchanged.

Production MATLAB changes were 17 additions and 57 deletions, a net reduction
of 40 lines. The README loses six obsolete lines. Together with the previous
milestones, the branch production reduction is 80 lines. Static verification
passed `git diff --check`, stale-name searches, the 40-line benefit gate, and
MATLAB Code Analyzer with zero findings in
`resolvePlannerOptions.m`, `planCorridorQuintic.m`,
`resolveExampleOptions.m`, and `bmtpEngine/solve.m`.

Focused tests passed as follows:

- planner options, example invariants, BMTP engine and planner contract:
  43/43;
- cancellation and offline-sandbox behavior: 14/14;
- legacy partial and fully resolved option replay warns exactly once with
  `planTrajectory:DeprecatedPerSeedWorkBudgetMultiplier`, is not also unknown,
  and is absent from resolved options;
- the public seven/eight-input, three-output BMTP restart tests remain green.

Recursive result comparisons ignored only runtime fields and the declared
`PerSeedWorkBudgetMultiplier` / `WorkLimitReached` migration. Every comparison
passed at `1e-9`:

| Case | Policy | Arrival (s) | Motion (deg) | Candidate wall (s) | Result |
| --- | --- | ---: | ---: | ---: | --- |
| Straight target alternating occlusion | fixed | 20.8695652174 | 13.5713266002 | 25.1877224 | exact; seed 5 selected |
| Protected rectangle | earliest | 7.57454176632 | 11.4118613877 | 5.0345331 | exact |
| Protected rectangle | balanced | 7.54855735896 | 11.2161345431 | 7.2766595 | exact; composite 18.764691902 deg |
| Extreme outline | earliest | 5.81065318159 | 23.3457566443 | 72.80709 | exact |
| Moving/deforming outline | earliest | 7.91666666667 | 40.2805670061 | 27.1419769 | exact |
| Expected no path | earliest | NaN | NaN | 2.7477454 | exact `noValidatedSeed` |

The controlled pre-edit completed-seed walls were 6.1394393 seconds for the
earliest rectangle, 7.1542421 seconds for balanced, and 72.2275885 seconds for
the extreme outline. Runtime is secondary and noisy; these comparisons show
no material extra engine cost after both sides complete the same seeds. The
operational cost comes from allowing formerly cut-off losing seeds to finish.
In the serial default sweep, the extreme outline grew from the previous
42.1929796-second cutoff run to 67.4136091 seconds. This unfavorable result is
retained visibly and accepted under the declared gate.

All 17 maintained examples then ran serially and headlessly with jerk enabled.
Sixteen returned planner success plus independent example-validation success;
`exampleNoPath` returned the expected independently validated
`noValidatedSeed`. Every successful result was collision-free and passed
velocity, acceleration, jerk and dynamics checks; applicable certificates
passed. Exact lengths, durations, wall times and applicability are recorded in
`benchmark.csv`. The fixed alternating-occlusion sweep selected seed 5 with
27.950433436 degrees of seed polyline, 13.5713266002 degrees of final motion,
and 20.8695652174 seconds duration. The extreme-outline sweep retained
22.070643085 degrees of seed polyline, 23.3457566443 degrees of final motion,
and 5.81065318159 seconds duration.

The hidden failure-plot gate created one invisible figure titled
`Azimuth/elevation motion plan | noValidatedSeed | seeds 1 | expanded 1 |
rejected 2`. The visible obstacle-free gate created two figures with
`Visible="on"`. The complete test tree passed 115/115 with zero failed or
incomplete, 79.2300998 seconds aggregate test duration, and 85.891126 seconds
wall time.

Several reporting/setup mistakes did not invalidate saved planner evidence:
the first sandboxed MATLAB start failed before repository code and was rerun
with the approved isolated process; early reporting expressions queried the
obsolete top-level names `Trajectory`, `BalancedArrivalTradeoff_deg_s`, and
`ExpandedStateCount`; the no-path inspection initially expected successful-
trajectory validation on an expected failure; and one older moving/deforming
artifact stored its result under `evidence.Result`. In each case the planner
result had already been saved, recursive comparison had passed where
applicable, and a fresh read-only artifact inspection corrected only the
reporting expression. No required gate remains untested.

## Dormant waypoint warm-start option removal - 2026-09-01

The authoritative baseline was commit
`93a28e62efc4e81f97368928e7a3c1a747665ec4` in detached worktree
`tmp/bmtp-warmstart-baseline`; the candidate was branch
`bmtp-cleanup-codex`. Tracked state was clean before editing except for the
intentional untracked `bmtp_cleanup_handoff.md`. Every MATLAB command used a
fresh private `MATLAB_PREFDIR`, `TEMP`, and `TMP`, and maintained examples ran
one process at a time.

Read-only caller tracing established that the optional
`+obstacleAvoidance/+planner/ruckigWarmStart.m` file is absent and no planner
or engine reads `WaypointWarmStartMode`,
`RequestedWaypointWarmStartMode`, or `IsWaypointWarmStartAvailable`. The
separate restart input/output on `trajectory/planTrajBmtp.m` and
`trajectory/+bmtpEngine/solve.m` is documented and directly tested, so it was
excluded from the change. The accepted edit removes 29 and adds 13 lines in
`resolvePlannerOptions.m` (net -16), adds six net production lines to
`examples/resolveExampleOptions.m` for one-release forwarding, and therefore
removes ten production lines overall. The two restart files have zero diff.

Focused verification completed as follows:

- pre-change `testPlannerOptions`: 4/4 passed; defaults contained 24 fields;
  explicit `passThrough` resolved to `none`, echoed requested mode and
  unavailable state, and warned `planTrajectory:RuckigWarmStartUnavailable`;
- post-change `testPlannerOptions`: 5/5 passed;
- `testExampleInvariants`: 9/9 passed;
- `testObstacleAvoidanceSandboxDiagnosis`: 11/11 passed;
- `testBmtpEngine`: 12/12 passed, including public restart production/reuse;
- MATLAB Code Analyzer: zero findings for `resolvePlannerOptions.m` and
  `resolveExampleOptions.m`;
- `git diff --check`: passed and no modified line exceeded 100 characters.

Four repeated obstacle-free runs replayed the deprecated option through the
public example boundary. Baseline and candidate success, termination,
selection, histories, validation, and non-runtime diagnostics matched at
`1e-9` after allowing only the declared removal of the three option fields.
Warmed medians were 0.0777916 s baseline and 0.0835580 s candidate (+7.413%).
An unsuppressed end-to-end call emitted exactly
`planTrajectory:DeprecatedWaypointWarmStartOptions`, not an unknown-field
warning, and returned 4.472135955 deg polyline/smoothed length and
4.53112887415 s duration with collision and kinematic validation passing.

The structurally different moving/deforming example passed with
40.2805670061 deg polyline/smoothed length and 7.91666666667 s duration. The
full serial headless matrix then ran all 17 maintained examples: 16 succeeded
and independently validated, and `exampleNoPath` returned the expected
validated `noValidatedSeed` failure. Exact metrics are in `benchmark.csv`.
The hidden no-path plot created one figure titled
`Azimuth/elevation motion plan | noValidatedSeed | seeds 1 | expanded 1 |
rejected 2`. The visible obstacle-free run created two figures, both visible.
The complete test tree passed 113/113 with zero failed or incomplete,
85.8302862 s aggregate duration and 93.3293115 s wall time.

One default `exampleStraightTargetAlternatingOcclusion` candidate run selected
seed 5 while baseline and a candidate repeat selected seed 1. Saved summaries
localized the difference to the existing wall-clock work budget: seed 5
finished in the first candidate run but returned
`seedWorkBudgetExhausted` in the others. Arrival and validation were identical;
the seed-5 final motion was shorter (13.5713266002 deg versus
13.5986641387 deg). A controlled baseline/candidate comparison with the
existing `PerSeedWorkBudgetMultiplier=100` removed that cutoff variability.
Both selected seed 5 with identical 27.950433436 deg polyline,
13.5713266002 deg final motion and 20.8695652174 s duration. Recursive
non-runtime result comparison passed; walls were 24.8525406 s and
25.1338125 s (+1.132%). No planner setting was changed in production.

Five setup/inspection failures produced no accepted planner evidence: the
first candidate repeated-run command lost the comparator path after the runner
reset MATLAB's path; the first comparator omitted
`ElapsedPlanningTime_s` from its runtime exclusions; two inline commands were
truncated by shell quoting before MATLAB executed; and the first seed-summary
inspection queried a nonexistent top-level `SelectedSeedSource` field. Each
was corrected in a fresh process. A post-result MATLAB connector shutdown
exception occurred once after evidence was saved and the process still exited
zero. No required gate remains untested for this option-only milestone.

## BMTP immutable-SOCP-cache removal - 2026-09-01

The frozen comparison baseline was
`5c0a6c97bf68e9db03ace5281bda2e0f84243a8c` in detached worktree
`tmp/bmtp-cleanup-baseline`; the candidate was branch
`bmtp-cleanup-codex`. Every MATLAB invocation used a new private
`MATLAB_PREFDIR`, `TEMP`, and `TMP`, and only one example process ran at a
time.

The focused comparison called `exampleTargetExitsObstacle` once for warmup and
three timed repetitions in each worktree with
`CollisionClearanceTolerance_deg=1e-4`, plane reuse enabled, two seeds, and all
plots/animation disabled. A separate MATLAB process compared the saved records
recursively while excluding runtime-only fields. All four pairs matched in
planner success, independent validation, termination, selected seed/source,
certificate decisions, non-runtime search/solver diagnostics, and sampled
time, position, velocity, acceleration, and jerk. Maximum sampled numerical
difference was exactly zero against the `1e-9` gate. Arrival was 24 s, selected
polyline length was 21.7425467317 deg, and smoothed motion length was
21.9416287312 deg. Warmed medians were 10.7475989 s baseline and 11.7573298 s
candidate, a 9.395% candidate increase within the declared 25% limit.

Focused and full test commands completed as follows:

- `runtests('tests/testBmtpEngine.m')`: 12/12 passed in 3.4384 s;
- `runtests('tests/testPlannerContract.m')`: 15/15 passed in 48.646 s;
- `runtests('tests')`: 111/111 passed, zero failed or incomplete,
  94.1434 s aggregate test duration and 106.8387038 s wall time;
- MATLAB Code Analyzer: zero findings for both candidate and baseline
  `trajectory/+bmtpEngine/solve.m`;
- `git diff --check`: passed.

All 17 maintained examples ran in separate serial headless MATLAB processes
with jerk constrained. Sixteen returned planner success and independent
example-validation success; `exampleNoPath` returned the expected
`noValidatedSeed`, no validation warning, and documented empty trajectory
metrics. Every successful motion was collision-free and passed continuous
velocity, acceleration, jerk, and dynamics checks. Exact per-example lengths,
durations, certificate status, and wall times are recorded in `benchmark.csv`.
A second hidden `exampleNoPath` run created one search diagnostic figure titled
with `noValidatedSeed`, `seeds 1`, `expanded 1`, and `rejected 2`. A visible
`exampleObstacleFree` run passed and created two figures with `Visible="on"`.

Three setup failures produced no planner evidence and were corrected before
the recorded runs: MATLAB initially reported `System Error: File system
inconsistency` until the user-authorized MathWorks support processes were
stopped; the first temporary baseline harness lacked its primary-function
`end`; the next omitted the separate `trajectory` package path; and the first
visible smoke command had a shell-quoting parse error. Each corrected run used
a new private MATLAB directory. No required gate remains untested for this
cache-only milestone. The non-jerk motion configuration was not rerun because
this change does not alter motion-profile construction or constraint meaning;
the full test tree still exercised both BMTP and Ruckig owners.

## Balanced travel-time and route-realization correction - 2026-08-31

Baseline commit was `25757dcad64bff0ff57423132691109de94181e3` on
`novel-rep`. All MATLAB commands added the repository and `trajectory` package
to the path. Code Analyzer returned zero messages for every changed MATLAB
production, example, sandbox, and test file. `git diff --check` returned no
whitespace errors (Git emitted only the repository's CRLF conversion warnings).

Focused suites passed after the implementation:

- `testPlannerOptions`: 4/4;
- `testBmtpEngine`: 11/11;
- `testStaticPlanningProjection`: 2/2, including a structurally different
  translating-rectangle projection and a lower-route no-overshoot assertion;
- `testPlannerContract`: 14/14;
- `testTimedBmtpPlanning`: 2/2;
- `testRuckigWaypointMotion`: 2/2;
- `testUnsupportedTimedTopologyPolicy`: 2/2;
- `testObstacleAvoidanceSandboxDiagnosis`: 11/11.

The first complete `runtests('tests')` invocation took 113.599 s and passed
107/108. The single failure was
`testTimedOpeningLowerBoundPassesAndPolicyRejects`: its certificate fixture had
relied on the former implicit goal-time default even though the certificate is
defined only for strict earliest arrival. The fixture now explicitly declares
`earliestArrival`; its focused suite passes 3/3. After all planner, UI, example,
test, and record edits, the final complete-suite rerun passed 108/108 with zero
incomplete tests in 111.388 s.

Supplied rogue cases, each independently validated:

| case | policy | selected seed (deg) | motion (deg) | arrival (s) | result |
| --- | --- | ---: | ---: | ---: | --- |
| hidden Ruckig fallback | earliest | 248.239063 | 233.911502 | 104.261457 | BMTP success; six-segment Ruckig rejected before execution |
| sine trajectory | earliest fixed clock | 146.976783 | 146.976783 | 70.344251 | one-sided family beats 153.472521-degree progress family |
| static shrimp | balanced, 1 deg/s | 175.168391 | 175.780063 | 82.389498 | maximum elevation 39.288753 deg |
| non-ideal moving circle | balanced, 1 deg/s | 227.905578 | 228.491135 | 144 | cost 372.491135 beats fast-detour cost 379.503102 |

Every maintained example was run in a separate headless MATLAB process with
jerk enabled, plots and animation disabled, and its metrics written to
`benchmark.csv`. Sixteen returned independently validated successes. The sole
expected failure, `exampleNoPath`, returned `noValidatedSeed`; a separate hidden
plot run created one figure, one axes, four line objects, and three text objects.
A separate visible `exampleObstacleAvoidance` run created one visible figure
and independently validated. The alternating-occlusion example now explicitly
uses BMTP because its route exceeds the intentional two-segment Ruckig limit.

## Fixed-clock route-economy refinement - 2026-08-31

The fixed-clock lateral excursion now refines its coarse failing/passing
amplitude bracket with the full public trajectory validator. The direct
physical clock, obstacle geometry, safety margin, motion limits, and final
acceptance rules are unchanged. On the centered protected-circle gate, travel
fell from 16.822181 to 16.700092 degrees while the validated 7.333333-second
arrival remained unchanged. The retained path is within one percent of the
16.638-degree tangent-and-arc geometric lower bound.

The route-economy suite passed 3/3 for a circle, irregular static concave
outline, and the same outline moving across the route. The focused planner and
sandbox selection passed 38/38. All 17 maintained examples were run headlessly
and recorded in `benchmark.csv`; all 16 expected-success examples passed
independent collision and kinematic validation, and `exampleNoPath` returned
the expected `noValidatedSeed`. A visible `exampleObstacleAvoidance` run
created two figures and passed validation.

The improvement adds full validation calls to the fixed-clock boundary search.
The centered-circle focused run took 3.123552 seconds, but no identically
instrumented pre-change runtime was retained, so no runtime ratio is claimed.
The static-U maintained example still expands from a 34.942588-degree seed to
a 40.255029-degree smooth motion; this change does not claim to solve that
separate multi-waypoint smoothing inefficiency.

Current worktree evidence is summarized in
[BMTP immutable SOCP cache - 2026-08-30](#bmtp-immutable-socp-cache---2026-08-30).
Earlier sections are retained as historical checkpoints.

## BMTP immutable SOCP cache - 2026-08-30

The retained change caches only the per-seed trajectory-SOCP data that the
profile showed was rebuilt unchanged: derivative, endpoint, and continuity
matrices; equality right-hand side; objective; bound template; and
second-order cones. Plane-dependent rows and the active horizon bound are
reconstructed on every alternation in the original order. No solver
tolerance, acceptance rule, iteration cap, restart policy, or warm start
changed.

### Production timing

The production comparison excluded the temporary call ledger. MATLAB R2024b
Update 4 used a separate isolated `MATLAB_PREFDIR` for each snapshot. Each
snapshot received one discarded warm-up and three recorded repetitions in
one session. The two Target modes were interleaved A/B, B/A, A/B.

| Case and mode | Baseline raw times (s) | Baseline min / median (s) | Candidate raw times (s) | Candidate min / median (s) |
| --- | --- | ---: | --- | ---: |
| Target Exits, maintained BMTP `1e-7 deg` | 15.8698495, 15.6846937, 15.7403272 | 15.6846937 / 15.7403272 | 14.2601636, 13.4320352, 14.1946827 | 13.4320352 / 14.1946827 |
| Target Exits, diagnostic BMTP `1e-4 deg` | 41.7338001, 42.8482502, 41.2565451 | 41.2565451 / 41.7338001 | 35.3456280, 35.3323942, 35.3340565 | 35.3323942 / 35.3340565 |
| Straight Target, maintained Ruckig waypoint | 4.4833886, 4.2972902, 4.3850744 | 4.2972902 / 4.3850744 | 4.6562452, 4.5019727, 4.1759325 | 4.1759325 / 4.5019727 |
| Straight Target, diagnostic BMTP override | 18.2344675, 19.5057861, 18.3498866 | 18.2344675 / 18.3498866 | 17.7112745, 18.0195969, 19.1323885 | 17.7112745 / 18.0195969 |

The maintained Target Exits median improved by 9.82%; the `1e-4 deg`
diagnostic median improved by 15.34%. The maintained Straight Target control
still makes zero BMTP and zero `coneprog` calls; its 2.67% median movement in
the unfavorable direction is wall variance, not a cache result. The explicit
BMTP override improved by 1.80% at the median.

A separate observational profile measured
`solve>solveTrajectorySocp` self time falling from
`2.882206656 s / 17 calls` to `0.394152606 s / 17 calls` for maintained
Target Exits, and from `7.664537344 s / 42 calls` to
`0.448020402 s / 42 calls` at `1e-4 deg`. The corresponding complete Target
call structures stayed at 17 outer iterations / 138 conic calls and
42 / 433. This locates the retained benefit in removal of immutable
MATLAB-side reconstruction, not in a looser conic problem.

### Exact formulation and answer checks

A temporary same-call oracle constructed the legacy and cached trajectory
formulations immediately before each trajectory `coneprog` call. It compared
the complete objective, cones, sparse inequality and equality matrices,
right-hand sides, lower bounds, and horizon-adjusted upper bounds exactly.
It passed `testBmtpEngine`, all 17 trajectory formulations in the maintained
Target 138-call run, all 42 trajectory formulations in the `1e-4 deg`
433-call run, and the explicit Straight BMTP path including horizon expansion.
The unchanged maximum-margin-plane solver was outside the cache and oracle.
The oracle was removed before production timing and commit.

The instrumented Target sequences also retained exact per-seed structure:
default outer counts `1 / 5 / 11` and conic-call counts `2 / 45 / 91`;
`1e-4 deg` outer counts `1 / 35 / 6` and calls `2 / 385 / 46`. All exit flags
were `+1`, and neither Target mode used a horizon expansion or caller restart.

A detached archive of committed baseline `747f46c` and the candidate were
then run in separate MATLAB processes for all 17 maintained examples. Every
example used jerk constraints and passed its independent example validation.
The maximum candidate-minus-baseline arrival, selected-polyline, and
smoothed-path deltas were each exactly zero, within the required `1e-9`.
Success states, termination reasons, selected seeds, and aggregate BMTP
outer-, trajectory-SOCP-, and plane-SOCP counts were also identical. Exact
candidate metrics and walls are recorded in `benchmark.csv`.

The explicit Straight BMTP timing diagnostic is subject to its wall-clock
per-seed budget, so its observed outer and conic call counts varied between
runs as they did before this change. Its arrival and both path lengths stayed
exact. The maintained Straight example is the zero-BMTP Ruckig row and is not
subject to that caveat.

### Verification and remaining limits

Final MATLAB Code Analyzer output contained zero findings in `solve.m`.
Focused suites passed 31/31 tests: `testBmtpEngine`, `testPlannerContract`,
`testPlannerStageTiming`, and `testArchitectureBoundaries`. A visible
`exampleObstacleFree` run created two figures with five axes. A hidden
expected `exampleNoPath` run passed its failure validation and created one
search-diagnostic figure with one axis. A zero-second BMTP budget returned
`WorkLimitReached = true`, made zero trajectory or plane conic calls, and
returned the stable `noOptimizedFeasibleIterate` failure.

The required staged gate ran at the supplied path in one MATLAB invocation
with an isolated `MATLAB_PREFDIR`. Every stage passed, including the Rogue
sentinel and the complete 94/94 test suite, in about 101 seconds of observed
tool wall time.

The cache does not cure the `1e-4 deg` seed-2 oscillation: that solve still
reaches the 35-outer-iteration cap. It only makes each immutable reconstruction
cheaper. Counted production increased from 11,975 to 12,015 lines. The 4,515
line excess over 7,500 gives the existing allowance formula
`0.25 * 4515 / 100 = 11.2875`, or 1,128.75%; no size-compliance claim is made.
`solve.m` is 1,065 physical lines, up from 1,026, and 936 noncomment lines, so
it remains above the 900-line target. Its patch contains more than 50 added
lines because the original inline assembly was partitioned into immutable
template construction and dynamic row assembly; the net physical growth is
39 lines. A separate helper would broaden the internal source surface without
reducing total production size, so the one-call-site invariant remains local.
The exact-formulation oracle, 31 focused tests, 17 detached comparisons, and
94-test gate are the retained checks for that larger edit.

## BMTP conic profiler baseline - 2026-08-30

Source was `novel-rep` at `cf6733d` plus a temporary observational ledger in
`trajectory/+bmtpEngine/solve.m`. The ledger timed calls and retained exit
flags; it did not change a matrix, solver option, tolerance, stopping rule,
candidate, or selection decision. It is excluded from the measurement commit.

MATLAB R2024b Update 4 ran in one process with a fresh `MATLAB_PREFDIR`.
Plots, animation, and kinematic figures were disabled. Each case received one
discarded warm-up, then three recorded repetitions. Default and `1e-4 deg`
Target Exits runs were interleaved A/B, B/A, A/B. Profiled runs were separate
because profiler overhead is not a wall-time baseline.

| Case and mode | Raw wall times (s) | Min / median (s) | BMTP solves / outer iterations / `coneprog` calls | `coneprog` min / median total (s) |
| --- | --- | ---: | ---: | ---: |
| Target Exits, maintained BMTP `1e-7 deg` | 19.2216859, 17.6795313, 17.6197381 | 17.6197381 / 17.6795313 | 3 / 17 / 138 on every run | 9.9852706 / 10.0226759 |
| Target Exits, diagnostic BMTP `1e-4 deg` | 49.1185155, 48.2423766, 51.4278263 | 48.2423766 / 49.1185155 | 3 / 42 / 433 on every run | 33.4553216 / 34.7197311 |
| Straight Target, maintained Ruckig waypoint | 5.6869915, 5.2936647, 5.4580266 | 5.2936647 / 5.4580266 | 0 / 0 / 0 on every run | 0 / 0 |
| Straight Target, diagnostic BMTP override | 21.2219000, 20.9824146, 21.2279523 | 20.9824146 / 21.2219000 | 5 / 31 / 195 on every measured run | 14.4342698 / 14.5378415 |

The maintained Straight Target example explicitly uses
`TrajectoryMethod="ruckigWaypoint"`. Its reported 5.5--6 second runtime is not
inside `bmtpEngine.solve`; all three measured runs and its profile contained
zero BMTP solves and zero `coneprog` calls. The BMTP row is an explicitly
labeled diagnostic override, not a substitute maintained result.

Every recorded result passed planner and independent validation. Exact result
metrics were stable within each mode:

| Case and mode | Arrival (s) | Selected polyline (deg) | Smoothed path (deg) |
| --- | ---: | ---: | ---: |
| Target Exits, `1e-7 deg` | 24 | 32.5940497846889 | 22.5540060420224 |
| Target Exits, `1e-4 deg` | 24 | 32.5940497846889 | 22.5551638893269 |
| Straight Target, maintained Ruckig waypoint | 20.8695652173913 | 20.7720160748463 | 20.7720160748463 |
| Straight Target, BMTP override | 20.8695652173913 | 20.7720160748463 | 13.9954554403792 |

### Decisive clearance comparison

The median-wall representatives prove the slow mode takes a longer bounded
alternation path; it is not a caller restart, a horizon retry, or one unusually
slow `coneprog` call.

| Target Exits seed | Default outer / calls / conic s | `1e-4` outer / calls / conic s | First collision-free iteration | Horizon expansions | Exit flags |
| --- | ---: | ---: | ---: | ---: | --- |
| 1, blocked direct edge | 1 / 2 / 0.0671718 | 1 / 2 / 0.0784015 | unavailable | 0 / 0 | all `+1` |
| 2, visibility route | 5 / 45 / 3.8770388 | 35 / 385 / 31.1791465 | 2 / 2 | 0 / 0 | all `+1` |
| 3, visibility route | 11 / 91 / 6.0410600 | 6 / 46 / 3.4621831 | 2 / 2 | 0 / 0 | all `+1` |

Seed 2 is the entire adverse mechanism. At default clearance, its feasible
trial durations were
`15.2541832934, 13.0243197240, 12.9792492794, 12.9785763379 s` and the fifth
outer iteration met the existing improvement tolerance. At `1e-4 deg`, it
remained collision-free from iteration 2 onward but its duration oscillated
between `12.9627946391` and `12.9825425646 s` after the initial descent. The
one-sided consecutive-improvement test never fired, so the solve reached the
hard 35-iteration cap. That added 30 trajectory calls and 310 maximum-margin
plane calls. There is no caller restart loop; the third `restart` output is
ignored by the adapter.

Representative seed-2 per-iteration wall times were:

- default: `1.0539504, 1.0535098, 1.0472976, 0.9020167, 1.0033996 s`;
- `1e-4`: `1.1079259, 1.0703438, 1.0944885, 1.0700774,
  1.0324632, 1.1893641, 1.0355548, 1.1546870, 1.2635565,
  1.1751728, 1.1279355, 1.0952053, 1.1217369, 1.2456730,
  1.1184008, 1.1314867, 1.0896716, 1.2302039, 1.1178210,
  1.1632448, 1.0601267, 1.1149967, 1.2208972, 1.1131718,
  1.0875190, 1.0931486, 1.1452155, 1.1337634, 1.2251764,
  1.1707898, 1.0737866, 1.2017295, 1.1583726, 1.0717253,
  1.0363081 s`.

All 35 slow seed-2 trajectory calls exited `+1`; their per-call times ranged
from `0.6240110` to `0.8759522 s` with `0.7168136 s` median. Its 350 plane
calls all exited `+1`, ranged from `0.0110804` to `0.0297394 s`, and had
`0.01594915 s` median. The default seed-2 trajectory and plane medians were
`0.6687750 s` and `0.01448765 s`. Individual calls were therefore only
modestly slower; the 8.56-times call-count increase dominates the 2.78-times
wall increase.

### MATLAB profiler self time inside `trajectory/+bmtpEngine`

| Target Exits function | Default self / calls (s) | `1e-4` self / calls (s) |
| --- | ---: | ---: |
| `solve>solveTrajectorySocp` | 2.882206656 / 17 | 7.664537344 / 42 |
| `solve>evaluateBezier` | 0.596560112 / 402 | 1.228873507 / 1017 |
| `createCoordinateTolerances>updateScale` | 0.506126410 / 102858 | 0.516137303 / 103668 |
| `createCoordinateTolerances` | 0.237270705 / 25761 | 0.246332501 / 26031 |
| `solve>controlIndexOf` | 0.147970703 / 41536 | 0.318927702 / 115466 |
| `solve` | 0.068327601 / 3 | 0.079272500 / 3 |
| `solve>verifyPlane` | 0.053696001 / 301 | 0.107748601 / 571 |
| `solve>findSampledCollisionPairs` | 0.042231701 / 20 | 0.060788000 / 45 |
| `solve>fixedPlaneRows` | 0.038245901 / 120 | 0.090562401 / 380 |
| `solve>solveMaximumMarginPlane` | 0.028016301 / 121 | 0.071504100 / 391 |

The default profile made 138 conic calls totaling `11.6722840 s`; the `1e-4`
profile made 433 totaling `32.7149386 s`. `solveTrajectorySocp` self time is
the largest engine-owned cost outside `coneprog` and scales with rebuilding
the same endpoint, continuity, derivative, bound, and cone structures on each
outer iteration. This supports one bounded behavior-preserving experiment:
reuse only that immutable per-seed SOCP template while rebuilding the
plane-dependent rows and horizon bounds exactly as before. The change must be
reverted if any maintained-example arrival or either path length moves by more
than `1e-9`, if solver call/exit structure changes, or if warmed repeated wall
time does not improve without a regression in the representative BMTP cases.

Measurement verification before the profile session: MATLAB Code Analyzer
reported zero findings in the temporary ledger and harness, and
`testBmtpEngine` passed 7/7. The measurement commit changes no production
code, so the recorded production size remains 11,975 counted lines, 4,475
above 7,500. The unchanged allowance formula is
`0.25 * 4475 / 100 = 11.1875`, or 1,118.75%; this measurement milestone makes
no size-compliance claim.

## Ruckig-to-BMTP collision gate, step 2 — 2026-08-30

- The standalone certificate first tries an exact constant separator, then
  uses BMTP's degree-one maximum-margin conic form and directly replays the
  validator's Bernstein-product, clearance, roundoff, and normal-norm checks.
- The final focused run passed `testBmtpWarmStartConversion` 5/5 and
  `testArchitectureBoundaries` 9/9. Code Analyzer reported zero findings in
  the certifier and both changed tests. The final code edit also passed
  `gate(1)` with unchanged Two-U, README quick-start, and no-path sentinels.
- The decisive census used maintained static cases and route candidates. The
  intermediate Ruckig request used earliest arrival with a 3,600-second upper
  horizon so the final request horizon did not confound geometric conversion.
  Every curve in the denominator independently passed the public validator
  against the original static obstacles before conversion.
- Eight curves converted. Per-curve maximum errors and certificate results:
  Obstacle Avoidance seeds 2 and 3, `1.74313715049989e-5 deg`, both failed;
  Static U seeds 2 and 3, `7.85168079719939e-5 deg`, both failed; Target Exits
  seed 2, `1.34309047285578e-5 deg`, passed; Target Exits seed 3,
  `5.68171630073286e-5 deg`, failed; Alternating Slalom seed 2,
  `9.50061068454221e-6 deg`, failed; Dense Concave seed 2,
  `9.59122759523563e-6 deg`, failed.
- The collision-certificate fraction was therefore `1/8 = 12.5%`, below the
  predeclared one-third kill threshold with the required denominator of at
  least six. The experiment stopped before planner wiring. No outer-iteration
  or warm/cold timing result was run.
- A constant-plane-only preliminary census produced 0/8 but was retracted
  because it was more restrictive than the specified degree-one check. The
  recorded 1/8 result is the corrected measurement.
- The five cold maintained-example runs used jerk constraints and all passed
  independent validation, collision, and kinematic checks. Their exact result
  rows are appended to `benchmark.csv`; they are not warm-start benchmarks.
- The certifier contains 382 physical lines and 323 counted production lines.
  Experimental production is now 11,975 counted lines, 4,475 above the
  literal 7,500 target. The allowance formula is
  `0.25 * 4475 / 100 = 11.1875`, or 1,118.75%. The kill result provides no
  runtime reduction and makes no size-compliance claim.

## Ruckig-to-BMTP warm-start conversion, step 1 — 2026-08-30

- Source before the experiment: `novel-rep` at `e842356`.
- The converter is standalone and has no planner caller. It uses a scalar
  equal-span time, Chebyshev-Lobatto least-squares fits, and a denser error
  grid augmented with source switching times.
- The final `gate(1)` run retained Two-U at `21.6333333333333 s`, the README
  quick start at `7.5745417663213 s`, and `exampleNoPath` as
  `noValidatedSeed`.
- `testBmtpWarmStartConversion` passed 2/2 and
  `testArchitectureBoundaries` passed 9/9. MATLAB Code Analyzer reported zero
  findings in the converter, its test, and the updated architecture test.
- One six-phase Ruckig fixture measured a `4.96506830649455e-15 deg`
  single-phase-span maximum at degree 7. The same degree with two uniform
  multi-break spans measured `0.000534264339868523 deg`; degree 16 measured
  `3.52475369970282e-5 deg`. No unmeasured error bound is asserted.
- `auditProductionSize(20000)` reported 74 files and 11,652 counted lines.
  Removing the new converter's 128 counted lines reproduces the exact 11,524
  branch baseline. Against the literal 7,500-line target, the current excess
  is 4,152 lines and the allowance formula is
  `0.25 * 4152 / 100 = 10.38`, or 1,038%. No runtime reduction has yet been
  measured, so this experimental commit does not claim size compliance.
- Two sandboxed MATLAB startups failed before loading code with the documented
  filesystem-inconsistency error despite isolated preferences. Fresh isolated
  preference directories outside the sandbox completed every result above.
- `benchmark.csv` was not changed because the converter is not planner-wired
  and no warm-start planner benchmark was executed.

## Certified multi-axis direct progress — 2026-08-27

- Source: `HS3-planner` at `e72957c+direct-progress-worktree`.
- Only `hs3/solveTrajHS3.m` and its standalone tests changed. Existing
  user/Claude line-ending changes and untracked files were not staged.
- Code Analyzer reported zero findings for `solveTrajHS3.m`.
- Focused standalone HS3 tests passed 17/17. The complete warnings-enabled
  repository suite passed 131/131, with zero failures or incomplete tests, in
  79.596835 seconds wall and 69.4810232 seconds summed duration. Existing
  singular-matrix warnings remained confined to unrestricted fallback cases.
- Exact pre-change and candidate measurements used MATLAB R2024b, the same
  segment count, sample time, inputs, limits, and current machine. The
  three-case warmed median changed from 1.6038312 to 0.5261287 seconds.

| Neutral case | Baseline / candidate arrival | Convex boundary | Baseline / candidate path excess | Baseline / candidate wall s |
| --- | ---: | ---: | ---: | ---: |
| balanced 3-D | 3.96604572064 / 3.96604532423 | 3.96605623245 | 0.00364763382 / -6.81e-9 | 3.6527880 / 3.1921851 |
| anisotropic 3-D | 4.94413658256 / 4.94413624303 | 4.94417476654 | 0.00495067325 / -5.27e-9 | 0.8584585 / 0.5261287 |
| anisotropic 4-D | 6.35746001936 / 6.35745949091 | 6.35746944809 | 0.00239304950 / -4.86e-9 | 1.6038312 / 0.4151296 |
| moving target 3-D fixed sweep | 4.09937101364 / 4.09937101364 | 4.09937101364 | 0.000188808074 / 1.78e-15 | 0.2210358 / 0.2851910 |

Every maintained example ran headlessly in a separate fresh MATLAB process.
Jerk constraints were enabled in every row.

| Example | Planner / validation | Polyline deg | Smoothed deg | Duration s | Collision / kinematic | Wall s | Termination |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0193197983 | 16.8275815277 | 11.1855739607 | 1 / 1 | 7.9936153 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7007215595 | 13.9293484742 | 8.64603162385 | 1 / 1 | 6.4177555 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 | 5.7958533 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 | 2.7723482 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.31007759339 | 7.31051404817 | 6.11702719116 | 1 / 1 | 7.6548894 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.2314453125 | 1 / 1 | 20.3673764 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12 | 12.7689032232 | 8.64603156476 | 1 / 1 | 10.1046380 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40 | 43.0751355347 | 7.96286899667 | 1 / 1 | 180.1148022 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN | 1.3979389 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4464747617 | 7.57952069664 | 1 / 1 | 5.5272338 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.5458984375 | 1 / 1 | 3.2776348 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10 | 11.8560791016 | 1 / 1 | 47.7892301 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 41.5363500661 | 22.6308876389 | 1 / 1 | 16.6033839 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 21.4031702791 | 13.678271908 | 20.8695652174 | 1 / 1 | 6.5797566 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.5244479986 | 20.6764423274 | 24 | 1 / 1 | 7.8621671 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.035784715 | 24.4189853364 | 21.9090835611 | 1 / 1 | 8.2008795 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.2394635087 | 24.6064786878 | 6.3679977362 | 1 / 1 | 50.3206416 | `goalReached` |

A visible successful run created three figures with six axes. A visible
expected failure created two diagnostic figures with two axes and retained
`noValidatedSeed`. No pre-existing MATLAB process was stopped or signaled.

## Unified spatial and timed seed equivalence — 2026-08-27

- Source: `HS3-planner` at `5cf2d87+seed-equivalence-worktree`.
- `temporalSeedDuplicates` and `routeDuplicates` are replaced by one local
  `seedDuplicates` helper. Spatial comparison receives an empty duration;
  timed comparison retains the exact `1e-9` relative duration tolerance.
- Exact detached-baseline comparisons on a static concave U and moving barrier
  found complete seeds and diagnostics `isequaln` in both cases.
- Static route-search median changed from 0.0536149 to 0.0545641 seconds
  (-1.77%). Moving route-search median changed from 0.6353544 to 0.6119082
  seconds (+3.69%). Both miss the 5% gate; no runtime improvement is claimed.
- Eight focused route-search tests passed 8/8 with zero Code Analyzer findings.
  The complete repository suite passed 128/128 with zero failures or incomplete
  tests in 80.343082 seconds. Known `fmincon` conditioning warnings remained
  visible during the complete suite.
- Every maintained example ran serially in its own fresh headless MATLAB
  process with jerk enabled. Sixteen successes and the expected
  `noValidatedSeed` failure passed independent validation. Collision and
  continuous kinematic certificates passed for every success. The 17-run wall
  sum was 349.9932267 seconds. The headless harness suppressed only MATLAB's
  near-singular and singular warning IDs to prevent repeated console stacks;
  planner decisions and validation were unchanged.
- Visible `exampleObstacleFree` produced three visible figures and six axes.
  Visible `exampleNoPath` produced two visible diagnostic figures and two axes.
- The file changes from 891 to 885 physical lines, 749 to 746 executable lines,
  97 to 96 comment lines, and 45 to 43 blank lines. The main McCabe value stays
  27 and `createVisibilityGraph` stays 18. The prior equivalence helpers totaled
  McCabe 7; the shared helper is 5.
- Production decreases from 11,682 to 11,676 lines; production plus tests
  decreases from 16,136 to 16,130. The unresolved 4,176-line overage requires
  `0.25 * 4176 / 100 = 10.44`, or 1044%, under the literal allowance formula.

Maintained headless example results:

| Example | Planner / validation | Polyline deg | Smoothed deg | Duration s | Collision / certificate | Wall s | Termination |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| exampleAlternatingSlalom | 1 / 1 | 16.0193197983 | 16.8275815277 | 11.1855739607 | 1 / 1 | 8.0186437 | goalReached |
| exampleDenseConcaveObstacle | 1 / 1 | 12.7007215595 | 13.9293484742 | 8.64603162385 | 1 / 1 | 5.5049454 | goalReached |
| exampleFourAcceleratingCircles | 1 / 1 | 20 | 20 | 22 | 1 / 1 | 4.3457098 | goalReached |
| exampleInterceptMovingTargetAtSetTime | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 | 2.2709324 | goalReached |
| exampleInterceptMovingTargetEarliest | 1 / 1 | 7.31007759339 | 7.31051404817 | 6.11702719116 | 1 / 1 | 6.5677632 | goalReached |
| exampleMovingBarrierWait | 1 / 1 | 10 | 10 | 10.2314453125 | 1 / 1 | 18.8965414 | goalReached |
| exampleMovingCircleNoAzimuthWrap | 1 / 1 | 12 | 12.7689032232 | 8.64603156476 | 1 / 1 | 8.7721664 | goalReached |
| exampleMovingDeformingUSOutlineVisibility | 1 / 1 | 40 | 43.0751355347 | 7.96286899667 | 1 / 1 | 163.8578868 | goalReached |
| exampleNoPath | 0 / 1 | NaN | NaN | NaN | NaN / NaN | 1.2503210 | noValidatedSeed |
| exampleObstacleAvoidance | 1 / 1 | 11.152119519 | 11.4464747617 | 7.57952069664 | 1 / 1 | 5.0406538 | goalReached |
| exampleObstacleFree | 1 / 1 | 4.472135955 | 4.472135955 | 4.5458984375 | 1 / 1 | 2.6825200 | goalReached |
| exampleOpeningUShapedObstacle | 1 / 1 | 10 | 10 | 11.8560791016 | 1 / 1 | 43.8988849 | goalReached |
| exampleStaticUShapedObstacle | 1 / 1 | 34.9425880405 | 41.5363500661 | 22.6308876389 | 1 / 1 | 13.3978099 | goalReached |
| exampleStraightTargetAlternatingOcclusion | 1 / 1 | 21.4031702791 | 13.678271908 | 20.8695652174 | 1 / 1 | 6.3422684 | goalReached |
| exampleTargetExitsObstacle | 1 / 1 | 20.5244479986 | 20.6764423274 | 24 | 1 / 1 | 6.7491033 | goalReached |
| exampleTwoOpposingUVisibilityGraph | 1 / 1 | 24.035784715 | 24.4189853364 | 21.9090835611 | 1 / 1 | 7.7847060 | goalReached |
| exampleUSOutlineExtremeVisibility | 1 / 1 | 22.2394635087 | 24.6064786878 | 6.3679977362 | 1 / 1 | 44.6123703 | goalReached |

## Route-candidate file-cap cleanup — 2026-08-27

- Source: `HS3-planner` at `01505f4+route-file-cap-worktree`.
- Repository-health scope removed 33 duplicate narrative-comment or blank lines
  from `createRouteCandidates.m`. A zero-context Git-diff audit found zero
  executable lines added, removed, or changed.
- The file decreased from 924 to 891 physical lines. Production decreased from
  11,715 to 11,682 lines, and no production file now exceeds the hard 900-line
  limit; `solveRouteCandidate.m` is the largest at 899 lines.
- The cleanup does not reduce McCabe complexity: the route-candidate owner
  remains 27 and its visibility-graph helper remains 18. Consolidating its two
  seed-equivalence implementations is a separate behavior-bearing candidate.
- Production remains 4,182 lines above target. The size formula is
  `0.25 * 4182 / 100 = 10.455`, requiring a 1045.5% reduction, and production
  plus tests remains 16,136 lines. The cleanup narrows but does not resolve the
  repository-wide size debt.
- MATLAB R2024b failed during process startup with `System Error: File system
  inconsistency` on four attempts, including isolated preferences and `-nojvm`.
  Existing MATLAB processes were left running. Code Analyzer and the planned
  eight route-search tests therefore did not execute after this comment-only
  cleanup; no example or performance claim is attached to it.

## Prepared dynamic boundary-edge queries — 2026-08-27

- Source: `HS3-planner` at `8059595`.
- Baseline: detached exact commit `750e9c7` under MATLAB R2024b with unchanged
  examples, planner options, obstacle geometry, limits, and validation.
- Retained mechanism: dynamic non-support corridor rows query one canonical
  prepared boundary edge. Support rows and final independent validation retain
  complete geometry. Stale/partial caches and unsafe extreme interpolation use
  the complete-boundary fallback.
- Exact parity: 178 direct queries across closed, query-time closure,
  multi-ring, topology-union, single-slice, exact-time, interpolated-time,
  inactive-time, and missing-edge cases matched the complete canonical oracle.
  A deforming-edge constraint test matched complete inequality, equality, and
  both gradient matrices exactly.
- Historical three-pair A/B evidence compares exact baseline `750e9c7` wall
  times of 179.5093499, 179.4877722, and 180.0643631 seconds with the initial
  fast-path candidate at 148.7280267, 147.6009931, and 148.5100961 seconds.
  That candidate's median reduction was 17.27%. The later safeguarded worktree
  ran once at 165.7597307 seconds in a different session. It retained exact
  route, arrival, solver, and validation evidence, but it was not paired with a
  contemporaneous baseline; the previously reported 7.66% comparison is
  therefore historical context, not a causal final-implementation claim.
- A post-push, counterbalanced comparison at exact parent `8059595` evaluated
  a proposed removal of repeated public-query validation. Parent wall times
  were 167.9463006 and 165.2065477 seconds; candidate times were 165.7019183
  and 163.1769446 seconds. Median wall time decreased only 1.28%, from
  166.57642415 to 164.43943145 seconds, and median reported planner time
  decreased 1.08%, from 139.30012795 to 137.80243725 seconds. Exact path,
  arrival, collision, kinematic, and validation outputs were unchanged. The
  candidate missed the declared 5% retention threshold and was reverted.
- A second counterbalanced experiment exposed sparse nonlinear Jacobians only
  at the neutral HS3 `fmincon` boundary. On Opening-U, parent wall times were
  43.0839846 and 43.8874203 seconds and candidate times were 42.6435272 and
  42.3384344 seconds. Median wall time decreased 2.29%, from 43.48570245 to
  42.4909808 seconds, and reported planner time decreased 2.07%. The exact
  19-iteration objective, residuals, route, arrival, collision, and validation
  evidence were unchanged. The candidate missed the 5% gate and was reverted.
- Limited-memory BFGS was rejected after one Opening-U run increased wall time
  to 69.5511834 seconds while preserving the same 19-iteration solution.
- Focused tests passed 21/21; the complete suite passed 128/128 in 93.8577066
  seconds. Code Analyzer reported zero findings across 87 maintained MATLAB
  files.
- Visible gates: `exampleObstacleFree` created three figures and six axes;
  `exampleNoPath` returned the expected failure and created two diagnostic
  figures with two axes.
- Production growth is 205 net lines and focused tests add 173 net lines. The
  production tree has 11,715 physical lines, 4,215 above the 7,500-line target.
  The size allowance formula is `0.25 * 4215 / 100 = 10.5375`, requiring a
  1053.75% reduction; no possible wall-time result can satisfy that literal
  gate. Production plus tests is 16,169 lines, above the independent 12,000-line
  limit. The retained commit therefore has unresolved size debt and is not
  justified by the repository's performance-based size allowance.

Every maintained example ran headlessly in its own fresh MATLAB process with
plots and animation disabled. Jerk was enabled in every case.

| Example | Planner / validation | Polyline deg | Smoothed deg | Duration s | Collision / certificate | Wall s | Termination |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| exampleAlternatingSlalom | 1 / 1 | 16.0193197983 | 16.8275815277 | 11.1855739607 | 1 / 1 | 7.5219106 | goalReached |
| exampleDenseConcaveObstacle | 1 / 1 | 12.7007215595 | 13.9293484742 | 8.64603162385 | 1 / 1 | 5.6663066 | goalReached |
| exampleFourAcceleratingCircles | 1 / 1 | 20 | 20 | 22 | 1 / 1 | 4.3017149 | goalReached |
| exampleInterceptMovingTargetAtSetTime | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 | 3.0141734 | goalReached |
| exampleInterceptMovingTargetEarliest | 1 / 1 | 7.31007759339 | 7.31051404817 | 6.11702719116 | 1 / 1 | 6.5378950 | goalReached |
| exampleMovingBarrierWait | 1 / 1 | 10 | 10 | 10.2314453125 | 1 / 1 | 18.1116314 | goalReached |
| exampleMovingCircleNoAzimuthWrap | 1 / 1 | 12 | 12.7689032232 | 8.64603156476 | 1 / 1 | 8.5957080 | goalReached |
| exampleMovingDeformingUSOutlineVisibility | 1 / 1 | 40 | 43.0751355347 | 7.96286899667 | 1 / 1 | 165.7597307 | goalReached |
| exampleNoPath | 0 / 1 | NaN | NaN | NaN | NaN / NaN | 1.3331914 | noValidatedSeed |
| exampleObstacleAvoidance | 1 / 1 | 11.152119519 | 11.4464747617 | 7.57952069664 | 1 / 1 | 5.1689199 | goalReached |
| exampleObstacleFree | 1 / 1 | 4.472135955 | 4.472135955 | 4.5458984375 | 1 / 1 | 2.8408890 | goalReached |
| exampleOpeningUShapedObstacle | 1 / 1 | 10 | 10 | 11.8560791016 | 1 / 1 | 42.2203932 | goalReached |
| exampleStaticUShapedObstacle | 1 / 1 | 34.9425880405 | 41.5363500661 | 22.6308876389 | 1 / 1 | 13.4359407 | goalReached |
| exampleStraightTargetAlternatingOcclusion | 1 / 1 | 21.4031702791 | 13.678271908 | 20.8695652174 | 1 / 1 | 6.3886567 | goalReached |
| exampleTargetExitsObstacle | 1 / 1 | 20.5244479986 | 20.6764423274 | 24 | 1 / 1 | 6.9468265 | goalReached |
| exampleTwoOpposingUVisibilityGraph | 1 / 1 | 24.035784715 | 24.4189853364 | 21.9090835611 | 1 / 1 | 7.3445507 | goalReached |
| exampleUSOutlineExtremeVisibility | 1 / 1 | 22.2394635087 | 24.6064786878 | 6.3679977362 | 1 / 1 | 44.5641943 | goalReached |

## Time-invariant obstacle geometry cache — 2026-08-27

- Source: `HS3-planner` at `81f2f8b+static-geometry-worktree`.
- Baseline: immutable archive of exact commit `81f2f8b` under the same MATLAB
  R2024b installation and unchanged example inputs, options, and validation.
- Retained mechanism: prepared obstacle records cache one complete shape and
  geometry record only when one slice or exact zero vertex speed proves that
  every active-time boundary is identical. Moving and topology-changing
  histories retain the original query path.
- Direct benchmark: 20,000 varying-time static queries decreased from
  0.9724214 to 0.4832129 seconds with checksum 100000 in both variants.
- Profiler evidence: 8,317 Static-U `shapeAtTime` calls decreased from
  0.7005765 to 0.4038666 seconds.
- Warmed, counterbalanced Static-U A/B: baseline runs were 10.6829540 and
  10.4528525 seconds; changed runs were 10.2648969 and 10.0535007 seconds.
  Median wall time decreased 3.87%, from 10.56790325 to 10.1591988 seconds.
  Success, validation, selected seed, sampled position, duration, and the
  complete polynomial were exactly equal.
- The moving-circle control retained its exact prior trajectory and reported
  `IsTimeInvariant=false`, proving that dynamic interpolation was not bypassed.
- Code Analyzer reported zero findings for the three changed MATLAB files.
  The focused obstacle suite passed 8/8 and the complete suite passed 119/119.
- A visible Static-U run created three figures. A hidden plotted expected
  no-path run created two diagnostic figures and two axes.
- Production growth is 29 net lines and retains one additional shape and
  geometry record per exactly static obstacle; focused tests add 12 net lines.

Every maintained example ran headlessly in its own fresh MATLAB process with
plots and animation disabled. Jerk was enabled in every case.

| Example | Planner / validation | Polyline deg | Smoothed deg | Duration s | Collision / certificate | Wall s | Termination |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| exampleAlternatingSlalom | 1 / 1 | 16.0193197983 | 16.8275815277 | 11.1855739607 | 1 / 1 | 6.2118336 | goalReached |
| exampleDenseConcaveObstacle | 1 / 1 | 12.7007215595 | 13.9293484742 | 8.64603162385 | 1 / 1 | 4.6918363 | goalReached |
| exampleFourAcceleratingCircles | 1 / 1 | 20 | 20 | 22 | 1 / 1 | 3.5876624 | goalReached |
| exampleInterceptMovingTargetAtSetTime | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 | 1.7641874 | goalReached |
| exampleInterceptMovingTargetEarliest | 1 / 1 | 7.31007759339 | 7.31051404817 | 6.11702719116 | 1 / 1 | 5.1296778 | goalReached |
| exampleMovingBarrierWait | 1 / 1 | 10 | 10 | 10.2314453125 | 1 / 1 | 16.1420977 | goalReached |
| exampleMovingCircleNoAzimuthWrap | 1 / 1 | 12 | 12.7689032232 | 8.64603156476 | 1 / 1 | 8.6384737 | goalReached |
| exampleMovingDeformingUSOutlineVisibility | 1 / 1 | 40 | 43.0751355347 | 7.96286899667 | 1 / 1 | 183.4952231 | goalReached |
| exampleNoPath | 0 / 1 | NaN | NaN | NaN | NaN / NaN | 1.0251391 | noValidatedSeed |
| exampleObstacleAvoidance | 1 / 1 | 11.152119519 | 11.4464747617 | 7.57952069664 | 1 / 1 | 4.1951049 | goalReached |
| exampleObstacleFree | 1 / 1 | 4.472135955 | 4.472135955 | 4.5458984375 | 1 / 1 | 2.2140933 | goalReached |
| exampleOpeningUShapedObstacle | 1 / 1 | 10 | 10.0912159691 | 11.8560791016 | 1 / 1 | 38.9154151 | goalReached |
| exampleStaticUShapedObstacle | 1 / 1 | 34.9425880405 | 41.5363500661 | 22.6308876389 | 1 / 1 | 11.5817246 | goalReached |
| exampleStraightTargetAlternatingOcclusion | 1 / 1 | 21.4031702791 | 13.678271908 | 20.8695652174 | 1 / 1 | 5.3855974 | goalReached |
| exampleTargetExitsObstacle | 1 / 1 | 20.5244479986 | 20.6764423274 | 24 | 1 / 1 | 5.4040393 | goalReached |
| exampleTwoOpposingUVisibilityGraph | 1 / 1 | 24.035784715 | 24.4189853364 | 21.9090835611 | 1 / 1 | 6.0280702 | goalReached |
| exampleUSOutlineExtremeVisibility | 1 / 1 | 22.2394635087 | 24.6064786878 | 6.3679977362 | 1 / 1 | 39.1583268 | goalReached |

## Dimension-neutral polynomial map caches — 2026-08-27

- Source: `HS3-planner` at `e1db1ed+hs3-map-cache-worktree`.
- Baseline: immutable archive of exact commit `e1db1ed` under the same MATLAB
  R2024b installation. No solver options, example inputs, geometry, limits, or
  validation policy changed.
- Retained mechanism: one-entry exact caches for duration-independent affine
  mesh structure and repeated subinterval Bernstein restriction maps. Both
  changes are inside `hs3Internal.polynomial` and remain independent of
  coordinate dimension and obstacle-planner semantics.
- Direct affine-map benchmark: 600 varying-duration calls at 40 segments and
  161 evaluation coordinates decreased from 0.6946311 to 0.4367565 seconds;
  the checksum remained 1301.1856152604162.
- Direct subinterval-map benchmark: 10,000 identical 40-interval calls at 40
  segments decreased from 1.5852113 to 0.1228665 seconds; the checksum remained
  20000.
- Warmed, counterbalanced Static-U A/B: baseline runs were 10.4545751 and
  10.3534417 seconds; changed runs were 10.3299481 and 10.3856388 seconds.
  Median wall time decreased 0.44%, from 10.4040084 to 10.35779345 seconds.
  Success, validation, selected seed, sampled position history, duration, and
  the complete polynomial were exactly equal.
- Code Analyzer reported zero findings for both changed production helpers.
- Focused polynomial and standalone HS3 suites passed 21/21. The complete
  suite passed 119/119 with zero failures or incomplete tests.
- A visible `exampleObstacleFree` run created three figures. A hidden plotted
  `exampleNoPath` run created two diagnostic figures and two axes, and its
  expected `noValidatedSeed` result passed independent failure validation.
- Production growth is 30 net lines across two helpers; focused tests add
  seven net lines. The full-planner gain is deliberately reported as small
  because nonlinear solver factorization remains dominant.

Every maintained example ran headlessly in its own fresh MATLAB process with
plots and animation disabled. Jerk was enabled in every case.

| Example | Planner / validation | Polyline deg | Smoothed deg | Duration s | Collision / certificate | Wall s | Termination |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| exampleAlternatingSlalom | 1 / 1 | 16.0193197983 | 16.8275815277 | 11.1855739607 | 1 / 1 | 6.4578554 | goalReached |
| exampleDenseConcaveObstacle | 1 / 1 | 12.7007215595 | 13.9293484742 | 8.64603162385 | 1 / 1 | 5.4847187 | goalReached |
| exampleFourAcceleratingCircles | 1 / 1 | 20 | 20 | 22 | 1 / 1 | 4.5913123 | goalReached |
| exampleInterceptMovingTargetAtSetTime | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 | 1.7342782 | goalReached |
| exampleInterceptMovingTargetEarliest | 1 / 1 | 7.31007759339 | 7.31051404817 | 6.11702719116 | 1 / 1 | 5.1683023 | goalReached |
| exampleMovingBarrierWait | 1 / 1 | 10 | 10 | 10.2314453125 | 1 / 1 | 16.327504 | goalReached |
| exampleMovingCircleNoAzimuthWrap | 1 / 1 | 12 | 12.7689032232 | 8.64603156476 | 1 / 1 | 8.8122815 | goalReached |
| exampleMovingDeformingUSOutlineVisibility | 1 / 1 | 40 | 43.0751355347 | 7.96286899667 | 1 / 1 | 183.3544747 | goalReached |
| exampleNoPath | 0 / 1 | NaN | NaN | NaN | NaN / NaN | 1.0338079 | noValidatedSeed |
| exampleObstacleAvoidance | 1 / 1 | 11.152119519 | 11.4464747617 | 7.57952069664 | 1 / 1 | 4.2712838 | goalReached |
| exampleObstacleFree | 1 / 1 | 4.472135955 | 4.472135955 | 4.5458984375 | 1 / 1 | 2.2288032 | goalReached |
| exampleOpeningUShapedObstacle | 1 / 1 | 10 | 10.0912159691 | 11.8560791016 | 1 / 1 | 39.2150586 | goalReached |
| exampleStaticUShapedObstacle | 1 / 1 | 34.9425880405 | 41.5363500661 | 22.6308876389 | 1 / 1 | 11.7281417 | goalReached |
| exampleStraightTargetAlternatingOcclusion | 1 / 1 | 21.4031702791 | 13.678271908 | 20.8695652174 | 1 / 1 | 5.6900266 | goalReached |
| exampleTargetExitsObstacle | 1 / 1 | 20.5244479986 | 20.6764423274 | 24 | 1 / 1 | 5.6179184 | goalReached |
| exampleTwoOpposingUVisibilityGraph | 1 / 1 | 24.035784715 | 24.4189853364 | 21.9090835611 | 1 / 1 | 6.1385721 | goalReached |
| exampleUSOutlineExtremeVisibility | 1 / 1 | 22.2394635087 | 24.6064786878 | 6.3679977362 | 1 / 1 | 39.8016126 | goalReached |

## Documentation and interactive-example cleanup — 2026-08-26

- Source: `HS3-planner` after `13c3553`.
- Removed eight tracked subfolder `README.md` files while retaining the root
  `README.md` as the single current guide.
- Removed `examples/exampleAzElInteractiveSandbox.m`; 18 maintained example
  functions remain.
- Removed the root guide's stale links and interactive-example requirement, and
  removed the now-dead sandbox exclusion from `testExampleRequirements`.
- `testExampleRequirements` passed 6/6 in 0.731870 seconds, its modified source
  had zero Code Analyzer findings, and `git diff --check` passed.
- No maintained example was executed for this documentation/manual-tool
  cleanup, so no row was appended to `benchmark.csv`.

## Same-homology spatial route cleanup — 2026-08-26

- Source: `HS3-planner` at `de372d5+spatial-route-cleanup-worktree`.
- Baseline state: commit `de372d5`, with only the user-owned untracked
  `Rogue Examples/` directory present. The baseline command called
  `azElSearch.generateTopologySeeds` directly with deterministic fixed-arrival
  inputs, so it measured spatial search without HS3 timing noise.
- Retained mechanism: after homology-augmented visibility search returns a
  spatial route, the cleanup evaluates nonadjacent route vertices as direct
  replacement edges. It accepts the largest reduction, repeats to a fixed
  point, and retains the original route unless the direct edge is visible in
  the protected swept geometry, the repository's winding signature is
  unchanged, and length decreases by more than the named floating-point
  tolerance. Direct seeds, timed routes, visibility search, and `hs3/` are
  unchanged.
- Primary rectangle gate: four searched classes decreased from an aggregate
  101.0883439161 to 101.0818433928 degrees, a 0.0065005233-degree reduction.
  Two shortcuts were accepted. Independent tests recomputed route lengths,
  winding signatures, and positive protected-geometry clearance.
- Structurally different curved-obstacle gate: four 16-sided protected circles
  accepted eight shortcuts and reduced aggregate class-route length by
  86.1926109665 degrees. The homology guard rejected 172 otherwise eligible
  direct replacements; every retained route independently matched its original
  searched signature and remained outside protected geometry.
- No-improvement gate: the existing two-rectangle homology case evaluated 62
  candidates, accepted none, and returned bit-identical route lengths.
- Diagnostics: search output now records cleanup route/candidate counts,
  visibility and homology rejections, accepted replacements, and total length
  reduction. Work scales with bounded spatial route point count and homology
  class count; the visibility graph remains capped at 96 candidate vertices.
- Tests: the focused three-case gate passed 3/3; `testHs3Planner` passed 59/59
  in 45.865558 seconds; the complete repository suite passed 114/114 in
  58.503675 seconds. Code Analyzer reported zero findings on both modified
  MATLAB files.
- Maintained examples: all 18 ran serially in separate fresh MATLAB processes.
  Seventeen independently validated successes passed collision and kinematic
  certificates, and the expected no-path case independently validated
  `noValidatedSeed`. `exampleAlternatingSlalom` shortened its selected spatial
  seed from 16.0604396350 to 16.0193197983 degrees;
  `exampleTwoOpposingUVisibilityGraph` shortened from 24.5077116377 to
  24.0357847150 degrees. Exact rows, including unchanged and unfavorable wall
  times, are appended to `benchmark.csv`.
- Graphics: a visible success produced three figures and six axes. The visible
  expected failure produced two diagnostic figures and two axes with 15
  rejected transitions.
- Known limitations: this is a deterministic local shortcut cleanup, not a
  globally shortest-path proof. Existing moving-barrier/opening-U singular
  solver warning floods remain, and the extreme-outline wall time measured
  61.034329 seconds versus the preceding 56.218460-second observation; no
  runtime improvement is claimed.

## Certified direct-path collinearity — 2026-08-26

- Source: `HS3-planner` at `69cef57+direct-collinearity-worktree`.
- Scope: Az/El eligibility, terminal/jerk constraint assembly, and tests only;
  no file under the dimension-neutral `hs3/` product changed.
- Primary baseline: saved `Rogue Examples/Bend2.mat`, fixed random seed and
  exported options. Baseline success/validation were 1/1, arrival
  39.5285834641 seconds, sampled length 95.3959651184 degrees, maximum direct
  line deviation 2.47268757136 degrees, and saved planner time 0.8398472
  seconds.
- Retained result: success/validation 1/1, identical 39.5285834641-second
  arrival, exact 94.4183046111-degree Euclidean motion within sampling
  precision, 2.91322521662e-13-degree line deviation, and 3.0007202 seconds
  planner time. Collision and every applicable kinematic/dynamics certificate
  passed. The 3.57-times saved-runtime increase is accepted as the measured
  cost of the requested shortest-path correctness; no speedup is claimed.
- Eligibility: fixed-position goal, collinear seed with obstacle-free,
  visibility-graph, or timed-search provenance, line-compatible endpoint
  derivatives, and a common limiting axis across all finite normalized
  derivative limits. Moving goals and incompatible endpoint states are
  deliberately excluded.
- Focused gates: exact saved-case geometry, a direct certified obstacle case,
  incompatible endpoint derivatives, and exact constraint-gradient parity all
  passed. The two affected suites passed 64/64 in 50.026257 seconds.
- Static analysis: `checkcode` reported zero findings across the four changed
  production files and two changed test files.
- Full tests: 112/112 passed in 62.332678 seconds.
- Maintained examples: all 18 ran headlessly and serially in separate MATLAB
  processes. Seventeen independently validated successes passed collision and
  kinematic certificates; the expected no-path result independently validated
  `noValidatedSeed`. Fresh rows are appended to `benchmark.csv`.
- Graphics: visible obstacle-free success created three figures; visible
  expected failure created two diagnostic figures.
- Known limitation: moving-barrier and opening-U retain the pre-existing
  near-singular/singular `fmincon` warning flood. The warning volume explains
  the oversized sandbox log observed in the rogue failure bundle and remains
  separate follow-up work.

## Two-product normal-folder architecture — 2026-08-26

- Source: `main` at `64d0935+two-product-layout-worktree`.
- Layout: root production MATLAB entries changed from six Az/El packages,
  `+hs3`, and eight loose public functions to two normal folders:
  `planAzElMotion/` and `hs3/`. The repository root now has zero `.m` files
  and zero `+*` packages.
- Az/El API: planning, interception, and independent validation remain the
  three unqualified public functions in `planAzElMotion/`. Obstacle calls are
  now `azElObstacles.*`; plotting is `azElPlotting.plotMotion`. The redundant
  plotting facade was removed.
- HS3 API: `hs3/solveTrajHS3.m` is the single public neutral entry;
  `hs3/+hs3Internal/` owns all generic numerical and polynomial helpers. The
  Az/El solver continues to route through those dimension-neutral helpers.
- Path setup: add `planAzElMotion/` and `hs3/`; add `examples/` or `sandbox/`
  only when those tools are needed.
- Static analysis: `checkcode` reported zero findings across both production
  trees and `tests/testArchitectureBoundaries.m`. `which` resolved the public
  planner, obstacle constructor, plotter, and `solveTrajHS3` from their new
  owners.
- Focused tests: architecture, obstacle infrastructure, and standalone HS3
  passed 29/29. Example-requirement tests passed 6/6 after updating hashes for
  package-qualified calls; protected scenario geometry was not changed.
- Full tests: 108/108 passed in 63.545976 seconds.
- Headless examples: all 18 maintained examples ran serially in one MATLAB
  process in 185.535980 seconds. Seventeen successes passed independent
  example validation, collision checks, and kinematic certificates. The
  expected no-path example returned `noValidatedSeed` and passed independent
  failure validation. Exact rows are in `benchmark.csv` under
  `64d0935+two-product-layout-worktree`.
- Graphics: visible `exampleAzElPlanning` created three figures and passed;
  hidden `exampleNoPathAzElMotion` created two failure-diagnostic figures,
  retained search diagnostics, and passed with `noValidatedSeed`.
- Environment limitation: repeated fresh MATLAB launches intermittently
  failed with a MathWorks launcher `File system inconsistency` before any
  repository code executed. The existing user MATLAB process was not stopped
  or modified; the matrix therefore used one process with strictly serial
  example calls.
- Unfavorable evidence: existing near-singular `fmincon` warnings remain in
  the moving-barrier and opening-U examples. Both final motions independently
  validated, but the warnings were not hidden or reclassified.

## Fixed-arrival geometric lower-bound proof — 2026-08-26

- Environment: `HS3-planner` at
  `855a569+fixed-lower-bound-worktree`, MATLAB R2024b Update 4, AMD64 Family
  23 Model 113, no Parallel Computing Toolbox or worker pool.
- Baseline diagnosis: the 42-second four-circle history contained 421 slices.
  With plots disabled, wall time was 62.021637 seconds and planner time was
  60.0989 seconds. Stage timing was 34.9519 seconds corridor construction,
  21.6022 collision checking, 2.1221 motion solving, 0.4585 topology, 0.2055
  final validation, and 0.7587 unattributed.
- Profiler: 41 `polyshape.union` calls consumed 48.2364 inclusive seconds;
  `seedEnvelopeContainsObstacles` consumed 54.4786 seconds. The selected
  direct seed's HS3 call used only 0.4489 seconds.
- Retained changes: the example supplies obstacle samples only through its
  requested 22-second planning horizon while independently checking the full
  conceptual 42-second profile endpoints. Dynamic fixed-arrival seed order is
  shortest-geometric-first, and search stops after an independently validated
  motion reaches the Euclidean start-goal lower bound.
- Focused result: clipping history alone reduced wall time to 26.148349
  seconds. The complete change reached 5.722992 seconds in the focused run and
  5.634472 seconds in the final serial matrix. The result remained exactly 20
  degrees at exactly 22 seconds with collision, velocity, acceleration, jerk,
  dynamics, endpoint, and example shortest-route validation passing.
- Structural regression: a distinct far-moving-obstacle fixed-arrival test
  exposes multiple seeds, attempts one, reaches its computed geometric lower
  bound, and passes. The moving-barrier waiting regression also passes.
- Static analysis and tests: Code Analyzer reported zero findings in the
  changed files. The complete suite passed 106/106 in 69.471882 seconds.
- Examples: 18 fresh serial `PlotOutputs=false` processes completed in
  297.286362 seconds. Seventeen successes passed independent collision and
  kinematic certificates; the expected no-path case independently validated
  `noValidatedSeed`. Exact rows are in `benchmark.csv` under
  `855a569+fixed-lower-bound-worktree`.
- Graphics: the visible four-circle success produced two figures and retained
  the shortest-route certificate. The hidden expected failure produced two
  diagnostic figures and retained nine rejected edges.
- Unfavorable evidence: `exampleMovingCircleNoAzimuthWrap` independently
  validated at 8.707031 seconds, 0.0609997 seconds later than its preceding
  row. The changed ordering applies only to fixed arrival, so no causal
  improvement or regression claim is made for this timing-sensitive case.

## Fixed-arrival length-first candidate quality — 2026-08-26

- Baseline: clean `855a569`, `HS3-planner`, identical scenario inputs and
  deterministic focused controls.
- Retained rule: fixed-arrival candidates keep the requested terminal time and
  rank by independently validated sampled motion length, then integrated jerk
  and stable seed index. Earliest-arrival behavior is unchanged.
- Focused benefit: four accelerating circles shortened from 27.8702009821 to
  25.9348981999 degrees (6.943986%) at exactly 22 seconds.
- Structural fixed cases: alternating occlusion shortened 3.809973%; target
  exits obstacle shortened 4.772009%; the already-straight specified-time
  intercept remained unchanged at 9.53894054682 degrees and 12 seconds.
- Physical validity: every improved result retained independent collision,
  velocity, acceleration, jerk, dynamics, endpoint, and fixed-time checks.
- Center-line diagnosis: the four protected circles block elevation zero from
  7.3664844164 to 12.6335155836 seconds. Both early and late direct passages
  violate the time/dynamics bounds; the direct seed was attempted and returned
  `optimizerInfeasible`.
- Tests: the focused planner suite passes 54/54, including stable early-failure
  diagnostics and static multi-route shortest-selection coverage.
- Examples: all 18 maintained examples ran serially in fresh headless
  processes in 254.1345943 seconds; exact rows are in `benchmark.csv` under
  `855a569+fixed-length-worktree`.
- Graphics: the improved fixed-arrival four-circle result independently
  validated and created three visible figures.
- Runtime tradeoff: static fixed-arrival planning may now attempt every retained
  seed, bounded by `MaximumSeedCount` and `MaximumPlanningTime_s`, rather than
  stopping after the first validated seed.

## Flat architecture and frozen HS3 boundary — 2026-08-26

- Environment: `HS3-planner` at
  `ad3139c+flat-architecture-worktree`, MATLAB R2024b Update 4.
- Architecture: six flat Az/El packages plus frozen `+hs3`; no nested
  production package directories, duplicate production MATLAB basenames, or
  legacy `+azElInternal` / `+azElPlannerMethods` trees remain.
- Dependency boundary: Az/El seed solving delegates optimization,
  reconstruction, and evaluation to `hs3`; numerical optimizer calls occur
  only in `+hs3/optimize.m`; HS3 source remains domain-neutral.
- Static checks: Code Analyzer reported zero findings. `git diff --check`
  passed, and `git diff -- +hs3` was empty.
- Tests: 104/104 passed, zero failed or incomplete, in 52.560 seconds.
- Examples: all 18 maintained examples ran serially and headlessly in fresh
  processes in 241.822 seconds. Seventeen independently validated successes
  passed collision and kinematic certificates; the expected no-path example
  independently validated `noValidatedSeed`. Exact rows are in
  `benchmark.csv`.
- Graphics: a visible obstacle-free success created three figures. The
  expected no-path case created two hidden diagnostic figures and retained its
  search grid.
- Known weakness: moving-barrier and opening-U optimization emitted repeated
  near-singular or singular working-precision warnings. The returned motions
  independently validated, but the conditioning issue remains visible.

## Severe-static fixed-time quality search — 2026-08-26

- Environment: `HS3-planner` at `7661321+fixed-quality-worktree`, MATLAB
  R2024b Update 4, Optimization Toolbox 24.2, serial fresh processes while two
  user-owned MATLAB processes remained untouched.
- Profile: wide U spent 12.87 of 14.23 planner seconds in motion solving and
  11.98 seconds in `fmincon`; topology generation used 0.48 seconds. This
  localized the gap to the motion transcription rather than route discovery.
- Retained rule: on the first quality decision, a non-timed earliest-arrival
  static route with relative sampled-motion inflation above
  `2.5 / segmentCount` receives a maximum-mesh fixed-arrival feasibility solve.
  Existing timed bisection then shortens the horizon while preserving the
  original topology seed. Other route families retain their prior flow.
- Wide-U evidence: the final matrix reaches 22.6308876389 seconds with a
  34.9425880405-degree polyline, 41.5363500661-degree sampled motion,
  64 segments, one mesh pass, passing collision and kinematic certificates,
  and 17.688485 seconds wall. Arrival improves by 0.4075710153 seconds and wall
  by 1.429006 seconds versus the preceding worktree, leaving a
  0.7981520967-second arrival gap to 325.
- Negative controls: two opposing U remains exactly 21.9090824092 seconds;
  forty moving circles remains 61.2011842765 seconds; extreme U.S. remains
  6.3679977362 seconds; moving/deforming U.S. remains 8.75061035156 seconds;
  fixed-arrival and causal-timing examples remain valid.
- Rejected probes: global static fixed-time search slowed two opposing U and
  did not match the retained wide-U result. Interior-point-convex restored
  feasibility but introduced 80--87-second probes and a run exceeding
  12 minutes. Finer extreme-U.S. meshes regressed arrival, so none were kept.
- Rogue replays: `failure.mat` validates at 88.2939404925 seconds in
  7.219560 seconds wall; `successwhenincreasehorizon.mat` validates at
  88.2939359679 seconds in 7.831260 seconds wall. Both use 20 segments and one
  mesh pass; their arrival difference is 4.525 microseconds.
- Full tests: 82/82 passed, zero failed or incomplete, in 50.675781 seconds.
- Maintained examples: all 18 ran serially in fresh MATLAB processes. Exact
  rows are appended under `7661321+fixed-quality-worktree` in `benchmark.csv`;
  17 successes and the expected failure independently pass.
- Graphics: visible basic planning produced two figures. Expected no path
  produced two diagnostic figures and retained `noValidatedSeed`.
- Static audit: recursive Code Analyzer reports zero messages across 84 MATLAB
  files. Nine HS3 files contain exactly 2,000 nonblank, noncomment lines.
- 325 comparator: remaining arrival gaps are wide U +0.7981520967 seconds,
  forty moving circles +0.8393466878 seconds, and extreme U.S.
  +0.3610623598 seconds. Two opposing U is 0.2018852770 seconds earlier.
  Moving/deforming U.S. is 0.3892445513 seconds earlier but 38.0431317 seconds
  slower wall. No global optimality or uniform runtime claim is made.

## Derivative-slack continuation quality pass — 2026-08-26

- Environment: `HS3-planner` at `7661321+slack-quality-worktree`, MATLAB
  R2024b Update 4, Optimization Toolbox 24.2, serial fresh processes while two
  user-owned MATLAB processes remained untouched.
- Retained rule: after one valid same-mesh relinearization, an
  earliest-arrival non-timed spatial candidate with acceleration and jerk
  peaks each below 75% of their limits may receive one 2x mesh pass initialized
  from the validated motion. Length-inflation quality passes retain their
  original-seed initialization.
- Two-opposing-U evidence: two focused repeats and the final matrix reproduce
  21.9090824092-second arrival, 24.5077116377-degree polyline,
  24.4201122273-degree sampled motion, 20 segments, one mesh pass, passing
  collision and kinematic certificates. This improves the preceding worktree
  by 0.9660319268 seconds and beats the 325 row by 0.2018852770 seconds.
  Final wall is 13.972852 seconds, so the improvement carries an explicit
  9.3646144-second wall increase versus the preceding row.
- Negative controls: forty moving circles remains exactly at
  61.2011842765 seconds and 125.185941203 degrees; wide U remains
  23.0384586542 seconds; extreme U.S. remains 6.3679977362 seconds; moving
  circle remains 8.64603156476 seconds. Basic and alternating-slalom results
  also remain numerically unchanged.
- Rogue replays: `failure.mat` validates at 88.2939404925 seconds in
  7.295837 seconds wall; `successwhenincreasehorizon.mat` validates at
  88.2939359679 seconds in 7.919359 seconds wall. Both use 20 segments and one
  mesh pass, and their arrival difference is 4.525 microseconds.
- Full tests: 82/82 passed, zero failed or incomplete, in 50.245827 seconds.
  An earlier 81/82 invocation intentionally does not count because its harness
  suppressed the warning required by the sole failing warning-requirement test.
- Maintained examples: all 18 ran serially in fresh MATLAB processes after the
  final ordering fix. Exact rows are appended under
  `7661321+slack-quality-worktree` in `benchmark.csv`; 17 successes and the
  expected failure independently pass.
- Graphics: visible basic planning produced three figures. Expected no path
  produced two diagnostic figures and retained `noValidatedSeed`.
- Static audit: recursive Code Analyzer reports zero messages across 84 MATLAB
  files. Nine HS3 files contain exactly 2,000 nonblank, noncomment lines.
- 325 comparator: remaining arrival gaps are wide U +1.2057231120 seconds,
  forty moving circles +0.8393466878 seconds, and extreme U.S.
  +0.3610623598 seconds. Moving/deforming U.S. is 0.3892445513 seconds earlier
  but measured 55.917228 seconds wall versus 18.2106663 seconds on 325. No
  global optimality or uniform runtime claim is made.

## Dynamic spatial quality pass — 2026-08-26

- Environment: `HS3-planner` at `7661321+dynamic-quality-worktree`, MATLAB
  R2024b Update 4, Optimization Toolbox 24.2, serial fresh processes.
- Forty-circle localization: the 10-, 20-, and 30-segment transcriptions use
  the identical 110.807922148-degree topology. They independently validate at
  64.5557730468, 61.2011842765, and 60.1588345587 seconds respectively.
  Starting at 20 and 30 segments costs 23.9281868 and 31.5052284 seconds wall.
- Retained bounded rule: after a valid earliest-arrival spatial candidate,
  sampled-motion inflation above one mesh interval permits one quality pass.
  Dynamic geometry uses 2x segments; static severe inflation retains the
  existing 3x choice. Fixed-arrival and timed-topology candidates are excluded.
- Final forty-circle evidence: 20 segments, 61.2011842765-second arrival,
  125.185941203-degree sampled motion, passing collision and kinematic
  certificates, and 18.6109941 seconds wall. This improves the immediately
  preceding worktree by 3.3545887703 seconds arrival for 2.1703734 seconds wall
  and leaves a 0.8393466878-second gap to the 325 row.
- Negative controls: moving circle remains at 10 segments, zero passes, and
  8.64603156476 seconds. Moving/deforming U.S. remains a timed topology at
  8.75061035156 seconds and zero mesh passes. Four accelerating circles retains
  fixed 22-second behavior. Three dense single-obstacle moving probes under
  timed-search suppression remained below the inflation threshold and did not
  refine.
- Tests: 82/82 passed, zero failed or incomplete, in 50.3745157 seconds.
- Maintained examples: all 18 ran serially in fresh MATLAB processes. Exact
  final rows are appended under `7661321+dynamic-quality-worktree` in
  `benchmark.csv`; 17 successes and the expected failure independently pass.
- Graphics: visible basic planning produced three figures and 526 objects.
  Expected no path produced two figures and 341 objects, 15 rejected
  transitions, and 9 retained rejected edges.
- Static audit: recursive Code Analyzer reports zero messages across 84 MATLAB
  files. `git diff --check` reports no whitespace errors beyond line-ending
  notices. Nine HS3 files contain 1,999 nonblank, noncomment lines.
- 325 comparator: remaining arrival gaps are wide U +1.205723112 seconds,
  forty moving circles +0.8393466878 seconds, two opposing U
  +0.7641466498 seconds, and extreme U.S. +0.36106235982 seconds.
  Moving/deforming U.S. is 0.3892445513 seconds earlier but 31.5077561 seconds
  slower wall. No global optimality or uniform runtime claim is made.

## Ordered-boundary and route-quality Pareto — 2026-08-26

- Environment: `HS3-planner` at
  `7661321+geometry-fastpath-worktree`, MATLAB R2024b Update 4,
  Optimization Toolbox 24.2, serial fresh processes.
- Geometry fast path: `shapeAtTime` now reports ordered-single-region,
  convexity, and outward-orientation evidence directly from canonical vertices.
  HS3 consumes that record without constructing a transient `polyshape`.
  Multi-region or degenerate geometry still constructs and queries the exact
  shape. `testShapeQueryReportsOrderedBoundaryProperties` compares the
  lightweight orientation against `isinterior` and checks convex, concave, and
  multi-region classifications.
- Forty-circle profile before the change: 22,920 `shapeAtTime` calls and
  19.270743 profiled seconds in `buildCorridor`, dominated by repeated
  `polyshape`, `area`, and `isinterior` calls. Final matrix evidence preserves
  every physical metric and reduces wall from 20.6046246 to 16.4406207 seconds.
- Static ordering: detour proposals are ranked by geometric length because
  candidate work is already separately bounded. Extreme U.S. now attempts the
  22.2394635087-degree route before the 22.3733117302-degree route and validates
  at 6.3679977362 seconds, 20 segments, and 24.6064786878 degrees of sampled
  motion. Final wall is 64.0234264 seconds.
- Bounded mesh rule: route inflation above one coarse mesh interval triggers
  one 2x pass; severe inflation above 2.5 intervals triggers one 3x pass.
  Wide U retains 30 segments and 23.0384586542 seconds. Extreme U uses 20
  segments. The neutral-circle regression expects 20 segments and the complete
  HS3 suite passes.
- Rogue replays: `failure.mat` independently validates on seed 2 at
  88.2939404925 seconds, 20 segments, 204.669079083-degree sampled motion,
  0.0061099373-degree clearance, and 6.9218305 seconds wall.
  `successwhenincreasehorizon.mat` validates on seed 2 at 88.2939359679
  seconds, 20 segments, 204.667439115-degree sampled motion,
  0.0061101734-degree clearance, and 7.5901807 seconds wall. Both collision and
  kinematic certificates pass; their arrival difference is 4.52 microseconds.
- Full tests: 82/82 passed, zero failed or incomplete, in 49.6429588 seconds.
  Focused HS3 plus obstacle-infrastructure tests passed 57/57 in
  44.1194061 seconds.
- Full examples: all 18 maintained examples ran serially in fresh processes.
  Seventeen independently validated successes and the expected validated
  failure are recorded under `7661321+geometry-fastpath-worktree` in
  `benchmark.csv` and were reported directly in chat.
- Graphics: visible basic planning produced three figures and 526 graphics
  objects. Expected no path produced two figures and 341 objects, including
  15 rejected transitions and 9 retained rejected edges.
- Static audit: recursive Code Analyzer reports zero messages across 84 MATLAB
  files. `git diff --check` reports no whitespace errors beyond line-ending
  notices. Nine HS3 files contain 1,998 nonblank, noncomment lines.
- 325 comparator: versus `da52da8+quintic-root-recovery-worktree` on
  `325-full-suite`, remaining arrival gaps are forty moving circles
  +4.1939354581 seconds, wide U +1.205723112 seconds, two opposing U
  +0.7641466498 seconds, and extreme U.S. +0.36106235982 seconds.
  Moving/deforming U.S. is 0.3892445513 seconds earlier but 31.1939185 seconds
  slower wall. No global optimality or uniform runtime claim is made.

## Static quality and time-expanded retiming — 2026-08-26

- Environment: `HS3-planner` at `7661321+timed-retiming-worktree`, MATLAB
  R2024b Update 4, Optimization Toolbox 24.2, serial fresh processes, plots and
  animation disabled for benchmark timing.
- Static rogue replay: `failure.mat` validates on seed 2 with 30 segments at
  87.1503426168 seconds, 200.592721765-degree sampled motion, and
  0.0027516996-degree clearance. `successwhenincreasehorizon.mat` validates on
  the same seed and mesh at 87.1503401418 seconds, 200.134547736-degree sampled
  motion, and 0.0028127474-degree clearance. The 180/360-second horizons differ
  by about 2.5 microseconds. `straightline.mat` and `skeptic.mat` remain valid
  at 57.5394882088 and 57.5394875671 seconds.
- Mesh evidence on the rogue topology: 20 segments validate at
  88.2939404925 seconds, 30 at 87.1503426168 seconds, and 40 at
  86.5467293065 seconds. The retained policy permits one 3x quality pass and
  therefore chooses the measured 30-segment runtime/quality point. The saved
  historical 40-segment 86.5088536619-second trajectory still passes the
  current continuous validator; the remaining local-quality gap is explicit.
- Structurally different regression:
  `testStaticLengthInflationTriggersOneQualityMeshPass` uses a neutral
  48-vertex circle, verifies the one-pass 30-segment mesh, and requires an
  arrival below 82 seconds. The earlier tall-detour horizon regression remains
  in the suite.
- Timed-topology repair: the moving/deforming U.S. example improves from
  30.1605224609 to 8.75061035156 seconds by testing one arc-length timing law
  on the same `timeExpandedVisibilityGraph` seed. Its final polyline is
  41.5785140688 degrees, sampled motion is 40.7424283094 degrees, and wall time
  is 52.9070181 seconds. Collision and all kinematic certificates pass.
- Direct-wait guard: arc-length retiming is ineligible for `directWait` seeds,
  because removing their repeated position destroys the causal law they
  encode. Final serial runs preserve moving barrier at 10.2314453125 seconds
  and opening-U at 11.8560791016 seconds; both select direct-wait seeds and pass
  independent validation.
- Full maintained matrix: 18/18 example outcomes match their requirements in
  separate serial MATLAB processes: 17 validated successes plus the validated
  expected no-path result. Exact per-example geometry, duration, certificates,
  termination, and wall time are appended to `benchmark.csv` under
  `7661321+timed-retiming-worktree` and were reported directly in chat.
- Test command: `runtests('tests')` with only MATLAB's singular-matrix warning
  IDs suppressed. Result: 81/81 passed, zero failed or incomplete, in
  52.4952677 seconds. A prior run with all warnings disabled produced one
  expected test-harness failure because the unknown-option warning requirement
  was intentionally hidden; that run is not counted as code evidence.
- Static command: recursive `checkcode(..., '-id')` over all MATLAB sources.
  Result: zero messages across 84 files. The three modified HS3 solver files
  also passed a focused Code Analyzer run.
- Graphics: visible `exampleAzElPlanning` passed and produced three figures
  with 526 objects. Hidden `exampleNoPathAzElMotion` passed independent failure
  validation and produced two diagnostic figures with 15 rejected transitions
  and 9 retained rejected edges.
- Repository checks: `git diff --check` found no whitespace errors beyond
  line-ending notices. Nine HS3 MATLAB files contain exactly 2,000 nonblank,
  noncomment lines, which meets the hard cap with zero spare lines.
- Comparator: the requested branch baseline is the later serial matrix under
  `da52da8+quintic-root-recovery-worktree` on `325-full-suite`, not only the
  earlier commit state. Current moving/deforming U.S. arrival is
  0.3892445513 seconds earlier than that like-for-like row, while wall time is
  34.6963518 seconds slower. Unfavorable current arrival gaps remain visible
  for forty moving circles (+4.1939354581 seconds), extreme U.S. visibility
  (+2.21937577543 seconds), the wide U (+1.205723112 seconds), and two opposing
  U obstacles (+0.7641466498 seconds). No global optimality or uniform runtime
  claim is made.

## Timed-arrival and exhaustive-failure repair — 2026-08-26

- Rogue duration-role diagnosis: identical static geometry and limits differed
  only in the 180/360-second horizon. Topology geometry was identical, while
  seed 2's warm-start duration had changed from 93.4400819229 seconds to the
  74.4431617582-second independent-axis lower bound. Solving from the
  conservative duration succeeds and independently validates under both
  horizons. The retained repair separates reachability/pruning from solver
  initialization.
- Rejected alternatives: fixed-time QP probes at 86.5, 88.3, 90, 91, and
  91.02 seconds were optimizer-infeasible under the current affine corridor,
  even though the nonlinear formulation validates at 91.019 seconds; extending
  that QP to static spatial seeds would therefore lose feasible motions. A
  nonlinear 85-second warm start converged to the same 91.018899717-second
  local solution, while the 74.4431617582-second physical-bound start failed
  before optimization with a nonfinite initial constraint. Neither alternative
  improved arrival without unacceptable robustness loss, so neither was
  retained.
- Serial rogue replays after the repair:
  `failure.mat` passes on seed 2 at 91.0188996291 seconds, 186.880163846-degree
  polyline, 217.919941498-degree sampled motion length, and 5.544338 seconds
  wall; `successwhenincreasehorizon.mat` passes on seed 2 at
  91.0189002025 seconds, the same polyline, 217.830384396-degree sampled
  motion length, and 5.280859 seconds wall. Both collision and kinematic
  certificates pass. `straightline.mat` remains valid at 57.5394882088 seconds
  in 2.759510 seconds wall; `skeptic.mat` remains valid at 57.5394875671
  seconds in 3.419472 seconds wall.
- Focused regression:
  `testDetourWarmStartDoesNotDefineReachability` constructs a tall multi-axis
  detour, measures its physical lower bound and conservative warm duration,
  then verifies that an identical request with a horizon between those values
  retains the route and clamps only the warm start. It passes in 0.79775
  seconds.
- Fresh maintained controls ran serially and headlessly. `exampleAzElPlanning`
  passes at 7.57952069664 seconds in 4.706205 seconds wall;
  `exampleMovingCircleNoAzimuthWrap` passes at 8.64603156476 seconds in
  10.553267 seconds wall; `exampleNoPathAzElMotion` returns the expected,
  independently validated `noValidatedSeed` with zero HS3 attempts in
  1.150478 seconds wall. Exact rows were appended to `benchmark.csv`.
- Final verification: 79/79 repository tests passed in 51.045611 seconds;
  Code Analyzer reported zero messages across 84 MATLAB files; `git diff
  --check` reported no whitespace errors beyond line-ending notices.

- Environment: `HS3-planner` at `7661321+worktree`, MATLAB R2024b Update 4,
  Optimization Toolbox 24.2, no Parallel Computing Toolbox, figures and
  animation disabled for timing.
- Explicit comparator: `325-full-suite` at `67bc087`, whose maintained
  examples select `corridorQuintic` by default.
- Profiled no-path baseline: 55.6428 seconds planner total, including
  54.2559 seconds motion solving, 0.3598 seconds topology, and 0.1177 seconds
  final validation. Five `fmincon` calls across three meshes spent 32.3781
  seconds in augmented-matrix factorization. The retained exact exhaustive
  static certificate returns the same validated failure in 1.3769188 seconds
  headless with zero HS3 attempts.
- Timed repair: fixed-time feasibility bisection preserves absolute event
  timing. Opening-U reaches 11.8560791016 seconds in 12.9626115 seconds wall;
  moving barrier reaches 10.2314453125 seconds in 12.6557865 seconds wall.
  Both pass independent collision and kinematic validation.
- Arrival regression guard: moving circle retains 8.64603156476 seconds after
  rejecting a faster fixed-QP-only incumbent probe that arrived at
  8.70703125 seconds. Its final 10.7064233-second wall remains slower than the
  9.0140762-second pre-repair measurement.
- Earlier verification: full repository tests passed 78/78 in 44.1178 seconds. Visible
  basic planning created three figures and 527 objects. Expected no path
  created two diagnostic figures with 15 rejected transitions.
- Size: nine HS3 MATLAB files contain 1,927 nonblank, noncomment lines;
  `plan.m` contains 575 physical lines.
- Untested: the user stopped the fresh 18-example serial comparison matrix
  after the focused regressions were isolated. No global optimality,
  completeness, or uniform runtime claim is made.

## Evidence scope

- Branch: `plan-325`.
- Baseline commit for the prepared-obstacle experiment: `4f59472`.
- Verified state: the current uncommitted Plan 325 implementation.
- Runtime: MATLAB R2024b Update 4 with Optimization Toolbox.
- Date: 2026-08-20.
- Every maintained example used a finite jerk limit.

Each maintained example ran in its own MATLAB process. Runs were serial.
Headless controls disabled plots, animation, and pauses.

## Implemented requirement changes

- Workspace bounds moved from planner options to
  `limits.azimuthInterval_deg` and `limits.elevationInterval_deg`.
- `MaximumPlanningTime_s` and the whole-planner deadline were removed.
- Required work uses deterministic seed, graph, iteration, evaluation,
  collocation, and refinement limits.
- Optional HS3 improvement retains its separate 15-second default limit.
- Stable verbose messages use the `[AzEl]` prefix and stage prefixes.
- Fixed-goal examples state earliest arrival. Moving-target examples state
  their target-time policy.
- The slalom elevation interval is `[-5 5]` degrees.
- All maintained examples route `MaxJerk_deg_s3` into physical limits.
- The plotter retains the `main` branch visual style.
- A planning-runtime non-regression gate now applies to later changes.
- Dynamic obstacle slices and interval data are prepared once per planning
  call. The data remains time-dependent and shape-dependent.

## Size checks

| Scope | Files | Physical lines | Limit | Result |
| --- | ---: | ---: | ---: | --- |
| Core production, without plotting | 27 | 6,559 | 7,000 hard limit | pass |
| Plotting | 1 | 499 | separate report | pass |
| Production MATLAB | 28 | 7,058 | 7,000 target plus allowance | pass |
| Complete MATLAB tree | 54 | 11,769 | 12,000 hard limit | pass by 231 |
| Complete MATLAB tree | 54 | 11,769 | 10,500 target | fail by 1,269 |

No production MATLAB file is longer than 900 lines. The preferred complete
tree target does not pass.

## Final headless example results

`P/V` means planner success and independent example-validation pass. `C/K`
means collision and kinematic certificate pass. `NaN` means unavailable after
an expected failure. Fixed target durations are required target times, not
minimum-time results.

| Example | Goal mode | P/V | Polyline (deg) | Motion (deg) | Duration (s) | C/K | Wall (s) | Reason |
| --- | --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | earliest | 1/1 | 16.060439635 | 16.758281983 | 12.180917402 | 1/1 | 30.2005183 | `goalReached` |
| `exampleAzElPlanning` | earliest | 1/1 | 11.152119519 | 11.303432110 | 7.817268021 | 1/1 | 20.0718797 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.700721560 | 12.807761070 | 8.817608547 | 1/1 | 36.8415733 | `goalReached` |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685 | 122.962176120 | 64.556766026 | 1/1 | 22.4426104 | `goalReached` |
| `exampleFourAcceleratingCircles` | fixed target | 1/1 | 24.363303007 | 27.712518684 | 22 | 1/1 | 56.3132192 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | fixed target | 1/1 | 9.538940547 | 9.538940547 | 12 | 1/1 | 3.3125197 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | earliest target | 1/1 | 10.097524449 | 7.342215833 | 6.275807672 | 1/1 | 4.8222865 | `goalReached` |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10 | 10.139859112 | 10.544227894 | 1/1 | 29.9058765 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12.113593185 | 12.113593185 | 12.293137410 | 1/1 | 16.7853734 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 63.084805147 | 71.508173805 | 12.986426213 | 1/1 | 36.3672785 | `goalReached` |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN/NaN` | 15.5711128 | `noValidatedSeed` |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.472135955 | 4.472860956 | 4.613406127 | 1/1 | 6.2329199 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10 | 10 | 15 | 1/1 | 16.7397940 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | fixed target | 1/1 | 13.341664064 | 19.229413228 | 20.869565217 | 1/1 | 23.9341472 | `goalReached` |
| `exampleTargetExitsObstacle` | fixed target | 1/1 | 19.824386759 | 22.879930804 | 24 | 1/1 | 12.5645525 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 23.853720884 | 24.302835532 | 22.876124561 | 1/1 | 66.0249043 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.942588040 | 42.580115766 | 26.492875600 | 1/1 | 40.7620833 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.239463509 | 25.132264157 | 8.902682125 | 1/1 | 132.5539808 | `goalReached` |

The wide single-U request is unchanged. Its validated arrival duration is
26.493 seconds, which is below the requested 38-second threshold.

## Sparse visibility-graph experiment

The graph now tests deterministic Delaunay pairs plus all start and goal
connections. It does not change collision validation. The 40-circle case
tested 62 of 153 possible pairs and retained all 28 visible edges, the same
route, and the same arrival time. The wide U tested 55 of 120 possible pairs
and retained both homology classes and the same 26.493-second arrival. The
pair reduction is 59.5% and 54.2%, respectively. No wall-time gain was
confirmed, so this is a graph-work and memory improvement only.

## Runtime non-regression check

Prepared dynamic obstacle data passed the two required runtime gates before the
complete example run. The final serial 40-circle run decreased from
23.0063697 to 22.4426104 seconds, which is a 2.45% decrease. The final serial
moving-U.S. run decreased from 86.5107311 to 36.3672785 seconds, which is a
57.96% decrease. Both runs kept the same arrival duration within numerical
solver variation and passed independent validation. The prepared data stores
each source slice and each interpolation interval. It does not use one static
shape for a complete history.

The shared-jerk correction preserves the old `[2 2]` defaults. Seven changed
examples stayed within 2.7% of their recorded runs. The basic example received
an explicit A/B check:

- old source: 18.842, 18.979, and 19.518 seconds;
- corrected source: 19.090, 19.239, and 19.279 seconds.

The ranges overlap. The 1.37% median difference is inside observed process
noise. The returned path and duration were identical. No confirmed runtime
increase was accepted.

## Display and verbose checks

- Visible success: obstacle-free planning passed and created three figures.
- Visible failure: the no-path example passed failure validation and created
  two diagnostic figures with reason `noValidatedSeed`.
- Verbose success: the obstacle-free example printed setup, seed generation,
  first motion, ten-iteration HS3 updates, selection, and completion lines.
- Quiet runs produced no planner progress text.

## Automated checks

- Full tests: 56 passed, 0 failed, and 0 incomplete.
- Full test process time: 34.0101058 seconds.
- Code Analyzer: 54 MATLAB files and 0 messages.
- `git diff --check`: passed.
- MATLAB source lines longer than 100 characters: 0.
- Focused jerk-requirement tests: 4 passed, 0 failed.

## Superseded audit artifacts and cleanup decisions

The one-time `repo_inconsistencies_plan_325.md` and
`repo_cleanup_audit_plan_325.md` reports described old commit `b845880` and
were removed after their resolved decisions were preserved here and in
`branch_assessment.md`. Workspace ownership, verbose behavior, timeout removal,
production size, jerk routing, prepared obstacle reuse, and package ownership
now reflect the maintained implementation rather than that historical audit.

`certifySeedCorridor` remains because the production validator calls it.
`RandomSeed` remains for public compatibility because removing it would break
the result format. Polynomial sampling remains a measurement-first cleanup
candidate. The repository-owned `repository-cleanup` skill remains under the
parent workspace guidance directory rather than inside this project tree.

## Known limits and claim

- Spatial and timed proposals use finite samples and can miss a feasible
  topology.
- Reduced seed geometry can reject a useful proposal. Final validation uses
  the original protected obstacle history.
- The analytic motion stops at geometric waypoints. HS3 is local and can
  return a poor local result.
- Solver matrix-conditioning warnings can occur. Accepted motions still pass
  independent validation.
- Periodic obstacle images are not implemented.
- Optimization Toolbox is required for HS3.

A successful result is an independently validated motion from a finite,
deterministic proposal set. The planner does not claim global route
completeness or global time optimality.

## Adaptive Early-HS3 Verification — 2026-08-20

The planner now constructs the analytic fallback without immediately running
its continuous certificate for eligible spatial visibility seeds. It first
tries a denser HS3 motion and validates that motion independently. If HS3
fails, it validates the unchanged analytic fallback. Timed and wait seeds keep
the existing causal workflow. Reduced-geometry seeds do not receive a second
clearance expansion.

| Example | Prior duration (s) | New duration (s) | Prior wall (s) | New wall (s) | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| Wide single U | 26.492875600 | 22.828232905 | 40.7620833 | 16.8495066 | pass |
| 40 moving circles | 64.556766026 | 64.556780044 | 22.4426104 | 9.0772133 | equivalent within 0.001 s |
| Moving and deforming U.S. | 12.986426213 | 12.987386290 | 36.3672785 | 27.4422293 | equivalent within 0.001 s |

All 18 maintained examples ran serially in separate MATLAB processes. There
were 17 validated successes and one expected validated no-path result. The
visible success check created three figures. The visible no-path check created
two diagnostic figures and reported two rejected transitions. The final test
run passed 56 tests with no failures or incomplete tests. Code Analyzer checked
54 MATLAB files and returned zero messages. No MATLAB line exceeds 100
characters.

The complete MATLAB tree passes its 12,000-line hard limit. Production has
7,058 lines. The starting commit contained the same production line count
after prepared dynamic-obstacle data was added.

The performance allowance uses the declared wide-U, 40-circle, and moving-U.S.
benchmark set. A 58-line overage requires a 17.4 percent reduction because
`0.30 * 58 / 100 = 0.174`. Their wall-time reductions are 58.66, 59.55, and
24.54 percent. The minimum is 24.54 percent, so the allowance passes. The
wide-U arrival improves by 13.83 percent. The other two arrivals remain within
the configured 0.001-second equivalence tolerance. All three motions pass
independent collision and kinematic validation.

## Plan 325 continuation verification — 2026-08-21

This section supersedes earlier final-state counts and timing tables above. It
applies to commit `a023f1c` plus the current uncommitted optimized worktree.
Historical measurements remain visible for comparison.

### Accepted behavior

- Exact multi-obstacle `visibilityGraph` seeds can attempt HS3 before the
  analytic stop-at-waypoint fallback is accepted. The decision is based on
  obstacle count and seed provenance, not scenario geometry or route names.
- Early and later HS3 attempts share `MaximumHs3ImprovementTime_s`. Later work
  skips seeds already attempted early and uses only the remaining budget.
- Batched polynomial evaluation replaces repeated per-sample helper calls.
- Bernstein conversion accepts multiple columns. HS3 continuous-bound
  conversion is batched by segment and axis and then restored to the exact
  legacy inequality order.
- Seed and seed-summary templates are owned by the existing stable-result
  constructor so `planAzElMotion.m` remains below 900 physical lines.

The analytic motion remains a recoverable fallback and uses the same public
independent validation. No constraint tolerance, collision margin, obstacle
geometry, or iteration limit was weakened.

### Exact-equivalence checks

- Batched polynomial evaluation matched the scalar calculation bit for bit for
  uniform and nonuniform segment durations.
- Matrix Bernstein conversion matched a scalar column loop with maximum error
  zero.
- Complete continuous-bound vectors matched the prior segment/axis loop bit
  for bit with azimuth wrapping disabled and enabled.

### Declared runtime and size proof

Maintained production has 28 files and 7,139 physical lines. The 139-line
overage requires `0.30 * 139 / 100 = 0.417`, or a 41.7 percent wall-time
reduction. Before final A/B measurement, the declared representative set was
the extreme outline, dense concave, and U-shaped time-space examples.

Both sides ran serially in separate headless MATLAB processes. The baseline
was clean `a023f1c`; `Verbose=false` was supplied to both sides because the
baseline example resolver otherwise leaves that accessed field unset.

| Example | Baseline success/validation | Candidate success/validation | Baseline wall (s) | Candidate wall (s) | Reduction | Baseline arrival (s) | Candidate arrival (s) |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| `exampleUSOutlineExtremeVisibility` | 1/1 | 1/1 | 83.8056819 | 48.2212733 | 42.46% | 6.684968340018 | 6.684968340018 |
| `exampleDenseConcaveAzElMotion` | 1/1 | 1/1 | 43.6252843 | 16.8686791 | 61.34% | 8.817608547166 | 8.817608547166 |
| `exampleUShapedAzElTimeSpace` | 1/1 | 1/1 | 89.9305427 | 17.1115690 | 80.97% | 38.549593103900 | 22.819550649779 |

The minimum measured reduction is 42.46 percent, so the production allowance
passes by 0.76 percentage points. The narrow margin is recorded explicitly.

### Final serial headless examples

Every row used finite jerk (`JerkConstrained = 1`). `P/V` is planner success
and independent example-validation pass. `C/K` is collision and applicable
kinematic-certificate pass. The no-path row uses `NaN` for unavailable motion
metrics and reports its stable termination reason.

| Example | Goal mode | P/V | Polyline (deg) | Motion (deg) | Duration (s) | C/K | Wall (s) | Reason |
| --- | --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | earliest | 1/1 | 16.060439635036 | 16.260374075985 | 12.181917401593 | 1/1 | 20.9589414 | `goalReached` |
| `exampleAzElPlanning` | earliest | 1/1 | 11.152119519024 | 11.303432110211 | 7.817268020971 | 1/1 | 11.6422250 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.700721559528 | 12.807761070221 | 8.817608547166 | 1/1 | 16.6448439 | `goalReached` |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685255 | 122.955558287524 | 64.556780043561 | 1/1 | 7.9373138 | `goalReached` |
| `exampleFourAcceleratingCircles` | fixed | 1/1 | 24.363303007331 | 27.712518684341 | 22 | 1/1 | 41.7615641 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | fixed | 1/1 | 9.538940546821 | 9.538940546821 | 12 | 1/1 | 3.5031323 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | earliest | 1/1 | 10.097524449091 | 7.342215833181 | 6.275807672232 | 1/1 | 4.7489639 | `goalReached` |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10 | 10.211853153746 | 10.545227890750 | 1/1 | 26.3832859 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12.113593184851 | 12.113593184851 | 12.293137410146 | 1/1 | 16.9825396 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 63.084805146555 | 69.402637353295 | 12.987386289935 | 1/1 | 28.8491449 | `goalReached` |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN/NaN` | 10.8599145 | `noValidatedSeed` |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.472135955000 | 4.472860955593 | 4.613406126529 | 1/1 | 5.9277416 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10 | 10 | 15 | 1/1 | 16.9304397 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | fixed | 1/1 | 13.341664064126 | 19.229413227596 | 20.869565217391 | 1/1 | 22.9307595 | `goalReached` |
| `exampleTargetExitsObstacle` | fixed | 1/1 | 19.824386758954 | 22.879930804015 | 24 | 1/1 | 8.3465627 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 23.853720883753 | 24.302835531542 | 22.876124561206 | 1/1 | 34.8855048 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.942588040466 | 42.753271369061 | 22.819550649779 | 1/1 | 16.8144250 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.239463508699 | 26.617094587006 | 6.684968340018 | 1/1 | 47.9449594 | `goalReached` |

### Interactive and graphics checks

- A two-polygon interactive case previously accepted an analytic
  boundary-hugging route at about 20.718 seconds. The accepted multi-obstacle
  early-HS3 source selected a wider smooth route at about 8.859 seconds and
  passed independent validation.
- Default visible obstacle-free success passed at 4.613406126529 seconds,
  created four figures, and produced valid workspace, kinematic, and animation
  graphics. Wall time was 14.8765553 seconds.
- Visible expected no-path planning passed failure validation, created two
  figures, preserved two rejected transitions, and reported
  `noValidatedSeed`. Wall time was 14.6133705 seconds.

### Rejected experiments

- Returning after the first early multi-obstacle HS3 result regressed the
  two-U arrival to 23.9675706 seconds. It was removed; unattempted topologies
  now share the remaining HS3 budget.
- Applying continuation to broader reduced-obstacle cases increased the
  40-circle wall time to 24.217 seconds. Continuation was restricted to
  multiple exact obstacles. The final 40-circle result is 7.9373138 seconds.
- Template consolidation initially produced 14 focused-test errors because a
  stale local constructor call remained. The defect was corrected before any
  runtime result was accepted.

### Final automated and size checks

- Focused planner tests after recovery: 43 passed, 0 failed, 0 incomplete.
- Full tests after the final internal-interface cleanup: 56 passed, 0 failed,
  0 incomplete in 38.9511717 seconds.
- Code Analyzer: 55 MATLAB files, 0 messages.
- Production MATLAB: 28 files, 7,139 physical lines; allowance passes.
- Longest production files: `solveAzElHs3.m` 900 lines and
  `planAzElMotion.m` 888 lines.
- Maintained tracked MATLAB tree: 54 files and 11,873 physical lines; the
  12,000-line hard cap passes by 127 lines.
- Untracked interactive sandbox: 694 lines; excluded from maintained counts
  and not suitable to add without cleanup.

The current planner remains bounded and local. These results establish measured
improvement and independent feasibility for the exercised scenario families;
they do not establish global time optimality or complete reachability.

## Consolidated Plan 325 worktree verification — 2026-08-21

This section consolidates superseded worktree tables. Every executed historical
row remains in `benchmark.csv`, and chronological proof/recovery checkpoints
remain in `plan.md`. The current complete table follows in the next section.

### Accepted measured changes before the final checkpoint

- Exact multi-obstacle visibility seeds gained an early HS3 opportunity under
  one shared bounded budget. The manually drawn two-polygon case changed from
  about 20.718 to 8.859 seconds and selected a wider smooth motion. Opposing-U
  retained its independently valid route during that gate.
- Loop-free polynomial record evaluation was bit exact and reduced its isolated
  benchmark from 0.326166 to 0.103552 seconds over 15,000 repetitions.
- Batched polynomial reconstruction preserved coefficient and terminal-state
  bits for 1, 2, 7, and 19 segments.
- Lazy requested-output evaluation preserved two- through five-output calls bit
  for bit and reduced the position-only helper path by 54.84 percent.
- Batched seed-corridor conversion preserved the complete inequality vector bit
  for bit and reduced its isolated helper time by 78.22 percent.
- Frozen corridor times are now computed once per HS3 setup; the profiled
  extreme case had recomputed the invariant 34,203 times.
- Fixed-arrival speed-aware initialization and CG reduced the alternating-
  occlusion motion from 19.229413227596 to 15.324880519000 degrees and reduced
  the four-accelerating-circle wall from 43.0900 to about 29.2 seconds.
- Geometry-conditioned CG improved dense-concave arrival from
  8.817608547166 to 8.798638844754 seconds while exact multi-obstacle and timed
  families retained factorization.

### Removed experiments

- Stopping after the first early exact multi-obstacle result regressed opposing-
  U arrival from 22.8761246 to 23.9675706 seconds.
- Broad continuation increased the 40-circle wall to 24.217 seconds.
- Static-corridor vectorization improved dense locally but repeatedly regressed
  extreme serial pairs; cached Bernstein matrices also regressed dense.
- SQP exceeded a 60-second basic-case proof window. Global CG regressed opposing
  U shapes from 30.74 to 38.35 seconds, and timed CG regressed moving barrier.
- Earliest-arrival average-speed initialization regressed dense arrival to
  8.9027893 seconds. Five-percent seed expansion regressed wide-U arrival to
  24.3270 seconds.
- All removed forms were restored before subsequent measurements.

The user-approved production target is 7,500 physical lines. The separate
900-line production-file and 12,000-line tracked-MATLAB limits remain in force.

## Constraint-feasibility recovery follow-up — 2026-08-21

This section supersedes the preceding geometry-conditioned table and counts.

Earliest-arrival HS3 now retains a constraint-feasible primary minimum-time
solution instead of always running a second nonlinear solve that may trade up
to `ArrivalTimeTolerance_s` of arrival for lower integrated jerk. The second
solve remains available only when the primary nonlinear residual exceeds
`ConstraintTolerance`; it is therefore a feasibility recovery, not a routine
arrival relaxation. Fixed-arrival HS3 is unchanged because its primary
objective is already integrated squared jerk.

### Bounded experiment outcome

- Setting `ArrivalTimeTolerance_s` to `1e-5` recovered 0.99 milliseconds on
  basic, dense, and opposing-U examples, but the opposing-U wall increased to
  34.3097261 seconds. The default was not changed.
- Removing the second solve globally improved basic, dense, opposing-U,
  wide-U, and extreme runtime, but alternating slalom returned
  `noValidatedSeed` because its primary equality residual exceeded tolerance.
  That broad form was removed.
- The accepted condition runs recovery only when the primary residual exceeds
  `ConstraintTolerance`. Alternating slalom returns validated success, while
  already feasible primaries avoid the second solve.
- Limited-memory BFGS increased dense wall from the 13.09-second reference to
  13.5923721 seconds. PCG tolerances `0.01` and `0.2` took 13.2316361 and
  13.1500434 seconds without changing outer iteration counts. Both forms were
  removed.
- Scaling final-time `TypicalX` from the feasible guess produced only a 0.35
  percent extreme serial-pair improvement (40.4166606 to 40.2756680 seconds),
  below the proof threshold. It was removed.
- Nargout-sized allocation in the evaluator preserved every requested output
  but took 1.1225357 seconds versus 1.0924350 seconds for 20,000 position-only
  calls. It was removed.
- Accepted constraint-array reuse removed a duplicate selected-decision
  callback. Feasible basic and recovery-dependent alternating results remained
  bit for bit, 43 focused tests passed, and the solver decreased to 883 lines.
- Repository cleanup consolidated 506 superseded verification lines. Complete
  historical rows remain in `benchmark.csv`; checkpoints remain in `plan.md`.

### Final serial headless examples

| Example | Goal mode | P/V | Polyline (deg) | Motion (deg) | Duration (s) | C/K | Wall (s) | Reason |
| --- | --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | earliest | 1/1 | 16.060439635036 | 16.758281982866 | 12.180917402175 | 1/1 | 13.5232923 | `goalReached` |
| `exampleAzElPlanning` | earliest | 1/1 | 11.152119519024 | 11.411616815005 | 7.816267856881 | 1/1 | 8.2728629 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.700721559528 | 13.431299536656 | 8.797638855700 | 1/1 | 12.3737232 | `goalReached` |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685255 | 126.114009817632 | 64.555779916429 | 1/1 | 4.4305056 | `goalReached` |
| `exampleFourAcceleratingCircles` | fixed | 1/1 | 24.363303011158 | 27.712517413842 | 22 | 1/1 | 29.0747122 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | fixed | 1/1 | 9.538940546821 | 9.538940546821 | 12 | 1/1 | 3.4576386 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | earliest | 1/1 | 10.097524449091 | 7.342498781519 | 6.274806792200 | 1/1 | 3.2941048 | `goalReached` |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10 | 10.134483812277 | 10.544227895142 | 1/1 | 25.3593433 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12.113593184851 | 12.113593184851 | 12.293137410146 | 1/1 | 17.2145099 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 63.084805146555 | 71.436692325459 | 12.986386910606 | 1/1 | 27.7708022 | `goalReached` |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN/NaN` | 11.1774461 | `noValidatedSeed` |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.472135955000 | 4.484905564719 | 4.612405963436 | 1/1 | 3.3335129 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10 | 10 | 15 | 1/1 | 16.3750253 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | fixed | 1/1 | 13.341664064126 | 15.324880518989 | 20.869565217391 | 1/1 | 22.8903990 | `goalReached` |
| `exampleTargetExitsObstacle` | fixed | 1/1 | 19.824386758954 | 22.879843740594 | 24 | 1/1 | 6.9458763 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 23.853720883753 | 24.370904895056 | 22.875124576026 | 1/1 | 21.4478266 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.942588040466 | 43.259235381251 | 22.818548735851 | 1/1 | 7.7077780 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.239463508699 | 26.617251754385 | 6.683971648809 | 1/1 | 36.7603481 | `goalReached` |

Every successful row passed collision and applicable kinematic certificates.
The no-path row retained its independently validated failure format and two
rejected transitions. Earliest selected HS3 solutions are about one
millisecond earlier than the preceding jerk-relaxed results. Some are wider:
dense motion increases 4.86 percent, 40 circles 2.57 percent, wide U 1.18
percent, and opposing U shapes 0.28 percent. Those tradeoffs are visible and
retain hard jerk limits.

### Final checks and size

- Focused HS3 tests: 43 passed in 30.8298656 seconds; solver Code Analyzer
  messages: 0.
- Full tests: 56 passed, 0 failed, and 0 incomplete in 33.8326055 seconds.
- Code Analyzer: 55 MATLAB files, 0 messages.
- Visible success: three figures and 490 graphics objects with independently
  valid 4.612405963436-second duration.
- Visible failure: two diagnostic figures, two rejected transitions, and an
  independently valid `noValidatedSeed` result.
- Production: 28 files and 7,123 physical lines, 377 below the user-approved
  7,500-line target.
- Maintained tracked MATLAB tree: 54 files and 11,857 physical lines, 143
  below the 12,000-line hard cap.
- Core files: `solveAzElHs3.m` is 883 lines and `planAzElMotion.m` is 888.
- The 694-line interactive sandbox remains untracked.

The interactive two-polygon sandbox remains in the exact multi-obstacle early-
HS3 family. Its accepted wider-arc eligibility is unchanged, but the geometry
was not manually redrawn during this headless gate.

No global optimality, complete reachability, or machine-independent runtime
claim is made.

## Affine fixed-time constraint verification

Fixed-arrival HS3 constraints are affine in jerk because final time and every
obstacle query time are fixed. The final source evaluates that affine basis
once and passes it through fmincon's linear-constraint interface; earliest-
arrival requests retain the nonlinear time-decision callback. Diagnostics
report `linearFixedTime` or `nonlinearTimeDecision`, and focused tests exercise
both representations.

A seeded independent 30-variable affine system with 41 inequalities and six
equalities reproduced 100 random decisions with maximum residual error
7.11e-15, including the conversion from raw `c(x) <= 0` values to fmincon
matrix bounds.

| Fixed-arrival example | Previous wall (s) | Final wall (s) | Previous/final motion (deg) |
| --- | ---: | ---: | ---: |
| `exampleFourAcceleratingCircles` | 29.0747122 | 25.2320069 | 27.712517413842 / 20.372411016257 |
| `exampleInterceptMovingTargetAtSetTime` | 3.4576386 | 2.9311400 | 9.538940546821 / 9.538940546821 |
| `exampleStraightTargetAlternatingOcclusion` | 22.8903990 | 20.9311553 | 15.324880518989 / 14.220153980999 |
| `exampleTargetExitsObstacle` | 6.9458763 | 4.5878764 | 22.879843740594 / 22.879867003467 |

The target-exit change is 0.0000233 degrees; its integrated squared jerk
decreased from 0.146912667008831 on pushed checkpoint `2074c14` to
0.146912596533341, and its maximum solver violation decreased from
1.82e-14 to 2.00e-15. The accelerating-circle and alternating-occlusion cases
selected different, independently validated motions. Fixed arrival time and
all hard certificates were preserved.

All 18 maintained examples then ran serially and headlessly. Seventeen were
independently validated successes; `exampleNoPathAzElMotion` retained the
independently validated `noValidatedSeed` failure. Every earliest-arrival
metric was bit exact to the preceding feasibility-recovery sweep. The full
rows are appended to `benchmark.csv` under source tag
`2074c14+linear-fixed-constraints-worktree`.

The sweep initially exposed a pre-planning example-requirement failure:
`exampleMovingDeformingUSOutlineVisibility` read `Verbose` from a partial
planner-options structure. The shared example resolver now materializes the
single public planner default structure before applying scenario and user
overrides. A dedicated default/override test passes, the original headless
case passes, and the structurally different extreme-outline case also passes.

- Full tests: 57 passed, 0 failed, and 0 incomplete in 29.1130938 seconds.
- Code Analyzer: 55 maintained MATLAB files, 0 messages.
- Visible success: four figures and 643 graphics objects.
- Visible failure: two figures, 341 graphics objects, and two rejected
  transitions with `noValidatedSeed`.
- Production: 29 files and 7,185 physical lines, 315 below the approved 7,500.
- Maintained MATLAB tree: 55 files and 11,940 physical lines, 60 below the
  12,000 hard cap.
- Core files: `solveAzElHs3.m` is exactly 900 lines and `planAzElMotion.m` is
  888 lines. The 694-line interactive sandbox remains untracked.

The interior-point feasibility-mode recovery experiment was removed after it
retained the same eight iterations and 326 evaluations while slightly
increasing alternating-slalom wall time.

## Exact fixed-time objective gradient

After fixed-time constraints became linear, fmincon still estimated the exact
quadratic jerk-objective gradient by repeated objective calls. The retained
helper returns the closed-form gradient for fixed-arrival solves. A seeded
30-variable central-difference proof measured maximum absolute error
2.58e-9 and maximum relative error 5.33e-10. The set-time example's reported
objective function count decreased from 1,032 to 24.

The complete 18-example serial headless gate was repeated on the extracted
helper source. Seventeen successes and the expected no-path failure passed
independent validation. All earliest-arrival metrics were bit exact. The
largest measured fixed-arrival wall improvement was accelerating circles,
25.2320069 to 23.6112598 seconds; set-time, occlusion, and target-exit walls
were 2.9573416, 20.7929114, and 4.5272330 seconds. Their motion changes versus
the no-gradient affine source were zero, 1.41e-6, 5.81e-7, and 1.07e-6 degrees.
All collision and kinematic certificates passed.

An additional seeded central-difference matrix covered 1, 2, 5, and 9
segments in both fixed- and variable-time layouts. Across all eight cases, the
maximum absolute gradient error was 2.83e-9 and the maximum relative error was
1.14e-9.

- Full tests: 57 passed in 29.2256310 seconds.
- Code Analyzer: 56 maintained MATLAB files, 0 messages.
- Visible success/failure: four/two figures; failure retained two rejected
  transitions and `noValidatedSeed`.
- Gradient-checkpoint production: 30 files and 7,231 lines.
- Gradient-checkpoint maintained MATLAB tree after removing 29 redundant test-only blank
  lines: 56 files and 11,957 lines.
- Core files: solver 885 lines, planner 888 lines.

## Buffered convex-envelope membership

The final profile showed 3.74 seconds in repeated seed-envelope
`polyshape.isinterior` calls. The helper already rejects nonconvex regions and
buffers each accepted convex region by the same tolerance. The retained change
uses vectorized `inpolygon` on those exact buffered vertices. Focused inside,
outside, and concave-envelope cases pass.

Two serial accelerating-circle runs measured 22.5436799 and 23.2924850
seconds, a 22.9181-second median, versus 23.5150876 and 23.6112598 seconds
before the substitution, a 23.5632-second median. The median improvement is
2.7 percent. The final full sweep run was 23.2993549 seconds. Moving/deforming
U.S. wall decreased from 27.5321294 to 26.9813917 seconds in the serial sweep.
Small cases remain startup-noise dominated and no universal runtime claim is
made.

The complete 18-example headless gate was repeated. All metrics were exact to
the analytic-gradient source; 17 successes and the expected no-path failure
passed independent validation. The final suite passed 58 tests in 29.0953213
seconds, Code Analyzer found zero messages across 56 maintained files, visible
success produced four figures, and visible failure produced two figures with
two rejected transitions.

Final size is 7,231 production lines and 11,974 maintained MATLAB lines. The
solver is 885 lines and planner 888. The interactive sandbox remains untracked.

### Batched complete-history containment

Concatenating every canonical obstacle-history vertex reduces convex-envelope
membership from one polygon query per slice to one query per obstacle/region.
The strengthened regression rejects a history that starts inside and ends
outside, as well as outside and concave cases. All final trajectory metrics
remained exact.

Accelerating-circle serial pairs changed from 22.5436799/23.2924850 seconds
before history batching to 21.5364519/22.3062824 seconds after it, improving
the median from 22.9181 to 21.9214 seconds (4.35 percent). The final sweep run
was 22.5887965 seconds. Moving/deforming U.S. improved from 26.9813917 to
26.8349249 seconds in the final serial sweeps. The complete 18-example gate,
58 tests in 29.3425297 seconds, zero-message analysis across 56 files, and
visible success/failure checks all passed.

The final profile confirms the mechanism: envelope-related `inpolygon` calls
fell from 6,452 to 292 (95.5 percent), and envelope-helper time fell from
3.05102 to 1.92402 profiled seconds. `buildCorridor` fell from 4.73690 to
4.00813 profiled seconds. Profiler wall time is not compared to no-profile
benchmark rows.

After the resolver removed its duplicate defaults call, the exact frozen source
again passed all 58 tests in 29.1248773 seconds and Code Analyzer again found
zero messages across 56 maintained MATLAB files.

Against the complete feasibility-recovery sweep on the same branch, summed
18-example wall time decreased from 271.4097073 to 252.0683835 seconds (7.13
percent) and the median per-case reduction was 4.11 percent. Seventeen rows
were faster. Opposing-U increased from 21.4478266 to 21.7702201 seconds (1.50
percent) despite bit-exact trajectory metrics; this unfavorable single-pair
timing remains visible and no universal speedup is claimed.

A durable full-decision directional-gradient regression was then added while
removing the same number of redundant test separators. The absolute final
suite passed 59 tests in 29.2410235 seconds; Code Analyzer remained at zero
messages across 56 maintained MATLAB files. Production and maintained-tree
line counts did not increase.

The public obstacle constructor was also exercised with a row-oriented first
history slice and column-oriented second slice. Both coordinate histories were
canonicalized to columns, and the batched complete-history envelope query
returned true for a containing convex envelope.

A seeded randomized comparison across 100 convex buffered envelopes found zero
membership differences between the previous `polyshape.isinterior` predicate
and buffered-vertex `inpolygon` for 100,000 random points and all boundary
vertices.

A seeded nonuniform evaluator stress covered 5,000 random samples across 37
segments and position through jerk outputs. The maximum absolute difference
from an independent scalar loop was 1.75e-13.

A post-audit rerun of the complete test suite passed 59 tests with zero failed
or incomplete results in 26.0991124 seconds.

Relative to pushed `2074c14`, production changed from 7,139 to 7,231 lines
(+92) and the maintained planner/test tree excluding examples changed from
7,953 to 8,748 (+795).
Production is 269 lines below the user-approved 7,500-line target, so no
performance-based overage allowance is used. The solver shrank from 900 to
885 lines while the two focused internal helpers hold 111 lines.

The user then explicitly confirmed that example files have no repository line
cap and authorized tracking the 694-line interactive sandbox. The 24 example
files total 3,920 lines; the combined MATLAB tree is 12,668 lines, while the
12,000-line cap applies only to the maintained planner/test tree excluding
examples. Example files remain excluded from planner-growth claims.

Current policy: the 7,500-line production target remains unchanged, and any
production overage must earn at least a 25 percent wall-time reduction per
additional 100 production lines, using the smallest reduction in the declared
representative benchmark set. Historical checkpoints below may quote the
earlier 30-percent formula; those measurements are preserved as historical
evidence and are not the current acceptance rule.

## 2026-08-21 — 325-less-nlp evidence-gated implementation

### Scope and environment

Work ran in the isolated `325-less-nlp-implementation` worktree on branch
`325-less-nlp`, based on exact commit
`5a067112a9f880d015f52fb97538a99010871478`. MATLAB was R2024b Update 4 on
PCWIN64 with six reported cores. Optimization Toolbox 24.2 was available;
Spline/Curve Fitting functions and a parallel pool were unavailable. No commit
or push was requested or performed.

Three long inline MATLAB launches reported a transient startup `File system
inconsistency` before executing test code. A one-line runtime probe recovered
the runtime; these startup failures produced no planner or prototype result.

### Phase A — frozen HS3 scaling baseline

`benchmarkRepeatedTurnHs3([1 2 5 10 20])` ran serially with seed 325,
earliest-arrival mode, three maximum seeds, eight HS3 segments, no mesh
refinement, and a 60-second per-improvement allowance. The tracked final rows
are in `benchmarks/repeated_turn_hs3_phase_a.csv`.

| Turns | HS3 seed attempts / solves | Route vertices | Variables | Inequalities | HS3 stage (s) | Total (s) | Planner / validation | Reason |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | :---: | --- |
| 1 | 2 / 2 | 4 | 35 | 641 | 7.5631 | 9.1771 | 1 / 1 | `goalReached` |
| 2 | 2 / 2 | 7 | 35 | 706 | 13.2254 | 13.6974 | 1 / 1 | `goalReached` |
| 5 | 2 / 2 | 17 | 35 | 901 | 13.1546 | 13.3591 | 1 / 1 | `goalReached` |
| 10 | 2 / 6 | 31 | 35 | 1226 | 75.1263 | 75.3596 | 0 / 0 | `noValidatedSeed` |
| 20 | 2 / 6 | 61 | 35 | 1876 | 193.3010 | 193.8603 | 0 / 0 | `noValidatedSeed` |

The first raw five-case run was also retained in the work log rather than
silently replaced: its total walls were 10.590, 14.052, 16.965, 75.783, and
190.380 seconds. Repeat count is one because the 20-turn case alone costs
about three minutes; no median or variance claim is made.

Exact failure diagnosis found no search truncation. At 10 turns the direct
seed collides; the visibility seed is analytically time-window infeasible and
its optimized motion has -0.159396-degree minimum clearance. At 20 turns the
visibility seed is time-window infeasible and both HS3 candidates remain
nonlinear-constraint infeasible after two relinearizations each. The failures
are classified as motion construction/collision, not topology generation.

A five-turn MATLAB profile measured 22.620 seconds in HS3, 19.411 seconds in
`fmincon`, 15.363 seconds in finite-difference gradient/Jacobian work, and
14.530 seconds over 4,769 trajectory-constraint callbacks. Corridor
constraints were the largest callback component.

### Phase B — representation comparison

Research-only prototypes covered straight, 45-degree, 90-degree, S-turn,
horseshoe, and five-alternation routes. Both were exactly C3 in their scoped
checks. The quintic B-spline passed the maintained polynomial validator. The
fixed-stop septic Bezier interpolated every route vertex but was incompatible
with that quintic format and forced zero velocity, acceleration, and jerk at
every vertex.

Across five warm repeats, quintic motion durations ranged from 8.944 to 18.257
seconds on multi-turn cases; septic durations ranged from 28 to 84 seconds.
For the five-alternation route, quintic exposed 15 relative shape/timing
parameters, a flexible C3 septic would expose 35, and the restrictive
fixed-stop septic used five. Exact rows remain in
`benchmarks/spline_representation_phase_b.csv`. The rejected septic code and
candidate-only tests were removed under the bounded-experiment recovery rule.

### Phase C — deterministic low-dimensional optimizer

The retained research optimizer reduces the route by input arc length and
goal-horizon evidence, uses one normal-offset decision per interior control,
retimes analytically from continuous Bernstein derivative bounds, and uses
sampled clearance only as a search signal. Success is set only by an unchanged
`validateAzElTrajectory` call.

Final serial results from `benchmarkLowDimensionalSpline([1 2 5])` are in
`benchmarks/low_dimensional_spline_phase_c.csv`:

| Turns | Route vertices | Decisions / evaluations | Optimizer (s) | Total (s) | Motion (s) | Minimum clearance (deg) | Validation |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | :---: |
| 1 | 4 to 3 | 1 / 3 | 1.3008 | 1.8750 | 8.9443 | 0.0150470 | pass |
| 2 | 7 to 4 | 2 / 4 | 0.8039 | 1.1165 | 10.9851 | 0.0004261 | pass |
| 5 | 17 to 8 | 6 / 27 | 9.4410 | 10.5180 | 23.7683 | 0.0002560 | pass |

Relative to HS3-stage time, these are 82.80, 93.92, and 28.23 percent faster.
Their motion durations are respectively 34.96, 16.53, and 14.34 percent
longer than HS3. Those tradeoffs and the small clearance reserve prevent a
production replacement claim.

The 10-turn prerequisite failed in every bounded objective experiment. The
retained mean-penalty form took 131.70 seconds and remained colliding; a
worst-clearance form took 59.81 seconds with -0.14462-degree sampled clearance;
a per-obstacle form took 63.35 seconds with -0.10607-degree sampled clearance.
The two unsuccessful objective variants were removed. The 20-turn spline was
not run after the 10-turn gate failed.

Therefore supervised imitation, reinforcement learning, production planner
integration, and HS3 removal were skipped. No learned safety, completeness,
or optimality claim is made.

### Focused checks and size

- Code Analyzer returned zero messages for both benchmark harnesses, the
  shared scenario constructor, the quintic constructor and optimizer, and all
  retained research tests.
- The focused HS3 diagnostic format test passed 1 of 1 in 5.4625 seconds.
- The retained optimizer tests passed 3 of 3 in 3.9494 seconds after one test
  exposed and caused correction of a non-monotone route-size assumption.
- A maintained alternating-slalom headless run passed planner and independent
  validation with jerk enabled, 16.06044-degree selected polyline,
  16.75828-degree smoothed path, 12.18092-second motion, collision and
  kinematic certificates true, and 25.31959-second wall time.
- Production is 30 MATLAB files and 7,117 physical lines. The non-example
  MATLAB tree is 40 files and 11,153 lines, passing the 12,000-line limit by
  847. The solver is 894 lines and planner 888, both below 900.

### Final maintained tests and example sweep

Code Analyzer checked all 64 MATLAB files and returned zero messages. No
MATLAB line exceeded 100 characters. The complete `tests` suite passed 59 of
59 with zero failed or incomplete tests in 51.488825 seconds. The combined
focused HS3 and retained research suite separately passed 52 of 52 in
52.784031 seconds.

All 18 maintained examples ran headlessly and serially in separate successful
MATLAB processes. Jerk constraints were enabled in every row. `P/V` is planner
success / independent example validation; `C/K` is collision freedom /
kinematic certificate.

| Example | Goal mode | P/V | Polyline (deg) | Smoothed (deg) | Duration (s) | C/K | Wall (s) | Reason |
| --- | --- | :---: | ---: | ---: | ---: | :---: | ---: | --- |
| `exampleAlternatingSlalom` | earliest | 1/1 | 16.060439635036 | 16.758281982866 | 12.180917402175 | 1/1 | 21.5868513 | `goalReached` |
| `exampleAzElPlanning` | earliest | 1/1 | 11.152119519024 | 11.411616815005 | 7.816267856881 | 1/1 | 17.4983344 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.700721559528 | 13.431299536656 | 8.797638855700 | 1/1 | 23.9031665 | `goalReached` |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685255 | 126.114009817632 | 64.555779916429 | 1/1 | 8.7493230 | `goalReached` |
| `exampleFourAcceleratingCircles` | fixed | 1/1 | 20.000000000000 | 20.372412421524 | 22.000000000000 | 1/1 | 38.5619066 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | fixed | 1/1 | 9.538940546821 | 9.538940546821 | 12.000000000000 | 1/1 | 7.0938467 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | earliest | 1/1 | 10.097524449092 | 7.342498781519 | 6.274806792200 | 1/1 | 6.5559756 | `goalReached` |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10.000000000000 | 10.134483812277 | 10.544227895142 | 1/1 | 35.7830259 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12.113593184851 | 12.113593184851 | 12.293137410146 | 1/1 | 20.2116419 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 63.084805146555 | 71.436692325459 | 12.986386910606 | 1/1 | 44.6775511 | `goalReached` |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN/NaN` | 20.1083406 | `noValidatedSeed` |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.472135955000 | 4.484905564719 | 4.612405963436 | 1/1 | 5.5753072 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10.000000000000 | 10.000000000000 | 15.000000000000 | 1/1 | 20.4914773 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | fixed | 1/1 | 21.403170279055 | 14.220154562476 | 20.869565217391 | 1/1 | 27.1701093 | `goalReached` |
| `exampleTargetExitsObstacle` | fixed | 1/1 | 19.824386758954 | 22.879868072534 | 24.000000000000 | 1/1 | 8.4301465 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 23.853720883753 | 24.370904895056 | 22.875124576026 | 1/1 | 47.3626836 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.942588040466 | 43.259235381251 | 22.818548735851 | 1/1 | 16.2122849 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.239463508699 | 26.617251754385 | 6.683971648809 | 1/1 | 63.6908664 | `goalReached` |

The visible success rerun of `exampleObstacleFreeAzElMotion` passed and created
three visible figures in 14.3139553 seconds. The hidden failure rerun of
`exampleNoPathAzElMotion` passed its expected-failure validation and created
two diagnostic figures with one expanded state and two rejected transitions
in 33.3814964 seconds.

Before these serial invocations, an attempted PowerShell loop launched 18
MATLAB processes too rapidly; every process failed during MATLAB startup with
`File system inconsistency` before executing example code. Two later reporting
wrappers also used stale result/diagnostic field names after the example ran;
the affected forty-circle and no-path cases were rerun successfully with the
current format. These environment/reporting failures are not counted as
example passes or planner failures.

`exampleFourAcceleratingCircles` emitted extensive `Matrix is close to
singular or badly scaled` warnings from the interior-point linear systems.
Planner and independent validation still passed, but the warning is retained
as a numerical-conditioning issue rather than suppressed from the record.
## Bounded route-interpolation recovery — 2026-08-21

Frozen source and command:

```text
41f2f92fc15b7522360ee7044c670308aaa1bf44
report=benchmarkLowDimensionalSpline(10,struct('PrintProgress',true));
```

The pre-edit baseline returned `Success=false`, `maximumSweeps`, 31 original
and 14 reduced route vertices, 12 decisions, 145 evaluations, 43.8440379
seconds motion duration, -0.011725083 degrees minimum clearance, and 90.3091
seconds total wall time.

The single candidate made the interior B-spline knot targets interpolate the
input route. MATLAB Code Analyzer reported zero messages for the three touched
research files, and all five focused representation tests passed in 1.8834
seconds. The equivalent 10-turn run nevertheless returned `Success=false`,
`maximumSweeps`, 31 original and 8 reduced route vertices, 6 decisions, 70
evaluations, 44.7352610 seconds motion duration, -0.544570324 degrees minimum
clearance, and 55.7710 seconds total wall time. Runtime improved, but physical
feasibility regressed materially, so the candidate failed the primary gate.

The bounded-experiment recovery removed the interpolation option, linear
solve, diagnostics, and focused test. `git diff` then showed no source or test
change. The exact baseline command was rerun and reproduced 14 reduced route
vertices, 12 decisions, 145 evaluations, 43.8440379 seconds motion duration,
-0.011725083 degrees clearance, and `maximumSweeps`; total wall time was
95.4536 seconds. The deterministic result recovered exactly, while the timing
difference is treated as ordinary process variation. No candidate algorithm
or benchmark artifact remains.

## Five bounded spline-option experiments — 2026-08-21

Every measured run used commit `41f2f92`, seed 325, the frozen 10-turn static
scenario, a 60.5-second horizon, unchanged code, and maintained independent
validation. Each candidate changed one optimizer option in a fresh MATLAB
process. The complete gate required exact validation, at least 0.02 degrees
continuous clearance, and less than 95.4536 seconds total wall time.

| Candidate | Reduced vertices | Decisions | Evaluations | Wall (s) | Clearance (deg) | Exact validation | Decision |
| --- | ---: | ---: | ---: | ---: | ---: | :---: | --- |
| `TimingReserveFraction=1.0` | 24 | 22 | 156 | 140.7407 | 0.000144580 | pass | reject: reserve and runtime |
| `DurationWeight=0.01` | 14 | 12 | 145 | 92.4190 | -0.011725083 | fail | reject: baseline-equivalent |
| `CollisionPenaltyWeight=5000` | 14 | 12 | 145 | 93.5085 | -0.011725083 | fail | reject: baseline-equivalent |
| `InitialStepFraction=0.1` | 14 | 12 | 145 | 93.0052 | -0.393003257 | fail | reject: worse clearance |
| `MaximumNormalOffset_deg=5` | 14 | 12 | 145 | 98.0729 | -0.353568844 | fail | reject: worse clearance/runtime |

The initially proposed `DurationWeight=0` call was rejected by input
validation before planning because the option requirement requires a positive
weight. It was corrected once to the requirement-valid 0.01 value; no parameter
sweep followed.

The exact default recovery command then reproduced 14 reduced vertices,
12 decisions, 145 evaluations, 43.8440379-second motion duration,
-0.011725083-degree clearance, and `maximumSweeps`. Its 109.0203-second total
wall time is reported but not interpreted as a deterministic regression.
No candidate met the complete gate, so no 1/2/5 non-regression runs, default
changes, source changes, production integration, or learned-policy work were
performed.

## Feasibility-first corridor spline experiment — 2026-08-21

The bounded candidate reused the maintained seed-corridor builder, Bernstein
half-space inequalities, and independent corridor certificate. Its search
objective ranked maximum normalized corridor violation ahead of duration,
jerk, and offset quality, and required at least 0.02 degrees clearance.

MATLAB Code Analyzer initially reported zero messages for the three candidate
files. A missing single-obstacle envelope was corrected mechanically by
assembling canonical protected obstacle slices; no search parameter was tuned.
The unchanged focused one-turn proof then returned `Success=false`, validation
false, no corridor certificate, and `NaN` continuous clearance. Because the
candidate failed the smallest feasibility proof, the frozen 10-turn candidate
gate was not run.

All candidate MATLAB and test edits were removed. `git diff --name-only --
'*.m'` was empty and `git diff --check` passed. The exact default recovery run
reproduced 31-to-14 route reduction, 12 decisions, 145 evaluations,
43.844037904 seconds motion duration, -0.011725083225 degrees clearance, and
`maximumSweeps`. Reported total wall time was 106.9760 seconds. A post-run
reporting expression then used invalid one-subscript table indexing, but only
after the benchmark had printed and completed; it did not alter the measured
planner result.

## Worst-clearance-first spline objective — 2026-08-22

The retained 10-turn trace showed a ranking weakness: its selected candidate
had -0.746881006 degrees sampled clearance at 16 seconds against barrier 4,
while the zero-offset evaluation was less deeply colliding at -0.102462142
degrees. A bounded research-only candidate therefore separated infeasible
clearance/horizon ranking from duration and jerk quality and ranked the worst
normalized clearance deficit ahead of its mean squared tie-break.

Code Analyzer returned zero messages and all three focused optimizer tests
passed. The identical 10-turn benchmark nevertheless failed exact validation
with -0.075958561 degrees continuous clearance, a 43.740088788-second motion,
145 evaluations, `stepTolerance`, and 100.5287 seconds total wall time. The
candidate was rejected because it failed the 0.02-degree reserve and 95-second
runtime gates.

All candidate MATLAB edits were removed. The exact default recovery reproduced
14 reduced vertices, 12 decisions, 145 evaluations, 43.844037904 seconds
motion, -0.011725083225 degrees clearance, `maximumSweeps`, and 85.4054 seconds
total wall time. One preceding MATLAB launch failed at startup with `File
system inconsistency` and produced no planner result.

### Earliest-stage and low-D performance diagnosis

At 0.005-degree edge resolution, the original 31-vertex visibility route had
no occupied samples and 0.000999997 degrees minimum clearance. Uniform
arc-length reduction to 14 vertices created 3,111 occupied samples and
-0.093841740 degrees minimum clearance at barrier 8. The retained reducer thus
supplies a geometrically invalid route before spline construction; later
collision and ranking failures are symptoms rather than the earliest cause.

A MATLAB profile of the retained 5-turn case reported a valid 8.6189-second
total run. Low-D optimization used 7.5682 seconds, candidate evaluation used
7.0074 seconds, and sampled clearance used 6.0315 seconds across 28 calls.
Those calls caused approximately 22,000 obstacle-shape and point-polygon
evaluations. Static clearance batching is therefore the measured runtime
mechanism relevant to the next representation experiment.

## Static batching and protected-route experiments — 2026-08-22

The retained research-only optimization batches sampled clearance for exactly
static protected obstacle histories. Four focused optimizer tests passed,
including equality with the public query for static selected samples and the
unchanged public-query path for a moving triangle. Code Analyzer reported zero
messages for the optimizer and test. The 5-turn candidate reproduced the exact
prior decision, 23.768318698-second duration, 1.201967415 integrated jerk, and
0.000256039918-degree independent clearance; optimizer wall time was 1.4484
seconds versus the prior recorded 9.4410 seconds. No broad scaling claim is
made from this focused first-run comparison.

Three bounded 10-turn route/reserve variants were then rejected:

| Candidate | Route | Evaluations | Motion (s) | Clearance (deg) | Wall (s) | Result |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| topology-preserving subsequence | 31->22 | 77 | 60.449031 | 0.000912303 | 5.22298 | validation pass; reserve fail |
| expanded protected subsequence | 31->20 | 71 | 54.502822 | 0.000154950 | 8.45032 | validation pass; reserve fail |
| reserve-aware mean search | 31->20 | 217 | 54.374347 | 0.000785581 | 78.35975 | reserve fail; maximum sweeps |
| reserve-aware worst deficit | 31->20 | 217 | 54.342181 | 0.000342955 | 107.35371 | reserve and wall fail |

The topology reducer passed a structurally different protected-rectangle test,
and the expanded reducer preserved at least 0.02 degrees sampled polyline
clearance in that test. The unchanged continuous validator nevertheless showed
that the resulting noninterpolating quintic spans approached the benchmark
obstacles much more closely than their geometric routes. This localizes the
remaining defect to motion construction/corridor enforcement rather than route
topology.

All topology, expansion, reserve-acceptance, worst-deficit, diagnostic, helper,
and associated test edits were removed. Code Analyzer returned to zero messages.
The exact 10-turn recovery reproduced 14 reduced vertices, 12 decisions, 145
evaluations, 43.844037904-second motion, -0.011725083225-degree clearance,
`maximumSweeps`, and 2.359616 seconds total. Only the independently verified
static-clearance batching and its static/moving equivalence tests remain.

## Affine full-span corridor prototype — 2026-08-22

The existing `seedCorridorInequality` and `certifySeedCorridor` functions were
reused rather than duplicated. The missing inputs were complete convex
protected-history envelopes and correct ordered-edge-to-span correspondence.
With arc-length correspondence, the 31-vertex two-axis system was infeasible;
uniform ordered-edge correspondence made all 300 records feasible. That full
route certified 0.02 degrees clearance but required 65.9380 seconds.

Topology-preserving route trials then established the representation boundary:
20 and 21 expanded vertices met timing potential but their corridor systems
were infeasible, while 22 vertices were feasible. Edge-length timing weights
gave 60.7119 seconds. The general span-length allocation
`edgeLength_deg.^1.05` reduced the certified motion to the final retained gate:

```text
corridor10 validation=1 certificate=1
clearance_deg=0.0200000000000009
duration_s=60.2576877911092
wall_s=4.0949002
```

The retained `planCorridorConstrainedQuinticPrototype` uses 40 bounded two-axis
control-point variables and 210 complete Bernstein corridor records for that
request. The quadratic solve itself took about 0.18 to 0.20 seconds in repeated
proof runs; end-to-end method time includes route proposals, basis construction,
certificate, and independent validation.

MATLAB Code Analyzer returned zero messages for the new function and test.
Four focused tests passed: the frozen 10-turn gate, a structurally different
rectangle, a stable blocked-shortcut failure, and a moving-triangle history
envelope. The retained constructor and coordinate-optimizer suites also passed,
for 12 research tests total. The moving test was rerun warning-free after exact
history-coordinate deduplication and trusted convex-hull construction.

This is a research result, not production replacement evidence. No maintained
example or production planner calls the new method, no broad scaling matrix was
run, and the explicit 22-vertex benchmark override and 1.05 exponent are not
claimed optimal or complete for unseen route families.

## Repeated-hairpin feasibility and topology fallback — 2026-08-22

The declared stress input is a parameterized alternating-end maze with 12
horizontal protected walls. Consecutive openings alternate between workspace
edges, forcing eleven lateral reversals approaching 180 degrees. The initial
command was:

```matlab
benchmarkHairpinCorridorQuintic(12, struct( ...
    'FigureVisible', 'off', 'RouteVertexCount', 22));
```

Before the topology change, the request produced no visibility seed: 50 nodes,
47 accepted edges, five homology states, five expanded states, 166 rejected
transitions, and no truncation. One wall succeeded, while every case from two
through twelve walls failed at the same graph-connectivity boundary.

The retained topology change preserves the original Delaunay-sparse graph when
it connects start to goal. Only when that graph is disconnected does it use all
candidate pairs, and only when `candidatePairCount * boundaryEdgeCount` fits
the existing one-million-work budget. This leaves successful sparse seed order
unchanged. Two focused tests passed: the new two-wall reversal route and the
existing two-obstacle homology-diversity case. The complete planner test file
then passed 46/46 tests in 18.4028 seconds.

The first post-topology 12-wall route contained 50 protected-clear vertices.
The route-count target grew deterministically from 22 to the first clear
26-vertex subsequence, but that affine full-span corridor was infeasible. The
retained feasibility-first recovery retries once with the complete input route
when compressed corridor feasibility fails. The final result was:

```text
hairpins=12 route=50->50 success=1 validation=1 certificate=1
duration_s=369.337421187 clearance_deg=0.02
candidate_s=16.0761214 reason=corridorPrototypeValidated
compression target=22 first clear=26 fallback=50 C3=1
maximum velocity ratio=0.446277235527
maximum acceleration ratio=0.189227125012
maximum jerk ratio=0.0677886650004
```

The six focused corridor-prototype tests passed. The frozen 10-turn gate stayed
independently valid and corridor-certified at 60.2576877911092 seconds,
0.0200000000000009 degrees clearance, and 4.048403 seconds wall time. The
structurally different rectangle, moving-history envelope, adaptive target
growth, and unclear-source-route failure checks also passed.

The 20-turn scaling case remains unfavorable. Recovery reaches a certified
61-vertex spline with 0.02 degrees clearance, but its 122.474368665-second
arrival exceeds the 115.5-second horizon. It returns
`trajectoryValidationFailed` after 38.3098911 seconds rather than hiding the
deadline violation. This is a timing-quality limitation, not a collision or
span-stop failure.

Eighteen noninteractive maintained examples were run headlessly in separate
MATLAB processes. Seventeen planner successes passed independent validation,
collision checks, and all modeled kinematic certificates. The expected no-path
example returned `noValidatedSeed`, passed its example-level failure check, and
created two hidden diagnostic figures without a selected trajectory. A
separate visible `exampleAzElPlanning` run passed and created two figures. Full
per-example metrics and wall times are appended to `benchmark.csv` under source
`41f2f92+hairpin-worktree`. `exampleAzElInteractiveSandbox` was not executed
because it blocks for live mouse-drawn inputs. `exampleFourAcceleratingCircles`,
`exampleOpeningUShapedAzElTimeSpace`, and
`exampleStraightTargetAlternatingOcclusion` emitted extensive near-singular
fmincon warnings despite valid final results; `exampleMovingBarrierWait`
emitted one such warning.

MATLAB Code Analyzer reported zero messages for the changed topology generator,
planner test, corridor prototype/tests, and both new benchmarks. The topology
generator was reduced from 924 to 888 lines after replacing a local traversal
with MATLAB graph connectivity and dropping redundant diagnostics.

Repository size remains a blocking compliance issue for production replacement:
current production MATLAB is 9,206 lines versus the 7,000 target, and the
complete MATLAB tree is 16,483 versus the 12,000 hard cap. `HEAD` was already
8,389 and 14,690 lines respectively. No performance claim is used to waive the
hard tree cap. Generated MAT/CSV/PNG evidence is untracked, and no commit or
push was performed.

## Corridor-only final verification — 2026-08-22

- Static checks: MATLAB Code Analyzer returned zero messages for the public
  planner and touched corridor/topology helpers. `git diff --check` passed.
- Automated suite: `runtests('tests','IncludeSubfolders',true)` passed 53/53
  after the legacy implementation and its direct tests were removed.
- Maintained examples: all 18 noninteractive examples ran in separate fresh
  MATLAB processes. Seventeen successes passed independent validation and
  selected `corridorQuintic`; `exampleNoPathAzElMotion` returned the expected
  validated `noValidatedSeed`. All 18 passed the zero-HS3-attempt gate. Exact
  metrics are in `benchmark.csv` under `working-tree-corridor-only`.
- Generality guards: disconnected fixed-time target, expected no path,
  connected slalom, static/dynamic timing, obstacle-free exact jerk switching,
  and the 10-hairpin corridor passed. The target arrived at exactly 24 seconds;
  the hairpin motion retained 0.02-degree certified clearance without stops.
- Graphics: a visible slalom run produced three valid figures. A headless
  no-path run produced two diagnostic figures from returned diagnostics.
- Size: core production excluding plotting is 6,954 lines; plotting is 565;
  examples are 3,910; tests are 1,360; benchmarks are 830; and the maintained
  tree excluding examples/scratch is 9,709. The largest production file is
  `generateAzElTopologySeeds.m` at 900 lines. No allowance is required.
- Limitations: finite topology/time work bounds mean no completeness or global
  optimum claim. Direct zero-derivative obstacle-free motion uses an exact C2
  jerk-switching profile; multi-turn spline motion remains C3 continuous. The
  interactive sandbox was not run.

## Span-demand timing controller — 2026-08-22

The retained timing controller converts each quintic span's measured velocity,
acceleration, and jerk utilization to a common local time demand and applies
bounded proportional feedback to normalized log span durations. Every trial is
rebuilt by the existing affine corridor solver and must pass the unchanged
independent validator. The controller permits at most 16 trials and stops after
one probe when the initial arc-length allocation does not improve arrival.

On the same headless single-U request, the prior 180-trial coordinate search
returned 24.740511444152 seconds in 32.890 seconds wall time. The controller
returned 24.973219952131 seconds in 5.991363 seconds with 11 trials: 5.49 times
faster for a 0.94 percent arrival penalty. The motion remained independently
valid and collision-free with approximately 0.0001 degrees clearance. Exact
peaks were `[1.978202183578 1.983598081965] deg/s`,
`[0.749909506789 0.745360921560] deg/s^2`, and
`[0.976970345530 0.975378311450] deg/s^3`. Three repeated controller motions
were bit exact; warm wall times were 3.440425 and 3.168312 seconds.

The full automated tree passed 53/53 in 17.9147 seconds. All 18 maintained
examples then ran serially in fresh MATLAB processes: 17 independently valid,
collision-free successes and the independently valid expected no-path result,
all with zero HS3 time. The single U was 24.9732199521 seconds in 6.053188
seconds, opposing U was 31.9439273474 seconds in 13.236511 seconds, and
alternating slalom was 13.2008531355 seconds in 6.327214 seconds. Dense
concavity remained exactly 12.1408011078 seconds but took an unfavorable
49.841631 seconds; its selected result used zero controller trials, localizing
that cost to exact concave-corridor construction rather than controller work.
A visible success created four figures and a hidden expected failure created
two returned-diagnostic figures. The interactive sandbox was not run.

Final recount is 7,171 core production lines excluding 565 plotting lines and
9,928 maintained lines excluding examples/scratch. The largest production file
is 900 lines, and the maintained-tree 12,000-line cap passes. Core production is
171 lines above the 7,000-line target. A 51.3-percent minimum wall reduction
would be required for a size allowance, but the dense-concavity regression
prevents such a branch-wide claim; size therefore remains a completion blocker.

## Monotone controller and batched affine corridor evidence — 2026-08-22

The fixed-gain controller trace decreased certified U duration through its
fifth update and then increased it. A retained objective-monotone stop removes
all later trials. An immediate isolated-process A/B pair used identical inputs
and validation: the original 11-trial controller took `11.7948769 s`; the
six-trial controller took `10.3352666 s`. Both selected exactly
`24.973219952131 s` motion. Gain-only, route-expansion, smoothing-QP,
geometric-jerk-QP, and Bernstein peak/scale trials were unfavorable and were
fully removed.

Profiling the frozen 12-wall hairpin localized `19.4845` of `22.7422` profiled
solver seconds to protected-route visibility sampling, principally repeated
obstacle normalization in 1,225 edge queries. Visibility samples now retain
the same coordinates and occupancy policy but are queried once per start
vertex. After batching, repeated unit-offset spline construction became the
dominant stage. For zero endpoint velocity and acceleration, one exact affine
control-point-to-polynomial map now supplies the corridor Jacobian. Nonzero
endpoint derivatives retain the original full path; empty corridors skip the
absent Jacobian.

The frozen hairpin candidate baseline was `17.2736906 s`. Three final 12-wall
candidate times were `8.3679933`, `8.1033994`, and `8.5242998 s`; median
`8.3679933 s` is a `51.56%` reduction (`2.064x`). Corresponding total walls
were `9.7478098`, `9.5022615`, and `10.0311034 s`, median `9.7478098 s`.
Every result independently validated, remained collision-free and corridor-
certified, retained `0.02 deg` clearance, and returned bit-identical
`164.828287993221 s` motion. The final recorded run was:

```text
HAIRPIN_FINAL success=1 validation=1 totalWall=9.9147208
candidate=8.4738678 duration=164.828287993221
polyline=174.460973313305 smooth=190.315066870802
collision=1 clearance=0.02 cert=1 route=50 fallback=1
```

The final fresh U run used jerk constraints, six controller trials, and zero
HS3 time. It returned planner and independent-validation success, a
`34.942588040466 deg` polyline, `36.518869120532 deg` sampled smoothed length,
`24.973219952159 s` motion duration, collision freedom, and
`0.0000999999983853 deg` minimum clearance in `8.3241566 s` wall time. The
motion remains `9.44%` slower than the frozen main-branch
`22.818548735851 s` reference, so it misses the required `5%` limit of
`23.959476172644 s`. Runtime and hairpin gates pass; arrival quality does not.

Code Analyzer reported zero messages for the four affected MATLAB files. The
complete automated tree passed `53/53` in `27.2821021 s`. A full maintained-
example rerun was not performed after this runtime-only change; the interactive
sandbox also remains untested. The new converter file has 142 physical lines,
but it moves the prior local conversion responsibility out of
`buildAzElQuinticSpline`; exact request-local net growth cannot be separated
reliably from the already-untracked corridor replacement worktree.
Current recount is 7,267 core production lines, 565 plotting lines, and 10,024
maintained lines excluding examples/scratch. The maintained-tree cap passes,
but core production is 267 lines above the 7,000-line target.

## Exact continuous-derivative retimer — 2026-08-22

This section supersedes the immediately preceding U-gate result. The retained
method is control-theoretic rather than a gain sweep. For a bounded small
static system, it represents each continuous velocity, acceleration, and jerk
polynomial as an affine function of corridor offsets. LP bisection minimizes a
common derivative time scale; exact stationary points of the returned
polynomials are exchanged back as constraints. Subsequent span-time feedback
uses a bounded secant gain. Every retained trial still passes the independent
continuous collision and exact polynomial kinematic validator.

The input-driven eligibility rule requires earliest arrival, static geometry,
zero endpoint derivatives, and at most 100 decision-span work units. A bounded
experiment increased this cap to 200. Slalom duration remained exactly
`13.200853135488 s`, exact exchange was not retained by the selected result,
and wall increased to `6.2049921 s`; the cap change was fully reverted.

Focused final evidence:

- U: `23.958753333083 s`, `5.9070831 s` focused wall, four timing trials, 22
  LP solves, exact exchange accepted, collision-free with approximately
  `1e-4 deg` clearance. The duration passes the frozen five-percent limit
  `23.959476172644 s` by `0.000722839561 s`.
- Alternating slalom: `13.200853135488 s`, `4.8009601 s` focused wall,
  independently valid and path-identical to the retained legacy controller.
- Opposing U: `28.575922291343 s`, `4.9860068 s` focused wall, independently
  valid; the prior controller result was `31.9439273474 s`.
- 12-wall hairpin: `164.828287993221 s` duration, `6.3611859 s` candidate,
  `6.9003699 s` total wall, `0.02 deg` clearance, corridor certificate and
  independent validation passed. Exact exchange correctly remained outside
  its bounded work envelope.

All 18 maintained examples then ran headlessly, serially, and in separate
MATLAB processes. Jerk constraints were enabled in every row. `P/V` is planner
success / independent validation; `C/K` is collision / kinematic certificate.

| Example | Mode | P/V | Polyline (deg) | Smoothed (deg) | Duration (s) | C/K | Wall (s) |
| --- | --- | :---: | ---: | ---: | ---: | :---: | ---: |
| `exampleAlternatingSlalom` | earliest | 1/1 | 16.0604396350 | 16.0939038492 | 13.2008531355 | 1/1 | 5.0060698 |
| `exampleAzElPlanning` | earliest | 1/1 | 11.1521195190 | 11.3711961911 | 7.87966799510 | 1/1 | 3.7547719 |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.7007215595 | 12.7825547226 | 12.1408011078 | 1/1 | 63.7774602 |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685 | 110.720320521 | 105.698249574 | 1/1 | 8.4310602 |
| `exampleFourAcceleratingCircles` | fixed | 1/1 | 24.3633030073 | 24.1898172734 | 22 | 1/1 | 7.4556229 |
| `exampleInterceptMovingTargetAtSetTime` | fixed | 1/1 | 9.53894054682 | 9.53894054682 | 12 | 1/1 | 1.9469955 |
| `exampleInterceptMovingTargetEarliest` | earliest | 1/1 | 7.47189347130 | 7.47189347130 | 6.92326129150 | 1/1 | 2.5638684 |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10 | 10 | 10.9963769559 | 1/1 | 4.1996987 |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12 | 12 | 13.6192201977 | 1/1 | 4.8828633 |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 85.2741687071 | 86.3554199630 | 21.1566336854 | 1/1 | 38.1883850 |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN/NaN` | 1.4665151 |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.47213595500 | 4.47213595500 | 4.53112887415 | 1/1 | 1.7182018 |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10 | 10 | 12.1926012246 | 1/1 | 5.2892453 |
| `exampleStraightTargetAlternatingOcclusion` | fixed | 1/1 | 21.4031702791 | 13.7887424080 | 20.8695652174 | 1/1 | 7.0890508 |
| `exampleTargetExitsObstacle` | fixed | 1/1 | 20.1815464898 | 21.7103196601 | 24 | 1/1 | 6.4284400 |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 24.5077116377 | 24.5056885160 | 28.5759222913 | 1/1 | 8.2111642 |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.9425880405 | 35.8847577549 | 23.9587533331 | 1/1 | 5.3970581 |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.3733117302 | 22.4077742797 | 14.9628027709 | 1/1 | 36.1710905 |

All durations are preserved or improved relative to the preceding
corridor-only matrix. The frozen main/HS3 durations remain materially better
for several difficult static and moving cases, so an all-example
meet-or-beat claim is not made. The dense-concavity wall result is unfavorable
and retained as a separate performance problem.

Verification commands and results:

- `runtests('tests','IncludeSubfolders',true)`: 54/54 passed in 23.762344 s.
- Focused exact-retimer test file: 4/4 passed in 9.834277 s.
- Code Analyzer: 65 maintained MATLAB files, zero messages.
- `git diff --check`: no whitespace errors; only existing LF/CRLF notices.
- Visible obstacle-free success: four figures, 643 graphics objects, valid
  `4.531128874149 s` motion.
- Hidden expected no-path: two diagnostic figures, 341 graphics objects, two
  rejected transitions, independently valid `noValidatedSeed` result.

Final size is 7,312 core production lines excluding 565 plotting lines and
10,013 maintained MATLAB lines excluding examples/scratch. Each production
file is at most 900 lines. The 12,000-line maintained cap passes; core remains
312 lines over its 7,000-line target. The new 184-line
`optimizeAzElExactTraversal.m` owns exact derivative exchange so the main
corridor solver remains 745 lines; this is a new production responsibility,
not hidden inside an oversized solver. The interactive sandbox remains
untested.

## Active-set QP and ten-level exchange — 2026-08-22

This section supersedes the preceding exact-retimer timings. The frozen
performance environment was MATLAB R2024b Update 4 on an AMD64 Family 23
Model 113 CPU, with no Parallel Computing Toolbox/pool, deterministic seed
325, hidden figures, and unchanged example inputs and validation.

The declared performance cases were dense static concavity and the
moving/deforming U.S. outline. Profiling the dense baseline measured
`63.6346615 s` wall, including `60.4469126 s` in two dense interior-point
`quadprog` calls, `1.0777094 s` in spline construction, `0.5621513 s` in
validation, and `0.4179040 s` in seed generation. The driver was the affine
corridor inequality system, not topology or collision queries.

The retained solver first obtains an LP-feasible bounded point. Proven LP
infeasibility now skips the unnecessary QP; feasible systems use active-set
`quadprog` from that point. Dense concavity returned the bit-identical
`12.140801107795 s` duration with independent validation. Candidate walls
were `3.5709276`, `3.8215486`, and `3.6560574 s`; the final isolated matrix
recorded `3.7330015 s`, a 94.15-percent reduction from the prior
`63.7774602 s` isolated row. No topology, geometry, safety margin, limit, or
arrival policy changed.

Increasing exact derivative exchange from eight to ten bisection levels
improved U from `23.958753331534 s` to `23.801121658178 s`. A focused wall was
`5.9427083 s`, and the final isolated wall was `6.0206867 s`. Twelve levels
returned an unfavorable `24.464044542719 s` controller basin and were
immediately reverted; ten levels then reproduced bit exactly. Opposing-U
retained `28.575922287848 s`.

The 12-wall hairpin remained independently valid and corridor-certified at
`164.828287993153 s`, `0.02 deg` clearance, `6.6162314 s` candidate time, and
`7.1631349 s` total wall.

### Rejected dynamic and static experiments

- Applying the full timing controller to every validated dynamic candidate
  selected the historical `63.0848051466 deg` seed and improved duration from
  `21.1566336854` to `17.8606111859 s`, but wall increased to
  `123.0861456 s`; the change was removed and the baseline reproduced.
- A path-parameterized unit-gain fixed-point prototype avoided repeated
  obstacle queries and produced a valid `17.8505455378 s` motion in
  `3.1016398 s` of retimer work. It still missed the frozen
  `12.986386910606 s` reference and was not added to production.
- Unconstrained exact geometry exchange on the same dynamic seed accepted a
  60.55-percent derivative-scale reduction but crossed moving protected
  geometry; it was removed.
- Re-evaluating timing eligibility after the dense static fallback and exact
  geometry exchange on its 40-work system produced no duration improvement;
  both experiments were removed.
- The existing `codex/paper-retimer` branch contains a 918-line Debrouwere
  sequential-convex retimer, but its small-grid path invokes `fmincon`; it was
  not imported into this non-NLP, size-constrained branch.

### Final isolated maintained-example matrix

Every example used jerk constraints and zero HS3/NLP time. `P/V` is planner /
independent validation and `C/K` is collision / kinematic certificate.

| Example | Mode | P/V | Polyline | Smoothed | Duration | C/K | Wall |
| --- | --- | :---: | ---: | ---: | ---: | :---: | ---: |
| `exampleAlternatingSlalom` | earliest | 1/1 | 16.0604396350 | 16.0939038492 | 13.2008531355 | 1/1 | 5.6057875 |
| `exampleAzElPlanning` | earliest | 1/1 | 11.1521195190 | 11.3621319718 | 7.87638446302 | 1/1 | 3.7984968 |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.7007215595 | 12.7825547226 | 12.1408011078 | 1/1 | 3.7330015 |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685 | 110.720320521 | 105.698249574 | 1/1 | 8.1444993 |
| `exampleFourAcceleratingCircles` | fixed | 1/1 | 24.3633030073 | 24.1898172734 | 22 | 1/1 | 6.9149601 |
| `exampleInterceptMovingTargetAtSetTime` | fixed | 1/1 | 9.53894054682 | 9.53894054682 | 12 | 1/1 | 1.9762920 |
| `exampleInterceptMovingTargetEarliest` | earliest | 1/1 | 7.47189347130 | 7.47189347130 | 6.92326129150 | 1/1 | 2.9338914 |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10 | 10 | 10.9963769559 | 1/1 | 4.1621376 |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12 | 12 | 13.6192201977 | 1/1 | 5.0249601 |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 85.2741687071 | 86.3554199630 | 21.1566336854 | 1/1 | 37.3056387 |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN/NaN` | 1.6125348 |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.47213595500 | 4.47213595500 | 4.53112887415 | 1/1 | 1.8623578 |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10 | 10 | 12.1926012246 | 1/1 | 5.2572565 |
| `exampleStraightTargetAlternatingOcclusion` | fixed | 1/1 | 21.4031702791 | 13.7887424080 | 20.8695652174 | 1/1 | 7.1288445 |
| `exampleTargetExitsObstacle` | fixed | 1/1 | 20.1815464898 | 21.7103196588 | 24 | 1/1 | 6.6824366 |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 24.5077116377 | 24.5056885156 | 28.5759222878 | 1/1 | 8.2537591 |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.9425880405 | 35.8186005827 | 23.8011216582 | 1/1 | 6.0206867 |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.3733117302 | 22.4077742797 | 14.9628027709 | 1/1 | 34.8486747 |

The isolated wall sum decreased from `211.9775617` to `151.2662157 s`, a
28.64-percent aggregate reduction dominated by dense concavity. The smallest
per-example reduction was negative because fresh-process timing varied; no
uniform speedup or production-size allowance is claimed. Durations were
preserved within numerical roundoff except improvements to U and
`exampleAzElPlanning`.

Final checks: 54/54 tests passed in `24.886636 s`; Code Analyzer returned zero
messages for the three affected production files; `git diff --check` found no
whitespace errors beyond existing line-ending notices. Core production is
7,333 lines excluding 565 plotting lines, and maintained MATLAB excluding
examples/scratch is 10,034 lines. The 12,000-line and 900-line caps pass; core
remains 333 lines above target. The all-example historical duration goal and
core-size target remain open.

## Projection consolidation and control-law boundary — 2026-08-22

Repeated nearest-boundary projection was consolidated through the maintained
`pointPolygonClearance` invariant in static corridor construction, dynamic
seed expansion, and route-clearance expansion. One-use maximum, empty-record,
best-partial, and static-geometry helpers were inlined without changing their
requirements. Focused U, hairpin, moving/deforming, obstacle-free, and no-path
runs remained independently valid; U recovered at
`23.801121658982 s`, and the hairpin remained
`164.828287993153 s` with `0.02 deg` certified clearance.

The complete automated tree then passed 54/54 in `25.0606 s`, and Code
Analyzer returned zero messages for all 65 maintained MATLAB files. Core
production is now 7,186 physical lines excluding 565 plotting lines;
maintained MATLAB excluding examples/scratch is 9,887 lines. The 900-line
per-file and 12,000-line maintained caps pass, while core remains 186 lines
above target.

A bounded controller experiment tested the control-theoretic unit log-time
gain implied by derivative scaling. It improved U from `23.801121658982` to
`23.746859860594 s`, but planning regressed from `7.876384463025` to
`7.879667995135 s` and slalom from `13.200853135545` to
`13.233991799907 s`; it was removed. A smooth exact-exchange gain schedule
improved U to `23.752030563984 s` but regressed planning to
`7.877654207014 s`. A two-band schedule then entered a worse
`24.277638906889 s` U basin. Both schedules were removed. The retained
controller reproduced U at `23.801121658982 s`, planning at
`7.876384463025 s`, slalom at `13.200853135545 s`, and opposing U at
`28.575922287848 s` with independent validation.

The diagnostic secant trace explains the boundary. U began with log-demand
error norm `0.39193` and subsequently accepted gains `0.72343`, `0.42026`,
`1.0`, and `0.71226`; planning began near balance at `0.00807`; slalom did
not accept exact exchange and converged under the fixed gain. Because each
corridor QP may change geometry, this is a switched, nonsmooth response—not a
fixed plant whose optimal gain depends only on derivative order or error
magnitude. A future approach must estimate geometry response or enforce
trust-region acceptance; further scalar gain guesses are excluded.

## Retained one-step trust-region controller — 2026-08-22

The retained controller resolves the switched-response issue without another
gain formula. On the first accepted exact-exchange update only, it evaluates
the theoretically ideal unit log-time step and the established `0.7` damped
step, keeps the shorter successful response, and then resumes the existing
secant controller. The comparison costs at most one additional bounded
small-system solve and is selected from observed validated duration; it does
not branch on scenario identity.

Focused results preserved planning at `7.876384463025 s`, slalom at
`13.200853135545 s`, and opposing U at `28.575922287848 s`. U selected the
unit response and improved from `23.801121658982` to
`23.746859860594 s`, 4.07 percent above frozen main and below the five-percent
gate. Its fresh-process no-plot wall was `5.9768360 s`, including
`5.2146382 s` reported planner time. The 12-wall hairpin remained successful,
independently valid, and corridor-certified at `164.828287993153 s` and
`0.02 deg` clearance; candidate time was `8.7038268 s` under the current
machine load.

All 18 maintained examples ran in fresh processes with plots disabled. The
17 successes passed independent example validation, the expected no-path
case returned `noValidatedSeed` and passed its failure requirement, and every
row recorded zero HS3/NLP execution. Durations were unchanged from the prior
matrix except the U improvement.

| Example | Success/valid | Duration (s) | Wall (s) |
| --- | :---: | ---: | ---: |
| `exampleAlternatingSlalom` | 1/1 | 13.2008531355 | 5.5760827 |
| `exampleAzElPlanning` | 1/1 | 7.87638446302 | 3.9926940 |
| `exampleDenseConcaveAzElMotion` | 1/1 | 12.1408011078 | 3.7691847 |
| `exampleFortyMovingCircleGrid` | 1/1 | 105.698249574 | 8.7373155 |
| `exampleFourAcceleratingCircles` | 1/1 | 22 | 6.9427357 |
| `exampleInterceptMovingTargetAtSetTime` | 1/1 | 12 | 1.9055792 |
| `exampleInterceptMovingTargetEarliest` | 1/1 | 6.92326129150 | 2.5512170 |
| `exampleMovingBarrierWait` | 1/1 | 10.9963769559 | 4.1809937 |
| `exampleMovingCircleNoAzimuthWrap` | 1/1 | 13.6192201977 | 5.0294509 |
| `exampleMovingDeformingUSOutlineVisibility` | 1/1 | 21.1566336854 | 37.5709360 |
| `exampleNoPathAzElMotion` | 0/1 | `NaN` | 1.4948528 |
| `exampleObstacleFreeAzElMotion` | 1/1 | 4.53112887415 | 2.0797329 |
| `exampleOpeningUShapedAzElTimeSpace` | 1/1 | 12.1926012246 | 5.0517531 |
| `exampleStraightTargetAlternatingOcclusion` | 1/1 | 20.8695652174 | 7.4046856 |
| `exampleTargetExitsObstacle` | 1/1 | 24 | 6.8877105 |
| `exampleTwoOpposingUVisibilityGraph` | 1/1 | 28.5759222878 | 8.6294735 |
| `exampleUShapedAzElTimeSpace` | 1/1 | 23.7468598606 | 5.9768360 |
| `exampleUSOutlineExtremeVisibility` | 1/1 | 14.9628027709 | 35.8570521 |

The no-plot wall sum was `153.6382859 s`; fresh-process timing differs from
the prior matrix, so no uniform incremental speed claim is made. The earlier
active-set change still provides the material aggregate wall reduction.
Automated tests passed 54/54 in `24.9829 s`, Code Analyzer reported zero
messages across 65 maintained MATLAB files, and `git diff --check` found no
whitespace errors beyond line-ending notices. Current core is 7,200 lines
excluding 565 plotting lines; maintained MATLAB excluding examples/scratch is
9,901 lines. Core remains 200 lines above target, and the all-example frozen
main duration goal remains open.

## Minimum-jerk dynamic safe-side controller — 2026-08-22

The path-fixed timing controller first recovered the moving/deforming short
visibility seed at `17.8505455488212 s`. A feasibility-only safe-side exchange
improved it to `16.9391537555 s` but stalled with both acceleration and jerk
saturated. Replacing the arbitrary zero LP objective with a convex sampled,
limit-normalized integrated-jerk quadratic produced monotone geometry updates.
Keeping the obstacle activation horizon fixed while backtracking the decision
trust radius corrected a barrier-row loss found during the bounded prototype.

The retained production controller is input-driven: it runs only for earliest-
arrival dynamic routes with at least 12 route vertices, uses a `0.5 deg`
maximum control step, three bounded backtracks, at most 24 accepted iterations,
and validates every proposed motion against the original protected dynamic
geometry. It does not use example names, reference trajectories, hidden
waypoints, or relaxed validation.

Fresh production evidence for `exampleMovingDeformingUSOutlineVisibility`:

- duration `12.873502939647 s` versus frozen reference
  `12.986386910606 s` (improvement `0.112883970959 s`);
- selected polyline `63.0848051465551 deg`, smoothed path
  `64.4440622821245 deg`;
- independent validation, continuous collision, velocity, acceleration, and
  jerk checks all passed; minimum protected clearance
  `0.00176885089852413 deg`;
- planner time `40.6991780 s` and no-plot wall `58.6827768 s` in the focused
  run; the independent matrix row measured `59.3688435 s`. This is slower
  than the prior `33.4187830 s` fixed-point run, so no runtime improvement is
  claimed.

A reference-only representation experiment was used for diagnosis, never in
production: a doubled-knot C3 quintic fit reproduced the removed reference to
`0.0003631 deg` at `12.9864056 s`, proving that geometry smoothness—not scalar
timing gain—was the limiting mechanism. The reference file and all prototype
artifacts were removed after recording aggregate evidence.

All 18 maintained noninteractive examples then ran in separate fresh MATLAB
processes with plots disabled. The 17 expected successes passed independent
validation; `exampleNoPathAzElMotion` returned the expected
`noValidatedSeed`. The full row data is appended to `benchmark.csv` under
`working-tree-minimum-jerk`. Automated tests passed 54/54 and Code Analyzer
reported zero messages. The current size is 7,475 core production lines
excluding 565 plotting lines and 10,167 maintained MATLAB lines excluding
examples/scratch; the 900-line per-file, conditional 7,500-line core, and
12,000-line maintained limits pass.

HS3 execution files were already deleted on this branch. The remaining HS3-
only seed-summary fields, elapsed-time field, live benchmark flag, and current
test assertions were removed. Historical benchmark rows remain as evidence.
Several frozen main-branch example durations still outperform this branch, so
the broader all-example meet-or-beat objective remains open.

## Compact C3 dynamic duration controller — 2026-08-22

Repeated scalar gain changes were stopped after diagnosis showed that every
fixed-duration geometry solve naturally approaches the velocity boundary, so
post-solve derivative utilization does not identify the next feasible duration.
The retained controller instead uses a hybrid bounded law: command 15% shorter
durations until feasibility switches, then recover inside the valid/invalid
bracket. Every duration candidate is a convex minimum-jerk QP over an
eight-span, doubled-knot C3 quintic with exact endpoint conditions, sampled
workspace/velocity/acceleration/jerk inequalities, and time-correct protected
obstacle half-planes. The maintained validator remains authoritative.

The bounded prototype gates were:

- forty moving circles: `62.4777398626363 s` versus frozen
  `64.5557799164289 s`, independent clearance `0.0211388632 deg` in the
  513-point prototype;
- moving circle: `9.00245747230184 s`, below its frozen reference and valid;
- moving/deforming U.S.: the fixed eight-span representation was infeasible at
  `12.873502939647 s`, so the proven variable-route minimum-jerk controller
  was preserved rather than replaced.

Production runs at most one compact candidate for earliest-arrival static or
dynamic inputs with a shortest eligible seed of three through ten vertices.
A zero-length seed is eligible only when its bounded hold-recovery result
passed. A 256-vertex maximum per obstacle slice bounds polygon projection
work. The work gate was added after a dense geographic-outline experiment
improved duration to `6.31442280144023 s` but regressed wall to
`71.7744356 s`; after rejection, the established outline result returned at
`9.08019276709151 s` and `38.5873657 s` wall. This is an input-resolution
rule, not a scenario or obstacle-name branch.

Halving the compact constraint grid from 513 to 257 points survived the strict
proof window:

- forty circles retained `62.4777398626363 s`, passed independent validation,
  had `0.0018431217789 deg` clearance, and final wall `16.3138462 s`;
- moving circle reached `8.75122873615098 s`, passed independent validation,
  had `0.00126607744059 deg` clearance, and wall `6.6309076 s`.

The same unchanged controller was then tested on static compact topology.
Dense concavity reached `8.70379660028619 s` versus frozen
`8.79764398400 s`, with `0.00383572502922 deg` clearance and production wall
`5.4895786 s`. Basic planning reached `7.64965634404255 s` versus frozen
`7.81626785688 s`, with `0.0154824394667 deg` clearance and `5.2756101 s`
wall. Expanding the seed cap to ten improved slalom to `10.8556642583563 s`.
A seventh duration trial improved U to `22.6408601069837 s`, beating main with
`8.1805762 s` wall and narrow positive `0.0000217071124385 deg` clearance.
A recovered direct-wait seed improved moving barrier to `10.3713875624741 s`.

For direct rest-to-rest fixed-arrival trials, the exact jerk-switching profile
is now uniformly stretched to the requested duration. Position is preserved;
velocity, acceleration, and jerk coefficients decrease by the first, second,
and third powers of the time scale. This moved earliest target intercept to
`6.11153430175781 s` versus frozen `6.275807672 s`. Specified intercept, four
accelerating circles, alternating occlusion, and target-exit fixed-time rows
all retained their required arrival and independent validation.

Rejected dense/opposing experiments remain visible: opposing U reached only
`26.6497104563525 s` with 12 spans, alternate homotopies were infeasible, and
a 6-degree offset region regressed it to `29.1180993363482 s`. Dense
nearest-vertex linearization first exposed two indexing failures; the
source-correct form remained at `9.08019276709151 s` and was removed because
it produced no verified improvement. A direct interpolated-edge projection
also accepted no shorter motion (`37.3217196 s` wall without occupancy sign,
`40.2701656 s` with sign) and was removed. Exact original geometry remains in
both generation and validation.

The retained fresh-process matrix is recorded in `benchmark.csv` under
`working-tree-compact-c3`:

| Example | Duration (s) | Wall (s) |
| --- | ---: | ---: |
| `exampleAlternatingSlalom` | 10.8556642584 | 9.4585237 |
| `exampleAzElPlanning` | 7.64965634404 | 5.2756101 |
| `exampleDenseConcaveAzElMotion` | 8.70379660029 | 5.4895786 |
| `exampleFortyMovingCircleGrid` | 62.4777398626 | 16.3138462 |
| `exampleFourAcceleratingCircles` | 22 | 7.0245682 |
| `exampleInterceptMovingTargetAtSetTime` | 12 | 1.7462867 |
| `exampleInterceptMovingTargetEarliest` | 6.11153430176 | 2.2133061 |
| `exampleMovingBarrierWait` | 10.3713875625 | 5.3361420 |
| `exampleMovingCircleNoAzimuthWrap` | 8.75122873615 | 6.6309076 |
| `exampleMovingDeformingUSOutlineVisibility` | 12.8735029396 | 63.6398616 |
| `exampleNoPathAzElMotion` | `NaN` | 1.4197695 |
| `exampleObstacleFreeAzElMotion` | 4.53112887415 | 1.7442480 |
| `exampleOpeningUShapedAzElTimeSpace` | 12.1926012246 | 5.3723642 |
| `exampleStraightTargetAlternatingOcclusion` | 20.8695652174 | 6.9819562 |
| `exampleTargetExitsObstacle` | 24 | 6.3354136 |
| `exampleTwoOpposingUVisibilityGraph` | 27.5872457615 | 12.6033494 |
| `exampleUShapedAzElTimeSpace` | 22.6408601070 | 8.1805762 |
| `exampleUSOutlineExtremeVisibility` | 9.08019276709 | 38.5873657 |

The final wall sum is `204.3536736 s` versus frozen `214.3340749 s`, a `4.66%`
reduction (`1.049x`). All 17 expected successes and the expected no-path row
passed. Fifteen of 17 success durations meet or beat the frozen main matrix.
Remaining gaps are extreme outline `+35.85%` and opposing U `+20.60%`.

The 12-wall hairpin remained independently valid at
`164.828287993152 s`, `0.0199999999878 deg` clearance, `9.4377117 s`
candidate time, and `10.8024597 s` total benchmark wall in the final run.
The prior faster timing was not substituted for this unfavorable evidence.

Final recovery verification produced 54/54 passing tests in `32.3868484 s`, zero Code
Analyzer messages across 76 maintained MATLAB files, and no `git diff --check`
errors beyond line-ending notices. Literal core production is 7,500 lines
excluding 565 plotting lines; maintained MATLAB excluding examples/scratch is
10,267 lines; the largest production file is 881 lines. All C3 prototype MAT
files and scratch harnesses were removed after integration.

## Batched stationary geometry and feasibility-switched C3 — 2026-08-22

The declared focused baseline used fresh MATLAB R2024b Update 4 processes,
`PlotOutputs=false`, deterministic example defaults, no parallel toolbox, and
unchanged protected geometry, margins, limits, arrival policy, and independent
validation. Before this change, opposing U was `27.5872457615295 s` path and
`13.290439 s` wall; extreme outline was `9.08019276709151 s` path and
`36.6313948 s` wall. The retained single-U reference was
`22.6408601069837 s` with `8.1805762 s` wall.

Profiling the headless extreme sequence measured `44.6659108 s` profiler wall.
`validateAzElTrajectory` accounted for `30.927561 s`, compact/corridor planning
for `29.613187 s`, `obstacleShapeAtTime` for `14.954427 s`, and
`pointPolygonClearance` for `14.373534 s`. The driving input was up to 5,352
boundary vertices. Identical static history slices were rebuilding the same
polyshape at every interior query. Reusing the prepared shape reduced an
otherwise unchanged extreme run from `36.6313948 s` to `26.7460725 s`; its
path and clearance were unchanged. Opposing U decreased from `13.290439 s` to
`12.2319648 s`, also with unchanged path and clearance.

The exact point-to-edge kernel was then extended from scalar to batched points.
Against the former scalar implementation, 490 trajectory/random dense queries
had maximum clearance error `6.8834e-15 deg`, zero nearest-point error, zero
edge-index mismatches, and `4.0734x` speedup. All three geographic regions
measured `4.04x` to `5.88x`; after matching scalar projection arithmetic at
vertex ties, both U obstacles had zero error and zero edge-index mismatches.
A maintained matrix/scalar equivalence test now covers signs, nearest points,
and deterministic edge order. Dynamic and topology-changing histories retain
the scalar time-local path; dense eligibility is opened only when preparation
proves stationary intervals.

Compact C3 previously spent its one solve on the shortest geometric seed even
when that seed had no valid motion. Requiring a validated eligible topology
selected the valid dense seed and produced `6.22216662414646 s` extreme motion
versus frozen main `6.684968340018 s`. The final focused run took
`29.0349372 s`, passed independent collision/kinematic validation, and had
`0.00709851977447 deg` clearance. This is a `39.44%` wall reduction relative
to the `47.9449594 s` frozen-main row and a `20.74%` reduction relative to the
fresh non-NLP baseline.

Opposing-U profiling showed a `24.8681467553 deg` smooth path versus main
`24.3709048951 deg`, but only `0.331` acceleration and `0.095` jerk utilization;
the long-duration minimum-jerk basin, not route length, caused the gap. A
probe at 10 percent from the physical jerk-limited lower bound toward the
validated upper bound now selects the controller mode. If feasible, the solver
bisects the lower failed/upper feasible bracket; if infeasible, it switches to
the established high-to-low continuation. Eight trials include the probe plus
the former seven-trial budget. This invariant is derived from feasibility and
contains no example, obstacle, route, or benchmark identity.

Focused fresh-process results after the switch were:

| Example | Frozen main duration (s) | Current duration (s) | Wall (s) | Clearance (deg) |
| --- | ---: | ---: | ---: | ---: |
| `exampleAlternatingSlalom` | 12.1809174022 | 10.8556642584 | 4.8657687 | 0.000502959179063 |
| `exampleUShapedAzElTimeSpace` | 22.8185487359 | 22.6408601070 | 6.1954661 | 0.0000217071124385 |
| `exampleTwoOpposingUVisibilityGraph` | 22.8751245760 | 22.1609457614 | 9.1406562 | 0.0143650783404 |
| `exampleUSOutlineExtremeVisibility` | 6.68496834002 | 6.22216662415 | 29.0349372 | 0.00709851977447 |

All four passed the maintained independent validator with continuous collision
and exact kinematic limits; the three compact rows selected compact C3 and no
production HS3 implementation exists. The single U remains below ten seconds
wall and beats main. Opposing U now beats main path by `3.12%` and reduces wall
`58.01%` versus the `21.7702201 s` frozen reference. Slalom is unchanged in
path and improves in measured wall.

Focused tests pass 6/6: four dynamic-timing tests plus stationary-shape reuse
and batched/scalar projection equivalence. Code Analyzer reports zero messages
for the four affected production files. Core production is 7,498 literal lines
excluding 565 plotting lines; maintained MATLAB excluding examples/scratch is
10,296 lines; maximum production file size is 881 lines. The temporary oracle
harness and task-specific temporary directory were removed.

The required current full 18-example matrix, full suite, and hairpin rerun are
not yet claimed. After the focused runs, new MATLAB processes began failing
before initialization with `Fatal Startup Error: System Error: File system
inconsistency`, including a zero-work batch and a separate non-batch startup.
One delayed `disp('STARTUP_OK')` launch later succeeded, but every subsequent
fresh benchmark launch failed again despite 6/30-second cooldowns and verified
service-host restarts. No maintained example ran in those failed matrix
attempts. The last complete 18-row matrix above remains historical evidence
only until fresh-process MATLAB execution recovers reliably.

## Final controlled fresh-process proof — 2026-08-22

The user authorized a reset of the MathWorks coordination state. With MATLAB
stopped, the exact `ServiceHost` and `MATLABConnector` directories were moved
to timestamped sibling backups rather than deleted. MATLAB regenerated them
and passed two zero-work startup checks. Rapid consecutive launches still
reproduced the pre-initialization file-system error, so those six failed
launches executed no example code and are not benchmark rows. Stopping only
the regenerated MathWorks Service Host and allowing a ten-second settle before
each launch then produced 18 consecutive fresh, serial MATLAB processes.

All 17 expected successes passed the examples' independent validation,
continuous collision checks, and kinematic certificates. Every success
selected `corridorQuintic`; no production HS3 implementation exists. The
expected no-path case returned `noValidatedSeed` and passed its stable failure
requirement. Exact CSV rows are under source
`working-tree-batched-c3-final`.

| Example | Frozen main duration (s) | Current duration (s) | Wall (s) |
| --- | ---: | ---: | ---: |
| `exampleAlternatingSlalom` | 12.180917402175 | 10.855664258356 | 4.4654387 |
| `exampleAzElPlanning` | 7.816267856881 | 7.649656344043 | 4.0561583 |
| `exampleDenseConcaveAzElMotion` | 8.797638855700 | 8.690573182986 | 3.9323402 |
| `exampleFortyMovingCircleGrid` | 64.555779916429 | 62.477739862636 | 15.3927512 |
| `exampleFourAcceleratingCircles` | 22 | 22 | 7.1808018 |
| `exampleInterceptMovingTargetAtSetTime` | 12 | 12 | 1.6243784 |
| `exampleInterceptMovingTargetEarliest` | 6.274806792200 | 6.111534301758 | 2.0817962 |
| `exampleMovingBarrierWait` | 10.544227895142 | 10.371387562474 | 5.0897298 |
| `exampleMovingCircleNoAzimuthWrap` | 12.293137410146 | 8.751228736151 | 6.9248772 |
| `exampleMovingDeformingUSOutlineVisibility` | 12.986386910606 | 12.873502939647 | 53.0425551 |
| `exampleNoPathAzElMotion` | `NaN` | `NaN` | 1.1949310 |
| `exampleObstacleFreeAzElMotion` | 4.612405963436 | 4.531128874149 | 1.5160743 |
| `exampleOpeningUShapedAzElTimeSpace` | 15 | 11.735378678642 | 5.5235209 |
| `exampleStraightTargetAlternatingOcclusion` | 20.869565217391 | 20.869565217391 | 6.7002168 |
| `exampleTargetExitsObstacle` | 24 | 24 | 6.2389747 |
| `exampleTwoOpposingUVisibilityGraph` | 22.875124576026 | 22.160945761398 | 8.5208632 |
| `exampleUShapedAzElTimeSpace` | 22.818548735851 | 22.640860106984 | 5.5963246 |
| `exampleUSOutlineExtremeVisibility` | 6.683971648809 | 6.222166624146 | 27.9310043 |

Thus all 17 success durations meet or beat the frozen optimized main rows;
the only positive difference is `3.0e-13 s` fixed-arrival roundoff. The
current example-wall sum is `167.0127367 s`, a `33.74%` reduction from the
`252.0683835 s` optimized-main matrix and an `18.27%` reduction from the prior
`204.3536736 s` complete corridor-only matrix. This is an aggregate result,
not a uniform per-example speedup: forty moving circles, moving/deforming U.S.,
and target-exits-obstacle are slower in wall time than optimized main. The
moving/deforming row is the material outlier at `53.0425551 s` versus
`26.8349249 s`; its path nevertheless improves by `0.112883970959 s` and
remains independently valid.

The acceptance cases pass directly. Single U is `22.640860106984 s` with
`5.5963246 s` wall and `2.17071124385404e-05 deg` minimum clearance. Opposing
U is `22.160945761398 s` with `8.5208632 s` wall. Extreme outline is
`6.222166624146 s` with `27.9310043 s` wall. A fresh 12-wall hairpin run
returned the exact `164.828287993152 s` motion, `0.0199999999878 deg`
clearance, corridor certificate, and independent validation in
`8.0691334 s` candidate / `9.3474237 s` total wall.

The complete automated suite passed `56/56` in `28.7732939 s`. The combined
test/analyzer reporting command then used an invalid character/string path
expression after the tests had finished; a corrected analyzer-only rerun
checked 66 nonscratch MATLAB files with zero messages. A visible successful
planning run passed and created three figures with 529 graphics objects. The
expected failure passed and created two diagnostic figures with 343 objects,
one expanded state, and two rejected transitions. No graphics check reran the
planner to fabricate diagnostics.

Final repository audit found 7,498 core production lines excluding the
565-line plotting module, 10,296 maintained nonscratch/nonexample MATLAB lines,
and an 881-line largest production file. The final CSV source contains exactly
18 rows, 17 successes, 18 validation passes, and the recorded
`167.0127367 s` wall sum. Production search found no live `solveAzElHs3` or
`fmincon` call. `git diff --check` found no whitespace errors; its output was
limited to existing LF-to-CRLF notices. The temporary matrix wrapper was
removed. No commit or push was performed.

## Dynamic seed-slot coverage experiment — 2026-08-22

The bounded baseline used eight deterministic moving-circle fields with master
seed `3252026`, three allowed seeds, unchanged protected geometry and limits,
and public independent validation. The command was
`probeDynamicTopologyCoverage()`. Baseline planner success and independent
validation were `2/8`; total measured wall was `51.710458 s`. In two failed
cases the exact timed search produced a direct wait, the extended timed search
produced no unique additional seed, and the reserved third slot remained
unused even though a spatial visibility route existed.

The retained change computes the extended timed proposal before allocating
spatial search capacity. A slot is reserved only when that proposal is
nonempty and distinct from the already generated exact-time seeds; spatial
seeds are still appended before the timed proposal, preserving established
seed precedence. No public option, work cap, obstacle tolerance, or validation
rule changed. The final identical probe passed `3/8` in `60.5390732 s`. The new
six-circle success is independently valid at `21.3080494189 s` duration and
`0.00264401481202 deg` minimum clearance. The other five no-path results remain
visible; this is a focused coverage gain, not a completeness claim.

The complete 18-example headless gate passed: 17 independently validated,
collision-free and kinematically certified successes plus the expected
`noValidatedSeed` requirement. All 17 motion durations exactly match the frozen
`working-tree-batched-c3-final` rows within `1e-6 s`; the measured wall sum was
`124.039315 s` in one warmed MATLAB process and is not compared with the prior
fresh-process wall sum. Exact rows are recorded in `benchmark.csv` under
`working-tree-topology-slot-proof`. A visible success created three figures
with 487 graphics objects; the expected failure created two diagnostic figures
with 341 objects without rerunning search. The final automated suite passes
`57/57`, including the new deterministic seed-slot regression test, and Code
Analyzer reports zero messages for the changed MATLAB files.

Route-size experiments were unfavorable and removed. On the same moving
10-wall request, the original 42-vertex dynamic fallback returned an
independently valid solution in `15.5401058 s`. Skipping expansion finished in
`5.9110476 s` but lost the solution. Densifying a sampled-clear 22-vertex
subsequence finished in `8.5416433 s`, and preserving its original temporal
coordinates finished in `9.1188355 s`; both returned `noValidatedSeed`.
Therefore the known route-dimensional runtime cliff remains unresolved rather
than trading correctness for speed.

Final size is unchanged at 7,498 core production lines excluding the 565-line
plotter. The maintained nonscratch/nonexample tree is 10,348 lines, and the
largest production file is 887 lines. The production diff is net zero lines;
the dynamic-timing test file adds 52 lines for one deterministic moving-circle
fixture and its focused assertion. Temporary stress and benchmark harnesses
were removed. No commit or push was performed.

## Shallow collision-residual feedback experiment — 2026-08-22

The collision-stage baseline was the pushed `034a6a4` topology-slot commit.
Fixed moving-circle case 1 constructed kinematically valid motions but all
three seeds failed authoritative collision validation; the visibility seed's
reported clearance was `-0.0790211794215 deg`. Reconstructing its 13-vertex
retimed spline localized the actual feedback starting residual to
`-0.00108381917352 deg` at `28.4660205183 s` duration.

The retained controller linearizes signed protected-obstacle clearance against
interior spline controls. For a violated row, the single-row minimum required
decision norm is `-bound / norm(row)`. The applied feedback gain is the lesser
of one and the trust radius divided by the largest required norm. A bounded
minimum-norm QP applies that partial residual correction. For an interior
point, the closest-boundary vector is reversed so the gradient points toward
increasing signed clearance. Intermediate steps must remain kinematically
valid and strictly improve independently measured minimum clearance; only a
fully validated trajectory can become a planner success.

The first prototype took one accepted step and reached
`0.00612677738555 deg` clearance with unchanged duration. Integrated production
case 1 passes at `0.00570897255047 deg` selected clearance. The final identical
eight-case moving-circle sweep passes and independently validates `4/8`, versus
the pushed `3/8` baseline. Cases 3, 4, 6, and 8 remain reproducible
`noValidatedSeed` outcomes; no redraw, tolerance change, or topology special
case was used.

An initial all-residual implementation failed the benchmark gate. It recovered
three extreme-outline candidates starting at `-0.00662790746078`,
`-0.0112188785752`, and `-0.0102058874618 deg`, changing the final-region path
from `6.22216662414646 s` to `8.39529809634767 s`. Disabling recovery restored
the frozen path exactly. The retained input-driven local-linearization rule
therefore limits recovery to penetrations no deeper than `0.005 deg`, one tenth
of the controller's `0.05 deg` clearance target. Case 1 still passes, while a
fresh extreme-outline rerun restores the exact baseline metrics.

The final 18-example gate used one fresh serial MATLAB process per example.
All 17 expected successes passed independent validation, collision checks, and
kinematic certificates; the expected no-path result passed its stable failure
requirement. Every successful polyline length, smoothed length, and duration is
exact to the pushed topology-slot evidence. The fresh-process wall sum is
`172.6919951 s`; the moving/deforming outline remains the unfavorable wall
outlier at `53.2656011 s`. Exact rows are in `benchmark.csv` under
`working-tree-residual-feedback-final`.

The focused controller regression passes in `13.3148997 s`. The final complete
suite passes `58/58` in `48.4440443 s`, and Code Analyzer reports zero messages
across all 66 nonscratch MATLAB files. A visible success produced three figures and
487 graphics objects. The expected failure produced two diagnostic figures and
341 objects from its original search record, including one expanded state and
two rejected transitions. Final size is 7,500 core production lines excluding
the 565-line plotter, 10,367 maintained nonscratch/nonexample MATLAB lines, and
an 887-line largest production file. The two temporary proof harnesses were
removed; the pre-existing untracked benchmark artifacts were left untouched.

## Disconnected visibility boundary support — 2026-08-22

The baseline was pushed commit `0c8bf66`. On the fixed moving-circle family
with master seed `3252026`, three allowed seeds, unchanged geometry, and public
independent validation, cases 3, 4, 6, and 8 returned `noValidatedSeed`; the
baseline therefore passed 4/8. Stage diagnosis classified cases 3, 4, and 8 as
bounded-topology failures. Case 6 had a sampled-clear timed polyline with
`0.199951692541 deg` clearance, but its smoothed motion collided at
`-0.522166959136 deg` and no spatial fallback was available.

The retained invariant is limited to a disconnected bounded visibility graph.
After the existing offset/exhaustive retry ladder reaches retry three, the four
input workspace corners become ordinary visibility-tested support nodes and a
three-seed portfolio preserves one spatial-diversity opportunity. Connected
retry-zero graphs retain their established temporal seed portfolio. The
corners are not hidden waypoints: occupied or blocked corners receive no usable
edge, and the unchanged workspace-spanning wall remains `noValidatedSeed`.

Two broader prototypes were rejected. Adding corners to every graph changed
the shallow collision-controller densification factor and failed its focused
regression. Replacing the third temporal seed on every three-seed request
changed `exampleMovingCircleNoAzimuthWrap` from the frozen
`8.75122873615098 s` motion to `8.77956166926098 s`. Restricting both behaviors
to graphs that exhausted retry three restored the exact frozen moving-circle
polyline, smoothed length, and duration.

The final identical eight-case sweep passed and independently validated 8/8 in
`232.089424 s`:

| Case | Duration (s) | Minimum clearance (deg) |
| --- | ---: | ---: |
| 1 | 20.9069577259 | 0.00570897255047 |
| 2 | 21.1343850031 | 0.00537827059448 |
| 3 | 21.1728041688 | 0.00268927771046 |
| 4 | 21.2089164576 | 0.0104420223329 |
| 5 | 20.5473740302 | 0.000355731152304 |
| 6 | 21.1748286081 | 0.00165612365924 |
| 7 | 20.5780642844 | 0.0012584101104 |
| 8 | 20.9089388561 | 0.000739339048806 |

The focused dynamic suite, including the four-case replay and static wall,
passed 7/7 in `95.3418714 s`. The final complete suite passed 59/59 in
`134.081193 s`; Code Analyzer reported zero messages across 66 nonscratch
MATLAB files. The new deterministic regression requires retry-three topology,
a selected `visibilityGraph` seed, positive homology coverage, and positive
independent continuous clearance for all four formerly failing cases.

The definitive 18-example gate used one fresh serial MATLAB process per
example. All 17 expected successes passed independent collision and kinematic
validation, and the expected no-path result passed its stable failure requirement.
Every successful polyline length, smoothed length, and duration is exact to the
pushed `working-tree-residual-feedback-final` rows within `1e-6`; exact new
rows are appended to `benchmark.csv` under
`working-tree-boundary-support-final`. Measured wall sum was an unfavorable
`205.6452420 s` versus `172.6919951 s` for the pushed evidence, so no runtime
improvement is claimed. The moving/deforming outline and extreme outline were
the largest walls at `61.1979723 s` and `39.7610225 s`.

A visible U-shaped success passed and created three figures with 522 graphics
objects. The visible expected failure passed its example requirement and created
two returned-diagnostic figures with 342 objects, three expanded states, and
15 rejected transitions. MATLAB then returned Windows graphics teardown code
`1073807364` after printing those results; a hidden retry failed before startup
with `-1073741205`. This GUI-host fault is retained as unfavorable environment
evidence rather than reported as a planner or figure-generation pass with a
clean process exit.

Production remains exactly 7,500 core physical lines excluding the 565-line
plotter. The maintained nonscratch/nonexample MATLAB tree is 10,392 lines, and
the largest production file remains `generateAzElTopologySeeds.m` at 887
lines. The production diff is net zero lines through local compaction; the
dynamic-timing test adds 25 lines. Temporary diagnostic and example-gate
harnesses were removed, and the twelve pre-existing untracked benchmark
artifacts were left untouched. No commit or push was performed.

## Repository cleanup and module refactor — 2026-08-22

The repository cleanup retained every production algorithm and removed only
resolved generated outputs. Twenty-three internal implementation files moved
from one flat package into `geometry`, `obstacles`, `search`, `motion`, and
`validation` subpackages. Internal names now rely on package context, for
example `azElInternal.search.generateTopologySeeds` and
`azElInternal.motion.solveCorridorQuintic`. The public planner, obstacle,
validation, plotting, and example entry points did not change.

Source cleanup consolidated protected/original boundary-history
normalization, polyshape construction, and boundary-edge extraction. Dense
history vertices are collected through preallocated cells. Corridor solves
reuse immutable prepared obstacle caches passed by the planner; raw benchmark
calls still normalize and prepare once. No planner tolerance, work limit,
seed order, geometry, validation rule, or result field changed.

Seven tracked `scratch/` MATLAB/CSV outputs were removed and remain
recoverable from Git. Ignore rules now cover reproducible scratch, MAT, PNG,
run-CSV, and summary-CSV outputs. The twelve pre-existing untracked benchmark
artifacts remain on disk and are now ignored.

Verification evidence:

- Pre-change baseline: Code Analyzer `0`; tests `59/59` in `139.173458 s`.
- Focused normalization/provenance checks: `5/5` in `2.306672 s`.
- Post-refactor Code Analyzer: `0` across all nonscratch MATLAB files.
- Post-refactor tests: `59/59` in `150.384070 s`.
- Protected and original boundary-size errors retained their exact identifiers.
- Fresh-process examples: 17 independently validated successes and one
  validated expected `noValidatedSeed` failure. Every successful polyline,
  smoothed length, and duration exactly matches the
  `working-tree-boundary-support-final` evidence.
- Fresh-process example wall sum: an unfavorable `222.7331866 s` versus the
  prior `205.6452420 s`; no runtime improvement is claimed. The largest walls
  remain moving/deforming outline `60.1933815 s` and extreme outline
  `41.5000613 s`.
- Four-wall hairpin raw-solver benchmark smoke: planner/validation/corridor
  certificate `1/1/1`, duration `54.6635704076809 s`, candidate solve
  `2.6237182 s`, and `corridorPrototypeValidated` termination.
- Exact example rows are appended to `benchmark.csv` under
  `working-tree-repository-cleanup`.

Core MATLAB changes from 35 files / 7,500 physical lines / 6,468 executable
lines / 898 comment lines to 37 files / 7,773 physical lines / 6,450
executable lines / 1,200 comment lines. Executable code decreased by 18 lines.
The 273-line physical overage is documentation/comment growth explicitly
authorized by the user after regression. The largest production file is
`search/generateTopologySeeds.m` at 886 lines; all production files remain
below 900 lines.

The Windows MATLAB launcher returned a pre-startup `File system
inconsistency` error for long inline commands and one nested shell loop.
Short harness-based fresh processes completed all reported checks. The
temporary harness is removed before handoff. No commit or push was requested.

## Readability and short-file audit — 2026-08-23

The follow-up readability pass is comments and formatting only. Section 0 is
now reserved for each file's primary function: all 86 local Section 0 headers
were removed, all 229 local functions begin with a direct purpose sentence,
and duplicate local `PURPOSE` blocks were removed. The 131 internal loops retain
immediately preceding explanations, and additional decision comments describe
fallbacks, work limits, candidate acceptance, retiming, and early exits in the
four largest motion/search files.

All 22 production MATLAB files below 100 nonblank, noncomment code lines were
audited for textual callers. None is uncalled. Eighteen have multiple callers;
the four single-caller files own a stable result format or a distinct algorithm
extracted from an already-large orchestrator. The complete rationale is in
`short_file_rationale.md`.

Text-only checks found zero local Section 0 headers, missing local-function
purpose comments, unexplained internal loops, bare assignment continuations,
code continuations at or below 120 characters, trailing whitespace, or
physical-requirement hash mismatches. `git diff --check` passed. Per the user's
explicit instruction for this session, MATLAB tests, Code Analyzer, and
examples were not rerun after this comments-only pass. The earlier 59/59 and
18-example results above predate the latest comment changes and are not claimed
as post-pass execution evidence.

## Persistent interactive sandbox — 2026-08-23

The supplied two-tab UI is now `sandbox/azElInteractiveSandbox.m`. Goal Mode
automatically requests the start, then the goal, then obstacle strokes. Free
Mode requests the start, then its first endpoint, then obstacle strokes;
`Add Segment` remains for optional later endpoints. Separate start and goal
buttons are not present.

Both tabs now expose **Add Obstacle** for additional strokes. The initial
start-to-goal sequence still enters the first obstacle automatically, then
returns to idle after retaining it. The axes use an outer-position constraint
so the azimuth ticks and label remain inside their reserved area instead of
overlapping the action-button row.

The planning controls were subsequently condensed into three labeled groups.
Workspace and kinematic values share headings and use one row per setting;
timing and obstacle values also use one row each. The verbose checkbox moved
to the unused strip beneath the panel, removing the final-row overlap visible
under Windows display scaling.

The planning-control rows were moved below the panel title so the workspace
azimuth label is visible under common Windows display scaling. Canvas redraw
deletes all axes children, including graphics with hidden handles, before
resetting and reconstructing the axes; this removes the stale obstacle outline
that previously survived Reset.

Text-only checks found one primary Section 0 header, direct opening comments on
all 61 local functions, explanations immediately before all 20 loops, no bare
assignment continuations, no continuation blocks at or below 120 characters,
and no trailing whitespace. `git diff --check` passed. MATLAB, Code Analyzer,
and regression tests were not run because the user explicitly prohibited test
execution in this session.

Obstacle layers now use hidden legend handles for raw traces, simplified line
centerlines, polygon boundaries, original obstacle fills, and protected safety
outlines. Start, goal, requested routes, solved motion, and failure-route
entries retain their legend labels. Obstacle rendering and planner data are
unchanged.

## Combined corridor/HS3 method suite — 2026-08-23

The `325-full-suite` worktree combines two complete, physically isolated
planner snapshots behind `PlannerMethod`:

- corridor source: `325-less-nlp` at `2852663`;
- HS3 source: `plan-325` at `5a06711`.

Each branch's moving-target adapter is also isolated because their earliest
intercept policies differ. Corridor retains its chronological fixed-arrival
search and bracket refinement. HS3 retains its one-call moving-goal
earliest-arrival solve. Neither public dispatcher falls back to the other
method.

### Fresh-process maintained-example comparison

All 18 noninteractive maintained examples were run in a fresh MATLAB process
for each method. Each call disabled plots, animation, kinematics, search edges,
visibility graphs, swept surfaces, and verbose output. HS3 runs reapplied only
the collocation and improvement-time settings recorded by the Plan-325 source
examples.

The comparison gate required exact matches for goal-time policy, jerk policy,
planner success, independent example validation, collision/certificate fields,
and termination reason. Selected polyline length, returned-motion length, and
motion duration had to match within `1e-6`. Wall time was recorded but was not
an equality gate.

Results:

| Method | Baseline source | Cases | Validated successes | Validated expected failures | Gated differences | Wall sum (s) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `corridorQuintic` | `working-tree-repository-cleanup` | 18 | 17 | 1 | 0 | 163.9501049 |
| `hs3` | `2074c14+batched-envelope-history-worktree` | 18 | 17 | 1 | 0 | 238.5162172 |

Both expected failures were `exampleNoPathAzElMotion` with planner
`Success=false`, example validation passed, and termination reason
`noValidatedSeed`. Exact fresh rows are appended to `benchmark.csv` under
`2852663+325-full-suite-corridor-worktree` and
`5a06711+325-full-suite-hs3-worktree`.

An initial PowerShell-variable loop triggered MATLAB's pre-startup Windows
`File system inconsistency` launcher error and executed no cases. The final
literal-command fresh-process runs completed every recorded case on its first
attempt. This launcher issue is not hidden or counted as planner evidence.

### Physical unplugging proof

Two task-owned temporary copies were created. The HS3 folder was deleted from
one and the corridor folder from the other. In the corridor-only copy,
`exampleObstacleFreeAzElMotion` returned success, passed independent example
validation, and echoed `corridorQuintic`. In the HS3-only copy, the same checks
passed and the result echoed `hs3`. The temporary copies were deleted after
the proof.

### Current verification boundary

MATLAB Code Analyzer checked all 109 intended MATLAB files and returned zero
messages. The scan excluded only the unrelated untracked
`report_evidence_tmp.m` and `single_u_report_tmp.m` scripts. Text audits also
found explanations immediately before all 368 loops, exactly one primary
Section 0 in every function file, no local-function Section/PURPOSE boilerplate,
and no production MATLAB file without an executable caller. Of the 49
production files below 100 code lines, nine have one caller and are justified
individually in `short_file_rationale.md`.

Canonical source comparisons found no executable divergence in the 34-file
corridor package or the 26-file HS3 package after accounting for the approved
namespace and public-entry renames. Backend dependency scans found no call to
the root planner or shared root internals and no reference to the sibling
method. Only the four public dispatchers reference both method packages.

The repository regression test suite was not run because the user explicitly
prohibited tests unless requested in this session. The example matrix above is
fresh execution evidence, not a substitute claim for unrun automated tests.
Visible interactive sandbox verification also remains unrun; its planner
selector received static and Code Analyzer checks only.

## Compact stage-timing checkpoint — 2026-08-23

This checkpoint compares branch `325-full-suite` with the frozen
`27070ac5fac6f90624731a753d4b029e7ecea8e5` (`27070ac`) baseline. The public
`SearchDiagnostics.StageTiming` requirement contains exactly these seven fields:

1. `TopologyElapsedTime_s`
2. `CorridorConstructionElapsedTime_s`
3. `MotionSolvingElapsedTime_s`
4. `CollisionCheckingElapsedTime_s`
5. `FinalValidationElapsedTime_s`
6. `UnattributedElapsedTime_s`
7. `TotalElapsedTime_s`

The five named stages are exclusive: nested work is charged once, to its
owning stage. Repeated candidates and attempts are additive, including work
later discarded. `TotalElapsedTime_s` is measured independently and
`UnattributedElapsedTime_s` reconciles it against the five named stages;
over-attribution is rejected instead of being hidden.

### Maintained-example and focused verification

Each maintained example gate used fresh serial MATLAB processes.

| Method | Cases | Validated outcome | Direct process wall sum (s) | Harness example wall sum (s) | Planner sum (s) |
| --- | ---: | --- | ---: | ---: | ---: |
| `hs3` | 18/18 | 17 successes + expected validated `noPath` | 560.7800023 | 336.8454606 | 260.8179100 |
| `corridorQuintic` | 18/18 | 17 successes + expected validated `noPath` | 413.7346972 | 220.3806093 | 161.7237387 |

All 36 returned `StageTiming` records reconciled. A visible corridor smoke
run returned planner success `1`, independent validation `1`, and created two
figures. The focused final-source tests passed 23/23.

The earlier full-suite run passed 125/125 tests in 210.6225 seconds. That run
predates the final focused test additions and is recorded only as earlier
evidence. On the final source, the complete suite passed 127/127 tests with
zero failures or incomplete tests in 174.7697065 seconds. MATLAB Code Analyzer
then checked all 93 maintained production and test files and reported zero
findings.

### Frozen-baseline A/B timing

The A/B run comprised 48 successful invocations: two methods, four
representative examples, three repetitions, and two source versions. Every
observation used a fresh headless serial MATLAB process. Pair order was
balanced as baseline-candidate, candidate-baseline, baseline-candidate. The
table reports the exact median percentage changes from the recorded summary;
negative values favor the candidate.

| Method | Example | Direct-process wall | Harness wall | Planner time |
| --- | --- | ---: | ---: | ---: |
| `hs3` | `exampleObstacleFreeAzElMotion` | +1.308% | -9.959% | -4.938% |
| `hs3` | `exampleUShapedAzElTimeSpace` | +2.650% | +0.111% | +3.327% |
| `hs3` | `exampleFourAcceleratingCircles` | -0.198% | -3.193% | -3.377% |
| `hs3` | `exampleNoPathAzElMotion` | -4.035% | -4.995% | -4.884% |
| `corridorQuintic` | `exampleObstacleFreeAzElMotion` | -5.652% | -0.090% | +6.899% |
| `corridorQuintic` | `exampleUShapedAzElTimeSpace` | +0.848% | -1.667% | -0.160% |
| `corridorQuintic` | `exampleFourAcceleratingCircles` | -3.841% | -6.203% | -5.011% |
| `corridorQuintic` | `exampleNoPathAzElMotion` | +2.899% | +5.907% | +14.443% |

All 48 runs preserved status, validation, termination, policy, certificate,
and selected-seed outputs. The largest baseline/candidate physical-value
difference was `1.3056e-13`, below the `1e-6` audit threshold. The unfavorable
`corridorQuintic` no-path planner median increased by 14.443%; this is retained
as a regression signal, not averaged away or presented as a speedup.

### Exact size and current limits

Physical-line counts were recomputed from the frozen baseline after the timing
cleanup and canonical obstacle-infrastructure consolidation.

| Scope | Baseline | Current | Delta |
| --- | ---: | ---: | ---: |
| Production MATLAB | 76 files / 15,634 lines | 76 files / 15,140 lines | 0 files / -494 lines |
| Maintained MATLAB (production and tests) | 84 files / 19,057 lines | 84 files / 18,609 lines | 0 files / -448 lines |

The maintained tree still exceeds the former 12,000-line hard cap. The A/B
sample has only three observations per cell and includes fresh-process launch
noise, so it supports behavioral equivalence and identifies possible runtime
regressions; it does not establish a general speedup.

`+azElInternal` is now the neutral shared obstacle layer. The two method-local
combine, normalize, query, boundary-traversal, and signed-clearance copies were
removed. A pre-deletion equivalence gate passed 5/5 across all three copies and
the new shared query. After deletion, the permanent infrastructure tests passed
5/5 and both planner unit files passed 92/92. The broader suite was started but
stopped when the user requested no additional tests before pushing, so no
post-consolidation full-suite result is claimed.

## Corridor helper consolidation — 2026-08-23

This behavior-preserving cleanup was measured against task baseline `3280bb0`.

| Scope | Baseline | Current | Delta |
| --- | ---: | ---: | ---: |
| Fixed-goal corridor runtime closure | 42 files / 7,713 lines | 36 files / 7,366 lines | -6 files / -347 lines |
| Production MATLAB | 76 files / 15,133 lines | 70 files / 14,787 lines | -6 files / -346 lines |
| Production and tests | 84 files / 18,602 lines | 78 files / 18,256 lines | -6 files / -346 lines |

Five method-local copies were replaced with their executable-equivalent
`azElInternal` implementations. `buildStraightJerkProfile` became a local
function of its only caller, `buildQuinticSpline`. Search, candidate selection,
result construction, and independent final validation remain separate.

Verification produced:

- pre-change focused tests: 52/52 passed;
- post-change focused tests: 52/52 passed;
- MATLAB Code Analyzer: 0 messages across 14 changed MATLAB files;
- dependency audit: 36/36 runtime files reachable, 0 orphaned;
- stale deleted-helper references: 0;
- `git diff --check`: passed.

The complete maintained-example, visible-graphics, and full regression matrices
were not rerun. No `benchmark.csv` row was added because this checkpoint
executed tests and static audits, not a maintained-example benchmark.

## Input-history collision broad phase and ungrouped feasibility — 2026-08-23

This checkpoint modifies only the corridor validator in production. The
representative case is `exampleFortyMovingCircleGrid`, headless and serial in
fresh MATLAB R2024b Update 4 processes, seed 325, with plots and animation
disabled. Obstacle histories, limits, safety margin, validation, and route
quality were identical between baseline and candidate.

This is the only maintained example setting `SeedClusterDistance_deg` above
zero. Its swept input has `SourceRegionCount=1`, so the requested 2-degree
clustering creates `ClusterGroupCount=0`. A zero-distance run generated the
same 18 nodes, 62 candidate pairs, 28 visibility edges, 80 rejected
transitions, 21 expanded states, two seeds, 110.807929685255-degree selected
polyline, and 62.4777398626363-second validated motion. Ungrouped operation
was already feasible; clustering was not providing the runtime bound.

Profiling recorded 47,793 calls each to `shapeAtTime` and
`pointPolygonClearance`, with collision checking dominant. The retained broad
phase derives a lower bound from every incoming obstacle's supplied
`InternalPreparation.HistoryBounds_deg` and the caller's velocity limits. It
assumes no constant speed, rigid shape, fixed size, or named scenario. Only an
input-derived proof skips exact geometry; near obstacles keep the existing
shape-at-time query and adaptive certificate.

Three alternating fresh-process pairs produced:

| Metric | Baseline runs | Candidate runs | Median change |
| --- | --- | --- | ---: |
| Planner time (s) | 14.3362922 / 14.5753369 / 14.5689666 | 7.1588191 / 7.0040011 / 6.9326244 | -51.925% |
| Collision stage (s) | 8.2384 / 8.2169 / 8.3139 | 0.5109 / 0.5553 / 0.5085 | -93.799% |
| Direct example wall (s) | 16.1544479 / 16.5913308 / 16.4316327 | 7.9557515 / 7.7722001 / 7.6849762 | -52.703% |
| Selected collision checks | 3440 / 3440 / 3440 | 56 / 56 / 56 | -98.372% |

Every pair preserved success, independent validation, termination, graph
counts, route, sampled path length, duration, minimum clearance, collision
state, and kinematic certificate. The smallest paired planner reduction was
50.067%; no averaged result hides a slower pair. The retained default grouped
option also formed zero groups and completed in 7.3898134 seconds with
identical physical output.

The user also requested a materially stronger input history for the
moving/deforming U.S. example. Its transform now starts at the native outline,
accumulates 12 degrees of rotation, grows nominally by 18% azimuth and 14%
elevation, and supplies larger nonrigid ripples, shear, and translation at
every five-second slice. Measured protected extents changed from
57.9658931-by-24.4877162 degrees to 67.7072812-by-30.1003469 degrees (+16.805%
and +22.920% after rotation). The changed example returned a
78.0069578875528-degree polyline, 76.5702550667827-degree sampled motion,
19.2943308306518-second duration, collision freedom, kinematic validity, and
independent validation.

Verification on the retained source produced:

- 59/59 focused tests passed in 110.6526933 seconds;
- all 18 corridor examples passed in literal fresh processes: 17 validated
  successes and the expected validated `noValidatedSeed` failure;
- the visible deforming-U.S. smoke succeeded with three visible figures;
- the complete repository suite passed 132/132 in 133.3061309 seconds;
- MATLAB Code Analyzer reported zero messages across 106 MATLAB files;
- `benchmark.csv` retained its original 17-column format, and `git diff
  --check` reported only line-ending conversion warnings.

The task adds 35 production MATLAB lines to a 13,978-line production baseline.
Applying the growth formula to the 35 task-added lines requires a 10.5%
representative reduction; the smallest paired reduction is 50.067%. The
14,013-line production tree remains above the 7,000-line target, which is an
existing repository limitation rather than something hidden by this local
speedup. Evidence covers the 40-obstacle family plus structurally different
deforming, static, and no-path checks; it proves no global scaling or
completeness claim.

Two PowerShell-variable matrix launches and one earlier static-check launch
hit MATLAB's pre-startup Windows `File system inconsistency` error and executed
no governed case. Literal commands completed every recorded example. One early
report used a stale field after a planner run; that incomplete log was excluded
from benchmark evidence.

## Shared option, goal, and obstacle helper consolidation — 2026-08-23

Task baseline: local `325-full-suite` at `a51f6e9`. The existing untracked
`docs/` directory was preserved and excluded from the change.

Three corridor helpers and seven HS3-local helpers were executable copies or
behavior-equivalent variants of neutral `azElInternal` requirements. All callers
were redirected to shared option, logical, goal, obstacle, polynomial, and
Bernstein implementations before the private files were removed. The shared
shape-at-time implementation retains the same interpolation and conservative
topology-change policy and additionally reuses a prepared shape for a
stationary matching-topology interval. No search, motion-construction,
candidate-selection, solver, validation, or public-dispatch algorithm changed.

Verification produced:

- focused pre-change baseline: 33/33 passed across shared infrastructure and
  both planners' option, moving-goal, dynamic-obstacle, and interpolation paths;
- post-change unit gate: 101/101 passed across
  `testObstacleInfrastructure`, `testFixedDurationAffineModel`,
  `testHs3Planner`, and `testAzElPlanner`;
- MATLAB Code Analyzer: zero messages across 20 modified MATLAB files;
- deleted-helper reference audit: zero matches;
- corridor-to-HS3 and HS3-to-corridor call audit: zero matches;
- `git diff --check`: passed with Windows line-ending conversion warnings only.

Physical MATLAB counts, using the same PowerShell `Get-Content` method on the
task baseline and worktree:

| Scope | Baseline | Current | Delta |
| --- | ---: | ---: | ---: |
| Production MATLAB | 70 files / 14,822 lines | 62 files / 14,256 lines | -8 files / -566 lines |
| Production and tests | 78 files / 18,291 lines | 70 files / 17,725 lines | -8 files / -566 lines |

The maintained examples, visible graphics, and complete repository suite were
not rerun because this cleanup changed ownership without changing planner
behavior. No `benchmark.csv` row was added because no maintained example
benchmark was executed. The first sandboxed MATLAB baseline launch failed
before startup with Windows `File system inconsistency`; the approved literal
launch completed the recorded 33-test baseline.

## Shared validation, timed-search, and test-requirement cleanup — 2026-08-23

Task baseline: pushed `325-full-suite` commit `625b243`. The unrelated
untracked `docs/`, `sandbox/explainAzElPlannerWalkthrough.m`, and
`sandbox/explainSingleUQuinticWalkthrough.m` paths were preserved and excluded
from the change and static-analysis count.

This cleanup made `+azElInternal` the single owner of three additional exact
or parameterized invariants: seed-corridor Bernstein inequalities, polynomial
format/dynamics/history validation, and time-expanded visibility search. The
polynomial validator accepts the method's range certificate as a callback, so
corridor retains exact stationary-point extrema while HS3 retains conservative
Bernstein bounds. The seed generators retain method-specific graph creation,
candidate ordering, diagnostics, and top-level policy. Twenty-four
behavior-identical planner requirements and seven fixture builders moved into
shared test support; method-specific tests remain in their original suites.

| Scope | Baseline physical / code | Current physical / code | Delta |
| --- | ---: | ---: | ---: |
| Production | 14,256 / 10,756 | 13,926 / 10,416 | -330 / -340 |
| Tests | 3,469 / 2,955 | 3,360 / 2,785 | -109 / -170 |
| Production and tests | 17,725 / 13,711 | 17,286 / 13,201 | -439 / -510 |

Verification on the retained worktree produced:

- focused post-extraction planner gates: 92/92 after each shared production
  or requirement-suite change;
- complete regression: 132/132 passed in 245.517 seconds, compared with the
  pre-change 132/132 baseline in 174.860 seconds. This single wall-time increase
  is unfavorable but is not attributed to production behavior without a
  repeated controlled timing comparison;
- MATLAB Code Analyzer: zero messages across 102 intended MATLAB files;
- corridor maintained examples: 18/18 in 119.882 seconds, comprising 17
  independently validated successes and the expected validated
  `noValidatedSeed` failure;
- HS3 maintained examples: 18/18 in 304.914 seconds with the same 17-success,
  one-expected-failure split;
- every successful example reported collision freedom and passing velocity,
  acceleration, jerk, and dynamics certificates;
- visible obstacle-free smoke: success, independent validation, and three
  visible figures;
- hidden expected-failure smoke: independently validated `noValidatedSeed`,
  zero selected seed, and two diagnostic figures without rerunning planning;
- `git diff --check`: passed with Windows line-ending conversion warnings.

HS3 emitted extensive existing `fmincon` near-singular or singular-matrix
warnings during several maintained cases, notably the accelerating-circles,
moving-barrier, and no-path examples. Their returned outcomes still passed the
independent gates, but the warnings remain adverse numerical-robustness
evidence and are not suppressed. Two long inline MATLAB launches and one
earlier nested-Git static launch failed before executing governed work with the
recorded Windows `File system inconsistency` startup error; short literal
commands and the temporary serial runner completed. The temporary runner was
removed after the matrix.

The 36 fresh example rows were appended to `benchmark.csv` under
`625b243+dedup-worktree`. This checkpoint is a maintainability and deployment-
size improvement; no planner-runtime, completeness, optimality, or trajectory-
quality improvement is claimed.

## Compact corridor cutover — 2026-08-24

This checkpoint replaces the legacy corridor motion stack on branch
`325-full-suite` from worktree source `8111d0f+compact-cutover-worktree`.
Static straight requests retain the exact direct endpoint quintic. Every
obstacle-path request is assembled through `solveCompactC3Candidate` and uses
compact C3/C4 motion; no HS3 or nonlinear solve is invoked by the corridor
method.

### Size and dependency evidence

Final obstacle-path ownership is 1,023 nonblank, noncomment MATLAB lines, below
the user-authorized 1,200-line limit:

| File | Noncomment lines |
| --- | ---: |
| `solveCompactC3.m` | 563 |
| `solveCompactC3Candidate.m` | 129 |
| `runCorridorPlanner.m` | 260 |
| `buildFixedDurationAffineModel.m` | 37 |
| `expandRouteClearance.m` | 34 |

The final count is 33 lines above the earlier 990-line checkpoint because the
two duration brackets now share an explicit preparation cache instead of
rebuilding the same affine basis. Against committed `8111d0f`, the four deleted
legacy files contain 1,309 physical and 1,011 noncomment lines:
`solveCorridorQuintic`, `retimeDynamicRoute`, `optimizeExactTraversal`, and
`spanTimeDemand`. Repository-wide text inspection found no executable caller
of those names. Raw `solveCompactC3` has one production caller, the shared
candidate adapter used by the planner and both scaling benchmarks.

### Correctness and regression evidence

The first complete suite run found two failures: the U-case timing and stage-
accounting tests expected one affine-basis build but observed two. Diagnostics
showed that an infeasible 0.5 route bracket performed two trials, then a 0.8
bracket rebuilt the identical basis and succeeded. Removing the first bracket
changed a dynamic selected topology, so that experiment was reverted. The
retained fix caches preparation across both brackets. Both affected test files
then passed 16/16, and a fresh complete run passed 133/133.

The final maintained-example capture ran all cases serially in one MATLAB
process. It produced 17 independently validated successes and the expected
validated `noValidatedSeed` failure. Every successful duration met or beat the
frozen legacy duration:

| Example | Compact duration (s) | Frozen legacy (s) | Wall (s) |
| --- | ---: | ---: | ---: |
| Alternating slalom | 10.7822098011 | 10.8556642584 | 3.3593887 |
| Az/El planning | 7.64390306784 | 7.64965634404 | 0.9348455 |
| Dense concave | 8.68804103025 | 8.69057318299 | 0.6536946 |
| Forty moving circles | 60.3618388755 | 62.4777398626 | 5.9253491 |
| Four accelerating circles | 22 | 22 | 4.1871735 |
| Intercept at set time | 12 | 12 | 0.0741688 |
| Intercept earliest | 6.11153430176 | 6.11153430176 | 0.1952022 |
| Moving barrier wait | 10.2149013519 | 10.3713875625 | 3.2450860 |
| Moving circle, no wrap | 8.68792447418 | 8.75122873615 | 6.3141087 |
| Moving/deforming U.S. | 14.1949781492 | 19.2943308307 | 21.3253900 |
| Expected no path | `NaN` | `NaN` | 0.0834465 |
| Obstacle free | 4.53112887415 | 4.53112887415 | 0.0501164 |
| Opening U | 11.7332839966 | 11.7353786786 | 5.7116175 |
| Straight moving target | 20.8695652174 | 20.8695652174 | 2.0964864 |
| Target exits obstacle | 24 | 24 | 1.9774286 |
| Two opposing Us | 22.1109676862 | 22.1609457614 | 0.8391154 |
| Single U | 21.8327355422 | 22.640860107 | 0.8751825 |
| Extreme U.S. outline | 6.10504292559 | 6.22216662415 | 11.9142326 |

Complete polyline, smoothed-length, collision, kinematic-certificate, and wall-
time fields are appended to `benchmark.csv`; no metrics were copied from an
earlier run. The no-path row records planner success false and independent
example validation true.

A structurally different obstacle detour with nonzero initial and terminal
velocity and acceleration passed independent validation at 7.49992177607 s.
Endpoint errors were `7.68e-16` and `3.79e-14 deg/s` for velocity and
`1.22e-14` and `1.37e-12 deg/s^2` for acceleration. The maintained regression
is `testCompactDetourPreservesNonzeroEndpointStates`.

### Scaling, graphics, and static evidence

| Case | Compact duration (s) | Frozen legacy (s) | Candidate wall (s) |
| --- | ---: | ---: | ---: |
| 1 turn | 6.60420575985 | 7.36692262286 | 1.7638058 |
| 5 turns | 19.8905274829 | 29.9538760389 | 0.9634780 |
| 10 turns | 36.3238796555 | 41.8816850826 | 0.8148864 |
| 20 turns | 68.3588042743 | 83.4675614946 | 2.9578101 |
| 12 hairpins | 140.56091613 | 164.828287993 | 3.7061584 |

All five scaling results passed independent validation. The first combined
scaling command used the wrong hairpin report-field name after the four turn
cases had passed; the hairpin was rerun with its documented
`IndependentValidation` field and passed. MATLAB Code Analyzer checked 99
intended production, benchmark, example, and test files with zero messages.
A visible U-case smoke produced three figures and passed; a hidden expected-
failure smoke produced two diagnostic figures and no selected motion.

The compact duration and topology searches remain finite. This evidence proves
the maintained cases and synthetic scales only; it is not a completeness,
global-optimality, or uniform wall-time-speedup claim.

## Lean HS3 bounded composite — 2026-08-24

The standalone 3,868-noncomment-line HS3 planner was replaced by a bounded
composite with exactly 1,200 HS3-owned nonblank, noncomment MATLAB lines. The
count includes the HS3 facade, option resolver, improvement controller, and
remaining solver kernel. It excludes shared/public infrastructure and the
compact baseline that the composite intentionally invokes; it is not a claim
that the complete transitive execution closure fits in 1,200 lines.

The compact result is immutable unless an opt-in HS3 attempt independently
validates and is no later, has no greater integrated squared polynomial jerk,
and strictly improves at least one measure. The default is
`EnableHs3Improvement=false`, so normal HS3 calls return the validated compact
motion with composition diagnostics and without nonlinear-solver work. A
failed compact result may recover to any independently valid HS3 candidate.
Rejected candidates and cooperative time-limit overruns remain reported.
Requested mesh refinement is retained in diagnostics but explicitly reports
`RefinementSupported=false`.

### Final verification matrix

- MATLAB Code Analyzer checked 94 live `.m` files with zero findings.
- The complete test suite passed 138/138 in 205.211 seconds. New coverage
  includes exact default-off compact success/failure parity, nonzero start and
  terminal velocity/acceleration, moving-target terminal derivatives, timed
  waits, direct-facade recursion safety, optional-improver timing, and
  compact-failure recovery diagnostics.
- All 18 maintained examples ran serially in separate MATLAB processes for
  both `corridorQuintic` and `hs3`. All 36 example requirements passed. The 17
  successful pairs had identical arrival times and physical trajectories; the
  no-path pair returned the same `noValidatedSeed` failure and passed its
  expected-failure requirement. The HS3 runs selected `corridorQuintic`, as
  required by the default-off bounded policy.
- A second serial HS3 capture produced the complete fresh rows appended to
  `benchmark.csv` under `hs3-compact-composite-worktree`; no compact metrics
  were copied into those rows.
- Graphics gates produced four valid U-shaped success figures with 680
  graphics objects and two expected-failure diagnostic figures with 342
  objects. The failure retained `noValidatedSeed` and selected seed zero.

### Scaling and hairpin comparison

Each public-method timing comparison used identical canonical inputs, seed
325, three interleaved repetitions, and exact comparison of the selected seed,
sampled position, velocity, acceleration, jerk, and arrival time.

| Case | Duration (s) | Compact median (s) | HS3 median (s) | HS3/compact | Compact max (s) | HS3 max (s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 turn | 6.60420575985 | 0.5845021 | 0.4958138 | 0.848267 | 2.3496011 | 0.6624357 |
| 5 turns | 19.8905274829 | 0.3807402 | 0.3773811 | 0.991177 | 1.0238277 | 0.4704385 |
| 10 turns | 36.3238796555 | 0.7929697 | 0.7951018 | 1.002689 | 0.8257864 | 0.8282841 |
| 20 turns | 68.3588042743 | 3.1064024 | 3.0717906 | 0.988858 | 3.1310520 | 3.1094026 |
| 12 hairpins | 140.56091613 | 1.9999662 | 2.0402493 | 1.020142 | 2.5180664 | 2.2851960 |

All five cases succeeded, passed independent validation, and produced exact
compact/HS3 physical parity. The 10-turn median was 0.27 percent slower and the
hairpin median was 2.01 percent slower, both within the predeclared 5 percent
median allowance; these unfavorable observations are retained. The hairpin
HS3 maximum was lower in this three-repeat sample. Timing noise and finite
benchmark topology prevent a claim of uniform speedup, completeness, or global
optimality.

### Diff-growth disclosure

The existing `solveCompactC3.m` changes by +382/-58 lines against `8111d0f`.
Its added responsibilities are the production C3/C4 representations, endpoint-
derivative mapping, exact jerk accounting, safe-side QPs, and reusable affine
preparation needed to replace four solver paths. Keeping the compact solver as
an optional sidecar was rejected because it preserved duplicate production;
removing the first duration bracket was tested and reverted after it changed a
dynamic selected topology. The existing `runCorridorPlanner.m` changes by
+125/-491 lines: additions normalize compact/direct candidates and preserve
stable diagnostics while 491 lines of legacy recovery orchestration disappear.
Both files are covered by the 133-test suite, 18-example matrix, scaling gate,
nonzero-endpoint regression, Code Analyzer, and graphics smokes.

`verification.md` itself changes by +121/-1 lines to retain the evidence above,
while `plan.md` changes by +51/-61 and therefore shrinks overall. New source is
the 139-physical-line shared compact candidate adapter and the 41-physical-line
clearance helper; they centralize maintained planner/benchmark behavior rather
than moving legacy code between files.

## Standalone Hermite-Simpson restoration — 2026-08-24

This section supersedes the compact-composition checkpoint above for the
current worktree. `PlannerMethod="hs3"` dispatches directly to a standalone
Hermite-Simpson planner. HS3 does not call the compact/corridor planner, use a
compact result as a seed or fallback, merge method results, or emit composition
diagnostics. Compact remains a separate public method and appears below only as
an external frozen comparator.

The HS3 production package contains 1,602 nonblank, noncomment MATLAB lines
across nine files, below the 2,000-line limit. Exact affine sensitivity maps
replace decision-variable finite differences for fixed-time constraints and
most earliest-arrival derivatives. The public planner preserves nonzero
initial and final velocity and acceleration, uses neutral topology seeds,
bounded collision relinearization and mesh refinement, and accepts only motion
that passes the canonical independent validator.

### Final verification matrix

- Complete automated suite: 137 passed, 0 failed, 0 incomplete in 360.861356 s.
- Maintained examples: 18/18 scenario outcomes passed. This includes 17
  independently validated successes and the expected diagnosable no-path
  result. Every success reported `SelectedMotionSource="hs3"`.
- Focused regression examples: static U 29.484334 s, timed-opening U 20.101672
  s, and moving-barrier wait 53.121601 s; each passed its scenario validator.
- Exact-sensitivity tests: 6/6 passed, including nonzero endpoint derivatives,
  static and deforming obstacles, and event-knot timing.
- Code Analyzer reported zero messages for the changed dispatcher, HS3 sources,
  and neutral request helpers. `git diff --check` passed.

### Standalone scaling and hairpin gate

All measurements used the public HS3 dispatch with deterministic seed 325 and
were independently revalidated. The benchmark rejects compact composition.

| Case | Total wall (s) | HS3 arrival (s) | Frozen compact arrival (s) | Outcome |
| --- | ---: | ---: | ---: | --- |
| 1 turn | 3.776525 | 6.61105444584 | 6.60420575985 | pass; HS3 0.006849 s later |
| 5 turns | 7.563524 | 18.62266933 | 19.8905274829 | pass; HS3 earlier |
| 10 turns | 63.799021 | 33.5725360484 | 36.3238796555 | pass; HS3 earlier |
| 20 turns | 64.116174 | 114.561539749 | 68.3588042743 | pass; HS3 materially later |
| 12 hairpins | 93.339104 | 128.179676226 | 140.56091613 | pass; under 120 s and earlier |

The hairpin run produced a 50-vertex route, returned HS3-owned motion, and
passed a separately rerun validator. The 20-turn arrival is the clearest
remaining quality regression; the results do not establish global optimality,
completeness, or uniform superiority over compact.

### Remaining limitations

Static scenes stop after the first independently valid HS3 topology to protect
the wall-time budget, so another untried static topology may arrive earlier or
use less jerk. Timed topology proposals are solved at their input-derived fixed
arrival before comparison, which preserves their wait law but may miss a faster
solution on the same topology. The planning deadline is cooperative and can be
slightly overrun by solver setup or final validation. Several difficult
earliest-arrival solves still emit near-singular `fmincon` warnings even when
independent validation passes.

## Extreme deforming U.S. and moving-sun example — 2026-08-24

`exampleMovingDeformingUSOutlineVisibility` now uses two independently moving
obstacles. The U.S. outline is active from 0 through 240 s, grows smoothly from
8% to 135% scale, deforms during the interior of the history, completes an
actual 180-degree endpoint reversal, and is inactive at the 300 s mission end.
A 32-vertex starburst sun traverses the lower scene for the complete mission.
Scenario validation measured an initial/peak U.S. area ratio of
0.00351165980796, endpoint-geometry cosine -1, maximum scale 1.35, inactive
final U.S. geometry, 34.6708365607 degrees of sun-centroid travel, and maximum
sun boundary elevation 16.6 degrees, below the 18-degree route start.

Both public planner methods passed independent and scenario validation with
jerk constraints enabled. Compact used a 41.6645269891-degree polyline,
41.9569827325-degree smoothed motion, 9.14130766846 s duration, and 40.659661 s
wall time. Standalone HS3 used the same polyline, a 42.4287030058-degree
smoothed motion, 75 s duration, and 108.428247 s wall time. The longer HS3
duration and wall time are retained as unfavorable evidence. Code Analyzer
reported zero messages for both modified source files, the maintained
example-requirement suite passed 5/5, and a plot-enabled run created three figures
with six axes while scenario validation remained passing.

The existing example changes by +109/-12 lines. Its net growth owns the second
obstacle, explicit scenario-history assertions, preserved result metadata, and
the local sun transform; the earlier script had no sun and could only validate
the planned trajectory, not the requested growth/rotation/disappearance
history. Keeping those assertions in a test-only duplicate was rejected because
the maintained example result must remain self-verifying. The private U.S.
helper changes by +39/-17 lines to centralize one transformation profile shared
by geometry generation and diagnostics. Both paths are covered by Code Analyzer,
the 5/5 requirement suite, compact and HS3 headless runs, and the graphics smoke.

## Interactive export and randomized moving-polygon stress — 2026-08-24

Both interactive tabs now enable `Export Bundle` after a planner call. The MAT
file stores the versioned `azElSandboxDiagnosis-v1` bundle: raw and canonical
scene geometry, exact planner inputs and resolved options, retained segment
results, latest success or failure result, independent validation, sandbox log,
environment metadata, and reproduction commands. Graphics handles and
callbacks are deliberately excluded. `testAzElSandboxDiagnosisExport` passed
3/3 cases: successful round-trip and reproduction, preservation of an
`endpointBlocked` failure, and hidden-UI button state on both tabs. Code
Analyzer reported zero messages for the exporter, sandbox, and focused test.

`benchmarkRandomMovingPolygonStress` generated deterministic three-obstacle
scenes with 5-to-12-vertex polygons, source radii from 6.5 to 8 degrees,
35-degree cross-frame translations, and 180-to-360-degree rotations. Each case
also retained an independently calculated lower-bound clearance for a boundary
witness route. Compact passed 11/12 seeds (`1001:1012`); seed 1011 returned
`noValidatedSeed` despite a 2.40166-degree witness lower bound. Standalone HS3
passed the seven exercised seeds (`1001:1006` and `1011`) with independent
validation, including seed 1011 in 30.061148 seconds.

Seed 1011 is a compact motion-construction failure, not an input, obstacle,
topology, or physical-feasibility failure. The primary 10-point visibility
seed exists, HS3 solves the identical normalized request, and successful-exit
compact QP trials become collision-free when validated with only the workspace
bounds widened. The compact QP constrains workspace position at 257 samples
while minimizing jerk; its seed-anchored barrier drives the curve to the
-20-degree elevation boundary. Exact continuous validation finds between-
sample undershoot of 0.0000017 to 0.000642 degrees and rejects the motions
before collision certification. The reported collision-resolution and
azimuth-wrap failures are therefore downstream prerequisite symptoms. No
planner behavior was changed or tolerance weakened in this checkpoint.

Final-source verification passed 140/140 automated tests with zero failures or
incomplete tests. The maintained example matrix passed 18/18 for compact in
91.618892 seconds and 18/18 for standalone HS3 in 551.197924 seconds; each
method produced 17 independently validated successes plus the expected
validated no-path outcome. The changed/new MATLAB files had zero Code Analyzer
messages. The modified U.S. example's plot-enabled smoke created three figures
and six axes while retaining passing scenario validation, and
`git diff --check` passed.

## Unified obstacle construction owner — 2026-08-24

`makeAzElObstacleData` now owns all three canonical construction operations:
fresh static or sampled construction, normalization of one imported canonical
record, and absolute reinflation of canonical arrays or nested cells. The
separate `normalizeAzElTimeObstacleData.m` and `inflateAzElObstacleData.m`
implementations were removed. `combineAzElObstacles`, focused tests, public
documentation, and the safety-margin idempotence requirement now call the single
owner. Established normalization and inflation error/warning identifiers were
preserved so malformed-input diagnostics did not change silently.

This is an ownership and file-count consolidation, not a source-size claim.
The three former owners contained 533 physical / 339 nonblank, noncomment
lines; the unified owner contains 576 / 464. Including the one-line caller
expansion in `combineAzElObstacles`, production changes by +44 physical / +126
noncomment lines while removing two public files. The added code is the
input-type dispatch and explicit local requirements needed to expose three
unambiguous call forms in one public function. Thin compatibility wrappers
were considered but rejected because the requested outcome was one owner and
all repository callers are migrated. The file remains below the 900-line
per-file limit.

Verification on the final source produced zero Code Analyzer messages for the
unified owner, combiner, and changed tests. The focused obstacle suite matched
its 5/5 baseline; the complete suite passed 140/140 in 337.935477 seconds.
The maintained examples passed 18/18 for compact in 91.419464 seconds and
18/18 for standalone HS3 in 549.982944 seconds. Each method retained 17
independently validated successes and the expected validated
`noValidatedSeed` outcome. Plot-enabled success and failure smokes produced
four figures/seven axes and two figures/two axes respectively, with both
scenario gates passing. No planner algorithm, obstacle geometry, margin,
tolerance, seed, or expected result changed, so no new benchmark row or
performance improvement is claimed.

## Corridor-quintic quality and runtime recovery — 2026-08-24

Two exported Rogue bundles isolated three general regressions in the compact
motion owner. First, refined exact-C3 routes doubled every interior knot and
could cross a dense-QP dimension cliff: one unselected 22-point route created
80 decisions and 2,142 barrier rows, then spent 65.0545 s in its motion solve.
Second, route refinement replaced span weights with equal weights even though
the resulting `173vs131` spans ranged from 2.801 to 28.599 degrees. That made
the equal-time peak velocity demand 629.189 versus 419.656 under geometric
allocation, a 49.93% inflation. Third, six earliest-arrival trials could leave
a multi-second feasible/infeasible bracket despite the public 0.001 s arrival
tolerance.

The retained input-driven rules keep doubled-knot exact C3 only through 48
decision variables, use continuous-C4 exact motion above that bound, allocate
refined span time from actual edge lengths, and use 14 trials for exact
earliest-arrival searches. Obstacle geometry, safety margin, limits, topology
opportunities, public options, independent validation, and HS3 were unchanged.
The saved `az_el_sandbox_goal_20260824_174716.mat` trajectory is bit-for-bit
unchanged in time, position, velocity, acceleration, and jerk at 37.845175 s,
while wall time fell from 77.9230141 to 9.2077994 s (88.18%). The formerly
dominant long seed now uses 40-decision C4, completes its motion work in 2.697 s,
and remains correctly unselected.

For `173vs131.mat`, the saved compact result arrived at 173.25 s with a
370.353-degree smoothed path. Fresh standalone HS3 arrived at 131.642423799 s
with a 319.454-degree path and 0.46758 jerk-squared. The retained compact rules
arrived at 136.042437744 s in 8.1911 s wall time, with a 311.101948-degree path,
0.02-degree clearance, and 0.0957002 jerk-squared. They recover 37.2076 s of
the 41.6076 s arrival regression, but compact remains 4.400014 s (3.34%) later
than HS3 on this case. A larger C3 map, pure length weighting, and axis-demand
weighting were measured and rejected because they either retained the timing
defect or worsened path length, jerk, or wall time.

The distinct 12-hairpin scale case passed independent validation with 96 C4
decisions, 138.455023011 s arrival, 8.2178 s wall time, and 0.02-degree
clearance; its prior frozen duration was 140.560916 s. All 18 maintained
compact example requirements passed serially: 17 independently valid successes
and the expected independently valid `noValidatedSeed` outcome. Plot-enabled
success produced three figures/six axes; the expected failure produced two
figures/two axes without rerunning planning. The complete automated suite
passed 144/144 in 643.151742 s wall time (632.433713 s summed test duration).
Code Analyzer reported zero messages for the changed production owner and its
focused regression test, and `git diff --check` found no whitespace errors.
HS3 moving-target tests still emit existing near-singular `fmincon` warnings;
those unfavorable diagnostics did not fail validation and were not suppressed.

Before push, the six reviewed task files were applied by themselves to a
detached `da52da8` worktree so unrelated dirty corridor, HS3, sandbox, and test
changes could not influence the gate. Code Analyzer again reported zero
messages, the focused planner suites passed 59/59, and the complete isolated
suite passed 142/142 with zero failures or incomplete tests in 709.815981 s
wall time (697.739404 s summed duration). The isolated count is two below the
dirty-worktree count because preexisting uncommitted tests were deliberately
excluded. The same expected HS3 near-singular warnings remained visible.

The cumulative dirty-worktree diff for `solveCompactC3.m` is +53/-12 lines;
reliable history cannot separate all earlier user edits from this task. The
681-line file now also owns the representation bound, geometric span
allocation, and bounded duration-search count because all three govern the
same compact exact-motion construction. Splitting those constants into a new
owner would add an interface without removing responsibility. The retained
alternatives and the two supplied bundles, focused regression, hairpin scale
case, full example matrix, graphics smokes, Code Analyzer, and full suite cover
that growth; the file remains below the 900-line limit.

## HS3-only production cutover — 2026-08-25

At source commit `67bc087` on branch `HS3-planner`, the public dispatcher was
reduced to HS3 and the complete corridor-quintic implementation and its
method-specific tests, benchmarks, and spline artifacts were removed. Active
source and documentation contain no references to `corridorQuintic`,
`azElPlannerMethods.corridor`, `solveCompactC3`, `compact planner`, or `quintic
planner`; historical benchmark and verification records remain unchanged.

Code Analyzer reported zero messages across all 83 remaining MATLAB files. A
direct HS3 planning request succeeded, passed independent validation, returned
`SelectedMotionSource = "hs3"` and `PlannerMethod = "hs3"`, and produced a
five-second trajectory. The focused suites passed 67/67 in 388.369877 seconds:
`testHs3OptionOwner`, `testHs3AffineSensitivity`, `testHs3Planner`,
`testPlannerStageTiming`, and `testExampleRequirements`. Existing near-singular
`fmincon` warnings were visible in moving-barrier and moving-target coverage.

The maintained examples were then launched serially in separate MATLAB
processes, but all attempts failed during MATLAB startup before example code
ran with `System Error: File system inconsistency`. A single-example retry
failed identically. Consequently no fresh example metrics or benchmark rows
were recorded, and the visible-success and expected-failure diagnostic-figure
checks remain untested in this environment.

A later single MATLAB process successfully ran the complete post-cutover test
suite: 75/75 passed, with zero failures or incomplete tests and 366.849286
seconds summed test duration. The same near-singular HS3 `fmincon` warnings were
visible and were not suppressed.

## Sandbox export recovery — 2026-08-25

The bundle writer successfully created and reloaded a 318,976-byte MAT file
from a real guidata-backed sandbox state, establishing that bundle assembly and
the core save operation were healthy. Two subsequent visible user runs showed
that converting only the success notification was insufficient. The identical
`Cell elements must be character arrays` error occurs at the earlier
`uiputfile` filter boundary because its cell elements were MATLAB strings.

The public sandbox snapshot now exposes
`ExportBundle(filePath, modeName)`, the UI button calls the same explicit-path
owner after its dialog returns, and the writer verifies both a nonempty file
and the required `diagnosisBundle` MAT variable. UI failures display their
actual message, identifier, and first source location in an error dialog. The
file-dialog filter, success notification, `save`, `whos -file`, `version`, and
`datetime` calls now receive character arguments for cross-version
compatibility. The focused export suite passed 4/4 and Code Analyzer reported
zero messages before this visible-only compatibility correction; concurrent
MATLAB startup failed before a final focused or visible callback rerun.

Pre-run export now prepares a copy of the live mode state by reading current
controls and rebuilding canonical obstacle geometry without calling the
planner. Complete Goal Mode scenes retain exact replayable planner inputs;
partial Goal Mode and Free Mode scenes retain their explicit request geometry
and controls. Pre-run bundles use `PlanningState = "notRun"`,
`HasPlannerResult = false`, an empty `Result`, and no validation claim. A new
focused test exercises the public no-dialog export before any planner call.
The expanded export suite passed 5/5 with zero failures or incomplete tests,
and Code Analyzer reported zero messages across the two production files and
focused test.

## Dynamics-timescale mesh verification — 2026-08-26

Changed `+azElPlannerMethods/+hs3/plan.m` so an untimed spatial detour with
more than two route legs starts at twice the configured HS3 mesh only when its
estimated duration per base segment exceeds twice the supplied
`maxVelocity_deg_s ./ maxAcceleration_deg_s2` time scale. Direct and timed
seeds retain the configured mesh, and the existing maximum segment count still
bounds the result. Code Analyzer reported zero findings and the complete HS3
package remains exactly 2,000 noncomment production lines.

Focused A/B evidence:

- 40 moving circles: 61.2011842765 -> 58.6189853057 s arrival;
  smoothed length 125.185941203 -> 123.380530717 degrees; current wall
  18.873958 -> 24.875451 s; independent validation and certificates pass.
- Rogue 180/360 horizons: 86.5467293065 and 86.5467226767 s, both valid,
  differing by 0.000006629817 s. The preceding commit reported
  88.2939404925 and 88.2939359679 s.
- Static U control: unchanged at 22.6308876389 s. Timed moving-circle
  control: unchanged at 8.64603156476 s.
- Rejected blanket-20 diagnostics: static U regressed to 22.6623174130 s;
  moving circle improved to 8.560546875 s but wall rose to 16.976155 s.

Every maintained example was then run headlessly in its own serial MATLAB
process. All 18 expected outcomes passed: 17 independently validated successes
and the independently validated `noValidatedSeed` case. Exact metrics and wall
times are appended to `benchmark.csv`. A visible `exampleAzElPlanning` run
created three figures and 526 graphics objects; a hidden plotted no-path run
created two diagnostic figures with nine rejected edges and no trajectory.

`testHs3Planner` passed 51/51 in 43.414460 seconds. The authoritative complete
suite passed 82/82 in 50.011338 seconds with warnings enabled. An earlier
81/82 diagnostic run is invalid as a suite result because its harness disabled
all warnings, preventing the required unknown-option warning from reaching
`verifyWarning`; no repository assertion failed in that run. `git diff
--check` reports only existing LF-to-CRLF conversion notices.

## Direct dynamics-mesh jump — 2026-08-26

The pushed dynamics-timescale rule reached its best long-detour results by
starting at 20 segments and then refining to 40. The retained follow-up keeps
the ordinary configured 10-segment first solve and, only when the same
input-derived long untimed multi-leg predicate is true, makes its single
quality pass jump directly by 4x. Other candidates retain the ordinary 2x
refinement. This removes a redundant intermediate transcription without a new
option, scenario identifier, obstacle property, seed, tolerance, or extra
production line; the complete HS3 package remains exactly 2,000 noncomment
lines.

Focused evidence preserves the 40-circle result exactly at 58.6189853057
seconds arrival, 110.807922148-degree polyline, and 123.380530717-degree
smoothed motion. Final serial wall time was 22.727164 seconds versus 24.875451
seconds in the pushed 20-to-40 verification. A focused repeat took 22.398936
seconds. The structurally distinct neutral-circle regression improved from
80.2105179472 seconds at 20 segments to 78.7444420156 seconds at 40 segments;
its wall time rose from 4.140629 to 7.139318 seconds. The regression now
requires one 40-segment quality pass and arrival below 79 seconds.

A direct 10-to-30 alternative was measured and rejected despite its lower
19.113692-second 40-circle wall time: arrival regressed to 60.1588345587
seconds and smoothed motion grew to 124.541423742 degrees. The retained 40-
segment result therefore does not trade away the arrival improvement for the
lower runtime. No uniform speedup is claimed.

Both rogue horizons reproduce on seed 2 and 40 segments. `failure.mat` at the
180-second horizon reaches 86.5467293065 seconds in 21.522385 seconds wall;
`successwhenincreasehorizon.mat` at 360 seconds reaches 86.5467226767 seconds
in 20.076234 seconds wall. Their 6.630-microsecond arrival difference preserves
the repaired horizon invariance, and both pass independent collision and all
derivative certificates.

All 18 maintained examples ran serially in fresh MATLAB processes. Seventeen
successes and the expected `noValidatedSeed` outcome independently validate;
their exact metrics are appended to `benchmark.csv`. The moving/deforming U.S.
case remains the dominant wall-time weakness at 49.573514 seconds. A visible
success produced three figures and 526 objects. A corrected failure plot probe
produced two figures, 341 objects, and nine rejected edges; an earlier reporter
queried the wrong diagnostics nesting after the valid example and is retained
as a `NaN`-wall benchmark row rather than hidden.

The focused long-detour regression passed in 7.1857 seconds,
`testHs3Planner` passed 51/51 in 42.668797 seconds, and the warnings-enabled
complete suite passed 82/82 in 49.490930 seconds. Code Analyzer reported zero
findings across 84 MATLAB files. `git diff --check` reports only existing
LF-to-CRLF notices. Two user-owned MATLAB processes, PIDs 8516 and 31968,
remained alive and were never signaled.

## Deforming-outline stage diagnosis — 2026-08-26

An end-to-end profile at local commit `6427ce9` separated the 51.619861-second
moving/deforming U.S. example into 20.669104 seconds of scenario construction
and 26.1176 seconds of planning. Construction spent 16.556995 seconds in 60
`polybuffer` calls. Planning reported 8.3759 seconds topology, 3.4312 seconds
corridor construction, 11.0734 seconds motion solving, 1.0686 seconds
collision checking, and 0.3503 seconds final validation. This establishes that
the largest remaining runtime is shared between exact obstacle construction
and planning rather than being solely an HS3 solver issue.

A boundary-classification opt-out reduced an isolated 14,000-vertex geometry
microbenchmark from 0.898543 to 0.386355 seconds with identical coordinates,
but failed the end-to-end retention gate. Its profiled example took 52.626923
seconds versus 51.619861 seconds, and a fresh run's 49.326170 seconds was only
0.247344 seconds below the preceding 49.573514-second matrix row. The extra
interface was reverted. The focused obstacle suite remained 6/6, and no
production code from this diagnostic is retained.

Exact buffer reuse is already active for the translated sun. It cannot be
applied to the U.S. history because every interior slice independently changes
scale and applies coordinate-coupled nonlinear deformation before rotation.
Exact duplicate and collinear removal was previously measured as negligible,
and vectorized buffering improved only about 1.5%. Geometry reduction,
coarser history, parallel-toolbox dependence, and affine reuse were therefore
not presented as safe runtime improvements. The 49-second unfavorable runtime
remains explicit.

## Obstacle-free bounded fixed-time search — 2026-08-26

### Retention gate and implementation

The baseline was local commit `6427ce9` on `HS3-planner`, with only the
documented runtime-profile records dirty. The primary metric was independently
validated obstacle-free earliest arrival; collision, workspace, endpoint,
velocity, acceleration, jerk, stable API, diagnostic format, package size, and
representative runtime were hard invariants. The retained change reuses the
existing bounded fixed-arrival feasibility search for an obstacle-free
earliest-arrival request and begins that search at twice the configured
collocation mesh, still capped by `MaximumCollocationSegmentCount`. The
condition is determined only by empty obstacle input and goal-time policy.
Nonempty-obstacle and fixed-arrival paths do not enter it.

The maintained example improves from 4.60777936881 to 4.5458984375 seconds,
0.06188093131 seconds or 1.343% earlier. Final wall time improves from
3.882538 to 2.956477 seconds, a 23.85% reduction. A distinct direct request
from `[-3, 1]` to `[5, -2]` with asymmetric velocity, acceleration, and jerk
limits independently validates at 5.70751953125 seconds on 20 segments using
the linear fixed-time constraint representation. The former local
`topologyAlignedSegmentCount` helper was removed and the bounded expression
was kept in the owning execution sequence. The complete nine-file HS3 package
remains exactly 2,000 nonblank, noncomment lines.

An isolated detached worktree at `67bc087` supplied the requested
325-full-suite comparison. Its obstacle-free example reached 4.53112887415
seconds in 4.588076 seconds wall; current remains 0.01476956335 seconds later
but runs 1.631599 seconds faster. Its identical wide-U input reached
21.8327355422 seconds in 6.057014 seconds, versus current 22.6308876389 seconds.
Current fixed-time attempts at the 325 duration remained collision-unresolved,
including a solve seeded from the 325 trajectory. An 80-segment current solve
reached only 22.5797733 seconds and raised wall time from 13.019699 to
18.815125 seconds. Warm-start, doubled corridor sampling, and doubled reserve
experiments were neutral or regressive and were fully reverted. These results
do not support broadening the obstacle-free rule to obstacle cases.

### Final serial maintained matrix

Every maintained example ran in its own fresh MATLAB process with figures and
animation disabled. Jerk was enabled in every row.

| Example | Planner / validation | Polyline deg | Smoothed deg | Duration s | Collision / certificate | Wall s | Termination |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.060439635 | 16.7100287566 | 11.1855739606 | 1 / 1 | 7.340792 | `goalReached` |
| `exampleAzElPlanning` | 1 / 1 | 11.152119519 | 11.4464747617 | 7.57952069664 | 1 / 1 | 4.887374 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | 1 / 1 | 12.7007215595 | 13.9293484742 | 8.64603162385 | 1 / 1 | 5.618792 | `goalReached` |
| `exampleFortyMovingCircleGrid` | 1 / 1 | 110.807922148 | 123.380530717 | 58.6189853057 | 1 / 1 | 22.624950 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 24.3633026735 | 27.8702009821 | 22 | 1 / 1 | 25.939702 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 | 2.334870 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.31007759339 | 7.31051404817 | 6.11702719116 | 1 / 1 | 5.975042 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.2314453125 | 1 / 1 | 13.101632 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12 | 12.7689032232 | 8.64603156476 | 1 / 1 | 9.562227 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 41.5785140688 | 40.7424283094 | 8.75061035156 | 1 / 1 | 50.142665 | `goalReached` |
| `exampleNoPathAzElMotion` | 0 / 1 | NaN | NaN | NaN | NaN / NaN | 1.434626 | `noValidatedSeed` |
| `exampleObstacleFreeAzElMotion` | 1 / 1 | 4.472135955 | 4.47285938216 | 4.5458984375 | 1 / 1 | 2.956477 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | 1 / 1 | 10 | 10.0912159691 | 11.8560791016 | 1 / 1 | 16.487221 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 21.4031702791 | 14.2200520815 | 20.8695652174 | 1 / 1 | 9.071585 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.1815464898 | 21.3509241121 | 24 | 1 / 1 | 8.791014 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.5077116377 | 24.4201122273 | 21.9090824092 | 1 / 1 | 12.949268 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | 1 / 1 | 34.9425880405 | 41.5363500661 | 22.6308876389 | 1 / 1 | 16.447739 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.2394635087 | 24.6064786878 | 6.3679977362 | 1 / 1 | 48.172937 | `goalReached` |

The 40-circle, static-U, moving-circle, moving/deforming-U.S., and extreme-U.S.
arrival values match the preceding committed behavior. The final matrix has 17
independently validated successes plus the independently validated expected
`noValidatedSeed` result. Existing near-singular solver warnings remained
visible on timed cases.

### Rogue, graphics, tests, and static gates

With seed 2 and a 40-segment cap, `failure.mat` at a 180-second horizon reaches
86.5467293065 seconds in 20.844898 seconds wall. The 360-second
`successwhenincreasehorizon.mat` reaches 86.5467226767 seconds in 20.803370
seconds wall. Both independently pass collision and every derivative
certificate; their arrival difference is 6.630 microseconds.

A visible `exampleAzElPlanning` run produced three figures, six axes, and 530
graphics objects. A hidden plotted expected failure produced two diagnostic
figures, two axes, 344 objects, and no trajectory. The warnings-enabled
complete suite passed 83/83 with zero failures or incomplete tests in
51.100108 seconds wall and 44.390953 seconds summed duration. Code Analyzer
reported zero findings across all 84 MATLAB files. `git diff --check` reported
only the existing LF-to-CRLF notices. User-owned MATLAB PIDs 8516 and 31968
remained alive and were never signaled. A final two-file Code Analyzer retry
passed with zero findings after one preceding MATLAB process failed during
startup with `System Error: File system inconsistency`; that failed process
did not execute repository code.

## Shared helpers and HS3 internal subpackages — 2026-08-26

Source under test was `HS3-planner` at
`0302439+helper-and-hs3-subpackage-worktree`. Each example ran in its own fresh
MATLAB R2024b batch process with `PlotOutputs=false`,
`FigureVisible="off"`, `ShowAnimation=false`, and `ShowKinematics=false`.

| Example | Jerk | Planner / independent validation | Polyline (deg) | Smoothed (deg) | Duration (s) | Collision / certificate | Wall (s) | Termination |
| --- | ---: | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleMovingCircleNoAzimuthWrap` | 1 | 1 / 1 | 12 | 12.7689032232 | 8.64603156476 | 1 / 1 | 13.708932 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | 1 | 1 / 1 | 10 | 10.0912159691 | 11.8560791016 | 1 / 1 | 14.261098 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | 1 | 1 / 1 | 34.9425880405 | 41.5363500661 | 22.6308876389 | 1 / 1 | 15.192708 | `goalReached` |

All three trajectory metrics exactly match the preceding
`de372d5+spatial-route-cleanup-worktree` records. The opening-U run retained
the known repeated singular or badly scaled optimizer warnings, but its final
trajectory independently passed collision and every applicable derivative
certificate. No broader tests, other examples, visible graphics, or Code
Analyzer checks were run in this focused pass.

## High-level namespace, plotting dashboard, and result-format verification

Source under test was HS3-planner at
0302439+namespace-and-format-worktree on 2026-08-27. Production source moved
to +obstacleAvoidance; HS3 remained the separate normal hs3 product.

- Code Analyzer: 0 findings across 93 maintained MATLAB files.
- Unit tests: 117 passed, 0 failed, 0 incomplete.
- Plot dashboard/GIF smoke test: 4 synchronized kinematic axes and a
  95,507-byte GIF.
- Actual example result: 27 normal planner fields, with no Example-prefixed or
  PlotHandles fields.
- Visible exampleObstacleFree: success and validation passed; 3 figures.
- Expected exampleNoPath: independently validated noValidatedSeed;
  workspace and visibility diagnostic figures both created.
- git diff --check: no whitespace errors; only Git's existing LF/CRLF
  conversion notices.

Every maintained example ran headlessly in its own fresh MATLAB process with
plots and animation disabled. Jerk was enabled in every row.

| Example | Planner / validation | Polyline deg | Smoothed deg | Duration s | Collision / certificate | Wall s | Termination |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| exampleAlternatingSlalom | 1 / 1 | 16.0193197983 | 16.8275815277 | 11.1855739607 | 1 / 1 | 7.570137 | goalReached |
| exampleDenseConcaveObstacle | 1 / 1 | 12.7007215595 | 13.9293484742 | 8.64603162385 | 1 / 1 | 5.618033 | goalReached |
| exampleFortyMovingCircleGrid | 1 / 1 | 110.807922148 | 123.380530717 | 58.6189853057 | 1 / 1 | 23.641960 | goalReached |
| exampleFourAcceleratingCircles | 1 / 1 | 20 | 20 | 22 | 1 / 1 | 4.484687 | goalReached |
| exampleInterceptMovingTargetAtSetTime | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 | 2.338153 | goalReached |
| exampleInterceptMovingTargetEarliest | 1 / 1 | 7.31007759339 | 7.31051404817 | 6.11702719116 | 1 / 1 | 7.346781 | goalReached |
| exampleMovingBarrierWait | 1 / 1 | 10 | 10 | 10.2314453125 | 1 / 1 | 13.920832 | goalReached |
| exampleMovingCircleNoAzimuthWrap | 1 / 1 | 12 | 12.7689032232 | 8.64603156476 | 1 / 1 | 12.392204 | goalReached |
| exampleMovingDeformingUSOutlineVisibility | 1 / 1 | 41.5785140688 | 40.7424283094 | 8.75061035156 | 1 / 1 | 54.129820 | goalReached |
| exampleNoPath | 0 / 1 | NaN | NaN | NaN | NaN / NaN | 1.466201 | noValidatedSeed |
| exampleObstacleAvoidance | 1 / 1 | 11.152119519 | 11.4464747617 | 7.57952069664 | 1 / 1 | 5.193712 | goalReached |
| exampleObstacleFree | 1 / 1 | 4.472135955 | 4.472135955 | 4.5458984375 | 1 / 1 | 3.260685 | goalReached |
| exampleOpeningUShapedObstacle | 1 / 1 | 10 | 10.0912159691 | 11.8560791016 | 1 / 1 | 13.700528 | goalReached |
| exampleStaticUShapedObstacle | 1 / 1 | 34.9425880405 | 41.5363500661 | 22.6308876389 | 1 / 1 | 17.295045 | goalReached |
| exampleStraightTargetAlternatingOcclusion | 1 / 1 | 21.4031702791 | 13.678271908 | 20.8695652174 | 1 / 1 | 8.449397 | goalReached |
| exampleTargetExitsObstacle | 1 / 1 | 20.1815464898 | 20.3320561588 | 24 | 1 / 1 | 7.184225 | goalReached |
| exampleTwoOpposingUVisibilityGraph | 1 / 1 | 24.035784715 | 24.4189853364 | 21.9090835611 | 1 / 1 | 8.649243 | goalReached |
| exampleUSOutlineExtremeVisibility | 1 / 1 | 22.2394635087 | 24.6064786878 | 6.3679977362 | 1 / 1 | 55.227758 | goalReached |

The moving-barrier and opening-U rows were rerun after their local warning
suppression was added; both produced concise output and retained identical
trajectory metrics and independent certificates. The warning state is restored
immediately after each planner call.

## Prepared constraint-layout verification — 2026-08-27

Source under test was `HS3-planner` at
`0534edb+constraint-layout-worktree`. The retained adapter change prepares
corridor interval maps, row offsets, and final-time event locations once per
solver attempt, then passes them through the existing HS3 constraint callback.
Moving geometry values, polynomial values, collision checking, and validation
remain evaluated at every required time.

### Matched profile and focused gates

The identical headless Opening-U profile improved from 41.5774063 to
40.2039387 seconds (3.30%). `evaluateTrajectoryConstraints` improved from
5.45811353 to 4.13240582 seconds, `createConstraintMatrices` from 2.31449291
to 1.83661551 seconds, and its corridor value block from 1.25470921 to
0.641739103 seconds. The solver still spent 22.1272051 seconds in augmented
matrix factorization. Output remained exactly 10 / 10.0912159691 degrees and
11.8560791016 seconds.

Code Analyzer reported zero findings for the three edited production files and
the edited test. The clean-path focused suites passed 8/8 polynomial and
constraint tests, 59/59 planner tests, and 24/24 standalone/architecture tests.
The new dynamic-geometry test compares prepared and unprepared inequality,
equality, and both gradient matrices exactly. The complete suite passed 120/120
with zero failures or incomplete tests in 41.9212 seconds summed test time.

The first focused invocation omitted the repository package path and therefore
reported five unresolved-package errors before exercising those tests. A later
`genpath` invocation exposed an untracked `.claude` worktree test file and
reported only the older seven-test inventory. Both environment mistakes were
diagnosed; authoritative reruns used only the repository root, `hs3`,
`examples`, and `tests` paths, and `which` confirmed the edited checkout.

### Final serial maintained matrix

Each example ran in its own fresh MATLAB process with plots and animation
disabled. Jerk was enabled in every row.

| Example | Planner / validation | Polyline deg | Smoothed deg | Duration s | Collision / kinematic | Wall s | Termination |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0193197983 | 16.8275815277 | 11.1855739607 | 1 / 1 | 6.459507 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7007215595 | 13.9293484742 | 8.64603162385 | 1 / 1 | 5.1187829 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 | 3.9738544 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 | 2.0905206 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.31007759339 | 7.31051404817 | 6.11702719116 | 1 / 1 | 5.8297634 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.2314453125 | 1 / 1 | 16.1610637 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12 | 12.7689032232 | 8.64603156476 | 1 / 1 | 8.8386864 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40 | 43.0751355347 | 7.96286899667 | 1 / 1 | 182.935442 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN | 1.4162175 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4464747617 | 7.57952069664 | 1 / 1 | 4.5816492 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.5458984375 | 1 / 1 | 2.6313584 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10.0912159691 | 11.8560791016 | 1 / 1 | 38.9261867 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 41.5363500661 | 22.6308876389 | 1 / 1 | 11.7993308 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 21.4031702791 | 13.678271908 | 20.8695652174 | 1 / 1 | 5.7791864 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.5244479986 | 20.6764423274 | 24 | 1 / 1 | 5.7329698 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.035784715 | 24.4189853364 | 21.9090835611 | 1 / 1 | 6.4064281 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.2394635087 | 24.6064786878 | 6.3679977362 | 1 / 1 | 39.5964486 | `goalReached` |

The first moving/deforming run measured 200.066640 seconds, then repeated at
182.935442 seconds against the 183.4952231-second baseline. Its path and arrival
metrics were identical in both runs, so the unfavorable first wall time remains
documented as noise rather than omitted. A visible `exampleObstacleFree` run
created three figures and six axes. A hidden plotted `exampleNoPath` run created
two diagnostic figures and two axes and returned `noValidatedSeed`.

No MATLAB process that predated this work was stopped or signaled. The retained
change does not claim global optimality, completeness, or uniform runtime gain.

## Batched occupancy and deferred shape allocation — 2026-08-27

Source under test was `HS3-planner` at
`2cc0988+batched-occupancy-worktree`. Multi-ring occupancy batches points into
the existing signed-clearance helper, and active shape queries no longer create
empty geometry or `polyshape` placeholders that are immediately replaced.

### Matched profile and focused gates

The exact Opening-U profile baseline at `2cc0988` was 41.1939427 seconds. The
final matched profile was 40.2238949 seconds with exact output parity:
10 degrees selected, 10.0912159691 degrees smoothed, and 11.8560791016 seconds
duration. Targeted profile reductions were:

- `queryObstacleOccupancyAtTime`: 1.63920621 to 1.24773731 seconds;
- `occupancyOnly`: 1.16577671 to 0.913307407 seconds;
- `complexShapeOccupancy`: 0.627571103 to 0.546502604 seconds;
- `shapeAtTime`: 0.880268904 to 0.580085704 seconds over 8,075 calls;
- `pointPolygonClearance`: 1,972 to 1,782 calls with the same decisions.

Code Analyzer reported zero findings for both changed production files, the
focused obstacle test, and the temporary serial-example harness. The focused
obstacle suite passed 10/10. It compares batched and pointwise multi-ring
queries under both boundary policies and freezes full/geometry-only return
types for inactive, interpolated, single-slice, and topology-changing cases.
The complete repository suite passed 122/122 with zero failed or incomplete
tests in 48.6476833 seconds.

### Final serial maintained matrix

Every example ran headlessly in its own fresh MATLAB process with plots and
animation disabled. Jerk was enabled in every row.

| Example | Planner / validation | Polyline deg | Smoothed deg | Duration s | Collision / kinematic / certificate | Wall s | Termination |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0193197983 | 16.8275815277 | 11.1855739607 | 1 / 1 / 1 | 6.1661358 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7007215595 | 13.9293484742 | 8.64603162385 | 1 / 1 / 1 | 4.7708265 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 / 1 | 3.6128053 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 / 1 | 1.7547402 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.31007759339 | 7.31051404817 | 6.11702719116 | 1 / 1 / 1 | 5.1434321 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.2314453125 | 1 / 1 / 1 | 16.2283167 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12 | 12.7689032232 | 8.64603156476 | 1 / 1 / 1 | 8.002814 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40 | 43.0751355347 | 7.96286899667 | 1 / 1 / 1 | 178.568018 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN / 1 | 0.8864936 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4464747617 | 7.57952069664 | 1 / 1 / 1 | 4.3293132 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.5458984375 | 1 / 1 / 1 | 2.3133985 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10.0912159691 | 11.8560791016 | 1 / 1 / 1 | 38.4144474 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 41.5363500661 | 22.6308876389 | 1 / 1 / 1 | 11.4157737 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 21.4031702791 | 13.678271908 | 20.8695652174 | 1 / 1 / 1 | 4.8174658 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.5244479986 | 20.6764423274 | 24 | 1 / 1 / 1 | 5.167399 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.035784715 | 24.4189853364 | 21.9090835611 | 1 / 1 / 1 | 6.0696728 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.2394635087 | 24.6064786878 | 6.3679977362 | 1 / 1 / 1 | 38.7741662 | `goalReached` |

All 16 successes independently passed collision, continuous kinematic, and
collision-resolution certificates. Their path lengths and arrival times match
the preceding `0534edb+constraint-layout-worktree` matrix exactly. The expected
no-path result independently validated its stable failure diagnostics. A
visible no-path run took 4.5840717 seconds and created two diagnostic figures
with two axes. No pre-existing MATLAB process was stopped or signaled.

## Independent engines and obstacle-owned routing — 2026-08-27

Source under test was `HS3-planner` at checkpoint
`6e15a36+separated-engine-worktree`. The neutral trajectory dispatcher and its
shared internal package were removed. Direct calls are now
`ruckigEngine.solve(...)` and `hs3Engine.solve(...)`; obstacle-aware selection
occurs inside `obstacleAvoidance.planner.plan`.

### Static and focused verification

- `git diff --check` reported no whitespace errors.
- `testRuckigEngine` and `testArchitectureBoundaries` passed 21/21.
- `testStandaloneHs3Kernel` passed 18/18.
- `testPlannerStageTiming` and `testPlannerOptions` passed 8/8.
- `testHs3Planner` passed 62/62 after an azimuth-wrapping regression was fixed
  by preserving the existing periodic-axis exemption in the Ruckig request.
- The complete `tests` tree passed 144/144 with zero failed or incomplete tests
  in 55.099274 summed test seconds.

The full suite used only the repository, `tests`, `trajectory`, and `examples`
paths. Matrix-conditioning warnings were suppressed in the runner output only;
no planner tolerance or assertion was weakened.

### Warm routing cost

Thirty alternating repetitions of the same two-axis rest-to-rest request gave:

| Call | Median ms | p10 ms | p90 ms |
| --- | ---: | ---: | ---: |
| `ruckigEngine.solve` | 1.8168 | 1.4375 | 2.77175 |
| `obstacleAvoidance.planTrajectory` | 7.0647 | 6.27895 | 8.2675 |

The 5.2479 ms median difference includes obstacle-planner input normalization,
endpoint checks, local result translation, and canonical independent
continuous validation. It excludes the removed neutral dispatcher, previously
measured at approximately 0.4--0.6 ms of duplicate routing work.

### Final serial maintained matrix

Each example ran in a fresh MATLAB process with plots and animation disabled.
Jerk was enabled in every row.

| Example | Planner / validation | Polyline deg | Smoothed deg | Duration s | Collision / kinematic | Wall s | Termination |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0193197983 | 16.8275815277 | 11.1855739607 | 1 / 1 | 7.253537 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7007215595 | 13.9293484742 | 8.64603162385 | 1 / 1 | 5.645959 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 | 4.494188 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 | 2.309880 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.31007759339 | 7.31051404817 | 6.11702719116 | 1 / 1 | 6.218571 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.2314453125 | 1 / 1 | 17.242832 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12 | 12.7689032232 | 8.64603156476 | 1 / 1 | 8.991519 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40 | 43.0751355347 | 7.96286899667 | 1 / 1 | 166.070652 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN | 1.292197 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4464747617 | 7.57952069664 | 1 / 1 | 4.717992 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 | 0.976784 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10 | 11.8560791016 | 1 / 1 | 41.859897 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 41.5363500661 | 22.6308876389 | 1 / 1 | 13.717231 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 21.4031702791 | 13.678271908 | 20.8695652174 | 1 / 1 | 5.744790 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.5244479986 | 20.6764423274 | 24 | 1 / 1 | 5.910711 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.035784715 | 24.4189853364 | 21.9090835611 | 1 / 1 | 6.724818 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.2394635087 | 24.6064786878 | 6.3679977362 | 1 / 1 | 43.421555 | `goalReached` |

A visible `exampleObstacleFree` run routed through `ruckigDirect`, passed
canonical validation, and created three figures with six axes in 7.490440
seconds. A visible `exampleNoPath` run returned `noValidatedSeed` and created
two diagnostic figures with two axes in 7.564985 seconds. No pre-existing
MATLAB process was stopped or signaled.

## Bounded interactive sandbox planning — 2026-08-27

Source under test was `HS3-planner` at `dbdb9ae+sandbox-worktree`. The focused
benchmark approximated the reported screenshot with start `[-35 -2]` degrees,
goal `[-7 -75]` degrees, one four-corner polygon, a 180-second horizon, and the
shown `[95 -25]` degree zero-start obstacle translation. Plotting and animation
were absent and both comparisons used earliest arrival plus the same limits,
geometry, safety margin, and public independent validation.

The production-default baseline succeeded in 123.0002 seconds at 40.7616
seconds arrival. Exclusive stage timing was 1.5332 seconds topology, 0.6203
seconds corridor construction, 119.3994 seconds motion solving, 0.5740 seconds
collision checking, 0.2596 seconds final validation, and 0.5618 seconds
unattributed. Thus HS3 motion solving accounted for 97.1% of total time.

The retained sandbox-only defaults use two seeds, 8 initial and 16 maximum
collocation segments, no mesh-refinement pass, 80 NLP iterations, 5,000
function evaluations, and 0.05-second arrival tolerance. The identical request
succeeded and independently validated in 4.3057 seconds at 51.7410 seconds
arrival. Stage timing was 0.5784 seconds topology, 0.1580 seconds corridor
construction, 2.8452 seconds motion solving, 0.1025 seconds collision checking,
0.0991 seconds final validation, and 0.4687 seconds unattributed. This is a
28.57x wall-time improvement with a documented 26.93% arrival-time penalty.
Production planner defaults were not changed.

The same polygon without motion provided the second declared case. Production
defaults succeeded in 14.0421 seconds at 41.7830 seconds arrival. The retained
sandbox defaults succeeded and independently validated in 5.2767 seconds at
43.9204 seconds arrival: 2.66x faster with a 5.12% later arrival.

Code Analyzer reported zero findings for the sandbox and focused test files.
The focused sandbox-diagnosis suite passed 8/8, including a new contract test
that freezes the UI work limits and confirms the production defaults remain at
five seeds and two mesh-refinement passes. `git diff --check` reported no
whitespace errors. No maintained examples were executed for this sandbox-only
change.

The subsequent Run-animation addition was exercised through the actual Goal
Mode Run-button callback in a visible sandbox. An obstacle-free request
succeeded, passed independent validation, and produced live animation figure
and axes handles. The smoke runner then closed only the two figures it created.
Automatic animation is conditional on success, independent validation,
`AnimateOnRun=true`, and a visible sandbox; hidden tests do not create or pause
for animation. Code Analyzer again reported zero sandbox findings, and the
focused sandbox-diagnosis suite passed 8/8 before the visible callback smoke.

## Rogue sandbox visibility-route correction — 2026-08-27

Saved input: `Rogue Examples/az_el_sandbox_goal_20260826_192542.mat`.

The saved result was independently valid but contained only
`directVisibilityEdge` and `directWait` seeds. Its selected geometric seed was
87.6756595117 degrees while the returned motion was 112.432758778 degrees.
A fixed-arrival diagnostic replay shortened that motion to 90.3707367692
degrees, localizing the loop to motion construction rather than collision
validation. Separately, the two-candidate limit localized the absent geometric
route to candidate-budget allocation.

The retained replay used three sandbox candidates, selected seed 3 from
`visibilityGraph`, and passed independent validation. Reported metrics were:

- selected polyline: 90.1324376889 degrees;
- pre-cleanup motion: 99.8503182971 degrees;
- returned motion: 90.3025879374 degrees;
- arrival: 50.9444849555 seconds;
- collision-free and kinematic certificate: true;
- wall time: approximately 7.42 seconds.

Code Analyzer reported zero findings in all six modified MATLAB files. The
focused `testHs3Planner` plus `testObstacleAvoidanceSandboxDiagnosis` run passed
72/72. The solver emitted its existing ill-conditioned-matrix warning flood in
the deforming-obstacle test; no test failed or was incomplete.

Two structurally different maintained examples were run serially and headless:

| Example | Jerk | Planner / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / certificate | Wall (s) |
| --- | ---: | --- | ---: | ---: | ---: | --- | ---: |
| `exampleMovingCircleNoAzimuthWrap` | 1 | 1 / 1 | 12 | 12.7171175863 | 8.64603261241 | 1 / 1 | 11.8788 |
| `exampleObstacleAvoidance` | 1 | 1 / 1 | 11.152119519 | 11.4464747617 | 7.57952069664 | 1 / 1 | 5.71515 |

The first two attempts to report the moving example used one unsupported
override and then one nonexistent validation field. Planning completed in both
attempts, but only the third command above is an authoritative recorded run.
The complete maintained-example matrix and complete repository test tree were
not rerun for this focused route-quality correction.

## Sandbox Add panel — 2026-08-27

`obstacleAvoidanceSandbox.m` now exposes four working constructor controls in a
left-side `Add` panel: Polygon, Circle, Hand Drawn, and Square. The sandbox UI
contract test verifies the panel title, all four action handles, and their
labels. The full focused sandbox suite passed 8/8, Code Analyzer reported zero
findings, and `git diff --check` reported no whitespace errors. No planner
algorithm or maintained example changed for this UI addition.

## Named trajectory entry points — 2026-08-27

Added `trajectory/planTrajHs3.m` and `trajectory/planTrajRuckig.m`, each with
zero-input defaults and three-, four-, and five-input planning calls. Updated
the obstacle planner's direct-motion routing and README examples to use the new
names. The engine-qualified solve functions remain compatibility APIs.

- Code Analyzer: 0 findings across both entries, the planner, and architecture
  test.
- `testArchitectureBoundaries`: 14/14 passed.
- `testRuckigEngine`: 8/8 passed, including a solve through
  `planTrajRuckig`.
- Named HS3 fixed-time focused test: 1/1 passed through `planTrajHs3`.

No maintained example was executed because this change forwards identical
inputs to the existing engines and does not alter their algorithms.

The sandbox animation default was subsequently accelerated from frame stride
5 and pause 0.01 seconds to frame stride 20 and pause 0.001 seconds. The
focused sandbox option contract was updated to preserve these defaults.

## Completed multi-seed diagnostics — 2026-08-28

Failure class for the reported moving-circle selection: `RANKING` investigated,
no defect found. The deterministic default run generated direct, direct-wait,
and lower visibility seeds. All three passed independent continuous validation.
The direct candidate arrived at 8.64603261240521 seconds; the visibility
candidate arrived at 8.64603337466937 seconds; direct-wait arrived at
12.48486328125 seconds. Direct also beat visibility in motion length and
integrated squared jerk. The cyan lines in the screenshot were accepted graph
edges, not complete selected routes.

Added `CollectAllSeedCandidates`, stable `CandidatePaths`, per-seed elapsed and
limit diagnostics, and `ShowSeedPaths`. Completed-path labels include seed and
motion length, arrival, source, and validation. Normal planning defaults both
new modes off. Focused default-option test passed 1/1, sandbox suite passed 8/8,
and Code Analyzer reported zero findings. The initial combined Code Analyzer
command incorrectly passed a cell array to `checkcode` and reported six harness
findings; the corrected per-file run reported zero.
The complete focused HS3 planner file passed 64/64, giving 72/72 across HS3
and sandbox diagnostics with no failed or incomplete tests.

The moving-circle one-obstacle mode resolved to 45 seconds per seed and retained
3/3 paths in 10.746203 seconds. The two-obstacle mode resolved to 60 seconds per
seed. Its first gate exposed normal arrival pruning still active in collection
mode (2/3 retained), which was corrected. The next gate exposed a 0.308685-second
post-solver overrun, which led to the retained two-second finalization reserve.
The final gate retained 3/3 paths with every per-seed elapsed time below 60
seconds and an independently valid selected result.

## Optional non-stopping waypoint warm start — 2026-08-28

Source under test was `HS3-planner` at
`f0d12f8+pass-through-worktree`. MATLAB Code Analyzer reported zero findings
in the option resolver, planner, pass-through helper, and HS3 candidate solver.
Focused tests passed: `testPlannerOptions` 4/4, `testRuckigEngine` 10/10,
`testPassThroughWaypointWarmStart` 1/1, and `testHs3Planner` 64/64 in
64.569466 test seconds. One earlier HS3 suite attempt produced 63/64 only
because the test harness disabled all warnings while a test expected an
unknown-option warning; rerunning with normal warning state passed 64/64.

Serial headless A/B evidence with jerk enabled:

| Example / mode | Planner / validation | Polyline deg | Motion deg | Duration s | Collision / certificate | Wall s |
| --- | --- | ---: | ---: | ---: | --- | ---: |
| `exampleStaticUShapedObstacle`, default | 1 / 1 | 34.9425880405 | 41.5363500661 | 22.6308876389 | 1 / 1 | 13.717231 |
| `exampleStaticUShapedObstacle`, pass-through | 1 / 1 | 34.9425880405 | 41.2461615078 | 21.9798565134 | 1 / 1 | 16.139615 |
| `exampleTwoOpposingUVisibilityGraph`, default | 1 / 1 | 24.035784715 | 24.4189853364 | 21.9090835611 | 1 / 1 | 5.977472 |
| `exampleTwoOpposingUVisibilityGraph`, pass-through | 1 / 1 | 24.035784715 | 24.4189853364 | 21.9090835611 | 1 / 1 | 9.375114 |

The separately recorded four-sweep experiment reached the best validated
single-U arrival of 21.4038773205 seconds with a 41.056539875-degree motion,
but required 34.3518164 seconds for the full profile, repair, polish, and
validation pipeline. It remains benchmark evidence rather than a default.

## Nonuniform segment-placement gate — 2026-08-28

Focused verification passed `testNonuniformHs3Mesh` 6/6,
`testHs3PolynomialOperations` 10/10,
`testPassThroughWaypointWarmStart` 1/1, the Ruckig signature/mesh tests 4/4,
and `testHs3Planner` 64/64. The first combined polynomial-test command omitted
the repository root from MATLAB's path and produced package-resolution harness
errors. A fresh process with both package roots passed all 10 tests.

Single-U placement results, all independently valid:

| Mesh | Segments | Duration (s) | Gap from uniform 20 (s) |
| --- | ---: | ---: | ---: |
| Uniform | 20 | 21.4038773205 | 0 |
| Ruckig signature | 19 | 21.4852160195 | 0.0813386990 |
| Uniform | 19 | 21.6190328443 | 0.2151555238 |
| Ruckig signature | 14 | 21.8372833147 | 0.4334059942 |
| Jerk-fit coarsened | 14 | 21.8814530191 | 0.4775756986 |

Placement carries useful information, but no tested 14- or 19-segment mesh
reproduced the 21.4038773205-second reference. The adaptive placement policy
was removed from the pass-through pipeline; the general nonuniform HS3
representation and its tests remain.

Fresh headless static gates used `WaypointWarmStartMode="passThrough"` and
finite jerk limits:

| Example | Jerk | Planner / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / certificate | Wall (s) |
| --- | ---: | --- | ---: | ---: | ---: | --- | ---: |
| `exampleStaticUShapedObstacle` | 1 | 1 / 1 | 34.9425880405 | 41.2433751280 | 21.9798547914 | 1 / 1 | 17.4851590 |
| `exampleTwoOpposingUVisibilityGraph` | 1 | 1 / 1 | 24.0357847150 | 24.4189852783 | 21.9090835611 | 1 / 1 | 9.4036758 |
| `exampleDenseConcaveObstacle` | 1 | 1 / 1 | 12.7007215595 | 14.0108528198 | 8.60599186623 | 1 / 1 | 11.0079427 |

## Adaptive static hybrid verification — 2026-08-28

Source under test was `HS3-planner` at
`f0d12f8+adaptive-hybrid-worktree`. The retained decisions are based only on
route geometry, resolved options, a Ruckig timing profile, validated HS3
activity, and continuous-motion validation. The rejected 40-segment dense
probe (8.50653509124-second arrival, 95.5607259-second wall) and the rejected
Single-U clustered shortcut (21.8151192504 seconds, 29.5015091-second wall)
remain recorded here because neither meets the bounded runtime objective.

Retained serial record gates, jerk enabled:

| Example | Planner / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic | Wall (s) |
| --- | --- | ---: | ---: | ---: | --- | ---: |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 40.4882254189 | 21.2540286325 | 1 / 1 | 10.4296843 warm; 17.7360472 cold |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 23.8537208838 | 24.4031321189 | 21.7254621235 | 1 / 1 | 20.2258638 |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7007215595 | 13.7561799502 | 8.55509702317 | 1 / 1 | 9.9209119 warm; 17.9190903 cold |

These improve the declared prior hybrid records: Single U
21.4038773205 seconds / 34.3518164 seconds wall, Two U
21.7309195016 seconds / 67.482064 seconds wall, and dense concave
8.5551003021 seconds / approximately 13.5 seconds wall. The warm/cold labels
are intentional; process startup and first-use solver work are not hidden.

Deterministic random static A/B gates:

| Seed / obstacles | Pass-through duration / wall (s) | Ordinary duration / wall (s) | Outcome |
| --- | ---: | ---: | --- |
| 101 / 2 | 15.8643105795 / 11.2243794 | 16.5995347238 / 5.3768225 | 0.735224-second gain for 5.847557 seconds extra work |
| 303 / 3 | 15.7631244134 / 10.5809183 | 15.7631244134 / 6.9965109 | measured-potential gate skipped repair/polish |
| 404 / 3 | 16.0903836179 / 3.2277611 | 16.0903836179 / 5.0975473 | measured-potential gate skipped repair/polish |

All six random trajectories above passed independent collision and kinematic
validation. Seed 202 also matched ordinary arrival, path, and 20-segment mesh;
its earlier serial walls were 7.719 seconds pass-through and 9.588 seconds
ordinary and are treated only as order-sensitive generalization evidence.

The broader maintained static-family pass-through sweep produced:

| Example | Success / example validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic | Wall (s) |
| --- | --- | ---: | ---: | ---: | --- | ---: |
| `exampleAlternatingSlalom` | 1 / 1 | 16 | 17.2251036845 | 12.1510445858 | 1 / 1 | 13.0849257 |
| `exampleNoPath` | 0 / 1 expected failure | NaN | NaN | NaN | NaN | 5.8366406 |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4466826495 | 7.57952066338 | 1 / 1 | 10.7509637 |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 | 4.946881 |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.2394635087 | 24.6373466428 | 6.3679977502 | 1 / 1 | 55.740453 |

Pass-through matched the current ordinary trajectory on alternating slalom,
the basic obstacle, obstacle free, and the extreme static outline. It does not
beat the older `325-full-suite` alternating-slalom and extreme-outline records,
so no all-history/all-branch record claim is made.

Verification commands and outcomes:

- Code Analyzer: zero findings in the modified planner, pass-through solver,
  search classifier, activity mesh, HS3 candidate solver, and HS3 engine.
- `testHs3Planner`: 64/64 passed.
- Pass-through classification, warm-start, activity-mesh, nonuniform-mesh,
  and Ruckig signature/refinement suites: 18/18 passed.
- `testPlannerOptions` and `testRuckigEngine`: 14/14 passed.
- Total focused result: 96/96 passed.
- The complete HS3 suite's deforming-obstacle case emitted extensive
  near-singular `fmincon` warnings but completed successfully. This remains a
  visible conditioning weakness rather than a suppressed or passing claim.

## Certified continuation and random HS3 milestone — 2026-08-28

The broad near-direct shortcut was rejected after it returned an intermediate
25.92-second Two-U result instead of reaching its declared final duration.
The retained condition is narrower and input-driven: one active endpoint axis,
monotone progress on that axis, route length within 2% of direct displacement,
and at least two lateral sign reversals. An incomplete continuation ladder is
now rejected rather than silently promoting its last feasible intermediate.

Fresh serial static gates, jerk enabled:

| Example | Success / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic | Planner wall (s) |
| --- | --- | ---: | ---: | ---: | --- | ---: |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0193197983 | 16.0904389848 | 10.7625 | 1 / 1 | 4.7953407 |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 40.4882254189 | 21.2540286325 | 1 / 1 | 11.7809874; 19.8996114 fresh post-cleanup |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 23.8537208838 | 24.4031321189 | 21.7254621235 | 1 / 1 | 17.0938997 |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7007215595 | 13.7561799502 | 8.55509702317 | 1 / 1 | 12.9727886 |

The alternating result is an HS3 fixed-time solution, not a Ruckig-only
fallback. It improves the current ordinary/hybrid 12.1510445858-second arrival,
17.2251036845-degree motion, and 13.0849257-second wall record. The other three
protected static results remain numerically unchanged. Single-U fresh-process
runtime is still variable and is not claimed below ten seconds.

Three predeclared deterministic random static A/B cases prove the first
milestone reaches HS3 without unbounded work:

| Seed / obstacles | Hybrid arrival / wall (s) | Ordinary arrival / wall (s) | Hybrid overhead (s) | Selected HS3 |
| --- | ---: | ---: | ---: | ---: |
| 101 / 2 | 15.8643105795 / 11.4773750 | 16.5995347238 / 5.1656831 | +6.3116919 | attempted / feasible / validated |
| 303 / 3 | 15.7631244134 / 12.7055879 | 15.7631244134 / 8.5891386 | +4.1164493 | attempted / feasible / validated |
| 404 / 3 | 16.0903836179 / 3.9028172 | 16.0903836179 / 6.2603182 | -2.3575010 | attempted / feasible / validated |

All six random trajectories passed independent collision and kinematic
validation. Hybrid overhead stayed below the declared 10--15 second ceiling in
all three cases; seed 101 improved arrival by 0.7352241443 seconds, while 303
and 404 matched the ordinary HS3 solution exactly.

Post-cleanup verification reported zero Code Analyzer findings in the six
hybrid planner files, 16/16 hybrid/nonuniform focused tests, 14/14 option and
Ruckig tests, and 64/64 HS3 planner tests. The full HS3 run took 169.75339
seconds and again exposed the deforming-obstacle near-singular warning flood.
Against `f0d12f8`, production growth is 2,123 physical added lines and 131
removed lines (1,992 net); excluding blank and comment-only additions gives
1,795 executable physical lines. This is below two thousand net lines but is
still a material implementation-size cost, so no small-change claim is made.

## Bounded moving/deforming runtime closeout — 2026-08-28

The final retained source contains none of the runtime experiments below:

- exact obstacle-query caching had only a 9.0167% theoretical hit fraction;
- failed stage-two recovery cost 0.307568 seconds and remained infeasible;
- initial 20-segment topology escalation took 192.026619 seconds and reached
  40 segments, despite returning a valid 7.941370833-second motion;
- a 50-iteration coarse pass returned a fully valid 7.962868741-second motion
  in 56.098832 planner seconds, but its 43.132171247-degree smooth length was
  0.059904737 degree longer than the clean 43.072266510-degree reference.

The last candidate was rejected because its 8.3% apparent runtime improvement
was below a robust retention threshold under the observed process variance and
regressed path length by 0.139%. The production hunk was removed before final
verification.

Post-removal checks:

- Code Analyzer reported zero findings for `plan.m`.
- The seven focused planner/hybrid/mesh/option test files passed 93/93 in
  137.442642 seconds. The deforming test again emitted the known near-singular
  `fmincon` warning flood; no warning was suppressed.
- `exampleStaticUShapedObstacle`, jerk enabled: success/validation 1/1,
  34.942588041-degree polyline, 40.520436104-degree motion,
  21.254028732-second arrival, collision/kinematic 1/1, 30.260707-second wall.
- `exampleTwoOpposingUVisibilityGraph`, jerk enabled: success/validation 1/1,
  23.853720884-degree polyline, 24.403132126-degree motion,
  21.725462124-second arrival, collision/kinematic 1/1, 30.073726-second wall.
- Visible `exampleObstacleFree`, jerk enabled: success/validation 1/1,
  4.472135955-degree polyline and motion, 4.531128874-second arrival,
  collision/kinematic 1/1, four figures, frame stride 20.
- `exampleNoPath`, jerk enabled: expected success 0 / validation 1,
  unavailable path and arrival metrics, `noValidatedSeed`, and two search-
  diagnostic figures.

The final production line audit after consolidation is +2,132/-133 physical
lines versus `f0d12f8`, or +1,999 net, with 1,807 nonblank/non-comment
additions. Duplicate validation in the one-caller activity-mesh policy and two
derivable classification/activity diagnostic fields were removed. The
activity decision, nonuniform mesh, and HS3 candidate behavior remain intact.

Post-consolidation verification:

- 3/3 activity-mesh tests, then 8/8 activity/classification/pass-through tests;
- all 17 maintained examples run serially: 16 independently validated
  successes and one independently validated expected no-path result;
- exact retained moving/deforming result: 40-degree selected polyline,
  43.0722665096-degree motion, 7.96286792066-second arrival, collision and
  kinematic certificates passed, 55.7054096-second planner time;
- visible obstacle-free smoke: 4.472135955-degree path and motion,
  4.53112887415-second arrival, four figures, frame stride 20;
- no-path smoke: `noValidatedSeed`, three expanded states, two diagnostic
  figures;
- final focused suite: 93/93 passed in 115.924289 seconds. This duplicate run
  suppressed only the already-recorded near-singular MATLAB console flood;
  the prior unsuppressed 93/93 run passed in 137.442642 seconds.

Post-consolidation random A/B evidence, all selected hybrid results explicitly
HS3-attempted, optimizer-feasible, independently validated, collision-free,
and kinematically certified:

| Seed | Hybrid arrival / wall (s) | Ordinary arrival / wall (s) | Hybrid wall delta (s) |
| ---: | ---: | ---: | ---: |
| 101 | 15.8643105795 / 17.7135074 | 16.5995347238 / 8.4561331 | +9.2573743 |
| 303 | 15.7631244134 / 15.9672482 | 15.7631244134 / 10.8656958 | +5.1015524 |
| 404 | 16.0903840021 / 5.2379460 | 16.0903840021 / 8.3214712 | -3.0835252 |

Thus the current worktree proves the first milestone on three deterministic
random static fields without exceeding the 10--15-second hybrid overhead cap.
Seed 101 also proves the second milestone with a 0.7352241443-second arrival
gain for 9.2573743 seconds of additional wall work. This is bounded evidence,
not a claim of global path or time optimality.

The final post-consolidation Single-U A/B directly proves simultaneous metric
improvement on the same physical request:

| Mode | Arrival (s) | Motion length (deg) | Wall (s) | HS3/validation |
| --- | ---: | ---: | ---: | --- |
| Ruckig/HS3 hybrid | 21.2540287320 | 40.5204361036 | 28.7366170 | attempted, feasible, passed |
| Ordinary HS3 | 22.6308871020 | 41.5367249083 | 40.8732228 | attempted, feasible, passed |

The hybrid delta is -1.3768583700 seconds arrival, -1.0162888047 degrees
motion length, and -12.1366058 seconds wall. Both rows used the same maintained
example, finite jerk limits, and independent validation. The ordinary solve's
near-singular warnings remain visible in the execution record.

## Four-File Hybrid Cleanup Verification

The cleanup scope was limited to:

- `+obstacleAvoidance/+planner/classifyPassThroughSearch.m`;
- `+obstacleAvoidance/+planner/createHybridActivityMesh.m`;
- `+obstacleAvoidance/+planner/solvePassThroughSeedCandidate.m`;
- `trajectory/+hs3Engine/+polynomial/resolveSegmentMesh.m`.

Measured size changed from 815 to 758 physical lines and from 629 to 574
nonblank, non-comment lines. No public option, result field, diagnostic, or
validation rule was removed.

The complete production diff versus `main` is +2,075/-133 physical lines, or
+1,942 net. No replacement production file was introduced by the cleanup.

Verification performed after the final refactor:

- MATLAB `checkcode(..., "-id")`: zero messages on all four files;
- focused activity, classification, pass-through, and nonuniform-mesh tests:
  14/14 passed;
- full `tests` tree with subfolders: 164/164 passed, zero failed or incomplete,
  84.9595026 aggregate test seconds;
- all 17 maintained examples in separate serial MATLAB processes with plots
  and animation disabled: 16 validated successes and the expected validated
  `noValidatedSeed` result;
- exact Single-U result: 34.9425880405-degree polyline,
  40.5204361036-degree motion, 21.2540287320-second duration;
- exact Two-U result: 23.8537208838-degree polyline,
  24.4031321261-degree motion, 21.7254621235-second duration;
- exact moving/deforming result: 40-degree polyline,
  43.0722665096-degree motion, 7.96286792066-second duration;
- default visible obstacle-free run: validation passed and four figures;
- expected no-path diagnostic run: validation passed, three expanded states,
  and two figures.

Every successful example passed collision, workspace, velocity, acceleration,
and jerk checks. The cleanup preserves observed behavior; it does not prove
global optimality or a wall-time improvement.

## Restored Ruckig Route Integration Verification — 2026-08-29

The exact historical state-to-state Ruckig engine and `planTrajRuckig` facade
were restored under `trajectory/+ruckigEngine`. Obstacle routing remains in the
planner. The new general `TrajectoryMethod="ruckigWaypoint"` option composes
exact Ruckig state-to-state segments along deterministic route seeds, exposes
`ruckigWaypointComposition` provenance, and does not silently fall back to
BMTP. This local engine is not represented as Ruckig Pro's nonconvex waypoint
solver or as globally waypoint-time-optimal.

Verification performed after the adapter and Straight Target wiring:

- MATLAB `checkcode(..., "-id")`: zero messages on the modified production
  sources and maintained Straight Target example;
- exact engine suite: 10/10 passed;
- engine, adapter, options, and architecture focus: 24/24 passed;
- full `tests` tree: 78/78 passed, zero failed or incomplete, in
  26.8828596 seconds;
- structurally different static-box detour: earliest and fixed arrival both
  passed independent collision and kinematic validation;
- Straight Target with jerk enabled: planner success and independent validation
  passed; 20.7720160748-degree selected polyline and motion length;
  20.8695652173913-second exact fixed duration; collision, continuous
  kinematics, and continuous collision resolution passed; 5.8749177-second
  planner time and 10.1040635-second full example wall;
- production-size audit rule: 11,167 nonblank, noncomment MATLAB lines across
  67 production files, above the 4,999-line ceiling by 6,168 lines.

The Ruckig integration therefore meets the explicit Straight Target engine and
physical-clock gate, but does not meet the historical 13.678271908-degree path,
2.0964864-second wall, or repository-size records.
## Explicit timed-topology policy and moving BMTP projection — 2026-08-31

Baseline on `f383ae4` used a static U-shaped detour plus a distant moving
rectangle. All three topology seeds first returned
`unsupportedTimedTopology`, all three silently invoked Ruckig, and seed 3
returned a 31.4265 s, 34.9426 deg stop-at-waypoint success.

After the change:

- default `UnsupportedTimedTopologyPolicy="fail"` preserved
  `unsupportedTimedMultiWaypointRoute` and made zero Ruckig attempts when
  `MaximumNlpIterations=1` forced the unsupported boundary;
- explicit `"ruckigStopAtWaypoints"` reproduced the 31.4265 s recovery and
  reported the original reason, method, interior times, zero interior
  velocity/acceleration states, and forced-rest policy;
- the normal distant-mover request succeeded through static BMTP at
  20.8454 s and 39.5987 deg only after validation against the complete moving
  scene;
- a translating-rectangle detour succeeded through the conservative swept
  BMTP projection at the fixed 20 s horizon, with 10.3005480783 deg selected
  polyline, 10.7117850149 deg motion, 0.0657896049 deg minimum clearance, and
  complete collision and kinematic validation.

Verification performed after the final implementation:

- Code Analyzer reported zero messages on all modified MATLAB sources;
- focused option, Ruckig, sandbox, projection, and fallback-policy tests
  passed;
- the complete test tree passed 98/98 in 69.6771 s;
- all 17 maintained examples ran serially and headlessly: 16 planner and
  independent-validation successes plus the expected validated
  `exampleNoPath` failure;
- a visible `exampleObstacleFree` run passed with jerk enabled, 4.472135955
  deg polyline and motion length, and 4.53112887415 s duration;
- the unchanged static-U and moving-barrier sentinels returned
  20.712447786 s and 10.0903015137 s respectively.

The moving extension is deliberately described as a conservative static
projection, not a time-dependent separating-plane method. Time-cell BMTP and
wait-plus-detour support remain unimplemented and visible in the branch
assessment.

## HTML sandbox diagnosis-bundle export — 2026-08-31

The live HTML sandbox now enables **Save diagnosis bundle** only after a
matching live MATLAB result. `POST /plan` retains the unprojected public result
and exact canonical inputs in a server-owned
`obstacleAvoidanceSandboxDiagnosis-v2` MAT cache; `POST /bundle` returns that
cache only for the matching request identifier. Editing the scene, resetting,
or loading a result file clears browser eligibility, and stopping the server
deletes its cache. Offline JSON handoff remains bounded and does not claim it
can reconstruct omitted solver diagnostics.

Code Analyzer reported zero messages across the three changed MATLAB owners
and the focused test. `testOfflineSandboxDiagnosisBundle` passed 2/2 in
0.46553 seconds. A live loopback smoke produced a successful independently
validated obstacle-free plan, downloaded a 452,512-byte MAT response with the
MATLAB content type, and rejected a stale request identifier with HTTP 409.
`git diff --check` passed. A separate MATLAB reload inspection could not start
because the environment reported `System Error: File system inconsistency`
while several pre-existing MATLAB processes were active; the generated MAT
file had already been written and transported successfully. The in-app browser
also prohibited navigation to the local `file://` page, so visual layout was
not exercised through that browser surface.

## Balanced Wear Objective And Newheart Exact-Clock Repair — 2026-08-31

The `newheart.mat` diagnosis bundle already requested `balancedArrival` at
1 deg/s, so its visible S-bend was not stale UI state or an earliest-arrival
replay. The selected alternating progress polynomial cost 201.070948503 deg at
the certified 100.970425693 s physical time floor. A static visibility seed was
199.1997663 deg, proving the selected motion retained avoidable travel.

The retained implementation generalizes progress-polynomial composition to a
validated input basis and derives normalized one-sided beta bases from the
direct collision progress. It keeps the physical direct clock and enumerates
only amplitudes inside continuous Bernstein workspace, velocity, acceleration,
and jerk bounds. Sampled occupancy rejects proposals but never accepts them;
the public continuous validator remains authoritative. Actual path length
ranks all fixed-clock families.

Measured results after the final change:

- `newheart`, balanced rate 1: success/validation 1/1,
  199.268051966 deg selected polyline and motion, 100.970425693 s,
  collision/kinematic 1/1, `oneSidedBeta_1_4`, 65.7788 planner seconds;
- prior `newheart` alternating motion: 201.070948503 deg at the same clock;
- `sinetraj`: success/validation 1/1, 146.928879089 deg,
  70.344250998 s, collision/kinematic 1/1;
- `shrimp`, balanced rate 1: success/validation 1/1, 175.703912280 deg,
  81.455142283 s, collision/kinematic 1/1;
- `non-ideal`, balanced rate 1: success/validation 1/1,
  228.491135293 deg, 144 s, collision/kinematic 1/1;
- `hiddenruckigfallback`: success/validation 1/1,
  233.911502487 deg, 104.261456926 s, collision/kinematic 1/1, BMTP selected.

The structurally different near-start rectangle regression selected a
20.493950992-deg one-sided motion instead of its 20.585690610-deg validated
alternating motion, at the same 12.5 s physical clock. Its lateral offset never
crossed the direct chord.

Verification performed after the final source edits:

- Code Analyzer: zero findings across all 22 changed MATLAB files;
- focused BMTP engine: 12/12 passed;
- focused near-start rectangular route-economy regression passed;
- complete test tree: 110/110 passed, zero failed or incomplete,
  126.782359 s wall and 122.514226 s aggregate test duration;
- all 17 maintained examples in separate serial headless MATLAB processes:
  16 validated successes and expected `exampleNoPath` failure;
- every successful example passed collision, workspace, velocity,
  acceleration, and jerk checks;
- visible `exampleObstacleFree`: success/validation 1/1, two figures,
  4.472135955 deg and 4.531128874 s;
- `git diff --check` was run after record updates.

After the 17-example pass, repeated fresh MATLAB launches for the hidden
failure-figure smoke returned the environment-level fatal startup message
`System Error: File system inconsistency`. The headless no-path example itself
had just returned `noValidatedSeed` without an example-validation warning. An
earlier check on the same worktree had already created its hidden diagnostic
figure; no failure plotting file changed in this final exact-clock repair.

## HTML diagnosis replay and velocity-vector motion - 2026-08-31

This change affects the HTML authoring and loopback transport, not planner
selection or trajectory generation. Verification after the final source edits:

- Code Analyzer returned zero findings for `replayDiagnosisBundle.m`,
  `serveSandbox.m`, and the focused diagnosis-bundle tests.
- The focused diagnosis bundle suite passed 3/3. Its reproduction case included
  a moving polygon and verified exact preservation of keyframe time and original
  terminal geometry.
- The complete test tree passed 111/111 with zero failed or incomplete tests in
  123.685910 seconds.
- JavaScript syntax passed with the bundled Node.js runtime. The production
  `motionOffset_deg` function was extracted and exercised for constant,
  zero-start, trapezoidal, and out-and-back profiles. A `[3, 4]` drawn vector
  produced a displayed magnitude of exactly `5.000 deg/s`.
- A live MATLAB server on `127.0.0.1:52739` accepted a raw diagnosis MAT file at
  `/run-bundle` and returned `offlineSandboxResult/v1`, a new
  `bundle-replay-*` request identifier, `goalReached`, planner success, and
  independent validation success. Arrival was 1.86969384567 seconds; planner
  time was 0.927697 seconds and server time before transport was 1.498398
  seconds. The server then stopped cleanly and its explicit smoke artifacts
  were removed.
- The in-app browser rejected the local `file://` page under its URL policy, so
  no visual browser pass is claimed. Two fresh serial-example attempts were
  blocked before the first example by MATLAB's intermittent environment-level
  `System Error: File system inconsistency`. The planner source was unchanged;
  the prior successful 17-example matrix above remains the applicable planner
  baseline.

## Dead planner verbosity option removal - 2026-09-01

The public planner `Verbose` field had no production reader. It was removed
from defaults, examples, sandbox exports, offline requests, contracts, and
current interface documentation. Obstacle-construction verbosity remains live,
and the sandbox checkbox still controls caller-owned `evalc` capture. Direct
legacy planner input produces only
`planTrajectory:DeprecatedVerbose`, is stripped before ordinary resolution,
and is absent from returned `Options`.

An obstacle-free default run and a legacy-`Verbose=true` run were recursively
identical after removing measured elapsed-time fields. Both succeeded with a
4.41995024845-second duration and 4.472135955-degree motion. Final verification
after all source edits produced zero Code Analyzer findings and passed 118/118
tests in 81.436094 seconds wall time (78.6810012 aggregate test seconds).

Every maintained example ran headlessly in a separate serial MATLAB process
with jerk enabled:

| Example | Success / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic | Wall (s) | Reason |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0333716767 | 16.0333716767 | 10.5 | 1 / 1 | 1.3070865 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7952270203 | 12.7952270203 | 8.5 | 1 / 1 | 4.3292660 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 | 1.8896941 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 | 0.8572632 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.30889024019 | 7.30889024019 | 6.11111111111 | 1 / 1 | 0.8962971 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.0903015137 | 1 / 1 | 1.9732295 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12.1077259407 | 12.1077259407 | 8.5 | 1 / 1 | 6.7140330 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40.2805670061 | 40.2805670061 | 7.91666666667 | 1 / 1 | 26.6008908 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN | 2.5900853 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4118613877 | 7.57454176632 | 1 / 1 | 4.7387231 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 | 0.7821855 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10 | 11.5843333838 | 1 / 1 | 1.8459921 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 40.255028504 | 20.712447786 | 1 / 1 | 3.6708232 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 27.950433436 | 13.5713266002 | 20.8695652174 | 1 / 1 | 25.0493233 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.1357890335 | 20.6100682085 | 24 | 1 / 1 | 16.0339540 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.0767724922 | 24.0767724922 | 21.6333333333 | 1 / 1 | 4.2266279 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.070643085 | 23.3457566443 | 5.81065318159 | 1 / 1 | 68.4555210 | `goalReached` |

A separate visible `exampleObstacleFree` run created two figures with no
warning. A hidden `exampleNoPath` run retained `noValidatedSeed`, one attempted
seed, and one diagnostic figure. `git diff --check` is the final repository
gate. The change adds eight net production MATLAB lines because the required
one-release migration shim and tests outweigh deleting the dormant field; the
five-milestone branch total remains a 155-line production reduction.

## Automatic plane-reuse ownership - 2026-09-01

BMTP plane reuse remains active, but `EnablePlaneReuse` and
`PlaneReuseImprovementTolerance_s` are no longer planner choices. Automatic
reuse applies only when the retained-best duration improvement is within
`ArrivalTimeTolerance_s` and the tagged path--region pair set is unchanged.
Legacy fields warn once with
`planTrajectory:DeprecatedPlaneReuseOptions`, are ignored, and are absent from
returned options.

Saved pre-edit results and the candidate matched recursively outside elapsed
time and the two removed fields:

- tight-clearance Target Exits: 24 s, 21.9416287311844 deg, selected seed 2,
  reuse count 1, 60 plane SOCPs, 8 trajectory SOCPs;
- timed alternating occlusion: 20.8695652173913 s, 13.571326600194 deg,
  per-seed reuse counts `[0 1 1 0 1]`;
- static-U non-activation sentinel: 20.7124477860115 s,
  40.2550285040009 deg and zero selected-solve reuse.

The first comparison supplied legacy `EnablePlaneReuse=false` and a custom
reuse tolerance, proving the retired fields cannot disable or retune automatic
behavior. The expanded focused suite passed 48/48. Final Code Analyzer output
was clean, and the complete repository suite passed 120/120 in 81.7413358
seconds wall time (78.9273796 aggregate test seconds).

All 17 maintained examples ran in separate serial MATLAB processes with jerk
enabled. Sixteen succeeded and independently validated; `exampleNoPath`
returned the expected validated `noValidatedSeed`. Every success passed
collision and kinematic checks. The exact rows and wall times are recorded in
`benchmark.csv`. A visible obstacle-free run created two figures without a
warning, and the hidden no-path run created one diagnostic figure. The manual
exporter also reran `exampleObstacleAvoidance` successfully in 6.3053904
seconds and regenerated `WalkPlaneReuse` as `automatic`. Neither `pdflatex`
nor `pdftotext` is installed on this host, so the updated TeX sources and data
were not compiled or text-extracted from the tracked PDFs in this milestone.

The change adds three net production MATLAB lines because the compatibility
shim outweighs removing two option fields. The six-milestone branch total is
152 production lines smaller than `5c0a6c9`; this milestone claims a smaller
public interface and clearer invariant ownership, not fewer physical lines or
a speedup.

## Internal trajectory-solver cap ownership - 2026-09-01

`MaximumNlpIterations` actively controlled the BMTP trajectory `coneprog`
iteration cap despite its obsolete NLP name. The planner interface now omits
that implementation control while the engine retains the former default cap of
300. Direct legacy input warns once with
`planTrajectory:DeprecatedMaximumNlpIterations`, is ignored, and is absent from
returned options. Current sandbox, offline sandbox, examples, tests, and manual
data no longer set or display it.

The old unsupported-topology fixture used a cap of one to force its boundary.
At 300 the same request succeeded and required 65.7410579 seconds, proving the
fixture tested solver starvation rather than an input-driven topology policy.
The replacement uses a physically infeasible eight-second fixed-arrival
deadline. Default policy refuses fallback and explicit policy attempts it;
both revised tests pass in 2.9632806 seconds. The dedicated Ruckig test still
checks the two-segment limit.

Focused evidence:

- option migration: 10/10 in 1.9703629 seconds;
- BMTP engine: 12/12 in 3.7918162 seconds;
- sandbox diagnosis: 11/11 in 10.3521119 seconds;
- timed policy: 2/2 in 2.9632806 seconds;
- timed BMTP: 2/2 in 15.0755105 seconds versus 9.9854775 baseline;
- sandbox route economy: 3/3 in 14.6741929 seconds versus 14.7555260 baseline;
- combined focused gate: 40/40 in 44.5871511 seconds;
- Code Analyzer: zero findings across every changed MATLAB file;
- full suite: 121/121 in 88.5048981 seconds wall time and
  81.6421096 seconds aggregate test time.

Every maintained example ran headlessly in a separate fresh MATLAB process
with jerk enabled:

| Example | Success / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic / certificate | Wall (s) | Reason |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0333716767 | 16.0333716767 | 10.5 | 1 / 1 / NaN | 1.5159463 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7952270203 | 12.7952270203 | 8.5 | 1 / 1 / NaN | 4.3829100 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 / NaN | 1.9079066 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 / NaN | 0.9036065 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.30889024019 | 7.30889024019 | 6.11111111111 | 1 / 1 / NaN | 0.9448795 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.0903015137 | 1 / 1 / NaN | 1.9750579 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12.1077259407 | 12.1077259407 | 8.5 | 1 / 1 / NaN | 6.7703150 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40.2805670061 | 40.2805670061 | 7.91666666667 | 1 / 1 / NaN | 26.5642490 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN / NaN | 2.6439313 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4118613877 | 7.57454176632 | 1 / 1 / 1 | 4.7093528 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 / NaN | 0.8396021 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10 | 11.5843333838 | 1 / 1 / NaN | 1.9333544 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 40.255028504 | 20.712447786 | 1 / 1 / 1 | 3.6443382 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 27.950433436 | 13.5713266002 | 20.8695652174 | 1 / 1 / 1 | 24.3221035 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.1357890335 | 20.6100682085 | 24 | 1 / 1 / 1 | 15.9573421 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.0767724922 | 24.0767724922 | 21.6333333333 | 1 / 1 / NaN | 4.1915333 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.070643085 | 23.3457566443 | 5.81065318159 | 1 / 1 / 1 | 67.9672843 | `goalReached` |

The visible obstacle-free gate created two figures without warnings. The
hidden no-path gate created one figure whose text included `noValidatedSeed`
and reported one attempted seed. The manual-data exporter completed in
5.7923965 seconds and removed the retired option macro. The milestone reduces
non-test MATLAB by six lines under the established accounting, taking the
seven-milestone branch total to 158 lines removed from `5c0a6c9`.

## Internal BMTP segmentation ownership - 2026-09-01

`CollocationSegmentCount` previously doubled into a cap on static warm-route
and timed-cell spans. It is now a compatibility-only input: direct and example
legacy use warns once with
`planTrajectory:DeprecatedCollocationSegmentCount`, is ignored, and is absent
from returned options. Both internal sites retain the former default effective
cap of 20 spans. Public defaults contain 14 fields.

Recursive comparison at `1e-9`, after removing only runtime evidence and the
retired field, passed for static U and the moving-circle/static-U timed-cell
fixture. The timed case retained `bmtpTimedCell`, seven optimizer spans, seven
timed segments, 35 seconds, 36.6949453597 degrees, and exact coverage, plane,
trajectory, validation, and certificate records. The 30-edge engine sentinel
still reports `WarmRouteResampled=true` and exactly 20 optimizer spans. A
legacy value of 2 warned once, disappeared, and reproduced automatic static-U
output exactly.

Focused evidence:

- focused option/engine/stage/example/timed/sandbox gate: 56/56 in
  47.0155112 seconds;
- sandbox diagnosis: 11/11 in 10.1182416 seconds versus 10.3521119 baseline;
- sandbox route economy: 3/3 in 14.8954734 seconds versus 14.6741929 baseline;
- stage timing: 4/4 in 2.4564911 seconds;
- Code Analyzer: zero findings in every changed MATLAB file;
- manual-data exporter: validated result in 5.8262656 seconds;
- complete suite: 123/123 in 88.5229176 seconds wall time and
  81.7095918 seconds aggregate test time.

Every maintained example ran headlessly in a separate fresh MATLAB process
with jerk enabled:

| Example | Success / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic / certificate | Wall (s) | Reason |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0333716767 | 16.0333716767 | 10.5 | 1 / 1 / NaN | 1.3499269 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7952270203 | 12.7952270203 | 8.5 | 1 / 1 / NaN | 4.4132808 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 / NaN | 1.9587101 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 / NaN | 0.9464368 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.30889024019 | 7.30889024019 | 6.11111111111 | 1 / 1 / NaN | 0.9466535 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.0903015137 | 1 / 1 / NaN | 2.0030567 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12.1077259407 | 12.1077259407 | 8.5 | 1 / 1 / NaN | 6.8236954 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40.2805670061 | 40.2805670061 | 7.91666666667 | 1 / 1 / NaN | 26.5096838 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN / NaN | 2.6737009 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4118613877 | 7.57454176632 | 1 / 1 / 1 | 4.7690469 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 / NaN | 0.8466857 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10 | 11.5843333838 | 1 / 1 / NaN | 1.8883797 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 40.255028504 | 20.712447786 | 1 / 1 / 1 | 3.6598913 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 27.950433436 | 13.5713266002 | 20.8695652174 | 1 / 1 / 1 | 24.5195017 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.1357890335 | 20.6100682085 | 24 | 1 / 1 / 1 | 15.8739982 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.0767724922 | 24.0767724922 | 21.6333333333 | 1 / 1 / NaN | 4.2825783 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.070643085 | 23.3457566443 | 5.81065318159 | 1 / 1 / 1 | 67.7607284 | `goalReached` |

The visible obstacle-free gate created two figures without warnings. Hidden
`exampleNoPath` created one diagnostic figure containing `noValidatedSeed` and
one attempted seed. The milestone adds two net non-test MATLAB lines because
the one-release shim outweighs consumer simplification. The eight-milestone
branch total is therefore 156 lines smaller than `5c0a6c9`.

## External BMTP restart retirement - 2026-09-01

At tracked-clean baseline `5a8eee0`, no maintained planner, example, sandbox,
or benchmark consumed the third BMTP output or eighth restart input. Only two
direct wrapper tests exercised that surface. The retained candidate removes
restart validation, alternate initialization, collision-free supplied-net
retention, restart allocation, and restart export from `bmtpEngine.solve`.
`planTrajBmtp` keeps one-release callable compatibility: restart use warns once,
is ignored, and returns an empty record.

Fresh-process saved baselines and candidates compared recursively at `1e-9`
after removing only fields containing `Elapsed` and
`FirstValidatedMotionTime_s`:

- static degree-16 planner record: passed, maximum difference 0;
- true timed-cell degree-7 planner record: passed, maximum difference 0;
- direct cold candidate and diagnostics: passed, maximum difference 0;
- deprecated-call physical result versus old warm result: passed, maximum
  difference 0.

The direct fixed-arrival baseline used one `unitDirect` segment, a distant
rectangle, ten seconds, and 2/1/2 deg/s derivative limits. Its returned restart
had three spans and 3.33333333333-second segment time. Cold and warm results
both used two iterations and two trajectory SOCPs, with identical sampled
position, velocity, acceleration, ten-second duration, and
4.0000000019748008-degree length. The restart reduced this direct-only engine
time from 1.0651848 to 0.2434276 seconds; the candidate cold/legacy times were
1.0474138 and 0.2373834 seconds. Static planning was 3.0265116 baseline versus
3.0494709 candidate; timed planning was 13.7478176 versus 13.7950498 seconds.

Verification evidence:

- `testBmtpEngine`: 12/12 in 6.9514129 seconds;
- architecture, timed BMTP, planner contract/failure, and sandbox diagnostics:
  37/37 in 61.4287410 seconds wall and 56.7385531 aggregate test seconds;
- Code Analyzer: zero findings in the engine, facade, and changed tests;
- complete test tree: 123/123 in 88.4909933 seconds wall and
  81.5665061 aggregate test seconds;
- visible `exampleObstacleFree`: success, validation, two figures, no warning;
- hidden `exampleNoPath`: expected `noValidatedSeed`, one attempted seed, one
  figure, and the reason present in figure text.

The temporary evidence reporter initially exited after saving valid static
results because it referenced candidate-only `MotionDuration_s` and
`MotionLength_deg` fields. It was corrected to use the public result schema and
the same baseline was rerun cleanly. The first failure-figure inspection also
used obsolete `SeedAttemptCount`; rerunning with current
`AttemptedSeedCount` passed. Neither failure involved production planner code.

Every maintained example ran headlessly in a separate fresh MATLAB process
with jerk enabled:

| Example | Success / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic / certificate | Wall (s) | Reason |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0333716767 | 16.0333716767 | 10.5 | 1 / 1 / NaN | 1.3897094 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7952270203 | 12.7952270203 | 8.5 | 1 / 1 / NaN | 4.4522526 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 / NaN | 1.9946238 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 / NaN | 0.9867267 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.30889024019 | 7.30889024019 | 6.11111111111 | 1 / 1 / NaN | 1.0137154 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.0903015137 | 1 / 1 / NaN | 2.0419501 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12.1077259407 | 12.1077259407 | 8.5 | 1 / 1 / NaN | 6.8532760 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40.2805670061 | 40.2805670061 | 7.91666666667 | 1 / 1 / NaN | 26.5408143 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN / NaN | 2.7672403 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4118613877 | 7.57454176632 | 1 / 1 / 1 | 4.9816229 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 / NaN | 0.8768426 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10 | 11.5843333838 | 1 / 1 / NaN | 1.9987615 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 40.255028504 | 20.712447786 | 1 / 1 / 1 | 3.7470639 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 27.950433436 | 13.5713266002 | 20.8695652174 | 1 / 1 / 1 | 24.3210595 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.1357890335 | 20.6100682085 | 24 | 1 / 1 / 1 | 15.8694792 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.0767724922 | 24.0767724922 | 21.6333333333 | 1 / 1 / NaN | 4.2695347 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.070643085 | 23.3457566443 | 5.81065318159 | 1 / 1 / 1 | 68.2132158 | `goalReached` |

The engine removes 44 net production lines and the compatibility facade adds
four, for a net 40-line non-test MATLAB reduction. The nine-milestone branch is
196 lines smaller than `5c0a6c9`. `pdflatex` and `pdftotext` remain unavailable,
so the updated appendix source was not compiled or text-extracted on this host.

## Detailed plane-reuse trace retirement - 2026-09-01

At tracked-clean baseline `75dff7a`, only one focused contract read
`PlaneReuseIterationHistory`, `PlaneReuseControlDifference_deg`, or
`PlaneReuseDurationDifference_s`. No production decision, example, sandbox,
plotter, exporter, or manual consumed them. The candidate removes those arrays
and the pending control/duration snapshots that populated them while retaining
`PlaneReuseApplied`, `PlaneReuseCount`, reuse continuation, convergence,
collision histories, retained-best evidence, and certificates.

Two fresh-process reuse-triggering baselines were frozen before editing. Static
U was rejected because its selected constructor did not expose BMTP reuse, and
generic Obstacle Avoidance was rejected because its BMTP solve reported zero
reuse. The retained cases were:

- Target Exits with `CollisionClearanceTolerance_deg=1e-4` and two seeds:
  24 seconds, 21.7425467317-degree polyline, 21.9416287312-degree motion,
  reuse count 1, iteration 8, baseline/candidate walls 12.3447550 and
  12.3359060 seconds;
- Extreme US Outline: 5.81065318159 seconds, 22.070643085-degree polyline,
  23.3457566443-degree motion, reuse count 1, iteration 8,
  baseline/candidate walls 67.9107908 and 67.7257618 seconds.

Both baselines reused at iteration 7 and had zero recorded control and duration
difference at iteration 8. Recursive comparison at `1e-9`, after removing only
elapsed fields and the three declared retired arrays, passed both complete
results with maximum numeric difference exactly zero.

Verification evidence:

- `testPlannerContract`: 15/15 in 40.3201808 seconds;
- engine, architecture, timed, contract/failure, and sandbox diagnostics:
  49/49 in 63.6097376 seconds wall and 58.4247912 aggregate test seconds;
- Code Analyzer: zero findings in the engine and revised contract test;
- complete test tree: 123/123 in 88.5057376 seconds wall and
  81.5314915 aggregate test seconds;
- visible `exampleObstacleFree`: success, validation, two figures, no warning;
- hidden `exampleNoPath`: expected `noValidatedSeed`, one attempted seed, one
  figure, and the reason present in figure text.

Every maintained example ran headlessly in a separate fresh MATLAB process
with jerk enabled:

| Example | Success / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic / certificate | Wall (s) | Reason |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0333716767 | 16.0333716767 | 10.5 | 1 / 1 / NaN | 1.4242522 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7952270203 | 12.7952270203 | 8.5 | 1 / 1 / NaN | 4.5123487 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 / NaN | 2.0104120 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 / NaN | 0.9858665 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.30889024019 | 7.30889024019 | 6.11111111111 | 1 / 1 / NaN | 0.9838127 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.0903015137 | 1 / 1 / NaN | 2.0575241 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12.1077259407 | 12.1077259407 | 8.5 | 1 / 1 / NaN | 6.8032134 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40.2805670061 | 40.2805670061 | 7.91666666667 | 1 / 1 / NaN | 26.6797632 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN / NaN | 2.8039494 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4118613877 | 7.57454176632 | 1 / 1 / 1 | 4.7854012 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 / NaN | 0.9581027 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10 | 11.5843333838 | 1 / 1 / NaN | 1.9827518 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 40.255028504 | 20.712447786 | 1 / 1 / 1 | 3.7843060 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 27.950433436 | 13.5713266002 | 20.8695652174 | 1 / 1 / 1 | 24.3366819 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.1357890335 | 20.6100682085 | 24 | 1 / 1 / 1 | 15.8160799 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.0767724922 | 24.0767724922 | 21.6333333333 | 1 / 1 / NaN | 4.3152718 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.070643085 | 23.3457566443 | 5.81065318159 | 1 / 1 / 1 | 67.2328108 | `goalReached` |

The milestone removes exactly 17 engine lines and adds no production
replacement. The ten-milestone branch is 213 non-test MATLAB lines smaller
than `5c0a6c9`.

## Dead planner-option shim retirement - 2026-09-01

At tracked-clean baseline `12d72cc`, ten obsolete planner fields were absent
from defaults and had no algorithmic reader. Passing all ten directly produced
a resolved record exactly equal to defaults. Passing live options alongside the
nine obsolete planner-only example fields ultimately produced the same 14-field
resolved planner record after the legacy forwarding and stripping chain.

The candidate removes the seven bespoke deprecation-warning blocks and the
example forwarding allowlist. Direct planner calls now aggregate all obsolete
fields into `planTrajectory:UnknownOptions`; examples discard obsolete
planner-only inputs under `resolveExampleOptions:UnknownOptions`. Planner-level
`Verbose` is unknown, while the separate example display `Verbose` control is
unchanged. Candidate default, live, example-chain, and display option records
all matched their saved baselines exactly.

Fresh-process physical comparisons at `1e-9`, excluding only fields containing
`Elapsed` and `FirstValidatedMotionTime_s`, passed with maximum numeric
difference zero:

- `exampleObstacleFree`: baseline/candidate walls 0.7509786 and 0.7641284
  seconds, success/validation 1/1;
- `exampleTargetExitsObstacle`: baseline/candidate walls 15.9860619 and
  15.7913810 seconds, success/validation 1/1.

Verification evidence:

- Code Analyzer: zero findings in both changed production files and both
  revised tests;
- planner-option and example-boundary tests: 14/14;
- complete test tree: 113/113 in 88.1141873 seconds wall and 81.4978243
  aggregate test seconds;
- visible `exampleObstacleFree`: success, validation, two figures, no warning;
- hidden `exampleNoPath`: expected `noValidatedSeed`, one attempted seed, one
  figure, and the reason present in figure text.

The first Obstacle Free baseline capture omitted the `trajectory` folder from
the temporary MATLAB path and stopped before planning. The corrected fresh
process succeeded. The first no-path figure command had an unterminated shell
string and did not execute the example; the corrected script-based fresh
process passed. Neither failure involved production planner code.

Every maintained example ran headlessly in a separate fresh MATLAB process
with jerk enabled:

| Example | Success / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic / certificate | Wall (s) | Reason |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0333716767 | 16.0333716767 | 10.5 | 1 / 1 / NaN | 1.2755374 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7952270203 | 12.7952270203 | 8.5 | 1 / 1 / NaN | 4.3019420 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 / NaN | 1.8690915 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 / NaN | 0.8303314 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.30889024019 | 7.30889024019 | 6.11111111111 | 1 / 1 / NaN | 0.8724000 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.0903015137 | 1 / 1 / NaN | 1.9318499 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12.1077259407 | 12.1077259407 | 8.5 | 1 / 1 / NaN | 6.7659094 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40.2805670061 | 40.2805670061 | 7.91666666667 | 1 / 1 / NaN | 26.4149146 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN / NaN | 2.5300496 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4118613877 | 7.57454176632 | 1 / 1 / 1 | 4.6763214 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 / NaN | 0.7428393 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10 | 11.5843333838 | 1 / 1 / NaN | 1.8152035 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 40.255028504 | 20.712447786 | 1 / 1 / 1 | 3.5999470 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 27.950433436 | 13.5713266002 | 20.8695652174 | 1 / 1 / 1 | 24.1460202 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.1357890335 | 20.6100682085 | 24 | 1 / 1 / 1 | 15.7535135 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.0767724922 | 24.0767724922 | 21.6333333333 | 1 / 1 / NaN | 4.1600466 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.070643085 | 23.3457566443 | 5.81065318159 | 1 / 1 / 1 | 67.6307320 | `goalReached` |

The milestone removes 64 production lines and adds four ordinary field-owner
substitutions, a net reduction of 60 non-test MATLAB lines. The eleven-milestone
branch is 273 non-test MATLAB lines smaller than `5c0a6c9`.

## Travel-refinement trace retirement - 2026-09-01

At tracked-clean baseline `aa8d601`, all 37 `TravelRefinement*` references were
inside `trajectory/+bmtpEngine/solve.m`. Fifteen diagnostics fields and their
assignments had no external consumer. The candidate removes only this payload,
uses one local accepted-state boolean, and leaves the balanced/fixed refinement
portfolio, collision plane updates, objective comparisons, and selected motion
unchanged.

The default Obstacle Avoidance probe succeeded but did not attempt refinement
because its scenario defaults use earliest arrival, so it was rejected as
coverage. The retained explicit balanced run accepted all three successful
portfolio solves at selected rate 1 deg/s. Fixed Target Exits accepted its one
successful rate-1 refinement.

Fresh-process recursive comparisons at `1e-9`, excluding only fields containing
`Elapsed`, `FirstValidatedMotionTime_s`, and the fifteen declared
`TravelRefinement*` fields, passed with maximum numeric difference zero:

- balanced Obstacle Avoidance: baseline/candidate walls 6.7036477 and
  6.7250928 seconds, success/validation 1/1;
- fixed Target Exits: baseline/candidate walls 15.7873509 and 15.7295776
  seconds, success/validation 1/1.

Verification evidence:

- Code Analyzer: zero findings in the engine and revised contract test;
- `testBmtpEngine` plus `testPlannerContract`: 27/27 in 42.0186741 seconds
  wall and 38.2584586 aggregate test seconds;
- complete test tree: 113/113 in 87.9262869 seconds wall and 81.3773886
  aggregate test seconds;
- visible `exampleObstacleFree`: success, validation, two figures, no warning;
- hidden `exampleNoPath`: expected `noValidatedSeed`, one attempted seed, one
  figure, and the reason present in figure text.

The first focused batch passed all 27 tests but intentionally failed its final
assert because Code Analyzer exposed one now-unused `bestDuration_s`
assignment. Removing that diagnostic residue produced zero findings, and the
fresh 27-test rerun passed. No planner result failed.

Every maintained example ran headlessly in a separate fresh MATLAB process
with jerk enabled:

| Example | Success / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic / certificate | Wall (s) | Reason |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0333716767 | 16.0333716767 | 10.5 | 1 / 1 / NaN | 1.2947553 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7952270203 | 12.7952270203 | 8.5 | 1 / 1 / NaN | 4.3130783 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 / NaN | 1.8858599 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 / NaN | 0.8700775 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.30889024019 | 7.30889024019 | 6.11111111111 | 1 / 1 / NaN | 0.8817736 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.0903015137 | 1 / 1 / NaN | 1.9179395 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12.1077259407 | 12.1077259407 | 8.5 | 1 / 1 / NaN | 6.7246965 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40.2805670061 | 40.2805670061 | 7.91666666667 | 1 / 1 / NaN | 26.4802310 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN / NaN | 2.5940667 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4118613877 | 7.57454176632 | 1 / 1 / 1 | 4.6683283 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 / NaN | 0.7358218 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10 | 11.5843333838 | 1 / 1 / NaN | 1.8242300 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 40.255028504 | 20.712447786 | 1 / 1 / 1 | 3.5662948 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 27.950433436 | 13.5713266002 | 20.8695652174 | 1 / 1 / 1 | 24.3575863 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.1357890335 | 20.6100682085 | 24 | 1 / 1 / 1 | 15.8239935 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.0767724922 | 24.0767724922 | 21.6333333333 | 1 / 1 / NaN | 4.1479165 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.070643085 | 23.3457566443 | 5.81065318159 | 1 / 1 / 1 | 67.3130925 | `goalReached` |

The milestone deletes 50 and adds four core MATLAB lines, net minus 46, and
removes fifteen fields from every BMTP diagnostics record. The twelve-milestone
branch is 319 non-test MATLAB lines smaller than `5c0a6c9`.

## Deprecated BMTP facade removal - 2026-09-01

At tracked-clean baseline `9db9a23`, `trajectory/planTrajBmtp.m` had no
production caller. Its only references were migration/architecture tests,
current appendix text, and historical records. A direct fixed-arrival fixture
proved its candidate and diagnostics were recursively identical to
`bmtpEngine.solve` after excluding elapsed fields: 10 seconds,
4.00000000197 degrees, success, and accepted solver evidence.

The candidate deletes the complete 58-line facade, 100 lines of restart-only
tests/helpers, and its two current appendix references. The package engine and
maintained public planner remain. This is an intentional breaking change for
external direct callers of `planTrajBmtp`; no replacement facade was added.

Fresh-process comparisons at `1e-9`, excluding only fields containing
`Elapsed` and `FirstValidatedMotionTime_s`, all passed with maximum numeric
difference zero:

- direct package-engine candidate and diagnostics;
- `exampleObstacleFree`, baseline/candidate walls 0.7556770 and 0.7671999
  seconds;
- `exampleTargetExitsObstacle`, baseline/candidate walls 15.7366482 and
  15.7810529 seconds.

Verification evidence:

- Code Analyzer: zero findings in the revised architecture and engine tests;
- focused architecture plus BMTP engine tests: 18/18 in 5.5806524 seconds wall
  and 2.0638914 aggregate test seconds;
- complete test tree: 110/110 in 86.8524662 seconds wall and 80.3306783
  aggregate test seconds;
- visible `exampleObstacleFree`: success, validation, two figures, no warning;
- hidden `exampleNoPath`: expected `noValidatedSeed`, one attempted seed, one
  figure, and the reason present in figure text;
- tracked MATLAB has no live facade call; its sole name reference is the
  architecture assertion that the file is absent;
- `pdflatex` remains unavailable, so the current appendix source could not be
  compiled on this host.

Every maintained example ran headlessly in a separate fresh MATLAB process
with jerk enabled:

| Example | Success / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic / certificate | Wall (s) | Reason |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0333716767 | 16.0333716767 | 10.5 | 1 / 1 / NaN | 1.2863726 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7952270203 | 12.7952270203 | 8.5 | 1 / 1 / NaN | 4.2754486 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 / NaN | 1.8746644 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 / NaN | 0.8275977 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.30889024019 | 7.30889024019 | 6.11111111111 | 1 / 1 / NaN | 0.8774148 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.0903015137 | 1 / 1 / NaN | 1.9046406 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12.1077259407 | 12.1077259407 | 8.5 | 1 / 1 / NaN | 6.7484003 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40.2805670061 | 40.2805670061 | 7.91666666667 | 1 / 1 / NaN | 26.5350788 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN / NaN | 2.5406131 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4118613877 | 7.57454176632 | 1 / 1 / 1 | 4.6391760 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 / NaN | 0.7585376 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10 | 11.5843333838 | 1 / 1 / NaN | 1.8351888 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 40.255028504 | 20.712447786 | 1 / 1 / 1 | 3.5845367 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 27.950433436 | 13.5713266002 | 20.8695652174 | 1 / 1 / 1 | 24.2076020 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.1357890335 | 20.6100682085 | 24 | 1 / 1 / 1 | 15.7165294 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.0767724922 | 24.0767724922 | 21.6333333333 | 1 / 1 / NaN | 4.1540480 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.070643085 | 23.3457566443 | 5.81065318159 | 1 / 1 / 1 | 67.4661782 | `goalReached` |

The milestone removes the full 58-line production facade and one competing
public function. The thirteen-milestone branch is 377 non-test MATLAB lines
smaller than `5c0a6c9`.

## General BMTP final-plane solver - 2026-09-01

Baseline was clean commit `3e5d62e`; only the durable handoff and private
MATLAB evidence were untracked. The candidate deletes `createAxisPlane` and
routes every unresolved final certificate pair through the existing
`solveMaximumMarginPlane` implementation. It changes only
`trajectory/+bmtpEngine/solve.m`, removes 49 and adds nine lines, and reduces
the fourteen-milestone branch by 417 non-test MATLAB lines relative to
`5c0a6c9`.

Verification evidence:

- Code Analyzer: zero findings for changed `solve.m`;
- saved four-run Obstacle Avoidance comparison: maximum physical/history
  difference zero; warmed median 2.1769730 to 2.2928512 seconds, +5.323%;
- saved moving fixed-arrival Target Exits comparison: maximum physical/history
  difference zero; certificate passed with the same six reused pairs;
- focused `testBmtpEngine` plus `testPlannerContract`: 24/24 passed in
  40.7189783 seconds wall and 39.2945019 aggregate test seconds;
- complete test tree: 110/110 passed in 86.9001094 seconds wall and
  84.1559378 aggregate test seconds;
- visible `exampleObstacleFree`: success, validation, two visible figures,
  and no warning;
- hidden `exampleNoPath`: expected-failure validation, one figure, and
  `noValidatedSeed` present in figure text;
- `git diff --check` passed before record updates.

Every maintained example ran after the production change in its own fresh
serial MATLAB process with jerk enabled. Target Exits is the fresh focused
candidate process; every other row is from the serial broad sweep:

| Example | Success / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic / certificate | Wall (s) | Reason |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0333716767 | 16.0333716767 | 10.5 | 1 / 1 / NaN | 1.3650576 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7952270203 | 12.7952270203 | 8.5 | 1 / 1 / NaN | 4.3543889 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 / NaN | 1.9304695 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 / NaN | 0.9185245 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.30889024019 | 7.30889024019 | 6.11111111111 | 1 / 1 / NaN | 0.9666285 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.0903015137 | 1 / 1 / NaN | 1.9923528 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12.1077259407 | 12.1077259407 | 8.5 | 1 / 1 / NaN | 6.7345516 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40.2805670061 | 40.2805670061 | 7.91666666667 | 1 / 1 / NaN | 26.5575720 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN / NaN | 2.6079726 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4118613877 | 7.57454176632 | 1 / 1 / 1 | 4.7791563 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 / NaN | 0.8230975 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10 | 11.5843333838 | 1 / 1 / NaN | 1.8819802 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 40.255028504 | 20.712447786 | 1 / 1 / 1 | 3.6331342 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 27.950433436 | 13.5713266002 | 20.8695652174 | 1 / 1 / 1 | 34.6242062 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.1357890335 | 20.6100682085 | 24 | 1 / 1 / 1 | 18.8708768 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.0767724922 | 24.0767724922 | 21.6333333333 | 1 / 1 / NaN | 4.2137733 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.070643085 | 23.3457566443 | 5.81065318159 | 1 / 1 / 1 | 83.4566454 | `goalReached` |

The candidate total is 199.7103879 seconds versus 169.2320276 seconds for the
immediately preceding facade-removal record, an 18.010 percent aggregate
increase. Straight Target has the largest single cold increase at 43.030
percent. This milestone is retained for one general certificate algorithm and
40 fewer production lines; no runtime improvement is claimed.

## Uniform BMTP final certification - 2026-09-01

The baseline was exact detached worktree commit `514185b`. The candidate
removes the final retained-parent-plane restriction shortcut, so every
applicable final BMTP pair is solved and verified through
`solveMaximumMarginPlane`. Optimizer plane reuse inside trajectory optimization
is unchanged. The change removes 42 and adds nine lines in
`trajectory/+bmtpEngine/solve.m`.

Focused evidence:

- Obstacle Avoidance matched success, validation, reason, seed 3,
  7.57454176632-second duration, 11.4118613877-degree motion, and every sampled
  history value with maximum difference zero. Its certificate passed with 18
  conic and zero reused pairs instead of 12 conic and six reused pairs.
- Four-run Obstacle Avoidance baseline walls were 5.3917737, 2.6832424,
  2.2880017, and 2.2017332 seconds. Candidate walls were 4.9472324, 2.6379999,
  2.3334234, and 2.2597204 seconds. The warmed median grew 1.985 percent.
- Moving fixed-arrival Target Exits matched success, validation, reason, seed
  1, 24-second duration, 20.6100682085-degree motion, and every sampled history
  value with maximum difference zero. Its certificate passed with 12 conic and
  zero reused pairs instead of six conic and six reused pairs. Wall time grew
  from 18.8165606 to 19.7293136 seconds, 4.851 percent.
- Code Analyzer reported zero findings and `git diff --check` passed.
- Focused `testBmtpEngine` plus `testPlannerContract`: 24/24 passed in
  41.2646313 seconds wall and 39.8502065 aggregate test seconds.
- Complete test tree: 110/110 passed in 87.6470229 seconds wall and
  84.8867422 aggregate test seconds.
- Visible `exampleObstacleFree`: success, validation, two figures, one visible
  figure state, and no warning.
- Hidden `exampleNoPath`: expected `noValidatedSeed`, independent validation,
  one attempted seed, one diagnostic figure, and the reason present in figure
  text.

Every maintained example ran after the production change in its own fresh,
serial MATLAB process with jerk enabled. Obstacle Avoidance and Target Exits
are the focused candidate processes; all other rows are from the serial broad
sweep:

| Example | Success / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic / certificate | Wall (s) | Reason |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0333716767 | 16.0333716767 | 10.5 | 1 / 1 / NaN | 1.3351494 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7952270203 | 12.7952270203 | 8.5 | 1 / 1 / NaN | 4.4141372 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 / NaN | 1.9318843 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 / NaN | 0.9351499 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.30889024019 | 7.30889024019 | 6.11111111111 | 1 / 1 / NaN | 0.9524044 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.0903015137 | 1 / 1 / NaN | 2.0080579 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12.1077259407 | 12.1077259407 | 8.5 | 1 / 1 / NaN | 6.9376122 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40.2805670061 | 40.2805670061 | 7.91666666667 | 1 / 1 / NaN | 26.5714743 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN / NaN | 2.5781854 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4118613877 | 7.57454176632 | 1 / 1 / 1 | 4.9472324 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 / NaN | 0.8712688 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10 | 11.5843333838 | 1 / 1 / NaN | 1.9110820 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 40.255028504 | 20.712447786 | 1 / 1 / 1 | 3.7309015 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 27.950433436 | 13.5713266002 | 20.8695652174 | 1 / 1 / 1 | 35.7826915 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.1357890335 | 20.6100682085 | 24 | 1 / 1 / 1 | 19.7293136 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.0767724922 | 24.0767724922 | 21.6333333333 | 1 / 1 / NaN | 4.2052408 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.070643085 | 23.3457566443 | 5.81065318159 | 1 / 1 / 1 | 85.3161697 | `goalReached` |

The candidate total is 204.1579553 seconds versus 199.7103879 seconds for the
immediately preceding general-final-plane record, a 2.227 percent aggregate
increase. The milestone is retained for one final-certificate algorithm, 33
fewer production lines, and exact maintained physical results. No runtime
improvement is claimed.

## One moving-obstacle spatial projection - 2026-09-01

The baseline was clean commit `31577c4`. The candidate removes the
static-obstacle-only BMTP attempt from dynamic multi-waypoint orchestration,
leaving the conservative swept protected-history projection followed by true
timed-cell BMTP. The production edit removes 66 and adds four lines in
`+obstacleAvoidance/+planner/planCorridorQuintic.m`.

Focused evidence:

- The distant-translating-obstacle fixture moved from static-only BMTP to
  `SweptProjection.Outcome = acceptedAfterFullValidation`. It retained seed 2,
  the time-expanded source, 14.5963802711-second duration,
  34.9232288125-degree polyline, 39.6810414459-degree sampled motion, and every
  sampled state and derivative with maximum difference zero. Its certificate
  passed. Cold wall moved from 12.5067717 to 12.7454047 seconds.
- The moving-circle plus static-concave fixture still selected
  `bmtpTimedCell` after swept projection failed. It retained seed 2,
  35-second duration, 45.5741988392-degree polyline,
  36.6949453597-degree sampled motion, and every sampled state and derivative
  with maximum difference zero. Its certificate passed. Cold wall moved from
  15.8076512 to 10.7781744 seconds; no speed claim is based on one run.
- Code Analyzer reported zero findings and `git diff --check` passed.
- Focused orchestration and policy tests: 39/39 passed in 59.3958223 seconds
  wall and 54.3933985 aggregate test seconds.
- Complete test tree: 110/110 passed in 83.6459910 seconds wall and
  76.8673640 aggregate test seconds.
- Visible `exampleObstacleFree`: success, validation, two visible figures, and
  no warning.
- Hidden `exampleNoPath`: expected `noValidatedSeed`, one attempted seed, one
  diagnostic figure, and the reason present in figure text. An initial
  post-plan harness assertion used a nonscalar string array; the corrected
  scalar check passed in a fresh rerun.

Every maintained example ran after the production change in its own fresh,
serial MATLAB process with jerk enabled:

| Example | Success / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic / certificate | Wall (s) | Reason |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0333716767 | 16.0333716767 | 10.5 | 1 / 1 / NaN | 1.3209333 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7952270203 | 12.7952270203 | 8.5 | 1 / 1 / NaN | 4.3217680 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 / NaN | 1.8256271 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 / NaN | 0.8477667 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.30889024019 | 7.30889024019 | 6.11111111111 | 1 / 1 / NaN | 0.8459686 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.0903015137 | 1 / 1 / NaN | 2.1278543 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12.1077259407 | 12.1077259407 | 8.5 | 1 / 1 / NaN | 7.2826389 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40.2805670061 | 40.2805670061 | 7.91666666667 | 1 / 1 / NaN | 28.8030877 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN / NaN | 2.6978418 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4118613877 | 7.57454176632 | 1 / 1 / 1 | 5.5620738 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 / NaN | 0.7546874 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10 | 11.5843333838 | 1 / 1 / NaN | 2.3167401 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 40.255028504 | 20.712447786 | 1 / 1 / 1 | 3.7462718 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 27.950433436 | 13.5713266002 | 20.8695652174 | 1 / 1 / 1 | 37.8071321 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.1357890335 | 20.6100682085 | 24 | 1 / 1 / 1 | 20.0299007 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.0767724922 | 24.0767724922 | 21.6333333333 | 1 / 1 / NaN | 4.2540140 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.070643085 | 23.3457566443 | 5.81065318159 | 1 / 1 / 1 | 87.3143863 | `goalReached` |

The candidate total is 211.8586926 seconds versus 204.1579553 seconds for the
uniform-final-certificate milestone, a 3.772 percent aggregate increase. The
milestone is retained for one moving-obstacle spatial projection, 62 fewer
production lines, and exact maintained physical results. No runtime
improvement is claimed.

## Completion audit and timed-layer-budget repair - 2026-09-01

The tracked baseline was pushed commit `df6a85c`. A final replay of the five
supplied Rogue bundles found one unfavorable result change: balanced
`non-ideal` still succeeded and independently validated, but sampled motion was
228.680505208 degrees instead of the recorded 228.491135293 degrees. Detached
replays at `ebd12827`, `8ac19c5`, `8e5a3b1`, `7ed0cf6`, and `5a8eee0`
localized the first changed result to `5a8eee0`. The removed fixture option was
`CollocationSegmentCount = 8`; internalizing the former default had changed
the timed-cell cap from 16 to 20.

The option remains removed. `solveTimedBmtpTrajectory` now caps the clock
recovered from `seed.tau` at `MaximumTimeLayerCount - 1`, matching the maximum
number of intervals the search-layer budget can author. A focused test checks
that the selected timed BMTP coverage respects this invariant. The focused
file passed 3/3, and the repaired balanced `non-ideal` replay restored
228.491135293 degrees exactly at the same 144-second arrival.

Verification commands used fresh `matlab -batch` processes. The complete test
tree passed 111/111 with zero failures or incomplete tests in 83.6258138
seconds wall. The only warning was the maintained obstacle-normalization
fixture warning. Visible `exampleObstacleFree` succeeded, independently
validated, and created two visible figures. Hidden `exampleNoPath` returned
the expected `noValidatedSeed`, passed example validation, attempted a seed,
and created one diagnostic figure containing the reason. An initial diagnostic
harness queried a nonexistent `IterationCount` after correct planning; the
corrected fresh run used `AttemptedSeedCount` and passed.

Every maintained example ran serially after the repair in its own fresh MATLAB
process with jerk enabled:

| Example | Success / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic / certificate | Wall (s) | Reason |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | 1 / 1 | 16.0333716767 | 16.0333716767 | 10.5 | 1 / 1 / NaN | 1.4355073 | `goalReached` |
| `exampleDenseConcaveObstacle` | 1 / 1 | 12.7952270203 | 12.7952270203 | 8.5 | 1 / 1 / NaN | 4.4581631 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1 / 1 | 20 | 20 | 22 | 1 / 1 / NaN | 2.0411611 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 / NaN | 0.9786752 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1 / 1 | 7.30889024019 | 7.30889024019 | 6.11111111111 | 1 / 1 / NaN | 1.0077032 | `goalReached` |
| `exampleMovingBarrierWait` | 1 / 1 | 10 | 10 | 10.0903015137 | 1 / 1 / NaN | 2.0700575 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1 / 1 | 12.1077259407 | 12.1077259407 | 8.5 | 1 / 1 / NaN | 6.9392512 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | 1 / 1 | 40.2805670061 | 40.2805670061 | 7.91666666667 | 1 / 1 / NaN | 26.7286900 | `goalReached` |
| `exampleNoPath` | 0 / 1 | NaN | NaN | NaN | NaN / NaN / NaN | 2.7156151 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | 1 / 1 | 11.152119519 | 11.4118613877 | 7.57454176632 | 1 / 1 / 1 | 4.9372917 | `goalReached` |
| `exampleObstacleFree` | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 / NaN | 0.8625416 | `goalReached` |
| `exampleOpeningUShapedObstacle` | 1 / 1 | 10 | 10 | 11.5843333838 | 1 / 1 / NaN | 1.9661376 | `goalReached` |
| `exampleStaticUShapedObstacle` | 1 / 1 | 34.9425880405 | 40.255028504 | 20.712447786 | 1 / 1 / 1 | 3.7370093 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1 / 1 | 27.950433436 | 13.5713266002 | 20.8695652174 | 1 / 1 / 1 | 35.6673457 | `goalReached` |
| `exampleTargetExitsObstacle` | 1 / 1 | 20.1357890335 | 20.6100682085 | 24 | 1 / 1 / 1 | 19.6606053 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1 / 1 | 24.0767724922 | 24.0767724922 | 21.6333333333 | 1 / 1 / NaN | 4.7302067 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | 1 / 1 | 22.070643085 | 23.3457566443 | 5.81065318159 | 1 / 1 / 1 | 85.4569561 | `goalReached` |

Aggregate maintained-example wall time was 205.3929177 seconds versus
211.8586926 seconds at `df6a85c`, a 3.052 percent decrease in cold serial
runs. This is reported as observation only; no speedup is claimed.

Final fresh supplied-bundle replays:

| Bundle and policy | Success / validation | Selected source / solver | Polyline (deg) | Motion (deg) | Duration (s) | Wall (s) |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| `sinetraj`, saved earliest | 1 / 1 | fixed-clock lateral excursion | 146.928879089 | 146.928879089 | 70.3442509975 | 10.8391817 |
| `newheart`, saved balanced | 1 / 1 | fixed-clock lateral excursion | 199.268051966 | 199.268051966 | 100.970425693 | 69.9664476 |
| `shrimp`, documented balanced | 1 / 1 | fixed-clock lateral excursion | 175.703912280 | 175.703912280 | 81.4551422825 | 44.8156242 |
| `non-ideal`, documented balanced | 1 / 1 | timed visibility / `bmtpTimedCell` | 227.905577664 | 228.491135293 | 144 | 24.1637265 |
| `hiddenruckigfallback`, saved earliest | 1 / 1 | visibility / `bmtpStaticDegree16` | 248.239063282 | 233.911502487 | 104.261456926 | 22.3702489 |

The branch remains 513 non-test MATLAB lines smaller than `5c0a6c9`. The
repair changes ownership rather than restoring an option or adding a special
case; all retained specialized-looking paths have distinct measured fixtures
or public contracts, so no further deletion candidate passed the bounded
retention threshold in this audit.

## Orthogonal planner removal - 2026-09-01

Baseline was clean pushed commit `0f9c268`. The user explicitly classified the
orthogonal-cavity and timed-orthogonal-opening families as benchmark-specific
and required their complete removal while keeping both U examples. Baselines
from fresh processes were:

- Static U: success/validation 1/1, visibility seed 1, polyline
  34.9425880405 degrees, motion 40.255028504 degrees, duration
  20.712447786 seconds, wall 3.8604474 seconds. Its orthogonal portfolio had
  attempted three seeds and selected its second special candidate.
- Opening U: success/validation 1/1, `timedOrthogonalOpening`, polyline and
  motion 10 degrees, duration 11.5843333838 seconds, wall 2.0777559 seconds.

Deleted production files were `createOrthogonalCavityMotion.m`,
`certifyOrthogonalCavityLowerBound.m`,
`createTimedOrthogonalOpeningMotion.m`,
`certifyTimedOpeningRequestLowerBound.m`,
`certifyGuardedRectangleContainment.m`, and the now-dead
`evaluateArrivalCertificatePortfolio.m`. Every call and dedicated diagnostic
branch was removed from `planCorridorQuintic`; dedicated certificate tests and
manual stations were removed. An architecture test now requires every file to
remain absent and rejects orthogonal/cavity logic in the orchestrator.

No replacement was added. A bounded experiment broadened swept/timed BMTP from
time-expanded multi-waypoint seeds to every non-wait dynamic seed. Opening U
still selected the same 13.6175223541-second direct-wait motion; the additional
timed solves failed numerically and only added work. The two-line experiment
was removed before broad verification.

Focused post-removal results:

- Static U: success/validation 1/1 through visibility seed 3 and
  `bmtpStaticDegree16`; polyline 34.9425880405 degrees, motion
  39.4001427062 degrees, duration 20.7814508253 seconds, certificate passed.
- Opening U: success/validation 1/1 through general `directWait`; polyline and
  motion 10 degrees, duration 13.6175223541 seconds.
- Structurally different Dense Concave: success/validation 1/1 through the
  existing fixed-clock path, with unchanged 12.7952270203-degree motion and
  8.5-second duration.

Code Analyzer reported zero findings. Focused architecture, planner-contract,
example-invariant, stage-timing, timed-BMTP, and unsupported-policy tests passed
42/42 in 44.2016562 seconds wall. The complete test tree passed 108/108 with
zero failures or incomplete tests in 74.1749327 seconds wall. The only warning
was the expected two-vertex obstacle-normalization fixture warning. The test
count fell because the deleted constructors' dedicated tests were removed and
rose by one for the new absence boundary.

Visible `exampleObstacleFree` succeeded, independently validated, and produced
two visible figures. Hidden `exampleNoPath` returned the expected validated
`noValidatedSeed`, attempted a seed, and produced one diagnostic figure with
the reason in its text. The planner-manual exporter completed and returned
`ValidationPassed = 1`; an initial post-export harness queried a nonexistent
summary field after the export had already succeeded.

Every maintained example remained in place and ran serially in its own fresh
MATLAB process with jerk enabled:

| Example | Source | Success / validation | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic / certificate | Wall (s) | Reason |
| --- | --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | fixed-clock | 1 / 1 | 16.0333716767 | 16.0333716767 | 10.5 | 1 / 1 / NaN | 1.3899435 | `goalReached` |
| `exampleDenseConcaveObstacle` | fixed-clock | 1 / 1 | 12.7952270203 | 12.7952270203 | 8.5 | 1 / 1 / NaN | 4.4654619 | `goalReached` |
| `exampleFourAcceleratingCircles` | direct | 1 / 1 | 20 | 20 | 22 | 1 / 1 / NaN | 2.0247071 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | direct | 1 / 1 | 9.53894054682 | 9.53894054682 | 12 | 1 / 1 / NaN | 0.9570363 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | direct | 1 / 1 | 7.30889024019 | 7.30889024019 | 6.11111111111 | 1 / 1 / NaN | 1.0239669 | `goalReached` |
| `exampleMovingBarrierWait` | direct-wait | 1 / 1 | 10 | 10 | 10.0903015137 | 1 / 1 / NaN | 2.0200357 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | fixed-clock | 1 / 1 | 12.1077259407 | 12.1077259407 | 8.5 | 1 / 1 / NaN | 6.8763682 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | fixed-clock | 1 / 1 | 40.2805670061 | 40.2805670061 | 7.91666666667 | 1 / 1 / NaN | 26.6228851 | `goalReached` |
| `exampleNoPath` | none | 0 / 1 | NaN | NaN | NaN | NaN / NaN / NaN | 2.5994901 | `noValidatedSeed` |
| `exampleObstacleAvoidance` | visibility | 1 / 1 | 11.152119519 | 11.4118613877 | 7.57454176632 | 1 / 1 / 1 | 4.8989594 | `goalReached` |
| `exampleObstacleFree` | direct | 1 / 1 | 4.472135955 | 4.472135955 | 4.53112887415 | 1 / 1 / NaN | 0.8951998 | `goalReached` |
| `exampleOpeningUShapedObstacle` | direct-wait | 1 / 1 | 10 | 10 | 13.6175223541 | 1 / 1 / NaN | 11.7516735 | `goalReached` |
| `exampleStaticUShapedObstacle` | visibility | 1 / 1 | 34.9425880405 | 39.4001427062 | 20.7814508253 | 1 / 1 / 1 | 32.4429785 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | visibility | 1 / 1 | 27.950433436 | 13.5713266002 | 20.8695652174 | 1 / 1 / 1 | 35.9045711 | `goalReached` |
| `exampleTargetExitsObstacle` | direct visibility | 1 / 1 | 20.1357890335 | 20.6100682085 | 24 | 1 / 1 / 1 | 19.6584316 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | fixed-clock | 1 / 1 | 24.0767724922 | 24.0767724922 | 21.6333333333 | 1 / 1 / NaN | 4.2880719 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | visibility | 1 / 1 | 22.070643085 | 23.3457566443 | 5.81065318159 | 1 / 1 / 1 | 84.5912531 | `goalReached` |

Serial maintained-example wall time was 242.4110337 seconds versus
205.3929177 seconds at `0f9c268`, an 18.023 percent increase. Fifteen examples
retained exact physical metrics. Static U traded 0.0690030393 seconds of
arrival for 0.8548857978 degrees less sampled travel; Opening U retained its
10-degree path but arrived 2.0331889703 seconds later. These tradeoffs are
accepted for removing benchmark-shaped algorithms, not presented as an
optimization.

The milestone removes 1,804 additional non-test MATLAB lines and reduces
`planCorridorQuintic.m` from 1,023 to 875 physical lines. Cumulatively, the
branch is 2,317 non-test MATLAB lines smaller than `5c0a6c9`.

## Obstacle robustness item 18: explicit timed-search work - 2026-09-01

The adjacent baseline was `5c6a01a`. The retained change replaces the nominal
65,535-layer allowance with two user-visible estimates and bounds:
`MaximumTimedSearchStateCount = 50000` and
`MaximumTimedSearchTransitionCount = 1000000`. The existing 17-layer default
remains a requested ceiling. The search computes an effective layer ceiling
before allocation and reports the requested/effective layers, estimated states
and worst-case transitions, applied bounds, and termination reason.

The focused large-request case asked for 1,000,000 layers on a three-node graph.
It retained three layers with exactly nine estimated states and eighteen
estimated worst-case transitions, reached the goal, and reported both work
limits. A five-state/eight-transition case could not afford two layers and
returned documented empty route shapes with `timedSearchWorkLimit` and zero
allocated-work estimates. Parent and transition indices are now `uint32`; the
effective work-bounded clock ceiling travels with a timed seed into timed BMTP.

Validation uses one stable empty template and explicit assignments rather than
a positional 40-value cell array paired with a separate field-name list.
Timed-search diagnostics use the same pattern, and route diagnostics copy named
fields directly. Code Analyzer returned zero findings for all eight changed or
added MATLAB files. `git diff --check` passed. Options and work-limit tests
passed 7/7 in 1.6254 seconds. Planner contract, obstacle history, timed BMTP,
and route economy expanded focused verification to 35/35 in 60.2045 seconds.

Diff-growth disclosure: existing production files added more than 50 lines in
three cases because explicit stable schema replaced compact positional
construction: `createRouteCandidates.m` added 65/deleted 19,
`timeExpandedVisibilitySearch.m` added 123/deleted 31, and
`validateTrajectory.m` added 91/deleted 51. The change is diagnostic and
work-bounding infrastructure; no runtime speedup or added search completeness
is claimed.

## Opening U history-contract repair - 2026-09-01

The final maintained-example sweep stopped at a real `noValidatedSeed` result
for `exampleOpeningUShapedObstacle`. The earliest failing stage was obstacle
preparation, not search or motion solving. Detached `9ded021` succeeded through
a general `directWait` at 13.6175223541 seconds; detached `5429e23` first
failed. Its newly correct conservative fallback enclosed the example's
unverified one-ring-to-two-ring transition and filled the U cavity during the
transition. The initial wait candidate therefore failed collision validation
with -5.25653520966 degrees reported minimum clearance.

The example input now uses supported active-span semantics. Permanent open U
sides retain the two original side regions. A separate protected center gate
has identical samples from 0 through 7 seconds and is inactive after its last
sample. No production planner behavior, tolerance, safety margin, or hidden
waypoint changed. The repaired example selected `directWait`, passed collision
and independent example validation, retained a 10-degree motion, and arrived
at 13.6165771484 seconds in a 43.3826038-second cold process.

Code Analyzer reported zero findings for the example and the item-18 clock
handoff. Obstacle-history, infrastructure, explicit timed-search work, and
timed-BMTP suites passed 23/23 in 20.8482 seconds. A structurally different
moving-barrier example also passed through `directWait`, with 10-degree motion,
10.0905761719-second duration, and 1.0807334-second warm wall time.
