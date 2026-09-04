# Novel replacement branch assessment

## Exact sparse-cone storage screened out - 2026-09-04

The next runtime experiment preserved the complete polynomial model and every
optimization coefficient. Replacing dense A/d allocations in the two existing
trajectory-cone helpers with sparse storage was +12/-12 production lines. An
isolated coefficient oracle passed 12 degree/span/objective combinations. For
36 degree-16 spans with travel bounds, cone-factor payload fell from 25,025,088
to 8,401,840 bytes (66.426332%); this is not a whole-process memory measurement.

Both static-U and tracked Rogue probes retained exact motion arrays and passed
fresh independent validation. One warmup plus three interleaved Rogue timings
gave baseline min/median/max 34.8299620/35.1832482/43.4353643 s and trial
30.7914931/33.5089032/35.0078188 s. The 4.758927% median reduction missed the
predeclared 5% gate, with overlapping ranges and concurrent desktop load.
The storage change was removed before broader tests/examples. It remains useful
memory evidence, not a retained runtime improvement or proof of no benefit.

Fresh restored-source profiles explain the priority: trajectory solver calls
owned 10.399 of 15.135 static-U seconds and 22.611 of 30.680 Rogue seconds;
the trajectory-assembly function's direct self time was only 0.170/0.362 s.
Small loop cleanups alone have limited scope for the desired runtime reduction.
Numerical solver work remains the measured static bottleneck. These profiled
times include instrumentation and are not unprofiled speedup estimates.

## Degree-seven runtime trial rejected - 2026-09-04

The user explicitly wants to keep this trial as a candidate to revisit because
the runtime reduction may justify its quality tradeoff later. Preserve the
measurements and reconstruction details below. It remains disabled while the
current search prioritizes runtime improvements that preserve motion quality.

The zero-net-line trial changed ordinary exact-region curves from degree 16
to degree 7 while preserving three spans per seed edge, all solver controls,
protected geometry, and authoritative validation. This is a heuristic model
choice: it restricts representational freedom and is not an equivalent-work
optimization. It cannot establish faster or better behavior for every input.

Against `2bb1776`, one warmup and three interleaved full-planner repeats gave
static-U median 12.9614717 -> 6.1450158 s (52.590138% reduction) and the tracked
Rogue request 25.7838108 -> 12.5930101 s (51.159236% reduction). Both remained
independently valid, but both had longer paths and later arrivals. The Rogue
balanced objective also worsened from 216.426609307481 to 216.885375990660 deg.
These are rejected-trial measurements, not retained branch speedups.

All 18 maintained examples then ran headlessly with jerk enabled. Seventeen
motions passed independent collision, kinematic, and certificate validation;
the expected `exampleNoPath` failure remained independently valid. Six paths
became longer. The opposing-U duration increased 1.951746%, exceeding the
predeclared 1% screen. The trial was reverted without a degree/tolerance sweep.
Restored static-U and opposing-U requests reproduced the original motion
arrays exactly and passed fresh independent validation. Executed example rows
remain in `benchmark.csv`; detailed evidence is in `verification.md`.

One full headless pass per version, in identical example order and separate
fresh MATLAB sessions, measured 76.8598485 -> 59.0117140 s summed example wall
time (-17.8481345 s, -23.221662%). This includes example construction and its
own validation, excluding MATLAB startup and the recording helper's extra
validation. It is a screening comparison, not a repeated suite speedup estimate.
Three examples recorded higher trial times on paths unaffected by this change;
timing noise cannot be separated from those single-pass differences. The
restored suite again gave 17 independently valid successes and one valid
expected failure. The runtime benefit is not universal, and no degree change
is retained.

Three other runtime trials were removed before this suite: suppressing unused
clearance outputs slowed its warmed kernel by 10.82%; the Schur trajectory
solver falsely failed a known-feasible request; sharing prepared-shape
evaluation reduced dynamic median time 7.53%, below its 10% gate for 32 net
production lines. No production code from these four trials is retained.
The retained production change remains net +32 lines against `31d9084`.

## Runtime-first evaluation - 2026-09-04

Runtime is the current optimization priority; arrival and path length are
regression checks. No additional production change survived this evaluation.
The retained projection improvement remains 14.2% on the exact dynamic bundle
and 28.1% on the moving-barrier comparison below. Static-U solver runtime has
not demonstrated an improvement.

Three more alternatives were rejected and removed. Fixed-clock path refinement
increased final certified arrival despite holding the raw optimizer clock fixed.
Analytic C0-C3 variable elimination increased sampled length beyond its declared
0.001-degree allowance. Exact evaluated-shape caching preserved all compared
trajectory arrays and search counts, but its 9.5% median runtime reduction
missed the declared 15% gate for adding 32 net lines and persistent cache state.
The restored static and dynamic requests passed independent validation and
reproduced their baseline physical results. These negative results support
keeping the current implementation small; they do not establish that all
equivalent solver representations or geometry reuse strategies are ineffective.
The precise measurements and remaining verification limits are in
`verification.md`; no maintained example was executed in this evaluation.

## Direct-wait motion retiming - 2026-09-04

The timing repair adds 30 net physical production lines in the existing
direct-wait constructor/refiner, alongside the two-line projection change
below. A velocity-only seed estimate can fail construction and then stretch
its motion body to fill the request horizon. The old wait refinement froze
that unnecessarily long body, sometimes retaining a late arrival and sometimes
rejecting a request with an independently demonstrated feasible motion.

The refiner now proposes one shorter body using complete derivative bounds,
then sends it through the same full trajectory validator as every wait trial.
It replaces the incumbent only when that check passes. A collision-invalid
constructed body can also reach this repair; wait refinement still requires
a validated incumbent. Fixed-arrival requests and zero-refinement settings
retain their prior behavior. Four stable diagnostic fields report the trial
and its original/final direct-motion durations. No option or production file
was added. This is a bounded improvement proposal, not a minimum-time proof.

For a 10-degree path with two finite obstacle-activity intervals, earliest
requests with horizons 12 and 16 seconds changed from false failure to valid
8.750061035156-second arrivals. Horizons 24 and 32 changed from
21.400360107422 and 29 seconds to the same 8.750061035156 seconds. A translated
barrier with a diagonal path and initial time 3 seconds improved arrival from
20 to 13.149078369141 seconds; length stayed 10.198039027186 degrees. Common
normalized polynomial samples agreed within 1.1e-14 degrees on the successful
comparison cases. All final focused successes passed independent validation.

The repair is not a uniform runtime improvement. In warmed three-repeat
comparisons against HEAD plus projection batching, median runtime was
0.2338856 -> 0.3722676 seconds for the recovered failure, 0.5202663 ->
0.3767518 for the long-arrival case, 0.3381238 -> 0.6140266 for translation,
and 0.5487427 -> 0.5669623 for the unchanged moving barrier. The implementation
subsequently combined both trial kinds into one loop, removing duplicate
construction/check code; its focused trajectory results were reproduced.

Remaining proven timing defects are separate: the horizon-10 earliest and
horizon-12 balanced requests still reject a zero-dwell timed route despite a
known feasible solution, and non-monotone wait windows can still be missed by
the existing bisection. No path-length reduction or completeness claim is made.
All 124 tests pass, including the three example-executing tests run separately.
All 18 maintained examples were verified headlessly: 17 validated successes
and the expected independently validated `noValidatedSeed` failure. Every
successful motion passes collision, velocity, acceleration, jerk, and applicable
certificate checks. Default and explicit visible controls pass on the
obstacle-free example, and a hidden failure figure was created from retained
diagnostics without replanning. Code Analyzer reports no changed-file issues.
The graphics were inspected; the existing long success-workspace title clips
at the default figure width. Algorithm changes did not alter plotting code.

The final slow-dynamic replay with both retained changes returned exactly the
baseline motion arrays and passed fresh independent validation. Its single
wall time was 29.4817251 seconds; the warmed projection comparisons below,
not this single replay, support the speedup claim. The maintained opening-U
case rejected its new timing proposal and retained the original valid motion,
exercising the incumbent-preservation path.

## Projection-query batching by matrix size - 2026-09-04

The current retained change against `31d90843272c2bbf55b15ffb19c9492ba44160fc`
replaces the fixed 64-point projection block in `pointPolygonClearance` with
a 65,536-element target per temporary projection matrix. Small polygons can
process more points together; very large polygons process at least one complete
point-versus-edge row. The projection formula, edge order, occupancy sign,
tolerances, and all three public outputs are unchanged. This is two net
physical production lines, with no new option or production file.

Warmed 4,000-call kernel blocks, interleaved across the original and changed
bodies, reduced median runtime from 5.5589664 to 4.3462502 seconds for a
49-edge/320-query case, and from 18.0502134 to 13.7082648 seconds for a
64-edge holed/1,024-query case: 21.8% and 24.1%. An initial 400-call measurement
was too variable to support the first result and was repeated with longer
blocks without changing the implementation or acceptance threshold.

Complete warmed planner requests reduced median runtime from 41.5132825 to
35.5981197 seconds for the unchanged `runtimediagnosis.mat` request, and from
1.0005301 to 0.7198244 seconds for the stored moving-barrier request. Selected
routes, time/position/velocity/acceleration arrays, temporal counts, and explored
nodes matched exactly, and independent validation passed. Arrival and sampled
length remained 128.747653905761 seconds / 266.550824223376 degrees and
10.0903015136719 seconds / 10 degrees, respectively. No arrival-time or
path-length reduction is claimed for this change.

Exact geometry comparisons also cover empty, rectangular, concave, holed,
translated, and 4,000-edge polygons. Focused obstacle tests, all 124 tests,
the 18 maintained examples, and default/visible/failure plotting checks have
been completed as recorded above. Repository-wide size targets remain exceeded;
verification of these changes does not establish branch-wide completeness.

Four alternatives were rejected and completely removed: deleting the balanced
temporal objective override (no final quality gain and 97.4% slower on one
request), instantaneous bounding boxes (7.48% dynamic speedup below its 10%
gate), reducing static segmentation (arrival and length worsened), and batching
temporal queries across arrival layers (9.20% slower). Reproduction notes are in
`output/algorithm-improvements/negative-results.md`. Pre-existing user edits
and deleted documents were preserved.

## Collapse zero-information planner layers - 2026-09-03

Excluding pre-existing user edits in the working tree based on `1e321ce`, this
cleanup removes nine production MATLAB files and 357 physical production
lines: 129 files and 19,671 lines become 120 files and 19,314 lines. No public
option, result field, algorithm, or diagnostic was added. The deleted files
were forwarding aliases, subordinate construction steps with one caller, a
misplaced validation inequality, and a duplicated Ruckig result pass.

Polynomial and corridor callers now invoke their authoritative validators
directly. Endpoint validation no longer has a one-call forwarding alias.
Proposal geometry owns its sampled obstacle-union loop, and a visibility
attempt owns the sparse pair list that exists only for that attempt. Corridor
certification now owns its Bernstein inequality conversion instead of reaching
back into the search package. Ruckig solve now evaluates synchronized motion
inline and lets `validateResult` remain the single continuous-constraint
evaluation owner; the public values and rejection classification are
unchanged.

The complete suite passed 123/123 in 158.487585 seconds, including all planner
contract, timed BMTP, Ruckig, and offline-diagnosis tests. Code Analyzer found
zero issues in every changed MATLAB file, and the scoped diff passed whitespace
validation. All 19 maintained examples ran in fresh MATLAB processes with jerk
enabled. Eighteen independently validated successes and the expected validated
`noValidatedSeed` failure matched the `1e321ce` baseline's route lengths and
motion durations within `1e-9`. A visible successful example created both
expected figures, and a hidden failed example created its diagnostic figure.

The isolated 200-solve Ruckig benchmark improved from a 0.371088-second
baseline median to 0.285004 seconds, a 23.2 percent reduction; a separate
baseline repeat had a 0.492362-second median, so the deleted duplicate
validation was a measured cost rather than a source-only inference. The
extreme-visibility example remained runtime-neutral in the controlled repeat:
77.361609 seconds at baseline and 77.102002 seconds after cleanup, with exact
physical outputs. No end-to-end speedup is claimed.

The cleanup stops before helpers that own substantive mathematics, independent
validation, repeated loop invariants, stable public diagnostics, or deprecated
one-release compatibility. Inlining those would move complexity into larger
files rather than remove it. The principal remaining weakness is planner-wide
size, but another deletion pass needs a new, evidenced ownership boundary—not
a target file count.

## Certified continuous polynomial bounds - 2026-09-02

Continuous position, velocity, acceleration, and jerk checks now use a
degree-neutral Bernstein fast path. A complete in-range hull proves an interval;
a hull wholly outside one limit proves failure; one outlying control remains
ambiguous. Ambiguous intervals receive at most two midpoint de Casteljau
subdivisions before the established endpoint and stationary-point evaluation
resolves the result. Degree-specific power-to-Bernstein maps are cached because
reconstructing their binomial coefficients dominated the first implementation.

The focused cubic validation benchmark improved from a 0.225285700-second
median to 0.107645900 seconds per 100 validations, a 52.218 percent reduction.
On the structurally different degree-16, 18-segment obstacle trajectory, an
interleaved same-session comparison produced identical bound decisions and
improved the polynomial-validation-stage median from 0.957598300 to
0.183847000 seconds per 100 calls, an 80.801 percent reduction. The rejected
uncached implementation is recorded in `verification.md`; it was more than five
times slower and is not retained.

MATLAB Code Analyzer reported zero findings, and the complete test suite passed
118/118. All 17 maintained examples ran serially against the final code:
sixteen independently validated successes and the expected validated
`noValidatedSeed` result, with established physical metrics unchanged. The
final sweep took 451.1994148 seconds. This is 10.5 percent below the rejected
uncached sweep but 41.9 percent above the older 317.9755667-second record, so no
end-to-end speedup is claimed from those non-interleaved runs. The current
strength is a certified fast path that never treats one Bernstein coefficient
as exact rejection evidence. The remaining weakness is that ambiguous cases
still depend on polynomial-root conditioning in the conservative stationary
fallback.

## Remove benchmark-shaped orthogonal planners - 2026-09-01

At the user's direction, the branch removes the complete orthogonal-cavity and
timed-orthogonal-opening family rather than retaining it as a benchmark
shortcut. Six production helpers are deleted: both motion constructors, both
request/cavity certifiers, their private guarded-rectangle predicate, and the
now-unreferenced arrival-certificate portfolio. `planCorridorQuintic` no longer
detects an orthogonal cavity, constructs a cavity-shaped motion, recognizes an
opening event specially, or ranks those special candidates. Dedicated tests
and manual sections are removed, while both maintained U-shaped examples stay.

The retained planner passes both U examples through ordinary mechanisms.
Static U uses visibility-graph seed 3 and `bmtpStaticDegree16`, independently
validates its certificate, and returns a 39.4001427062-degree sampled motion in
20.7814508253 seconds. The prior cavity shortcut returned 40.255028504 degrees
in 20.712447786 seconds. Opening U uses the general `directWait` seed,
independently validates a 10-degree motion, and arrives in 13.6175223541
seconds versus 11.5843333838 seconds for the removed opening shortcut. These
quality and runtime costs are explicit; passing the examples no longer depends
on recognizing their orthogonal geometry.

Code Analyzer found no issue, focused architecture/contract tests passed
42/42, and the complete suite passed 108/108. All 17 maintained examples ran
serially: sixteen independently validated successes and the expected validated
`noValidatedSeed` result. Fifteen examples retained their established physical
metrics; only the two U examples changed as described above. Serial wall time
was 242.4110337 seconds versus 205.3929177 seconds at `0f9c268`, an 18.023
percent increase concentrated in the general U paths. No speedup is claimed.

This milestone removes 1,804 additional non-test MATLAB lines and reduces
`planCorridorQuintic.m` from 1,023 to 875 physical lines. The branch is now
2,317 non-test MATLAB lines smaller than `5c0a6c9`. Its largest strength is
that U-shaped examples remain real general-planner regressions instead of
being owned by shape detectors. Its largest weakness is that the general
paths are slower and do not reproduce the deleted shortcuts' arrival times;
improving that gap must come from a structurally general timed or static motion
algorithm, not a restored U/cavity detector.

## Timed BMTP follows the search-layer budget - 2026-09-01

The completion audit caught one result regression that the maintained suite did
not expose: removing `CollocationSegmentCount` had changed timed BMTP from the
saved Rogue fixtures' 16 segments to an unrelated internal cap of 20. The
`non-ideal` fixture still validated, but sampled travel grew from
228.491135293 to 228.680505208 degrees. A detached milestone bisect localized
the change to `5a8eee0`.

The public collocation option remains removed. Timed BMTP now enforces the
input-driven invariant that its time-cell count cannot exceed the search-layer
budget that authored the seed. With 17 search layers, the planner uses at most
16 timed segments. This restores the saved `non-ideal` result exactly and adds
a focused contract test without exposing conic dimension as a user choice.

The full test tree passed 111/111. All 17 maintained examples retained their
established physical metrics: sixteen independently validated successes and
the expected validated `noValidatedSeed` result. Their serial wall time was
205.3929177 seconds versus 211.8586926 seconds at `df6a85c`; cold-run timing
noise prevents a speed claim. Visible-success and hidden-failure diagnostic
gates passed.

All five supplied Rogue sentinels also succeeded and independently validated.
`sinetraj`, `newheart`, balanced `shrimp`, balanced `non-ideal`, and
`hiddenruckigfallback` retained sampled travel of 146.928879089,
199.268051966, 175.703912280, 228.491135293, and 233.911502487 degrees,
respectively. The largest current strength is therefore a 513-line-smaller
non-test MATLAB core with preserved measured outcomes across maintained and
external regression families. The largest remaining weakness is structural:
the retained general BMTP engine and corridor orchestrator are still large,
solver-dependent functions, while the audited timed-opening, direct-wait,
orthogonal-cavity, fixed-clock, travel-refinement, Ruckig, and timed-cell paths
are all load-bearing on distinct inputs. Further deletion needs a new bounded
hypothesis rather than another broad pruning pass.

## One moving-obstacle spatial projection - 2026-09-01

The sixteenth accepted `bmtp-cleanup-codex` milestone removes the opportunistic
static-only BMTP solve from moving-obstacle planning. Eligible dynamic topology
seeds now use one conservative swept protected-history projection before the
existing true timed-cell BMTP solver. Full-scene public validation remains
authoritative, and no option, fallback, solver, or scenario branch was added.

A static concave U plus distant translating polygon moved from the removed
static-only route to the swept projection with maximum numeric difference zero
across success, validation, termination, selected seed and source, arrival,
lengths, and sampled time, position, velocity, acceleration, and jerk. Its
plane certificate and full-scene validation passed. A structurally different
moving-circle plus static-concave case still fell through the conservative
swept representation and selected true timed-cell BMTP, also with maximum
physical difference zero and a valid certificate.

Code Analyzer found no issue, focused orchestration tests passed 39/39, and the
complete suite passed 110/110. All 17 maintained examples retained their
established physical metrics: sixteen independently validated successes and
the expected validated `noValidatedSeed` result. Both visualization gates
passed. Serial maintained-example wall time grew 3.772 percent, from
204.1579553 to 211.8586926 seconds. The focused swept winner grew from
12.5067717 to 12.7454047 seconds in one cold run; no speedup is claimed.

The milestone removes 62 net production MATLAB lines and reduces
`+obstacleAvoidance/+planner/planCorridorQuintic.m` from 1,085 to 1,023
physical lines. Across sixteen accepted milestones, the branch is 512 non-test
MATLAB lines smaller than `5c0a6c9`. Swept geometry remains conservative and
can reject a motion that true time-dependent geometry permits; timed-cell BMTP
is retained for that general case.

## Uniform BMTP final certification - 2026-09-01

The fifteenth accepted `bmtp-cleanup-codex` milestone removes the separate
retained-parent-plane restriction path from final BMTP certification. Every
applicable final output-span and obstacle-region pair now uses the same
degree-one maximum-margin conic solver. Optimizer plane reuse remains active
inside the outer BMTP solve; it is no longer treated as an alternative final
certificate algorithm.

Saved Obstacle Avoidance and moving fixed-arrival Target Exits results matched
the exact `514185b` baseline with maximum numeric difference zero across
success, validation, termination, selected seed, arrival, lengths, and sampled
time, position, velocity, acceleration, and jerk histories. Their certificates
remained independently valid. Ordinary BMTP certification now reports all 18
and all 12 applicable pairs, respectively, as conic pairs and zero as reused
pairs. Code Analyzer found no issue, focused tests passed 24/24, and the full
test tree passed 110/110.

All 17 maintained examples preserved their established physical results:
sixteen independently validated successes and the expected validated
`noValidatedSeed` failure. The focused warmed Obstacle Avoidance median grew
1.985 percent, from 2.2880017 to 2.3334234 seconds. Serial maintained-example
wall time grew 2.227 percent, from 199.7103879 to 204.1579553 seconds. The
runtime cost is accepted for one final-certificate algorithm; no speedup is
claimed.

The milestone removes 33 net production MATLAB lines and reduces
`trajectory/+bmtpEngine/solve.m` from 1,191 to 1,158 physical lines. Across
fifteen accepted milestones, the branch is 450 non-test MATLAB lines smaller
than `5c0a6c9`. The conic separator remains solver-dependent, and exact results
on the maintained families do not establish identical numerical behavior for
every unseen region geometry.

## General BMTP final-plane solver - 2026-09-01

The fourteenth `bmtp-cleanup-codex` milestone removes the BMTP engine's
cardinal-axis final-certificate shortcut. Every output span/region pair whose
retained optimizer plane cannot be reused now goes through the existing
degree-one maximum-margin conic solver. This deletes one separate certificate
algorithm without adding an option, fallback, helper, or replacement branch.

Saved Obstacle Avoidance and moving fixed-arrival Target Exits results matched
baseline with maximum numeric difference zero across time, position, velocity,
acceleration, jerk, arrival, duration, selected seed, and motion length. Their
plane certificates remained independently valid. Obstacle Avoidance shifted
12 analytic pairs to 12 conic pairs while retaining six parent planes; Target
Exits shifted five analytic plus one conic pair to six conic pairs while also
retaining six. Code Analyzer found no issue, focused tests passed 24/24, and
the complete suite passed 110/110 in 86.9001094 seconds wall time.

All 17 maintained examples preserved their established physical results:
sixteen independently validated successes and the expected validated
`noValidatedSeed` failure. The runtime cost is visible. The focused warmed
Obstacle Avoidance median grew 5.323 percent, from 2.1769730 to 2.2928512
seconds. Serial maintained-example wall time grew 18.010 percent in aggregate,
from the immediately preceding 169.2320276-second record to 199.7103879
seconds. The largest single cold movement was Straight Target, from 24.2076020
to 34.6242062 seconds (43.030 percent); Target Exits grew 20.070 percent and US
Outline Extreme grew 23.701 percent. Those are accepted maintainability costs,
not speed improvements or noise claims.

The milestone removes 40 net production MATLAB lines and reduces
`trajectory/+bmtpEngine/solve.m` from 1,231 to 1,191 physical lines. Across
fourteen milestones, the branch is 417 non-test MATLAB lines smaller than
`5c0a6c9`. The general conic separator remains solver-dependent, and exact
results on the maintained families do not prove identical numerical behavior
for every unseen region geometry.

## Deprecated BMTP facade removal - 2026-09-01

The thirteenth `bmtp-cleanup-codex` milestone deletes the 58-line
`planTrajBmtp` compatibility facade. Maintained planners already called
`bmtpEngine.solve` directly; only restart-migration tests and current appendix
text referenced the facade. The maintained public planner remains
`obstacleAvoidance.planTrajectory`, and the package engine remains directly
tested.

Before deletion, the facade and package engine returned recursively identical
fixed-arrival candidates and diagnostics. After deletion, the direct package
engine plus complete Obstacle Free and Target Exits results matched saved
baselines with maximum numeric difference zero after excluding only runtime.
Code Analyzer found no issues, focused tests passed 18/18, and the complete
suite passed 110/110 in 86.8524662 seconds wall time. The three removed tests
covered only the deleted restart and invalid-arity surface.

All 17 maintained examples retained established metrics: sixteen independently
validated successes and the expected validated `noValidatedSeed`. Both
visualization gates passed. This deliberately breaking direct-caller cleanup
removes one competing public function and 58 production MATLAB lines. The
thirteen-milestone branch is now 377 non-test MATLAB lines smaller than
`5c0a6c9`.

## Travel-refinement trace retirement - 2026-09-01

The twelfth `bmtp-cleanup-codex` milestone removes fifteen private
`TravelRefinement*` diagnostic fields and their assignments. No planner,
example, test, plotter, exporter, or sandbox consumed them. The balanced and
fixed-arrival refinement algorithms remain unchanged: rate portfolios,
collision-driven plane updates, objective comparisons, and accepted control
nets are still executed. One local boolean now owns the only behavior-bearing
accepted-state decision.

Explicit balanced Obstacle Avoidance accepted refinement from a three-rate
portfolio, while fixed Target Exits accepted its one-rate refinement. Both
complete candidate results matched saved baselines recursively with maximum
numeric difference zero after excluding only runtime and the retired trace
fields. Code Analyzer found no issues, focused tests passed 27/27, and the full
suite passed 113/113 in 87.9262869 seconds wall time.

All 17 maintained examples retained their established metrics: sixteen
independently validated successes and the expected validated
`noValidatedSeed`. Both visualization gates passed. The milestone removes 46
net core MATLAB lines and fifteen fields from every BMTP diagnostics record.
The twelve-milestone branch is now 319 non-test MATLAB lines smaller than
`5c0a6c9`.

## Dead planner-option shim retirement - 2026-09-01

The eleventh `bmtp-cleanup-codex` milestone removes special compatibility
handling for ten planner fields that had already stopped affecting behavior.
Direct planner calls now report them through the maintained aggregate
`planTrajectory:UnknownOptions` warning, and examples reject obsolete
planner-only fields at their own boundary instead of forwarding them. The live
example display `Verbose` control remains; only the dead planner field with the
same spelling lost bespoke handling.

Default, live override, example-chain, and display-option records match their
saved baselines exactly. Complete Obstacle Free and Target Exits results also
match recursively with maximum numeric difference zero after excluding only
runtime fields. Code Analyzer found no issues, focused tests passed 14/14, and
the complete test tree passed 113/113 in 88.1141873 seconds wall time. The
smaller test count is the intentional consolidation of twelve legacy-specific
warning and forwarding tests into two behavior-focused tests, not lost
live-option coverage.

All 17 maintained examples retained their prior metrics: 16 independently
validated successes and the expected validated `noValidatedSeed` failure.
Visible-success and failure-figure gates passed. This intentionally breaking
warning-surface cleanup removes 60 net production MATLAB lines. The
eleven-milestone branch is now 273 non-test MATLAB lines smaller than
`5c0a6c9`.

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

## Request-Owned Obstacle Preparation And Explicit History Contract — 2026-09-01

The largest current strength is that one request-owned obstacle collection now
normalizes, protects, and prepares source geometry before endpoint checks,
proposal creation, motion solving, wait refinement, and authoritative
validation. Preparation retains an exact public-source snapshot and version,
so any public geometry mutation rebuilds the collection rather than reusing
stale shapes, interval bounds, or boundary edges. The documented history model
uses verified linear corresponding-vertex motion, exact static equivalence,
and a conservative nested occupied-set transition when one endpoint set is
contained in the other. Nonnested unproven transitions retain the endpoint
convex-hull enclosure.

Tracing the unchanged opening-U example located the prior failure at the
earliest broken stage: an unconditional endpoint convex hull filled the U
cavity during its topology change, invalidated the wait state, and left no
validated seed. The nested-set contract preserves the larger exact occupied
set without filling its cavity. The unchanged example now selects the general
`directWait` seed and independently validates a 10-degree motion arriving at
13.6175223541 s, with 0.0000842562 degree minimum clearance. A structurally
different nested-L test and the existing separated-endpoint swept-gap test
protect both sides of the rule.

The largest current weakness remains dynamic multi-waypoint completeness and
runtime. Static spatial proposals still use a whole-history convex hull, which
can discard usable dynamic free space; timed BMTP supports only its current
bounded topology set. The request-local cache removes repeated preparation but
does not make those algorithms complete. `parfor` was evaluated separately,
but the parallel runtime entry points were unavailable in this environment, so
the serial seed loop remains and no nested parallel work was introduced.

In the closest same-process comparison, the pre-fix 17-example sweep used
282.4631766 s and the retained sweep used 317.9755667 s, an increase of
35.5123901 s or 12.572 percent. That aggregate is unfavorable but not a clean
performance comparison: opening U failed after 0.80716 s in the baseline and
succeeded with full validation in 26.1968938 s after the fix. Excluding that
failed-versus-successful case, the other 16 examples increased from
281.6560166 s to 291.7786729 s, or 10.1226563 s and 3.594 percent. No speedup
claim is made. All 17 expected example outcomes passed, the complete
non-example suite passed 106/106, and Code Analyzer reported zero findings.

## Certified Final-Plane Fast Path And Mixed Dynamic Example — 2026-09-02

The measured runtime owner is conic optimization inside BMTP. Before this
change, representative profiles attributed 57 to 5,240 calls per maintained
example to `coneprog`; in static U, 1,598 calls consumed 30.976 seconds and
motion solving consumed 39.858 of 41.099 planner seconds. Final collision-plane
certification was a material but redundant subset: the selected trajectory was
already fixed before every output-span/region pair solved another maximum-margin
SOCP.

The retained fast path tests separating axes from both the convex obstacle and
the final Bezier control hull. A pair is accepted only when the unchanged
`verifyPlane` routine proves the complete Bernstein control net, obstacle side,
normal bound, roundoff reserve, and required clearance. Hull overlap is
ambiguous rather than a rejection; it retains the tight `coneprog` fallback.
This is therefore a sufficient certificate, not a single-coefficient exact
rejection test.

Four-repeat controlled comparisons against clean `ca51871` used one warm-up
and three measured runs. Success, termination reason, selected seed, arrival,
both reported lengths, and complete sampled time, position, velocity,
acceleration, and jerk histories were exactly equal in every comparison.

| Example | Baseline median (s) | Candidate median (s) | Improvement | Final analytic / conic pairs |
| --- | ---: | ---: | ---: | ---: |
| `exampleObstacleAvoidance` | 2.683438 | 2.430598 | 9.422% | 18 / 0 |
| `exampleStaticUShapedObstacle` | 36.439496 | 32.801454 | 9.984% | 498 / 6 |
| `exampleStraightTargetAlternatingOcclusion` | 37.381200 | 26.072233 | 30.253% | 863 / 1 |
| `exampleUSOutlineExtremeVisibility` | 93.018745 | 74.159052 | 20.275% | 319 / 1 |

The 37-line production increase required a 9.25% measured benefit under the
declared bounded-change gate; the smallest measured benefit was 9.422%.
Post-change static-U profiling leaves trajectory optimization as the dominant
cost: `solveTrajectorySocp` used 29.631 seconds, and 601 remaining `coneprog`
calls used 26.492 seconds. Final certification fell to 0.387 seconds, including
0.292 seconds for 1,008 hull tests. Further trajectory-SOCP reduction was not
attempted because it participates in path and arrival selection rather than
replaying a fixed result.

`exampleMovingRotatingObstacleField` is now the eighteenth maintained example.
It plans without waypoints through three static centerline obstacles while a
five-slice rectangle translates and rotates across the competing route. It
independently validated at 20.7160388668 degrees and 9.04166666667 seconds.
The full serial matrix produced 17 validated successes plus the validated
`exampleNoPath` failure. The test audit reduced the suite from 119 to 110 tests
despite adding the new regression, consolidating duplicated example source
contracts and removing historical negative assertions; all 110 tests passed.

## Identical Trajectory SOCP Termination — 2026-09-02

The remaining BMTP cost was trajectory-generating `coneprog` work. When a
collision-free iterate did not improve the retained best motion, the engine
could retain unchanged separating planes and invoke the same earliest-arrival
trajectory SOCP again. At the request horizon every solver input was identical;
the repeated solve returned the same motion and convergence was then reported
from its zero improvement.

The engine now terminates at that fixed point. Expanded-horizon recovery still
continues because its next request-horizon SOCP is different. Controlled warm
medians improved 6.493% for static U, 5.237% for fixed-arrival occlusion, and
4.622% for the complex outline. The small-static no-op sentinel had unchanged
solver work and a noise-level favorable shift. All compared routes, arrivals,
motion lengths, and complete sampled histories were exactly unchanged.

The implementation adds six production lines and no function, option, public
field, or dependency. Its 1.5% size threshold was met by the smallest 4.622%
affected-case gain. All 18 maintained examples matched the prior physical
metrics and validated; the complete suite passed 110/110.

## Direct Sparse Derivative Rows — 2026-09-02

Trajectory-SOCP profiling localized 3.058 seconds in static U to 55,350
full-width sparse-row negations used to create lower derivative bounds. Each
row had only a short control-point stencil and one time-power entry. Writing
those entries directly creates the exact same sparse matrix without copying
the complete preceding row.

The source replacement is +2/-2 production lines and adds no interface,
helper, option, diagnostic, or dependency. Static-U and fixed-arrival warm
medians improved 7.332% and 9.460%; the small-static sentinel remained exact
with a noise-level 0.507% favorable shift. The targeted profiled line fell
from 3.058 to 0.587 seconds. All 18 examples retained their physical metrics
and validated, and all 110 tests passed.

## Block-Sparse Derivative Bounds — 2026-09-02

The remaining scalar sparse writes now assemble as one exact block per segment
and derivative order. Fifty-six degree/segment construction cases produced
bit-for-bit equal inequality matrices. Static-U profiling reduced
`solveTrajectorySocp` from 25.265 to 23.941 seconds without changing its 31
calls, and the controlled warm median improved 4.009%. A fixed-arrival sentinel
was exact and 1.678% favorable relative to the current baseline record.

The production implementation is three lines smaller and adds no interface,
helper, option, or dependency. All 18 maintained examples preserved path length
and arrival time and independently validated, and all 110 tests passed. The
full-matrix wall sum was only 0.469% favorable, so the profile and controlled
static-U comparison—not the aggregate—are the evidence for retention.

## Shared Dynamic Shape Differences — 2026-09-02

Dynamic preparation no longer repeats the same two directed `polyshape`
subtractions when classifying a non-equivalent endpoint pair as nested or
non-nested. One comparison now derives both predicates from the same areas
with the same tolerance and branch outcomes.

The change removes 13 production lines. The moving/deforming-outline controlled
warm median improved 5.344%, while the structurally different moving/rotating
sentinel was exact and 0.790% favorable. All 18 maintained examples preserved
path length and arrival time and independently validated, all 110 tests passed,
and the complete serial matrix was 1.348% favorable as a secondary observation.

## Heuristic Completeness And Runtime Safety — 2026-09-03

The supplied `Rogue Examples/failed.mat` artifact is now an exact regression
fixture rather than a manually interpreted screenshot. Its unchanged request
selects a 143.928296-degree visibility polyline, produces a
145.143798-degree validated smooth motion, and arrives at 71.282812 seconds.
The complete replay test took 41.812 seconds in the final test sequence, while
the trajectory arrived within the requested 80-second planning horizon. Before
the retained changes, the same test sequence took 67.365 seconds. Because
MATLAB warm-up affects wall time, the
stronger localized evidence is topology work: route-cleanup candidates fell
from 32,312 to 590 and measured route-search time fell from 5.819 to
0.663 seconds without changing either selected spatial route.

The retained changes remove three unsupported rejection mechanisms. Spatial
route duration guesses no longer discard seeds or shorten their request
horizon. Time-expanded edges use only the componentwise velocity lower bound;
the prior rest-to-rest acceleration expression was not valid at through-moving
intermediate nodes. Estimated-time BMTP and direct-wait attempts now preserve
the full request horizon, and direct waits repair an optimistic search schedule
with the exact direct-motion duration. A coarse moving-obstacle time-cell solve
is still tried first, but failure retries the full search-layer resolution.
Similarly, static convex-region grouping remains an inexpensive first attempt,
but a grouped failure retries every exact region. The balanced-arrival
`[0.1, 1, 10]` objective-rate portfolio and the same-class waypoint sweep were
removed; only the caller's declared exchange rate and the primary shortest
route in each discovered class remain.

The current heuristics fall into two materially different groups:

- Recoverable proposal accelerators cannot authorize final failure by
  themselves: exact direct and fixed-clock proposal families, greedy
  class-preserving route shortening, sampled BMTP overlap tags, conservative
  static grouping with exact fallback, coarse timed cells with fine fallback,
  and the endpoint-velocity duration lower bound. Every retained motion still
  passes the authoritative continuous validator.
- Explicit work bounds can still make the planner incomplete: the default
  five-seed budget (maximum nine), input-derived discrete time layers,
  10,000-vertex proposal
  switch, 1,000,000 pair-edge visibility budget, sparse Delaunay graph,
  13-sample timed-edge screen, one winding reference per connected occupied
  region, the requested route-class count, and finite BMTP degree, segment,
  and iteration budgets. Dense-envelope use defers timed search until every
  cheaper validated-motion source fails. Higher winding components are no
  longer discarded; their already-found routes are solved if all ordinary
  winding candidates fail. These limits are reported, but they are not a proof
  of no path. `HomologySearchTruncated` now means only that the requested class
  count stopped exploration.

The optional moving-target wrapper retains another bounded method for cases
outside its exact obstacle-free piecewise-linear kernel: 16 chronological
fixed-time intervals followed by at most 16 refinement trials. The optional
Ruckig stop-at-waypoint recovery also supports at most two normalized route
segments. Neither is claimed complete or globally optimal.

The structurally different regressions now protect a nonrest intermediate-node
timed route, an under-timed direct-wait proposal with ample horizon, fine
time-cell recovery after coarse overconstraint, and 66 separated static regions
whose grouped hulls close a valid corridor. The final serial shipped-example
gate passed 19/19 expected outcomes, including the expected validated no-path
case, and the complete MATLAB suite passed 117/117 tests. No scenario name,
expected route, or supplied-artifact geometry was added to production logic.

## Deferred Dense Timed-Search Recovery — 2026-09-03

A dense spatial proposal previously disabled exact-history timed search with
the reason `timedQueryWorkLimit`. That work threshold could therefore authorize
`noValidatedSeed` even though a wait or time-dependent passage existed. A
discriminating contract test first reproduced the suppression. A structurally
different 1,200-vertex moving barrier then showed the end-to-end consequence:
the cheap direct and spatial attempts failed, while the deferred exact-history
search created a direct-wait seed that passed full independent validation.

The retained coordinator now treats the dense shortcut only as work ordering.
It resumes timed search after all initially offered candidates fail validation,
unless a separately generated exact motion has already passed. Only the newly
recovered timed seed is solved; unchanged direct and spatial attempts are not
repeated. The existing `searchRoutes` coordinator now accepts its prior deferred
route set for recovery, runs only the timed portion, and returns before its one
spatial-search call site. Stable diagnostics preserve the initial deferral,
recovery attempt, timed-search record, recovered seed, and candidate validation.
The regression also preserves a sentinel in the prior spatial-search record,
proving recovery reused rather than rebuilt it.

An always-on version was rejected before retention. The dense deforming-outline
sentinel crossed 90 seconds before the post-run reporting expression failed,
whereas the prior warm record was 29.292 seconds. Those failed reporting calls
are not benchmark rows. With lazy recovery, the valid maintained run was
29.108917 seconds with the unchanged 40.2805679610824-degree geometric and
smoothed path and 7.91666666666667-second duration. That timing difference is
within normal noise and is treated as neutral, not as a speedup.

The exact supplied-bundle suite passed 4/4; its unchanged feasible replay took
41.938 seconds and still arrived at 71.282812 seconds. The complete MATLAB suite
passed 119/119 tests in the final 147.042709-second run. The final required
fresh-process example matrix also passed 19/19 with 199.959769 seconds summed
example wall time; path lengths and physical arrival times were identical. A
visible success created two visible figures, and the expected no-path plot
created 158 graphics objects with `noValidatedSeed` in its title. This change
removes one false-negative authority without claiming completeness for the
remaining node, edge, seed, time-layer, winding, or solver work bounds.

The final production change is 94 added and 10 removed lines relative to the
pre-existing dirty baseline, for 84 lines of growth and no new production file.
A rejected intermediate commit introduced an 86-line timed-search wrapper; the
follow-up consolidation deletes it and is net 36 production lines smaller than
that commit. The production tree remains materially above its 7,500-line target,
so no size or runtime-efficiency claim is made for this correctness milestone.

One fresh-process moving/rotating run was an unfavorable 6.096837-second
outlier versus the wrapper version's 3.663449 seconds. Three immediate retained
code repeats measured 3.501831, 3.620015, and 3.589213 seconds with identical
physics. Their 3.589213-second median does not support a regression, but the
small favorable difference is treated as noise rather than a speedup.

## Unbounded Winding With Lazy Motion Recovery — 2026-09-03

The former `[-1, 1]` winding-component rejection was an explicit completeness
defect, not a validity rule. A 25-edge spiral-chain counterexample has one
collision-free start-to-goal route with winding magnitude two. The old search
returned no route after rejecting that transition; the retained search returns
the class-two route. Ordinary graph reachability is checked first so an
unreachable goal beside a reachable winding cycle terminates with one stored
start state instead of creating an unbounded lifted search.

Eagerly solving every newly visible winding class was rejected. On the extreme
geographic-outline example it found four classes and 177 lifted states, then
raised wall time from a controlled capped 62.296466 seconds to 78.213287
seconds while selecting the exact same 22.0706469074562-degree polyline,
23.3604967801989-degree smooth motion, and 5.80443397354784-second arrival.
The cost was not route search: the returned region's topology time changed
from 1.212179 to 1.595468 seconds, while motion solving changed from 20.300903
to 35.450938 seconds.

The retained design performs the unbounded winding search once, stores higher
winding routes in the existing route set, and solves those routes only if every
ordinary winding candidate and exact motion fails validation. It does not
repeat graph construction, spatial search, or a lower-winding motion solve.
Diagnostics expose both the deferred-route count and whether recovery consumed
it. This scheduling rule prevents a winding restriction from authorizing
`noValidatedSeed`; because the planner already has a finite route-class budget,
it does not claim that a successful bounded run compared every possible winding
class for global objective optimality.

With lazy motion recovery, the same extreme example found four classes and 177
states but left its one multi-winding motion unsolved after an ordinary route
validated. Focused wall time was 61.977703 seconds and the final fresh-matrix
run was 62.704676 seconds, consistent with the controlled capped baseline.
The exact supplied bundle retained its 143.92829584254-degree polyline,
145.143797542061-degree smooth motion, 71.2828117654205-second arrival, and
independent collision, kinematic, and certificate validation; direct replay
took 42.867156 seconds and its four-test suite passed 4/4 in 46.868438 seconds.

The final complete MATLAB suite, including the exact bundle, passed 120/120 in
151.611569 seconds. The final fresh-process example matrix passed 19/19 with
203.090791 seconds summed wall time and exact prior path and arrival values.
Code Analyzer reported zero findings in the five changed MATLAB files. The
milestone adds no production file, wrapper, public option, or dependency.
Relative to the pre-existing dirty baseline it is +98/-46 production lines,
net +52; the working production tree is 19,733 physical lines. The 12,233-line
excess above the 7,500-line target would require a 3,058.25% wall-time reduction
under the documented allowance, which is absent. This is therefore recorded as
a correctness recovery with neutral controlled runtime, not as a size or
speedup claim.

## Complete Input-Derived Time Layers — 2026-09-03

Baseline commit was `a7ef285` on `bmtp-cleanup-codex`, with the documented
user-owned dirty files preserved. The deleted time-layer selector could reduce
105 supplied planning times to 17 uniformly distributed representatives. In a
moving-barrier graph, it dropped the brief opening around 4.0--4.2 seconds and
returned no route; the complete supplied set returned a route arriving at
4.1 seconds. The retained 51-layer regression uses a different horizon and
reaches the same structural opening at 4.0 seconds.

Timed visibility search now keeps every supplied endpoint, obstacle source,
midpoint, and uniform request time. `MaximumTimeLayerCount` remains one option
with one responsibility: it bounds timed BMTP segments plus one. The 62-line
`boundedTimeLayers.m` helper was deleted. No replacement helper, wrapper,
fallback search, public option, or dependency was added. Layer parent indices
use `uint32`, so removing the former 65,535-layer option bound does not create
an index overflow.

A 24-node complete moving-obstacle graph exercised the prior performance
signature. Seventeen layers took 0.847459 seconds and expanded 275 states;
all 41 supplied layers took 0.861912 seconds and expanded 702 states, a
1.017055 wall-time ratio. This single smoke comparison is evidence against a
large regression after batched edge checking, not a speedup claim.

The supplied `Rogue Examples/failed.mat` replay remained physically exact:
143.92829584254 degrees selected polyline, 145.143797542061 degrees smooth
motion, and 71.2828117654205 seconds arrival. Independent validation,
collision, kinematic, and certificate checks passed; wall time was 45.883803
seconds. Its timed search was not invoked, so this is a regression sentinel
rather than benefit evidence.

The complete suite passed 121/121 with no failures or incomplete tests in
154.377006 seconds. The final single-session example matrix passed all 19
expected outcomes in 163.823447 summed planner seconds. Every successful
example passed independent collision and kinematic validation, while
`exampleNoPath` retained its independently validated `noValidatedSeed`
outcome. Physical metrics matched the `a7ef285` benchmark rows. Repeated
fresh-process launches reported a MATLAB startup `File system inconsistency`
before any example code ran; the aggregate is therefore not compared with the
prior fresh-process sum.

Relative to `a7ef285`, production is 59 physical lines smaller. The working
production tree contains 19,674 physical MATLAB lines and 13,845
nonblank/noncomment lines. This milestone removes one production file and one
false-negative heuristic; it makes no performance-based size allowance or
global completeness claim. Finite seed/node budgets, input-derived temporal
discretization, sampled timed-edge screening, and finite solver budgets remain
explicit completeness limits.

## Safe-Wait Arrival Dominance — 2026-09-03

The time-expanded search still had an implicit false-negative rule after time
layer thinning was removed: for each spatial edge it tested only the first
velocity-feasible target layer. A collision at that time discarded the edge,
although a slower traversal could be clear and the source node could be unsafe
to wait at. The retained search tests the first clear arrival in each target
interval connected by verified stationary waits. That arrival dominates later
entries in the same interval because it has the same spatial cost and can
reproduce their state and time by following the already-validated waits.

Enumerating every later target layer was rejected because it nearly doubled
warm scaling-probe time. The retained dominance implementation stays inside
the existing search, batches the same collision predicate, and adds no helper,
wrapper, option, diagnostic field, production file, or dependency. Its claim is
exact only relative to the supplied time layers and existing 13-sample edge
predicate; those discretizations remain explicit completeness limits.

## Background Protection Of Dense Histories — 2026-09-04

The largest measured runtime strength is now dense obstacle construction.
Positive-margin histories with at least 500,000 supplied vertex samples use
MATLAB's six-worker background pool to run the existing independent slice
buffers concurrently. Smaller histories, zero-margin histories, older MATLAB
releases, one-worker installations, and a busy background pool keep the
existing serial path. The change adds no geometry approximation, public
option, wrapper, production file, or toolbox dependency.

On the 716,037-vertex moving/deforming U.S. history, the warmed full-example
median fell from 28.7382835 to 14.5639991 seconds, a 49.321959 percent
reduction. The selected and smoothed paths remained exactly
40.2805679610824 degrees and the independently validated motion duration
remained 7.91666666666667 seconds. A structurally different 524,288-vertex
moving polygon was bit-identical to the serial constructor and improved from
3.7730391 to 3.4153157 seconds in a fresh process, including worker startup.

The main limit is that the automatic threshold was measured only on MATLAB
R2024b Update 4 and an AMD Ryzen 5 3600. Below that scale, worker startup can
cost more than it saves, so the implementation intentionally stays serial.
The improvement reduces obstacle-construction wall time; it does not make the
planner's search or motion solver faster. The retained production diff is
+24/-5 lines, net +19, explicitly accepted for this measured large-history
gain even though the repository remains above its production-size target.

## Exact Static Occupancy Batching — 2026-09-04

Prepared obstacles already prove when a complete history is time invariant.
The shared occupancy query now uses that existing proof to classify all points
inside the obstacle's active span against one cached shape. Moving histories
keep the same time-by-time query path. The loop remains in the existing public
query and adds no heuristic, pruning decision, option, wrapper, helper file, or
dependency.

For the exact `Rogue Examples/failed.mat` request, the warmed planner median
fell from 26.7229846 to 19.8143866 seconds, a 25.852644 percent reduction. The
143.92829584254-degree route, 145.143797542061-degree smooth motion, and
71.2828117654205-second arrival were bit-identical and independently valid.
The fixed-clock screen fell from 7.8687329 to 0.8869675 seconds. Profiling
confirmed the cause: repeated `shapeAtTime` calls fell from 17,074 to 65 and
`pointPolygonClearance` calls fell from 17,047 to 38.

A structurally different static U-shaped example improved from a 10.0607444-
second warmed median to 9.5346848 seconds while retaining exact physical
outputs. A moving-barrier control measured 0.3268389 seconds before and
0.3715701 seconds after; its ranges overlapped, the absolute difference was
0.0447312 seconds, and its dynamic execution path and physical outputs were
unchanged, so no moving-case speedup is claimed. The remaining Rogue cost is
the unchanged BMTP solve, especially its repeated `coneprog` calls.

The retained production hunk is +20/-5 lines, net +15, in one existing file.
One 18-line contract test preserves finite-history and single-sample activity
semantics. The repository remains above its production-size target, so the
performance-based size allowance is not met and is not claimed.
