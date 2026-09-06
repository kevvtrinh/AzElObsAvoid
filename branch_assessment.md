# BMTP branch assessment

## Disconnected HTTP clients and original transform handles - 2026-09-05

Source inspection located an escape path from HTTP response writes inside the
planner cancellation callback. An abandoned health connection could therefore
raise a Java SocketException into an otherwise unrelated active plan. The
response writer now contains SocketException failures and reports an
undeliverable response; other exceptions propagate. A previously accepted
cancel request retains its true return value even if its acknowledgement fails.
This matches the reported write-error class; the user's complete scenario was
not reproduced. No planner algorithm or validation rules changed.

MATLAB R2024b passed all four tests in testOfflineSandboxTransport, including a
real reset peer, a closed socket, polling after abandonment with subsequent
health/cancel requests, and propagation of non-socket errors. MATLAB initially
failed to initialize in the restricted environment; the authorized unrestricted
run completed successfully. No maintained planner examples were executed.

Original obstacles now expose resize and rotation handles immediately after
creation, alongside direct body dragging. Out-of-workspace transforms are
refused. All 22 Node sandbox tests pass, including original-shape resizing and
rotation. Browser visual verification remains unavailable under the previously
reported local-file policy restriction.


## HTML sandbox final-pose editing - 2026-09-05

Added Delete to the selected polygon's floating menu. Set motion now exposes a
mission-end ghost with direct body/corner/rotation-handle editing, a finish
checkmark, and collapsed numeric controls in the sidebar. No transform-tool
buttons or top pose bar remain. A local obstacle preview animates without
endpoints, a result, or MATLAB and shows only untraveled centroid paths and retains final-pose
ghosts. Ending preview restores editing and preserves any existing planner result. Copy retains final-pose settings;
Make stationary resets them. Initial geometry is preserved. Translation keeps
the existing center-speed limit; angular and scale motion are separate inputs.
The handoff schema is unchanged: the browser samples transformed vertices and
uses matching piecewise linear interpolation for request preview. Angular steps
are at most 5 degrees. This is sampled polygon motion, not exact rigid arcs;
MATLAB's existing correspondence/enclosure contract still applies.

The obstacle-speed control is now a 0–10 deg/s slider with a live readout inside
the canvas for the selected editable obstacle;
exact components are collapsed and accept -10 to 10 deg/s. Slider, vector,
and destination checks use the 10 deg/s magnitude limit requested by the user. It retains direction through zero and preserves
the selected profile. New motion defaults to +Az when no direction was set.

The Rectangle tool (R) creates ordinary four-vertex polygons by dragging
opposite corners; drafts are bounded to the workspace and zero-area drafts
are refused. Escape and pointer cancellation discard the rectangle draft.

Verification: 20 focused Node tests pass for distinct convex/concave shapes,
rotation-only and combined motion, stretch in rotated axes, angle-wrap handling,
preview/export consistency across translation profiles, edit locks, invalid
inputs, copy independence, ghost dragging, playback-time selection, deletion,
icon/menu structure, offline preview advancement, direct handle hit testing,
remaining preview paths (including reverse scrubbing) and persistent final
ghosts, returning to editing, and speed-slider
direction retention, bounds, edit locks, canvas-overlay placement, and
rectangle drawing in all four directions with degenerate rejection. JavaScript syntax and
static markup checks pass. Visual browser interaction and MATLAB integration
remain unverified; browser policy previously blocked the local-file preview.
No MATLAB examples were run and benchmark.csv has no new rows for this change.


## HTML sandbox visual refresh - 2026-09-05

Refreshed the standalone HTML sandbox with light surfaces and canvas, stronger
plot colors, larger controls, a three-step orientation guide, explicit offline
command numbering, and collapsed workspace bounds. Planner behavior and
request/result interfaces are unchanged. Verification: JavaScript syntax,
existing element IDs and script behavior equivalence apart from color literals,
and HTML nesting checked statically. Browser visual and interaction checks
remain unverified because browser policy blocked the local file URL. No MATLAB
examples were executed and no benchmark rows were added for this UI change.


## Adopted degree-eight MATLAB configuration - 2026-09-05

The user ended the solver research and selected uniform degree eight with
MATLAB only. Production now uses degree-eight Bezier curves for ordinary,
timed and conservatively grouped conic requests, retaining the original span
allocations of three and one respectively. Trajectory and separating-plane
programs use MATLAB coneprog. The superseded fastcone package and its tests,
documentation and benchmark entry points are removed; the native hybrid
prototype, portable build tools and binaries are also removed. No external MEX
solver is required. Geometry, search horizons, physical limits, numerical
tolerances and the independent validator are unchanged.

This decision supersedes the open research plans and adoption gates in the
dated history below. Research is closed at the user's request; a 3x cold-start
speedup was not demonstrated. The accepted degree reduction trades some motion
quality for runtime, as measured and disclosed before adoption.

The initial adoption matched the previously executed degree-eight candidate
file for file, except for explanatory comments in createSolveRequest.m. That
18-example fresh-process suite passed all 17 feasible examples and preserved
the expected noValidatedSeed outcome. All successful collision, kinematic and
continuous checks passed, including the applicable plane certificates. Across
the eight examples that called coneprog, cold example time totaled 63.9495 s
versus 105.2679 s for the original degree-16/7 coneprog configuration: 1.646x
observed speedup, or 39.25% less time. These were single fresh passes rather
than interleaved medians; setup and validation are included and MATLAB process
launch is excluded. Nonconic examples are excluded from the speedup claim.

The accepted differences include +0.0107931 s arrival for ObstacleAvoidance,
+0.0176447 s for StaticU, and +0.2531241 s for TwoOpposingU. At fixed arrival,
Occlusion uses +0.0117515 deg travel and TargetExits uses +0.0750785 deg.
All remain independently valid. The unchanged failed.mat replay took
40.1713 s versus 66.6826 s originally and selected identical motion; subsequent
degree-eight controls took 41.7647 and 35.7660 s. The user-owned bundle remains
untouched, SHA256 E754CF5D4C1ECA3B5B50865DDCFBC9673A64CC252615D6521A062A75B18C9625.

Initial production verification passed 123/126 tests. One tight 21 s static
recovery fixture failed: its third seed retained a collision-free 23.7150 s
iterate, then coneprog rejected the next fixed-plane problem at 21 s and the
alternation stopped. This is a motion-construction horizon transition failure,
not evidence that the route is infeasible. Two timed tests instead assumed a
particular method even though the public planner found valid static-projection
motion against the full moving scene.

Bounded correction: when the deadline-constrained fixed-plane
subproblem is infeasible and a longer collision-free iterate is retained,
continue alternating at that retained duration inside the existing iteration
budget. Reimpose the original deadline for the next trial; final acceptance
still requires the original horizon and independent validation. This permits
the separating planes to improve instead of treating a local convex problem
as a proof of route infeasibility. RetainedHorizonRetryCount exposes these
extra attempts; conic counts and timing include all their work. The unchanged
21 s fixture now succeeds and independently validates at 20.8537 s on seed 2,
so no later-seed recovery is necessary. Its regression now requires this
earlier success with both MaximumSeedCount=2 and 5 and still checks the original
deadline. The static-box contract also passed. All three timed tests passed
after exercising the timed kernel directly, preserving their full-resolution
coverage and independent validation assertions while allowing the public
planner to retain an earlier valid static-projection motion. Focused result:
5/5 passed. The complete production regression rerun then passed 126/126 tests
with zero failures or incomplete tests. Its three maintained headless example
invocations also passed extra independent validation and were appended to
benchmark.csv, with NaN wall time because those individual test calls were not
separately timed. Their metrics match the preceding test pass. Temporary
capture calls were removed from maintained tests after recording the results.

Final cold recheck: all eight conic examples retained their expected outcomes,
and their polyline length, smoothed length and motion duration exactly match
the initial degree-eight CSV measurements. All seven successful examples
passed independent collision, kinematic, continuous and plane checks; NoPath
remained the expected noValidatedSeed. Total fresh example time was 77.3941 s
versus the recorded original 105.2679 s, or 1.360x observed speedup. This repeat
was slower than the initial degree-eight pass (63.9495 s, 1.646x); neither
single-pass ratio is a repeated median or evidence of 3x performance.

| Conic example | Final cold wall (s) | Polyline (deg) | Smoothed (deg) | Motion (s) |
|---|---:|---:|---:|---:|
| AlternatingSlalom | 8.8033652 | 16.0193197983 | 16.3388514176 | 10.5409567875 |
| NoPath | 3.7854072 | NaN | NaN | NaN |
| ObstacleAvoidance | 5.9834265 | 11.1521195190 | 11.4406845062 | 7.5646814987 |
| StaticUShapedObstacle | 10.5738140 | 34.9425880405 | 39.3840155922 | 20.8500067174 |
| StraightTargetAlternatingOcclusion | 7.8729552 | 13.3416640641 | 13.6104156607 | 20.8695652174 |
| TargetExitsObstacle | 9.1607790 | 20.1357890335 | 20.6851467568 | 24.0000000000 |
| TwoOpposingUVisibilityGraph | 5.8933688 | 24.0357847150 | 24.7050424912 | 22.1006280522 |
| USOutlineExtremeVisibility | 25.3209624 | 22.0706469075 | 23.3542523251 | 5.7964986754 |

The final unchanged failed.mat replay took 51.8916 s and passed independently
with identical selected fixedClockLateralExcursion motion: polyline and
smoothed length 143.444156590 deg, duration 69.062225080 s, goalReached.
Collision, kinematic and continuous checks passed; plane certification was
not applicable to the selected motion. It executed degree-eight conic work.
All cold checks used fresh MATLAB R2024b processes, seed 0 and six computational
threads. Example wall times include setup and extra independent validation;
the saved bundle covers planning and extra validation. Process launch is
excluded. The eight actual maintained examples were appended to benchmark.csv;
the saved-request replay was not appended as an example invocation.

Benchmark scripts and measurement artifacts remain under ignored output and
are not source changes. The user reviewed the 41-file change set and authorized
its commit and push. The user-owned modified failed.mat is excluded.

The native hybrid screen completed before the MATLAB-only steering was
processed: StaticU baseline 19.6839123 s, hybrid 11.7619320 s. Both passed
planner and independent collision, kinematic, continuous and plane checks,
with polyline 34.9425880405 deg. Baseline smoothed length/duration were
39.3787713567 deg / 20.8323620005 s; hybrid values were 39.3957010744 deg /
20.8368376206 s. Both ended goalReached, but the hybrid failed its original
arrival-quality gate. It was not adopted or tested further. These were saved
request replays, not maintained-example invocations; their evidence is preserved
under ignored output and adds no benchmark.csv example rows.

## Historical research record

The following entries preserve the decisions and measurements at their dates;
references to ongoing research or the original degree-16/7 production baseline
are superseded by the adoption decision above.

## Parametric-duration LP outcome - 2026-09-05

The latest goal turn made progress through a measured rejection. A fixed-plane
duration-root formulation, using MATLAB's HiGHS LP solver with dual updates,
was tested on the original degree-16 StaticU planner request. Fresh cold
baseline 20.6156600 s; candidate 23.4968329 s. The candidate accepted one of
14 trajectory replacements, fell back to coneprog 13 times and spent 3.1987 s
in 20 LP calls. It was 13.98% slower and its isolated source was removed.
Fresh recovery took 19.4847235 s and retained exactly the baseline result.
All three runs passed planner and independent collision, kinematic, continuous
and plane-certificate checks: polyline 34.9425880405 deg, smoothed
39.3787713567 deg, duration 20.8323620005 s, goalReached. The full hypothesis,
gate and diagnostic limitations are recorded at the end of this assessment.

These were saved-request replays; no maintained-example rows were appended.
The unchanged failed.mat hash was reverified, production stayed on the
uncommitted coneprog reset, and git diff --check passed. No broad tests or
commit were justified by this rejected experiment. The original eight-case
3x cold-start goal remains active and unproven.

## Residual planner-cost investigation - 2026-09-05

The previous goal turn made progress: 19 cold degree-eight runs completed,
quality changes were measured and the executed degree was verified. The 3x
goal remains active and unproven. Current worktree inspection confirms the
production coneprog reset and existing dirty user bundle; no active MATLAB job
is being restarted. Lower-degree models remain isolated user-requested research.

Investigation completed: the failed bundle's dominant measured owner was timed
search occupancy, while U.S. planner time was mainly motion solving. A single
prepared-boundary boolean-query prototype was slower (49.9830 s versus a fresh
41.7647 s degree-eight control) and was removed immediately. The recovery control
then succeeded in 35.7660 s with identical motion and independent checks. This
also demonstrates material cold-run variation: successive degree-eight bundle
controls were 40.1713, 41.7647 and 35.7660 s. No new algorithm was retained or
committed. Details and the negative evidence are recorded below; the original
eight-case 3x goal still requires a complete qualified candidate.

Before another algorithm change, inspect residual cost in the degree-eight
planner on the unchanged failed bundle and the U.S. saved request. Existing
timers separate U.S. full-example wall 20.9108 s from planner 7.0408 s and extra
validation 0.1822 s; setup is not solver time. Profile fresh first planner calls
for ownership only, with original geometry and tolerances and a 300 s profiling
cancellation guard. Report inclusive/self/call counts without summing nested
timers. Save profiles and stage/seed attribution under ignored output; profiled
timings are not speedup measurements. These are saved-request replays and do
not append maintained-example benchmark rows. Choose one next bounded
mechanism from the measured owner, not another unrequested degree sweep.

## Degree-eight cold suite requested - 2026-09-05

The user explicitly requested degree eight across the entire example suite.
Run all 18 maintained examples and the previously requested unchanged failed.mat
replay in separate fresh MATLAB R2024b processes, seed 0, six computational
threads, headless controls and original physical/numerical tolerances. This
user-authorized degree comparison is isolated under output/cold-method-research-
20260905/degree8-planner. Production remains HEAD 3d60f83 plus the uncommitted
coneprog reset; preserve all existing dirty work and the unchanged user bundle.

Use degree eight for every conic trajectory request: ordinary degree 16 becomes
8, and timed/grouped degree 7 becomes 8. Keep original span allocations (three
for ordinary and one for timed/grouped models), horizon, seed search, coneprog
options and complete public validation unchanged. No coarse initialization is
added. Verify executed diagnostic degrees after timing. This uniform degree-
eight interpretation is explicit; prior 5->10 retained timed/grouped degree 7.

Measure the same cold full-example scope as the preceding degree-seven pass,
including setup, example checks and extra public validation; record MATLAB
launch separately. The bundle timer covers planner plus public validation after
loading the unchanged bundle, with only its unavailable cancellation callback
cleared identically to prior comparisons. Compare with existing fresh original,
5->10 and degree-seven measurements, disclose quality/failure results and do not
claim repeated-median qualification from this single pass. Append actual
maintained runs to benchmark.csv and retain unfavorable measurement history.
No commit or production adoption before the complete 3x cold retention gate.

Completed 19 fresh processes: 18 maintained examples and the exact bundle.
The suite returned 17 independently valid successes and the expected
noValidatedSeed failure. Every successful run passed collision, kinematic and
continuous checks; all successful conic examples passed plane certification.
Actual engine diagnostics confirmed degree 8 for every executed conic model.
The ten non-conic examples retained their original motion results and did not
call coneprog. Their plane certificates are not applicable. All 18 maintained
invocations were appended to benchmark.csv; the bundle replay was saved apart.

| Conic example | Original cold s | Degree 7 cold s | Degree 8 cold s | Observed degree 8 speedup vs original |
| --- | ---: | ---: | ---: | ---: |
| AlternatingSlalom | 10.8050191 | 7.2534432 | 5.9829056 | 1.806 |
| NoPath | 4.6713839 | 4.3233119 | 3.1909797 | 1.464 |
| ObstacleAvoidance | 6.6057899 | 5.0450972 | 4.3681097 | 1.512 |
| StaticUShapedObstacle | 20.9892007 | 9.2528887 | 10.3848825 | 2.021 |
| StraightTargetAlternatingOcclusion | 10.1097185 | 5.5816085 | 5.6431306 | 1.791 |
| TargetExitsObstacle | 14.7640674 | 7.0798060 | 7.7364299 | 1.908 |
| TwoOpposingUVisibilityGraph | 9.3890009 | 6.2946245 | 5.7322623 | 1.638 |
| USOutlineExtremeVisibility | 27.9337322 | 22.9480395 | 20.9108075 | 1.336 |

Degree-eight sum 63.9495078 s versus original 105.2679126 and degree seven
67.7788195 s: observed 1.6461x versus original and 1.0599x versus degree seven
(5.6497% less runtime). These compare successive single cold passes, not
interleaved repeats or medians. Unchanged non-conic cases also shifted in runtime
(for example DenseConcave 2.8271 -> 2.0145 s between the earlier 5->10 and current
passes). Thus the small aggregate advantage over degree seven is tentative and
must not be attributed wholly to the polynomial degree without repeated controls.

There are five strict objective regressions versus the original, all smaller
than the corresponding degree-seven regression: ObstacleAvoidance motion
7.5646814987 s (+0.0107930717), StaticU 20.8500067174 s (+0.0176447169),
Opposing U 22.1006280522 s (+0.2531241011). Fixed-arrival smoothed travel is
13.6104156607 deg for Alternating Occlusion (+0.0117515220) and
20.6851467568 deg for Target Exits (+0.0750785484). All are physically valid;
these differences do not meet the original objective tolerance. The ordinary
polylines stay unchanged. The grouped U.S. model really ran degree eight and
improved motion from 5.8044339735 to 5.7964986754 s and smoothed length from
23.3604967802 to 23.3542523251 deg.

The unchanged failed.mat replay took 40.1712728 s, versus preceding original
66.6826051 s, 5->10 59.4638917 s and degree seven 45.7997495 s. All returned
the identical independently validated fixed-clock excursion: polyline and
smoothed length 143.444156590 deg, duration 69.0622250800 s, goalReached,
collision/kinematic/continuous checks passed and plane certification not
applicable to the selected motion. Conic work was executed and its degree-eight
diagnostics were verified. Source SHA256 stayed
E754CF5D4C1ECA3B5B50865DDCFBC9673A64CC252615D6521A062A75B18C9625.

The package audit confirmed only the two degree assignments differ from the
production engine; every other engine source file matches it. No production
algorithm was changed or committed. Full measurements are retained in ignored
degree-eight-comparison.csv, degree-eight-suite-results.csv,
degree-eight-bundle-comparison.csv and degree-eight-summary.json. The requested
comparison is complete; the 3x cold adoption goal remains unachieved.

## Degree-seven and unchanged diagnosis-bundle comparison requested - 2026-09-05

After the completed 5->10 full-example pass, the user requested comparison
with degree seven alone and specifically requested both methods on the failed
case. Interpret that as the supplied Rogue Examples/failed.mat, whose SHA256
was rechecked unchanged: E754CF5D4C1ECA3B5B50865DDCFBC9673A64CC252615D6521A062A75B18C9625.
Run three fresh planner-plus-public-validator bundle replays: original
coneprog, 5->10 and degree seven alone, without the older replay helper's
solver/property warm-up. Clear only the unavailable cancellation callback,
identically for all methods. Do not rewrite, resave, stage or regenerate the
bundle. Its expected success requires independent physical validation.

Also run the eight original conic-using maintained examples with degree seven
alone in fresh MATLAB processes. Compare with the just-completed original and
5->10 full-example runs, preserving their timing scope and data. Degree seven
alone changes ordinary degree 16 to 7 and retains three spans per edge; original
timed/grouped degree-seven models remain unchanged. All solvers remain coneprog.
The previous specific StaticU degree-seven arrival tradeoff stays accepted;
other quality differences are measured and disclosed. User-authorized research
is isolated in output; no production adoption or commit is earned by a single
screen. Append actual maintained invocations to benchmark.csv; bundle replays
are saved separately. Pending results do not complete the 3x cold goal.

Completed the 11 fresh processes. The exact unchanged bundle succeeded and
independently validated with all three methods. Its selected fixed-clock
excursion was identical in every run: polyline and smoothed length
143.444156590 deg, motion 69.0622250800 s, goalReached. Collision, kinematic
and continuous checks passed; plane certification was not applicable to the
selected motion. Conic attempts were present in all three planner diagnostics.

| Unchanged failed.mat replay | Cold planner + validation s | Speedup vs original |
| --- | ---: | ---: |
| Original coneprog | 66.6826051 | 1.000 |
| 5 -> 10 | 59.4638917 | 1.121 |
| Degree 7 alone | 45.7997495 | 1.456 |

Degree seven used 22.98% less runtime than 5->10 on this bundle, with identical
selected motion quality and full validation. This compares with the current
coneprog reset, not the historical 90-minute run. Source bundle SHA256 remained
unchanged after all measurements. Bundle replays did not append benchmark.csv.

| Maintained conic example | Original cold s | 5->10 cold s | Degree 7 cold s | Degree 7 speedup vs original |
| --- | ---: | ---: | ---: | ---: |
| AlternatingSlalom | 10.8050191 | 7.7762545 | 7.2534432 | 1.490 |
| NoPath | 4.6713839 | 4.7029084 | 4.3233119 | 1.081 |
| ObstacleAvoidance | 6.6057899 | 6.1148164 | 5.0450972 | 1.309 |
| StaticUShapedObstacle | 20.9892007 | 11.0482246 | 9.2528887 | 2.268 |
| StraightTargetAlternatingOcclusion | 10.1097185 | 7.2571064 | 5.5816085 | 1.811 |
| TargetExitsObstacle | 14.7640674 | 11.8029039 | 7.0798060 | 2.085 |
| TwoOpposingUVisibilityGraph | 9.3890009 | 9.4575299 | 6.2946245 | 1.492 |
| USOutlineExtremeVisibility | 27.9337322 | 26.6727321 | 22.9480395 | 1.217 |

All seven degree-seven successes independently validated with collision,
kinematic, continuous and plane checks passing; the expected NoPath returned
noValidatedSeed. Degree-seven sum 67.7788195 s versus original 105.2679126 and
5->10 84.8324762 s: 1.5531x versus original and 1.2516x versus multilevel
(20.1027% less runtime). These are single cold example trials, with setup and
example checks included, not repeated medians. The ten previously non-conic
examples were not rerun for degree seven; the affected conic cohort is explicit.

Degree seven has five strict quality regressions relative to the original,
including the already accepted StaticU +0.0573907035 s tradeoff. Additional
ones are ObstacleAvoidance +0.0146854645 s and Opposing U +0.4264076967 s;
fixed-arrival smoothed travel rises 13.5986641387 -> 13.6126705616 deg for
Alternating Occlusion and 20.6100682085 -> 20.7250188732 deg for Target Exits.
Opposing U smoothed length is 24.7138421607 deg, compared with original
24.6652397599 deg. These are objective changes, not physical validation failures.
No quality waiver is inferred for the additional cases. Full precision, flags
and paths are preserved in ignored degree-seven-comparison.csv,
degree-seven-summary.json and failed-bundle-comparison.csv. The eight actual
maintained degree-seven invocations were appended to benchmark.csv.

Neither 5->10 nor degree seven alone earns the requested 3x cold adoption or a
commit. Production remains the uncommitted coneprog reset. The user-requested
comparison is complete; the broader optimization goal remains active. No new
production algorithm or tests were introduced by this comparison.

## Full cold maintained-example pass requested - 2026-09-05

The user requested a quick cold pass of the 5->10 candidate through all
examples. Run each of the 18 maintained example functions in its own fresh
MATLAB R2024b process, headless with default finite jerk limits and seed 0.
Run matching original coneprog examples for the eight original conic-using
cases, alternating method order between cases. One trial per method only;
do not call this repeated-median qualification. Keep numerical and physical
validation unchanged and continue recording unfavorable cases. No concurrent
MATLAB benchmarking. The isolated degree-five-to-ten source is frozen during
this pass; its timed/grouped degree-seven behavior stays original.

Unlike prior saved-request probes, these execute actual example functions.
Primary reported cold wall time includes example construction and checks plus
an additional independent public validation; process launch is recorded
separately. Also retain the planner's internal elapsed time and the additional
validation time for attribution. Do not directly equate the new full-example
times to prior planner-only probes. Append every actual invocation to the
existing benchmark.csv schema with the method and timing scope in Notes.
Research scripts and MAT/CSV/log artifacts stay ignored under output. Pending
results do not earn a commit or complete the full 3x cold goal.

Completed all 26 fresh processes: 18 candidate maintained examples and eight
matched original conic examples. Candidate results were 17 independently valid
successes and the expected noValidatedSeed failure. Every success passed
collision, kinematic and continuous checks; conic-selected motions passed plane
certification. The ten original non-conic examples stayed non-conic and matched
their original motion metrics. Actual invocations were appended to benchmark.csv.

| Original conic example | Original cold s | 5->10 cold s | Observed speedup |
| --- | ---: | ---: | ---: |
| AlternatingSlalom | 10.8050191 | 7.7762545 | 1.389 |
| NoPath | 4.6713839 | 4.7029084 | 0.993 |
| ObstacleAvoidance | 6.6057899 | 6.1148164 | 1.080 |
| StaticUShapedObstacle | 20.9892007 | 11.0482246 | 1.900 |
| StraightTargetAlternatingOcclusion | 10.1097185 | 7.2571064 | 1.393 |
| TargetExitsObstacle | 14.7640674 | 11.8029039 | 1.251 |
| TwoOpposingUVisibilityGraph | 9.3890009 | 9.4575299 | 0.993 |
| USOutlineExtremeVisibility | 27.9337322 | 26.6727321 | 1.047 |

Sum 105.2679126 -> 84.8324762 s: 1.2409x, a 19.4128% reduction. Internal
planner time plus the separately timed extra validation summed 78.1832665 ->
60.1936654 s (1.2989x); this is attribution and does not replace the externally
timed full-example result. Single probes, not repeated medians; near-unity
differences are not established improvements. The requested 3x gate is unmet.

Three quality regressions must remain visible: fixed-arrival Alternating
Occlusion kept duration 20.8695652174 s but smoothed travel rose
13.5986641387 -> 13.6101924710 deg; fixed-arrival Target Exits kept 24 s but
travel rose 20.6100682085 -> 20.6369503298 deg. Opposing U motion duration rose
21.8475039511 -> 21.9346437965 s (+0.0871398454 s), despite its shorter path
24.6652397599 -> 24.3886482215 deg and essentially unchanged runtime. These
do not pass the original quality gate. No safety, numerical or geometric
tolerance was weakened. The candidate remains unadopted; production is the
unchanged requested coneprog reset. Full measurements are in ignored
all-cold-example-comparison.csv, all-cold-example-results.csv and
all-cold-example-summary.json. The user has now requested the separate
degree-seven and exact-bundle comparison recorded above.

## User-requested lower-degree coarse-to-fine comparison - 2026-09-05

The user explicitly requested coarse degree 4 / fine degree 10 and other
degrees; the fine model no longer needs to return to degree 16. Run the bounded
grid 4->10, 5->10, 7->10 on StaticU and Slalom, with fresh MATLAB processes and
all original physical checks. This explicit request authorizes the degree
comparison. Preserve original input geometry, route search and numerical
tolerances. Compare against original degree-16 coneprog and disclose motion
quality changes, coarse failures and all initialization costs. No prior-request
numerical warm start. No production adoption or commit before the complete
runtime/correctness gate. The accepted degree-seven StaticU tradeoff persists.

The first 7->16 screen failed the runtime gate on StaticU: original 21.2820108 s,
multilevel 23.7845558 s, identical independently valid 20.8323620005 s motion
and 34.9425880405 / 39.3787713567 deg lengths. Its direct coarse proposal was
unavailable (0.5393 s); the detour's transferred separator failed verification
(1.7500 s), so the original initialization was correctly retained. Slalom's
transfer applied: 11.2000020 -> 8.3094041 s, motion 10.6126605873 ->
10.6136435510 s (within original 1 ms tolerance), polyline 16.0193197983 deg,
smooth 16.2829658411 -> 16.4208284859 deg. Both methods passed independent
collision, kinematic, continuous and plane checks, goalReached. These are
single cold probes, not adoption proof. Exact transfer passed 12 deterministic
mesh cases with maximum position-through-jerk discrepancy 2.4568e-10.

The lower-degree engine and request harness remain isolated under ignored
output/cold-method-research-20260905. Degree four cannot represent a nonzero
rest-to-rest move with one polynomial span. Give its single-edge coarse request
three spans (matching the fine mesh), and use finite route-shaped quartic
initial controls solely as a proposal; the coarse optimizer still imposes
endpoint position/velocity/acceleration and C3 knot continuity. This avoids the
original degree>=5 initialization formula's division by zero without relaxing
any motion acceptance condition. Other coarse requests keep one span per true
route edge; fine requests keep three. Timed and grouped compact models retain
their original degree-seven representation.

The requested degree grid passed both saved requests in every variant. Before
retaining coarse-stage complexity, measure a degree-ten-only control on the
same two requests in fresh MATLAB processes. It uses the identical final
engine and original initialization, bypassing only the coarse proposal. This
control attributes the benefit between a smaller final model and multilevel
initialization; it does not change physical or numerical acceptance.

Measured results (first planner call plus additional public validation in each
fresh MATLAB R2024b process, six computational threads, seed 0; process launch
excluded). Every row passed planning, independent validation, collision,
kinematics, continuous resolution and plane certification, with goalReached.
Polyline lengths stayed 34.9425880405 deg for StaticU and 16.0193197983 deg for
Slalom. These are saved-request replays, not maintained-example invocations;
benchmark.csv was not appended. No repeated-median adoption claim is made.

| Case | Degree method | Cold s | Speedup | Motion s | Smoothed deg |
| --- | --- | ---: | ---: | ---: | ---: |
| StaticU | original 16 | 21.7026776 | 1.000 | 20.8323620005 | 39.3787713567 |
| StaticU | 4 -> 10 | 15.8018256 | 1.373 | 20.8281223123 | 39.5557247674 |
| StaticU | 5 -> 10 | 10.8403515 | 2.002 | 20.8289361627 | 39.5973651055 |
| StaticU | 7 -> 10 | 16.1112293 | 1.347 | 20.8281223123 | 39.5557247674 |
| StaticU | 10 alone | 14.0707393 | 1.542 | 20.8281223123 | 39.5557247674 |
| Slalom | original 16 | 9.6059765 | 1.000 | 10.6126605873 | 16.2829658411 |
| Slalom | 4 -> 10 | 9.1067265 | 1.055 | 10.5555866339 | 16.2630920826 |
| Slalom | 5 -> 10 | 7.8179461 | 1.229 | 10.5562283432 | 16.4591331378 |
| Slalom | 7 -> 10 | 7.8959849 | 1.217 | 10.5914111948 | 16.4432921567 |
| Slalom | 10 alone | 8.5138105 | 1.128 | 10.5787785107 | 16.3341253044 |

Degree-five initialization is the strongest measured candidate on this screen,
though its Slalom timing is effectively tied with degree seven at this sample
count. Its transfer applied to each selected detour. StaticU's expensive final
degree-ten trajectory solves fell from 12 to 4 on the selected seed; Slalom's
fell from 5 to 2. Total coarse work across both attempted seeds was 1.3112 s
and 1.1742 s respectively, included in wall time. Each direct-route coarse
attempt failed and retained original initialization. StaticU degrees four and
seven also rejected the detour's unverified transferred separator, so those
variants paid coarse work and then used the degree-ten-only initialization.
No unsupported separator or sampled-only collision result authorized success.

The ratio of summed two-case cold times is about 1.68x for 5->10 against the
original planner, and 1.21x against degree ten alone. The requested 3x full
eight-case, repeated-cold gate remains unmet and unproven. The shorter measured
motion durations mean these two cases do not need the user's previously
accepted degree-seven arrival tradeoff; their longer smoothed lengths remain
disclosed. Keep only the 5->10 prototype and degree-ten control for further
qualification. Removed the quartic-only initialization path, unsupported degree
selectors and obsolete experiment runners; preserved measured MAT/CSV/log
evidence, including unfavorable coarse failures. Production is still the
requested coneprog reset, uncommitted. No push. The unchanged user bundle hash
was rechecked and git diff --check passed. Transfer verification for the lower
degrees passed 18 deterministic meshes, maximum discrepancy 7.0429e-11 through
jerk. No new production tests were run for these isolated experiments.

## Coarse-to-fine planner experiment - 2026-09-05

User steering: if coarse-to-fine does not work, the measured degree-seven
StaticU tradeoff is acceptable: 20.8897527040 s motion instead of 20.8323620005 s
(+0.0573907035 s), with the observed approximately halved cold runtime and
unchanged independent physical validation. This conditional acceptance persists.
Do not reject that specific result again solely for its arrival difference.
Other cases' quality changes must still be measured and disclosed. The user
has not waived full physical validation or the overall 3x cold-runtime target.

The preceding goal turn made progress: analytical whole-planner shortcuts were
measured, native status disagreements were diagnosed, and a faster but lower-
quality curve representation was rejected and removed. Production remains the
uncommitted coneprog reset at HEAD 3d60f833e9d72baef2008c0a34548e8a66e17c59 plus
the previously recorded reset changes. The user-owned failed.mat is preserved.

Bounded hypothesis: compute a degree-seven proposal with one span per true
route edge, then exactly subdivide and elevate it into the original degree-16,
three-span representation. Use the transferred curve and directly verified
separator constraints to initialize the original alternating solve. The coarse
result is a proposal only; it cannot authorize success, discard a seed, shorten
the horizon, or replace final original-model optimization and public validation.
If coarse construction or exact nested-mesh transfer is unavailable, retain the
original seed initialization and record all additional work. No prior planner
call supplies an initial solution, cached factorization or cached trajectory.

Acceptance remains >=3x summed cold case medians across all eight original
conic-using requests, no >10% case slowdown, original arrival/travel objective
and full independent physical validation. All initialization, transfer and
fallback work is inside the timed first planner call. First verify subdivision
and degree elevation preserve position through jerk on deterministic different
meshes. Then one focused cold StaticU comparison and a structurally different
Slalom check. Reject on a quality/validity failure or absent useful runtime
benefit, without a degree sweep. Owned code is only the ignored multilevel-
planner package and transfer/test helpers under output/cold-method-research-
20260905. No commit is earned before the complete gate, including maintained
examples, regressions and the exact unchanged diagnosis bundle.

## Analytical fixed-clock research continuation - 2026-09-05

The previous answer-only goal turn made no experimental progress. Current
worktree inspection confirms production remains the uncommitted coneprog reset;
the user-owned diagnosis bundle is excluded from this work.

An isolated constant-jerk outward/return lobe on a stationary coordinate can
preserve the direct motion's certified componentwise time floor. Scaling one
feasible jerk word reduces static obstacle bounding rectangles to forbidden
amplitude intervals; the public continuous validator still approves every
returned motion against the original protected geometry. This is a C2 analytic
motion proposal, not a claim that all C3 BMTP problems have closed forms.

The first amplitude-equation cold ObstacleAvoidance probe took 2.2937787 s
versus 8.2766373 s for a separate matching neutral-directory coneprog probe
(3.608x single pair). It passed planning, independent collision, kinematic,
and continuous validation; plane certificates are not applicable. Polyline
and sampled smoothed length were 11.334098922 deg, motion 7.500000000 s,
goalReached. The reference retained 11.152119519 / 11.430861536 deg and
7.553888427 s. This is promising screening evidence, not repeated adoption
proof. Slalom fell back to BMTP in 13.3832024 s, preserving validated lengths
16.019319798 / 16.282965841 deg and duration 10.612660587 s. A matching cold
reference is being measured; historical timings are not used for its ratio.

Before extending the experiment, restore the original spline proposal loop
after the new analytic attempt, including when a constructed jerk lobe fails
validation. No original proposal family is to be removed. All sources remain
ignored under output/cold-method-research-20260905.

Next bounded hypothesis: a static protected polygon can certify a completely
occupied straight cut across the allowed workspace. If the endpoints lie on
opposite sides, continuity rules out every workspace-contained trajectory.
Prove the complete cut lies strictly inside one actual polygon ring using
oriented edge halfspaces plus an interior anchor, never a bounding box. Require
the obstacle to remain active for the entire request horizon. The same proof
on one half-workspace can skip only new one-sided jerk proposals while leaving
the original spline attempts intact. Focused cases: NoPath and Slalom; verify
different cut orientations, a concave non-cut, and inactive/moving rejection.
Reject the isolated proof if any invalid certificate or useful-runtime failure
occurs. The full eight-request cold 3x gate remains unchanged; neither this
certificate nor one successful analytical example completes the goal.

The workspace-cut prototype passed ten deterministic geometry gates (including
both orientations, a slanted ring, misleading concavity, finite inactivity,
motion, and workspace roundoff). The saved NoPath request retained recognized
failure diagnostics with zero attempted seeds and was plotted successfully.
Its first cold pair was 5.3753935 s coneprog versus 1.4874787 s with the cut.
Both had no returned motion, NaN motion metrics, and noValidatedSeed; public
motion-validation flags are false because there is no trajectory. The new
record separately exposes the static infeasibility certificate.

After restoring all original spline attempts and using half-workspace cuts
only to skip impossible new one-sided jerk proposals, cold Slalom took
9.2850546 s versus its preceding neutral-directory baseline 11.2610543 s.
Both retained the exact independently validated motion reported above.
ObstacleAvoidance took 2.3100406 s, again independently valid at 7.5 s and
11.334098922 deg. These are isolated screening probes, not interleaved repeated
medians. The benchmark harness now checks the original travel objective for
fixed arrival and travel-plus-time for balanced arrival, in addition to time
and physical checks. It records executed conic diagnostics and selected source.

Next independent solver hypothesis: test Clarabel's native primal-dual conic
method on the captured original trajectory models. Its C API can be called
from a small MATLAB MEX gateway without a Python runtime. Source reference:
https://github.com/oxfordcontrol/Clarabel.cpp (Apache-2.0). Start with the six
ObstacleAvoidance programs and the fourteen structurally different StaticU
programs, retaining all statuses, original matrix residuals, and objectives.
Only a passing numerical screen permits full-planner integration. No relaxed
tolerances or unreported solver recovery; the existing full cold retention
gate still applies. All toolchain, source, binaries and scripts are owned by
the ignored research output directory, and production remains coneprog.

Clarabel's first program passed in 0.076891 s versus coneprog's 0.370060 s.
The second returned primal infeasible while coneprog reported success, so the
initial status-matching screen stopped and no planner integration occurred.
Postmortem evidence changes the interpretation: the returned dual ray passed
an independent bounded Farkas check using every finite decision bound,
dual-cone membership, nonzero stationarity, and conservative floating-point
dot-product error. Its unnormalised certificate margin was approximately 1;
the ray-normalised gap was 1.003e-9. Coneprog's original endpoint-equality
residual was 3.842e-5. This is an inconsistent intermediate conic model, not
evidence that the final independently validated baseline motion is invalid.

The numerical screen is therefore corrected to accept either a sufficiently
accurate optimal solution or an independently verified infeasibility ray;
matching coneprog's positive flag on a disproven-feasible model is not a valid
correctness requirement. This changes no physical tolerance or full-planner
quality/runtime gate. An unverified ray remains an unresolved solver failure.
No new solver is accepted for production on this basis. The next screen records
every such disagreement explicitly before any full-planner evaluation.

The corrected screen passed the six ObstacleAvoidance models, including the
certified inconsistent model, then stopped on StaticU program 5 at a reduced-
accuracy native exit. Its independently bounded objective gap was 1.6287e-5
against the original 1e-6 optimality tolerance. The native reported gap was
6.8227e-7, but that alone did not meet the declared independent bound check.
No physical invalidity is inferred from this subproblem screen; it is an
unqualified replacement and was not integrated. The original offending plane
rows had coefficient norms near 2.01e-9 and RHS -1.864e-7. This explains the
earlier status disagreement without accusing the valid final motion of failure.

The user explicitly reiterated that every planner stage may change. The next
bounded hypothesis changes the motion representation while retaining coneprog:
use the existing degree-seven C3 Bezier representation on ordinary static
regions, with the original three spans per true route edge. This halves the
number of controls per span and reduces third-derivative coefficient scale
from 3360 to 210. It is a different admissible curve family, so success and
runtime alone are insufficient: retain the original arrival/travel quality
gate and independent full-motion checks. No route, margin, tolerance, or seed
budget changes. Focused full cold comparison: StaticU; structurally different
case: Slalom. Reject rather than tune a degree sweep if either fails. Owned
sources are output/cold-method-research-20260905/compact-planner. Production
still uses the original degree selection and coneprog; no commit is earned.

The compact representation failed the focused quality gate. StaticU cold
runtime was 22.4896238 s baseline versus 11.2850466 s candidate (1.993x), with
both passing planner, independent, collision, kinematic, continuous, and plane
checks and goalReached. The selected polyline stayed 34.9425880405 deg; the
smoothed length changed 39.3787713567 -> 39.6252468091 deg and motion duration
20.8323620005 -> 20.8897527040 s. The 0.0573907035 s delay exceeds the unchanged
0.001 s tolerance. The different-case run was correctly skipped, and the
isolated compact implementation was removed. This is a rejected planner-level
speed/quality tradeoff, not an accepted speedup.

Next planner-level hypothesis to investigate: coarse-to-fine continuation can
use a small model only to construct a feasible curve, then transfer that exact
curve by Bezier subdivision and degree elevation into the original degree-16
space. This retains the original final admissible family while potentially
avoiding expensive collision-discovery iterations. It is not yet implemented
or timed. Keep the same seed families, final model, original public quality,
physical validation and cold full-planner gates; prior solves from other
planner calls remain forbidden. The smaller model alone does not qualify.

After removal, the original StaticU request again passed planner, independent,
collision, kinematic, continuous, and plane checks with goalReached, exact
baseline lengths 34.9425880405 / 39.3787713567 deg and duration 20.8323620005 s.
The fresh replay took 20.5270977 s internally and 40.0567705 s launch-to-exit.
The baseline variation reinforces that the reported single pairs are screening
observations rather than repeatable speed claims. The final diff check passed;
the user-owned failed.mat hash remained unchanged. All new research sources
remain ignored, rejected native/compact implementations were removed, and no
commit or push occurred. The full eight-case 3x cold target remains unproven.

## Corridor initialization research continuation - 2026-09-05

The prior goal turn made progress by rejecting five isolated candidates with
measured status, quality, or runtime failures; none changed production. A
subsequent single-computational-thread cold probe preserved ObstacleAvoidance
success and every independent check, with motion 7.5538884272 s, but took
7.2426022 s versus the preceding default-thread 7.4854214 s observation.
The process default was six computational threads; restoration was asserted.
The approximately 1.03x single-sample ratio is not useful speed evidence.
The experimental thread override was removed. No pool or global setting was
left changed. Reference: https://www.mathworks.com/help/matlab/ref/maxnumcompthreads.html.

New bounded hypothesis: analytically construct an initial convex corridor
around every separable input-route span, removing only halfspaces whose
redundancy is verified against the retained corridor polygon. Use its
certified planes in the first existing trajectory SOCP to avoid unconstrained
collision-discovery rounds. If a complete seed corridor cannot be certified,
keep the original initialization and report that outcome. All obstacles,
seeds, horizons, later plane updates, and public validators remain in force.
This changes initialization from current inputs, never reuses a prior run.
First full-planner check: ObstacleAvoidance; second: StaticU. The same cold
3x aggregate and per-case quality/validity gate governs retention. Experimental
package ownership is output/cold-method-research-20260905/corridor-planner;
no production or benchmark helper commit is authorized without proof.

The corridor first-call screen passed all physical checks but failed the
arrival/runtime gate: 7.4287746 s wall time, motion 7.567957385 s versus the
baseline 7.553888427 s (0.014069 s later, public tolerance 0.001 s). Selected
polyline was 11.152119519 deg, smoothed length 11.431862080 deg, termination
goalReached, with planning, independent validation, collision, kinematic,
continuous, and plane checks true. The candidate was rejected and removed;
its initial geometric certificate did not demonstrate a useful overall gain.

## Requested coneprog reset and cold-only research gate - 2026-09-05

Production now calls MATLAB coneprog directly in both trajectory and plane
solves. The MATLAB fastcone package, associated tests, guide, benchmark
helpers, and obsolete recovery path were removed at the user's request.
The preserved experimental native binaries and old implementation copies
were also deleted; historical measurements below describe removed methods.
The stable conic diagnostics schema remains, with retired method counts zero.
These changes are uncommitted. No new method earns a commit unless the
user's cold-start performance gate is proven.

The full restored-baseline suite initially passed 124/126 tests. The two
failures were the package file inventory and a backend-specific convergence
expectation. After correcting those expectations, all 39 architecture and
planner-contract tests passed. Every current test was therefore covered by
the full run plus focused rerun; this is not a claim of a single 126/126 run.
All 18 maintained examples produced their expected outcomes: 17 independently
validated successes and the expected NoPath/noValidatedSeed failure. Actual
run metrics were appended to benchmark.csv. These behavior checks ran in one
MATLAB process and are not cold timing evidence. The user-owned failed.mat
is unchanged (SHA256 E754CF5D4C1ECA3B5B50865DDCFBC9673A64CC252615D6521A062A75B18C9625).

Research gate: identical saved inputs/options, seed 0, first planner call plus
independent validation in a fresh MATLAB R2024b process; no numerical warmup.
Report launch-to-exit separately. Require at least 3x by summed case medians
across the eight conic-using maintained requests, disclose each case, reject
any greater-than-10% case slowdown, preserve original arrival tolerance,
physical constraints, public validation, and exact diagnosis-bundle behavior.
A kernel-only or warmed benefit cannot qualify. Scripts remain ignored under
output/cold-method-research-20260905, outside the committed engine.

Cold profiling (attribution only, not speedup evidence) found coneprog used
1.385 s of a 4.009 s ObstacleAvoidance run and 15.835 s of a profiled 22.521 s
StaticU run. Options construction, geometry, and independent validation are
material costs. Merely replacing the numerical solver cannot be assumed to
provide 3x for the whole planner.

The first new hypothesis eliminates time cones analytically for a fixed-plane
earliest-arrival subproblem: with q=T^3, derivative bounds have concave right
sides q^(1/3), q^(2/3), and q. Supporting tangents yield LP lower bounds;
direct derivative maxima give feasible upper bounds on time. This monotonicity
is local to fixed normalized planes and does not assert moving-obstacle
feasibility is monotone. Its 20-program screen failed: 47.813 s versus
coneprog's 11.894 s, with only 16/20 matching the required status, feasibility,
and objective checks. One LP reported infeasibility where coneprog returned
success; three hit the iteration cap. This is not cold full-planner evidence.
The candidate was rejected without production integration or a commit.

The next isolated hypothesis analytically eliminates Bezier endpoint and C3
continuity equalities before calling coneprog. It preserves polynomial degree,
spans, objective, inequalities, and cone tolerances. Its acceptance gate is
unchanged; reduced dimension alone is not evidence of speed.

The unscaled equality elimination failed its first screen: 0.218 s versus
0.375 s, but a reconstructed derivative inequality was violated by 0.312 in
original matrix units despite coneprog reporting success. The retained
baseline's maximum endpoint-equality error was 5.08e-5, so raw solver
residuals cannot replace physical validation of either formulation. No
integration was retained. A separate, explicitly scaled formulation will
use an exactly feasible quintic endpoint curve as its affine origin and
normalize the transformed rows; its physical acceptance gates stay fixed.

That separate scaled formulation also failed screening at the third
ObstacleAvoidance program: motion time 7.525718 s versus 7.524976 s, beyond
the declared tolerance, with only a 1.15x subproblem timing ratio. The first
call was slower (0.357 s versus 0.255 s). It was rejected without integration.
The next reference candidate is the independent ECOS native MATLAB solver
(https://github.com/embotech/ecos-matlab); its source stays in ignored output
for experimentation. Its GPL-3.0 license is a deployment consideration if it
ever qualifies. No native replacement has been retained or committed.

The exact unchanged diagnosis bundle passed a separate restored-coneprog
replay and independent validation: 53.8944204 s wall time, 69.06222508 s
motion duration, goalReached. This single replay is not speedup evidence.
The full baseline test suite also passed the supplied-bundle regression.

ECOS (MATLAB interface commit 2acb7f472f0021a3d226da187cd2941341199893,
core 2954b2a640f2194bf91dbf51e682be17012d7698) was compiled locally with
Microsoft Visual C++ 2022. Its first six captured trajectory programs were
6-22x faster, but this did not survive the full-planner gate. A single fresh
ObstacleAvoidance pair was 6.397730 s coneprog versus 4.355865 s ECOS (1.47x),
both independently valid. ECOS motion duration was 7.524794 s versus 7.553888 s;
smoothed length 11.499835 deg versus 11.430862 deg, with identical 11.152120 deg
selected polylines. ECOS made 12 calls, taking 0.117092 s, with no fallback.
The structurally different StaticU request failed with noValidatedSeed after
4.313587 s, 36 ECOS calls, and one nonoptimal solver exit. All success,
validation, collision, kinematic, continuous, and plane flags were false;
motion metrics were NaN. The baseline is known feasible. ECOS was rejected
and its experimental source, native binary, and integration were removed.
These single cold probes do not establish repeatable timing statistics.

Correction to the scaled-Bezier screen: its preliminary 1e-6 subproblem
objective threshold was stricter than the public 1e-3 arrival tolerance.
The 0.000743 s subproblem time difference alone does not establish a public
quality regression. It was not independently validated in the full planner,
and its small measured runtime benefit did not justify integration.

Next hypothesis: choose coneprog's documented prodchol trajectory step solver
for the sparse trajectory matrix with dense time-power columns. Keep plane
solver, original model, default tolerances, and all planning stages unchanged.
Reference: https://www.mathworks.com/help/optim/ug/compare-speeds-coneprog-algorithms.html.
The cold full-planner retention gate remains unchanged.

The prodchol trajectory-step variant failed the first cold full-planner
case: ObstacleAvoidance returned noValidatedSeed after 7.3830203 s. Planner,
independent validation, collision, kinematic, continuous, and plane flags
were false, with NaN motion metrics. Its isolated package was removed.
No tested candidate currently qualifies for the requested 3x cold speedup.
Production remains the uncommitted coneprog reset; no new solver was retained.
After removal, the original ObstacleAvoidance request again passed planning,
independent validation, collision, kinematic, continuous, and plane checks:
7.4854214 s cold wall time, polyline 11.152119519 deg, smoothed length
11.430861536 deg, motion 7.553888427 s, goalReached. The difference from the
preceding 6.3977298 s baseline probe reinforces that these single samples
are screening evidence, not stable speedup estimates. The final diff check
passed, and the user-owned diagnosis bundle hash remained unchanged.

## Fresh-process cold comparison, including prior MEX fastcone - 2026-09-05

Each of the same eight saved conic-using requests ran once per fresh MATLAB
R2024b process, seed 0, original limits/options and independent validation.
There were three fresh processes per method per case: 48 launches alternating
coneprog/current MATLAB fastcone, followed by 24 launches of the saved prior
MEX-backed fastcone at the user's request. No warmup preceded the timed call.
These are fresh-process measurements with Windows file caching intact, not
cold-boot tests. Input loading precedes the inner timer. The outer timer spans
process launch through exit, including setup, reporting and shutdown.

| Request | coneprog first-call median (s) | MATLAB fastcone (s) | Prior MEX fastcone (s) |
|---|---:|---:|---:|
| AlternatingSlalom | 5.5084287 | 4.2180411 | 3.6886754 |
| NoPath | 2.2834241 | 2.5805275 | 2.5407968 |
| ObstacleAvoidance | 3.8650023 | 3.4693862 | 3.0217061 |
| StaticUShapedObstacle | 12.3548761 | 5.1173693 | 4.5515608 |
| StraightTargetAlternatingOcclusion | 4.8066317 | 4.0084202 | 3.5840305 |
| TargetExitsObstacle | 8.1528264 | 5.6479202 | 5.2168433 |
| TwoOpposingUVisibilityGraph | 4.3998600 | 4.4519761 | 3.8311501 |
| USOutlineExtremeVisibility | 7.0616042 | 6.7674912 | 5.8347846 |
| Sum of case medians | 48.4326535 | 36.2611318 | 32.2695476 |

Current MATLAB fastcone is 1.3357x faster than coneprog by summed first-call
medians; its NoPath result is 13.01% slower and TwoOpposingU is effectively
tied (1.18% slower with overlapping ranges). MEX is 1.1237x faster than current
MATLAB and 1.5009x faster than coneprog on that aggregate. MEX/Matlab NoPath
ranges overlap and do not establish a meaningful difference.

Summed launch-to-exit medians were 133.1285768 / 121.0192690 / 116.6766350 s
for coneprog / MATLAB / MEX: only 1.1001x for MATLAB versus coneprog, and
1.0372x for MEX versus MATLAB. MEX was measured as a later follow-up cohort,
not interleaved with the other two; small differences should not be attributed
solely to its compiled kernel. The prior MEX version also has different plane
and recovery algorithms from the current MATLAB solver. This is not a test of
compiling today's direct equations.

All 72 launches exited successfully: 63 goalReached motions passed every
independent collision, kinematic, continuous and plane-certificate check;
the nine NoPath runs retained noValidatedSeed with planner/validator/certificate
flags false and NaN motion metrics. Current MATLAB and coneprog cold motion
metrics matched their warmed results within 1e-9. MEX kernel execution was
confirmed in all 24 MEX runs; reference recovery remained explicit. Full
per-run motion metrics, min/max timing ranges, launch records and the MEX
comparison are ignored under output/cold-coneprog-3d60f83-20260905.

The MEX binary was the preserved benchmark copy under +nativeBaseline,
SHA256 35782568E5BE5AC15B4FD441106ED72EF1A9E9D5E1FDC7AA0A5E28B74A964DCB.
Production remains MATLAB-only. No benchmark script or binary was added to
source control, and benchmark.csv was not appended for saved-request replays.

## Full planner comparison with coneprog on 3d60f83 - 2026-09-05

The eight maintained examples with actual conic calls were replayed from their
saved public inputs/options in MATLAB R2024b, seed 0, headless, with default
finite jerk limits. Production was unchanged. An ignored dispatch harness
selected a frozen copy of the current MATLAB fastcone package or coneprog for
every conic call. Executed-call counts confirmed the selected backend, with
zero native calls. Two warmups and three interleaved measured runs per method
gave 80 total runs. Wall time includes the public planner and an additional
independent validation; it excludes artifact saving and MATLAB startup.

| Request | coneprog median (s) | fastcone median (s) | Speedup | fastcone recoveries/calls |
|---|---:|---:|---:|---:|
| AlternatingSlalom | 2.6475373 | 0.7395568 | 3.5799x | 8/38 |
| NoPath | 0.1957021 | 0.1454587 | 1.3454x | 2/3 |
| ObstacleAvoidance | 1.2260341 | 0.2659976 | 4.6092x | 3/12 |
| StaticUShapedObstacle | 9.7713896 | 1.5944930 | 6.1282x | 17/199 |
| StraightTargetAlternatingOcclusion | 1.9217796 | 0.6135839 | 3.1321x | 8/27 |
| TargetExitsObstacle | 5.5039130 | 2.2903646 | 2.4031x | 23/56 |
| TwoOpposingUVisibilityGraph | 1.5954299 | 0.8050170 | 1.9819x | 13/46 |
| USOutlineExtremeVisibility | 4.2649666 | 2.9231906 | 1.4590x | 33/134 |

The sum of case medians decreased from 27.1267522 to 9.3776622 s: 2.8927x,
or 65.4302% less wall time. All eight measured ranges were disjoint in favor
of fastcone. Conic-solver timer medians summed to 22.3084619 versus 4.6182110 s
(4.8305x); these are nested attribution times, not additive to total runtime.
The planners may take different iterations, so these are end-to-end results,
not same-program microbenchmarks. No 10x general planner speedup is claimed.

All successful runs on both methods passed the planner, independent validator,
collision, kinematic, continuous-collision and plane-certificate checks and
returned goalReached. NoPath returned noValidatedSeed on both: planner and
validator false, certificate flags false, motion metrics NaN. Polyline lengths
were identical between methods. Final coneprog/fastcone smoothed lengths and
motion durations were:

| Request | Smoothed length (deg), coneprog / fastcone | Duration (s), coneprog / fastcone |
|---|---:|---:|
| AlternatingSlalom | 16.2829658411 / 16.2930579142 | 10.6126605873 / 10.5049858565 |
| ObstacleAvoidance | 11.4308615359 / 11.4270998053 | 7.5538884270 / 7.5247939340 |
| StaticUShapedObstacle | 39.3787713567 / 39.3902842072 | 20.8323620005 / 20.7678669439 |
| StraightTargetAlternatingOcclusion | 13.5986641387 / 13.6172278486 | 20.8695652174 / 20.8695652174 |
| TargetExitsObstacle | 20.6100682085 / 20.6095651504 | 24 / 24 |
| TwoOpposingUVisibilityGraph | 24.6652397599 / 24.7642513406 | 21.8475039511 / 21.8341992694 |
| USOutlineExtremeVisibility | 23.3604967802 / 23.3649471841 | 5.8044339735 / 5.7999328153 |

Fastcone's motion durations were equal or shorter, while smoothed length was
up to 0.4014% longer. Original tolerances and independent validation remained
unchanged. All per-run metrics, min/max timing ranges, final results and the
temporary harness are ignored under output/coneprog-planner-3d60f83-20260905.
These were saved-request replays, so benchmark.csv was not appended again.

## Fresh full example run on 3d60f83 - 2026-09-05

All 18 maintained example functions ran headlessly in one MATLAB R2024b
process, with seed 0 and original default motion/jerk limits. Seventeen
returned goalReached and passed independent collision, kinematic and
continuous-collision validation; all applicable plane certificates passed.
NoPath returned the expected noValidatedSeed failure, with planner/validator
false and unavailable motion metrics NaN. There were no execution errors and
no native solver calls. All seven successful examples with conic calls used
explicit coneprog recovery somewhere in their search; NoPath also did so.

Summed example wall time was 51.4668492 s, including example setup and its own
validation but excluding MATLAB startup and the additional independent check.
The slowest observations were MovingDeformingUSOutlineVisibility 16.6991703 s,
USOutlineExtremeVisibility 11.3470044 s and AlternatingSlalom 8.7458688 s.
These are single sequential runs with shared JIT/cache state, not warmed
interleaved speedup measurements. Eighteen actual rows with source commit
3d60f833e9d72baef2008c0a34548e8a66e17c59 were appended to benchmark.csv.
Detailed results and the temporary harness remain ignored under
output/all-examples-3d60f83-20260905. The user's failed.mat was not modified.

## Committed MATLAB solver and focused cleanup - 2026-09-05

The MATLAB replacement was committed first as 9dec1bf. The subsequent cleanup
removed unused cone initialization, unreachable equality scaling after exact
elimination, and duplicated recovery metadata. Fastcone now has 1,328 physical
MATLAB lines versus 1,350 in that commit. Its 15-file mathematical structure,
public diagnostics, certificate gates and explicit reference recovery remain.
There are no production MEX files, native sources or Eigen dependencies.

All 200 saved requests retained bit-identical numerical results and non-timing
diagnostics against a frozen copy of 9dec1bf. All 64 affected solver/BMTP/planner
contract tests passed; the complete 151-test suite and all 18 maintained
examples had passed before this behavior-preserving cleanup. No additional
benchmark script or generated artifact is tracked. The user-owned failed.mat
is unchanged by this work and excluded from both commits.

Fresh complete-adapter measurements used four warmups and nine interleaved
measurements per method, identical inputs/options and MATLAB R2024b:

| Captured plane | Exhaustive MATLAB (ms) | Native (ms) | 9dec1bf (ms) | Cleaned MATLAB (ms) | coneprog (ms) | Exhaustive/current |
|---|---:|---:|---:|---:|---:|---:|
| 9 | 62.0974 | 5.8277 | 3.2364 | 3.4019 | 3.9646 | 18.2537x |
| 11 | 62.9616 | 4.7971 | 3.2944 | 3.2232 | 3.7181 | 19.5339x |

Current MATLAB is 1.1654x/1.1535x faster than coneprog on these two requests.
The cleanup itself is retained for removing dead work, not a claimed speedup:
its measurements vary +5.1%/-2.2% relative to the preceding commit. Both current
MATLAB results pass the original residual and objective gates, residuals
2.22e-16/1.87e-16. On request 9, native recovery and coneprog both report success
but have original residual 1.0811e-6 against tolerance 1e-6. The benchmark first
stopped on that baseline assertion; its corrected reporting preserves this
unfavorable result instead of weakening the candidate's gate. Request 11's
reference residual is zero. Timings above are not evidence of uniform speedup
or a general closed-form solution for every conic program.

## MATLAB-only production integration - 2026-09-05

Production fastcone now uses MATLAB contact equations and a MATLAB conic
predictor-corrector kernel. Both BMTP call sites execute it. Native source,
builder and Eigen dependencies have been removed; the locked retired binary
was moved into ignored output without interrupting the user's MATLAB desktop.
Unresolved programs retain explicit coneprog recovery. Positive results require
original feasibility and objective-bound checks; negative certificates require
the original tolerance-expanded program to be excluded by a raw dual witness.
No claim of solving every possible conic program or uniform speedup is made.

All 18 maintained examples ran on actual production with no native calls:
17 independently validated successes and the expected NoPath failure. Their
actual metrics are appended to benchmark.csv. Every successful run passed
collision, kinematic and continuous-collision checks; applicable plane
certificates passed. The unchanged user diagnosis bundle also passed with its
original options: wall 27.1458707 s (single observation), polyline/smoothed
length 143.44415659 deg, motion duration 69.06222507996 s, goalReached.
Its representation does not use a plane certificate. Jerk-disabled verification
was not executed: the public interface rejects infinite jerk limits before
planning. No input validation was weakened to manufacture that mode.

The two slow plane requests improved 18-20x in warmed complete-adapter tests.
The original-input infeasibility certificate reduced tight-U median runtime
from native 3.3026244 s to MATLAB 2.0265543 s. Batched Lorentz Gram preparation
then reduced the moving-occlusion replay to native 0.5217166 s versus MATLAB
0.5673396 s (8.7448% slower, within the declared 10% gate; nine interleaved
measurements after three warmups). Its previous 14.55% regression is retained
below as unfavorable history. Six reduced-program comparisons verified
bit-identical Gram maps; the largest favorable setup/solve measurement was
42.0913 to 26.3969 ms, while one case regressed 12.2162 to 13.3030 ms.
These prototype timings must not be confused with fresh production timings.

The final production suite passed 151/151 tests, zero failures or incomplete
tests. All 200 captured requests matched the verified prototype bit-for-bit
in candidate, objective, status and recovery choice: 199 positive results,
12 explicit recoveries, zero native calls. The original residual gate passed
for every positive result. These checks supersede older pending-migration
statements below; the requested code-pruning stage follows the first commit.

## Original-input infeasibility certificate - 2026-09-05, isolated

The remaining tight-U overhead was mostly coneprog recovery after the MATLAB
kernel already found a reduced infeasibility ray. The expensive requests are
not identical: requests 63 and 64 have different A/b, so result caching is not
an appropriate explanation or repair. Each spends about 0.04 s in the MATLAB
candidate and another 0.4 s in coneprog.

The new candidate reverses row scaling and exact cone simplifications only to
propose a dual witness. Acceptance uses the original, unscaled matrices, not
the reduced bound. For raw cone-feasible z and arbitrary equality multiplier
lambda, it bounds the zero objective by

`-h'*z - d'*lambda + min_box (G'*z+E'*lambda)'*x`.

The box is expanded outward by ConstraintTolerance and rounding allowance.
The bound additionally subtracts the tolerance times scalar/head dual weights
and absolute equality multipliers, plus a dimension-scaled arithmetic error
allowance. A strictly positive remaining bound excludes even the original
tolerance-expanded feasible set. Otherwise the original recovery remains.
The witness and its raw bound, tolerance, allowance, and margin are retained.
Certified failure returns -2 explicitly, never positive acceptance.

Four expensive tight-U requests have raw margins 0.0291-0.1128, versus arithmetic
allowances below 1.8e-7, and no longer need reference recovery. Two near-boundary
failed controls did not certify and retain coneprog. Seventy independent
small checks passed: linear/equality and Lorentz infeasibility, near-boundary
tolerance controls, and 64 constructed feasible random programs (seed 58031).

With three warmups and three interleaved measurements and both frozen engine
variants resident, tight U measured native 3.3026244 s versus MATLAB 2.0265543 s
(1.6297x faster). Its third-seed selection, validated motion and arrival remain
the same as the plane-recovery candidate below. Every successful motion check
passed and the candidate executed no native code. The wider validation and
production migration remain pending; this is not yet a committed replacement.

## MATLAB plane-selection recovery - 2026-09-05, isolated

The tight-U correctness regression below now has a verified experimental
recovery. When a trajectory solve fails after direct multiple-contact planes,
the engine recomputes those planes with coneprog using the exact stored source
curves and original plane options, then retries the trajectory. Each plane
source is consumed once; reference calls and recovery events are counted.
No seed, horizon, protected geometry or feasibility tolerance is discarded.
This reorders recovery work; it does not label the failed direct plane optimum
invalid or prove a route infeasible. The requested deadline still constrains
the final independently validated motion.

The first version recovered the third tight-U seed but measured 4.655211 s
versus native 3.2797149 s (41.9% slower). Saved per-call timings identified
duplicate infeasible trajectory solves costing about 0.4 s each. Applying the
existing allowed warm-start horizon expansion before the reference-plane retry
reduced the candidate median to 3.7105884 s versus 3.2875076 s (12.9% slower),
with the same motion. Both variants used three warmups and three interleaved
measurements. A separate comparison kept frozen copies of both complete engine
packages loaded to avoid repeated package/JIT invalidation: native 3.2872557 s,
MATLAB 3.7056346 s, ratio 1.127273. Thus the remaining overhead is not explained
by that measurement concern. The declared 10% full-planner speed gate still
fails; production remains unchanged and no commit or push has been made.

The recovery-enabled prototype also replayed the other four saved requests:

| Request | Native median (s) | MATLAB median (s) | MATLAB/native |
|---|---:|---:|---:|
| Obstacle avoidance | 0.2451889 | 0.2525531 | 1.030035 |
| Moving alternating occlusion | 0.5206881 | 0.5743166 | 1.102995 |
| Static U | 1.5562432 | 1.6106196 | 1.034941 |
| No path | 0.1565575 | 0.1410910 | 0.9012088 |

All successful jerk-enabled replays passed planner and independent collision,
kinematic, continuous-collision and plane-certificate checks (goalReached).
Tight-U MATLAB polyline/smoothed length/duration were 34.94258804047 deg,
39.1469628764 deg, 20.76809864315 s; native values were 34.94258804047 deg,
39.1459691515 deg, 20.76813613373 s. The other successful MATLAB motion metrics
match the contact-exchange table below. NoPath retained noValidatedSeed,
planner/validation false, all certificate flags false and numeric motion
metrics NaN. Every MATLAB replay executed zero native calls. These are saved
public-request replays, not maintained example invocations; benchmark.csv was
not appended. Recovery remains an ignored experiment pending the speed gate,
structurally broader production verification, and integration.

## Two-plane contact exchange - 2026-09-05

This supersedes the earlier prototype speed-gate status below; unfavorable
historical measurements remain recorded. The first full MATLAB planner replay
regressed 64% on obstacle avoidance and 45% on moving alternating occlusion.
Two multiple-contact plane requests spent about 62 ms enumerating 52,224
profiles. Contact exchange now solves a small working set using scalar
distance derivatives, quadratic unit-normal equations, and four-contact
determinants. Certified contact bases and previous support vertices order
work. Every original product row is checked after each exchange; the original
primal/dual gap and physical residual authorize acceptance. Unresolved cases
retain explicit coneprog recovery.

Four warmups and seven interleaved complete-adapter timings gave:

| Captured plane | Exhaustive MATLAB (ms) | Contact exchange (ms) | Native adapter (ms) | MATLAB improvement |
|---|---:|---:|---:|---:|
| Obstacle avoidance request 9 | 67.0441 | 3.7652 | 5.8145 | 17.8063x |
| Obstacle avoidance request 11 | 65.3146 | 3.2686 | 4.4802 | 19.9824x |

Both passed original feasibility and objective checks, residuals below 2.3e-16.
Twelve deterministic structurally different plane requests also ran: two
degree-16 cases obtained direct certificates, while ten explicitly remained
unresolved. This is not evidence of a complete analytical solver.

A separate conic stall reached 300 iterations. A positive weak-dual bound on
the zero objective, with an arithmetic allowance, now requests reference
recovery after eight iterations. It does not return infeasibility itself.
The recovered flag and original residual remain visible. One captured program
has a coneprog-positive result above its requested physical residual tolerance
in both versions; it is not a certified intermediate solve.

Three warmups and three interleaved complete public saved-request replays,
MATLAB R2024b Update 4, seed 0, original jerk-enabled options, measured:

| Saved request | Native median (s) | MATLAB median (s) | MATLAB/native |
|---|---:|---:|---:|
| exampleObstacleAvoidance | 0.2726315 | 0.2591129 | 0.9504144 |
| exampleStraightTargetAlternatingOcclusion | 0.5780752 | 0.6244180 | 1.080167 |
| exampleStaticUShapedObstacle | 1.5439314 | 1.6278869 | 1.054378 |
| exampleNoPath | 0.1660474 | 0.1425973 | 0.8587747 |

All three successful motions passed independent collision, kinematic,
continuous-collision and plane-certificate checks, terminating goalReached.
Polyline/smoothed lengths (deg), duration (s) respectively:
11.15211951902/11.42709980534/7.52479393397;
13.34166406413/13.6172278486/20.86956521739;
34.94258804047/39.39028420716/20.76786694392.
Arrival differences were below 2.2e-5 s versus the unchanged 0.001 s tolerance.
NoPath retained noValidatedSeed, planner/validation false and unavailable
lengths/duration NaN. No MATLAB candidate invoked native code. These saved
requests pass the declared 10% full-planner regression gate; they are not
maintained example invocations or a broad production test pass. Generic MATLAB
kernel calls can still be slower than native. Production migration and broader
verification follow. Ignored experiment logs preserve the measurements.

### Production integration rejected by broader correctness check

The isolated solver was migrated temporarily into +fastcone. The first 15
solver tests passed. A missing package qualifier in the new recursive contact
call was found and fixed; the multiple-contact polygon regression then passed.
All 200 captured production calls matched the measured prototype bit-for-bit
in primal variables, objective and exit flag, with matching recovery usage.

Broader planner tests nevertheless found a new failure in
testPlannerContract/testLaterSeedsRunOnlyAfterFirstTwoFail: the exact 21-second
static U request no longer recovered a validated third seed. This is distinct
from the pre-existing obstacle-avoidance numeric-reference mismatch, which
also remains. The broad correctness gate therefore fails despite the four
favorable saved-request replays. Production integration was restored in full;
the native implementation, builder, dependency files and MEX remain intact.
The candidate and regression logs are saved only in ignored output. No commit
or push was made. The user-owned failed.mat SHA256 remains E754CF5D4C1ECA3B5B50865DDCFBC9673A64CC252615D6521A062A75B18C9625.
Unchanged-input owner isolation found: native planes + MATLAB kernel succeeds
and validates (arrival 20.7680820879 s); MATLAB planes + native kernel fails,
as does the fully MATLAB candidate. Fully native succeeds and validates
(arrival 20.7681361337 s). These are single diagnostic timings, not benchmarks.
The first changed plane output is captured call 22; five plane choices change
before trajectory-program input 35 differs. These direct three-contact planes
pass their conic gates but choose very different normals from coneprog's
approximate solutions. Thus individually better conic optima are insufficient
to establish preserved feasibility of the alternating planner. The plane-stage
cause is isolated; a general repair preserving the tight-horizon motion
capability has not been established. No tolerance or assertion was weakened.
After restoration, the original production testLaterSeedsRunOnlyAfterFirstTwoFail
passed again in a fresh MATLAB process. Tight-U native replay metrics were
polyline 34.94258804047 deg, smoothed 39.1459691515 deg, duration 20.76813613373 s;
native planes plus MATLAB kernel gave 34.94258804047 deg, 39.14303481687 deg,
20.76808208792 s. Both passed independent collision, kinematic, continuous and
plane checks with goalReached. Both MATLAB-plane variants had noValidatedSeed,
planner/validation false, all certificate flags false and lengths/duration NaN.

## MATLAB-only fastcone investigation - in progress, 2026-09-05

The production engine remains the native `36512e2` implementation. An isolated
MATLAB port preserves the selected conic certificates but still takes about
39-44 ms on large programs versus 25-31 ms natively. Direct three-contact plane
profiles now use circle/quadratic equations and nonnegative dual certificates.
Seven focused cases passed, with measured speedups ranging from 1.18x to 5.96x
in one warmed comparison; those plane baselines include coneprog recovery.
They do not measure the native Newton kernel alone.

The first complete 200-program replay replaced 45 coneprog recoveries, but its
sum of per-program median times was 1.1423772 s versus 0.9073317 s (26% slower).
Case 200 changed from native-adapter status -7 to a directly certified solution.
Case 198 exceeded its requested tolerance in both existing recovery paths.
No maintained examples were rerun for this prototype. The speed gate has not
passed; no MATLAB-only replacement or subsequent cleanup has been committed.
Ignored experiment outputs retain per-case timings and unfavorable results.

The next isolated replay added four-contact determinant profiles and reduced
recoveries to ten. Six background workers processed the same 200 independent
requests in a warmed median 0.6264373 s versus 1.1159425 s serially (1.78141x),
including client-side coneprog recovery. All serial/background outcomes matched
and every successful background solution passed the original residual and
objective-comparison gates. The first background batch was slower: 3.9044163 s
versus 1.8144198 s serially. These are independent-request throughput timings,
not a full planner speedup or evidence that dependent solves can run together.
R2024b rejected coneprog's optim.coneprog.socp on background thread workers;
the analytical and MATLAB Newton paths ran successfully. Production is unchanged.

The four-contact proposal ordering is not ready for production: it certified ten
additional captured cases, but cases 50, 63, and 199 regressed relative to the
previous MATLAB profile ordering. The general MATLAB kernel still misses the
native speed gate. Sparse border splitting, scalar boundary vectorization, and
compact scalar scaling experiments did not pass their retention gates; their
kernel edits were restored. Changed-value and changed-pattern cache checks gave
bit-identical cached/fresh solutions on three representative conic requests.

A direct comparison against MATLAB coneprog (three warmups and three interleaved
measurements per original request) measured 8.5836286 s versus 1.0102061 s for
the current serial MATLAB prototype, summing the 200 per-request medians:
8.49691x overall. The prototype was faster on 166 requests and slower on 34,
with ten explicit coneprog recoveries. All 199 successful candidate results
passed original feasibility and objective-comparison checks; the remaining
request retained the expected failure. This does not establish universal speed,
full-planner speed, or parity with the existing native fastcone implementation.

An independently valid native replay captured eleven real alternating-planner
plane groups of sizes [2 7 6 13 26 4 30 1 31 31 31]. Replaying those groups with
the MATLAB prototype gave sum-of-group-medians 0.384121 s serially versus
0.3077873 s with background workers (1.248008x). Small groups often slowed down;
the earlier 1.78141x aggregate-batch result must not be applied to these smaller
dependent planner phases. No background planner integration has been retained.

## Overwritten diagnosis bundle runtime - 2026-09-05

The replacement `Rogue Examples/failed.mat` (SHA256 prefix `E754CF5D4C1E`)
records a successful, independently valid plan in 90.0742 **seconds**, not
minutes. Its selected motion lasts 69.0622 s. Topology/search owns 86.7068 s
of the saved runtime, versus 2.8230 s of motion solving. A fresh MATLAB
profile attributes most search time to repeated timed-edge collision queries:
76 nodes, 41 retained time layers, and roughly 1.68 million rejected
transitions against one moving obstacle with 21 time samples of 49 vertices.
This case's bottleneck is search work, not the fastcone numerical kernel.

The focused optimization rejects a strictly more expensive incoming motion
before collision queries when a cheaper reachable arrival can traverse the
same verified safe-wait component. It preserves cost ties, every supplied
time layer, geometry, objective, and validation tolerance. It applies only
to full-horizon searches: the first unrestricted trial changed the future
frontier of an early-stopping search, despite preserving its selected route.
The corrected version preserves routes, times, explored states, frontiers,
and reachable goal-layer counts on 36 deterministic graphs (seed 4701)
covering empty/static/moving geometry and all three arrival policies.
Single small-graph timing probes are mixed; no universal speedup is claimed.

Against the frozen `5efbea3` search implementation, one warmup per variant
and three interleaved full-planner repeats in MATLAB R2024b Update 4 gave:

| Full planner | Minimum (s) | Median (s) | Maximum (s) |
|---|---:|---:|---:|
| Baseline | 67.6591 | 102.1505 | 129.7499 |
| Cost pruning | 25.6147 | 29.2417 | 33.3816 |

The median improvement is 3.4933x (71.37% less runtime), exceeding the
predeclared 30% reduction gate. All eight warmup/measured replays passed
fresh independent validation. Selected seed arrays match exactly, motion
histories match within 1e-9, and retained search states/frontiers match.
Desktop timing varies substantially; the ranges are retained rather than
presenting a single favorable run. The remaining runtime is still dominated
by topology work. Worst-case search complexity and bounded completeness are
unchanged. The optimization adds 19 production lines and no option,
dependency, tolerance change, or benchmark source file.

All 18 maintained examples passed independent validation with their default
jerk limits, including the expected `exampleNoPath` failure. Failure graphics
and a visible successful example were verified. The test assessment is
145 passed, one pre-existing numeric-reference failure, zero incomplete.
The static quick-start test expects 7.52479716350113 s and
11.4274584581313 deg; the installed baseline and candidate both produce
7.52479561971388 s and 11.4270648853893 deg, with identical full motion
histories and valid physical certificates. A baseline profile confirms zero
calls to the changed timed-search function in that example. Its original
1e-6 assertions are preserved; the full suite is not reported as green.

## Fastcone dependency ownership - 2026-09-04

Eigen now lives at `trajectory/+fastcone/third_party/eigen`, beside the solver
that owns the dependency. All 345 dependency files, including original license
notices, match their pre-move SHA256 hashes. The build resolves Eigen relative
to the fastcone package; the repository-level `third_party` folder is removed.
The default MATLAB build succeeds with Microsoft Visual C++ 2022, and all 27
focused solver and architecture tests pass. No numerical algorithm, benchmark
script, or benchmark record changed, and no runtime improvement is claimed.

## Documentation cleanup - 2026-09-04

The root Markdown set is reduced from eleven files to five. Removed the
superseded architecture audit, two implementation plans, old engine file-size
rationale, and duplicate local workspace note. References are consolidated in
`README.md`, with retired HS3 and waypoint-refinement methods labeled historical.
The public obstacle contract, solver guide, repository rules, dependency
notices, and measured verification records remain. No engine, solver, test,
example, benchmark script, or measured CSV changed in this cleanup.

One removed proposal held distinct negative evidence, retained here: a
historical Ruckig pre-screen reordered all BMTP seeds but preserved arrival
and path length while increasing total measured runtime by about 14 percent.
The recorded statistic was the warmed minimum of three runs, not a median:

| Historical case | Baseline (s) | Pre-screen (s) |
|---|---:|---:|
| exampleObstacleAvoidance | 2.472 | 2.380 |
| exampleStaticUShapedObstacle | 1.912 | 1.579 |
| exampleStraightTargetAlternatingOcclusion | 5.488 | 6.013 |
| exampleTargetExitsObstacle | 18.073 | 20.731 |
| rogueBundle | 7.575 | 9.888 |

The source revision was not identified in that table; do not compare it to
current fastcone timings. The proposed Ruckig-to-Bernstein warm-start converter
was never implemented, and its external restart interface has since been
removed. Unequal jerk-phase intervals cannot generally be represented exactly
by one polynomial per equal-duration span. These are historical limits and
negative results, not a current implementation plan or a new measurement.

That proposal also reported a 38-row kernel comparison with no arrival wins
and no additional validated cases over BMTP. It did not identify the raw
corpus or source revision there. Preserve this as a historical reported
negative result, not a fresh verification or a universal impossibility claim.

## Accepted fastcone integration - 2026-09-04

The user accepted the measured blocks engine and explicitly requested adding
it to `bmtp-cleanup-codex`, wiring BMTP to it, and pushing the branch. Both
conic construction paths now use `fastcone.solve`. The original problem,
physical limits, solver tolerances, candidate comparison, and independent
trajectory validation remain in place. Native or analytical acceptance and
original-coneprog recovery are visible in phase diagnostics.

The accepted pre-integration measurements give 3.774x across eleven qualifying
full-planner examples and 9.50836x for 1,028 identical conic programs. These
are different benchmark boundaries. The fixed-arrival degree-7/eight-span
replay slows to 0.656x, and two full-planner objectives regress slightly.
Those outcomes remain in the [per-case records](benchmarks/results/fastcone/README.md).
This is user-authorized adoption of the measured tradeoff; it is not a claim
that the earlier zero-quality-regression gates or the initial 10x target pass.

The package adds the selected MATLAB setup, analytical equality and plane
profiles, independently checked native kernel, explicit recovery, local MEX
builder, and pinned Eigen headers. New ordered-kernel and plane-box experiments
are excluded. Build instructions and mathematical scope are in
[docs/fastcone.md](docs/fastcone.md). Source-size overages, exact validation,
and prior failed reference checks are recorded in `verification.md`; existing
size thresholds have not been raised to manufacture a passing result.

Integration was prepared against `a072037` in an isolated worktree so the
original HS checkout's unrelated changes could be preserved. Existing branch
history is retained; this work does not stage those unrelated changes.

## Fixed time-power bound rejected - 2026-09-04

An exact mathematical reformulation replaced the p0=1 equality with matching
lower/upper bounds and removed three net production lines. Static-U duration
improved 20.8323620005277 -> 20.8133759737414 s and sampled length
39.3787713567495 -> 39.2120182097737 deg. The tracked Rogue request also improved
slightly: 71.2828117654205 -> 71.2788182890289 s and
145.143797542061 -> 145.142116157488 deg. Both passed independent validation.

The fixed-arrival target-exits request stayed valid at 24 s but length increased
20.610068208467 -> 20.612611037524 deg, exceeding the predeclared 1e-6-deg
regression allowance. The full hunk was removed before repeated timing or broad
tests. Restoration reproduced the original fixed-arrival length and passed
fresh independent validation. Equivalent feasible sets can still change the
finite numerical iterations and resulting motion; no runtime gain is claimed.

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

### Prepared-boundary occupancy experiment gate - 2026-09-05

Profiles completed on the degree-eight saved requests. Failed bundle: planner
56.041 s inclusive; timed search 45.909 s; occupancy 46.997 s / 1,880 calls;
point clearance 27.778 s / 21,369 calls; shapeAtTime 12.910 s / 21,410 calls;
coneprog seed diagnostics total 0.593 s / 8 calls. U.S.: planner 11.937 s,
coneprog 3.614 s / 134 calls, occupancy 2.630 s / 12 calls. Nested profile
times are attribution only and cannot be added or used as cold speedups.
Both replays passed independent collision, kinematic and continuous checks;
failed bundle retained 143.444156590 deg / 69.062225080 s; U.S. retained
polyline 22.0706469075 deg, smooth 23.3542523251 deg, duration 5.7964986754 s
and its plane certificate. No maintained-example benchmark rows were added.

One isolated hypothesis: boolean occupancy can use verified numeric boundary
edges, avoiding repeated polyshape construction, edge extraction and nearest-
point output work. Keep authoritative shapeAtTime interval selection, source
cache checks, all 13 search samples, all candidate transitions and public
validation. A direct winding/projection calculation handles only points safely
away from numerical/occupancy boundaries; ambiguous points use the original
polyshape/pointPolygonClearance decision. Detailed query outputs remain original.
No cross-request caches, changed obstacle protection, seeds or horizon pruning.
The input scale is queried time groups times boundary edges times query points;
bounded vector blocks keep projection memory linear in a fixed working block.

Baseline: HEAD 3d60f83 plus existing dirty coneprog reset, unchanged failed.mat
SHA256 E754CF5D4C1ECA3B5B50865DDCFBC9673A64CC252615D6521A062A75B18C9625,
isolated degree-eight source from completed suite, MATLAB R2024b, six threads,
seed 0, original options and no warmup. Experiment-owned copy and harnesses
under ignored output/cold-method-research-20260905/prepared-query-planner only.
First compare unchanged failed bundle end to end, then structurally different
moving/concave/hole/boundary query tests and U.S. saved request. Require identical
query decisions, routes, selected motion and independent checks; retain this
component only if at least 20% failed-bundle cold improvement survives three
interleaved fresh-process comparisons without a >10% sentinel slowdown.
Remove the isolated source if the focused correctness/benefit gate fails.
This is a component gate; the original eight-case 3x adoption gate remains.

Rejected the prepared-boundary occupancy prototype after its first focused
cold end-to-end comparison. Degree-eight reference: 41.7647137 s; prepared
query: 49.9830390 s (19.6777% slower). Both succeeded with identical selected
polyline/smoothed length 143.444156590 deg and duration 69.062225080 s, passed
independent collision, kinematic and continuous checks, and goalReached.
The selected excursion does not use a plane certificate. No performance
benefit was demonstrated, so no repeats, further tuning or broader tests were
justified. The drafted structural query verification did not run. Removed the
entire isolated prepared-query package, temporary reference/query verification
functions, focused controller and method selectors. Retained the unfavorable
MAT/CSV/log evidence and this note, as required by AGENTS.md. Production was
never changed. A fresh degree-eight recovery control follows after deletion.

## Parametric-duration LP experiment - 2026-09-05

Previous turn classification: progress. The prepared-query hypothesis was
measured, rejected and removed; recovery confirmed the untouched degree-eight
result. Current worktree was inspected: only the requested uncommitted coneprog
reset, existing assessment/benchmark edits and protected dirty failed.mat remain.
No live MATLAB process was found. The 3x eight-case goal remains unchanged.

Measured owner: original StaticU profile spends 15.2101 s in 14 trajectory-step
calls out of 22.2087 s planner, with coneprog interior-point work dominant. A
bounded new hypothesis is to solve the unchanged fixed-plane earliest-arrival
model as a one-dimensional duration root over linear programs. Unlike the
rejected tangent-envelope LP, this changes only right-hand-side duration powers
in a fixed LP matrix, using dual sensitivity to update duration. Original
polynomial degrees (16 ordinary, 7 timed/grouped) and span allocations remain.

Derivation: for p0=1, p1^2<=p2 and p2^2<=p1*p3 imply p1<=p3^(1/3) and
p2<=p3^(2/3). Replacing them by T and T^2 at fixed p3=T^3 only enlarges the
derivative limits and satisfies both cones. Thus the earliest-arrival
fixed-plane model can minimize T subject to linear controls with derivative
right-hand sides limit*T^order. This monotonicity applies only to one fixed
plane/active-region model, never global moving-obstacle time feasibility.
A derivative-only phase-I LP in q=(T/Tref)^3 has decreasing convex value;
its dual supplies a duration update. A candidate's required duration is
recomputed directly from all original derivative rows. Original plane rows,
endpoint/continuity equations, workspace and horizon are checked explicitly.
Unqualified LP status, residual, duration gap or exhausted iteration budget
returns to the original coneprog solve; disclose and count all attempt costs.
Travel-weighted/fixed-arrival SOCPs and plane solvers stay coneprog.

Baseline: HEAD 3d60f83 plus existing reset, MATLAB R2024b, six computational
threads, fresh process first planner call, original saved StaticU request,
seed 0 and unchanged options/validation. No saved solution enters the solver.
Experiment copy: output/cold-method-research-20260905/parametric-lp-planner;
helper/runner under the same ignored research root. First run original and
candidate full StaticU planner+validator cold. Require at least 20% focused
benefit and unchanged physical validation/arrival tolerance before retaining
or broadening. If favorable, verify structurally different ObstacleAvoidance
and the exact unchanged failed.mat, then interleave three cold repeats and
qualify the original full eight-case >=3x gate before production integration.
If the focused gate fails, remove experiment-owned source and method selector,
retain the negative measurements and verify recovery. No tuning sweep.

Primary references inspected: MATLAB R2024b local linprog code confirms
available dual-simplex-highs and lambda outputs; current MathWorks docs
https://www.mathworks.com/help/optim/ug/linprog.html and
https://www.mathworks.com/help/optim/ug/linear-programming-algorithms.html.
Do not use newer R2026a sixth-output sensitivity APIs. Dual sensitivity
background: Boyd/Vandenberghe, Convex Optimization, perturbation/sensitivity,
https://web.stanford.edu/~boyd/cvxbook/. The duration reduction above is our
problem-specific derivation and still requires measured numerical qualification.

Rejected after the first full-planner cold StaticU comparison: original
20.6156600 s; parametric LP 23.4968329 s (13.98% slower). Both returned planner
success and independent collision, kinematic, continuous and plane-certificate
success, goalReached, polyline 34.9425880405 deg, smoothed 39.3787713567 deg
and duration 20.8323620005 s. The candidate ran 20 LPs in 14 trajectory attempts,
accepted one small-model replacement, and fell back to coneprog 13 times.
All 12 large-model first LPs were unqualified under the configured LP budget;
the recorded diagnostics do not distinguish the exact underlying LP exit flag.
Do not infer infeasibility from that generic research reason. Total attempted
LP work was 3.1987 s. No structural or broad example suite was run after failure
of the focused benefit gate. These were saved-request replays, not maintained
example invocations; benchmark.csv was not appended.

Removed the complete isolated LP engine package, focused controller, method
selector and mixed-solver allowance from the shared research harness. Kept
MAT/CSV/log measurements, including parametric-lp-trials.csv. The initial trial
export had an empty-struct assignment error; corrected only the export and read
the same saved result successfully, without rerunning or altering measurement.
Production remained coneprog throughout. Fresh baseline recovery follows.

## Certificate-gated native hybrid experiment - 2026-09-05

Previous goal turn: progress through measured LP rejection and verified
recovery. Current worktree and saved evidence were rechecked; no MATLAB process
was live, and both rejected prepared-query and parametric-LP sources are absent.
The production reset and all user work remain unchanged. No degree reduction,
additional tuning sweep or tolerance change is part of this experiment.

Hypothesis: use Clarabel for a trajectory subproblem only when independent
original-row primal and bounded dual-objective checks pass the previously
required tolerances; otherwise execute the unchanged coneprog subproblem.
This is a certificate-gated hybrid, not the previously rejected standalone
backend. No native infeasibility result becomes a planner rejection: the
original coneprog attempt still executes, including on a certified inconsistent
intermediate model, because the existing planner can derive a valid final motion
from its numerical iterate. Full public validation remains authoritative.
The rejected reduced-accuracy kernel stays rejected under the same bound check.
There are no case detectors, hidden fallbacks, omitted candidate seeds, reduced
horizons, new public options or weakened geometry/kinematic tolerances.

Evidence motivating a hybrid: saved native trajectory kernel times were
0.01-0.14 s versus coneprog 0.09-1.20 s, but the standalone screen stopped at
an independent bound failure. A hybrid may preserve correctness while keeping
enough accepted native solves to improve full cold runtime. Its total runtime
must include conversion, fresh native construction, independent certificates,
failed native work, all coneprog fallbacks and final public validation.
No numerical solver object, factorization, solution or cache crosses requests.

Baseline: HEAD 3d60f83 plus uncommitted coneprog reset; original degree 16/7,
span allocations, saved inputs/options, seed 0, MATLAB R2024b and unchanged six
MATLAB computational threads. Build the C API MEX in an isolated output-owned
folder, with portable Rust tooling there; the runtime itself requires no
Python or Rust toolchain. Keep upstream licenses and exact source revisions.
The implementation and all build artifacts are owned by ignored
output/cold-method-research-20260905/native-hybrid. Prototype planner copies
are isolated in native-hybrid-planner. Strict native acceptance: original
primal residual <=1e-7 and an independently conservative original-objective
gap bound <= original coneprog OptimalityTolerance (1e-6 default), plus finite
values and valid dual-cone membership. Solver-reported convergence alone is
insufficient; an unqualified result falls back without changing any coefficient.
Plane solvers initially remain coneprog; actual native/coneprog calls and
fallback reasons must be explicit in experimental diagnostics.

First verify gateway matrix/sign conventions on analytical LP/equality/SOC
problems without planner timing claims. Then fresh cold original versus hybrid
StaticU full planner+validator, requiring >=20% benefit and original public
arrival/physical/certificate gates. Only a favorable focused result permits
ObstacleAvoidance and exact unchanged failed.mat, then three interleaved cold
repeats and original eight-case >=3x aggregate gate before production adoption.
If focused benefit or correctness fails, remove experiment-owned source,
portable tools and binaries, preserve unfavorable measurements, and recheck
baseline. No commit until the whole objective is verified.

Primary source: https://github.com/oxfordcontrol/Clarabel.cpp (Apache-2.0),
C/C++ wrapper over Clarabel.rs. The preceding standalone numerical failure
remains recorded and is not relabeled a passing result.

## 2026-09-05: Shorter paths with unchanged arrival times

User-supplied `Rogue Examples/pathtoolong.mat` and `pathtoolong2.mat` replayed
unchanged using `offlineSandbox.replayDiagnosisBundle` against 40323d8. Both
were successful and independently valid; this was a travel-quality issue.
The first-passing fixed-clock lobe caused a broad unnecessary tail. Balanced
candidate ranking itself matched its declared objective.

The planner now compares all existing signed peak proposals, then uses a
bounded coordinate refinement of free interior offset-spline coefficients.
The governing coordinate and arrival clock remain fixed. Only a shorter
motion passing unchanged full public validation can replace the incumbent.
The numerical grid has eight intervals plus the starting peak; eight step
levels and two sweeps bound local work. Rejected trials do not prune route
search or establish infeasibility. Diagnostics retain initial axis reports,
final spline offsets, trial/accepted counts, and before/after travel.
This is local improvement, not a globally shortest-path certificate.

Measured MATLAB R2024b unchanged artifact replay:

| Bundle | Original length deg | Final length deg | Original/final duration s | Original/final replay wall s |
|---|---:|---:|---:|---:|
| pathtoolong | 233.058989023 | 230.563196579 | 117.744031226 | 7.499 / 12.129 |
| pathtoolong2 | 242.064492067 | 236.331761713 | 117.250241772 | 2.426 / 7.786 |

Both independent validators pass. First rows are cold within their MATLAB
sessions; wall measurements are observations, not a formal speed comparison.
The customer explicitly accepted more computation but required unchanged
arrival time. Retention gate of >=1 degree shortening in both artifacts,
unchanged arrival and full validity passed. Preliminary enumeration-only
shortening was just 0.338/0.315 degrees; free-offset refinement supplies the
larger benefit. No input, margin, kinematic tolerance, or ranking-policy change.

Verification: 44/44 route-economy, planner-contract, and architecture tests
pass, including unmodified artifact replays, circle, concave static/moving
outlines, reflected progress and near-start barriers. Headless maintained
ObstacleAvoidance and MovingBarrierWait pass independent validation; NoPath
returns expected noValidatedSeed. Actual example metrics appended to
benchmark.csv. No full example sweep or graphical UI verification claimed.
Existing unrelated UI/socket work and user artifacts remain intact.

## 2026-09-05: Concise implementation comments

Reviewed all 99 MATLAB files under +obstacleAvoidance and trajectory.
Simplified 322 comment blocks across 77 files, removing 246 comment lines net.
Removed repeated control-flow narration and replaced vague wording with direct
explanations. Kept numerical assumptions, collision/validation distinctions,
units, public help sections, and third-party license notices.

Compared every file with its pre-edit snapshot: all non-comment lines match,
and public help/license blocks match after normalizing line endings.
No planner behavior changed. No planner examples or runtime tests were rerun
for this comment-only change; benchmark.csv is unchanged.

## 2026-09-05: BMTP-only planner selection

Removed the selectable ruckigWaypoint path from planTrajectory, solveOneSeed,
and solveExactCandidates. TrajectoryMethod remains in the options schema for
saved BMTP requests, but accepts only bmtp; old Ruckig selections raise
planTrajectory:InvalidTrajectoryMethod rather than silently changing methods.
The HTML sandbox already uses BMTP and had no method selector to remove.
Standalone Ruckig utilities and the explicitly enabled ruckigStopAtWaypoints
fallback remain intact. The fallback remains disabled by default.

Updated option, architecture, waypoint-utility and intercept tests. Intercept
coverage now uses BMTP-compatible rest states instead of selecting Ruckig for
nonzero endpoint derivatives. Existing unsupported-derivative checks remain.
MATLAB R2024b: 52/52 tests pass across planner options, planner contract,
architecture, retained Ruckig waypoint composition, and route economy. Both
saved long-path bundles still pass the shortening and independent-validation
gates. No maintained examples were executed for this selection-only change;
benchmark.csv is unchanged. Earlier comment edits and unrelated user work
remain in the working tree.

## 2026-09-05: Explicit planner-stage inputs

Replaced the five-field request wrapper with explicit arguments in scene
preparation, proposal geometry, visibility graph construction, route search,
seed creation, exact candidate solving, seed solving, dynamic seed solving,
and recovery. These nine stage interfaces begin with obstacles, initialState,
goalState, limits, options; stage-specific data follows. Removed duplicate
planning inputs from seedSolveContext and recoveryContext. Generated scene,
graph, candidate and timing records remain separate stage data.

Updated all production callers, direct stage tests, and public help. The
public planTrajectory signature and returned result schema are unchanged.
The zero-input exact-candidate diagnostic call remains supported. Deferred
route-search recovery still distinguishes its optional priorRouteSet argument
at the updated position. No motion/search algorithms or tolerances changed.

MATLAB R2024b: all 52 existing option, planner-contract, architecture, Ruckig
utility and route-economy regressions pass. The added argument-order guard
also passes in the eight-test architecture suite: 53 distinct passing tests.
Checks include static/moving geometry, deferred recovery and both saved
long-path bundles with independent validation. No maintained examples were
executed and benchmark.csv is unchanged. Earlier local comment and BMTP-only
selection edits remain uncommitted alongside this refactor.

## 2026-09-05: Remove the obsolete trajectory-method field

Removed TrajectoryMethod from planner defaults, validation rules, the option
reference, and active callers. BMTP is implicit. Old saved requests follow
the existing unknown-option rule: warn once and ignore the retired field;
resolved options no longer contain it. This supersedes the earlier temporary
compatibility field and InvalidTrajectoryMethod rejection policy.

MATLAB R2024b: 43/43 planner-option, planner-contract, and route-economy tests
pass. Both unmodified saved long-path bundles replay successfully, issue the
expected unknown-field warning, and pass independent validation. No maintained
examples were executed; benchmark.csv is unchanged.

## 2026-09-05: Simplify arrival modes and validation ownership

Earliest arrival is the default, ranking validated motions by arrival time and
then travel length. Fixed arrival remains supported. Removed balanced-arrival
ranking, its savings-rate option, and both sandbox controls for that rate.
Legacy balancedArrival requests warn and migrate to earliestArrival. The two
saved long-path requests still pass their shorter-path and unchanged-clock
regressions. The existing fixed-clock refinement and all motion validators
retain their constraints and tolerances; no global shortest-path claim is made.

Default-only option calls now return constants directly. Removed duplicate
public-option checks in planTrajectory and trusted-input guards in prepared-scene,
proposal-geometry, and exact-candidate stages. The shared option merger trusts
caller-owned defaults. User overrides still receive validation at the boundary;
being an advanced caller does not bypass physical validation.

MATLAB R2024b: 74 distinct tests pass across options (7), planner contract (32),
route economy (5), architecture (8), MATLAB sandbox (11), BMTP engine (7), and
offline bundles (4). Node: 23/23 standalone-page tests pass. Initial runs exposed
obsolete assertions requiring balanced-mode timing repair, the retired 2 deg/s
UI limit, and route-class diagnostics after a validated exact fast path. Those
assertions were updated to the current contracts, preserving independent motion
validation, wait reduction, and the historical travel-plus-duration bound.
The affected failed tests passed on rerun. Option tests passed again after the
last default-validation cleanup. No browser visual verification was performed.

The contract suite invokes exampleTargetExitsObstacle with jerk limits enabled;
it passed but did not retain separate numeric metrics (NaN in benchmark.csv).
A measured rerun passed planning and independent validation, including collision,
kinematic, and applicable certificate checks: polyline 21.742546732 deg, smoothed
length 21.932157017 deg, duration 24 s, wall time 9.9252707 s, goalReached.
Both actual invocations are appended to benchmark.csv. An initial measurement
runner failed before invoking the example because its path was incorrect; the
corrected runner produced the recorded result. Unrelated user bundle changes
remain untouched. Changes are local and have not been pushed.
