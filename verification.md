# Plan 325 verification

## Evidence scope

- Branch: `plan-325`.
- Baseline commit for the prepared-obstacle experiment: `4f59472`.
- Verified state: the current uncommitted Plan 325 implementation.
- Runtime: MATLAB R2024b Update 4 with Optimization Toolbox.
- Date: 2026-08-20.
- Every maintained example used a finite jerk limit.

Each maintained example ran in its own MATLAB process. Runs were serial.
Headless controls disabled plots, animation, and pauses.

## Implemented contract changes

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
- Focused jerk-contract tests: 4 passed, 0 failed.

## Downloaded audit artifacts and cleanup decisions

The branch includes `repo_inconsistencies_plan_325.md` and
`repo_cleanup_audit_plan_325.md`. The workspace skill directory includes the
downloaded `repository-cleanup` skill.

The audits describe commit `b845880`, so some findings are stale after this
work. Workspace ownership, verbose behavior, timeout removal, production size,
and jerk routing are now corrected. `certifySeedCorridor` is retained because
the current production validator calls it. `RandomSeed` is retained for public
compatibility because removal would be a breaking result-schema change.
Polynomial sampling remains a measurement-first cleanup candidate. Prepared
dynamic obstacle data is now reused after it passed the 40-circle and
moving-U.S. runtime gates.

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
The no-path row retained its independently validated failure schema and two
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

The sweep initially exposed a pre-planning example-contract failure:
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
(+92) and the maintained MATLAB tree changed from 11,873 to 11,974 (+101).
Production is 269 lines below the user-approved 7,500-line target, so no
performance-based overage allowance is used. The solver shrank from 900 to
885 lines while the two focused internal helpers hold 111 lines.
