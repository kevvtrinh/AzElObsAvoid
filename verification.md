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

## Superseded audit artifacts and cleanup decisions

The one-time `repo_inconsistencies_plan_325.md` and
`repo_cleanup_audit_plan_325.md` reports described old commit `b845880` and
were removed after their resolved decisions were preserved here and in
`branch_assessment.md`. Workspace ownership, verbose behavior, timeout removal,
production size, jerk routing, prepared obstacle reuse, and package ownership
now reflect the maintained implementation rather than that historical audit.

`certifySeedCorridor` remains because the production validator calls it.
`RandomSeed` remains for public compatibility because removing it would break
the result schema. Polynomial sampling remains a measurement-first cleanup
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
with that quintic schema and forced zero velocity, acceleration, and jerk at
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
- The focused HS3 diagnostic schema test passed 1 of 1 in 5.4625 seconds.
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
current schema. These environment/reporting failures are not counted as
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
validation before planning because the option contract requires a positive
weight. It was corrected once to the contract-valid 0.01 value; no parameter
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
contracts. Focused U, hairpin, moving/deforming, obstacle-free, and no-path
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
case returned `noValidatedSeed` and passed its failure contract, and every
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
contract. Exact CSV rows are under source
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
`noValidatedSeed` contract. All 17 motion durations exactly match the frozen
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
contract. Every successful polyline length, smoothed length, and duration is
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
validation, and the expected no-path result passed its stable failure contract.
Every successful polyline length, smoothed length, and duration is exact to the
pushed `working-tree-residual-feedback-final` rows within `1e-6`; exact new
rows are appended to `benchmark.csv` under
`working-tree-boundary-support-final`. Measured wall sum was an unfavorable
`205.6452420 s` versus `172.6919951 s` for the pushed evidence, so no runtime
improvement is claimed. The moving/deforming outline and extreme outline were
the largest walls at `61.1979723 s` and `39.7610225 s`.

A visible U-shaped success passed and created three figures with 522 graphics
objects. The visible expected failure passed its example contract and created
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
the four single-caller files own a stable result schema or a distinct algorithm
extracted from an already-large orchestrator. The complete rationale is in
`short_file_rationale.md`.

Text-only checks found zero local Section 0 headers, missing local-function
purpose comments, unexplained internal loops, bare assignment continuations,
code continuations at or below 120 characters, trailing whitespace, or
physical-contract hash mismatches. `git diff --check` passed. Per the user's
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
