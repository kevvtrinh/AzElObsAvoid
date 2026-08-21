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
