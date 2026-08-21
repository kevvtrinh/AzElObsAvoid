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
