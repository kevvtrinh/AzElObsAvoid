# Plan 325 verification

## Evidence scope

- Branch: `plan-325`.
- Verified source commit: `b238e6e`.
- Runtime: MATLAB R2024b Update 4 with Optimization Toolbox.
- Date: 2026-08-19.
- All examples used finite jerk limits.

Each maintained example ran in a separate MATLAB process. The runs were
serial. The headless controls disabled all plots, animation, and pauses.

## Requested policy changes

- All 13 fixed-goal examples now state `GoalTimeMode="earliestArrival"`.
- Five moving-target examples retain their target-time policy. Four use a
  specified intercept time. One uses earliest target intercept.
- `exampleAlternatingSlalom` uses `ElevationInterval_deg=[-5 5]`. This bound
  prevents a route above all three barriers and makes the route alternate.
- `plotAzElMotion` uses the `main` branch visual conventions. Original
  obstacles are red and solid. Protected obstacles are orange and dashed.
  Candidate routes are gray. The selected route is blue and dashed. The
  returned motion is black. The target track is purple. The animation uses
  the light complete path, cyan elapsed path, and red current-state marker.
- The single-U example work limit is 75 seconds. Its first 35-second run
  reached the validation deadline. Its physical request did not change.

## Size checks

| Scope | Files | Physical lines | Limit | Result |
| --- | ---: | ---: | ---: | --- |
| Production MATLAB | 26 | 7,000 | 7,000 hard limit | pass exactly |
| Complete MATLAB tree | 52 | 11,621 | 12,000 hard limit | pass by 379 |
| Complete MATLAB tree | 52 | 11,621 | 10,500 target | **fail by 1,121** |

No production file is longer than 900 lines. The preferred complete-tree
target does not pass.

## Final headless example results

`P/V` means planner success and independent example-validation pass. `C/K/A`
means collision, kinematic, and applicable certificate pass. The duration is
the returned motion duration. A fixed duration is a required target time. It
is not a minimum-time result.

| Example | Goal mode | P/V | Polyline (deg) | Motion (deg) | Duration (s) | C/K/A | Wall (s) | Reason |
| --- | --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | earliest | 1/1 | 16.0425349764 | 16.7453475476 | 12.1834571132 | 1/1/1 | 39.3680526 | `goalReached` |
| `exampleAzElPlanning` | earliest | 1/1 | 11.1521195190 | 11.3034321242 | 7.81726894407 | 1/1/1 | 28.9861455 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.7007215595 | 12.8081112211 | 8.7986387782 | 1/1/1 | 52.7231970 | `goalReached` |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685 | 126.211945429 | 64.5710759977 | 1/1/1 | 31.6790719 | `goalReached` |
| `exampleFourAcceleratingCircles` | fixed target | 1/1 | 24.3633030073 | 27.7125177045 | 22 | 1/1/1 | 81.0022806 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | fixed target | 1/1 | 9.53894054682 | 9.53894054682 | 12 | 1/1/1 | 6.3994823 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | earliest target | 1/1 | 10.0975244491 | 7.34221560094 | 6.27580651627 | 1/1/1 | 10.1208152 | `goalReached` |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10 | 10.1029522435 | 10.5465620376 | 1/1/1 | 40.9986616 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12 | 14.7158492237 | 12.0310423352 | 1/1/1 | 18.1454960 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 63.0848051466 | 63.0848051466 | 25.614496552 | 1/1/1 | 167.6347756 | `goalReached` |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN` | 21.1099172 | `planningTimeLimit` |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.472135955 | 4.472135955 | 5 | 1/1/1 | 15.0995185 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10 | 10 | 15 | 1/1/1 | 17.8362603 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | fixed target | 1/1 | 20.2576694717 | 15.7138346765 | 20.8695652174 | 1/1/1 | 61.3248657 | `goalReached` |
| `exampleTargetExitsObstacle` | fixed target | 1/1 | 19.824386759 | 22.8799304252 | 24 | 1/1/1 | 23.3057664 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 23.8537208838 | 24.369761272 | 22.875124576 | 1/1/1 | 64.9545585 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.9425880405 | 42.4634512333 | 26.4922113988 | 1/1/1 | 50.8781644 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.2394635087 | 26.8344018212 | 7.23988773611 | 1/1/1 | 488.9379664 | `goalReached` |

The extreme U.S. wall time covers the Hawaii, Croatia, and Philippines
sequence. All three region results passed planner and independent validation.

## Display and failure checks

The visible obstacle-free run created two visible figures. Planning and
independent validation passed. The selected-route and timed-motion style
checks passed. It returned a 4.472135955-degree polyline, a
4.47286670355-degree motion, and a 4.61339492501-second duration. Its wall
time was 21.0560088 seconds.

The visible no-path run created the workspace and time-expanded diagnostic
figures. It returned `Success=false`, independent failure validation passed,
and the obstacle-style checks passed. Its wall time was 25.2107197 seconds.
Its reason was `noValidatedSeed`. The separate headless run reached
`planningTimeLimit`. Both reasons are recognized bounded-search outcomes.

## Automated checks

- Full tests: 51 passed, 0 failed, and 0 incomplete.
- Summed MATLAB test duration: 68.390448600 seconds.
- Complete test wall time: 75.096428500 seconds.
- Code Analyzer: 52 MATLAB files and 0 messages.
- Focused no-path plot test: 1 passed.
- Programmatic `main` plot-style checks: passed for success, animation, and
  expected failure.
- `git diff --check`: passed.
- No MATLAB source line is longer than 100 characters.

The first complete test run had 49 passes and one contract-hash failure. The
single-U work-limit change caused the expected source hash to change. The
reviewed hash was updated. The complete rerun passed all 51 tests.

## Runtime warnings and failed attempts

- `exampleAzElPlanning`, `exampleMovingBarrierWait`, and
  `exampleUSOutlineExtremeVisibility` emitted fmincon matrix-conditioning
  warnings. Their final motions passed all independent checks.
- The single-U example failed twice with the old 35-second work limit. The
  second visibility seed reached a 38.5496-second motion, but independent
  validation returned `validationTimeLimit`. A 75-second work limit produced
  the final validated result. Geometry, safety margin, states, and physical
  limits did not change.
- Deadline checks are cooperative. One active nonlinear solve or geometry
  operation cannot be stopped inside MATLAB. The extreme geographic sequence
  took 488.9379664 seconds.

## Known limits and claim

- Spatial and timed proposals use finite samples. They can miss a feasible
  topology.
- Reduced seed geometry can reject a useful proposal. Final validation uses
  the original protected obstacle history.
- The analytic motion stops at geometric waypoints. HS3 is local and can
  return a poor local result.
- Periodic obstacle images are not implemented. Wrapped obstacle requests are
  not supported.
- Optimization Toolbox is required for HS3.

A successful result is an independently validated motion from a finite,
deterministic proposal set. The planner does not claim global route
completeness or global time optimality.
