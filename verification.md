# Plan 325 verification

## Evidence scope

- Branch: `plan-325`.
- Verified source commit: `6ddabac`.
- Runtime: MATLAB R2024b Update 4 with Optimization Toolbox.
- Date: 2026-08-19 through 2026-08-20.
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
| Complete MATLAB tree | 52 | 11,653 | 12,000 hard limit | pass by 347 |
| Complete MATLAB tree | 52 | 11,653 | 10,500 target | **fail by 1,153** |

No production file is longer than 900 lines. The preferred complete-tree
target does not pass.

## Final headless example results

`P/V` means planner success and independent example-validation pass. `C/K/A`
means collision, kinematic, and applicable certificate pass. The duration is
the returned motion duration. A fixed duration is a required target time. It
is not a minimum-time result.

| Example | Goal mode | P/V | Polyline (deg) | Motion (deg) | Duration (s) | C/K/A | Wall (s) | Reason |
| --- | --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | earliest | 1/1 | 16.0193197983 | 16.7248866356 | 12.1804707715 | 1/1/1 | 39.5196278 | `goalReached` |
| `exampleAzElPlanning` | earliest | 1/1 | 11.1521195190 | 11.3034321229 | 7.81726894407 | 1/1/1 | 28.4886087 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.7007215595 | 12.8081112211 | 8.7986387782 | 1/1/1 | 46.6578609 | `goalReached` |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685 | 126.216929774 | 64.5557806844 | 1/1/1 | 24.0566557 | `goalReached` |
| `exampleFourAcceleratingCircles` | fixed target | 1/1 | 24.3633030073 | 27.7125177045 | 22 | 1/1/1 | 107.5521149 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | fixed target | 1/1 | 9.53894054682 | 9.53894054682 | 12 | 1/1/1 | 5.1742519 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | earliest target | 1/1 | 10.0975244491 | 7.34221560094 | 6.27580651627 | 1/1/1 | 8.1218220 | `goalReached` |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10 | 10.1651478652 | 10.5442278948 | 1/1/1 | 40.7034441 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12.1135931849 | 12.1135931849 | 12.2931374101 | 1/1/1 | 17.3012810 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 63.0848051466 | 63.0848051466 | 25.614496552 | 1/1/1 | 140.0016750 | `goalReached` |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN` | 20.7743080 | `noValidatedSeed` |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.472135955 | 4.47286095255 | 4.61340725517 | 1/1/1 | 15.0402003 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10 | 10 | 15 | 1/1/1 | 17.1663490 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | fixed target | 1/1 | 20.2576694717 | 15.5645053728 | 20.8695652174 | 1/1/1 | 60.5651788 | `goalReached` |
| `exampleTargetExitsObstacle` | fixed target | 1/1 | 19.824386759 | 22.8799304252 | 24 | 1/1/1 | 16.5908680 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 24.035784715 | 24.3888467003 | 22.875114336 | 1/1/1 | 64.9563617 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.9425880405 | 34.9425880405 | 38.5495931039 | 1/1/1 | 68.5958720 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.2394635087 | 26.8344018212 | 7.23988773611 | 1/1/1 | 464.6851317 | `goalReached` |

The extreme U.S. wall time covers the Hawaii, Croatia, and Philippines
sequence. All three region results passed planner and independent validation.

## Display and failure checks

The visible obstacle-free run created three visible figures. Planning and
independent validation passed. Its wall time was 16.9215069 seconds.

The visible no-path run created the workspace and time-expanded diagnostic
figures. It returned `Success=false`, and independent failure validation
passed. Its wall time was 23.9859971 seconds. Its reason was
`noValidatedSeed`, which matched the separate headless run.

## Automated checks

- Full tests: 52 passed, 0 failed, and 0 incomplete.
- Summed MATLAB test duration: 65.740641700 seconds.
- Code Analyzer: 52 MATLAB files and 0 messages.
- Visible success and no-path checks: passed with three and two figures.
- `git diff --check`: passed.
- No MATLAB source line is longer than 100 characters.

The focused one-obstacle and two-obstacle homology tests passed before the
complete 52-test run.

## Runtime warnings and failed attempts

- `exampleAzElPlanning` emitted repeated fmincon matrix-conditioning warnings.
  The final motion passed all independent checks. The two related MATLAB
  warning identifiers were hidden in later long runs to keep logs bounded.
- MATLAB R2024b returned a file-system startup fault between isolated
  processes. The runtime service was reset before each remaining process.
  No two example processes overlapped, and scenario inputs did not change.
- Deadline checks are cooperative. One active nonlinear solve or geometry
  operation cannot be stopped inside MATLAB. The extreme geographic sequence
  took 464.6851317 seconds.

## Known limits and claim

- Spatial and timed proposals use finite samples. They can miss a feasible
  topology. The 2-D homology signature does not classify continuous
  Az/El/time paths.
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
