# Plan 325 verification

## Evidence scope

- Branch: `plan-325`.
- Verified source commit: `f06fa9c`.
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
| `exampleAlternatingSlalom` | earliest | 1/1 | 16 | 16.691842238 | 12.180624977 | 1/1/1 | 33.4689311 | `goalReached` |
| `exampleAzElPlanning` | earliest | 1/1 | 11.152119519 | 11.303432110 | 7.817268021 | 1/1/1 | 20.3210242 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | earliest | 1/1 | 12.700721560 | 12.808110687 | 8.798640057 | 1/1/1 | 41.6276308 | `goalReached` |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685 | 122.955593942 | 64.556780013 | 1/1/1 | 22.6353983 | `goalReached` |
| `exampleFourAcceleratingCircles` | fixed target | 1/1 | 24.363303007 | 27.712518684 | 22 | 1/1/1 | 65.0622422 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | fixed target | 1/1 | 9.538940547 | 9.538940547 | 12 | 1/1/1 | 3.1102479 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | earliest target | 1/1 | 10.097524449 | 7.342215833 | 6.275807672 | 1/1/1 | 4.6761014 | `goalReached` |
| `exampleMovingBarrierWait` | earliest | 1/1 | 10 | 10.211853154 | 10.545227891 | 1/1/1 | 40.5701264 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | earliest | 1/1 | 12.113593185 | 12.113593185 | 12.293137410 | 1/1/1 | 17.1009844 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 63.084805147 | 63.084805147 | 25.614496552 | 1/1/1 | 127.8696377 | `goalReached` |
| `exampleNoPathAzElMotion` | earliest | 0/1 | `NaN` | `NaN` | `NaN` | `NaN` | 20.3535105 | `noValidatedSeed` |
| `exampleObstacleFreeAzElMotion` | earliest | 1/1 | 4.472135955 | 4.472860956 | 4.613406127 | 1/1/1 | 5.9388048 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10 | 10 | 15 | 1/1/1 | 16.8039567 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | fixed target | 1/1 | 13.341664064 | 15.299430296 | 20.869565217 | 1/1/1 | 27.6978874 | `goalReached` |
| `exampleTargetExitsObstacle` | fixed target | 1/1 | 19.824386759 | 22.879930804 | 24 | 1/1/1 | 12.7454655 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 23.853720884 | 24.302835532 | 22.876124561 | 1/1/1 | 63.9628870 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.942588040 | 42.580183511 | 26.492831986 | 1/1/1 | 41.1340763 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.239463509 | 26.834419733 | 7.239885850 | 1/1/1 | 210.4986503 | `goalReached` |

The extreme U.S. wall time covers the complete maintained geographic case.

## Display and failure checks

The visible obstacle-free run created two visible figures. Planning and
independent validation passed.

The visible no-path run created one search diagnostic figure. It returned
`Success=false`, and independent failure validation passed. Its reason was
`noValidatedSeed`, which matched the separate headless run.

## Automated checks

- Full tests: 52 passed, 0 failed, and 0 incomplete.
- Summed MATLAB test duration: 37.438059200 seconds.
- Full test process wall time: 41.594795500 seconds.
- Code Analyzer: 52 MATLAB files and 0 messages.
- Visible success and no-path checks: passed with two and one figures.
- `git diff --check`: passed.
- No MATLAB source line is longer than 100 characters.

The focused one-obstacle and two-obstacle homology tests passed before the
complete 52-test run.

## Runtime warnings and failed attempts

- An initial final-code `exampleAzElPlanning` run found duplicate normalized
  interpolation parameters during corridor reassociation. The planner now
  removes those numerical duplicates. The repaired example and the complete
  verification passed.
- The two known fmincon matrix-conditioning warning identifiers were hidden in
  the long serial runs to keep logs bounded. Returned motions still passed the
  independent checks.
- MATLAB R2024b returned a file-system startup fault between isolated
  processes. The runtime service was reset before each remaining process.
  No two example processes overlapped, and scenario inputs did not change.
- Deadline checks are cooperative. One active nonlinear solve or geometry
  operation cannot be stopped inside MATLAB. The extreme geographic case took
  210.4986503 seconds.

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
