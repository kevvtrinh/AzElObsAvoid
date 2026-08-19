# Plan 325 verification

## Evidence scope

- Branch: `plan-325`.
- Final planner-code commit: `c5fb6e908fcf0b704ff9c272027856927a575ded`.
- Final geographic example-budget commit:
  `f1014e796951eef521fc7a18e02bc8f8d2838c15`.
- Runtime: MATLAB R2024b Update 4.
- Nonlinear solver: Optimization Toolbox `fmincon`.
- Date: 2026-08-19.

The final headless matrix used the planner code at `c5fb6e9`. The later
`f1014e7` commit changes only the runtime budget of the full geographic
example. It does not change the planner.

Each maintained example ran in a separate MATLAB process. Runs were serial.
Plots and animation were off. The runner displayed the first MATLAB matrix
conditioning warning. It then suppressed repeated displays of
`MATLAB:nearlySingularMatrix` and `MATLAB:singularMatrix`. This suppression
did not change the solver or its decisions.

## Size checks

| Scope | Files | Physical lines | Limit | Result |
| --- | ---: | ---: | ---: | --- |
| Production MATLAB | 26 | 6,997 | 7,000 hard limit | pass by 3 |
| Complete MATLAB tree | 52 | 11,588 | 12,000 hard limit | pass by 412 |
| Complete MATLAB tree | 52 | 11,588 | 10,500 target | **fail by 1,088** |

The complete-tree target does not pass. No production file is longer than
900 lines. The largest production files are:

| File | Physical lines |
| --- | ---: |
| `+azElInternal/generateAzElTopologySeeds.m` | 900 |
| `+azElInternal/solveAzElHs3.m` | 891 |
| `planAzElMotion.m` | 878 |
| `validateAzElTrajectory.m` | 654 |
| `+azElInternal/buildAzElStopWaypointMotion.m` | 529 |

## Final headless example results

All 18 examples used jerk constraints. `P/V` means planner success and
independent example-validation pass. `C/K/A` means collision, kinematic, and
applicable certificate pass. `First valid` is
`FirstValidatedMotionTime_s`. All successful runs have `C/K/A = 1/1/1`.
All final runs report zero planning-deadline overrun.

The duration column is the returned motion duration. A fixed-arrival duration
is the configured horizon. It is not a measured global minimum.

| Example | Goal mode | P/V | Polyline (deg) | Motion (deg) | Duration (s) | C/K/A | Wall (s) | Motion source | First valid (s) | Reason |
| --- | --- | --- | ---: | ---: | ---: | --- | ---: | --- | ---: | --- |
| `exampleAlternatingSlalom` | fixed | 1/1 | 16.0193197983 | 16.1141335808 | 22 | 1/1/1 | 32.9304142 | `hs3` | 26.7228244 | `goalReached` |
| `exampleAzElPlanning` | fixed | 1/1 | 11.1521195190 | 11.4311025996 | 12 | 1/1/1 | 14.5619300 | `hs3` | 12.3027110 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | fixed | 1/1 | 12.7007215595 | 12.9224772123 | 15 | 1/1/1 | 37.0002224 | `hs3` | 36.1259540 | `goalReached` |
| `exampleFortyMovingCircleGrid` | earliest | 1/1 | 110.807929685 | 126.216883774 | 64.5557855485 | 1/1/1 | 25.6657512 | `hs3` | 3.7957388 | `goalReached` |
| `exampleFourAcceleratingCircles` | fixed | 1/1 | 24.3633030073 | 27.7125177045 | 22 | 1/1/1 | 73.6208546 | `hs3` | 67.0239622 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | fixed | 1/1 | 9.53894054682 | 9.53894054682 | 12 | 1/1/1 | 5.0729102 | `analyticStopWaypoint` | 0.5027143 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | earliest | 1/1 | 10.0975244491 | 7.34221560094 | 6.27580651627 | 1/1/1 | 8.6248889 | `hs3` | 8.2246734 | `goalReached` |
| `exampleMovingBarrierWait` | fixed | 1/1 | 10 | 10 | 12 | 1/1/1 | 17.5995991 | `analyticStopWaypoint` | 2.0269156 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | fixed | 1/1 | 12 | 12.0023764645 | 15 | 1/1/1 | 16.5597066 | `hs3` | 3.3171748 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | earliest | 1/1 | 63.0848051466 | 63.0848051466 | 25.614496552 | 1/1/1 | 138.9406737 | `analyticStopWaypoint` | 48.4980758 | `goalReached` |
| `exampleNoPathAzElMotion` | fixed | 0/1 | `NaN` | `NaN` | `NaN` | `NaN` | 20.3882423 | none | `NaN` | `noValidatedSeed` |
| `exampleObstacleFreeAzElMotion` | fixed | 1/1 | 4.472135955 | 4.472135955 | 8 | 1/1/1 | 3.2369726 | `analyticStopWaypoint` | 0.4773441 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | earliest | 1/1 | 10 | 10 | 15 | 1/1/1 | 17.1437539 | `analyticStopWaypoint` | 2.1718114 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | fixed | 1/1 | 20.285321411 | 14.1448495404 | 20.8695652174 | 1/1/1 | 60.5649175 | `hs3` | 11.5670302 | `goalReached` |
| `exampleTargetExitsObstacle` | fixed | 1/1 | 19.824386759 | 22.8799304252 | 24 | 1/1/1 | 17.4438625 | `hs3` | 16.4805158 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | earliest | 1/1 | 23.8537208838 | 24.303112655 | 22.8761003411 | 1/1/1 | 62.7020007 | `hs3` | 3.8510137 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | earliest | 1/1 | 34.9425880405 | 34.9425880405 | 38.5495931039 | 1/1/1 | 62.3235920 | `analyticStopWaypoint` | 28.2061179 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | earliest | 1/1 | 22.2394635087 | 26.8344018212 | 7.23988773611 | 1/1/1 | 463.8727127 | `hs3` | 63.6080467 | `goalReached` |

The final geographic row reports the last region motion and the wall time for
the complete Hawaii, Croatia, and Philippines sequence. All three regions
passed planner and independent validation checks.

## Display and failure diagnostics

The zero-input `exampleObstacleFreeAzElMotion()` call ran with visible default
settings. It passed planning and independent validation. It returned the same
4.472135955-degree motion and 8-second duration as the headless run. It took
12.5776286 seconds and created four figures.

The visible no-path run returned `Success = false` and
`TerminationReason = "noValidatedSeed"`. Independent failure validation
passed. The run took 24.8037494 seconds. It created one failure-diagnostic
figure with three expanded states and four rejected transitions.

## Automated checks

- Focused tests: 50 passed, 0 failed, and 0 incomplete. The summed MATLAB
  test duration was 56.414948200 seconds.
- Code Analyzer: 52 MATLAB files checked and 0 messages returned.
- Every maintained headless example completed in its own MATLAB process.
- The visible default example passed.
- The visible expected-failure diagnostic passed.
- No final run reported a planning-deadline overrun.

The tests cover the analytic stop-at-waypoint constructor, its per-edge
duration certificate, polynomial evaluation, fixed and earliest arrival,
moving targets, static and moving obstacles, safety provenance, continuous
collision checks, endpoint events, state and coefficient agreement, finite
jerk, deadline results, search diagnostics, and example contracts.

One numerical polynomial-equivalence test first used a tolerance of `1e-10`.
It failed with a maximum difference of `6.96e-9`. The test passed at `1e-8`.
This is tighter than the `1e-7` constraint tolerance. Feasibility, termination
reason, motion time, and jerk cost did not change.

## Exact comparison runs

These are single recorded runs. They are not repeated performance studies.
The spatial references used jerk constraints and the same physical scenario.
The Plan 502 row is the recorded Plan 502 result for the two-U case.

| Case and source | Polyline (deg) | Motion (deg) | Duration (s) | Wall (s) |
| --- | ---: | ---: | ---: | ---: |
| 40 circles, Plan 325 | 110.807929685 | 126.216883774 | 64.5557855485 | 25.6657512 |
| 40 circles, spatial-jerk reference | 110.297882 | 110.232980 | 77.022740 | 40.793195 |
| Moving U.S., Plan 325 | 63.0848051466 | 63.0848051466 | 25.614496552 | 138.9406737 |
| Moving U.S., spatial-jerk reference | 62.655333 | 62.652193 | 29.667632 | 50.403850 |
| Large U, Plan 325 | 34.9425880405 | 34.9425880405 | 38.5495931039 | 62.3235920 |
| Large U, spatial-jerk reference | 35.030136 | 35.012080 | 43.976516 | 1.546111 |
| Two opposing U obstacles, Plan 325 | 23.8537208838 | 24.303112655 | 22.8761003411 | 62.7020007 |
| Two opposing U obstacles, Plan 502 | 24.035785 | 24.298600 | 22.876125 | 91.218125 |

Plan 325 finds a shorter-duration 40-circle motion and takes less wall time
than the spatial reference. Its returned motion is longer. For the moving U.S.
case, Plan 325 finds a shorter-duration motion, but its wall time is much
longer. For the large U, Plan 325 improves the route and motion duration by a
small amount, but its wall time is much longer. For the two-U case, Plan 325
has almost the same returned motion and duration as Plan 502, with less wall
time.

## Failed diagnostic iterations

Failed and replaced runs are not final evidence. They explain the changes:

1. An early moving-U.S. run exhausted a 360-second budget during seed
   generation. It did not leave time for motion validation. Seed-stage work
   limits and stage ordering corrected this failure.
2. An early 40-circle run reserved a timed-search slot that dense-query work
   could not use. The dense work gate stopped this reservation and kept the
   available spatial seed.
3. Before the certified per-edge-duration change, the moving-U.S. case passed
   in 264.9882634 seconds. Its motion took 52.2035772752 seconds. The final
   code passed in 138.9406737 seconds with a 25.614496552-second motion.
4. An early opening-U attempt returned a 34.9426-degree route, but independent
   example validation failed. The timed-wait correction produced the final
   10-degree motion and a validation pass.
5. The geographic sequence failed in Hawaii twice with a 120-second region
   budget. An isolated 180-second run passed. The final sequence used the
   explicit full-region budget from `f1014e7`, and all three regions passed.
6. The first numerical equivalence threshold of `1e-10` failed because the
   observed difference was `6.96e-9`. The final `1e-8` threshold passed and
   remains below the planner constraint tolerance.

`exampleAzElPlanning` emitted repeated matrix-conditioning warnings from
`fmincon`. The final motion passed all independent checks. The runner
suppressed only repeated warning display after the first warning.

## Known limits and claim

- Spatial and time-layer proposal searches use finite samples. A finite seed
  set can miss a feasible topology.
- Dense moving histories can exceed the timed-query work limit. The planner
  then omits that timed seed family and reports incomplete search coverage.
- Conservative dense envelopes and obstacle clusters can reject a useful
  seed. Final validation uses the original protected obstacle history.
- The analytic first motion stops at geometric waypoints. It does not preserve
  through-velocity at those points. It can be slower than a smooth motion.
- Nonzero endpoint derivatives and earliest moving-goal intercepts require the
  HS3 stage.
- HS3 uses a frozen local corridor and local `fmincon` optimization. It can
  reject a feasible route or converge to a poor local result.
- Deadline checks occur between bounded operations. A running solver or
  geometry operation is not preempted. The result reports observed overrun.
- Periodic obstacle images are not implemented. Azimuth wrapping is supported
  only for an obstacle-free fixed-position goal.
- The slow geographic result shows that dense or complex geometry still has a
  high runtime cost.
- Optimization Toolbox is required for HS3.

A successful result is an independently validated motion from the finite
deterministic proposal set. The planner does not claim global route
completeness, global time optimality, or proof that a failed proposal set means
that no feasible trajectory exists.
