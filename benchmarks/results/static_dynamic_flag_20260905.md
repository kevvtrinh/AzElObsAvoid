# Static scenes through the dynamic planning flag

**Retain static-scene detection.** Forcing the dynamic branch increases runtime substantially on several scenes and sometimes returns a longer or later motion. All successful timed runs pass independent validation.

Measured against 4344795 plus the working edits frozen in the source manifest. No production flag was changed. Default finite jerk limits; original goal policies; graphics off; RNG seed 0; one warm-up per mode, then three interleaved measured runs. Identical 180-second cancellation budgets. Times below measure the public planner call and exclude scene construction, record serialization, and the additional independent validator. Moving-target examples retain their target setup; these timings isolate the resulting public trajectory-planner calls.

## Planner runtime and result

| Example / scene | Normal median [min–max] s | Forced dynamic median [min–max] s | Valid normal / dynamic | Motion duration normal → dynamic s | Smoothed length normal → dynamic deg |
|---|---:|---:|---:|---:|---:|
| ObstacleFree#1 | 0.020 [0.012–0.131] | 0.047 [0.015–0.190] | 3/3 / 3/3 | 4.531 → 4.531 | 4.472 → 4.472 |
| ObstacleAvoidance#1 | 1.418 [1.271–1.708] | 1.498 [1.394–1.785] | 3/3 / 3/3 | 7.565 → 7.565 | 11.441 → 11.441 |
| StaticUShapedObstacle#1 | 9.344 [8.526–9.534] | 9.022 [7.258–9.742] | 3/3 / 3/3 | 20.850 → 20.955 | 39.384 → 40.103 |
| AlternatingSlalom#1 | 4.057 [4.042–4.133] | 4.052 [3.971–6.137] | 3/3 / 3/3 | 10.541 → 10.541 | 16.339 → 16.339 |
| DenseConcaveObstacle#1 | 5.854 [5.678–7.128] | 6.647 [5.387–7.349] | 3/3 / 3/3 | 8.500 → 8.500 | 12.768 → 12.768 |
| NoPath#1 | 0.567 [0.430–0.614] | 0.204 [0.193–0.355] | 0/3 / 0/3 | NaN → NaN | NaN → NaN |
| TwoOpposingUVisibilityGraph#1 | 2.402 [2.072–2.609] | 2.372 [2.357–2.853] | 3/3 / 3/3 | 22.101 → 22.101 | 24.705 → 24.733 |
| TargetExitsObstacle#1 | 7.746 [6.352–8.160] | 13.293 [11.575–13.669] | 3/3 / 3/3 | 24.000 → 24.000 | 20.685 → 21.940 |
| StraightTargetAlternatingOcclusion#1 | 2.862 [2.743–2.994] | 11.087 [9.910–11.349] | 3/3 / 3/3 | 20.870 → 20.870 | 13.610 → 13.619 |
| InterceptMovingTargetAtSetTime#1 | 0.030 [0.020–0.036] | 0.026 [0.019–0.047] | 3/3 / 3/3 | 12.000 → 12.000 | 9.539 → 9.539 |
| InterceptMovingTargetEarliest#1 | 0.022 [0.021–0.036] | 0.029 [0.027–0.043] | 3/3 / 3/3 | 6.111 → 6.111 | 7.309 → 7.309 |
| USOutlineExtremeVisibility#1 | 13.376 [13.282–13.635] | 12.796 [11.568–13.996] | 3/3 / 3/3 | 4.314 → 4.314 | 12.920 → 12.920 |
| USOutlineExtremeVisibility#2 | 3.009 [2.777–4.374] | 13.715 [13.565–15.473] | 3/3 / 3/3 | 3.268 → 3.268 | 7.632 → 7.632 |
| USOutlineExtremeVisibility#3 | 13.437 [12.902–13.857] | 115.169 [98.229–121.865] | 3/3 / 3/3 | 5.796 → 6.216 | 23.354 → 23.953 |

## Executed branches

Counts are per measured planner call. Changing the scene flag does not disable exact-motion early exits. Dynamic seed solving can use a conservative static projection; a dynamic call is not proof that the timed-cell kernel ran.

| Example / scene | Normal static / dynamic / timed-search calls | Forced static / dynamic / timed-search calls | Selected source normal → dynamic | Termination normal → dynamic |
|---|---|---|---|---|
| ObstacleFree#1 | 0 / 0 / 0 | 0 / 0 / 0 | directRestToRest → directRestToRest | goalReached → goalReached |
| ObstacleAvoidance#1 | 2 / 0 / 0 | 0 / 2 / 1 | visibilityGraph → timeExpandedVisibilityGraph | goalReached → goalReached |
| StaticUShapedObstacle#1 | 2 / 0 / 0 | 0 / 2 / 1 | visibilityGraph → timeExpandedVisibilityGraph | goalReached → goalReached |
| AlternatingSlalom#1 | 2 / 0 / 0 | 0 / 2 / 1 | visibilityGraph → timeExpandedVisibilityGraph | goalReached → goalReached |
| DenseConcaveObstacle#1 | 0 / 0 / 0 | 0 / 0 / 0 | fixedClockLateralExcursion → fixedClockLateralExcursion | goalReached → goalReached |
| NoPath#1 | 1 / 0 / 0 | 0 / 1 / 1 |  →  | noValidatedSeed → unsupportedTimedMultiWaypointRoute |
| TwoOpposingUVisibilityGraph#1 | 2 / 0 / 0 | 0 / 2 / 1 | visibilityGraph → timeExpandedVisibilityGraph | goalReached → goalReached |
| TargetExitsObstacle#1 | 2 / 0 / 0 | 0 / 2 / 1 | directVisibilityEdge → visibilityGraph | goalReached → goalReached |
| StraightTargetAlternatingOcclusion#1 | 2 / 0 / 0 | 0 / 2 / 1 | directVisibilityEdge → timeExpandedVisibilityGraph | goalReached → goalReached |
| InterceptMovingTargetAtSetTime#1 | 0 / 0 / 0 | 0 / 0 / 0 | directRestToRest → directRestToRest | goalReached → goalReached |
| InterceptMovingTargetEarliest#1 | 0 / 0 / 0 | 0 / 0 / 0 | directRestToRest → directRestToRest | goalReached → goalReached |
| USOutlineExtremeVisibility#1 | 0 / 0 / 0 | 0 / 0 / 0 | fixedClockLateralExcursion → fixedClockLateralExcursion | goalReached → goalReached |
| USOutlineExtremeVisibility#2 | 2 / 0 / 0 | 0 / 2 / 1 | visibilityGraph → timeExpandedVisibilityGraph | goalReached → goalReached |
| USOutlineExtremeVisibility#3 | 2 / 0 / 0 | 0 / 2 / 1 | visibilityGraph → timeExpandedVisibilityGraph | goalReached → goalReached |

## Full run metrics

Includes warm-ups (repeat 0), failures, and all geographic subcases. Jerk limits are enabled for every run. C/K/P denotes collision, kinematic, and applicable continuous polynomial certificate checks; false checks on expected no-path results do not denote an unsafe returned success. NaN means unavailable.

| Example / scene | Mode | Repeat | Planner / independent validation | Polyline deg | Smoothed deg | Duration s | C/K/P | Wall s | Termination |
|---|---|---:|---|---:|---:|---:|---|---:|---|
| ObstacleFree#1 | normal | 0 | true/true | 4.472 | 4.472 | 4.531 | 1/1/1 | 1.394 | goalReached |
| ObstacleFree#1 | forcedDynamic | 0 | true/true | 4.472 | 4.472 | 4.531 | 1/1/1 | 0.141 | goalReached |
| ObstacleFree#1 | normal | 1 | true/true | 4.472 | 4.472 | 4.531 | 1/1/1 | 0.131 | goalReached |
| ObstacleFree#1 | forcedDynamic | 1 | true/true | 4.472 | 4.472 | 4.531 | 1/1/1 | 0.047 | goalReached |
| ObstacleFree#1 | forcedDynamic | 2 | true/true | 4.472 | 4.472 | 4.531 | 1/1/1 | 0.190 | goalReached |
| ObstacleFree#1 | normal | 2 | true/true | 4.472 | 4.472 | 4.531 | 1/1/1 | 0.012 | goalReached |
| ObstacleFree#1 | normal | 3 | true/true | 4.472 | 4.472 | 4.531 | 1/1/1 | 0.020 | goalReached |
| ObstacleFree#1 | forcedDynamic | 3 | true/true | 4.472 | 4.472 | 4.531 | 1/1/1 | 0.015 | goalReached |
| ObstacleAvoidance#1 | normal | 0 | true/true | 11.152 | 11.441 | 7.565 | 1/1/1 | 10.213 | goalReached |
| ObstacleAvoidance#1 | forcedDynamic | 0 | true/true | 11.152 | 11.441 | 7.565 | 1/1/1 | 3.072 | goalReached |
| ObstacleAvoidance#1 | normal | 1 | true/true | 11.152 | 11.441 | 7.565 | 1/1/1 | 1.708 | goalReached |
| ObstacleAvoidance#1 | forcedDynamic | 1 | true/true | 11.152 | 11.441 | 7.565 | 1/1/1 | 1.394 | goalReached |
| ObstacleAvoidance#1 | forcedDynamic | 2 | true/true | 11.152 | 11.441 | 7.565 | 1/1/1 | 1.785 | goalReached |
| ObstacleAvoidance#1 | normal | 2 | true/true | 11.152 | 11.441 | 7.565 | 1/1/1 | 1.271 | goalReached |
| ObstacleAvoidance#1 | normal | 3 | true/true | 11.152 | 11.441 | 7.565 | 1/1/1 | 1.418 | goalReached |
| ObstacleAvoidance#1 | forcedDynamic | 3 | true/true | 11.152 | 11.441 | 7.565 | 1/1/1 | 1.498 | goalReached |
| StaticUShapedObstacle#1 | normal | 0 | true/true | 34.943 | 39.384 | 20.850 | 1/1/1 | 9.395 | goalReached |
| StaticUShapedObstacle#1 | forcedDynamic | 0 | true/true | 34.923 | 40.103 | 20.955 | 1/1/1 | 9.281 | goalReached |
| StaticUShapedObstacle#1 | normal | 1 | true/true | 34.943 | 39.384 | 20.850 | 1/1/1 | 8.526 | goalReached |
| StaticUShapedObstacle#1 | forcedDynamic | 1 | true/true | 34.923 | 40.103 | 20.955 | 1/1/1 | 7.258 | goalReached |
| StaticUShapedObstacle#1 | forcedDynamic | 2 | true/true | 34.923 | 40.103 | 20.955 | 1/1/1 | 9.022 | goalReached |
| StaticUShapedObstacle#1 | normal | 2 | true/true | 34.943 | 39.384 | 20.850 | 1/1/1 | 9.534 | goalReached |
| StaticUShapedObstacle#1 | normal | 3 | true/true | 34.943 | 39.384 | 20.850 | 1/1/1 | 9.344 | goalReached |
| StaticUShapedObstacle#1 | forcedDynamic | 3 | true/true | 34.923 | 40.103 | 20.955 | 1/1/1 | 9.742 | goalReached |
| AlternatingSlalom#1 | normal | 0 | true/true | 16.019 | 16.339 | 10.541 | 1/1/1 | 6.744 | goalReached |
| AlternatingSlalom#1 | forcedDynamic | 0 | true/true | 16.019 | 16.339 | 10.541 | 1/1/1 | 4.653 | goalReached |
| AlternatingSlalom#1 | normal | 1 | true/true | 16.019 | 16.339 | 10.541 | 1/1/1 | 4.057 | goalReached |
| AlternatingSlalom#1 | forcedDynamic | 1 | true/true | 16.019 | 16.339 | 10.541 | 1/1/1 | 3.971 | goalReached |
| AlternatingSlalom#1 | forcedDynamic | 2 | true/true | 16.019 | 16.339 | 10.541 | 1/1/1 | 4.052 | goalReached |
| AlternatingSlalom#1 | normal | 2 | true/true | 16.019 | 16.339 | 10.541 | 1/1/1 | 4.133 | goalReached |
| AlternatingSlalom#1 | normal | 3 | true/true | 16.019 | 16.339 | 10.541 | 1/1/1 | 4.042 | goalReached |
| AlternatingSlalom#1 | forcedDynamic | 3 | true/true | 16.019 | 16.339 | 10.541 | 1/1/1 | 6.137 | goalReached |
| DenseConcaveObstacle#1 | normal | 0 | true/true | 12.768 | 12.768 | 8.500 | 1/1/1 | 8.167 | goalReached |
| DenseConcaveObstacle#1 | forcedDynamic | 0 | true/true | 12.768 | 12.768 | 8.500 | 1/1/1 | 8.239 | goalReached |
| DenseConcaveObstacle#1 | normal | 1 | true/true | 12.768 | 12.768 | 8.500 | 1/1/1 | 5.678 | goalReached |
| DenseConcaveObstacle#1 | forcedDynamic | 1 | true/true | 12.768 | 12.768 | 8.500 | 1/1/1 | 5.387 | goalReached |
| DenseConcaveObstacle#1 | forcedDynamic | 2 | true/true | 12.768 | 12.768 | 8.500 | 1/1/1 | 7.349 | goalReached |
| DenseConcaveObstacle#1 | normal | 2 | true/true | 12.768 | 12.768 | 8.500 | 1/1/1 | 7.128 | goalReached |
| DenseConcaveObstacle#1 | normal | 3 | true/true | 12.768 | 12.768 | 8.500 | 1/1/1 | 5.854 | goalReached |
| DenseConcaveObstacle#1 | forcedDynamic | 3 | true/true | 12.768 | 12.768 | 8.500 | 1/1/1 | 6.647 | goalReached |
| NoPath#1 | normal | 0 | false/false | NaN | NaN | NaN | 0/0/0 | 0.798 | noValidatedSeed |
| NoPath#1 | forcedDynamic | 0 | false/false | NaN | NaN | NaN | 0/0/0 | 0.535 | unsupportedTimedMultiWaypointRoute |
| NoPath#1 | normal | 1 | false/false | NaN | NaN | NaN | 0/0/0 | 0.567 | noValidatedSeed |
| NoPath#1 | forcedDynamic | 1 | false/false | NaN | NaN | NaN | 0/0/0 | 0.193 | unsupportedTimedMultiWaypointRoute |
| NoPath#1 | forcedDynamic | 2 | false/false | NaN | NaN | NaN | 0/0/0 | 0.355 | unsupportedTimedMultiWaypointRoute |
| NoPath#1 | normal | 2 | false/false | NaN | NaN | NaN | 0/0/0 | 0.614 | noValidatedSeed |
| NoPath#1 | normal | 3 | false/false | NaN | NaN | NaN | 0/0/0 | 0.430 | noValidatedSeed |
| NoPath#1 | forcedDynamic | 3 | false/false | NaN | NaN | NaN | 0/0/0 | 0.204 | unsupportedTimedMultiWaypointRoute |
| TwoOpposingUVisibilityGraph#1 | normal | 0 | true/true | 24.036 | 24.705 | 22.101 | 1/1/1 | 2.838 | goalReached |
| TwoOpposingUVisibilityGraph#1 | forcedDynamic | 0 | true/true | 24.027 | 24.733 | 22.101 | 1/1/1 | 2.124 | goalReached |
| TwoOpposingUVisibilityGraph#1 | normal | 1 | true/true | 24.036 | 24.705 | 22.101 | 1/1/1 | 2.402 | goalReached |
| TwoOpposingUVisibilityGraph#1 | forcedDynamic | 1 | true/true | 24.027 | 24.733 | 22.101 | 1/1/1 | 2.853 | goalReached |
| TwoOpposingUVisibilityGraph#1 | forcedDynamic | 2 | true/true | 24.027 | 24.733 | 22.101 | 1/1/1 | 2.357 | goalReached |
| TwoOpposingUVisibilityGraph#1 | normal | 2 | true/true | 24.036 | 24.705 | 22.101 | 1/1/1 | 2.072 | goalReached |
| TwoOpposingUVisibilityGraph#1 | normal | 3 | true/true | 24.036 | 24.705 | 22.101 | 1/1/1 | 2.609 | goalReached |
| TwoOpposingUVisibilityGraph#1 | forcedDynamic | 3 | true/true | 24.027 | 24.733 | 22.101 | 1/1/1 | 2.372 | goalReached |
| TargetExitsObstacle#1 | normal | 0 | true/true | 20.136 | 20.685 | 24.000 | 1/1/1 | 6.764 | goalReached |
| TargetExitsObstacle#1 | forcedDynamic | 0 | true/true | 21.743 | 21.940 | 24.000 | 1/1/1 | 13.481 | goalReached |
| TargetExitsObstacle#1 | normal | 1 | true/true | 20.136 | 20.685 | 24.000 | 1/1/1 | 6.352 | goalReached |
| TargetExitsObstacle#1 | forcedDynamic | 1 | true/true | 21.743 | 21.940 | 24.000 | 1/1/1 | 13.293 | goalReached |
| TargetExitsObstacle#1 | forcedDynamic | 2 | true/true | 21.743 | 21.940 | 24.000 | 1/1/1 | 11.575 | goalReached |
| TargetExitsObstacle#1 | normal | 2 | true/true | 20.136 | 20.685 | 24.000 | 1/1/1 | 7.746 | goalReached |
| TargetExitsObstacle#1 | normal | 3 | true/true | 20.136 | 20.685 | 24.000 | 1/1/1 | 8.160 | goalReached |
| TargetExitsObstacle#1 | forcedDynamic | 3 | true/true | 21.743 | 21.940 | 24.000 | 1/1/1 | 13.669 | goalReached |
| StraightTargetAlternatingOcclusion#1 | normal | 0 | true/true | 13.342 | 13.610 | 20.870 | 1/1/1 | 3.433 | goalReached |
| StraightTargetAlternatingOcclusion#1 | forcedDynamic | 0 | true/true | 15.103 | 13.619 | 20.870 | 1/1/1 | 11.977 | goalReached |
| StraightTargetAlternatingOcclusion#1 | normal | 1 | true/true | 13.342 | 13.610 | 20.870 | 1/1/1 | 2.743 | goalReached |
| StraightTargetAlternatingOcclusion#1 | forcedDynamic | 1 | true/true | 15.103 | 13.619 | 20.870 | 1/1/1 | 9.910 | goalReached |
| StraightTargetAlternatingOcclusion#1 | forcedDynamic | 2 | true/true | 15.103 | 13.619 | 20.870 | 1/1/1 | 11.087 | goalReached |
| StraightTargetAlternatingOcclusion#1 | normal | 2 | true/true | 13.342 | 13.610 | 20.870 | 1/1/1 | 2.994 | goalReached |
| StraightTargetAlternatingOcclusion#1 | normal | 3 | true/true | 13.342 | 13.610 | 20.870 | 1/1/1 | 2.862 | goalReached |
| StraightTargetAlternatingOcclusion#1 | forcedDynamic | 3 | true/true | 15.103 | 13.619 | 20.870 | 1/1/1 | 11.349 | goalReached |
| InterceptMovingTargetAtSetTime#1 | normal | 0 | true/true | 9.539 | 9.539 | 12.000 | 1/1/1 | 0.065 | goalReached |
| InterceptMovingTargetAtSetTime#1 | forcedDynamic | 0 | true/true | 9.539 | 9.539 | 12.000 | 1/1/1 | 0.035 | goalReached |
| InterceptMovingTargetAtSetTime#1 | normal | 1 | true/true | 9.539 | 9.539 | 12.000 | 1/1/1 | 0.020 | goalReached |
| InterceptMovingTargetAtSetTime#1 | forcedDynamic | 1 | true/true | 9.539 | 9.539 | 12.000 | 1/1/1 | 0.026 | goalReached |
| InterceptMovingTargetAtSetTime#1 | forcedDynamic | 2 | true/true | 9.539 | 9.539 | 12.000 | 1/1/1 | 0.047 | goalReached |
| InterceptMovingTargetAtSetTime#1 | normal | 2 | true/true | 9.539 | 9.539 | 12.000 | 1/1/1 | 0.030 | goalReached |
| InterceptMovingTargetAtSetTime#1 | normal | 3 | true/true | 9.539 | 9.539 | 12.000 | 1/1/1 | 0.036 | goalReached |
| InterceptMovingTargetAtSetTime#1 | forcedDynamic | 3 | true/true | 9.539 | 9.539 | 12.000 | 1/1/1 | 0.019 | goalReached |
| InterceptMovingTargetEarliest#1 | normal | 0 | true/true | 7.309 | 7.309 | 6.111 | 1/1/1 | 0.077 | goalReached |
| InterceptMovingTargetEarliest#1 | forcedDynamic | 0 | true/true | 7.309 | 7.309 | 6.111 | 1/1/1 | 0.027 | goalReached |
| InterceptMovingTargetEarliest#1 | normal | 1 | true/true | 7.309 | 7.309 | 6.111 | 1/1/1 | 0.036 | goalReached |
| InterceptMovingTargetEarliest#1 | forcedDynamic | 1 | true/true | 7.309 | 7.309 | 6.111 | 1/1/1 | 0.043 | goalReached |
| InterceptMovingTargetEarliest#1 | forcedDynamic | 2 | true/true | 7.309 | 7.309 | 6.111 | 1/1/1 | 0.029 | goalReached |
| InterceptMovingTargetEarliest#1 | normal | 2 | true/true | 7.309 | 7.309 | 6.111 | 1/1/1 | 0.022 | goalReached |
| InterceptMovingTargetEarliest#1 | normal | 3 | true/true | 7.309 | 7.309 | 6.111 | 1/1/1 | 0.021 | goalReached |
| InterceptMovingTargetEarliest#1 | forcedDynamic | 3 | true/true | 7.309 | 7.309 | 6.111 | 1/1/1 | 0.027 | goalReached |
| USOutlineExtremeVisibility#1 | normal | 0 | true/true | 12.920 | 12.920 | 4.314 | 1/1/1 | 17.602 | goalReached |
| USOutlineExtremeVisibility#2 | normal | 0 | true/true | 7.278 | 7.632 | 3.268 | 1/1/1 | 3.331 | goalReached |
| USOutlineExtremeVisibility#3 | normal | 0 | true/true | 22.071 | 23.354 | 5.796 | 1/1/1 | 16.339 | goalReached |
| USOutlineExtremeVisibility#1 | forcedDynamic | 0 | true/true | 12.920 | 12.920 | 4.314 | 1/1/1 | 15.110 | goalReached |
| USOutlineExtremeVisibility#2 | forcedDynamic | 0 | true/true | 7.278 | 7.632 | 3.268 | 1/1/1 | 17.034 | goalReached |
| USOutlineExtremeVisibility#3 | forcedDynamic | 0 | false/false | NaN | NaN | NaN | 0/0/0 | 129.395 | planTrajectory:UserCancelled |
| USOutlineExtremeVisibility#1 | normal | 1 | true/true | 12.920 | 12.920 | 4.314 | 1/1/1 | 13.282 | goalReached |
| USOutlineExtremeVisibility#2 | normal | 1 | true/true | 7.278 | 7.632 | 3.268 | 1/1/1 | 4.374 | goalReached |
| USOutlineExtremeVisibility#3 | normal | 1 | true/true | 22.071 | 23.354 | 5.796 | 1/1/1 | 13.857 | goalReached |
| USOutlineExtremeVisibility#1 | forcedDynamic | 1 | true/true | 12.920 | 12.920 | 4.314 | 1/1/1 | 12.796 | goalReached |
| USOutlineExtremeVisibility#2 | forcedDynamic | 1 | true/true | 7.278 | 7.632 | 3.268 | 1/1/1 | 13.565 | goalReached |
| USOutlineExtremeVisibility#3 | forcedDynamic | 1 | true/true | 22.238 | 23.953 | 6.216 | 1/1/1 | 115.169 | goalReached |
| USOutlineExtremeVisibility#1 | forcedDynamic | 2 | true/true | 12.920 | 12.920 | 4.314 | 1/1/1 | 13.996 | goalReached |
| USOutlineExtremeVisibility#2 | forcedDynamic | 2 | true/true | 7.278 | 7.632 | 3.268 | 1/1/1 | 15.473 | goalReached |
| USOutlineExtremeVisibility#3 | forcedDynamic | 2 | true/true | 22.238 | 23.953 | 6.216 | 1/1/1 | 98.229 | goalReached |
| USOutlineExtremeVisibility#1 | normal | 2 | true/true | 12.920 | 12.920 | 4.314 | 1/1/1 | 13.635 | goalReached |
| USOutlineExtremeVisibility#2 | normal | 2 | true/true | 7.278 | 7.632 | 3.268 | 1/1/1 | 3.009 | goalReached |
| USOutlineExtremeVisibility#3 | normal | 2 | true/true | 22.071 | 23.354 | 5.796 | 1/1/1 | 12.902 | goalReached |
| USOutlineExtremeVisibility#1 | normal | 3 | true/true | 12.920 | 12.920 | 4.314 | 1/1/1 | 13.376 | goalReached |
| USOutlineExtremeVisibility#2 | normal | 3 | true/true | 7.278 | 7.632 | 3.268 | 1/1/1 | 2.777 | goalReached |
| USOutlineExtremeVisibility#3 | normal | 3 | true/true | 22.071 | 23.354 | 5.796 | 1/1/1 | 13.437 | goalReached |
| USOutlineExtremeVisibility#1 | forcedDynamic | 3 | true/true | 12.920 | 12.920 | 4.314 | 1/1/1 | 11.568 | goalReached |
| USOutlineExtremeVisibility#2 | forcedDynamic | 3 | true/true | 7.278 | 7.632 | 3.268 | 1/1/1 | 13.715 | goalReached |
| USOutlineExtremeVisibility#3 | forcedDynamic | 3 | true/true | 22.238 | 23.953 | 6.216 | 1/1/1 | 121.865 | goalReached |

## Solver details and limitations

| Forced scene | Selected kernel | Static projection accepted | Timed-cell kernel attempted | Waypoint fallback attempted |
|---|---|---|---|---|
| exampleObstacleFree#1 | analyticRestToRest | false | false | false |
| exampleObstacleAvoidance#1 | bmtpStaticDegree8 | true | false | false |
| exampleStaticUShapedObstacle#1 | bmtpStaticDegree8 | true | false | false |
| exampleAlternatingSlalom#1 | bmtpStaticDegree8 | true | false | false |
| exampleDenseConcaveObstacle#1 |  | false | false | false |
| exampleNoPath#1 |  | false | false | false |
| exampleTwoOpposingUVisibilityGraph#1 | bmtpStaticDegree8 | true | false | false |
| exampleTargetExitsObstacle#1 | bmtpStaticDegree8 | true | false | false |
| exampleStraightTargetAlternatingOcclusion#1 | bmtpStaticDegree8 | true | false | false |
| exampleInterceptMovingTargetAtSetTime#1 | analyticRestToRest | false | false | false |
| exampleInterceptMovingTargetEarliest#1 | analyticRestToRest | false | false | false |
| exampleUSOutlineExtremeVisibility#1 |  | false | false | false |
| exampleUSOutlineExtremeVisibility#2 | bmtpStaticDegree8 | true | false | false |
| exampleUSOutlineExtremeVisibility#3 | bmtpStaticDegree8 | true | false | false |

112 comparison calls recorded (84 timed repetitions), plus one initial successful warm-up before a logger initialization error. All 103 returned successes passed independent validation and the applicable collision, kinematic, and polynomial certificate checks. The expected NoPath case must be read as an expected failure, not an unsafe success. The forced-dynamic Philippines warm-up exhausted the shared 180-second example budget after 129.395 seconds inside its planner call; no trajectory was returned. This budget includes earlier regional planning, setup, recording and extra validation, so 129.395 seconds is not a universal planner timeout. The running MATLAB batch retained its original repetition schedule after a proposed skip edit was not reloaded. Later repetitions are reported separately.

All completed paired requests matched exactly after removing the cancellation callback from the comparison, and all completed scenes were independently classified static. The timeout had no returned request record; equality and scene classification for that failed call were not independently captured.

The initial logger incorrectly used optional plane/corridor certificates as the applicable-certificate metric. Final rows recompute the metric from retained independent polynomial-format, continuity, endpoint/history-consistency and resolved-collision checks, without rerunning or weakening validation. Raw results and original logs remain available in the ignored experiment directory.

Source-integrity check: 121 source files compared; 0 changes since the snapshot (none). No production static/dynamic flag, input geometry, limit, tolerance, seed budget or arrival policy was modified. Snapshot-only instrumentation includes call counting, result capture and the forced scene flag.

Conclusion: retain automatic static-scene detection. This flag changes route search and candidate construction, not just a timing optimization. Forcing dynamic routing provides no consistent benefit and worsens runtime or motion quality in several measured scenes. Tiny timings on shared exact-motion early exits do not establish a solver speed difference. These examples do not establish global optimality or general failure rates.
