# Circle detour: arrival first, then travel

Accepted against commit 22babeed7342eeab5839253a4c3e5dab2d2a12be plus the starting working edits. MATLAB R2024b; exact saved circle request and options; no geometry, tolerance, or limit changes.

The early detour waypoint was only a spline through-point. Its actual elevation peak occurred much later and reached 55 deg despite the obstacle top being approximately 26 deg. The added proposal fixes complete-axis velocity to zero at that waypoint while leaving acceleration free. The existing through-point family remains available. Every retained proposal passes the independent continuous validator.

The user explicitly prioritized arrival, then path length, subject to hard physical limits. The former integrated-squared-jerk rejection inside travel refinement was removed; its diagnostic values remain. Integrated jerk may increase. No arrival-versus-length trade or global shortest-path claim is made.

| Case | Travel before / after (deg) | Duration before / after (s) | Warmed median before / after (s) |
| --- | ---: | ---: | ---: |
| circle | 249.201569069 / 227.456480998 | 113.691362616 / 113.691362616 | 2.067310 / 3.910954 |
| rectangle | 112.397597877 / 103.355070722 | 52.966666667 / 52.966666667 | 0.938971 / 1.629758 |

One warm-up and three measured calls per case and implementation. All 16 complete calls passed independent validation. The rectangle swaps the governing axis and uses an asymmetric box and a nonzero start time. Additional planning work is a constant-factor expansion of the fixed-clock proposal family; this is a route-quality improvement, not a planner speedup.

MATLAB passed 60 affected tests: circle, timing, options, planner contract, BMTP engine, and the three available generated route-economy cases. Code Analyzer reported zero messages in the four changed MATLAB files.

18 maintained examples ran headlessly in both implementations with their default finite jerk limits. Per-example outcomes and motion metrics are below. Fixed-arrival behavior was also covered by contract and timing tests. The full repository test suite was not rerun. Two route-economy tests dependent on previously deleted MAT files were excluded; the files were not restored or replaced.

| Example | Version | Planner / validator | Polyline / motion (deg) | Duration (s) | Collision / kinematic / certificate | Wall (s) | Termination |
| --- | --- | --- | ---: | ---: | --- | ---: | --- |
| exampleObstacleFree | baseline | 1 / 1 | 4.472135955 / 4.472135955 | 4.531128874 | 1 / 1 / 1 | 0.666153 | goalReached |
| exampleStaticUShapedObstacle | baseline | 1 / 1 | 34.942588040 / 38.678082287 | 20.872548349 | 1 / 1 / 1 | 8.165431 | goalReached |
| exampleMovingCircleNoAzimuthWrap | baseline | 1 / 1 | 12.482421495 / 12.482421495 | 8.500000000 | 1 / 1 / 1 | 1.051650 | goalReached |
| exampleOpeningUShapedObstacle | baseline | 1 / 1 | 10.000000000 / 10.000000000 | 13.617522354 | 1 / 1 / 1 | 1.517265 | goalReached |
| exampleNoPath | baseline | 0 / 0 | NaN / NaN | NaN | 0 / 0 / 0 | 0.517593 | noValidatedSeed |
| exampleObstacleFree | candidate | 1 / 1 | 4.472135955 / 4.472135955 | 4.531128874 | 1 / 1 / 1 | 0.700460 | goalReached |
| exampleStaticUShapedObstacle | candidate | 1 / 1 | 34.942588040 / 38.678082287 | 20.872548349 | 1 / 1 / 1 | 7.960906 | goalReached |
| exampleMovingCircleNoAzimuthWrap | candidate | 1 / 1 | 12.453788460 / 12.453788460 | 8.500000000 | 1 / 1 / 1 | 1.848734 | goalReached |
| exampleOpeningUShapedObstacle | candidate | 1 / 1 | 10.000000000 / 10.000000000 | 13.617522354 | 1 / 1 / 1 | 2.552035 | goalReached |
| exampleNoPath | candidate | 0 / 0 | NaN / NaN | NaN | 0 / 0 / 0 | 0.498200 | noValidatedSeed |
| exampleAlternatingSlalom | baseline | 1 / 1 | 16.019319798 / 16.034753581 | 10.550093893 | 1 / 1 / 1 | 6.498397 | goalReached |
| exampleDenseConcaveObstacle | baseline | 1 / 1 | 12.797481220 / 12.797481220 | 8.500000000 | 1 / 1 / 1 | 1.635465 | goalReached |
| exampleFourAcceleratingCircles | baseline | 1 / 1 | 20.000000000 / 20.000000000 | 22.000000000 | 1 / 1 / 1 | 5.022434 | goalReached |
| exampleInterceptMovingTargetAtSetTime | baseline | 1 / 1 | 9.538940547 / 9.538940547 | 12.000000000 | 1 / 1 / 1 | 0.135257 | goalReached |
| exampleInterceptMovingTargetEarliest | baseline | 1 / 1 | 7.308890240 / 7.308890240 | 6.111111111 | 1 / 1 / 1 | 0.195520 | goalReached |
| exampleMovingBarrierWait | baseline | 1 / 1 | 10.000000000 / 10.000000000 | 10.090301514 | 1 / 1 / 1 | 0.758490 | goalReached |
| exampleMovingDeformingUSOutlineVisibility | baseline | 1 / 1 | 40.285727354 / 40.285727354 | 7.916666667 | 1 / 1 / 1 | 24.250617 | goalReached |
| exampleMovingRotatingObstacleField | baseline | 1 / 1 | 20.716208791 / 20.716208791 | 9.041666667 | 1 / 1 / 1 | 1.553143 | goalReached |
| exampleObstacleAvoidance | baseline | 1 / 1 | 11.152119519 / 11.220421805 | 7.572045030 | 1 / 1 / 1 | 1.373084 | goalReached |
| exampleStraightTargetAlternatingOcclusion | baseline | 1 / 1 | 13.341664064 / 13.610415661 | 20.869565217 | 1 / 1 / 1 | 1.984613 | goalReached |
| exampleTargetExitsObstacle | baseline | 1 / 1 | 20.135789033 / 20.685146757 | 24.000000000 | 1 / 1 / 1 | 3.894668 | goalReached |
| exampleTwoOpposingUVisibilityGraph | baseline | 1 / 1 | 24.035784715 / 24.205764414 | 22.100628052 | 1 / 1 / 1 | 1.867053 | goalReached |
| exampleUSOutlineExtremeVisibility | baseline | 1 / 1 | 22.070646907 / 23.257992600 | 5.827605024 | 1 / 1 / 1 | 21.561689 | goalReached |
| exampleAlternatingSlalom | candidate | 1 / 1 | 16.019319798 / 16.034753581 | 10.550093893 | 1 / 1 / 1 | 6.947483 | goalReached |
| exampleDenseConcaveObstacle | candidate | 1 / 1 | 12.761104518 / 12.761104518 | 8.500000000 | 1 / 1 / 1 | 3.176620 | goalReached |
| exampleFourAcceleratingCircles | candidate | 1 / 1 | 20.000000000 / 20.000000000 | 22.000000000 | 1 / 1 / 1 | 4.610718 | goalReached |
| exampleInterceptMovingTargetAtSetTime | candidate | 1 / 1 | 9.538940547 / 9.538940547 | 12.000000000 | 1 / 1 / 1 | 0.058461 | goalReached |
| exampleInterceptMovingTargetEarliest | candidate | 1 / 1 | 7.308890240 / 7.308890240 | 6.111111111 | 1 / 1 / 1 | 0.125703 | goalReached |
| exampleMovingBarrierWait | candidate | 1 / 1 | 10.000000000 / 10.000000000 | 10.090301514 | 1 / 1 / 1 | 0.533118 | goalReached |
| exampleMovingDeformingUSOutlineVisibility | candidate | 1 / 1 | 40.248219224 / 40.248219224 | 7.916666667 | 1 / 1 / 1 | 19.649366 | goalReached |
| exampleMovingRotatingObstacleField | candidate | 1 / 1 | 20.716208791 / 20.716208791 | 9.041666667 | 1 / 1 / 1 | 3.988774 | goalReached |
| exampleObstacleAvoidance | candidate | 1 / 1 | 11.152119519 / 11.220421805 | 7.572045030 | 1 / 1 / 1 | 1.541255 | goalReached |
| exampleStraightTargetAlternatingOcclusion | candidate | 1 / 1 | 13.341664064 / 13.610415661 | 20.869565217 | 1 / 1 / 1 | 2.321967 | goalReached |
| exampleTargetExitsObstacle | candidate | 1 / 1 | 20.135789033 / 20.685146757 | 24.000000000 | 1 / 1 / 1 | 3.642801 | goalReached |
| exampleTwoOpposingUVisibilityGraph | candidate | 1 / 1 | 24.035784715 / 24.205764414 | 22.100628052 | 1 / 1 / 1 | 2.046036 | goalReached |
| exampleUSOutlineExtremeVisibility | candidate | 1 / 1 | 22.070646907 / 23.257992600 | 5.827605024 | 1 / 1 / 1 | 24.735397 | goalReached |

For the expected no-path result, zero check flags mean no motion was available; numeric motion metrics are NaN.

| Case | Version | Run (1 = warm-up) | Wall time (s) | Travel (deg) | Duration (s) | Integrated squared jerk (deg2/s5) | Valid |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| circle | baseline | 1 | 4.6608164 | 249.201569068894 | 113.691362616008 | 7.525034823642 | 1 |
| circle | baseline | 2 | 3.3358514 | 249.201569068894 | 113.691362616008 | 7.525034823642 | 1 |
| circle | baseline | 3 | 2.0673100 | 249.201569068894 | 113.691362616008 | 7.525034823642 | 1 |
| circle | baseline | 4 | 1.7139446 | 249.201569068894 | 113.691362616008 | 7.525034823642 | 1 |
| rectangle | baseline | 1 | 1.2556114 | 112.397597876977 | 52.966666666667 | 7.552289619111 | 1 |
| rectangle | baseline | 2 | 0.9122252 | 112.397597876977 | 52.966666666667 | 7.552289619111 | 1 |
| rectangle | baseline | 3 | 1.0418915 | 112.397597876977 | 52.966666666667 | 7.552289619111 | 1 |
| rectangle | baseline | 4 | 0.9389710 | 112.397597876977 | 52.966666666667 | 7.552289619111 | 1 |
| circle | candidate | 1 | 5.5513770 | 227.456480997858 | 113.691362616008 | 7.529311591438 | 1 |
| circle | candidate | 2 | 4.2067464 | 227.456480997858 | 113.691362616008 | 7.529311591438 | 1 |
| circle | candidate | 3 | 3.9109539 | 227.456480997858 | 113.691362616008 | 7.529311591438 | 1 |
| circle | candidate | 4 | 3.6516629 | 227.456480997858 | 113.691362616008 | 7.529311591438 | 1 |
| rectangle | candidate | 1 | 2.0561710 | 103.355070722481 | 52.966666666667 | 7.618375528199 | 1 |
| rectangle | candidate | 2 | 1.6183548 | 103.355070722481 | 52.966666666667 | 7.618375528199 | 1 |
| rectangle | candidate | 3 | 1.6297578 | 103.355070722481 | 52.966666666667 | 7.618375528199 | 1 |
| rectangle | candidate | 4 | 1.7474619 | 103.355070722481 | 52.966666666667 | 7.618375528199 | 1 |
