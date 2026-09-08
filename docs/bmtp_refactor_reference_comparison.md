# Matched pinned-reference comparison

MATLAB R2024b, 2026-09-08. Both pinned implementations received the same frozen physical requests, including explicit per-axis limits. Each case was warmed once, then timed three times in serial. The reference input metadata was translated without changing the requested motion. These measurements exclude example setup and the interception wrapper search. Every successful motion was checked by a freshly prepared public validator, and its length integrated adaptively from polynomial speed. Values below are transcribed from preserved logs; CSV retains all repetitions.

Cleanup: c04f3b2. Reference: 26c343b9. The reference loses the frozen earliest-interception example request (which requests the fixed arrival selected by its wrapper). Hawaii is later, longer, and slower. Several other cases improve quality but increase runtime. This evidence rejects wholesale replacement; it does not establish global infeasibility for failed requests.

All successful entries pass collision, kinematic, and applicable certificate checks and terminate with goalReached. Cleanup has 19 successes; reference has 18. NoPath is an expected failure on both. Unavailable motion measurements are NaN.

| Case | Cleanup median s | Reference median s | Cleanup duration s | Reference duration s | Cleanup arc units | Reference arc units | Reference outcome |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| exampleAlternatingSlalom | 2.2132497 | 0.294858 | 10.550093893364 | 10.5000000001 | 16.0347739670252 | 16.0200124878 | goalReached |
| exampleDenseConcaveObstacle | 1.3846535 | 0.44943 | 8.5 | 8.50000000009 | 12.761168228437 | 12.7557749996 | goalReached |
| exampleFourAcceleratingCircles | 0.8205743 | 2.246767 | 22 | 22 | 20 | 20 | goalReached |
| exampleInterceptMovingTargetAtSetTime | 0.0091861 | 0.009127 | 12 | 12 | 9.53894054682111 | 9.53894054682 | goalReached |
| exampleInterceptMovingTargetEarliest | 0.008406 | 0.015615 | 6.11111111111111 | NaN | 7.30889024019446 | NaN | noOptimizedFeasibleIterate |
| exampleMovingBarrierWait | 0.2099746 | 0.052538 | 10.0903015136719 | 10.0900888957 | 10 | 10 | goalReached |
| exampleMovingCircleNoWrap | 0.7589303 | 0.273643 | 8.5 | 8.50000000009 | 12.4538380036333 | 12.4487977067 | goalReached |
| exampleMovingDeformingUSOutlineVisibility | 2.9056015 | 4.102944 | 7.91666666666667 | 7.91666666797 | 40.2482682085329 | 40.2380531626 | goalReached |
| exampleMovingRotatingObstacleField | 1.091184 | 0.280962 | 9.04166666666667 | 9.041666667 | 20.7162516634517 | 20.4559307712 | goalReached |
| exampleNoPath | 0.2065734 | 0.011757 | NaN | NaN | NaN | NaN | noVisibilityRoute |
| exampleObstacleFree | 0.0076241 | 0.009739 | 4.53112887414927 | 4.53112887424 | 4.47213595499958 | 4.472135955 | goalReached |
| exampleOpeningUShapedObstacle | 0.6945447 | 0.085491 | 13.617522354126 | 11.5843334475 | 9.99999999999998 | 10 | goalReached |
| exampleStaticUShapedObstacle | 4.323807 | 4.974039 | 20.8725483491005 | 20.7628013681 | 38.6784456458047 | 37.7792525503 | goalReached |
| exampleStraightTargetAlternatingOcclusion | 1.1151446 | 1.984654 | 20.8695652173913 | 20.8695652174 | 13.6104194208543 | 13.5563603378 | goalReached |
| exampleTargetExitsObstacle | 2.6172711 | 0.923367 | 24 | 24 | 20.6851514236741 | 20.504271975 | goalReached |
| exampleTwoOpposingUVisibilityGraph | 1.168299 | 0.331493 | 22.1006280522374 | 21.6333333367 | 24.2057853510092 | 24.0480351008 | goalReached |
| exampleVietnamKeepoutSlew | 10.1879008 | 2.242258 | 30 | 30 | 17.3054420695914 | 17.1441411537 | goalReached |
| GeographyCroatia | 1.1943847 | 0.614449 | 3.2750913251299 | 3.24930547581 | 7.51903810119471 | 7.45277336909 | goalReached |
| GeographyHawaii | 2.2110267 | 4.721238 | 4.3142358035265 | 5.47210633785 | 12.9198952189363 | 13.5295918701 | goalReached |
| GeographyPhilippines | 5.5686077 | 7.285259 | 5.82760502419256 | 5.20493999178 | 23.2586701098087 | 18.8014078814 | goalReached |
