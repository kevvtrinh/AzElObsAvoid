# BMTP refactor feature gates

The implementation baseline is cleanup `c04f3b2`; the reference core is
`26c343b9`. These checks preserve demonstrated behavior, not global search
completeness. The untouched cleanup suite previously passed 187 tests. The
retained offset-translation change passed 188. New work must pass these same
features before replacing their production owners.

| Capability | Existing regression evidence |
| --- | --- |
| Nonzero endpoint velocity and acceleration | `testPlannerContract`: non-rest initial state, non-rest fixed terminal state, specified interception matching velocity/acceleration independently and together |
| Earliest and specified interception | `testPlannerContract`: fixed-arrival echo, bounded PCHIP target search, analytic linear target interception; maintained interception examples |
| Later route timing without unsafe source waiting | `testPlannerContract/testTimedSearchCanArriveLaterWithoutWaitingAtBlockedSource`; `testTimedRouteArrivalHandoff` |
| Static concavity, disconnected regions, holes | `testConvexPolygonRegions`, `testStaticPlanningProjection`, `testStaticRegionGrouping`, maintained geographic sequence |
| Changing topology and activity spans | `testObstacleHistoryContract`: swept-gap coverage, nested concavity, static activity span; `testTimedVisibilityScreening`: authoritative samples and rounded endpoint times |
| Periodic axes | `testCoordinateWrappingPlotting`: separate and unequal periods; unsupported obstacle wrapping is explicit |
| Explicit waypoint fallback | `testUnsupportedTimedTopologyPolicy`; baseline capture's `ExplicitRuckigBackup` record |
| Continuous independent acceptance | `testPlannerContract`: interior polynomial violation, nondyadic tangency; `testTimedBmtpPlanning`: rejected certificate fallback |
| Stable failures and invalid input | `testPlannerContract`: endpoint failure schema and no path; `testPlannerLimits`, `testPlannerOptions`, `testObstacleInfrastructure` |

`testLaterInterception` now establishes the dedicated public interception case
on the pinned cleanup baseline: an early physical meeting is blocked, the
obstacle deactivates after five seconds, and a later validated meeting succeeds
at 6.410054054260255 seconds. Shifting the time origin by seven seconds shifts
the returned arrival by seven seconds within 1e-4 seconds. The initial test's
guessed arrival immediately after deactivation was incorrect: the complete
motion must also enter the cleared region after deactivation. Its failed log
is preserved; the corrected contract checks later success within the horizon,
independent validation, continued search, and time-origin invariance.

## Current geometry-reuse experiment

The existing dense-obstacle profile attributes 0.359 seconds inclusive to 3,841
`preparedShapeAtTime` calls, including 0.184 seconds of repeated boundary
classification. These nested costs are not additive. Sample and interval
geometry for time-invariant histories are now prepared once; moving histories
retain the original query-time calculation. There is no persistent cache.
Preparation version 2 and the existing full source snapshot invalidate old data.

Acceptance requires exact shape and geometry equality against the frozen query
implementation, unchanged successful motions, and a matched timing benefit.
The query regression covers concave and convex rings, holes, disconnected
regions, translation, deformation, topology changes, empty samples, endpoints,
and inactive times. Source-change tests cover time origin, geometry, margin
metadata, and original history changes. The public validator now discards
caller-supplied preparation and rebuilds it from canonical source histories;
an additional test supplies a false cache whose source snapshot appears current.

All three focused MATLAB tests pass. The matched microbenchmark alternates
original/cached order for three repetitions after warming both paths. Each
measurement contains 500 identical physical-time queries.

| Vertices | Original median (s) | Cached median (s) |
| ---: | ---: | ---: |
| 12 | 0.024373 | 0.0044827 |
| 120 | 0.025032 | 0.0037013 |
| 1200 | 0.033269 | 0.0035960 |

This establishes a query-level benefit only. The first full comparison retained
identical physical records but exposed an accelerating-history slowdown. A
matched rerun confirmed it: the original median was 0.851545 seconds and the
eager-cache median was 0.919316 seconds. Separate profiles showed geometry calls
increase from 242 to 1,430 (boundary classification from 242 to 1,269). Eager
moving-history preparation was therefore rejected. The static-only revision
restores both counts to 242 on that request. Mixed static/moving preparation
keeps a uniform structure layout with empty cache fields for movers.

The final static-only 60-record comparison again passes exact physical, search,
and adaptive-length equality. The complete final feature rerun passes all 192
MATLAB tests.

## Controlled physical-input capture

`benchmarkFrozenRequests` uses explicit normalized per-axis limits and frozen
source histories. It warms each case once and records three individual planner
timings, returned motion, solver/search diagnosis, fresh public validation, and
arc length from adaptive integration of each polynomial span's speed. Display
sampling does not enter the arc-length calculation. All three geographic cases
(Hawaii, Croatia, Philippines) are captured separately through the example's
optional third output; its first two outputs remain unchanged.

The first baseline attempt was interrupted because saving the complete MAT
capture after every repetition dominated total wall time. Its log is preserved
in ignored scratch storage and is excluded from controlled comparisons. The
restarted harness saves the complete record once after all measurements.
