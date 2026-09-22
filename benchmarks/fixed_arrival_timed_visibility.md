# Unified fixed-arrival guide selection

Fixed-arrival moving-obstacle planning has one deterministic production flow.
The planner first solves from the exact initial visibility route. If that
proposal is solver-infeasible, it evaluates the exact arrival-snapshot route;
only another solver-level infeasibility admits the exact timed visibility route.
Each route receives the same BMTP implementation. The caller does not choose a
method, and no example identity changes this decision.

Each proposal boundary comes from the solver's own evidence. Successful spatial
requests in the random suite either prove on the initial mesh or after the
first refinement. The four hard requests still contain unresolved curve-region
pairs after that refinement and formerly repeat the same homotopy for all 35
iterations. Their timed visibility routes choose the other side of the moving
obstacle, then prove in one or two iterations. The rejected spatial attempt is
retained in `VisibilityGraph.SpatialSeedDiagnostics` whenever the timed guide is
selected.

No public search selector remains. Fixed-arrival moving targets and
earliest-arrival requests use the same input-driven planner policy.

## Full random-suite comparison

The comparison uses the same 80 deterministic random azimuth cases, each with
and without its static obstacle (160 requests total). Every successful result
was checked with the public independent validator.

| Measure | Production baseline | Unified policy |
| --- | ---: | ---: |
| Valid results | 156 / 160 | 160 / 160 |
| Total planner wall time | 432.347 s | 166.182 s |
| Median request time | 0.408 s | 0.726 s |
| 95th-percentile request time | 1.774 s | 3.005 s |
| Maximum request time | 106.484 s | 7.674 s |

The measured machine was also in interactive use during the unified run, so
the small-case wall-time distribution is conservative. More importantly, the
four pathological 65--106 s failed searches became 4--8 s validated results;
there is no large-tail runtime regression.

All 156 requests that already succeeded retained exactly the same arrival time
and motion length. The four newly successful requests are random cases 26 and
36 with a static obstacle, plus both variants of case 62. Those are the only
requests that selected the timed guide.

## Maintained examples and verification

All 23 maintained examples retained their success/failure outcome, termination
reason, independent-validation result, arrival time, and motion length. The
earliest-arrival intercept example is unchanged. The saved moving-detour
regression now uses the unified flow; its 231.459647-unit result is 0.898% longer
than the former explicitly timed 229.400576-unit result and remains within the
one-sided 1% regression gate.

The complete MATLAB suite passes: 109 passed, 0 failed, 0 incomplete. The
changed production files have no new Code Analyzer findings, and `git diff
--check` is clean.

## Reproduce

```matlab
addpath('benchmarks');
[fixedRuns,fixedResults] = benchmarkFixedArrivalPolicy(3);
[randomRuns,randomResults] = benchmarkRandomAzimuth(1:80);
```

Both benchmarks exercise only the unified public flow. They report the selected
guide as output evidence rather than running duplicate caller-selected modes.
The compact benchmark uses random case 1 and the saved moving detour for the
spatial-proof path, plus random case 26 with a static obstacle for the timed-guide
path. The Vietnam boundary fixture is intentionally excluded from this
compact benchmark because of its size; its changing source boundary now
carries an exact proven correspondence and is covered by
`testVietnamBoundary` and the maintained example harness
(see `benchmarks/vietnam_boundary.md`).
