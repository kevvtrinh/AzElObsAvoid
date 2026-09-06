# Verification

Latest check: authorized review cleanup from `1843944`, MATLAB R2024b,
2026-09-06. The baseline source and existing user changes were frozen before
editing.

| Check | Outcome |
| --- | --- |
| Focused regression gate | 53 tests passed before the complete example matrix. |
| Broader MATLAB suite | 149 passed, zero failed or incomplete, including the additional runtime-comparison regression. There are 149 distinct tests across both runs. |
| Node sandbox suite | All 24 tests passed. |
| All 18 maintained examples, default finite jerk, before and after | 17 independently valid motions and expected `exampleNoPath` failure in each run. Full returned records and fresh public validation matched exactly after identified runtime fields were removed. |
| Timing regression | Both fixed and earliest arrival, with a rectangle and an asymmetric pentagon: finite first-validation time, exclusive solving totals, independently valid motion. |
| Optional outputs and sandbox compatibility | File-only, one-output, and two-output requests preserve motion; hidden graphics, goal controls, export, and saved failure replay passed. |
| Instrumented execution evidence | Broad-run profile retained calls and executed lines for 110 production MATLAB files; this is not exhaustive branch coverage or a speed measurement. |
| Production MATLAB analysis | No findings in planner, engines, sandbox, offline sandbox, or benchmark helpers. |
| Source size | Planner/engine/sandbox sources: 21,102 to 21,058 physical lines; 15,975 to 15,948 nonblank, noncomment lines. Core-only historical size gate remains failed: 12,708 against 11,482, with 105 files. |
| Preservation | User-owned `Rogue Examples/failed.mat` and `nocarryvelocity.mat` hashes unchanged; HS3 archive files retain their original hashes; autosave recovery copy verified. Original benchmark CSV bytes retained. |

## Review dispositions

All findings below are observed in source/data flow or execution. Changes preserve
public schemas, collision margins, tolerances, geometry, search budgets, and motion
objectives. Compatibility and behavior risks are covered by the checks listed;
manual interaction and exhaustive coverage remain outside this pass.

| Priority | Finding, correction, and confirming evidence |
| --- | --- |
| High | Benchmark failures could become their own expected outcome, and comparison ignored saved independent checks. [Capture](benchmarks/capturePlannerRefactorBaseline.m) now declares outcomes before running; [comparison](benchmarks/comparePlannerRefactorBaseline.m) requires unchanged physics and valid checks. Regressions reject identical but invalid results and unexpected failure, while accepting declared no-path results. |
| Medium | A validated fixed-clock detour could lose its first-acceptance timestamp when ordinary guesses began. [Planner](+obstacleAvoidance/planTrajectory.m) retains that timestamp; two different obstacle shapes pass both arrival modes. |
| Medium | Detour summary solving time included validation while ordinary attempts excluded it. [Exact-motion stage](+obstacleAvoidance/+planner/tryDirectAndFixedTimeMotions.m) now uses exclusive solving time; regression reconciles attempt and stage totals. |
| Medium | [Offline requests](offlinesandbox/+offlineSandbox/runPlanningRequest.m) built an unused optional diagnosis bundle. Construction now requires a second output; zero/one/two-output behavior passes. |
| Medium | Planner and sandbox duplicated known/nonempty option merging. Both use [the shared resolver](+obstacleAvoidance/+input/resolveOptions.m); defaults, empty overrides, warnings, and control/export tests pass. |
| Medium | [Validation](+obstacleAvoidance/+validation/validatePreparedTrajectory.m) paired distant positional values and field names. Named assignments now share one empty schema; complete before/after validation records match. |
| Medium | [Sandbox](sandbox/obstacleAvoidanceSandbox.m) routed through private multi-mode plumbing despite having one goal mode. Private arguments/helpers and the redundant tab-selection callback are removed; public `GoalMode`, `ActiveMode`, tab, and export interface remain compatible. Hidden-graphics/export tests pass. |
| Low | Removed the sandbox's caller-free `formatNumericVector` helper and the unread replay `activeRequestId` assignment in [the server](offlinesandbox/+offlineSandbox/serveSandbox.m). Normal request IDs and replay behavior remain intact. |
| Medium | [Architecture tests](tests/testArchitectureBoundaries.m) enforced internal filenames and source call counts/order. Removed those implementation locks; retained public/dependency boundaries and executed contract, engine, recovery, and topology behavior tests. |
| Low | [Size audit](benchmarks/auditProductionSize.m) described 11,482 lines as current. It now identifies that unchanged ceiling as historical and accepts explicit frozen-baseline ceilings. The remaining size failure is reported, not hidden by a higher threshold. |
| Medium | README, walkthrough, and appendix described removed controls/owners. Current sources now describe BMTP and Ctrl+C; obsolete HS3 sources/PDFs are [archived unchanged](docs/archive/hs3/README.md). Source references resolve; PDFs were not rebuilt. |
| Low | Ignored MATLAB `*.asv` files and removed the obsolete resolver autosave after verifying an identical recovery copy. Maintained saved-request MAT fixtures remain in place and were exercised. |

The example comparison additionally exposed missed runtime fields in
[the stripping helper](benchmarks/stripPlannerRefactorRuntime.m):
`ElapsedPlanningTime_s` and `ConicSolver.TotalTime_s`. It now removes those measured
fields without removing physical `TotalTime_s`, arrival, duration, or solver outcome
evidence. The regression verifies those distinctions. All 18 saved comparisons
then passed without rerunning the examples or deleting the initial failure log.

## Limits and unfavorable records

Three example-invoking tests were excluded from the broad runner:
`testConvergenceSummaryAndRetainedBestTrial`, `testObstacleAvoidanceRunsHeadlessly`,
and `testMovingRotatingObstacleFieldRunsHeadlessly`. Their default scenarios ran
in the complete example matrix; their exact assertions and the convergence test's
alternate clearance option were not rerun. Manual browser interaction and
interactive Ctrl+C were not checked. TeX sources were reviewed, but no LaTeX
compiler was available and PDFs were not regenerated.

An extra frozen-baseline `exampleMovingBarrierWait` probe with infinite jerk limits
was rejected in `resolveExampleOptions` before planner or validator execution.
No motion, validation, or wall measurement is reported for that attempt. It is
recorded as invalid input, not a cleanup regression; the remaining optional
infinite-jerk probes were not run because the example contract requires finite
jerk. An initial focused-runner API error also occurred before any tests executed;
the corrected runner and its tests subsequently passed. These logs are retained.

[benchmark.csv](benchmark.csv) preserves all original bytes and appends 36 completed
runs plus the invalid-input attempt using its existing 17-column schema. The
`review-summarize` rows are the remaining already-executed candidate examples loaded
from their saved capture, not extra runs. Geographic example metrics describe the
final returned region. Raw source snapshots, profiles, captures, comparisons, and
recovery files are ignored under `tmp/review-cleanup-20260906/`. No speedup or
exhaustive safety/reachability claim is made.

## Earlier cleanup evidence

Earlier check: execution/data-flow cleanup on `3877017`, MATLAB R2024b,
2026-09-06.

| Check | Outcome |
| --- | --- |
| Broad MATLAB run plus corrected focused gates | 143 distinct tests passed; initial stale owner/schema assertions corrected. |
| Final failed-motion output checks | 14 focused output/fallback/graphics tests and two empty-motion failure contracts passed. |
| Node sandbox functions | 24 tests passed. |
| All 18 maintained examples, default finite jerk | 17 independently valid motions; expected `exampleNoPath` failure. Frozen inputs, routes, polynomials, positions, and arrivals matched exactly. |
| Final no-path rerun | Unchanged `noValidatedSeed`; unavailable numeric metrics NaN. |
| Three saved-request comparisons, first call + three warm repeats | Physical outputs exact; independent validation passed. No demonstrated runtime improvement. |
| Preparation ownership | Static U and Philippines decomposition/grouping calls halved; editing one of two obstacles rebuilt only that obstacle and matched fresh preparation. |
| Static checks | MATLAB analysis, diff whitespace, and repository-health skill validation passed. |
| Helper consolidation | 49 focused MATLAB tests passed. Fixed-target and saved static-U request checks preserved motion exactly and passed independent validation. MATLAB analysis and diff whitespace checks passed. |

Three tests that invoke maintained examples directly were excluded from the broad
runner. Their default scenarios ran in the complete matrix, but their exact
assertions and the convergence test's alternate clearance option were not rerun.
No manual browser interaction or PDF regeneration was performed. Renamed fields
were updated in the documentation exporter.

See the [execution audit](benchmarks/results/planner_execution_audit_20260906.md)
for dispositions, reachable cold paths, measured timing limits, and initial gate
corrections. Raw profiles/runners remain ignored. [benchmark.csv](benchmark.csv)
retains the complete history and appends actual runs with unchanged columns.
Earlier verification prose and unfavorable experiment history remain in Git and
the retained benchmark CSVs.
