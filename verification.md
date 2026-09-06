# Verification

Latest check: execution/data-flow cleanup on `3877017`, MATLAB R2024b,
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
