# Verification

Latest implementation checked: `1ce83c4`, MATLAB R2024b.
These are recorded results from the 2026-09-05 refactor, not new runs.

| Check | Recorded outcome |
| --- | --- |
| MATLAB test inventory and focused reruns | 142 distinct tests passed. |
| Browser page functions through Node | 24 tests passed. |
| All 18 maintained examples, default finite jerk limits | 17 independently valid motions; `exampleNoPath` returned `noValidatedSeed`. |
| Four preparation before/after cases | Motion, polynomial, routes, outcome, and independent validation matched exactly except elapsed timing. |
| Hidden graphics | Result-only plots, failure diagnosis, coordinate wrapping, and sandbox controls passed. |
| Static checks | MATLAB code analysis and diff whitespace checks completed. |

Initial failures were corrected before completion: an opening-example helper
omitted diagnosis, test fixtures/assertions needed the new output interface, and
example hashes included changed output calls. Physical input sections and the
hash guard's scope were preserved. The failed example execution remains recorded.

[benchmark.csv](benchmark.csv) retains all recorded example runs, including
unfavorable history and unavailable metrics as NaN. Historical rows describe
their recorded revision; they are not current performance claims. Experiment
CSV files under [benchmarks](benchmarks/) are also retained in full.

Limits: no manual browser interaction or interactive Ctrl+C check; PDFs were not
regenerated. Two timing repeats for preparation ownership support equivalence
and reduced preparation calls, not a general speedup claim.

Use [planner decisions](branch_assessment.md) for the current choices and
[the preparation comparison](benchmarks/results/lean_workflow_20260905.md) for
its focused evidence. Older verification prose is available in Git history.
The 2026-09-06 documentation cleanup requires no planner rerun.
