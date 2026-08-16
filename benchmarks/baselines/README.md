# Spatial-jerk retimer refactor baseline

These artifacts compare the maintained example suite before and after the
repository maintainability refactor.

- Branch: `codex/spatial-jerk-retimer`
- Baseline revision: `0900e021a4a6052e4225b1def9e739881631f2ce`
- MATLAB release: recorded inside each MAT report
- Execution: serial, headless, plots and animation disabled
- Matrix: every `examples/example*.m` entry point with jerk disabled and
  enabled (14 examples, 28 runs)

Artifacts:

- `spatial_jerk_before_refactor_0900e02.mat` contains the complete baseline
  report and environment metadata.
- `spatial_jerk_before_refactor_0900e02_runs.csv` contains the baseline rows.
- `spatial_jerk_after_refactor.mat` and
  `spatial_jerk_after_refactor_runs.csv` contain the final rows.
- `spatial_jerk_before_after_comparison.mat` and
  `spatial_jerk_before_after_comparison.csv` contain paired deltas.
- `spatial_jerk_after_template_reduction.mat` and its `_runs.csv` companion
  contain the rerun after canonicalizing the retimer result templates.
- `spatial_jerk_template_reduction_comparison.mat` and its `.csv` companion
  compare that rerun directly with the original pre-refactor baseline.

Acceptance used a `1e-10` absolute tolerance for selected polyline length,
smoothed-path length, and minimum motion duration. Success state, independent
validation, termination reason, collision status, kinematic status, and
certificate status had to match exactly. All 28 rows passed, with zero observed
physical-metric delta.

Wall-clock timing is retained for diagnosis but is not an acceptance metric.
Single-run timings vary with MATLAB startup, cache state, and system load.

The moving U.S. outline example intentionally retains 4 of 61 obstacle
snapshots for route generation. Continuous collision validation still uses the
complete history, so returned trajectories remain checked against all source
geometry, but a route available only at an omitted snapshot can be missed.
