# Archived Shared Example Jerk-Constraint Benchmark

These results describe the pre-refactor implementation and are retained only
as the reproducible comparison baseline. Run the benchmark source on the
current branch before drawing runtime conclusions about the distilled
analytic retimer.

This report compares the complete finite-jerk and unconstrained-jerk planner
modes on the eight runnable scenarios shared with
[`kevvtrinh/AzElObsAvoid`](https://github.com/kevvtrinh/AzElObsAvoid) `main`
at revision `d25a0e840772c1d243b9c2aa59c25163150e7257`.

The benchmark executes the current feature worktree, not the upstream planner
implementation. The upstream example blobs provide audited scenario
provenance. For four examples whose upstream wrapper used different horizons
for the two modes, both runs use the upstream finite-jerk physical horizon:

- U-shaped: 120 s
- Opposing Us: 180 s
- Moving circle: 120 s
- Six-baffle slalom: 480 s

Every pair passed strict equality checks for obstacle data and time history,
initial state, goal or intercept request, non-jerk limits, configured finite
jerk, and resolved planner options. The applied jerk limit was the only input
difference: `[Inf Inf]` versus the example's finite limit.

## Environment And Method

- MATLAB R2024b Update 4, Windows `PCWIN64`
- Six MATLAB-reported cores
- Pre-refactor mesh optimizer was available for this archived run
- Serial execution (`UseParallel = "off"`)
- Figures, animation, swept surfaces, and kinematic plots disabled
- One discarded fixed-intercept warm-up per mode
- One measured repetition, with alternating within-pair mode order
- 16/16 example runs and 8/8 paired-input checks passed

One repetition is enough for deterministic path and arrival comparisons, but
wall-time ratios are indicative and sensitive to execution order and caches.
Use at least three repetitions for runtime conclusions.

## Results

| Example | No-jerk wall (s) | Jerk wall (s) | Wall ratio | No-jerk arrival (s) | Jerk arrival (s) | Jerk delay (s) |
|---|---:|---:|---:|---:|---:|---:|
| U-shaped | 2.150 | 8.198 | 3.813 | 26.701 | 28.217 | 1.516 |
| Opposing Us | 2.201 | 37.850 | 17.198 | 21.831 | 67.912 | 46.081 |
| Moving circle | 2.075 | 3.133 | 1.510 | 14.807 | 14.967 | 0.160 |
| Six-baffle slalom | 6.459 | 79.250 | 12.269 | 40.290 | 107.496 | 67.206 |
| Static U.S. outline | 17.080 | 6.411 | 0.375 | 19.579 | 18.730 | -0.849 |
| Moving U.S. outline | 85.420 | 39.735 | 0.465 | 19.848 | 18.348 | -1.500 |
| Earliest intercept | 8.738 | 8.043 | 0.921 | 7.227 | 7.460 | 0.233 |
| Fixed intercept | 0.318 | 0.331 | 1.043 | 12.000 | 12.000 | 0.000 |

This is a full-planner mode comparison, not an isolated retimer comparison on
one fixed path. Enabling finite jerk changes smoothing and can change the
selected candidate geometry. For example, the moving U.S. case selected a
different smooth path and arrived earlier with jerk enabled; that is a route
selection result, not evidence that adding a constraint speeds up the same
fixed path.

Finite-jerk rounded paths use continuous Bernstein interval bounds for the
first three path derivatives. Sampled derivative maxima remain diagnostic and
are not used as the hard jerk certificate.

## Reproduction

```matlab
addpath("benchmarks")
report = benchmarkAzElJerkConstraintExamples(struct( ...
    "RepetitionCount", 1, ...
    "RunWarmup", true, ...
    "Verbose", true));
```

Use `RepetitionCount = 3` or greater when comparing runtime medians. The
returned report includes raw runs, source provenance, input checks, summaries,
paired comparisons, retimer types, candidate counts, and optimizer diagnostics.
