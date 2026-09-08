# BMTP refactor progress — September 8, 2026

The shared [refactor plan](../bmtp_refactor.md) is imported on `bmtp_refactor`,
based on cleanup revision `c04f3b280725edd30824947f18751afe07a7a0d9`.
The emptycore reference is `26c343b9ab7704d0cd5a3d8f72af058e99c936b1`.
The original checkout and its existing `AGENTS.md` edit remain untouched.

**The complete eight-stage plan is not implemented.** The retained production
change batches offset-spline polynomial translation. The static-corridor port
remains an unretained experiment in the ignored `scratch/corridor_draft` folder.
Geometry batching, further preparation reuse, analytic waiting, corridor and
clock integration, continuous-length optimization, and broader consolidation
still require implementation and their respective verification gates. The
cross-branch benchmark with a uniform warm-up policy and explicit capture of
every geographic subcase is also outstanding.

## Retained change

`bmtpEngine.createOffsetSplineMotion` translates all output subintervals in
two calls, one for the base motion and one for the lateral spline. Previously
it translated each output span separately. Independent spans are batched while
the coefficient sums retain their original arithmetic order. This removes
repeated source lookup and binomial calculations without changing the motion,
solver selection, geometry, limits, validation, or tolerances.

The motivating MATLAB profile of `exampleDenseConcaveObstacle` recorded 368
offset-spline constructions and 9,472 translation calls. Translation took
0.611 s inclusive in that profiled run; `nchoosek` was called 146,816 times
across the profile. These nested times must not be added together or compared
directly with unprofiled wall time.

The physical production count, including comments and blank lines under
`+obstacleAvoidance` and `trajectory`, decreases from **15,528 to 15,519**.
Both revisions contain 108 production MATLAB files. Tests, benchmarks, and
scratch experiments are excluded from both counts.

## Verification

- MATLAB R2024b: the untouched cleanup baseline passes all **187 tests**.
- The candidate passes all **188 tests**, including a new scalar-versus-batch
  regression using one, two, and three dimensions, nonuniform knots, shifted
  clocks, prescribed interior velocities, and both cubic and quintic bases.
- The new regression requires exact equality of the complete motion record.
  The scalar implementation is retained only in `tests/private`.
- All 18 maintained examples meet their expected outcomes in each of three
  baseline runs and three candidate runs. Every successful returned motion
  passes fresh independent validation. `exampleNoPath` returns the expected
  unsuccessful outcome, `noValidatedSeed`.
- The first complete baseline/candidate capture comparison reports exact
  equality of all 18 runtime-stripped physical records and the additional
  focused records, including the explicitly selected waypoint fallback.
- Actual full-example measurements are appended to [benchmark.csv](../benchmark.csv).
  Those records preserve the established sampled-length measurement and label
  it explicitly. They do not establish the plan's adaptive arc-length gate.

The full-example runs have different process/JIT warm-up histories. Their raw
measurements are preserved, but their ratios are **not controlled speedup
estimates**. In particular, do not attribute changes in unaffected direct-motion
examples to this translation change.

The constructor benchmark warms both implementations separately, alternates
their measurement order, and records three repetitions of 30 identical calls
for each knot count. It checks exact motion equality before timing.

| Knots | Output spans | Scalar median (s/call) | Batch median (s/call) | Speedup |
| ---: | ---: | ---: | ---: | ---: |
| 3 | 8 | 0.000965467 | 0.000881597 | 1.095x |
| 9 | 14 | 0.00122545 | 0.000969247 | 1.264x |
| 25 | 30 | 0.00212054 | 0.00130732 | 1.622x |

These measurements establish a benefit for the changed constructor, not a
universal planner speedup. Larger span counts reduce more repeated translation
work. Reproduce the focused comparison with:

```matlab
addpath(pwd, fullfile(pwd, 'trajectory'), fullfile(pwd, 'tests'));
assertSuccess(runtests('tests/testOffsetTranslationBatch.m'));
measurements = benchmarkOffsetTranslations();
```

## Unretained corridor experiment

The draft uses emptycore's exact static facets, integrated quintic free-axis
motion, physical clock, polynomial subdivision, and source-region certificates.
It is checked with cleanup's unchanged public validator. It has not been added
to any production caller.

Using the second spatial seed on the captured cleanup requests:

| Case | Public validation | Draft / baseline duration (s) | Draft / baseline sampled length |
| --- | --- | ---: | ---: |
| Alternating slalom | Passed | 10.500000 / 10.550094 | 16.020014 / 16.034754 |
| Dense concave obstacle | Passed | 8.500000 / 8.500000 | 12.761074 / 12.761105 |
| Two opposing U obstacles | Passed | 21.633420 / 22.100628 | 24.449579 / 24.205764 |
| Static U obstacle | Inapplicable: non-monotone guide | NaN / 20.872548 | NaN / 38.678082 |
| Philippines | Passed | 6.425882 / 5.827605 | 24.028270 / 23.257993 |

The first slalom seed fails complete collision certification, and several
direct seeds yield unresolved corridor solves. The opposing-U draft is longer;
the Philippines draft is both later and longer. These results do not justify
replacing the existing static pipeline. No validation rule was weakened, and
no failing or inferior experimental motion is returned by production.

Raw MAT captures, profiles, logs, and draft code remain outside source control
in the worktree's ignored `scratch` directory. The frozen baseline worktree is
at `../cleanup_baseline`.
