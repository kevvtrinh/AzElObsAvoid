# Moving-obstacle timing at 220 vertices per snapshot

The source is a synthetic translating concave polygon with exactly 220 vertices
at each of 4,829 timestamps. The request runs from 2770 to 3000 seconds and covers
921 source snapshots. These are simulated times, not planner runtime.

Run in a fresh MATLAB R2024b session from the repository root:

```matlab
addpath('benchmarks');
[record,result] = benchmarkMovingObstacle220();
disp(record)
```

Input generation is outside the measured planner call. Public independent
validation runs inside planning and is checked again after timing. No generated
MAT files or plots are required to reproduce the input.

The same inputs are available as a maintained graphical example:

```matlab
addpath('examples');
result = exampleMovingObstacle220();
% Headless: exampleMovingObstacle220(struct('PlotOutputs',false,'Verbose',false))
```

This complements `exampleMovingRotatingObstacleField`: the mixed rotating field
retains its historical regression gates, while this dense translating obstacle
adds a distinct preparation and time-scoping workload. The example and benchmark
share `createMovingObstacle220Scenario` so their source geometry, timestamps,
endpoints, and default limits stay identical. This new input has no borrowed
historical reference values.

## What reduced the work

- Prepare only requested samples and bracketing intervals. Keep the complete
  source history; extend the same source-checked cache for overlapping requests.
- Replace quadratic cyclic-shift searches with circular correlation in both
  orientations. Verify the selected translation/convex interpolation; resolve
  symmetric ties independently of ring starting indices.
- Keep the motion mesh independent of the obstacle sampling rate. Constrain
  each curve span only over its exact overlap with each source interval, using
  exact Bezier restriction. Every source interval remains checked.
- Cache obstacle-edge normals and endpoint supports for repeated final
  all-pair certification. The alternating optimizer deliberately retains its
  original combined projection arithmetic so its selected motion is unchanged.
- Reverify the trajectory SOCP's existing planes first. Only when every active
  pair certifies is the terminal full separating-axis rebuild skipped; any failed
  pair triggers the unchanged complete update before another iteration.
- Enumerate the complete visibility graph. Exact boundary-cone rejection avoids
  full intersection work on provably inward edges, but accepted and rejected
  evidence still covers every node pair.

No geometry, margin, derivative limit, or validation tolerance was weakened.
Shared fragment normalization removes only lower-dimensional runs while
preserving timestamps, valid regions, and original/protected geometry.

## Recorded measurements

All timings below use MATLAB R2024b on the same machine. Single-run measurements
are observations, not a universal speed guarantee.

| Measurement | Before | After |
| --- | ---: | ---: |
| Full-history versus requested-window preparation | 5.17 s | 1.48 s |
| Windowed preparation, exhaustive versus bounded alignment | 1.55 s | 1.20 s |
| 4,800 samples at 220 vertices, exhaustive versus bounded alignment | 4.66 s | 2.60 s |
| Tight moving detour, before versus after bounded alignment | 32.22 s | 26.15 s |
| Reproducible detour, previous repeated-axis run versus cached certification | 27.54 s | 21.03 s |

The final two detour polynomials are exactly equal. Length is 121.5032363 units,
duration 230 s, minimum certified separating gap 0.0009850922 units, and sampled
closest boundary distance approximately 0.00100036 units. Independent validation
passes. The original d6b46d5 full-moving run was stopped after 669.9 seconds
without a result; it is not a completed timing and does not establish a speedup
factor.

Tradeoffs: the earlier full-sweep detour took 18.13 s but was wider (122.0724
units). Tightening raises integrated squared jerk from 0.009596765 to 0.052908212
units^2/s^5, within the same limits. Small overlapping-cache extensions were
slightly slower after correlation (p50 0.00917 s versus 0.00789 s).

The separate early-crossing case improved from a 9.574 s median to 0.525 s
(one warmup plus three measured runs); its polynomial and separating planes
were exactly unchanged.

## Verification and size

The compact core suite contains 29 tests across planning core, arrival-search
regressions, bounded correspondence, affine cells, certificate reuse,
time-scoped planes, and the dense moving-obstacle quality check.

Production contains 5,671 nonblank, non-comment-only lines across the same 58
MATLAB files: 90 net lines above d6b46d5. Tests, examples, and benchmarks are
excluded. Projection work is excluded from this change.
