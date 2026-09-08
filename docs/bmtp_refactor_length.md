# Executable length independent of display sampling

The shared motion record now measures arc length from normalized polynomial
speed. All candidate selection and fixed-clock offset refinement consume this
same measure. Spatial route length remains the length of the route's actual
polyline. Changing a display sample interval therefore cannot change the
length objective by hiding curvature between samples.

The optimizer's positive eight-point speed quadrature remains confined to the
eligible corridor subproblem. Final motion measurement uses a separate
adaptive pair of 16/32-point positive quadrature rules. Only unresolved
intervals subdivide; a bounded-depth or large unresolved set uses MATLAB's
adaptive `integral` routine. The largest speed-evaluation batch contains 65,536
points. Persistent state contains only the quadrature constants, never request
geometry or motion data. This numerical length estimate does not certify
collision or physical limits; the public independent validator remains
unchanged.

The prototype agreed with the benchmark's separate adaptive integral on all
19 successful frozen motions to about 5e-14 coordinate units. Across 180
random degree-3/5/8 polynomials at coordinate scales 1e-5/1/1e5, its maximum
relative error against a separate 1e-12 adaptive integration was 3.6323e-15.
Typical warmed measurement time on maintained motions was 0.04–0.12 ms;
the first slalom measurement was 0.39 ms and is retained in the raw log.

Four regressions cover independent integration and large coordinate offsets,
reversals and stationary spans, the 2,051-span batch boundary, and two curves
whose coarse sampled lengths tie despite different arc lengths. The shorter
curve wins in both arrival modes, while an earlier longer curve still wins
in earliest mode. A public-planner detour returns identical polynomials and
selected attempts with 0.5 s versus 0.01 s sampling in both timing modes.
These tests and the shared-export, maintained-example, and saved route-economy
regressions all pass (19 focused tests).

The Vietnam regression still requires its original sampled length, duration,
route length, and search counts. Only the summary-length assertion changes
from the old sampled approximation (17.3053746209) to polynomial arc length
(17.3054420696); its physical-result assertions are retained.

The full suite passes 213/213 tests, with no incomplete tests. All 60 frozen
quality comparisons preserve the exact previously measured arrivals and arc
lengths: 19 successful independently validated motions, and the expected
no-path result. Every successful case retains collision, kinematic, and
applicable certificate success and `goalReached`; NoPath retains
`noValidatedSeed` and unavailable motion measures. All 60 runs are appended
to the benchmark CSV.

For the six requests with matching three-warmup captures at the preceding
stage, median times before/after are slalom 0.38285/0.33217 s, dense concave
0.35882/0.33378 s, target exit 2.6282/2.6372 s, opposing U 1.2311/1.2480 s,
Hawaii 1.2505/1.1772 s, and Philippines 4.2335/4.1950 s. All 18 quality
comparisons pass. These timings show no major new cost; this is an objective
correctness change, not a claim that adaptive length itself speeds up planning.
The full capture also retains unfavorable measurements, including static U
at 4.4858 s versus the preceding 4.3335 s capture (whose prior compilation and
warmup differ). No favorable rerun replaces that history.

Production is 115 files and 16,477 physical lines. The full refactor's
production-size gate remains outstanding. The following visibility prototype
initially stopped because its harness offset an empty 0-by-0 boundary array;
that occurred after the 213-test suite and length comparisons passed, and did
not change their results.
