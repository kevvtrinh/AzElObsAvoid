# Bounded containment within clearance queries

The retained proposal replaces the all-point `polyshape.isinterior` query with
ray parity inside the existing edge-projection blocks. Each double projection
matrix targets at most 65,536 elements, allowing one complete edge row for a
larger polygon. All rings participate, preserving holes and disconnected regions.
Prepared convex shapes retain their existing half-space predicate, now within
the bounded blocks. Distance, nearest-point selection, edge order, boundary
snapping, and clearance settings are unchanged.

The first experiment failed near-edge equivalence, including at large coordinate
offsets. MATLAB R2024b's `polyshape` predicate includes a coordinate-scaled
determinant tolerance. The corrected implementation calls that original
predicate for points within its uncertainty band. This preserves the existing
occupancy policy instead of weakening the test. The failed log remains in
`scratch/containment_targeted.log`.

## Attribution and measurements

Separate pre-change profiling of the Philippines request measured 1.908 seconds
of self time in `polyshape.isinterior>check_inpolygon`, called 467 times, within
a 7.129-second profiled planner call. The U-shaped request instead spent most
of its runtime inside cone solves; this change does not target that cost.

The matched before/after profile of Philippines measured clearance inclusive
time of 2.5516 / 1.1519 seconds across the same 37 calls. The underlying
`check_inpolygon` self time fell from 1.9176 / 467 calls to 0.0563 / 450 calls:
the remaining calls handle only the uncertainty band. The deforming-US request
retained 4,213 clearance calls with inclusive time 0.3623 / 0.3564 seconds.
Its clearance self time rose from 0.2682 to 0.3063 seconds while the removed
containment work offset that cost. These are nested profile times, not additive
wall-clock components.

The microbenchmark varies vertex and query counts independently, checks exact
outputs, warms both implementations, alternates execution order, and retains
three measurements per input. Median seconds:

| Vertices | Queries | Original | Batched |
| ---: | ---: | ---: | ---: |
| 12 | 100 | 0.0003644 | 0.0002581 |
| 12 | 10000 | 0.0046851 | 0.0028575 |
| 120 | 100 | 0.0005794 | 0.0004394 |
| 120 | 10000 | 0.030259 | 0.016300 |
| 1200 | 100 | 0.0024610 | 0.0016473 |
| 1200 | 10000 | 0.317570 | 0.173090 |

## Full-request gates

All 194 MATLAB tests pass. The new tests compare signed clearance, nearest
points, and edge indices exactly on deterministic randomized rings, holes,
nested islands, disconnected shapes, boundary points, near-edge points,
coordinate offsets through 1e9, and prepared convex/concave queries.

All 60 full-request comparisons against pinned cleanup match returned
polynomials, histories, routes, certificates, search evidence, arrival times,
and adaptive arc lengths exactly. All 19 successful cases pass fresh independent
validation. The expected no-path case remains `noValidatedSeed`.

| Request | Pinned cleanup median s | All retained changes median s |
| --- | ---: | ---: |
| Alternating slalom | 2.21325 | 1.9011 |
| Dense concave | 1.38465 | 0.96253 |
| Opening U | 0.69454 | 0.50541 |
| Hawaii | 2.2110 | 1.3897 |
| Philippines | 5.5686 | 4.1861 |
| Static U | 4.3238 | 4.3017 |
| Deforming US | 2.9056 | 3.2201 |
| Moving barrier | 0.20997 | 0.22374 |
| Earliest-interception frozen request | 0.008406 | 0.011312 |

The complete CSV preserves every case and repetition, including unfavorable
results. The aggregate comparison includes the earlier offset translation and
static-cache changes; only the microbenchmark and matched profiles isolate
containment.

The one matched rerun against the immediately preceding containment function
measured deforming-US medians 3.0433 / 2.8391 seconds; its initial slowdown did
not persist. The barrier slowdown also did not persist. Early obstacle-free
interception repetitions in that rerun show substantial JIT startup despite
one untimed warmup, so they do not establish a speedup. The function-selection
assertion failed before any timing in the first rerun launch because MATLAB's
current directory outranks `addpath`; the corrected harness leaves the package
root and asserts the selected implementation. Both logs are retained.

Planner timings exclude example setup and the interception wrapper's search.
Full replay uses the same one-warmup/three-repetition policy on both sides;
this does not guarantee a fully steady JIT state. Profiles run separately.
The source remains MATLAB R2024b; other releases have not been verified.

Production is still 108 files and **15,577 physical lines**, versus 15,528 at
cleanup baseline. This stage improves a measured cost but does not finish the
required solver replacements or code reduction.
