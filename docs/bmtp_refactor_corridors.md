# Static corridors, physical clocks, and shared verification

The corridor integration passed all 209 behavioral tests and all 60 frozen
quality comparisons. A final policy adjustment retains the original excursion
order for prescribed arrivals; its 39 focused tests and 18 matched quality
records pass. Earlier failed experiments remain documented below. This stage
preserves the broader excursion constructor where it still supplies supported
motion quality; it does not complete the full refactor.

## Formulation and displaced work

The selected monotone visibility guide determines which side of each exact
protected source facet the motion must follow. Facet intervals are compiled
once, pulled back through the analytic clock, and imposed on restricted Bezier
polynomials. The free coordinate integrates quadratic Bernstein jerk to a
quintic position. Positive Gauss–Legendre weights minimize polynomial speed
quadrature at the prescribed clock. Benchmark acceptance independently
integrates speed adaptively with the existing tolerances.

The formulation requires earliest arrival, rest endpoints, finite jerk limits, a monotone guide,
and a unique limiting axis. Equal limiting clocks, nonzero endpoints,
non-monotone guides, and unresolved corridor solves remain explicit. It uses
the existing direct-motion generator and preserves its per-axis limit semantics.
It neither searches dilations inside the reduced solve nor treats failure at
the bound as physical infeasibility.

Static sampled-excursion work is deferred when route search can supply a
corridor. A validated corridor attaining the direct clock displaces that work.
If the primary motions do not attain the clock, the original excursion resumes
from the already validated direct candidate. A one-seed request retains its
existing excursion path. Moving obstacles retain their broader formulation.

The experiment's separate `corridorEngine` package is not a production
dependency. Production shares power export, derivative-time bounds, midpoint
preparation, motion records, and complete region certification with the existing
engine. The duplicate `createMotionOutput.m` implementation is removed. The
shared record uses the prior exporter's vectorized exact squared-jerk integral.
Its absolute-time sample grid removes duplicate or near-duplicate samples that the old exporter
could create by translating an already deduplicated relative grid. Polynomial
coefficients, duration, and scalar-clock arithmetic are preserved.

## Certification and bounded batching

Every final span is checked against every applicable exact source region.
Prepared geometry now carries distinct exact and grouped coverage metadata.
Exact corridors use singleton source membership, rather than accidentally
describing their full source list as an eight-group approximation. The public
validator still reconstructs authoritative geometry and checks the full motion.

For sufficiently many static pairs, a neighboring span's plane direction is
only a proposal. The next complete hull and obstacle are checked again using
the scalar verifier's Bernstein products, offset correction, normal bound,
clearance inequalities, and roundoff reserve. Failed reuse returns to the
original analytic separating-axis and conic checks. Small pair sets retain the
scalar path because batch setup can dominate them.

Batches contain at most 256 regions and 65,536 source vertices, and bound the
control-by-region projection size. One oversized source region uses the scalar
linear-storage verifier. Tests cover scalar flags and signed gaps, degrees,
large coordinate offsets, acceptance boundaries, 513-region batch boundaries,
empty batches, and an oversized region. The initial test caught MATLAB's
one-region `repelem` orientation; explicitly making owners a column fixed it.

## Preserved unfavorable results

The initial unconditional static-excursion skip passed all 20 example requests
but made the saved circle arrive 1.800816 seconds later and the early rectangle
arrive 1.660316 seconds later. Saved route-economy and one-seed tests also
failed. That unconditional skip was rejected. Restoring the deferred broader
excursion recovered the original duration and route-quality checks. Tests that
required the superseded numerical refinement loop were changed to identify the
executed corridor, retaining their duration and executable-length bounds.

The first corridor benchmark made Hawaii slower: median 1.3979 to 1.6748 s.
One matched rerun retained the regression, 1.6471 to 1.7726 s. Separate profiling
attributed 0.771816 s to final source-pair certification, including 4,624
separating-hull calls. Bounded direction reuse reduced that certificate profile
to 0.201286 s. The next full experiment measured Hawaii at 1.418009 s, retaining
its 4.314235805 s duration and shortening the independently integrated path
from 12.919895 to 12.689507 units. This was near, but slightly slower than, the
preceding 1.3979 s capture; final production timing remains to be assessed.

The synthetic scaling experiment alternates scalar/batch order after warmup
and separately verifies every resulting pair with the scalar checker:

| Spans | Regions | Scalar / reuse median s |
| ---: | ---: | ---: |
| 2 | 4 | 0.002040 / 0.009063 |
| 2 | 40 | 0.007749 / 0.004220 |
| 2 | 400 | 0.062144 / 0.040717 |
| 20 | 4 | 0.008028 / 0.004514 |
| 20 | 40 | 0.061185 / 0.012263 |
| 20 | 400 | 0.598402 / 0.099172 |
| 100 | 4 | 0.030817 / 0.017324 |
| 100 | 40 | 0.309860 / 0.049114 |
| 100 | 400 | 2.984789 / 0.400895 |

The smallest case is unfavorable and remains recorded. Production enables
reuse at 128 applicable pairs; it does not force batching on every request.

## Eligibility and rejected overhead

The consolidated production capture retained all 60 physical-quality gates,
but its matched rerun exposed failed clock attempts adding work before the
unchanged general solve. Target exit increased from 2.5816 to 3.2121 s and
Philippines from 4.1603 to 4.7871 s. Profiles attributed 0.7988 and 0.9015 s,
respectively, to two clock attempts. These unfavorable measurements remain in
the benchmark history.

Source-interior guide rejection now precedes optimization. A necessary
free-coordinate reachability envelope intersects bounds from both rest
endpoints, using the integrals of the original velocity, acceleration, and
jerk limits. A linear feasibility problem then avoids constructing speed
cones when the fixed-clock linear constraints remain unresolved. None of
these checks declares global infeasibility or changes final clearance.
The four obsolete dilation variables were eliminated: durations are prescribed
throughout this reduced model.

The linear-check capture passed all 60 quality gates and measured slalom
0.7088 s, dense concave 0.3514 s, Hawaii 1.2159 s, and Philippines 4.1683 s.
Target exit remained slower at 3.0120 s; a separate profile still attributed
0.5978 s to failed clock work. The reduced formulation is therefore retained
only for earliest arrival, its intended lower-bound objective. Prescribed
arrivals use the existing general solver. A coordinate-exchanged detour at a
translated time origin checks that this fallback still meets its prescribed
arrival and passes fresh public validation.

Earlier full-suite coverage was 205/206 passing, followed by a
passing focused correction of the new exporter test's sample-grid oracle.
All existing physical behavior checks passed that run; the corrected oracle
compares the same polynomial at actual sample times and permits only the
removal of duplicate or near-duplicate timestamps. New eligibility tests
also passed focused runs before the latest full suite. The final complete
suite passed 209/209 with no incomplete tests. The final 20-case capture
preserves all supported outcomes and passes all 60 arrival, validation, and
independently integrated length comparisons.

The benchmark CSV preserves 504 corridor-stage experiment and rerun records.
MAT captures, failed runs, matched reruns, and profiles remain in ignored
`scratch`. The original one-warmup policy shows startup effects in slalom;
the final capture after the full suite has additional prior compilation and
is used for quality, not a clean standalone timing claim.

## Matched timing with explicit warmup

Both revisions then received three untimed calls requesting both outputs,
followed by three timed calls on the same six affected requests. This protocol
is now an explicit option in the maintained frozen-request benchmark. Saving
as MAT v7 is also optional and requires exact loaded-report equality; it changes
capture I/O only, outside planner timing. The original default measurement
protocol remains available and its raw history is retained.

| Case | Before median s | After median s |
| --- | ---: | ---: |
| Slalom | 1.957401 | 0.38285 |
| Dense concave | 0.844514 | 0.358824 |
| Target exit | 2.618548 | 2.628206 |
| Opposing U | 1.121080 | 1.231064 |
| Hawaii | 1.399675 | 1.250474 |
| Philippines | 4.107309 | 4.233530 |

All 18 matched quality comparisons pass. Slalom arrives 0.050094 s earlier
with a 0.014749-unit shorter arc. Hawaii's arc is 0.230388 units shorter at
the same arrival within floating-point error. Dense concave retains arrival
and slightly shortens its arc. The remaining three physical results are
unchanged. Opposing U retains roughly 0.110 s of extra work and Philippines
roughly 0.126 s: the conditional attempt is not free when the broader solver
is still needed. This limitation is retained explicitly alongside the much
larger gains; no claim of a speedup on every request is made.
The separate opposing-U profile confirms two clock attempts (0.3871 s
inclusive), including one linear feasibility call (0.1230 s inclusive);
source-facet compilation accounts for only 0.0338 s. Profile timings are
instrumented and are not substituted for the unprofiled table above.

Production is 114 MATLAB files and 16,408 physical lines, including comments
and blanks, versus 110/15,824 at the preceding commit and 108/15,528 at the
cleanup baseline. The duplicated exporter and obsolete dilation variables are
removed, but total production size has increased. Final consolidation remains
required; this stage does not satisfy the size gate.
