# Bounded exact visibility predicates

The visibility checker retains the preceding intersection arithmetic,
closed-boundary policy, midpoint containment, collinear-overlap handling, and
one request-wide roundoff scale. Temporary segment-by-edge arrays now have
at most 65,536 entries, instead of growing with the complete Cartesian product.
Already rejected segments need no further edge tests; survivors check every
source edge. The prior implementation is retained only as an independent test
reference. Graph-node selection and existing bounded search coverage are
unchanged; this is not a claim of complete continuous planning.

Tests require exact decisions on random segments, endpoint contacts,
zero-length segments, collinear overlap, concave outlines, holes, disconnected
regions, and offsets of 0, 1e6, and 1e9. Graph costs and recorded accepted and
rejected edges are also checked against the independent predicate. The full
suite passes 215/215 tests. All 60 frozen quality comparisons retain the same
supported outcomes, arrival times, and independently integrated lengths.

## Scaling experiment

Order alternates between the old and bounded predicates on identical arrays;
each entry is the median of three calls after warmup. Clear means every
segment lies away from the polygon; mixed requests also exercise rejection.

| Edges | Segments | Clear | Before s | Bounded s |
| ---: | ---: | :---: | ---: | ---: |
| 12 | 64 | No | 0.000489 | 0.000441 |
| 12 | 64 | Yes | 0.000206 | 0.000215 |
| 12 | 1024 | No | 0.001158 | 0.000796 |
| 12 | 1024 | Yes | 0.000561 | 0.000573 |
| 12 | 4096 | No | 0.002747 | 0.002633 |
| 12 | 4096 | Yes | 0.001118 | 0.001180 |
| 120 | 64 | No | 0.000735 | 0.000630 |
| 120 | 64 | Yes | 0.000369 | 0.000361 |
| 120 | 1024 | No | 0.004073 | 0.003839 |
| 120 | 1024 | Yes | 0.001728 | 0.001986 |
| 120 | 4096 | No | 0.027777 | 0.017790 |
| 120 | 4096 | Yes | 0.015341 | 0.008530 |
| 1200 | 64 | No | 0.002646 | 0.002917 |
| 1200 | 64 | Yes | 0.001650 | 0.001944 |
| 1200 | 1024 | No | 0.060108 | 0.047930 |
| 1200 | 1024 | Yes | 0.033109 | 0.023057 |
| 1200 | 4096 | No | 0.276029 | 0.183494 |
| 1200 | 4096 | Yes | 0.155222 | 0.095934 |

Small calls can pay a modest batching overhead. The retained design decision
is bounded memory and faster large predicates, not a universal speedup.
The initial experiment harness failed while offsetting an empty boundary
array; it now tests the empty scene without inventing nonexistent edges.
That harness failure is preserved separately from the passing product suite.

## Whole-request timing and attribution

The initial 20-request capture passed all quality gates. A single matched
rerun of initially slower cases also passed all 12 quality records, but showed
dense concave 0.4000/0.4134 s, moving circle 0.7393/0.8944 s, deforming outline
2.7722/3.1863 s, and rotating field 0.9363/1.0931 s. These unfavorable timings
remain in the raw logs and benchmark history. Separate before/after moving-circle
profiles contain **zero calls** to the changed visibility function; both retain
270 motion-record constructions, 268 offset splines, and 85 validator calls.
The saved search records are empty for moving circle, deforming outline, and
rotating field: each returns its fixed-clock excursion before graph search.
The timing differences on those paths therefore do not measure the changed
predicate. No favorable timing rerun replaces the unfavorable history.

Dense concave does execute graph search. Its small whole-request overhead is
retained as the explicit bounded-memory tradeoff, consistent with the small
predicate measurements. The checker remains an exact decision replacement,
with a measured large-array benefit and no change in planning or validation
coverage. Production is 115 files and 16,488 physical lines before final
consolidation.
