# Vietnam maintained-example runtime follow-up (2026-09-07)

## Decision

Retain the fixed-arrival goal-cost bound, bounded candidate enumeration, and
diagnostic assembly changes. Replace `exampleObstacleAvoidance` with the exact
Vietnam keep-out request as `exampleVietnamKeepoutSlew`.

Acceptance required the frozen physical motion and independent public
validation to remain unchanged, the maintained scenario matrix to retain its
expected outcomes, the large temporary candidate tensor to have a fixed memory
bound, and interleaved runtime measurements to improve. The changes would have
been reverted for a validation regression, different returned motion, weakened
collision screening, or neutral/worse paired timing.

## Maintained example

The unchanged source MAT file moved from
`Rogue Examples/vietnam_keepout_slew_input.mat` to
`examples/data/vietnamKeepoutSlewInput.mat`. Its SHA-256 remains
`dbd475086da1398e4735887b78d3a5466a6c22deb67174fb889540de392c7cac`.
The new example uses the public planner and validator, accepts the common
headless/plot/jerk controls, and returns the planner result without adding
fields. Inventories, documentation, benchmark capture, and regression tests now
refer to the maintained name. The former static obstacle-avoidance example was
removed.

## Retained algorithm changes

- Fixed-arrival search keeps an incumbent route cost when the goal can safely
  wait to the final layer. A candidate is screened only when its accumulated
  spatial cost plus Euclidean distance to the goal is strictly worse than that
  incumbent. This is an admissible lower bound for Euclidean motion edges;
  equal-cost candidates remain eligible for the established tie policy.
- Candidate construction retains the vectorized enumeration from `a28ae64`, but
  processes consecutive source-node batches. Each temporary
  layer-by-node-by-source logical tensor is limited to 1,048,576 elements.
  Concatenating consecutive source batches preserves the original
  source/target/layer order. `CandidateBatchSplitCount` discloses when the
  bounded path is exercised. The Vietnam case remains a single batch.
- Accepted timed-BMTP solver diagnostics now have one complete owner instead of
  appearing both at the attempt root and under `TimedBmtp`. The flattened
  diagnostic table retains every field path while preallocating output and
  collecting homogeneous leaf-only structure arrays in blocks. Vietnam rows
  fell from 136,333 to 68,275.

The protected geometry, 65 temporal layers, 13 edge samples, motion limits,
validation tolerances, and fallback policy did not change. An experimental
convex-polygon occupancy fast path passed focused correctness checks but did not
improve timing and was removed.

## Vietnam result

The final result succeeds with `goalReached` and passes fresh independent
continuous validation. Its polyline length is 17.2979913158139 units, sampled
smoothed length is 17.3053746209189 units, and duration is 30 s. Collision and
all velocity/acceleration/jerk checks pass. The constructed time-cell plane
certificate passes, while the public validator records
`PlaneCertificateCertified = false` and completes adaptive continuous collision
validation with no unresolved intervals.

The route, time, position, velocity, acceleration, jerk, polynomial, arrival,
and duration arrays are exactly equal to the frozen `a28ae64` result. The goal
bound reduces accepted motion edges from 56,571 to 29,231 and records 214,274
goal-bound rejections; wait edges remain 5,618. Total rejected transitions fall
from 4,818,010 to 2,859,775.

## Runtime evidence

The early frozen-baseline repeats were 18.1285352, 17.9179281, and 17.7439961 s
(median 17.9179281 s). With the goal bound alone, identical-environment repeats
were 16.3070555, 16.2199419, and 16.3116872 s (median 16.3070555 s, 8.9903%
lower).

Later timings showed substantial host drift. An ABBAAB block had a 26.0121 s
baseline median and 20.7104 s candidate median (20.3817% lower), but included an
unfavorable 37.6722 s candidate outlier. That outlier is retained as evidence.
A symmetric follow-up produced candidate times 21.5921180 and 18.6957861 s and
baseline times 21.7180232 and 21.9528992 s.

The fully integrated, bounded-batching source was measured last in a CBBC block:

| Run | Variant | Wall time (s) | Success | Independent validation | Exact frozen motion |
| ---: | --- | ---: | ---: | ---: | ---: |
| 1 | Candidate | 17.8265419 | 1 | 1 | 1 |
| 2 | Baseline | 21.5374244 | 1 | 1 | 1 |
| 3 | Baseline | 22.1304158 | 1 | 1 | 1 |
| 4 | Candidate | 17.2935086 | 1 | 1 | 1 |

The final baseline median is 21.8339201 s and candidate median is 17.56002525 s,
a 19.5746% reduction. This is a measured improvement, not a runtime guarantee.

## Verification

- All 18 maintained examples produced the expected outcome: 17 independently
  valid motions and the expected `exampleNoPath` `noValidatedSeed` result.
- The full repository run initially passed 184 of 190 tests. The two failures
  introduced by integration (a source-contract pattern and legacy synthetic
  diagnostics without the new counters) were fixed and passed focused reruns.
  Thus all 186 tests not dependent on pre-existing Rogue Example fixture state
  passed across the full run and focused reruns.
- Four fixture-dependent tests remain unavailable or failing: three require the
  already deleted `failed.mat`, `pathtoolong.mat`, and `pathtoolong2.mat`; the
  `inefficientroute.mat` handoff expectation fails identically under the frozen
  implementation against the user's modified fixture.
- Code Analyzer reports no messages for the changed production, maintained
  example, and regression-test files. `git diff --check` is clean.

Actual maintained-example and final paired runs are appended to `benchmark.csv`.
Raw MAT profiles and comparison outputs remained outside source control.
