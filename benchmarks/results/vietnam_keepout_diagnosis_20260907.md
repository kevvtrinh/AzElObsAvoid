# Vietnam keep-out slew input diagnosis — 2026-09-07

The exact saved request succeeds on the current working source. Its main observed
problem is planning latency: 203.211984 s for a valid 30 s motion, with 180.112266 s
attributed to route search. A separate input-representation problem unnecessarily
selects conservative between-sample geometry for seven small regions.

No planner behavior or saved input was changed. This is diagnosis, not an
optimization comparison or an optimality claim.

## Reproduction

- Input: `Rogue Examples/vietnam_keepout_slew_input.mat`, SHA-256
  `DBD475086DA1398E4735887B78D3A5466A6C22DEB67174FB889540DE392C7CAC`.
- Revision: `5872d456d1fd6f31925b04a4491d3479ad4cb3e5`, branch
  `bmtp-cleanup-codex`, plus pre-existing working changes. The starting diff and
  source hashes are retained under ignored `tmp/vietnam-diagnosis-20260907/`.
- MATLAB R2024b Update 4, Windows. MATLAB failed to start inside the sandbox;
  the successful runs used the installed runtime outside it.
- Called `obstacleAvoidance.planTrajectory` with saved `protectedObstacles`,
  `initialState`, `goalState`, `limits`, and `options`, followed by a fresh
  `obstacleAvoidance.validateTrajectory(result)`.
- Eight moving regions, 31 samples each at 0:1:30 s, safety margin 1 unit.
  Start `[-0.0425414721, 3.5304588870]`; goal
  `[-0.6585415346, 12.8467632461]`; both at rest.
- Saved policy is `fixedArrival`, 30 s, sample time 0.05 s, no wrapping.
  Axis limits are velocity `[sqrt(2), sqrt(2)]`, acceleration `[2, 2]`,
  jerk `[4, 4]`, and workspace `[-42.4264068712, 42.4264068712]` on both axes.
  Resolved defaults include `MaximumSeedCount=2` and `MaximumTimeLayerCount=17`.
  The request has no random-seed field; this execution path uses deterministic
  route enumeration. No random input was regenerated.

| Exact replay metric | Measured value |
| --- | ---: |
| Planner / independent validation | Pass / pass |
| Termination | `goalReached` |
| Selected seed polyline length | 17.297991315814 units |
| Smoothed length, summed over returned samples | 17.305374620919 units |
| Motion duration | 30.000000000000 s |
| Public planner call wall time, including diagnosis assembly | 203.211984200 s |
| Recorded internal planning time | 197.507077500 s |
| First validated motion | 197.440283500 s |
| Collision / continuous kinematics | Pass / pass |
| Plane / seed-corridor certificate accepted | No / no |
| Continuous collision checks / unresolved intervals | 1699 / 0 |
| Reported minimum clearance beyond protected geometry | 0.000061818787 units |

The optional plane certificate is present but was not accepted by public
validation. Adaptive continuous collision validation passed instead; its empty
`CertificateRejectionReason` does not identify why the plane check declined.
This is a diagnostic limitation, not a failure of the returned motion. Sampled
peak velocities are `[1.1674760061, 1.0020209726]`, accelerations
`[1.1775893829, 0.9028664893]`, and jerks `[3.5861224741, 2.5729297307]`.
The independent polynomial checks also passed the continuous derivative limits.

![Validated motion and stage times](../../tmp/vietnam-diagnosis-20260907/result.png)

## Primary finding: timed visibility search dominates runtime

Failure class: `PERFORMANCE_LIMIT` (excessive observed work, with successful
completion). The expensive stage is topology/route search, before BMTP solving.

| Retained timing | Seconds |
| --- | ---: |
| Route search | 180.112266 |
| Motion solving | 14.083587 |
| Collision checking | 0.988099 |
| Other final validation | 0.187575 |
| Unattributed setup | 2.135552 |

The timed graph has 111 spatial positions and 65 time layers. Its complete
source/midpoint/uniform grid is retained; `MaximumTimeLayerCount=17` bounds
motion segmentation, not this search grid. The search expands 5,618 states,
accepts 56,571 motion edges and 5,618 waits, and records 4,817,696 timed
rejections (4,818,010 including static-graph rejections). Rejection counts
include cost and timing decisions, not just collision tests.

The spatial proposal has an estimated vertex work of 7,800, below the 10,000
dense-envelope threshold. It therefore does not defer timed search.
`searchRoutes.m:65` constructs all-pairs timed edge costs even though the
spatial visibility graph has only 149 accepted edges. Fixed arrival propagates
through the final layer, repeatedly checking candidate traversals. These
choices explain the scale; the static graph cannot simply replace timed edges
without potentially removing valid moving-obstacle traversals.

A separate profile of unchanged `searchRoutes` completed in 203.568703 s and
reproduced the same counts and 43-point timed route. Inclusive profile times
were 192.57 s in 2,392 `edgeIsClear` calls, 180.44 s in 2,457
`queryPreparedObstacles` calls, and 86.73 s in 85,728
`pointPolygonClearance` calls. Each edge uses 13 proposal collision samples.
These nested times must not be added together. Preparation and graph creation
were small in separate probes: 1.737 s and 0.635 s respectively.

The direct analytic motion is physically valid but collides. The early lateral
detour is ineligible because the fixed clock is 30 s while its physical lower
bound is 7.7947 s. The ordinary direct seed returns
`unsupportedDynamicDirectGuess`. The second, timed-visibility seed succeeds
through `bmtpTimedCell` after 10 iterations. Its preliminary static swept
projection reports a numerically unstable SOCP and returns no trajectory;
the later timed solve succeeds. That failed preliminary solve is not the
dominant cost or an overall planner failure.

The next performance investigation should target repeated obstacle/time queries
and safe candidate dominance in timed search, with identical-input correctness
and quality gates. No speedup has been demonstrated. Do not discard source times,
reduce protected geometry, or weaken continuous validation to accelerate it.

## Secondary finding: repeated closing vertices change preparation

Failure class: `INPUT` / `OBSTACLE` representation mismatch. The
[history contract](../../obstacle_history_contract.md) requires rings without
a repeated closing vertex. All 248 saved protected slices repeat that vertex;
normalization accepts and retains it.

In `prepareOneObstacle.m:231`, the repeated vertex creates a zero-length closing
edge. Non-translating rings then fail the strict-convexity check and select the
endpoint-hull enclosure. All 240 source intervals initially use
`conservativeEndpointConvexHull`.

A probe removed only exactly duplicated closing vertices from a copy of the
protected arrays. Every sampled polygon had exactly zero symmetric-difference
area. Seven regions then used `linearCorrespondingVertices` in all 210 of
their intervals. Region 2 remained a hull in all 30 intervals because its
concave motion does not satisfy the existing interpolation certificate.
A structurally different rotating triangle also used interpolation with an open
ring and a hull with an otherwise identical repeated closure.

The input-driven invariant is that an accepted ring's redundant closure must
not silently change its between-sample interpretation. A future correction
belongs in canonical ring normalization, or the exporter can follow the
documented open-ring format. No corrected-input planner run was performed,
so no path-length or runtime improvement is claimed from this probe.

For region 2, the saved protected area at t=0 is 47.135371 square units; the
prepared hull at t=0.5 is 64.582288 square units, about 37% larger. That fills
substantial concavity under the documented conservative model. Removing closing
vertices does not fix this separate limitation.

![Saved and prepared obstacle geometry](../../tmp/vietnam-diagnosis-20260907/geometry.png)

## Evidence and limits

Raw request, completed result, independent validation, profile, geometry probes,
source provenance, logs, and plots remain in ignored
`tmp/vietnam-diagnosis-20260907/` and `tmp/vietnam-*.log`. The exact successful
call is appended to `benchmark.csv`; stage-only probes are not planner runs.
Ten-minute external deadlines were set for the full replay and search profile;
both finished before those deadlines.

One exact planner run and one search profile establish the observed bottleneck,
not repeatable timing medians. Other diagnostic MATLAB processes briefly
overlapped; profiler overhead also affects the separate search measurement.
The supplied MAT file contains no prior result or reported exception to compare.
The diagnosis covers its saved planar model, not the upstream geographic
projection or spacecraft attitude dynamics. No maintained example matrix or
full test suite was run because production behavior was unchanged.

Two scratch reporting errors occurred after completed evidence had been saved:
an obsolete `SeedSummaries` result-field access and an incorrect aggregate
validation-timing field in plotting. They were corrected in the scratch scripts;
the successful planner result and its independent validation are unaffected.
