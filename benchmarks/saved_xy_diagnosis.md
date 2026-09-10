# Saved moving-detour diagnosis

The preserved request is `tests/fixtures/savedMovingDetour.json`, copied from
`RogueCasses/x-y-request.json`. It contains a translating rectangle with 21
keyframes, a rotating 38-vertex concave obstacle, a 180-second horizon, and the
original per-axis motion limits and safety margins.

## Retained dense-history path

Dense, fixed-position, rest-to-rest earliest-arrival requests first build a
time-expanded route proposal. The spatial nodes come from the sampled protected
obstacle union. The time layers contain each source time, each source-interval
midpoint, and nine uniform horizon samples. Forward search enumerates motion and
wait transitions and selects the first reachable goal layer. It does not compute
homology or enumerate homotopy classes.

The selected route is only a BMTP seed. A 16-span, degree-eight timed BMTP solve
discovers colliding span-region pairs by sampling and activates a maximum-margin
separating line for each newly discovered pair. It then refines travel at the
selected arrival clock. Success still requires a new certificate against the
exact affine time cells followed by the public independent validator. The
proposal's sampled visibility checks cannot approve a motion.

The timed path is used when the source history contains at least 16 intervals,
so its fixed mesh compresses the input history. Sparse histories retain the
existing chronological fixed-arrival search and validated delayed-chord
incumbent. If the timed proposal or BMTP solve fails, the same fallback runs.

## MATLAB R2024b measurements

All runs replayed the identical saved JSON with plots and profiling disabled.
The full public `planner` call includes route search, BMTP, exact recertification,
independent validation, and result assembly.

| Implementation | Full planner runtime (s) | Arrival (s) | Motion length (units) |
| --- | ---: | ---: | ---: |
| Prior chronological branch | 236.291 | 134 | 240.730295011 |
| `bmtp-cleanup-codex` (`c04f3b2`) | 55.443810, 49.511337, 48.159892 | 117 | 229.949945793 |
| Retained timed path | 16.987219, 14.909188, 14.412398 | 117 | 229.959020399 |

The retained median is 14.909188 seconds: 15.85 times faster than the prior
chronological run and 3.32 times faster than the cleanup branch median. A final
post-audit replay took 14.843730 seconds. Relative to cleanup, arrival is
identical and motion length increases by 0.009074606 units (0.00395%).

The solver considered 188 applicable span-region pairs, tagged 13, performed 11
trajectory SOCPs and 78 separating-line SOCPs, and exported a degree-eight C3
motion. The exact final certificate and public validator both passed.

The earlier roughly seven-second observation was a fixed-arrival active-set
prototype, not the full earliest-arrival planner. That generic prototype was not
retained because it changed the maintained 220-vertex path by 0.00134342 units
and changed its plane-update behavior. The dedicated timed path preserves the
220-vertex fixture exactly.

## Limits of the earliest-arrival claim

Arrival 117 seconds is the earliest reachable layer in this deterministic
time-expanded graph. `GlobalEarliestProven` remains false because the finite
layer set and sampled proposal edges do not prove a continuous-time global
minimum. Collision freedom and motion limits are exact for the returned
polynomial certificate; optimality between unsearched times is not claimed.

## Batched occupancy milestone

Profiling the committed timed planner attributed 12.26 seconds to route search,
including 8,702 public occupancy calls. Rechecking obstacle preparation inside
those calls consumed 4.12 seconds. The moving-scene search now batches each
edge's original thirteen samples into one public query, bounded to 262,144
sample positions per batch. Sample positions, times, boundary policy, candidate
transitions, and final certification are unchanged.

Fully batching stationary histories was rejected: a 49-layer crossing-barrier
search increased from a 0.282-second median to 1.466 seconds. The retained
change keeps the original occupancy cache whenever the prepared history has a
stationary interval, using batching only for continuously changing geometry.

Three paired search-only runs on the saved request had medians of 10.873656
seconds before and 7.871029 seconds after. Routes, clocks, and every search
record field matched exactly. Two structurally different stationary-history
comparisons (13 and 49 uniform layers, plus obstacle events) also matched
exactly and retained cache performance.

Full planner runs took 13.833663, 11.554937, and 11.078550 seconds, a median of
11.554937 seconds (22.5% below the previous 14.909188-second milestone). Each
returned exactly the previous polynomial, arrival 117 seconds, motion length
229.959020399 units, and a passing independent validator. The production change
adds only 16 lines to the existing search function.

## Early rejection and solver setup milestone

Profiling the batched version attributed 6.14 seconds to polygon tests. The
moving-scene batch now tests each edge's midpoint first and only batches the
remaining twelve samples for edges that pass. This preserves the original
sample positions and rejection predicate while avoiding work on blocked edges.
Stationary intervals continue to use the existing cache.

Three paired saved-case search runs produced medians of 7.914062 seconds before
and 5.977440 seconds after. All route, clock, and search-record fields matched
exactly. The 13- and 49-layer stationary-barrier comparisons also matched.

The timed solver also creates unchanged options outside its iteration loops,
and the one-use plane wrapper was removed. The isolated solver's three runs
were 4.149884, 2.842837, and 2.762659 seconds before and 4.170382, 2.780654, and
2.737168 seconds after. This small timing difference is not a substantial
standalone speedup; the retained change simplifies the code and returns exactly
the same polynomial and full collision certificate.

Together, these changes produced full planner times of 12.224884, 9.677679,
and 9.314708 seconds. The median is 9.677679 seconds, 16.2% below the previous
11.554937-second milestone. All three runs matched the retained polynomial,
route, and complete search record exactly and passed public validation. The
net production diff for this milestone removes two lines.

## Complete preparation reuse milestone

A stationary-geometry query grouping experiment was rejected: saved-case
search medians were 5.985077 seconds before and 6.126004 seconds after, with
identical results. Its bookkeeping cost outweighed the saved geometry calls.

The retained change instead skips preparation extension when every sample and
interval is already prepared. This check runs only after the existing source
snapshot equality check, so changes to source geometry still rebuild the cache.
It adds five production lines and leaves public query behavior unchanged.

Three paired saved-case searches had medians of 5.965344 seconds before and
5.555977 seconds after. The 13-layer stationary-history median improved from
0.058421 to 0.048296 seconds, and the 49-layer median from 0.285898 to 0.237074
seconds. Every route, clock, and search-record field matched in all nine pairs.

Full planner runs took 10.105315, 9.362836, and 8.773518 seconds, with median
9.362836 seconds versus the previous milestone's 9.677679 seconds. All three
preserved the exact polynomial and search record and passed public validation.
The new regression checks both partial-cache extension and rebuilding after
an authoritative source edit, including occupancy and blocking-obstacle output.

## Interior sample batching milestone

The moving-scene search now queries the quarter, midpoint, and three-quarter
samples together, rejecting blocked edges before querying the other ten original
samples. Acceptance still requires all thirteen original positions and times to
be clear. Stationary-history caching and independent certification are unchanged.

Paired search-only experiments rotated evaluation order across three runs. A
midpoint-then-quarter-points variant had a 4.228672-second median versus 5.515602
seconds for the prior implementation. Splitting the remaining checks into more
batches was slower: medians were 4.777291 and 5.686870 seconds. A second comparison
selected the combined three-interior-sample batch: its median was 4.093394 seconds
versus 4.207000 seconds for separate midpoint and quarter batches and 5.651579
seconds for the prior implementation. All routes, clocks, and complete search
records matched exactly.

Full planner runs took 9.515573, 8.053248, and 7.395710 seconds. The 8.053248-second
median is 14.0% below the previous 9.362836-second milestone. All three returned
the identical polynomial and search record, arrival 117 seconds, motion length
229.959020399 units, and passing independent validation. Counts remain 13 active
out of 188 applicable span-region pairs, 11 trajectory solves, and 78 plane solves.
The production diff adds 17 lines and removes 18, a net reduction of one line.
The continuously moving and stationary crossing-barrier cases also matched the
reference route, clock, and full search record.

## Prepared query snapshot milestone

The next profile attributed 5.07 seconds to search and 4.71 seconds to its 1,680
public occupancy queries, including 2.22 seconds in polygon tests and 0.64 seconds
in preparation across the profiled planner. These inclusive times overlap and
are not summed. The query count and repeated source checks motivated preparing
the local obstacle snapshot once at search entry.

Two alternatives were not retained. Grouping point indices by query time saved
only 1.5% in search (medians 4.198729 versus 4.136877 seconds). Wider initial
sample batches were slower: the retained quarter/midpoint/three-quarter batch took
4.097923 seconds, versus 4.130804 for sixth-position samples and 4.675055 or
4.958634 for five-point batches. Every tested variant preserved the full search
record.

The retained change extracts the unchanged occupancy computation into one shared
internal function. Public queries still validate inputs and refresh preparation;
search prepares its local snapshot once and queries it throughout the search.
The extracted polygon-query computation is textually identical. Boundary policy,
first blocking-obstacle order, source refresh, and final validation are preserved.

Three paired search runs had medians of 4.119161 seconds before and 3.803443 seconds
after, with identical routes, clocks, and complete search records. Full planner
runs took 9.146784, 7.430817, and 7.036871 seconds: median 7.430817 seconds versus
8.053248 seconds for the previous milestone, a 7.7% reduction. Each full run
preserved the exact polynomial, search record, arrival, path length, and solve
counts, and passed independent validation. The separate moving and stationary
crossing-barrier searches also matched their reference records exactly.
The net production increase is 20 lines, including the shared function's help.
Regression coverage checks boundary inclusion, first blocking index, inactive
times, empty and mismatched query shapes, partial search preparation, and source
edits that invalidate an existing cache.

## Sparse derivative assembly milestone

The latest profile attributed 4.03 seconds to BMTP, including 2.62 seconds in
90 conic solves. The trajectory-step builder constructed each derivative order's
identical sparse rows once per span: 576 repetitions across twelve steps in the
saved case. The retained change constructs three derivative blocks per step and
places them across all spans with a sparse Kronecker product. Original row order,
coefficient values, variable order, bounds, cones, objective, and options remain
unchanged. The net production diff removes five lines.

Captured complete coneprog inputs matched exactly in sixteen cases: one, four,
sixteen, and forty spans; degrees five and eight; and earliest- and fixed-arrival
objectives. Each case included active separating planes. Three paired full BMTP
runs took 4.091014, 2.786319, and 2.834638 seconds before, versus 2.909493, 2.737703,
and 2.699731 seconds after. Medians were 2.834638 and 2.737703 seconds. All runs
returned exactly the same polynomial and full collision certificate.

Full planner runs took 9.339165, 7.326564, and 6.954518 seconds, with median
7.326564 seconds versus the previous 7.430817 seconds. This 1.4% full-runtime
difference is modest, and the first run was slower than the previous milestone's
first run. The smaller implementation and paired BMTP measurements support
retention. All three preserved the exact polynomial and search record, arrival
117 seconds, motion length 229.959020399 units, and passing independent validation.

## Stationary obstacles within moving scenes milestone

The saved scene contains one changing four-vertex obstacle and one stationary
38-vertex obstacle. Whole-scene stationary caching cannot apply while the first
obstacle moves, leaving repeated polygon tests against the second obstacle.
The retained change caches the thirteen-sample result for each directed edge
against exactly unchanged source boundaries. Unknown edges are evaluated in
bounded batches. The cache is local to one prepared search snapshot and requires
only one byte per possible directed edge, subject to the existing 300 MiB budget.

Cached results apply only when every original sample time lies inside every
cached obstacle's active interval. Outside that interval the original complete
query runs. Moving-obstacle queries still test their original samples. The
existing whole-scene stationary cache is unchanged. This adds 38 net production
lines to the existing search function without adding a helper or changing any
geometry, boundary policy, search candidate, or final validation.

Three paired saved-case searches took 4.357932, 3.726654, and 3.630900 seconds
before, versus 1.244283, 1.104812, and 1.075063 seconds after. The medians fell
from 3.726654 to 1.104812 seconds, a 70.4% reduction. All routes, clocks, and
complete search records matched exactly. Eight structurally different mixed
scene comparisons also matched: scalar-time and finite-lifetime stationary
obstacles, an obstacle that appears at three seconds and disappears at six,
a near-equal moving boundary, and both obstacle input orders.

Full planner runs took 7.171086, 4.838624, and 4.517740 seconds. The median is
4.838624 seconds versus the previous 7.326564 seconds, a 34.0% reduction. All
three returned the identical polynomial and search record, arrival 117 seconds,
motion length 229.959020399 units, unchanged solve counts, and passing public
independent validation.

The saved search has 66 nodes, requiring 4,356 bytes for this cache. The final
profile recorded 1,356 prepared queries, 6,791 polygon calls, and 6,527 prepared
shape evaluations, versus 1,680, 12,270, and 10,705 before this change. Cached
stationary rejections also avoid subsequent moving-obstacle queries. All 33
MATLAB tests passed and Code Analyzer reported no issues in the changed files.

## Query-only boundary evaluation milestone

Sparse storage for conic constraint matrices was tested and not retained. Three
paired BMTP runs had medians of 2.756250 seconds before and 2.775011 seconds with
sparse cone storage, despite identical polynomials and full certificates.

The retained change lets occupancy queries skip convexity and orientation
classification while evaluating the exact same prepared boundary. Classification
remains enabled by default for every other caller. Active state, coordinates,
edges, speed bounds, source indices, and interpolation model remain unchanged;
the opt-out leaves classification flags false. The net production increase is
three lines, including documentation and the caller explanation.

Three paired saved-case searches took 1.676787, 1.220252, and 1.068012 seconds
before, versus 1.344851, 1.143647, and 1.039127 seconds after. The medians were
1.220252 and 1.143647 seconds; routes, clocks, and complete search records matched
exactly. Full planner runs took 6.765425, 4.598434, and 4.390252 seconds, a median
of 4.598434 seconds versus the previous 4.838624 seconds (5.0% lower). All three
returned the exact polynomial and search record, unchanged arrival and motion
length, and passing independent validation.

The new regression compares prepared geometry across convex, concave, and holed
boundaries at source times, an interpolated time, and inactive times. It also
checks that default convexity classification remains enabled.

## Cached exact sample identity milestone

Repeating the query-time grouping experiment after the earlier optimizations
was unfavorable: search medians were 1.073290 seconds before and 1.129546 seconds
after. A separate BMTP experiment compared six (the host default), one, and two
MATLAB computation threads. Medians were 2.757538, 2.769479, and 2.743980 seconds,
with identical polynomials. This small difference did not justify changing
thread configuration; the experiment restored the original setting.

The retained change stores preparation's existing exact source-array comparison
in `SamplesExactlyEqual`. Queries and stationary-edge caching reuse this flag
after the existing source-snapshot check. It is distinct from `IsTimeInvariant`,
which can also describe geometrically equivalent boundaries with different vertex
orders. Preparation version five rebuilds older caches. The net production diff
removes one line by eliminating the two duplicate comparisons.

Three paired searches took 1.537176, 1.106413, and 1.028898 seconds before, versus
0.891028, 0.717018, and 0.679611 seconds after. Medians fell from 1.106413 to
0.717018 seconds, with every route, clock, and search-record field unchanged.
Full planner runs took 6.376267, 4.266033, and 4.024277 seconds. The 4.266033-second
median is 7.2% below the previous 4.598434 seconds. All three retained the exact
polynomial and search record, unchanged arrival and length, and passing public
independent validation.

The new regression covers old-cache migration, an authoritative edit that changes
stationary samples into moving samples, and equivalent polygons whose starting
vertex differs. It verifies that geometric equivalence cannot substitute for
exact source-array identity in occupancy queries.

## Visibility rejection order milestone

Graph setup still performs midpoint occupancy checks and segment-boundary
intersection checks for each proposed edge. Two exact reorderings were compared
across all seven saved graph attempts. Testing midpoint occupancy first and
skipping intersection work for blocked midpoints had a 0.315204-second median.
Testing boundary crossings and collinear overlaps first, then midpoint occupancy
only for surviving edges, was faster at 0.188816 seconds versus the original
0.360321 seconds. Each variant preserved every graph-attempt field exactly,
including the nodes, edges, rejection counts, costs, and connectivity recovery.

The retained boundary-first ordering adds two net production lines. It preserves
the original coordinate-scale calculation over all input segments and all
intersection tolerances. Interior-only segments still undergo midpoint rejection;
boundary contacts and collinear overlaps still block an edge.

Full planner runs took 6.318004, 4.175128, and 3.875344 seconds. The median is
4.175128 seconds versus the previous 4.266033 seconds, a modest 2.1% reduction.
All three preserved the exact polynomial, complete timed-search record, and every
graph-attempt record, with unchanged arrival and motion length and passing public
independent validation. The new regression covers clear exterior edges, interior
segments, crossings, boundary contacts, an all-blocked batch, an empty scene,
and a polygon with a hole.
The all-blocked test caught MATLAB's rejection of an empty midpoint query; the
retained implementation returns immediately when no edge survives boundary checks.
A final replay after that guard took 4.075942 seconds and matched the entire timed
proposal record and polynomial exactly. All 36 MATLAB tests passed and Code
Analyzer reported no issues in the changed files.

## Smaller separating-plane assembly milestone

The timed separating-plane solve now assembles its two endpoint-normal cones and
two obstacle-row blocks directly. It removes the empty cone placeholder and loop
index bookkeeping while preserving the seven-variable layout, row order,
coefficients, bounds, objective, options, and independent verification. The net
production diff removes sixteen lines.

Captured complete coneprog inputs were identical in twelve configurations:
degrees three, five, and eight with three, four, sixteen, and sixty-four obstacle
vertices. A setup-only microbenchmark of 1,000 calls had medians of 0.117841 and
0.084462 seconds. This 28.3% assembly reduction is small relative to conic solve
time. With both full BMTP implementations warmed first, paired runs were
2.792644, 2.692809, and 2.716767 seconds before, versus 2.675528, 2.764553, and
2.718773 seconds after. Their medians, 2.716767 and 2.718773 seconds, show no useful
full-solver speed difference. Polynomials and full certificates matched exactly.

Full planner runs took 6.291331, 4.143754, and 3.831818 seconds. The median was
4.143754 seconds versus the prior 4.175128 seconds; this small difference is not
treated as a reliable speed gain. The change is retained for its smaller code and
unchanged numerical problem. All three returned the identical polynomial and
complete timed proposal record, with unchanged arrival, motion length, solve
counts, and passing independent validation.

## Rejected Bezier sampling batches

The overlap sampler was tested with de Casteljau evaluation batched across two,
four, eight, or sixteen spans instead of one span at a time. All five full BMTP
implementations were warmed before three timed repetitions with rotating order.
Every run returned the identical polynomial and complete collision certificate.

| Spans evaluated together | Full BMTP median (s) |
| --- | ---: |
| 1 (retained) | 2.696645 |
| 2 | 2.717576 |
| 4 | 2.710577 |
| 8 | 2.713449 |
| 16 | 2.705856 |

No batch size demonstrated a speed benefit. For the saved case's 1,201 samples
and degree-eight spans, the initial recurrence array grows from 172,944 bytes
for one span to 2,767,104 bytes for sixteen spans; these are array sizes, not
measured peak process memory. The original sampler is retained without any
production change.

## Rejected endpoint-variable elimination

An experimental conic formulation substituted the twelve endpoint-control
coordinates already fixed by endpoint position, velocity, and acceleration
equalities. It shifted linear right-hand sides and affine cone offsets, removed
the resulting zero equalities, and reconstructed the full control vector after
solving. Solver tolerances and the mathematical constraints were unchanged.

With both implementations warmed, three paired full BMTP runs had medians of
2.696318 seconds for the retained formulation and 2.421058 seconds when substitution
was used in both alternating optimization and final refinement. Arrival remained
117 seconds and each engine collision certificate passed. However, motion length
increased from 229.959020398834 to 229.964562511439 units, and integrated squared
jerk increased from 0.991397232846 to 0.993329047218 units squared per second to the
fifth power. The alternating process performed ten trajectory solves and 65 plane
solves instead of eleven and 78; the numerical reformulation changed its path and
stopping behavior.

A second paired comparison isolated the two phases:

| Endpoint substitution | Full BMTP median (s) | Motion length (units) |
| --- | ---: | ---: |
| None, paired reference | 2.647535 | 229.959020398834 |
| Alternating phase only | 2.397155 | 229.982241836290 |
| Final refinement only | 2.637803 | 229.959768507023 |

Every variant returned a longer path. Refinement-only substitution slightly
reduced squared jerk, but did not provide a useful runtime improvement. None is
retained: the faster variants trade away motion quality and require additional
reduction/reconstruction code. Production remains unchanged.

## Bounded per-search boundary cache milestone

The timed search now keeps a fresh map per prepared obstacle, keyed by exact
physical query time. It reuses only the active flag and protected boundary
coordinates; every occupancy query still tests its own points and boundary
policy. No time quantization, route pruning, or validation changes are involved.
The maps stay inside the search and are absent from returned prepared obstacles.
Source changes still invalidate preparation before public occupancy queries.

Entry capacity is the floor of 16,384 divided by obstacle count and the largest
source coordinate-array length for that obstacle. This limits entry count as
geometry grows; it is not a measured byte limit. When full, the map leaves new
times uncached and computes their geometry normally. The saved request used
482 of 1,024 entries for the moving obstacle and zero of 122 for the stationary
obstacle, which already bypasses interpolation.

Three paired search runs, alternating method order, took 1.433433, 0.761420,
and 0.708109 seconds before, versus 0.751232, 0.611988, and 0.540516 seconds
after. Medians were 0.761420 and 0.611988 seconds, a 19.6% reduction. All routes,
route times, and search-record fields matched exactly. These runs include
first-use compilation effects; the later repetitions also favored the cache.

Full planner runs took 6.402025, 3.888072, and 3.733804 seconds, including a
slower first run. The median was 3.888072 seconds versus the prior 4.143754
seconds, a 6.2% reduction across separate benchmark sessions. The paired search
comparison supports the mechanism more directly than that full-planner delta.
All three full runs preserved the exact polynomial and complete timed-search
record: arrival 117 seconds, length 229.959020398834 units, 13 tagged pairs out
of 188 applicable pairs, eleven trajectory solves, and 78 plane solves. Public
independent validation passed. Arrival remains the first reachable configured
time layer; continuous-time global earliest arrival is not proven.

The production change adds sixteen net lines across two existing functions.
A new moving-box regression checks capacity exhaustion, fresh points and
boundary policy on cache hits, source-coordinate invalidation, and active-time
invalidation. All 37 MATLAB tests passed and Code Analyzer was clear for the
changed MATLAB files.

## Smaller continuity and active-plane assembly milestone

A fresh profile after the boundary cache attributed 2.570432 of 4.805993
profiled planner seconds to coneprog, including 2.433209 seconds in the twelve
timed trajectory steps. These inclusive times overlap and must not be added.
The trajectory-step function's self time was 0.069665 seconds, so reducing its
assembly cost has limited potential for total runtime.

Continuity rows now use sparse Kronecker blocks for C0-C3 joins. Plane assembly
visits active pairs directly, transposing the active mask before `find` to
preserve the former segment-major row order. Endpoint rows, coefficients,
constraint order, variable layout, cones, bounds, objectives, and solver options
remain identical. The combined production diff removes eleven net lines from
one existing function.

Captured complete coneprog inputs matched exactly in 72 configurations: degrees
three, five, and eight; one, three, sixteen, and thirty-two spans; both arrival
modes; and zero, one, or multiple active planes. The multiple-plane cases varied
normals and mixed whole-span and restricted time fractions. One hundred
assembly-only calls had medians of 0.139443 seconds before and 0.068476 seconds
after. This comparison excludes the actual conic solve.

With both full BMTP implementations warmed, three paired runs in alternating
order took 2.747687, 2.684560, and 2.862167 seconds before, versus 2.694268,
2.727837, and 2.937539 seconds after. Medians were 2.747687 and 2.727837 seconds;
this small, noisy difference does not demonstrate a useful full-solver speedup.
Every polynomial and complete collision certificate matched exactly. A prior
continuity-only experiment likewise showed no useful full-solver improvement.

Full planner runs took 5.533135, 3.961056, and 3.732710 seconds, with median
3.961056 seconds versus the preceding 3.888072 seconds. These separate-session
measurements do not establish an end-to-end gain. The change is retained for
smaller code and faster assembly of the identical numerical problem. All three
returned the exact reference polynomial and timed-search record, with arrival
117 seconds, unchanged length and solve counts, and passing independent
validation. All 37 MATLAB tests passed and Code Analyzer was clear.

## Rejected join-constraint reductions

Three mathematically motivated reductions were tested in isolated copies of the
current timed BMTP implementation. Each comparison warmed both implementations
before three paired repetitions with alternating execution order. The seed,
regions, limits, horizon, solver options, and engine certification were identical.
These are full BMTP timings, not full public-planner timings. Every candidate
listed below arrived at 117 seconds and passed the engine plane certificate;
none was retained or promoted through a new public-planner validation run.

First, equal-duration C1-C3 continuity makes the last derivative bound on one
span identical to the first on the next. Removing one copy eliminates 180
inequality rows in the saved sixteen-span, degree-eight case. Captured solver
inputs in 24 configurations confirmed that every removed row differs from its
retained counterpart by a scaled continuity equality with zero right-hand side.
All other inputs were identical. Degrees three, five, and eight; one, three,
sixteen, and thirty-two spans; and both arrival modes were covered.

| Duplicate derivative bounds removed in | Paired reference median (s) | Variant median (s) | Length (units) | Integrated squared jerk (units squared / s^5) |
| --- | ---: | ---: | ---: | ---: |
| Neither phase, retained motion | -- | -- | 229.959020398834 | 0.991397232846 |
| Both phases | 2.703741 | 2.653702 | 229.951882889269 | 0.993718775493 |
| Alternating phase only | 2.696334 | 2.647173 | 229.935359875462 | 0.995186276728 |
| Final refinement only | 2.696334 | 2.639874 | 229.962940855175 | 0.993181057991 |

The modest timing reductions came with increased squared jerk in every variant;
refinement-only removal also lengthened the path. All retained eleven trajectory
solves and 78 plane solves. The original row formulation is kept.

Second, direct C0-C3 substitution expressed each span's first four controls in
the preceding span's last four. This removed 120 decision variables and join
equalities from the saved case, transforming the objective, constraints, and
cones before reconstructing all controls. Bounds on eliminated coordinates were
preserved as additional linear inequalities. The median increased from 2.678864
to 2.809179 seconds. Length increased to 229.968214701339 units and squared jerk
to 0.994232580704; solve counts stayed eleven and 78. It is rejected for both
runtime and motion-quality regressions.

Finally, sharing only the identical endpoint position at each join removed
thirty variables and C0 equalities while keeping C1-C3 equalities. The paired
median increased from 2.712557 to 2.916091 seconds. Length increased to
229.983750600724 units, although squared jerk fell to 0.990358714118. The
alternating process needed twelve trajectory solves and 91 plane solves.
This is also rejected. Fewer variables or redundant rows do not guarantee faster
conic solves or unchanged biconvex convergence. Production remains unchanged.

## Rejected cached bounding-box filters

Two variants added exact axis-aligned bounds to the existing per-time boundary
cache. The first filtered exterior query points before calling `inpolygon`;
the second skipped the call only when every query point was exterior. Both used
the same inclusive min/max predicate as MATLAB R2024b's own polygon prefilter.
No polygon, interpolation, margin, tolerance, time, or route changes were made.

An initial three-run comparison was inconclusive: search medians were 0.559840
seconds before and 0.579600 seconds with point filtering. A subsequent comparison
warmed all three methods and rotated execution order over five repetitions:

| Search query implementation | Median (s) | Profiled `inpolygon` calls |
| --- | ---: | ---: |
| Retained boundary cache | 0.539762 | 6,730 |
| Cached bounds, filter exterior points | 0.511027 | 1,124 |
| Cached bounds, reject wholly exterior queries | 0.525275 | 1,124 |

Every route, clock, and search-record field matched exactly. Although the
point-filter variant reduced median search time by 5.3% in the longer comparison,
it adds five production lines and stores four more numbers per cached boundary.
The smaller rejection variant adds four lines and was slower than point filtering.

Three full planner runs with point filtering took 6.331766, 4.064565, and 3.747171
seconds, compared with the previous milestone's median of 3.961056 seconds.
To separate startup effects from search savings, five further paired full
planner runs used the prefilter-free search query as the reference, with the
rest of the pipeline held identical. Both methods were warmed first and their
order alternated. Reference times were 3.808311, 3.691228, 3.567341, 3.593647, and
3.551801 seconds. Point-filter times were 3.588280, 3.624662, 3.533815, 3.614849,
and 3.595546 seconds. Medians of 3.593647 and 3.595546 seconds demonstrate no
useful full-planner improvement. Every full run retained the reference
polynomial, complete timed-search record, and passing independent validation.

The existing regressions passed, as did an experimental moving-hole check for
outer and inner boundary policy, points one floating-point spacing outside the
bounds, inactive times, and both cached and uncached queries. That fixture uses
an explicit middle keyframe: between unsupported multi-ring samples, preparation
correctly uses its conservative union rather than the initially assumed linear
hole translation. The test expectation was corrected without changing geometry.
Both optimization variants and the experiment-only test were discarded; the
simpler production implementation and maintained test suite remain unchanged.

## Newly discovered travel-plane initialization fix

A coordinate-scaling experiment reached an untested refinement path: a shorter
trial intersects a previously untagged obstacle region, so refinement solves for
and inserts a new separating plane. The returned plane lacked `TimeFraction`,
which was already present in the stored plane array. MATLAB consequently raised
`MATLAB:heterogeneousStrucAssignment` instead of continuing refinement.

The retained one-line fix initializes the new plane's full-span fraction to
`[0,1]`, as the alternating phase already does. Its solve covers the full control
span, so this records existing constraint coverage without weakening it.
A structurally different three-span detour around one static box reproduces the
exact exception with the old implementation. With the fix, refinement adds three
planes and reduces the control-polygon travel bound from 12 to 6.099031529653
units at the twelve-second clock. A new exact engine collision certificate passes.

The saved moving request still returns the exact reference polynomial and
complete timed-search record, with arrival 117 seconds, unchanged motion length
and solve counts, and passing public independent validation. A verification
replay took 4.508219 seconds; this single run is not a speed comparison. The
change fixes a crash rather than claiming a runtime gain. All 38 MATLAB tests
passed and Code Analyzer was clear for the changed files.

## Rejected conic variable normalization

Time powers were normalized by the maximum segment duration and its powers.
Position and travel variables were separately normalized by workspace extent,
with coordinates centered on the workspace. The combined variant applied both.
Positive scaling was carried through linear constraints, endpoint equalities,
bounds, and objective, then undone on the returned controls. Homogeneity of the
time-power and travel cones permits their original unit-coefficient forms in
these normalized variables. Physical limits and solver tolerances were unchanged.

All methods were warmed before three paired repetitions with rotating order.
Every listed candidate passed the engine collision certificate and arrived at
117 seconds. These are full BMTP timings, excluding the public planner's route
search and independent validation.

| Normalized variables | Median (s) | Length (units) | Integrated squared jerk (units squared / s^5) | Trajectory / plane solves |
| --- | ---: | ---: | ---: | ---: |
| None, paired reference | 2.770484 | 229.959020398834 | 0.991397232846 | 11 / 78 |
| Time powers | 2.579679 | 229.978457751056 | 0.994353107475 | 10 / 65 |
| Position and travel | 3.480864 | 230.041245567676 | 0.996804181662 | 12 / 91 |
| Both | 3.626676 | 230.008395785913 | 0.987674941408 | 11 / 78 |

Time normalization reduced median BMTP runtime by 6.9%, but lengthened the path
and increased squared jerk. A second paired comparison isolated its phases:

| Time normalization phase | Median (s) | Length (units) | Integrated squared jerk (units squared / s^5) |
| --- | ---: | ---: | ---: |
| None, paired reference | 2.692239 | 229.959020398834 | 0.991397232846 |
| Alternating only | 2.621945 | 229.984093543966 | 0.994045985115 |
| Final refinement only | 2.666395 | 229.959136058593 | 0.991345896919 |

Neither phase isolated a runtime improvement without a motion-quality tradeoff.
All normalization variants are discarded. Only the independently reproduced
plane-field crash fix is retained from this investigation.

## Regression coverage and code size

The suite now contains 38 MATLAB tests. The saved-request regression checks arrival 117,
independent validation, timed-route selection, and active-pair reduction.
Structurally different regressions retain the nine-second moving-circle detour,
the 82.5-second long request, the validated waiting incumbent, and the exact
220-vertex length of 121.503236303671 units. A crossing-barrier regression covers
both continuously moving and stationary source intervals, preserving the
reference graph's departure and arrival times.

The initial timed-path implementation added 1,749 lines in new production files and 81 net
lines in existing production files, for a net production increase of 1,830
lines. Reusing the existing boundary-only exact graph was tested as a smaller
alternative, but the saved request did not complete within one minute; the
measured 14.9-second route implementation was retained.
