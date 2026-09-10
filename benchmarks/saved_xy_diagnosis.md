# Saved moving-detour diagnosis

The preserved request is `tests/fixtures/savedMovingDetour.json`, copied from
`RogueCasses/x-y-request.json`. It contains a translating rectangle with 21
keyframes, a rotating 38-vertex concave obstacle, a 180-second horizon, and the
original per-axis motion limits and safety margins.

## Final optimization audit

The retained implementation on `build-core` was replayed three times after the
complete MATLAB R2024b test suite: 3.702200, 3.505458, and 3.515452 seconds,
with a 3.515452-second median. These are warmed full public-planner calls,
including search, BMTP, exact recertification, and independent validation.
Earlier fresh-process measurements include startup and JIT effects; this final
median is not a cold-start guarantee.

Every final replay passed independent validation and matched the original timed
implementation's complete polynomial and timed-search record exactly. Arrival
remained 117 seconds, motion length 229.959020398834 units, and integrated
squared jerk 0.991397232846 units²/s⁵. The solver still tagged 13 of 188 applicable
pairs, with 11 trajectory and 78 plane solves. `GlobalEarliestProven` remains
false: the planner selects the earliest reachable finite search layer, without
claiming a continuous-time global optimum.

The initial timed implementation's recorded median was 14.909188 seconds. The
retained optimizations reduce that to about 3.5 seconds without changing its
motion. The cleanup reference's recorded median was 49.511337 seconds; its path
was 0.009074606 units shorter, as documented below. The optimization pass does
not erase that original port-level quality difference.

All 39 tests passed in the final audit, including direct motion, detours,
expected no-path results, invalid inputs, tampering rejection, source/cache
refresh, moving obstacles, and the maintained 220-vertex quality regression.
The final production diff was reviewed and passed `git diff --check`.
Code Analyzer was clear for all 12 retained production MATLAB files changed
during the optimization pass; the thirteenth changed file was deleted.

Git's MATLAB-only production counts show 22 net lines beyond the original
timed implementation (`c5f28e9`). The original port itself added 1,834 net
production lines; the total since its parent is 1,856. These counts exclude
tests, examples, benchmarks, and documentation and correct the earlier
pre-commit count of 1,830 lines. Rejected experiments remain outside source
control. Milestone commits retain only verified improvements, the refinement
field fix, and the measured comparison record.

The experiment set covers geometry preparation and queries, search updates,
constraint construction, solver scaling and reductions, plane batching,
sampling, mesh size, polynomial degree, and cache alternatives. The current
implementation is the retained choice among these measured candidates; this is
not a proof that no future algorithm could be faster. Worker-pool concurrency
was not evaluated: although the MATLAB license test returned true, `parpool`
was unavailable on the active installation's path. No worker infrastructure or
additional runtime dependency was added.

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

## Rejected reachability-update rewrites

Three search implementations tested whether repeated state-update calls were
worth removing. The batched version passed all clear transitions for one target
layer to a helper that processed them sequentially. The shared-state version
made the helper nested, updating its parent's arrays without array arguments or
return values. The inline version placed the wait and motion updates directly
in the search loop and removed the helper. All retained the exact update order,
floating-point comparison expressions, and final-layer parent tie policy.

Each timing comparison warmed its methods before three repetitions with rotated
execution order. These are search-only timings on identical saved-case inputs:

| Implementation | Paired reference median (s) | Variant median (s) |
| --- | ---: | ---: |
| Batched sequential updates | 0.525199 | 0.531149 |
| Nested helper with shared arrays | 0.525199 | 0.535001 |
| Inline wait and motion updates | 0.551300 | 0.547199 |

An earlier paired batch comparison also favored the reference, 0.560137 versus
0.569044 seconds. The inline version's four-millisecond median difference is
too small to establish a useful gain. None warrants a new full-planner timing
claim or additional production complexity.

All saved-case routes, clocks, and complete search records matched exactly.
Each variant also matched sixteen mixed-obstacle searches spanning both arrival
modes, both obstacle orders, stationary obstacles with different lifetimes, and
an almost-stationary history. The maintained implementation is retained without
source or test changes.

## Consolidated separating-plane verifier milestone

The static-region timed verifier duplicated the general verifier's trajectory
Bernstein product, offset correction, normal bound, clearance calculation, and
roundoff policy. Those checks now live in one function. Static vertices make the
obstacle-side polynomial linear, so the shared verifier uses its two endpoint
coefficients, exactly as the removed timed verifier did. Affine moving vertices
still use all three quadratic Bernstein coefficients. The independent public
validator continues to recompute its required checks.

The sole production caller of `verifyTimedSeparatingLine` now calls the shared
`verifySeparatingLine`; the duplicate file is removed. The net production diff
removes fifty lines. No solver objective, constraints, tolerances, margins,
certificate coverage, or route-selection policy changes.

After warming both complete BMTP implementations, three paired repetitions with
alternating order took 2.775630, 2.635827, and 2.637951 seconds before, versus
2.735038, 2.681832, and 2.645259 seconds after. Medians were 2.637951 and 2.681832
seconds; this does not demonstrate a useful solver speed improvement. Every
polynomial and complete collision certificate matched exactly.

Full planner runs took 5.972431, 3.903859, and 3.667459 seconds, with median
3.903859 seconds. The small difference from the prior three-run median of
3.961056 seconds is not claimed as a reliable speed gain. All three full runs
returned the exact reference polynomial and timed-search record, arrival 117
seconds, unchanged motion length and solve counts, and passing independent
validation. The change is retained for its smaller implementation.

All 39 MATLAB tests passed and Code Analyzer was clear for the changed files.
The new regression checks that a static region and its identical two-endpoint
affine representation produce the same plane certificate, then both reject a
violating control point. The existing moving-obstacle interior-violation case
still rejects clear endpoints with an intervening collision, and the maintained
220-vertex path-length regression remains unchanged.

## Rejected plane-result caching and block solves

An instrumented replay captured the complete inputs to all 78 timed
separating-plane solves, including control points, vertices, target, reserve,
and solver options. No two input lists were exactly equal. Their recorded
solver time totaled 0.274087 seconds in that replay, with zero attributable
to exact repeats. An exact-result cache would therefore add overhead without
eliminating a solve in this request; none was implemented.

The captured independent problems were then replayed individually and in
block-diagonal conic programs. Each block retained the original seven variables,
maximum-margin objective, inequalities, and two unit-normal cones per problem.
The resulting individual planes were verified with the unchanged shared
verifier. Common solver options were checked outside the timed loop. All
implementations were warmed before three repetitions with rotated execution order.

| Plane problems per conic call | Median time for all 78 problems (s) |
| --- | ---: |
| 1, retained | 0.228343 |
| 2 | 0.256500 |
| 4 | 0.280584 |
| 13 | 0.411757 |

These times include assembly, solving, and plane verification, but exclude
trajectory optimization and public-planner work. Every conic call returned a
positive exit flag. Each method produced 77 verified intermediate planes, the
same count as the individual reference; these initialization-plane counts are
not a final motion certificate. The original instrumented BMTP replay returned
a passing final engine certificate.

Even this offline replay, which can group known problems without waiting for
the intervening trajectory iterations, showed no batching benefit. The larger
programs cost more than the saved call overhead. No block solver was integrated
into the planner, and production remains unchanged.

## Shape-only query milestone and rejected union deduplication

The proposal builder adds a sampled polygon for every obstacle and time layer.
An experiment skipped later copies when an obstacle's prepared source samples
were exactly identical. Although union is mathematically idempotent, changing
its operand list changed floating-point polygon construction: the symmetric
difference area was 1.7651206156660431e-10 square units, and the retained route
nodes changed. The arrival layers remained 0, 13.5, 22.5, and 117 seconds, but
BMTP failed with `A tagged pair crossed its retained separating plane.`
This variant is rejected; the operand sequence remains unchanged.

The retained alternative adds one production line to `preparedShapeAtTime`.
Once an active shape is constructed, a caller requesting only that first output
returns before the unused geometry-metadata construction. Two-output queries
retain their exact classification, edge records, and other metadata. No union
operand, geometry, node, route, time layer, or validation predicate changes.

After warming both implementations, three paired complete-proposal runs with
alternating order took 0.855302, 0.763717, and 0.752392 seconds before, versus
0.754094, 0.746965, and 0.736317 seconds after. Medians were 0.763717 and 0.746965
seconds, a 2.2% improvement in this stage. Every route, clock, and complete
proposal record matched exactly, including all offset attempts.

Full planner runs took 6.031234, 3.865943, and 3.710338 seconds, with median
3.865943 seconds versus the preceding 3.903859 seconds. This small
separate-session difference is not treated as a reliable end-to-end gain.
All three preserved the exact reference polynomial and timed-search record,
117-second arrival, motion length, solve counts, and passing independent
validation. The change is retained as a one-line removal of unused work.

All 39 MATLAB tests passed and Code Analyzer was clear. The existing prepared
geometry regression now also compares one-output and two-output shapes for
convex, concave, and holed obstacles at source times, between samples, and outside
their active interval. Production grows by one line; the test adds one setup line
and one assertion without introducing another helper or test case.

## Batched geometry-cache lookup milestone

A fresh warmed profile still attributed 2.508367 seconds to 90 conic solves
and 0.564784 seconds to 1,356 prepared occupancy queries. These are inclusive
times from a 4.638940-second profiled planner call, not independent stage totals.
The query loop fetched 5,871 cached boundaries one key at a time.

The retained change batches `isKey` and `values` once per moving obstacle and
query batch. Missing boundaries are still evaluated and inserted in the original
time order, under the same capacity limit. Every queried point still undergoes
the original occupancy and boundary-policy check; blocking-obstacle order,
geometry, route candidates, and validation remain unchanged. Production gains
six net lines and no helper.

Two independently warmed, alternating-order search comparisons returned exactly
the same route, clock, and full search record:

| Comparison | Before median (s) | After median (s) |
| --- | ---: | ---: |
| First three pairs | 0.634106 | 0.515790 |
| Second three pairs | 0.555236 | 0.476350 |

The second comparison improves this stage by 14.2%. Sixteen additional search
comparisons matched exactly across static-obstacle lifetimes, reversed obstacle
order, a nearly stationary source, and both arrival modes.

A preliminary full-planner comparison was superseded after fixing the copied
benchmark wrapper's checkout-path initialization. With both implementations
warmed and using the same production path, five alternating-order pairs took:

| Repetition | Before (s) | After (s) |
| --- | ---: | ---: |
| 1 | 3.809694 | 3.521000 |
| 2 | 3.618948 | 3.713644 |
| 3 | 3.569256 | 3.573788 |
| 4 | 3.543350 | 3.525495 |
| 5 | 3.460538 | 3.456040 |
| Median | 3.569256 | 3.525495 |

The observed end-to-end median improvement is 1.2%; individual runs overlap and
two pairs are slower. The clearer benefit is reduced search work, with a modest
overall effect because conic solving still dominates. All ten measured results
passed independent validation and preserved the exact reference polynomial and
complete timed-search record: arrival 117 seconds, length 229.959020398834 units,
13 tagged pairs, 188 applicable pairs, 11 trajectory solves, and 78 plane solves.

Regression coverage now exercises mixed cache hits and misses after capacity is
reached, repeated query times, cold and warm caches, boundary policies, and the
first blocking obstacle. These extend two existing tests without adding a helper.
All 39 MATLAB tests passed, and Code Analyzer was clear for both changed MATLAB
files. A subsequent public-planner run of the retained production code took
3.937479 seconds after the suite, passed independent validation, and again
matched the reference polynomial and complete timed-search record exactly.

## Rejected motion-mesh and polynomial-degree reductions

The next experiment targeted conic problem size rather than query overhead.
All variants used the same saved scene, selected route, 117-second goal clock,
limits, tolerances, and solver settings. The first sweep changed both the number
of conservative timed obstacle cells and the number of degree-eight motion spans.

| Motion spans / cell intervals | Engine outcome | Length (units) | Squared jerk (units²/s⁵) | Trajectory / plane solves |
| --- | --- | ---: | ---: | ---: |
| 16 / 16, retained | Passing engine certificate | 229.959020398834 | 0.991397232846 | 11 / 78 |
| 8 / 8 | Tagged pair crossed its retained plane | — | — | — |
| 12 / 12 | Trajectory SOCP reported infeasible | — | — | — |
| 20 / 20 | Passing engine certificate | 229.823416407958 | 1.440861792972 | 17 / 259 |
| 24 / 24 | Trajectory SOCP reported numerical instability | — | — | — |

To distinguish reduced motion freedom from coarser obstacle geometry, a second
sweep retained all original 16 obstacle-cell intervals and their polygons while
changing only the motion spans. Eight and fourteen spans crossed a retained
plane; twelve spans again produced an infeasible trajectory SOCP. The 16-span
reference reproduced its original motion and metrics. These failures describe
the tested optimizer, not physical infeasibility of the scene.

A third sweep retained the original 16 spans and 16 obstacle-cell intervals and
changed only polynomial degree, including its derivative and continuity rows:

| Degree | Engine outcome | Length (units) | Squared jerk (units²/s⁵) | Trajectory / plane solves |
| --- | --- | ---: | ---: | ---: |
| 8, retained | Passing engine certificate | 229.959020398834 | 0.991397232846 | 11 / 78 |
| 5 | Certified minimum exceeded the fixed arrival | — | — | — |
| 6 | Tagged pair crossed its retained plane | — | — | — |
| 7 | Tagged pair crossed its retained plane | — | — | — |
| 9 | Passing engine certificate | 230.166093498940 | 1.211121284995 | 12 / 104 |

The larger successful representations preserve arrival but increase squared
jerk by about 45% (20 spans) and 22% (degree nine). Degree nine also lengthens the
motion. Neither meets the no-quality-regression gate, so neither was promoted
to a full public-planner candidate. The retained representation passed its
engine certificate in each sweep and reproduced the reference motion metrics.

These were one-pass screening experiments, with the reference first in each
fresh MATLAB session; their wall times are not warmed runtime comparisons.
The 20-span engine call took 5.661325 seconds and degree nine took 3.813960
seconds, but quality and feasibility already reject these changes. Failed calls
are not counted as speedups. No mesh-selection heuristic, retry schedule,
production change, or additional test was retained. The previous paired public
planner median of 3.525495 seconds remains the latest retained measurement.

## Rejected fixed-time cone removal and sampling allocation reduction

Two further experiments kept the original 16-span, degree-eight representation,
the same route and arrival, and all validation predicates. Each implementation
was warmed before three paired BMTP runs with alternating execution order.

The first removed the two time-power cones only during fixed-arrival travel
refinement. Their variables are already fixed to the selected time powers by
equal lower and upper bounds. This one-line candidate did not improve runtime:

| Repetition | Original BMTP (s) | Without fixed-time cones (s) |
| --- | ---: | ---: |
| 1 | 2.703082 | 2.741371 |
| 2 | 2.672066 | 2.695066 |
| 3 | 2.686239 | 2.647646 |
| Median | 2.686239 | 2.695066 |

All candidate engine certificates passed, but the numerical solution changed.
Length increased from 229.959020398834 to 229.959347971727 units while squared
jerk decreased from 0.991397232846 to 0.991323543230 units²/s⁵. Arrival remained
117 seconds with 11 trajectory and 78 plane solves. The absent speed benefit
and increased path length reject this variant.

The second delayed expansion of the de Casteljau work array until its first
arithmetic operation. Constant curves retained explicit replication, and all
recurrence expressions and sample locations were unchanged. For 1,201 samples
of a planar degree-eight curve, the initial replicated array contains 172,944
bytes of doubles; the candidate initially stores only the 144-byte control
array. These are initial array sizes, not measured peak process memory.

| Repetition | Original BMTP (s) | Delayed sampling expansion (s) |
| --- | ---: | ---: |
| 1 | 2.698141 | 2.800366 |
| 2 | 2.711762 | 2.745254 |
| 3 | 2.738897 | 2.659652 |
| Median | 2.711762 | 2.745254 |

Every measured motion matched the reference polynomial exactly and passed its
engine certificate. The saved results also matched the complete certificate,
trial-duration and collision histories, and solve counts exactly.
The smaller initial allocation did not translate into a
runtime improvement, so its extra production line is also rejected. Neither
experiment was promoted to the public planner; production and the previously
passing 39-test suite remain unchanged.

## Rejected trajectory-assembly cache

A bounded single-entry cache retained the derivative and continuity matrices,
endpoint right-hand sides, and initial variable bounds between timed trajectory
steps. Its exact key included span count, degree, endpoints, limits, arrival
mode, and active-plane count. Separating-plane rows and physical time bounds
were rebuilt on every call. Requests above 4,096 variables or 65,536 inequality
rows bypassed cache insertion.

Captured complete solver inputs matched in 288 configurations: three degrees,
four span counts, both arrival modes, three active-plane patterns, and four
versions of each case including changed plane data, reserves, and horizons.
One hundred repeated assembly calls had warmed medians of 0.073477 seconds
before and 0.036438 seconds after. Three paired full BMTP runs also returned
identical polynomials and engine certificates, with medians 2.681556 and
2.666405 seconds respectively.

That small engine difference did not survive five warmed public-planner pairs:

| Repetition | Original planner (s) | Assembly cache (s) |
| --- | ---: | ---: |
| 1 | 3.777920 | 3.498001 |
| 2 | 3.510893 | 3.583043 |
| 3 | 3.485416 | 3.567950 |
| 4 | 3.472428 | 3.524494 |
| 5 | 3.416256 | 3.456057 |
| Median | 3.485416 | 3.524494 |

All ten public results passed independent validation and preserved the exact
reference polynomial and complete timed-search record. Four of five cached
runs were slower. The cache is rejected because its added state and ten net
production lines do not produce an end-to-end benefit; the simpler current
assembly remains in production.

## Rejected deferred query-time sorting

Another candidate preserved a scalar query time before broadcasting it across
points, and deferred `unique` for array-valued times until a moving obstacle
actually needed time groups. Exactly static obstacles could skip that sorting.
Geometry lookups, point tests, and first-blocker ordering were unchanged.

After warming both implementations, three alternating-order search pairs took
0.551817, 0.466194, and 0.452387 seconds for the current implementation versus
0.496009, 0.478157, and 0.457398 seconds for the candidate. The medians were
0.466194 and 0.478157 seconds. Complete route, clock, and search records matched,
as did sixteen mixed-obstacle comparisons covering both arrival modes and
reversed obstacle order. Since the saved-case search did not improve, this
variant was not promoted to a full planner comparison or retained in production.

## Regression coverage and code size

The suite now contains 39 MATLAB tests. The saved-request regression checks arrival 117,
independent validation, timed-route selection, and active-pair reduction.
Structurally different regressions retain the nine-second moving-circle detour,
the 82.5-second long request, the validated waiting incumbent, and the exact
220-vertex length of 121.503236303671 units. A crossing-barrier regression covers
both continuously moving and stationary source intervals, preserving the
reference graph's departure and arrival times.

The initial timed-path commit added 1,751 lines in new production files and 83 net
lines in existing production files, for a net production increase of 1,834
lines. Reusing the existing boundary-only exact graph was tested as a smaller
alternative, but the saved request did not complete within one minute; the
measured 14.9-second route implementation was retained.
