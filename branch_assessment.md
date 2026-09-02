# Novel replacement branch assessment

## Encode homology states numerically and reuse cleanup trees - 2026-09-01

The largest new strength is bounded numeric ownership of spatial-homology
states. Node and ternary winding digits now form an overflow-checked base-3
`uint64` key instead of allocating formatted strings. Unsupported key capacity
throws an identified error before search. Alternative cleanup builds one
shortest-path tree from the start and one from the goal per route class instead
of solving two shortest paths for every waypoint. Diagnostics expose the key
encoding, retained linear frontier, and exact cleanup-tree count.

On a deterministic 50-node cleanup graph repeated 50 times, total profiled
homology time fell from 1.814420 to 1.481489 seconds (18.349 percent). Key work
fell from 0.051524 to 0.005690 seconds across 4,900 states (88.956 percent), and
cleanup alternatives fell from 0.392447 to 0.137429 seconds (64.981 percent).
A binary-heap candidate preserved states and expansions but took 1.770479
seconds, 19.505 percent slower than the numeric-key linear scan, and was
removed. A maintained static median increased 3.898 percent from 3.2084006 to
3.3334726 seconds with exact physical output, so no end-to-end speedup is
claimed.

The largest current weakness is that the frontier remains a linear scan. That
is intentional evidence-based retention at current bounded state counts, not a
claim that a heap can never win at a larger explicit work bound. Item 18 must
make those state/transition limits explicit before frontier scaling is revisited.

## Reject explicit move-then-wait time expansion - 2026-09-01

No production change was retained for item 13. The candidate prioritized
source-event layers before midpoint/uniform fill, widened temporal parent and
transition indices from `uint16` to `uint32`, vectorized per-source duration and
target-layer calculations, preallocated explored nodes, and represented a move
at the existing minimum transition duration followed by an explicit destination
wait. Focused event-priority, event-overflow, and arrival-knot tests passed, as
did timed BMTP 3/3, planner contract 14/14, route economy 3/3, and the existing
6/7 adversarial state.

The implementation failed its bounded runtime gate. The first exact version
took about 180 seconds on `testTimedBmtpPlanning`; 154.334851 seconds were in
occupancy, with 374,872 `shapeAtTime` calls caused by per-edge arrival times. A
general exact batch path for static, rigid-translation, and conservative
enclosure histories reduced the suite to 30.1980109 seconds, but the immediate
item-12 baseline was 20.4758520 seconds. Explicit arrival/wait knots also raised
conic-wrapper solve calls from 269 to 680. The final candidate was therefore
47.482 percent slower and was removed completely rather than reducing collision
samples or hiding solver work.

The largest current strength is a measured negative boundary around a tempting
temporal-model cleanup. The largest remaining weakness is that the retained
time-expanded search still stretches a move over its layer gap and uses
`uint16` parents. Correcting that representation needs a formulation that does
not more than double timed-solver work; item 18 will still replace the nominal
layer allowance with an explicit state/transition work bound independently.

## Prune occupancy queries with exact-time boxes - 2026-09-01

The largest new strength is output-aware occupancy pruning. One- and two-output
calls discard points outside the exact-time protected box and stop processing a
point after its first blocker. Three-output diagnostic calls retain every
obstacle that could improve signed clearance and skip one only when positive
box distance proves it cannot beat the current exact value. Exact-time boxes
come from prepared boundary records; occupancy no longer constructs a convex
request-horizon `polyshape` merely to obtain a loose box.

The immediate item-11 seed-1011 profile retained identical success, validation,
17.9853686689-second arrival, 259 collision intervals, 57 occupancy calls, 167
route states, 268 expansions, and 4,852 `coneprog` calls. `polyshape`
construction fell from 711 to 423 (40.506 percent), while inexpensive
`shapeAtTime` boundary-record calls rose from 5,639 to 15,488. A static median
increased 4.157 percent from 3.0803439 to 3.2084006 seconds with exact physical
output, so no end-to-end speedup is claimed.

The largest current weakness is that MATLAB function-call volume rises sharply
even though expensive shape construction falls. This is a deliberate
geometry-cost exchange that passed the exact-call gate, not proof that every
occupancy workload is faster. A later refactor may centralize the repeated
prepared-boundary query, but must preserve the distinct first-blocker and
minimum-clearance rules.

## Certify easy continuous bounds in Bernstein form - 2026-09-01

The largest new strength is a three-outcome continuous range check. A complete
Bernstein hull inside the limit certifies an interval immediately; recursive
de Casteljau subdivision can prove smaller intervals inside or wholly outside;
and anything still ambiguous retains the independent endpoint/stationary-point
fallback. One coefficient outside the limit is never treated as a rejection.
The degree-only power-to-Bernstein transform is cached as immutable arithmetic,
so repeated validations do not rebuild it.

On the maintained static example, the exact item-10 profile made 600 range
checks and 537 `roots` calls, spending 0.391988 seconds in validation. Item 11
made the same 600 checks, made zero `roots` calls, and spent 0.226719 seconds in
validation. The warm-plus-three example median decreased from 3.3586421 to
3.0803439 seconds (8.287 percent) with exact motion and arrival. The exact root
reduction is the retention evidence; the sub-ten-percent end-to-end observation
is not generalized into a speedup claim.

The largest current weakness is that recursive Bernstein range checks still
share the already large `validateTrajectory.m` local-helper surface. The
stationary fallback remains necessary for high-degree or numerically ambiguous
inputs, and no global claim is made about the root-call mix on unseen
polynomials.

## Bound continuous collision work per interval - 2026-09-01

The largest new strength is a fail-closed adaptive certificate whose proof
bounds now belong to the interval being certified. Each polynomial segment is
converted to velocity Bernstein controls once; de Casteljau restriction gives
a conservative vector-speed envelope for every adaptive subinterval. Prepared
obstacle samples and verified linear interpolation provide the matching swept
box and obstacle-speed bound without constructing a `polyshape`. An unresolved
exit now preserves its last interval, limiting obstacle, path and obstacle speed
bounds, required and observed clearance, and configured minimum timestep.

On an adjacent warm-plus-three seed-1011 comparison, retained adaptive
intervals fell from 331 to 259 (21.752 percent) with the same successful,
independently validated 17.9853686689-second arrival and zero unresolved
intervals. Median wall time changed from 86.0659709 to 86.2290776 seconds, a
0.190 percent increase, so no runtime speedup is claimed. A fresh adjacent
static comparison changed from 3.5125306 to 3.3586421 seconds with exact
physical output, but that isolated observation is likewise not generalized.

The largest remaining weakness is temporal proposal quality: the adversarial
disjoint-window case still skips the first feasible intercept and selects
6.5829833374 seconds. The stronger certificate can validate earlier proposals
more tightly, but it cannot create a missing chronological proposal. That is
item 15, not evidence against the certificate.

## Reject direct cached-edge clearance - 2026-09-01

No production change was retained for item 9. A direct even-odd boundary
clearance query was proved equivalent on holes, disconnected regions, NaN-ring
separators, reversed orientation, exact sample times, verified interpolation,
and conservative fallback geometry. The candidate passed its 5/5 focused
equivalence tests and the existing obstacle, stage-timing, planner-contract,
timed-BMTP, and adversarial suites without changing their physical results.

The implementation nevertheless failed its bounded retention gate. On the
moving seed-1011 stress request, strict warm-plus-three medians were
81.4626468 seconds for item 8 and 81.3327973 seconds for the candidate, only a
0.159 percent reduction. Only 144 of 4,953 clearance queries avoided
`polyshape`; 4,809 still required the fallback representation. On the static
maintained example, the median regressed 8.008 percent from 2.9158384 to
3.1493434 seconds. The candidate also added 188 net production lines. It was
removed completely rather than trading clarity and static runtime for a
neutral moving-case result.

The largest current strength remains exact preparation ownership and reuse of
request-invariant geometry. The largest current weakness is that cached edges
do not by themselves eliminate most topology-sensitive geometry conversions;
future collision work must reduce a materially larger exact call population or
demonstrate at least a ten-percent median improvement without regression.

## Own and invalidate prepared obstacle geometry - 2026-09-01

The largest new strength is a request-owned preparation record whose cached
sample shapes, bounds, edges, fallback geometry, interpolation deltas, and
speed bounds are tied to an exact immutable source snapshot. Mutating any
canonical public field rebuilds the complete collection. A targeted regression
proves shape, occupancy, and projection queries cannot reuse the stale cache,
and preserving valid preparation reduced seed-1011 `polyshape` construction
from 855 to 741 without changing physical, search, collision, or solver work.

The largest current weakness is that cached edges and per-ring bounds are not
yet consumed by point-clearance hot loops; item 8 establishes ownership and
freshness, while item 9 must prove direct cached geometry equivalent before it
can remove the remaining repeated conversions. The profiled wall change was
only a 0.588 percent decrease, so no end-to-end speedup is claimed.

## Reuse invariant projection and decomposition work - 2026-09-01

The largest new strength is exact request ownership for static BMTP geometry.
An applicable request now creates one conservative static projection and one
source-derived convex representation rather than rebuilding both for every
seed. Independent public validation remains authoritative. Permanent static
and moving tests require the construction counts to equal one, and the fixed
seed-1011 profile reduced `polyshape` construction 37.908 percent, from 1,377
to 855, without changing arrival, route-search counts, collision resolution,
or the 4,852 `coneprog` calls. Fixed-clock boundary searches also expose and
obey a 0.001-degree physical target and twelve-iteration cap while keeping the
passing side of every independently validated bracket. The standalone BMTP
adapter retains its established canonical-obstacle input form through a
single-representation forwarding path.

The largest measured weakness remains trajectory optimization work. Profiled
wall time improved only 3.205 percent, from 143.0640531 to 138.4790277 seconds,
which does not support an end-to-end speedup claim. The post-correctness stress
request is still roughly five times slower and makes more than twice the
`coneprog` calls of the item-2 baseline. Reusing source-derived static geometry
does not yet cache dynamic sample shapes, edges, or interval bounds; those are
separate item-8 and item-9 responsibilities.

## Restrict proposal and broad-phase geometry to request time - 2026-09-01

The largest new strength is one horizon-selection invariant shared by
projection, dense-envelope, seed-corridor, occupancy, and validator paths.
The adversarial projection no longer grows from `[-1, 2]` to `[-101, 101]`
because of remote stored samples. Forty-five focused contract and compatibility
tests passed, and a maintained moving-obstacle example passed planning,
independent validation, collision, kinematic, and certificate checks.

The largest remaining limitation in the correctness block is temporal search:
the narrow first intercept window is still skipped in favor of the later
6.668424011230469-second window. Horizon geometry is recomputed at several
call sites in this item; prepared request-owned caching is deliberately left
for items 7-12, so no runtime improvement is claimed here.

## Make unsupported obstacle interpolation conservative - 2026-09-01

The largest new strength is that boundary array shape no longer establishes
vertex correspondence. Equivalent ring-order changes are canonicalized without
fabricated motion, and a topology mismatch now occupies the full convex
endpoint sweep instead of leaving the between-sample gap clear. The targeted
regressions and 16 related infrastructure, projection, timed-motion, and
fallback-policy tests passed. A maintained moving-circle example also passed
public planning and independent continuous validation.

The largest current limitation is proposal completeness for unsupported
histories: a convex endpoint enclosure may fill holes and bridge disconnected
regions. That is a deliberate fail-safe loss of free space, not an exact swept
shape or a completeness claim. Request-horizon projection still includes
irrelevant stored samples, and the narrow first intercept window is still
missed; their failing tests remain visible for later numbered items. No runtime
benefit is claimed from this correctness change.

## Add reproducible obstacle-work counters - 2026-09-01

The largest new strength is a decision-neutral profiler benchmark that turns
the moving-polygon stress request into stable work counts. On deterministic
seed 1011 it repeatedly observed 2,272 `coneprog` calls, 7,969 `shapeAtTime`
calls, 8,087 `polyshape` constructions, 55 occupancy queries, four full
independent validations, 105 homology states, and 284 expanded states. The
public validation record now also exposes adaptive collision intervals; the
latest profiled result retained 209 intervals and zero unresolved intervals.
All focused static and contract tests passed, and planner decisions and the
independent acceptance criteria were unchanged.

The largest measured weakness is still repeated conic and geometry work:
`coneprog` consumed 16.210907148 seconds of the 27.6499923-second warmed
profiled baseline, while occupancy queries consumed 3.815109682 seconds.
Profiler wall time later varied to 42.7303887 seconds with the same call
counts, so no runtime improvement or regression is claimed from this
diagnostic-only group. The counters identify work volume; they do not prove
which later reduction will preserve behavior.

## Reconfirm moving-obstacle robustness - 2026-09-01

No planner change was retained. The historical random-moving-polygon seed
1011 failure no longer reproduced at the pushed `81a94be` baseline: its
2.40165847462-degree analytic boundary witness accompanied an independently
validated `goalReached` result at 17.9833348954 seconds arrival. The focused
run took 26.526273 seconds.

The unchanged deterministic `1001:1012` corpus then passed 12/12 through the
public planner and independent validator. Per-case wall time ranged from
21.0016595 to 82.4723368 seconds, with a 34.32322055-second median and
509.2953517 seconds total. This proves the historical clear-witness failure is
absent on this fixed corpus; it does not prove completeness on unseen moving
obstacles. The largest unfavorable result is seed 1007's 82.4723368-second
wall time. Because no correctness failure remained, changing planner behavior
would have been speculative and was rejected before implementation.

## Retain coneprog after conic-backend comparison - 2026-09-01

This branch starts from the exact pushed `bmtp-cleanup-codex` commit
`44851b6e8fa438b607467e4fc497286c205f5878`. MATLAB R2024b Update 4 ran on
an AMD64 Family 23 Model 113 host with 12 logical processors. Optimization
Toolbox and `coneprog` were installed; MOSEK, Clarabel, ECOS, and SCS were not
installed on the MATLAB path at baseline.

Both repository-owned BMTP cone-program call sites now pass unchanged problem
records through one `+conicSolver` interface. The retained interface keeps
`coneprog` and its `auto` linear solver as the default, validates dimensions,
resolves partial options, warns once for unknown fields, and returns stable
success-or-failure evidence. The fixed benchmark separates a 96-variable,
48-cone trajectory fixture from a 3-variable, one-cone plane fixture and keeps
unavailable backends visible with their license and source repository.

Among `coneprog` configurations, `normal` had the lowest isolated medians in
the first 3-warmup/11-repeat run, but the focused public example did not finish
within two minutes. It was stopped and reverted. The final repeated comparison
placed `auto` at 0.0123925 seconds for the trajectory fixture and 0.0013927
seconds for the plane fixture. Routing through the retained wrapper changed the
warmed `exampleObstacleAvoidance` median from 2.39756075 to 2.4162405 seconds,
a 0.779 percent increase consistent with measurement noise. The physical
result remained exactly 11.152119519 degrees selected, 11.4118613877 degrees
smoothed, and 7.57454176632 seconds, with collision, kinematic, certificate,
and independent validation all passing. No speedup is claimed.

Two public license-free repositories were compiled outside the repository with
Microsoft Visual C++ 2022. MIT-licensed `bodono/scs-matlab` at
`d7720fcea2fba4a10a671602263eab36228bd8d9` passed both isolated fixtures with
maximum residuals below 2e-13 and medians of 0.0019295 and 0.0003284 seconds,
but the same validated public example took 55.1105597 seconds, about 23 times
the warmed incumbent median. GPL-3.0 `embotech/ecos-matlab` at
`2acb7f472f0021a3d226da187cd2941341199893` solved the plane fixture in a
0.000218-second median with a 2.0183e-9 maximum residual, but its production
trajectory calls did not all solve and the example ended `noValidatedSeed`.
Clarabel's official core is Apache-2.0 but has no official MATLAB interface;
MOSEK requires its proprietary dependency and license, neither of which was
present. All losing candidate adapters were removed, and the temporary builds
were not installed.

Two further BSD-3-Clause candidates were built outside the repository. The
official `qoco-org/qoco-matlab` interface passed all nine upstream MATLAB tests,
but using QOCO only for the plane problems produced a 2.6017821-second focused
median versus the fresh 2.2773948-second `coneprog` median, a 14.2 percent
regression. QOCOGen 0.1.9 produced a compiled degree-16/four-vertex plane
solver with the same validated physical output and a 2.2673445-second focused
median, only 0.44 percent faster. Its fixed sparsity and dimensions did not
cover other Bezier degrees or obstacle vertex counts, so it was not a general
backend and missed the 10 percent gate. Both adapters were removed.

A reopened ECOS plane-only adapter reached a 2.0481737-second focused median,
10.1 percent faster than the fresh incumbent, with the focused certificate
passing. The required asymmetric-triangle confirmation was faster as well
(2.3278577 versus 2.7633012 seconds), but only 17 of 18 plane pairs verified;
the minimum signed gap was 0.192109892896 degrees. Public trajectory validation
still passed, which is precisely why the independent plane certificate remains
authoritative. ECOS was rejected and removed rather than hiding that failure.

The follow-up call-reduction plan also retained no production change. Batching
18 independent final plane programs into one call kept validation passed but
was slower in comparable cold runs (8.20 versus 7.97 seconds). Stopping at the
first collision-free biconvex iterate improved the warmed focused median by
17.4 percent (1.8804262 versus 2.2773948 seconds) and retained validation, but
the smoothed path grew 5.1 percent from 11.4118614 to 11.9926831 degrees. Both
candidates were removed under their fixed gates.

The follow-up trajectory audit located the public GPL-3.0 work-in-progress
`iFR-OFC/Clarabel.m` adapter and built it against the official Apache-2.0
Clarabel C++/Rust core. After isolated compatibility repairs, its LP, QP, and
SOCP examples passed. Using Clarabel only for trajectory SOCPs reduced the
focused three-warmup, eleven-run median from 2.3941869 to 1.0663089 seconds
(55.5 percent), with all focused validation and 18 of 18 plane certificates
passing. It failed the structural quality gate: static U still certified all
504 pairs, but arrival grew from 20.7814508253 to 36.2565015075 seconds and
sampled motion length from 39.4001427062 to 48.2844877484 degrees. The adapter
was removed without tuning or a hybrid fallback.

QOCO and ECOS also failed trajectory-only correctness gates, while exact
fixed-time LP bisection returned a valid but lower-quality motion. An exact
constant-plane geometric fast path removed every final plane SOCP in the
focused case, but its weaker linearizations raised the successful-seed
trajectory-SOCP count from three to five and slowed the cold smoke. These
results reinforce that strong plane linearizations and trajectory solution
selection matter more than raw solver-call count. The retained production
files remain byte-for-byte equal to commit `e9af134` for both BMTP and the
conic wrapper.

Code Analyzer reported no issues in changed MATLAB files, focused conic tests
passed 6/6, and the complete tree passed 114/114. All 17 maintained examples
ran serially after the retained wrapper: sixteen independently validated
successes and the expected validated `noValidatedSeed` result. A visible
success produced two figures and five axes. The failure case produced one
search-diagnostic figure whose annotation included `noValidatedSeed`.

The largest strength is now an explicit, tested solver boundary plus measured
negative evidence against attractive microbenchmark results. The largest
weakness is that the branch adds a small wrapper cost without a runtime gain;
it is retained only to keep the two BMTP formulations isolated behind one
stable interface and to make future solver evaluations repeatable.

## Remove benchmark-shaped orthogonal planners - 2026-09-01

At the user's direction, the branch removes the complete orthogonal-cavity and
timed-orthogonal-opening family rather than retaining it as a benchmark
shortcut. Six production helpers are deleted: both motion constructors, both
request/cavity certifiers, their private guarded-rectangle predicate, and the
now-unreferenced arrival-certificate portfolio. `planCorridorQuintic` no longer
detects an orthogonal cavity, constructs a cavity-shaped motion, recognizes an
opening event specially, or ranks those special candidates. Dedicated tests
and manual sections are removed, while both maintained U-shaped examples stay.

The retained planner passes both U examples through ordinary mechanisms.
Static U uses visibility-graph seed 3 and `bmtpStaticDegree16`, independently
validates its certificate, and returns a 39.4001427062-degree sampled motion in
20.7814508253 seconds. The prior cavity shortcut returned 40.255028504 degrees
in 20.712447786 seconds. Opening U uses the general `directWait` seed,
independently validates a 10-degree motion, and arrives in 13.6175223541
seconds versus 11.5843333838 seconds for the removed opening shortcut. These
quality and runtime costs are explicit; passing the examples no longer depends
on recognizing their orthogonal geometry.

Code Analyzer found no issue, focused architecture/contract tests passed
42/42, and the complete suite passed 108/108. All 17 maintained examples ran
serially: sixteen independently validated successes and the expected validated
`noValidatedSeed` result. Fifteen examples retained their established physical
metrics; only the two U examples changed as described above. Serial wall time
was 242.4110337 seconds versus 205.3929177 seconds at `0f9c268`, an 18.023
percent increase concentrated in the general U paths. No speedup is claimed.

This milestone removes 1,804 additional non-test MATLAB lines and reduces
`planCorridorQuintic.m` from 1,023 to 875 physical lines. The branch is now
2,317 non-test MATLAB lines smaller than `5c0a6c9`. Its largest strength is
that U-shaped examples remain real general-planner regressions instead of
being owned by shape detectors. Its largest weakness is that the general
paths are slower and do not reproduce the deleted shortcuts' arrival times;
improving that gap must come from a structurally general timed or static motion
algorithm, not a restored U/cavity detector.

## Timed BMTP follows the search-layer budget - 2026-09-01

The completion audit caught one result regression that the maintained suite did
not expose: removing `CollocationSegmentCount` had changed timed BMTP from the
saved Rogue fixtures' 16 segments to an unrelated internal cap of 20. The
`non-ideal` fixture still validated, but sampled travel grew from
228.491135293 to 228.680505208 degrees. A detached milestone bisect localized
the change to `5a8eee0`.

The public collocation option remains removed. Timed BMTP now enforces the
input-driven invariant that its time-cell count cannot exceed the search-layer
budget that authored the seed. With 17 search layers, the planner uses at most
16 timed segments. This restores the saved `non-ideal` result exactly and adds
a focused contract test without exposing conic dimension as a user choice.

The full test tree passed 111/111. All 17 maintained examples retained their
established physical metrics: sixteen independently validated successes and
the expected validated `noValidatedSeed` result. Their serial wall time was
205.3929177 seconds versus 211.8586926 seconds at `df6a85c`; cold-run timing
noise prevents a speed claim. Visible-success and hidden-failure diagnostic
gates passed.

All five supplied Rogue sentinels also succeeded and independently validated.
`sinetraj`, `newheart`, balanced `shrimp`, balanced `non-ideal`, and
`hiddenruckigfallback` retained sampled travel of 146.928879089,
199.268051966, 175.703912280, 228.491135293, and 233.911502487 degrees,
respectively. The largest current strength is therefore a 513-line-smaller
non-test MATLAB core with preserved measured outcomes across maintained and
external regression families. The largest remaining weakness is structural:
the retained general BMTP engine and corridor orchestrator are still large,
solver-dependent functions, while the audited timed-opening, direct-wait,
orthogonal-cavity, fixed-clock, travel-refinement, Ruckig, and timed-cell paths
are all load-bearing on distinct inputs. Further deletion needs a new bounded
hypothesis rather than another broad pruning pass.

## One moving-obstacle spatial projection - 2026-09-01

The sixteenth accepted `bmtp-cleanup-codex` milestone removes the opportunistic
static-only BMTP solve from moving-obstacle planning. Eligible dynamic topology
seeds now use one conservative swept protected-history projection before the
existing true timed-cell BMTP solver. Full-scene public validation remains
authoritative, and no option, fallback, solver, or scenario branch was added.

A static concave U plus distant translating polygon moved from the removed
static-only route to the swept projection with maximum numeric difference zero
across success, validation, termination, selected seed and source, arrival,
lengths, and sampled time, position, velocity, acceleration, and jerk. Its
plane certificate and full-scene validation passed. A structurally different
moving-circle plus static-concave case still fell through the conservative
swept representation and selected true timed-cell BMTP, also with maximum
physical difference zero and a valid certificate.

Code Analyzer found no issue, focused orchestration tests passed 39/39, and the
complete suite passed 110/110. All 17 maintained examples retained their
established physical metrics: sixteen independently validated successes and
the expected validated `noValidatedSeed` result. Both visualization gates
passed. Serial maintained-example wall time grew 3.772 percent, from
204.1579553 to 211.8586926 seconds. The focused swept winner grew from
12.5067717 to 12.7454047 seconds in one cold run; no speedup is claimed.

The milestone removes 62 net production MATLAB lines and reduces
`+obstacleAvoidance/+planner/planCorridorQuintic.m` from 1,085 to 1,023
physical lines. Across sixteen accepted milestones, the branch is 512 non-test
MATLAB lines smaller than `5c0a6c9`. Swept geometry remains conservative and
can reject a motion that true time-dependent geometry permits; timed-cell BMTP
is retained for that general case.

## Uniform BMTP final certification - 2026-09-01

The fifteenth accepted `bmtp-cleanup-codex` milestone removes the separate
retained-parent-plane restriction path from final BMTP certification. Every
applicable final output-span and obstacle-region pair now uses the same
degree-one maximum-margin conic solver. Optimizer plane reuse remains active
inside the outer BMTP solve; it is no longer treated as an alternative final
certificate algorithm.

Saved Obstacle Avoidance and moving fixed-arrival Target Exits results matched
the exact `514185b` baseline with maximum numeric difference zero across
success, validation, termination, selected seed, arrival, lengths, and sampled
time, position, velocity, acceleration, and jerk histories. Their certificates
remained independently valid. Ordinary BMTP certification now reports all 18
and all 12 applicable pairs, respectively, as conic pairs and zero as reused
pairs. Code Analyzer found no issue, focused tests passed 24/24, and the full
test tree passed 110/110.

All 17 maintained examples preserved their established physical results:
sixteen independently validated successes and the expected validated
`noValidatedSeed` failure. The focused warmed Obstacle Avoidance median grew
1.985 percent, from 2.2880017 to 2.3334234 seconds. Serial maintained-example
wall time grew 2.227 percent, from 199.7103879 to 204.1579553 seconds. The
runtime cost is accepted for one final-certificate algorithm; no speedup is
claimed.

The milestone removes 33 net production MATLAB lines and reduces
`trajectory/+bmtpEngine/solve.m` from 1,191 to 1,158 physical lines. Across
fifteen accepted milestones, the branch is 450 non-test MATLAB lines smaller
than `5c0a6c9`. The conic separator remains solver-dependent, and exact results
on the maintained families do not establish identical numerical behavior for
every unseen region geometry.

## General BMTP final-plane solver - 2026-09-01

The fourteenth `bmtp-cleanup-codex` milestone removes the BMTP engine's
cardinal-axis final-certificate shortcut. Every output span/region pair whose
retained optimizer plane cannot be reused now goes through the existing
degree-one maximum-margin conic solver. This deletes one separate certificate
algorithm without adding an option, fallback, helper, or replacement branch.

Saved Obstacle Avoidance and moving fixed-arrival Target Exits results matched
baseline with maximum numeric difference zero across time, position, velocity,
acceleration, jerk, arrival, duration, selected seed, and motion length. Their
plane certificates remained independently valid. Obstacle Avoidance shifted
12 analytic pairs to 12 conic pairs while retaining six parent planes; Target
Exits shifted five analytic plus one conic pair to six conic pairs while also
retaining six. Code Analyzer found no issue, focused tests passed 24/24, and
the complete suite passed 110/110 in 86.9001094 seconds wall time.

All 17 maintained examples preserved their established physical results:
sixteen independently validated successes and the expected validated
`noValidatedSeed` failure. The runtime cost is visible. The focused warmed
Obstacle Avoidance median grew 5.323 percent, from 2.1769730 to 2.2928512
seconds. Serial maintained-example wall time grew 18.010 percent in aggregate,
from the immediately preceding 169.2320276-second record to 199.7103879
seconds. The largest single cold movement was Straight Target, from 24.2076020
to 34.6242062 seconds (43.030 percent); Target Exits grew 20.070 percent and US
Outline Extreme grew 23.701 percent. Those are accepted maintainability costs,
not speed improvements or noise claims.

The milestone removes 40 net production MATLAB lines and reduces
`trajectory/+bmtpEngine/solve.m` from 1,231 to 1,191 physical lines. Across
fourteen milestones, the branch is 417 non-test MATLAB lines smaller than
`5c0a6c9`. The general conic separator remains solver-dependent, and exact
results on the maintained families do not prove identical numerical behavior
for every unseen region geometry.

## Deprecated BMTP facade removal - 2026-09-01

The thirteenth `bmtp-cleanup-codex` milestone deletes the 58-line
`planTrajBmtp` compatibility facade. Maintained planners already called
`bmtpEngine.solve` directly; only restart-migration tests and current appendix
text referenced the facade. The maintained public planner remains
`obstacleAvoidance.planTrajectory`, and the package engine remains directly
tested.

Before deletion, the facade and package engine returned recursively identical
fixed-arrival candidates and diagnostics. After deletion, the direct package
engine plus complete Obstacle Free and Target Exits results matched saved
baselines with maximum numeric difference zero after excluding only runtime.
Code Analyzer found no issues, focused tests passed 18/18, and the complete
suite passed 110/110 in 86.8524662 seconds wall time. The three removed tests
covered only the deleted restart and invalid-arity surface.

All 17 maintained examples retained established metrics: sixteen independently
validated successes and the expected validated `noValidatedSeed`. Both
visualization gates passed. This deliberately breaking direct-caller cleanup
removes one competing public function and 58 production MATLAB lines. The
thirteen-milestone branch is now 377 non-test MATLAB lines smaller than
`5c0a6c9`.

## Travel-refinement trace retirement - 2026-09-01

The twelfth `bmtp-cleanup-codex` milestone removes fifteen private
`TravelRefinement*` diagnostic fields and their assignments. No planner,
example, test, plotter, exporter, or sandbox consumed them. The balanced and
fixed-arrival refinement algorithms remain unchanged: rate portfolios,
collision-driven plane updates, objective comparisons, and accepted control
nets are still executed. One local boolean now owns the only behavior-bearing
accepted-state decision.

Explicit balanced Obstacle Avoidance accepted refinement from a three-rate
portfolio, while fixed Target Exits accepted its one-rate refinement. Both
complete candidate results matched saved baselines recursively with maximum
numeric difference zero after excluding only runtime and the retired trace
fields. Code Analyzer found no issues, focused tests passed 27/27, and the full
suite passed 113/113 in 87.9262869 seconds wall time.

All 17 maintained examples retained their established metrics: sixteen
independently validated successes and the expected validated
`noValidatedSeed`. Both visualization gates passed. The milestone removes 46
net core MATLAB lines and fifteen fields from every BMTP diagnostics record.
The twelve-milestone branch is now 319 non-test MATLAB lines smaller than
`5c0a6c9`.

## Dead planner-option shim retirement - 2026-09-01

The eleventh `bmtp-cleanup-codex` milestone removes special compatibility
handling for ten planner fields that had already stopped affecting behavior.
Direct planner calls now report them through the maintained aggregate
`planTrajectory:UnknownOptions` warning, and examples reject obsolete
planner-only fields at their own boundary instead of forwarding them. The live
example display `Verbose` control remains; only the dead planner field with the
same spelling lost bespoke handling.

Default, live override, example-chain, and display-option records match their
saved baselines exactly. Complete Obstacle Free and Target Exits results also
match recursively with maximum numeric difference zero after excluding only
runtime fields. Code Analyzer found no issues, focused tests passed 14/14, and
the complete test tree passed 113/113 in 88.1141873 seconds wall time. The
smaller test count is the intentional consolidation of twelve legacy-specific
warning and forwarding tests into two behavior-focused tests, not lost
live-option coverage.

All 17 maintained examples retained their prior metrics: 16 independently
validated successes and the expected validated `noValidatedSeed` failure.
Visible-success and failure-figure gates passed. This intentionally breaking
warning-surface cleanup removes 60 net production MATLAB lines. The
eleven-milestone branch is now 273 non-test MATLAB lines smaller than
`5c0a6c9`.

## Detailed plane-reuse trace retirement - 2026-09-01

The tenth `bmtp-cleanup-codex` milestone removes diagnostic-only state around
automatic plane reuse. `PlaneReuseIterationHistory`,
`PlaneReuseControlDifference_deg`, and
`PlaneReuseDurationDifference_s` no longer appear in solver diagnostics. Their
pending control-net and duration snapshots never influenced a solver input,
continuation, convergence decision, retained candidate, or certificate.

The behavior-bearing mechanism remains intact. `PlaneReuseApplied` and
`PlaneReuseCount` still summarize use; the arrival-tolerance condition, stable
tagged-pair requirement, plane-preserving `continue`, collision histories,
retained-best evidence, and convergence diagnostics are unchanged. The focused
contract still requires reuse, convergence, initial collision evidence, and the
exact minimum collision-free retained duration.

Target Exits at `1e-4 deg` clearance and Extreme US Outline both reused once at
iteration 7 and finished at iteration 8. After removing only runtime fields and
the three retired arrays, their complete baseline/candidate results matched
recursively with maximum numeric difference zero. Target Exits retained 24
seconds and 21.9416287312 degrees; Extreme retained 5.81065318159 seconds and
23.3457566443 degrees. Code Analyzer reported zero findings, focused tests
passed 49/49, and the full suite passed 123/123 in 88.5057376 seconds wall time.

All 17 maintained examples retained their prior metrics: 16 independently
validated successes plus the expected validated `noValidatedSeed`. Visible and
failure-figure gates passed. The milestone removes exactly 17 engine lines and
adds no production replacement. The ten-milestone branch is now 213 non-test
MATLAB lines smaller than `5c0a6c9`.

## External BMTP restart retirement - 2026-09-01

The ninth `bmtp-cleanup-codex` milestone removes externally supplied restart
state from the BMTP engine. Neither maintained planner adapter consumed this
state: static and timed planning both called the engine with seven inputs and
two outputs. The core now has one seed-derived initialization path and no
restart validation, alternate initial-best branch, template allocation, or
restart export.

`planTrajBmtp` remains as a one-release compatibility facade. Ordinary
seven-input/two-output use is unchanged. A former eighth input or requested
third output warns once with `planTrajBmtp:DeprecatedRestart`; supplied state
is ignored and the returned restart record is empty. This is an intentional
compatibility loss for direct external callers: the measured test restart cut
one repeated direct solve from 1.0651848 to 0.2434276 seconds. No maintained
planner path received that benefit, and the old warm and cold fixture motions
were exactly identical.

Saved static degree-16, true timed-cell degree-7, and direct cold results
matched recursively with maximum numeric difference zero after removing only
runtime fields. The deprecated direct call also reproduced the old warm
trajectory exactly. Focused engine tests passed 12/12, the broader focused gate
passed 37/37, Code Analyzer reported zero findings, and the full suite passed
123/123 in 88.4909933 seconds wall time. All 17 examples retained their prior
metrics: 16 independently validated successes and the expected validated
`noValidatedSeed`. Visible and failure-figure gates passed.

The engine removes 44 net production lines; the migration facade adds four,
for a net 40-line non-test MATLAB reduction. The nine-milestone branch is now
196 non-test MATLAB lines smaller than `5c0a6c9`. Plane reuse, three-rate travel
refinement, timed cells, conservative grouping, specialized input-driven
constructors, certificates, validation, and failure diagnostics remain because
their maintained result ownership has not been replaced.

## Internal BMTP segmentation ownership - 2026-09-01

The eighth `bmtp-cleanup-codex` milestone removes the last public option used
only to size BMTP's conic construction. `CollocationSegmentCount` is no longer
resolved or echoed. Static warm routes and timed-cell routes retain the former
default effective cap of 20 spans. Legacy input warns once, is ignored, and
cannot retune segmentation. The timed helper also drops its now-unused options
argument.

Saved static-U and true timed-cell BMTP results matched recursively at `1e-9`
after removing only runtime evidence and the retired field. The timed fixture
retained `bmtpTimedCell`, seven optimizer spans, seven timed cells, 35 seconds,
36.6949453597 degrees, full coverage, planes, validation, and certificates.
The existing dense 30-edge engine fixture still resamples to exactly 20 spans.
A legacy value of 2 warned, disappeared, and matched automatic static-U output
recursively.

Focused tests passed 56/56. Sandbox diagnosis took 10.1182416 seconds versus
10.3521119 before the change; route economy took 14.8954734 versus
14.6741929 seconds, ordinary run noise. All 17 maintained examples then ran
serially with exact prior trajectory metrics: 16 validated successes plus the
expected independently checked `noValidatedSeed`. Visible and failure plotting
passed, Code Analyzer reported zero findings, and the full suite passed 123/123
in 88.5229176 seconds wall time.

The required migration shim makes this milestone two net non-test MATLAB lines
larger even though the public interface and two consumer paths are smaller.
Under the established accounting, the eight-milestone branch remains 156 lines
smaller than `5c0a6c9`. Public defaults now contain 14 meaningful fields, and
the audited public surface has no remaining solver-construction-only control.

## Internal trajectory-solver cap ownership - 2026-09-01

The seventh `bmtp-cleanup-codex` milestone removes the obsolete public
`MaximumNlpIterations` field without deleting its active safeguard. BMTP now
owns one fixed trajectory `coneprog` iteration cap of 300, equal to the former
public default. Legacy direct-planner input warns once, is ignored, and is not
returned. The sandbox and examples no longer present private solver tuning as
a request-level choice.

The former unsupported-topology integration fixture was not independent of
this option: `MaximumNlpIterations=1` manufactured a solver failure. With the
real cap it instead returned `goalReached` after about 66 seconds. The revised
fixture uses a physically infeasible eight-second fixed-arrival deadline, so
the default policy genuinely refuses fallback and the explicit policy genuinely
attempts it. The separate Ruckig unit test continues to own the two-segment
limit. Revised policy tests pass 2/2 in 2.9632806 seconds.

The measured tradeoff is unfavorable but bounded in one deliberately low-cap
test: timed BMTP rose from 9.9854775 to 15.0755105 seconds while retaining a
validated smooth result. Sandbox route economy remained effectively unchanged
at 14.6741929 versus 14.7555260 seconds. The combined focused gate passed
40/40; all changed MATLAB files had zero Code Analyzer findings; and the full
suite passed 121/121 in 88.5048981 seconds wall time.

All 17 maintained examples ran serially in fresh MATLAB processes. Sixteen
succeeded with independent validation, collision freedom, and kinematic
compliance; `exampleNoPath` retained its expected independently checked
`noValidatedSeed`. Every trajectory metric matched the preceding committed
milestone. Visible success created two figures without warnings and hidden
failure created one diagnostic figure containing the reason. The manual-data
exporter passed in 5.7923965 seconds. The milestone removes six net non-test
MATLAB lines under the established branch accounting, leaving the branch 158
lines smaller than `5c0a6c9`. The remaining active implementation option is
`CollocationSegmentCount`, which requires its own bounded experiment.

## Automatic plane-reuse ownership - 2026-09-01

The sixth `bmtp-cleanup-codex` milestone keeps BMTP separating-plane reuse and
its diagnostics while removing two public implementation controls:
`EnablePlaneReuse` and `PlaneReuseImprovementTolerance_s`. Reuse is now an
internal continuation invariant: the retained-best duration improvement must
be within `ArrivalTimeTolerance_s`, and the tagged path--region pair set must
be unchanged. Direct legacy fields warn once, are ignored, and cannot disable
or retune the mechanism.

This removes two false user choices and one duplicated tolerance relationship,
but the one-release migration shim costs three net production MATLAB lines.
The six-milestone branch total is therefore 152 production lines smaller than
`5c0a6c9`. The benefit is interface and ownership reduction, not a runtime or
physical-line claim.

Three pre-edit results were saved and compared recursively after removing only
elapsed-time evidence and the two retired option fields. The tight-clearance
Target Exits case remained 24 seconds and 21.9416287311844 degrees with reuse
count 1, 60 plane SOCPs, and 8 trajectory SOCPs. The structurally different
timed alternating-occlusion case remained 20.8695652173913 seconds and
13.571326600194 degrees, with per-seed reuse counts `[0 1 1 0 1]`. Static U,
which did not activate reuse, also remained exact at 20.7124477860115 seconds
and 40.2550285040009 degrees. A legacy `false` plus custom tolerance reproduced
the automatic Target Exits result exactly.

Broad verification passed 120/120 tests in 81.7413358 seconds wall time and
Code Analyzer reported zero findings. All 17 maintained examples ran in fresh
serial processes: 16 independently validated successes plus the expected
validated `noValidatedSeed`. Visible success and hidden failure plotting both
passed. The manual-data exporter ran successfully and now records plane reuse
as `automatic`.

The remaining public-surface candidates are active solver controls, not dead
fields. `MaximumNlpIterations` owns the `coneprog` iteration cap despite its
obsolete name, while `CollocationSegmentCount` bounds static/timed BMTP route
segmentation. Evaluate each independently; do not remove or retune either
without exact-result evidence.

## Dead planner verbosity option removal - 2026-09-01

The fifth `bmtp-cleanup-codex` milestone removes the public planner `Verbose`
field. A complete read audit found that the field was resolved, validated,
echoed, and forwarded but never read by planning, search, motion generation,
validation, or plotting. The live obstacle-construction verbosity controls
remain separate. The sandbox also retains its top-level verbosity checkbox,
which now owns console capture outside the planner instead of injecting dead
planner state. Direct legacy planner input warns once, is ignored, and is not
returned.

This is an interface reduction rather than a physical-line reduction. The
one-release compatibility shim and its ownership plumbing cost eight net
production MATLAB lines, so the branch total changes from 163 to 155 removed lines
across five milestones. That unfavorable line-count delta is explicit; the
benefit is one fewer false planner capability and clearer logging ownership.

Default and legacy-`Verbose` obstacle-free runs matched exactly after removing
only measured runtime fields. All 17 maintained examples then ran in separate
fresh MATLAB processes: 16 successes independently passed collision and
kinematic validation, and `exampleNoPath` retained the expected validated
`noValidatedSeed`. The complete suite passed 118/118 in 81.436094 seconds wall
time, Code Analyzer reported zero findings, the visible success created two
figures without warnings, and the hidden failure created its diagnostic figure.

The remaining option audit found no other unread default. At that milestone,
collocation, solver-iteration, and plane-reuse controls were all still active;
the later automatic plane-reuse milestone internalized only the two reuse
fields under exact-result gates.

## Dormant seed-clustering removal - 2026-09-01

The fourth `bmtp-cleanup-codex` milestone removes optional conservative hull
clustering from topology-seed generation. `SeedClusterDistance_deg` previously
defaulted to zero and no current maintained example, test, benchmark, or
sandbox enabled it. The 85-line `clusterSeedShape.m` helper is deleted and
route candidates always use the unclustered protected swept geometry. A
one-release option shim warns that a supplied legacy distance is deprecated
and ignored. The existing `SearchDiagnostics.Grid.SeedCluster` record remains
with its exact default values and source-region count so default diagnostic
schema and plotting consumers do not change.

The measurable maintainability benefit is a net reduction of 83 production
MATLAB lines: 100 removed and 17 added after the compatibility and diagnostic
cost. The branch has now removed 163 production lines across four committed
milestones while preserving route generation, continuous BMTP, separating-
plane reuse, static and time-varying obstacle handling, time policies, motion
constraints, validation, certificates, failure diagnostics, and the public
result/restart contracts.

The strongest correctness evidence is recursive comparison of every current
maintained example against frozen commit `11582e3`. All 17 results match at
`1e-9` outside the intentionally removed option and runtime fields. This
includes seed ordering, visibility graph counts, coverage flags, the retained
zero-valued cluster diagnostic, route and trajectory histories, validation,
certificates, and termination. Sixteen examples succeeded and independently
validated; the expected no-path example retained its validated
`noValidatedSeed` failure. The extreme outline retained 5.81065318159 seconds
arrival and 23.3457566443 degrees of motion, with 67.5731971 seconds wall time
versus 67.4136091 seconds at baseline.

A structurally different three-region fixture proved the removed behavior was
actually exercised. At distance zero the frozen baseline used 26 nodes and 46
visibility edges. At one degree it formed one conservative group and reduced
the graph to 10 nodes and 16 edges, while both returned the same validated
8.08716891419-degree motion at 6.5 seconds. The candidate legacy replay warned
once, used the unclustered 26-node/46-edge graph, and matched the zero-distance
baseline recursively. Its 4.3206596-second wall time was close to the
4.2952183-second clustered run; this small fixture does not establish a global
runtime ratio for fragmented fields.

Broad verification passed 117/117 tests. A hidden no-path run retained the
termination reason and search counts in its figure title, and a visible
obstacle-free run created two visible figures. MATLAB Code Analyzer reported
zero findings in the three changed production files, documentation no longer
claims the helper exists, benchmark rows record every executed example, and
`git diff --check` passed.

The largest remaining option-surface issue is not simple dead state.
`CollocationSegmentCount` actively caps BMTP warm-route and timed segmentation,
and `MaximumNlpIterations` actively sets the trajectory `coneprog` iteration
limit despite its outdated name. They should be evaluated for internal
ownership or clearer naming in independent bounded changes, not deleted as
unused. The next option audit should trace every remaining default and rank
truly unread fields ahead of active solver controls.

## Wall-clock seed cutoff removal - 2026-09-01

The third `bmtp-cleanup-codex` milestone removes the planner's
machine-load-dependent per-seed wall-clock cutoff. Every admitted seed now
runs to the existing deterministic BMTP iteration and cone-program limits, or
to the existing explicit cancellation boundary. The public
`PerSeedWorkBudgetMultiplier` is recognized for one release, warned as
deprecated and ignored, then stripped before ordinary option resolution. The
private `MaximumSolverTime_s`, `WorkLimitReached`, and
`seedWorkBudgetExhausted` paths are gone. `MaximumSeedCount`, the 35 BMTP
outer-iteration bound, nonlinear and cone-program limits, validation,
diagnostics, and the tested public restart API remain.

The measurable maintainability benefit is a net reduction of 40 production
MATLAB lines: 57 removed and 17 added, including the compatibility shim and
example-boundary forwarding. Together with the first two milestones, the
branch has removed 80 production lines while retaining the continuous BMTP
solver, separating-plane reuse, static and time-varying obstacle support,
arrival policies, motion limits, validation, certificates, failure
diagnostics, and public result contract.

The strongest correctness evidence is recursive comparison against frozen
commit `dd7a674` at `1e-9`. Fixed-arrival alternating occlusion, earliest and
balanced obstacle avoidance, the extreme outline, moving/deforming geometry,
and expected no-path results matched their completed-seed baselines outside
the declared option and diagnostic removal. Alternating occlusion now
deterministically completes seed 5 and selects its 13.5713266002-degree motion
instead of sometimes discarding it and selecting the 13.5986641387-degree
motion; arrival remains 20.8695652174 seconds. Earliest arrival remains
7.57454176632 seconds with 11.4118613877 degrees of motion. Balanced arrival
remains 7.54855735896 seconds, 11.2161345431 degrees of motion, and
18.764691902 degrees of declared composite cost.

Broad verification passed 115/115 tests. All 17 maintained examples ran in
separate serial headless processes: 16 planner/example-validation successes
and the expected validated `noValidatedSeed` result. Every successful motion
passed collision and kinematic checks. A hidden no-path run created one
diagnostic figure titled with `noValidatedSeed`, one seed, one expanded state,
and two rejected transitions. A visible obstacle-free run created two visible
figures. MATLAB Code Analyzer reported zero findings in all four changed
production files, and `git diff --check` passed.

The explicit unfavorable tradeoff is runtime. The extreme-outline default
wall time increased from the prior cutoff run's 42.1929796 seconds to
67.4136091 seconds in the full sweep. A controlled completed-seed comparison
was much closer: 72.2275885 seconds before the edit and 72.80709 seconds after
it, with exact non-runtime results. This is accepted because the user
prioritized a smaller deterministic core over early runtime and because the
old cutoff could discard a better valid result. Runtime ratios remain
case-specific; this milestone does not claim a universal slowdown bound.

The next highest-confidence cleanup candidate is dormant seed-region
clustering. Its default is zero, no maintained example or test enables it,
and a separate bounded experiment could remove approximately 90-100
production lines while requiring exact default-result equality. Plane reuse
itself remains explicitly retained: its completed removal experiment worsened
motion length and approximately doubled runtime.

## Dormant waypoint warm-start option removal - 2026-09-01

The second `bmtp-cleanup-codex` milestone removes the planner-option surface
for an implementation that is not present in the repository. No production
planner or trajectory engine consumed `WaypointWarmStartMode`,
`RequestedWaypointWarmStartMode`, or `IsWaypointWarmStartAvailable`; their
only behavior was validation, probing for the absent `ruckigWarmStart.m`, and
echoing fallback state. Current defaults and results no longer contain those
fields. A one-release migration shim recognizes all three legacy names, emits
one explicit deprecation warning, and strips them before ordinary option
resolution. The example option boundary forwards legacy names to that single
warning/strip owner instead of misclassifying them as unknown example fields.

The measurable maintainability benefit is a net reduction of 16 production
lines in `resolvePlannerOptions.m` and 10 production lines overall after the
six-line example-boundary compatibility cost. The tested public BMTP
eight-input/three-output restart interface is unchanged. Together with the
first milestone, the branch has removed 40 production lines while preserving
the active trajectory engine, route families, plane reuse, validation, and
diagnostics.

The strongest result-retention evidence is two controlled comparisons against
the exact accepted baseline commit `93a28e6`. Four paired obstacle-free runs
with a legacy option replay matched at `1e-9` for all non-runtime result data
outside the three intentionally removed option fields. Warmed medians were
0.0777916 s baseline and 0.0835580 s candidate, a 7.413% difference within the
declared 10% noise allowance. A structurally different fixed-arrival,
alternating-occlusion comparison used the existing
`PerSeedWorkBudgetMultiplier=100` diagnostic to remove wall-clock cutoff
variability. Both sides selected seed 5 and returned exactly
27.950433436 deg polyline, 13.5713266002 deg smoothed motion, and
20.8695652174 s duration; recursive comparison found no non-runtime result
difference, and wall time changed from 24.8525406 s to 25.1338125 s (+1.132%).

Broad verification passed 113/113 tests. All 17 maintained examples ran in
separate serial headless processes: 16 planner/example-validation successes
and the expected validated `noValidatedSeed` result. A hidden no-path run
created one diagnostic figure with the reason and search counts; a visible
obstacle-free run created two visible figures. MATLAB Code Analyzer reported
zero findings in both changed production files, and `git diff --check` passed.

The largest observed weakness is the existing wall-clock per-seed work budget.
One default alternating-occlusion run allowed seed 5 to finish and selected a
27.950433436 deg conservative seed whose final motion was 13.5713266002 deg;
other baseline and candidate runs stopped that seed at
`seedWorkBudgetExhausted` and selected the 13.3416640641 deg direct seed with
13.5986641387 deg final motion. Arrival and validity were identical, and the
alternate final motion was shorter rather than worse, but default selected
seed identity is timing-sensitive. The controlled high-budget comparison
shows this cleanup did not create the difference; deterministic work budgeting
remains a separate core-maintainability candidate.

## Self-contained BMTP SOCP construction - 2026-09-01

The first `bmtp-cleanup-codex` milestone removes the immutable trajectory-SOCP
template threaded through the BMTP alternation and travel-refinement loops.
Each trajectory solve now constructs its own equality rows, derivative rows,
bounds, time/travel cones, and active separating-plane rows in execution order.
The final-sized sparse inequality matrix is allocated once, so the prior base
matrix cache plus later enlargement/copy path is gone. Public planner inputs,
options, result fields, diagnostics, solver arguments, and selection policies
are unchanged. The edit is confined to `trajectory/+bmtpEngine/solve.m` and
reduces it from 1,380 to 1,350 physical lines and from 1,235 to 1,205
nonblank/noncomment lines.

The largest measured strength is exact result retention under a controlled
baseline/candidate comparison. The frozen baseline revision was
`5c0a6c97bf68e9db03ace5281bda2e0f84243a8c`. Four paired runs of
`exampleTargetExitsObstacle`, including one warmup and three timed repetitions
per side, had identical success, independent validation, termination, selected
seed/source, certificate decisions, non-runtime diagnostics, and sampled time,
position, velocity, acceleration, and jerk histories. The maximum sampled
numerical difference was zero against a `1e-9` gate. Both sides returned a
24 s motion, 21.7425467317 deg selected polyline, and 21.9416287312 deg
smoothed path.

The explicit unfavorable tradeoff is runtime. The warmed median increased from
10.7475989 s to 11.7573298 s, or 9.395%, on that repeated-SOCP case. This is
inside the predeclared 25% limit and was retained because eliminating hidden
cache state makes the optimization kernel smaller and self-contained. It is
one measured fixed-arrival case, not a general runtime ratio.

Verification covered the structurally different static degree-16 and timed
degree-7 BMTP paths, moving and deforming obstacles, fixed and earliest arrival,
successful and expected no-path outcomes, and graphics diagnostics. The full
test tree passed 111/111 with zero failed or incomplete tests. All 17 maintained
examples ran in separate serial headless processes: 16 planner/example-
validation successes and the expected validated `noValidatedSeed` result. A
hidden failure plot included the reason and search counts, a visible
obstacle-free run created two visible figures, Code Analyzer reported zero
findings for both baseline and candidate `solve.m`, and `git diff --check`
passed.

The largest remaining weakness is that this is intentionally only the first
cleanup milestone. The repository still contains plane reuse, the three-rate
travel-refinement portfolio, specialized exact-clock/timed-opening/cavity
constructors, and an externally visible restart surface. Several are measured
load-bearing for arrival or path length and cannot be deleted honestly until a
general mechanism reproduces their results. This milestone establishes neither
planner completeness nor global optimality; the next bounded experiment is to
evaluate plane-reuse removal independently.

## HTML bundle replay and velocity-authored obstacle motion - 2026-08-31

The HTML sandbox can now load an
`obstacleAvoidanceSandboxDiagnosis-v2` MAT file in live mode and run its
canonical request through the current planner. Replay reconstructs the initial
state, goal, limits, resolved options, original obstacle keyframes, and safety
margins; it does not reuse the result stored in the bundle. The reproduced
result becomes the current downloadable diagnosis bundle, so replay remains a
complete inspect-run-save workflow rather than a display-only import.

Moving-obstacle authoring now follows an explicit Set Motion interaction. The
selected polygon's arrow is a velocity vector in deg/s: its component values
and Euclidean magnitude are displayed, and its length in planning coordinates
equals that magnitude. Constant, zero-start, trapezoidal, and out-and-back
velocity laws are integrated into the 21 position keyframes supplied to the
planner. In particular, zero-start treats the arrow as final velocity, while
trapezoidal and out-and-back treat it as peak velocity. This replaces the old
and easily misread total-displacement arrow.

The measured strength is end-to-end reproduction without a second planner
interface. A raw MAT upload through `/run-bundle` returned a fresh
`goalReached` result in 1.498398 server seconds; its independent validation
passed. A moving-obstacle unit replay preserved both keyframe times and the
final original polygon slice. The complete test tree passed 111/111 in
123.685910 seconds, and Code Analyzer reported no findings in the changed
MATLAB files.

The largest current limitation is that MAT replay requires the loopback MATLAB
server and is capped at 128 MiB. The in-app browser policy blocked opening the
local `file://` page, so no browser-driven visual claim is made; production
JavaScript syntax, the extracted velocity-profile integration, HTML wiring,
MATLAB replay, and the live HTTP endpoint were verified independently. Two
fresh attempts to rerun the maintained example matrix were blocked before the
first example by MATLAB's already-recorded host startup error, `System Error:
File system inconsistency`. Planner sources were not changed by this UI and
transport work; the immediately preceding 17-example matrix remains the exact
planner baseline.

## Balanced travel-time planning and bounded waypoint fallback - 2026-08-31

The planner now separates hard feasibility from preference. `GoalTimeMode`
defaults to `balancedArrival`, whose explicit
`MinimumTravelSavingsRate_deg_s=1` policy selects a later validated motion only
when it saves more than one degree per second of delay. Jerk remains a hard
validated limit and is not a selection cost. Equal-cost candidates prefer the
earlier arrival, then greater mean normalized peak velocity, acceleration, and
jerk utilization. Every seed summary reports its degree-valued tradeoff cost
and utilization, and search diagnostics state the formula and retain the
secondary conic portfolio's trial rates, durations, lengths, and costs.

The largest measured strength is that one input-driven policy now corrects
three different failure mechanisms without scenario branches. The supplied
static shrimp already had a 175.168391-degree upper-boundary seed, but the
time-only BMTP kernel expanded it to 192.556229 degrees and 56.293 degrees of
elevation. A two-stage solve now first establishes a collision-free homotopy,
then minimizes a convex Bezier control-edge travel bound under retained and
newly discovered separating planes. A bounded three-rate portfolio rejects
dominated local scalarizations. The retained shrimp motion is 175.780063
degrees at 82.389498 s, never exceeds 39.288753 degrees elevation, and passes
independent collision, velocity, acceleration, and jerk validation.

For moving obstacles, balanced timed search retains the shortest ancestry at
the mission horizon but removes terminal goal dwell before motion realization.
That supplies a moving-aware later/shorter candidate while the spatial search
supplies the faster end of the comparison. On the supplied non-ideal case the
planner compares a validated 144 s / 228.491135-degree timed motion (cost
372.491135 degrees) with a validated 119.473594 s / 260.029509-degree detour
(cost 379.503102 degrees) and selects the former. This is a measured incumbent
comparison, not a completeness or global-Pareto-optimality claim.

The fixed-clock excursion now validates and compares its progress-polynomial
and one-sided families by actual travel at the identical physical clock. The
supplied sine case falls from 153.472521 to 146.976783 degrees without changing
its 70.344251 s duration or any constraint. Candidate selection no longer uses
integrated squared jerk.

Ruckig waypoint composition now has a visible two-segment hard limit. A route
with more segments returns `ruckigWaypointSegmentLimitExceeded` before Ruckig
runs. The supplied six-segment hidden-fallback request still succeeds through
velocity-carried BMTP at 104.261457 s and 233.911502 degrees; it is not
misreported as no-path and no interior state is forced to rest. The maintained
alternating-occlusion example was moved from explicit Ruckig to BMTP because
its route genuinely exceeds that public limit.

The main unfavorable tradeoff is runtime and boundedness. The shrimp balanced
solve took 32.54 s versus the saved pre-change planner result's 13.08 s; this
comparison includes different MATLAB sessions and is not a controlled speed
ratio. The three-rate refinement is deliberately bounded, and timed search is
still bounded by its node, layer, cell, and seed caps. A returned solution is
independently valid, but failure does not prove that no continuous trajectory
exists and success does not prove the complete Pareto frontier was found.

Code Analyzer reported zero findings on all changed MATLAB files. The full
suite initially passed 107/108 tests; the sole failure was a certificate fixture
that had unintentionally inherited the new default. After declaring its
intended `earliestArrival` policy, the focused certificate suite passed 3/3,
and the final complete suite passed 108/108 in 111.388 s. All 17 maintained
examples ran in separate headless MATLAB processes: 16 independently validated
successes and the expected `exampleNoPath` failure. A hidden no-path run created
one diagnostic figure, and a visible obstacle-avoidance run created one visible
validated figure.

## Sandbox route-economy coverage - 2026-08-31

Sandbox-scale route tests now measure accumulated two-axis travel and
meaningful lateral velocity reversals for a static circle, an irregular
concave static outline, and the same irregular outline moving across the
direct route. Each case also requires independent collision, velocity,
acceleration, and jerk validation. The static-circle guard compares the
returned motion with the exact tangent-and-arc geometric lower bound; the
irregular cases use direct endpoint distance as a conservative lower bound.

Before clearance-boundary refinement, the centered protected circle returned
a validated 7.333333-second fixed-clock motion of 16.822181 degrees. Refining
the coarse failing/passing amplitude bracket with authoritative continuous
validation retains the same arrival time and reduces travel to 16.700092
degrees. The protected-radius tangent-and-arc lower bound is about 16.638
degrees. The retained motion is therefore within one percent of that geometric
lower bound and contains one lateral reversal. A visibility-route alternative has a
16.636942-degree geometric seed, but its smooth realization is 17.216282
degrees and takes 8.169085 seconds. A waypoint-stop realization preserves the
16.636942-degree geometry but takes 15.675572 seconds and stops at every
interior point. Those alternatives were rejected because they increase time,
joint cycling, or both.

The route-economy checks limit regressions; they do not prove global
minimum-wear motion. The planner still prioritizes earliest validated arrival,
then path length and integrated squared jerk. Mechanical wear also depends on
loads, backlash, lubrication, and controller behavior that are not modeled.
The refined boundary requires additional full validation calls. The centered
circle planning call took 3.123552 seconds in the retained focused run; an
identically instrumented pre-change runtime was not recorded, so no runtime
ratio is claimed.

This file records the authoritative state of `novel-rep` and a concise ledger
of approaches already tried. Superseded benchmark matrices remain in
`benchmark.csv`; verification details remain in `verification.md`. Historical
work is retained here only when it records a mechanism, outcome, or warning
that should influence future planner work.

## Current state: smooth timed multi-waypoint BMTP - 2026-08-31

The planner now has a general smooth path for time-expanded multi-waypoint
seeds. It partitions each moving obstacle history into physical-time cells,
uses the convex hull of protected endpoint and midpoint geometry as a
conservative cell superset, and applies each cell only to overlapping equal-
duration Bezier spans. Static concave geometry remains exactly decomposed and
active for the complete motion. Interior search points guide the warm route;
they are not constrained to zero velocity or acceleration.

The primary frozen baseline at `fe076fe` was a moving circle plus a static
concave U, with two admitted seeds and the default fail policy. It returned
`unsupportedTimedMultiWaypointRoute` after 4.790300 s. With the retained
changes, that exact request returned an independently validated smooth motion
at 14.634958917 s with 39.676450938 deg sampled length; its minimum speed at
the four interior timed-seed points was 3.644660375 deg/s.

A stronger version held the circle in the detour for the first 10 s, which
made both static and swept projections fail. The time-cell solver then selected
the time-expanded seed and returned an independently certified 35 s motion:
45.574198839 deg selected polyline, 45.517670812 deg sampled smooth length,
and 1.469097516 deg/s minimum speed at interior timed-seed points. Its complete
91-pair time-cell certificate reconstructed every static region and moving
cell from the original protected obstacles before verifying the degree-one
Bernstein planes. A structurally different four-span translating-polygon
engine test also verifies that a region is enforced only on its overlapping
time spans.

The contact-linearization correction is also retained. A solved separator for
a visibility seed that merely touches protected geometry may initialize the
next alternating trajectory step even when it does not yet have the required
positive gap. It is never an acceptance certificate: final Bernstein plane
verification and public trajectory validation remain mandatory. This removed
a measured `3.3e-8 deg` numerical-contact dead end without relaxing collision
clearance or any public tolerance.

The largest current weakness is completeness and conservatism. Time cells use
convex supersets, the solver currently maps the search-layer clock to at most
the existing BMTP span budget, and earliest-arrival work is bounded to the
search estimate plus the request horizon. A failure is therefore evidence that
these bounded representations found no validated trajectory, not proof that
no dynamic path exists or that a returned arrival is globally optimal.

Final verification passed Code Analyzer on all six changed MATLAB files and
101/101 tests. All 17 maintained examples ran in separate serial headless
MATLAB processes: 16 independently validated successes and the expected
validated `exampleNoPath` failure. The visible moving-circle example created
two figures and passed. Existing static-U and moving-barrier sentinels retained
20.7124477860115 s and 10.0903015136719 s durations, respectively.

## Current state: stagnation stop trade - 2026-08-31

The earlier statement that the stagnation stop was dead on the default path is
wrong. The stop worked and was the more accurate of the two measured methods
on `exampleUSOutlineExtremeVisibility`.

| setting | arrival (s) | length (deg) | plane solves | wall (s) |
| --- | ---: | ---: | ---: | ---: |
| neither | 5.794009507455 | 23.354756039381 | 776 | 88.39 |
| stagnation stop | 5.794009507455 | 23.354756039381 | 468 | 64.73 |
| plane reuse | 5.810653181589 | 23.345756644341 | 160 | 52.42 |

The stagnation stop reached the exact original arrival and was 27% faster.
Plane reuse was 41% faster and arrived 0.016643674134 s later. Kevin chose
plane reuse, accepted that arrival-time cost, and chose to retain one method
rather than two.

At baseline `9d18840`, counted production lines were 11,618, the full suite
passed 92 of 92 tests, and all 17 reference examples verified. Earlier
stagnation-stop numbers below are historical records and do not supersede this
current-state decision.

## BMTP unchanged-plane reuse, initially gated off - 2026-08-30

`EnablePlaneReuse=false` preserves the existing plane reset and re-derivation
path. The four supplied default sentinels were bit-identical to their required
arrivals and sampled motion lengths: Target Exits
`24 / 22.554006042022394`, Obstacle Avoidance
`7.574541766321258 / 11.411861387735195`, Static U
`20.712447786011488 / 40.255028504000862`, and Two Opposing U
`21.633333333333336 / 24.096812127187516` (seconds / degrees). All passed
independent validation. The clean MATLAB suite passed 96/96, the changed files
were Code Analyzer clean, and `exampleNoPath` retained `noValidatedSeed`.

The enabled gate requires both a retained-best improvement no greater than
`PlaneReuseImprovementTolerance_s` and an unchanged tagged-pair set. It skips
only the reset/re-derivation and performs the next trajectory solve. In the
`1e-4 deg` Target case that repeated SOCP was bit-identical on this machine:
the maximum control-net and duration differences were both zero, then the
existing convergence test fired. Baseline / stagnation-only / reuse-only
respectively used `42 / 18 / 15` outer and trajectory SOCPs and `391 / 141 /
93` plane SOCPs. Their warmed minimum / median walls were
`53.0606007 / 53.5620057`, `23.8239231 / 24.4742484`, and
`19.5354281 / 19.7202448 s`; all retained the `24 s` arrival and
`22.555163889326948 deg` path. Reuse therefore achieved more plane-SOCP and
wall reduction than the existing stagnation stop in the measured wandering
case; they are not redundant there.

At default clearance, Target Exits similarly changed from `17 / 121` outer /
plane SOCPs to `12 / 73`, with a `20.8999410 s` baseline median and
`16.3771825 s` reuse median, with no result movement. Obstacle Avoidance did
not trigger the gate (`7 / 13` outer / plane SOCPs in both modes); Static U and
Two Opposing U did no BMTP conic work in either mode. The read-only Rogue
bundle was also a gate-null at both tested horizons: 180 s stayed at
`22 / 576`, `100.664824112242897 s`, and `221.885353904752918 deg`; 360 s
stayed at `20 / 507`, `100.675947361398343 s`, and
`220.666927423424511 deg`. Its warmed medians changed only within ordinary
wall variance (`25.0848222` to `24.8300924 s` at 180 and `23.0830120` to
`23.4646274 s` at 360). The supplied bundle succeeds at 180 s on this branch;
the measured 180 result agrees with the supplied known-good arrival and length
despite the brief's historical failure description.

The option remains off because this is a measured diagnostic/runtime tradeoff,
not a proof that every future alternating problem has deterministic SOCP
repeats. The retained reset remains load-bearing whenever the incumbent is
still improving or tagged pairs change.

### Reverted, then restored - decision record

This mechanism was reverted once (`438c0be`) and then restored. The revert
applied a kill criterion worded as "Regime C regresses in any way", against a
warmed 360 s median moving from 23.0830120 to 23.4646274 s. That criterion was
too strict for wall time and the rejection was wrong:

- The gate does not trigger at all on the Rogue bundle. Counts, arrival,
  length, and validation were identical at both horizons.
- The only added work on that path is one small logical-array copy and one
  `isequal` per outer iteration - microseconds across 20 iterations, not the
  0.38 s the median moved.
- The two horizons moved in opposite directions: 180 s improved from
  25.0848222 to 24.8300924 s while 360 s worsened. Opposite-signed movement on
  a gate-inert path is variance, not effect. The section above had already
  described it as ordinary wall variance.

Wall-clock variance on this machine has exceeded 30% between sessions, larger
than the difference that triggered the rejection. Behavioural invariance, not
wall time, is the criterion that should gate a change on a path the option
never touches.


## Historical: BMTP retained-best stagnation stop, off by default - 2026-08-30

The optional retained-best stop is inert at its default. Four supplied
sentinels kept their recorded arrivals and smoothed lengths: Target Exits
`24 / 22.554006042022394`, Obstacle Avoidance
`7.574541766321258 / 11.411861387735195`, Static U
`20.712447786011488 / 40.255028504000862`, and Two Opposing U
`21.633333333333336 / 24.096812127187516` (seconds / degrees). All four
passed independent validation.

With `EnableStagnationStop=true`, `StagnationIterationLimit=5`, and the
resolved arrival tolerance, the diagnostic `1e-4 deg` Target case reduced
from 42 outer iterations and 433 conic calls to 18 and 159 without moving its
returned `24 s` arrival or `22.555163889326948 deg` smoothed path. The default
Target case also triggered, reducing 17 / 138 to 15 / 120 while retaining its
exact returned result. The other three production sentinels executed no BMTP
conic calls, so the option was a mechanical null there.

After one discarded warm-up, the `1e-4 deg` raw off walls were
`57.1787115 / 56.4846390 / 55.8231577 s`; enabled walls were
`24.9188578 / 26.2214737 / 25.3317567 s`. Thus the observed min / median
changed from `55.8231577 / 56.4846390 s` to `24.9188578 / 25.3317567 s`.
This is a diagnostic-clearance benefit, not a claim about every production
scene or a reason to enable the option by default.

The adverse horizon sentinel is also a null: at 180 s, both modes returned
the independently validated `100.664824112242897 s` motion with 22 / 598
outer / conic work and no stagnation trigger. The 360 s control stayed at
`100.675947361398343 s`, also with no trigger. This does not establish a
production-wide performance win. The documented default remains off; enable
only as an explicit diagnostic/runtime tradeoff because the retained
best-before-stop can be worse than a later oscillating iterate even when the
returned example result does not move.

## BMTP immutable SOCP cache retained - 2026-08-30

The measured reconstruction bottleneck was reduced without moving an answer.
The maintained Target Exits warm median changed from `15.7403272` to
`14.1946827 s`, and its `1e-4 deg` diagnostic changed from `41.7338001` to
`35.3340565 s`. Corresponding `solveTrajectorySocp` profiler self time fell
from `2.882206656` to `0.394152606 s` at default clearance and from
`7.664537344` to `0.448020402 s` at `1e-4 deg`.

The cache is deliberately narrow: it reuses only immutable per-seed
trajectory-SOCP structure while rebuilding plane rows and horizon bounds in
their original order. A temporary legacy-versus-cache oracle found exact
trajectory `coneprog` arguments across all 17 trajectory calls in the
138-conic-call default Target sequence, all 42 trajectory calls in the
433-call slow sequence, and a horizon expansion. All 17 maintained examples
had exactly zero arrival, selected-polyline, and smoothed-path movement against
the archived `747f46c` baseline; aggregate solver structure was also exact.
Focused tests passed 31/31 and the staged gate passed 94/94.

The original instability remains visible. At `1e-4 deg`, seed 2 still
oscillates to the 35-iteration cap; the cache reduces reconstruction cost but
does not alter that stopping path. Maintained Straight Target remains a
zero-BMTP Ruckig control. Its explicit BMTP diagnostic is wall-budgeted, so
outer and conic call counts can vary with wall timing even when its answer is
unchanged.

The branch also remains oversized. Counted production is now 12,015 lines,
40 above the 11,975 baseline and 4,515 above the literal 7,500 target.
`solve.m` is 1,065 physical and 936 noncomment lines, above its 900-line
target. The runtime and exactness evidence supports retaining the cache, but
it is not a size-compliance result and does not resolve the slow-clearance
alternation mechanism.

## BMTP conic runtime localization - 2026-08-30

Fresh warmed, repeated profiling corrects two runtime attributions. Maintained
Straight Target is a Ruckig waypoint case: its three-run wall was
`5.2936647 s` minimum and `5.4580266 s` median with zero BMTP and zero
`coneprog` calls. Its 5.5--6 second runtime must not be used as evidence about
the conic solver. An explicitly labeled BMTP override measured `20.9824146 s`
minimum and `21.2219000 s` median.

Maintained Target Exits at the default `1e-7 deg` clearance measured
`17.6197381 s` minimum and `17.6795313 s` median. The otherwise identical
`1e-4 deg` diagnostic measured `48.2423766 s` minimum and `49.1185155 s`
median. The slow regime was reproduced in every recorded repetition.

The adverse mechanism is now measured. Both modes reached the seed-2 first
collision-free iterate at outer iteration 2 and used no horizon expansion or
caller restart. Default seed 2 converged in 5 iterations; `1e-4` stayed
collision-free but its objective oscillated until the 35-iteration cap. The
total therefore changed from 17 outer iterations and 138 conic calls to 42 and
433. All Target Exits calls exited `+1`. Individual seed-2 trajectory-call
medians changed only from `0.6687750` to `0.7168136 s`; repeated calls, not a
single pathological solve, dominate the near-threefold wall swing.

MATLAB profiler self time identifies immutable trajectory-SOCP reconstruction
as the largest engine-owned cost outside `coneprog`: `2.882206656 s` over 17
calls at default clearance and `7.664537344 s` over 42 calls at `1e-4`.
The next bounded experiment may reuse only those immutable per-seed matrices,
bounds, and cones. Plane-dependent rows and horizon limits must remain fresh;
all solver tolerances, acceptance rules, and outputs remain frozen. A movement
over `1e-9` in any maintained arrival or path length is an immediate revert.

## Ruckig-to-BMTP warm-start experiment stopped at step 2 — 2026-08-30

The approach failed its predeclared collision-survival gate and is not wired
into the planner. Across five maintained static cases, eight Ruckig route
motions first passed the unchanged public endpoint, dynamics, and collision
validation. All eight converted successfully to the exact degree-16 BMTP span
count, but only one converted control net passed the complete degree-one
Bernstein plane check: `1/8`, or 12.5%. The declared kill threshold was fewer
than one-third of at least six validated conversions.

| Case | Converted seeds | Maximum conversion error (deg) | Certified |
| --- | --- | ---: | ---: |
| `exampleObstacleAvoidance` | 2, 3 | 1.74313715049989e-5 each | 0/2 |
| `exampleStaticUShapedObstacle` | 2, 3 | 7.85168079719939e-5 each | 0/2 |
| `exampleTargetExitsObstacle` | 2, 3 | 1.34309047285578e-5; 5.68171630073286e-5 | 1/2 |
| `exampleAlternatingSlalom` | 2 | 9.50061068454221e-6 | 0/1 |
| `exampleDenseConcaveObstacle` | 2 | 9.59122759523563e-6 | 0/1 |

The census used an extended earliest-arrival horizon only for the intermediate
Ruckig source, then independently validated that complete source motion before
conversion. Direct seeds that failed collision validation were excluded from
the denominator. A preliminary constant-plane-only result of 0/8 was discarded
because it did not implement the specified degree-one validator; the 1/8 result
above uses the same conic plane form and direct Bernstein-product inequalities
as BMTP.

No planner option or caller was added, so no arrival, route, or cold BMTP
behavior changed. In accordance with the kill criterion, outer iterations to
first feasibility and repeated warm/cold wall times were not measured. Step 2
adds 323 counted production lines, taking the experimental total to 11,975;
this dead-end code has no measured performance allowance.

## Ruckig-to-BMTP warm-start experiment, step 1 — 2026-08-30

The standalone equal-duration converter is implemented but remains unreachable
from the planner. It fits each requested Bernstein span at Chebyshev-Lobatto
times and reports independently sampled position error; it does not claim that
the converted curve is collision-free or suitable for BMTP yet.

On one exact six-phase Ruckig rest-to-rest profile, degree 7 with 64 uniform
spans reproduced spans containing no interior jerk switch to
`4.96506830649455e-15 deg`; the maximum over all 64 spans was
`4.97014121608466e-9 deg`. With two uniform spans, each containing two
interior switches, the measured maximum was `0.000534264339868523 deg` at
degree 7 and `3.52475369970282e-5 deg` at degree 16. These are sampled errors
for one fixture, not general bounds.

The final step-1 edit passed the fast sentinel gate, the two converter tests,
and all nine architecture-boundary tests; Code Analyzer reported no findings
in the three changed MATLAB files. The production audit moved from the exact
branch baseline of 11,524 to 11,652 nonblank, noncomment lines: the unwired
converter adds 128 counted lines. Retention therefore remains conditional on
the later collision-survival and first-feasibility iteration gates.

## Static earliest-arrival horizon monotonicity — 2026-08-30

The frozen `180bad360good` request disproved the suspected seed-admission
failure. At both 180 s and 360 s the search generated and admitted the same two
seeds: a 153.358411534181 deg direct seed estimated at 76.6792057670906 s and a
281.401707597662 deg visibility seed estimated at 140.700853798831 s. Before
the correction, the visibility seed ended as `noOptimizedFeasibleIterate` at
180 s but reached 100.675947361398 s at 360 s.

The earliest divergence was inside the static BMTP biconvex solve. Six
horizon-bounded trajectory SOCPs remained colliding at durations through
179.203882993999 s, and the seventh became infeasible. The 360 s solve used the
same plane sequence to find a temporary 191.541694821082 s collision-free
iterate, then descended to the roughly 100.676 s validated motion. Thus the
request horizon incorrectly bounded a feasibility iterate even though final
horizon enforcement already existed in BMTP and public validation.

The retained correction activates only when the horizon-bounded SOCP reports
infeasibility before any collision-free iterate. It doubles the intermediate
horizon, capped by the finite 454.754593605252 s kinematic duration of the
seed-warm control net, and restores the request horizon immediately after the
first collision-free iterate. Work remains bounded by the existing 35 outer
iterations, per-SOCP iteration cap, and any active per-seed solver-time budget.
No seed gate, public validator, tolerance, obstacle geometry, or scenario rule
changed.

In the final single-process gate, the 180 s request succeeded with independent
validation at 100.664824112243 s in 19.2650543 s wall. The unchanged 360 s
request succeeded at 100.675947361398 s in 16.2218415 s wall. All 17 maintained
examples passed serial headless validation; `exampleNoPath` remained the
expected `noValidatedSeed` failure; visible success and hidden failure plots
were created. Code Analyzer reported zero findings, the complete suite passed
84/84 in 22.2649327 s, and production remained exactly 11,524 counted lines.

## Dynamic-scene findings — 2026-08-30

These supersede two claims in commit `4138f26`'s message, which were wrong.

### Correction to `4138f26`

That message states the rogue bundle "still takes about 177.8 s wall" and that
edge-query batching "is not attempted here". Both are false. The 177.8 s figure
was a bad measurement taken under process contention. Two independent
measurements of the same commit give **12.9 s and 14.5 s**, and the committed
reachability-frontier search already groups edge queries by layer pair:
`edgeIsClear` calls fell from 23,040 to 82, and `queryObstacleOccupancyAtTime`
from 23,093 to 222. The commit is pushed, so the record is corrected here
rather than by rewriting history.

### Sandbox bundle `az_el_sandbox_goal_20260829_212652`

Two moving obstacles, 180 s horizon, about 203 degrees of azimuth travel at
2 deg/s. Before `4138f26` this request never returned: cancelled at 180 s,
150 s, and 120 s. It now succeeds in about 12.9 s with independent validation,
arrival 107.632292801 s and length 227.751816227 deg. `HS3-planner` solves the
same request at 155.205670334 s, so this branch arrives 47.573378 s earlier.

The failure had two causes. The time-expanded search built the complete edge
list for every free node at every layer with no bound. Separately, with dynamic
obstacles `supportsStaticHorizon` is false, so every seed routed to
`createTimedSeedCandidate`, which accepts only `directWait` seeds and returned
`unsupportedTimedTopology` in about one millisecond — discarding the entire
topology search. A `noValidatedSeed` result with that per-seed signature is a
false negative, not evidence of infeasibility.

### The orthogonal-cavity path is load-bearing

A removal experiment deleted `createOrthogonalCavityMotion`,
`certifyOrthogonalCavityLowerBound`, `evaluateArrivalCertificatePortfolio`, and
their planner wiring, then measured the sentinels.
`exampleStaticUShapedObstacle` regressed from 20.7124477849715 s to
20.7814828183771 s, so the deletion was reverted in full. Those roughly 880
lines earn their keep and are not size-reduction candidates.

That experiment also invalidated an attribution method worth recording. Reading
the winning construction from `result.Seeds(SelectedSeedIndex).Source`
misreports cavity wins: when the cavity portfolio wins, `planCorridorQuintic`
passes the original topology seed to `finishFastPath`, so the source still
reads `visibilityGraph`. A census built that way reported zero cavity wins for
code that measurably changes the result. Attribute constructions from
`SearchDiagnostics`, not from the seed source.

### Roundoff reserve consistency correction

Unifying the plane-certificate `roundoffReserve_deg` formulas moved
`exampleStaticUShapedObstacle` from 20.7124477849715 s to
20.7124477860115 s, a measured +1.04e-9 s change. The constructor's
`eps(coordinateScale_deg)` form was the outlier; the shared helper now uses the
more conservative validator-owned `eps * coordinateScale_deg` formula so the
constructor and authoritative certificate check reason in the same reserve.

### Closed moving-barrier arrival gap

At `d0f00e1+worktree`, `exampleMovingBarrierWait` arrives at
10.0903015136719 s on the unchanged 10 deg path, improving both the prior
10.5 s branch result and `HS3-planner`'s recorded 10.2314453125 s result. The
accepted `directWait` seed now validates a zero-wait lower trial and bisects
the measured infeasible/feasible bracket through the unchanged public
validator. Sixteen deterministic trials refined the wait from 3 s to
2.59030151367188 s; the final measured infeasible lower wait was
2.5902099609375 s. This is a bounded refinement of one validated direct-wait
construction, not a request-wide minimum-arrival proof.

## Current verdict — 2026-08-29

The branch is a working research milestone, not yet the requested beta. It has
one public planner, separated BMTP and Ruckig trajectory engines, stable failure
diagnostics, and independent continuous validation. The last complete matrix
had 16 validated successes and one expected validated failure, but the branch
still misses the combined size, runtime, arrival, and path-record gates. That
matrix does not prove general completeness or global optimality.

- Current integration and cooperative-cancellation suite: 84/84 tests passed
  after the reachability-frontier timed-search port. The rogue bundle now
  terminates as `noValidatedSeed` in 149.484 seconds under its 180-second poll.
- The sandbox now has a Stop action that remains enabled during synchronous
  planning, polls the time-expanded and homology searches plus planner-stage
  boundaries, restores idle state, and enables a replayable pre-run export.
  Cancellation cannot preempt MATLAB inside one atomic solver or vectorized
  geometry call; it takes effect at the next safe checkpoint.
- Production size audit rule: 11,524 nonblank, noncomment lines across 72
  files at HEAD. That measured size is now the ceiling, replacing the
  earlier 4,999 target. The audit counts only `+obstacleAvoidance` and
  `trajectory`; `tests/`, `examples/`, `benchmarks/`, and `sandbox/` are
  outside the counted roots, so adding coverage costs nothing against it.
- Strongest result: Two opposing U reaches the exact `649/30 s` physical
  arrival floor with a 24.0968121271875 deg path and 3.0005152 s full wall.
- Straight Target now explicitly selects exact Ruckig waypoint composition. It
  retains the 20.8695652173913 s clock and passes every public check with a
  20.7720160748 deg path, 5.8749177 s planner time, and 10.1040635 s wall.
  Target Exits remains at the `944a738` BMTP result pending a fresh Ruckig gate.
- In that matrix, Obstacle Avoidance is about 46.57 ms late. Extreme is
  1.179 ms late, inside the user's 2.07 ms allowance but still above the
  literal historical record.
- Straight Target and same-input Target Exits remain longer than their
  historical records; the restored Ruckig route stops at its intermediate
  vertices and is not a claim of locally time-optimal waypoint motion.

## Current invariant boundary

- `obstacleAvoidance.planTrajectory` remains the sole public fixed-goal
  planner; obstacle construction, planning, validation, and frozen plotting
  remain separate.
- The public validator independently owns endpoint, time, workspace,
  derivative, collision, safety-margin, and certificate acceptance.
- Expected no-path and bounded-work outcomes retain stable results and search
  diagnostics. Obstacle grouping remains admissible only with reconstructed
  exact-region coverage.
- Every counted production file has a production or contract-test caller. The
  sub-5,000 target therefore needs a behavior-preserving algorithm cutover,
  not dead-file deletion. No transitive compact implementation has yet proved
  current diagnostics, continuous validation, and plane-certificate parity.

## Strongest current evidence

The Two-U request requires 20 degrees of elevation travel from rest to rest
under `1 deg/s`, `0.75 deg/s^2`, and `2.5 deg/s^3`. The independent scalar
switching lower bound is exactly `649/30 s`. A retained degree-15 progress
polynomial attains it and passes every public continuous check with
0.00618966852407 deg protected clearance. This proves globally minimum arrival
for that request, not globally minimum path length.

## Current blockers and next bounded gate

1. Do not grow past the 11,524-line ceiling. Reduction is welcome but is no
   longer a release gate: the one measured attempt, deleting the
   orthogonal-cavity path, regressed `exampleStaticUShapedObstacle` and was
   reverted, so remaining size is not obviously recoverable without losing
   capability. `trajectory/+ruckigEngine` (2,083 counted lines) is shared
   with other projects and `+obstacleAvoidance/+plotting` (425) is frozen,
   leaving about 7,138 counted lines in scope for any future reduction.
2. Recover pass-through path quality without relabeling the local state-to-state
   Ruckig engine as a waypoint-optimal solver.
3. Close the Obstacle Avoidance arrival gap and both path-record gaps without
   scenario branches or relaxed tolerances.
4. Reduce Extreme planner and scenario-construction wall while preserving
   exact protected geometry and the accepted arrival tolerance.
5. Rerun all maintained examples sequentially before another release claim.

The current fixed-arrival gate retains 20.8695652173913 s but still must reduce
Straight Target from 20.7720160748 deg and 10.1040635 s wall to the comparable
13.678271907957 deg and 2.0964864 s records. Target Exits must retain 24 s,
reach at most 20.6764423274 deg, and beat the 5.167399 s same-input wall record.
The older 20.2803317257 deg and 1.9774286 s rows used a different randomized
target endpoint and are not comparable to the maintained deterministic fixture.

Headless profiling at `4ed7f46` localized the Straight Target loss to motion,
not topology: BMTP spent 70.6255 of 75.4950 planner seconds and
selected a longer seed even though a shorter valid seed was available. Its
output uses 48 degree-16 certified spans, while the same-input historical path
record used ten quintic spans. The Target Exits profile spent 22.1507 of
31.3204 planner seconds in motion. Comparisons must include like-for-like
planner work and pass the unchanged public validator.

## Experiment ledger

### 2026-08-21 — early low-dimensional spline replacement

- **Bounded quintic B-spline:** one-, two-, and five-turn cases constructed
  faster than HS3 but arrived 14–35% later. Ten turns remained colliding after
  mean, worst-clearance, and per-obstacle objectives; 20 turns was not run.
- **Fixed-stop septic Bezier:** interpolated vertices but required 28–84 s
  motion and violated the maintained polynomial format; removed.
- **Interior-route interpolation:** reduced 10-turn wall but worsened
  clearance because route reduction discarded topology; removed.
- **Scalar option sweeps:** timing reserve retained route detail but took
  140.74 s with only 0.000145 deg clearance. Duration weight, collision
  penalty, step size, and offset-bound changes did not repair the mechanism.
- **Feasibility-first hard corridor and worst-clearance ranking:** both failed
  focused gates and were removed.
- **Topology-preserving batching/reduction:** produced valid 10-turn splines
  in 5.22–8.45 s but with less than 0.001 deg clearance.
- **Affine corridor prototype:** reached certified 0.02 deg clearance on a
  10-turn route and a 12-wall maze. The 20-turn motion exceeded its horizon,
  so no production replacement was established.
### 2026-08-22 — corridor-only replacement development

- **Corridor-only cutover:** removed dormant HS3/NLP paths and established an
  input-driven visibility/corridor planner. It passed maintained cases but did
  not meet global arrival or size requirements.
- **Span-demand controller:** replaced 180 timing trials with bounded
  per-span feedback. It was faster on Single U with a measured arrival
  penalty; later superseded.
- **Batched affine corridor work:** reduced repeated timing and constraint
  construction. Broader timing variants regressed records and were removed.
- **Dynamic seed-slot reservation:** recovered one moving-circle field while
  preserving deterministic order; broader timed-route replacement failed.
- **Shallow collision-residual feedback:** recovered narrow penetrations.
  Applying it to deeper residuals regressed Extreme to 8.395298096 s, so the
  broad form was rejected.
- **Retry-exhausted boundary support:** workspace-corner nodes recovered the
  fixed moving-circle sweep. Earlier activation regressed a maintained result.

### 2026-08-23 to 2026-08-25 — broad phase and legacy cutovers

- **Ungrouped collision broad phase:** materially accelerated 40 circles with
  exact trajectory parity.
- **Legacy cutovers:** **Combined method suite**, **Compact corridor cutover**,
  **Standalone Hermite-Simpson sequence**, and **HS3-only production cutover**
  were exercised. A cross-frame stress case exposed a compact-corridor seed
  collision; standalone HS3 had weak conditioning, runtime, and record
  quality; the HS3-only cutover removed 2,525 lines but kept those weaknesses.
- **Legacy recovery:** **Corridor regression recovery**, **Compact C3 duration
  controller**, and **Exact derivative retimer** repaired dense-route or
  selected U cases, but not every maintained arrival record.

### 2026-08-26 — quality, timing, and architecture experiments

- **Mesh and refinement family:** **Dynamics-timescale mesh start**, **Direct
  long-detour refinement**, **Severe-static fixed-time quality solve**,
  **Derivative-slack continuation**, and **Dynamic spatial quality** improved
  selected cases. Broader static, all-seed, or continuation use regressed
  arrival, runtime, or quality.
- **Route, ranking, and retiming family:** **Obstacle-free bounded arrival**,
  **Same-homology shortcutting**, **Certified direct collinearity**,
  **Fixed-arrival geometric lower bound**, **Fixed-arrival length-first
  ranking**, **Shortest-route-first ordering**, **Time-expanded retiming**, and
  **Timed-arrival repair** were tried. Benefits were local; one final matrix
  was slower and one timed case remained late.
- **Deforming-outline localization:** polygon buffering and nonlinear
  constraints were dominant; a classification micro-optimization was removed.
- **Two-product and flat architectures:** both package layouts were tried and
  superseded; they did not change the algorithmic limits.

### 2026-08-27 — performance and invariant consolidation

- **Direct motion and moving geometry:** **Exact third-order switching and
  scalar progress**, **Prepared dynamic boundary queries**,
  **Convex direct-route arrival search**, and **Monotonic direct-line progress**
  improved eligible direct cases, but one moving-target timing observation was
  adverse.
- **Invariant batching and caching:** **Batched occupancy and deferred
  allocation**, **Static geometry and polynomial-map caches**, and
  **Prepared constraint-layout reuse** produced measured local wins. A timed
  moving-barrier CG variant regressed.
- **Bounded sandbox planning** made one moving result 26.93% later.
  **Unified seed equivalence** reduced ownership but missed its runtime gate.

### 2026-08-28 — hybrid replacement experiments

- **Direct Ruckig before topology**, **Optional pass-through warm start**, and
  **Nonuniform mesh** each gave a local benefit or retained engine capability
  but no general runtime or selection win; their broad policies were removed.
- **Adaptive static hybrid** and **Alternating-slalom hybrid** improved selected
  cases, including a 10.7625 s validated slalom, but variability and a broader
  near-direct trigger prevented a general replacement.
- **Moving/deforming runtime gate:** query reuse was 9.02%; skipping the coarse
  basin took 192.03 s, and a short iteration cap regressed path quality.

### 2026-08-29 — BMTP replacement branch

- **Restored exact Ruckig engine and explicit route method:** restored the
  independent state-to-state switching engine and facade, added the general
  `TrajectoryMethod="ruckigWaypoint"` route adapter, and wired Straight Target
  through it without a BMTP fallback. The engine tests passed 10/10, the full
  suite passed 78/78, and a structurally different static-box detour passed
  earliest and fixed arrival. Straight Target passed at the exact fixed clock,
  but its stop-at-waypoint path and wall remain above the historical records.
- **Exact waypoint fallback:** composed exact jerk-limited stops along a failed
  spatial seed and recovered a failed multi-edge request. Swept-envelope HS3
  was rejected.
- **BMTP cutover:** replaced legacy production with a separated Bezier/plane
  engine, exact direct/event kernels, and complete region certificates.
- **Exact-clock progress polynomial:** attained the Two-U global arrival lower
  bound with sub-10-second wall and truthful route provenance.
- **Full fixed-corridor Bezier QP:** rejected. Static U produced
  30.047333018 s and about 45.6554866 deg versus retained 20.712447785 s and
  40.2550285 deg. Missing certificate ownership was diagnosed, but quality
  already failed, so experiment code and artifacts were removed.
- **Fixed-clock null-space family:** a 109-variable form beat the dense path
  target and validated but took 35.14 s. A seven-coefficient form retained
  quality, then passed Straight Target with a fully plane-certified
  13.582258304 deg path at the exact 20.869565217 s clock in 8.03538 helper
  seconds. Target Exits remained valid but took 22.1424908 helper seconds and
  produced 21.811076622 deg versus the comparable 20.676442327 deg record.
  Shared preparation and direct validation were excluded from that timer, so
  no general or end-to-end speed claim was retained.
- **Rest-state safe-interval search:** rejected as a universal replacement.
  On Alternating Slalom, its 8-vertex visibility route was only
  16.0193197983 deg, but exact rest-to-rest jerk primitives summed to
  24.6732769008 s versus the retained 10.5 s fly-through motion. Stopping at
  every graph vertex cannot meet the maintained arrival records.
- **Open-quintic route smoother:** rejected at its first frozen gate. It made
  a fully validated Straight Target trajectory in 3.0661778 s candidate scope
  at the exact 20.869565217 s clock, but its 13.7395585901 deg path missed the
  13.678271908 deg record. Target Exits was not run after that gate failed.
- **Historical eight-span C3 sampled-barrier QP:** rejected at its first
  frozen gate. Straight Target remained fully valid at the exact
  20.869565217 s clock, but its 13.7165279811 deg path missed the
  13.678271908 deg record and projected planner scope was 5.0271313 s versus
  the 2.0964864 s wall record. Target Exits was not run, and all experiment
  code and artifacts were removed. A later same-input reconstruction confirmed
  that length-first selection cannot improve it because seed 2 is already its
  shortest validated motion. Increasing the compact basis through 9, 10, 12,
  and 14 spans preserved every public check but only reached 13.7164337975 deg;
  warm walls were 4.4719357, 3.4983968, 3.8779253, and 5.1402292 s. The quality
  gap is therefore representation-level, not a candidate-ranking defect.
- **Fixed-clock velocity-energy C3 QP:** rejected after the Straight Target
  gate. Six relinearized QPs passed continuous motion validation and beat the
  path record at 13.6049323647 deg on the exact 20.869565217 s clock, but took
  3.5579439 s in candidate scope and did not provide the required current
  plane-certificate parity. A one-QP form took 2.7812950 s and regressed path
  length to 14.1539862749 deg. Both missed the 2.0964864 s wall record; Target
  Exits was not run, and the experiment code was removed.
- **One-shot eight-span Bernstein velocity-energy QP:** rejected at the
  Straight Target runtime gate. The 447-line experiment eliminated 96 control
  scalars to 28 QP variables and reached the common exporter, but the focused
  end-to-end example did not finish within 30 measured seconds versus the
  2.0964864 s record. It was stopped before a valid path or certificate result
  was available; Target Exits was not run, and the hook and helper were removed.
- **Fixed-clock QP witness reuse and active-set solve:** rejected at the
  Straight Target gate. Directly retained constant support planes certified all
  288 output span-region pairs with zero analytic or conic fallbacks in
  0.0554667 s. Switching the same 28-variable QP from interior point
  (7.2986105 s) to MATLAB's active-set algorithm reduced successful seed solves
  to 0.1599676 s, 0.0308900 s, and 0.0235469 s. All five seeds were attempted,
  and the selected motion passed the exact 20.8695652173913 s clock, public
  collision, kinematic, and plane-certificate checks. Its 14.2707707658 deg
  path and 10.0545878 s full wall still missed the 13.678271908 deg and
  2.0964864 s records; topology alone took 2.3570636 s. Target Exits was not
  run, and all probe code and instrumentation were removed.
- **Continuous Bernstein safe-corridor QP:** rejected at its first frozen gate.
  Its 451-line candidate passed the public collision, kinematic, exact-clock,
  and seed-corridor checks on Straight Target, but produced a
  14.1505333646 deg path in 90.488413 s candidate scope versus the
  13.678271908 deg and 2.0964864 s records. Target Exits was not run, and all
  experiment code and artifacts were removed.

## Cross-cutting rejected solver and micro-optimization trials

- First-valid stopping, broad early continuation, average-speed starts,
  tighter arrival tolerance, globally skipped recovery, and larger clearance
  expansion each regressed a maintained arrival, failure, or runtime case.
- Vectorized static corridors, Bernstein and other cache variants, `TypicalX`,
  sparse Jacobians, repeated-query removal, and nargout-sized polynomial
  allocation won only microbenchmarks or missed their end-to-end gates.
- Broad SQP, conjugate-gradient, limited-memory BFGS, PCG, Parallel Computing
  Toolbox, and `parfor` variants timed out, regressed, or established no sound
  end-to-end benefit.

## Per-Seed Work Budget Verification — 2026-08-29

The README fixed-goal protected-rectangle request was rerun on
`422f887+worktree` with `PerSeedWorkBudgetMultiplier=3`. The retained selected
motion is seed 3 with a 7.574541766-second arrival and 11.411861388-degree
motion length. Final wall time was 14.060 seconds. The losing fourth BMTP seed
ended after 4.857 seconds with `seedWorkBudgetExhausted`, rather than an
independent-validation failure. Exclusive final stage timing was 0.4830 seconds
topology, 12.0845 seconds motion solving, 0.3230 seconds collision checking,
0.3292 seconds final validation, and 0.7899 seconds unattributed, totaling
14.0096 seconds. This is one measured static request, not a general runtime or
optimality claim. The full MATLAB suite passed 84/84; the timed and cavity
certificate coverage is direct, while the geometric lower-bound pruning proposal
was deliberately not implemented because a topology seed is not a mandatory
optimized vertex chain.

## Explicit timed-topology policy and conservative moving BMTP — 2026-08-31

The largest current strength is that a changing obstacle no longer causes an
undocumented switch to rest-to-rest waypoint composition. The public
`UnsupportedTimedTopologyPolicy` defaults to `"fail"`; an intentionally
work-limited static-U-plus-mover request returned the earliest accurate
`unsupportedTimedMultiWaypointRoute` reason with zero fallback attempts. The
same request succeeded at 31.4265 s only when
`"ruckigStopAtWaypoints"` was explicit, and diagnostics retained the original
failure plus every forced zero-velocity and zero-acceleration interior state.

Candidate-specific relevance removes the old request-wide static-kernel veto.
A static U with a distant moving obstacle succeeded through static BMTP at
20.8454 s and 39.5987 deg under the default fail policy after full continuous
validation against both obstacles. A structurally different translating
rectangle used the conservative protected-history convex-hull projection and
produced one globally smooth 20 s, 10.7117850149 deg BMTP motion with
0.0657896049 deg minimum clearance against the original moving geometry.

The largest current weakness remains genuine time dependence. The retained
projection is a conservative static swept-history superset; it cannot exploit
an obstacle opening later, couple separating planes to physical-time cells, or
guarantee a wait-plus-detour solution. Those cases still return
`unsupportedTimedMultiWaypointRoute` unless the explicitly labeled
stop-at-waypoints recovery is enabled. This branch therefore demonstrates one
moving-detour family, not a general dynamic BMTP completeness or optimality
result.

Final verification on `f383ae4+worktree` passed 98/98 tests in 69.6771 s.
All 17 maintained examples ran in separate serial headless MATLAB processes:
16 independently validated successes and the expected validated
`exampleNoPath` failure. A visible obstacle-free run also passed. The static-U
sentinel remained 20.712447786 s and the moving-barrier direct-wait sentinel
remained 10.0903015137 s.

## Balanced Selection And One-Sided Exact-Clock Economy — 2026-08-31

The largest current strength is that route choice now represents the stated
gimbal-wear trade rather than using jerk as a preference or choosing arrival
time lexicographically. The default balanced objective is actual motion travel
plus `MinimumTravelSavingsRate_deg_s` times elapsed time; jerk remains a hard
constraint, and normalized kinematic utilization is only a deterministic
tie-break. Static exact-clock detours now enumerate asymmetric, one-sided
progress polynomials whose peak locations are derived from direct-path
collision progress. Every proposal retains the clock-owning coordinate's
physical-limit motion and is accepted only after the unchanged continuous
validator passes.

On the motivating `newheart` bundle, the prior alternating fixed-clock motion
was 201.070948503 deg at the 100.970425693 s physical time floor. The retained
`oneSidedBeta_1_4` motion is 199.268051966 deg at the identical clock, has no
sign reversal relative to the direct chord, and passes continuous collision,
workspace, velocity, acceleration, and jerk checks. This removes
1.802896537 deg of travel. A structurally different near-start rectangle also
selected a one-sided exact-clock basis: 20.493950992 deg versus
20.585690610 deg for the validated alternating family.

The same change improved, rather than traded against, the existing rogue
sentinels: `sinetraj` reached 146.928879089 deg at 70.344250998 s, and balanced
`shrimp` reached 175.703912280 deg at 81.455142283 s. Balanced `non-ideal`
remained 228.491135293 deg at 144 s, while `hiddenruckigfallback` remained a
validated velocity-carried BMTP result with no silent Ruckig substitution.

The largest current weakness is bounded family coverage and runtime. The
one-sided portfolio applies only to static, two-axis, rest-to-rest requests
with one straight-progress coordinate owning the physical clock. It does not
prove globally shortest travel, and alternating/multi-obstacle homotopies may
still require topology BMTP. `newheart` used 154 full validation calls and
65.7788 planner seconds versus the saved 32.7045-second prior run. Runtime is
therefore an explicit regression on that rogue case, retained because
correctness and 1.8029 deg less gimbal travel have higher repository priority.

Final verification passed 110/110 tests in 126.782359 s. All 17 maintained
examples ran serially and headlessly: 16 continuously validated successes and
the expected `noValidatedSeed` result without an example-validation warning.
A visible obstacle-free smoke passed and created two figures. A fresh repeat
of the hidden failure-figure smoke was blocked after the example pass by
MATLAB's environment-level `System Error: File system inconsistency`; the same
worktree's earlier failure-plot check had already created the diagnostic figure,
and no plotting source changed in this final algorithm step.
