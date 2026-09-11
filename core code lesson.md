# Core Code Lesson: Static BMTP Formulation and Runtime

Date: 2026-09-11

## Bottom line

The saved `RogueCasses/x-y-request` was slow because the former static solver formulated too much work: it constrained essentially every curve-region pair, ran a large initial SOCP, then used a nonlinear repair and another SOCP. The obstacle geometry itself was not the main problem, and an arbitrary target such as “four planes” was not a sound design requirement.

The retained static earliest-arrival formulation now follows the BMTP structure directly:

1. Start from the exact visibility-graph route.
2. Allocate a fixed total number of Bezier spans in proportion to route-edge length.
3. Solve the trajectory SOCP with only the currently active separating planes.
4. Detect curve-region pairs intersected by that proposal.
5. Add maximum-margin, degree-one time-varying separating planes only for those pairs.
6. Alternate until a feasible incumbent no longer improves or the bounded iteration limit is reached.
7. Do one fixed-arrival travel-length refinement.
8. Accept nothing until the independent continuous validator certifies collision clearance, dynamics, endpoint state, and C3 continuity.

This is both smaller and more reliable than keeping the old quintic corridor solver as a fallback. The old static-corridor, quintic/fmincon, and corridor-consolidation implementations were removed after the replacement passed the regression set.

## Measured results

All comparisons used identical inputs and the unchanged independent validator. Timings are machine-local and should be compared by ratio, not treated as portable absolute performance.

### Saved x-y request

| Formulation | Valid | Planner time (s) | Arrival (s) | Path length |
|---|---:|---:|---:|---:|
| Former all-pairs quintic/fmincon baseline | yes | 35.18 | 128.438 | 270.919 |
| Retained active-pair BMTP | yes | about 18.0 | 128.854 | 262.475 |

The retained formulation is about 49% faster. Arrival is 0.416 seconds later (0.32%), while path length is about 3.1% shorter. This is the intended trade: runtime reduction has priority when arrival changes only slightly and validation remains unchanged.

The baseline spent about 16.8 seconds in conic optimization and 13.9 seconds in nonlinear optimization. The new path has no nonlinear optimization phase.

### Deterministic random corpus

The corpus contains 33 fixed-seed cases across six structurally different families: convex detours, concave cavities, slaloms, dense scenes, moderate C-shaped cavities, and harder curved cavities.

| Family | Passed | Cases | Retained solver time (s) |
|---|---:|---:|---:|
| Convex | 5 | 5 | 4.11 |
| Cavity | 5 | 5 | 32.59 |
| Slalom | 5 | 5 | 8.03 |
| Dense cavity | 5 | 5 | 35.30 |
| Moderate C | 8 | 8 | 56.14 |
| Curved cavity | 5 | 5 | 66.53 |
| **Total** | **33** | **33** | **202.70** |

The former baseline passed 22 of 33 and consumed 360.32 seconds, including time spent on failed cases. The retained version passes 33 of 33 and reduces aggregate solver time by 43.7%.

The `bmtp-cleanup-codex` branch was correctly suspected to be faster. As written, it took 162.78 seconds but passed 32 of 33; one trajectory reached the optimizer and then failed the exact final certificate. Certificate-directed Bezier subdivision repaired that case in the sandbox. The retained implementation incorporates that correctness behavior, but its 202.70-second aggregate is still 24.5% slower than the cleanup branch measurement. That unfavorable comparison is real and is the clearest remaining optimization target.

## What was learned about planes and convex regions

Plane count is a consequence of the formulation, not a goal by itself.

- A static convex obstacle can require different separators for different curve spans. One global plane may not separate a path that bends around the obstacle.
- Many plotted planes were redundant because the old formulation generated them for pairs that never became relevant. Collision-driven pair activation removes this source of clutter and cost.
- A degree-one time-varying plane is useful around a corner because its normal and offset can rotate over a span. This consolidates what would otherwise require several constant planes without weakening geometry.
- Maximum-margin plane solves were necessary on the harder C-shaped cases. The cheaper analytic plane update failed cases that the maximum-margin formulation solved.
- Exact merged convex obstacle regions performed better than forcing raw triangulation. On the saved x-y case, triangulation expanded the input from 9 merged regions to 56 triangles and increased plane work. Keeping exact merged convex regions reduced both model size and bookkeeping.
- Four hand-selected planes were tested as a hypothesis and were not sufficient as a general formulation. No fixture-specific plane layout was retained.

The BMTP paper describes the same essential pattern: alternate trajectory and maximum-margin hyperplane subproblems, introduce collision constraints only for colliding obstacle-trajectory pairs, and certify continuous-time behavior using Bezier control bounds. The project implementation and examples also use degree-eight curves. See [Bounded-Magnitude Trajectory Planning](https://arxiv.org/abs/2608.02834), the [BMTP project page](https://wernerpe.github.io/bmtp-website/), and the [reference implementation](https://github.com/wernerpe/pybmtp).

## Why segmenting the solve is not automatically faster

Independent path chunks look smaller, but position, velocity, acceleration, jerk continuity, and the shared arrival objective couple adjacent chunks. Solving chunks independently would either duplicate boundary-state search or silently lose global feasibility. More spans also increase trajectory variables and the number of possible curve-region pairs.

The useful decomposition is therefore limited and deterministic:

- keep one coupled trajectory SOCP;
- preserve a fixed total mesh budget;
- distribute that budget by route-edge length so long edges do not receive the same resolution as tiny corner edges;
- subdivide only when the exact certificate shows that a safe curved span cannot yet be certified by one affine separator.

This distinction mattered on the saved x-y request. With length-balanced allocation, it returned a validated result in about 18 seconds. Disabling only that allocation made the same request terminate without an optimized feasible iterate after about 9.7 seconds. The helper is therefore live correctness/conditioning code, not cosmetic mesh cleanup.

Other mesh experiments were unfavorable: a substantially reduced span count failed, while a larger mesh could validate but increased runtime and produced a longer path. Neither was retained.

## Numerical and certification lessons

- Solver output is a proposal, never proof. Sampled overlap detection is allowed to discover active pairs cheaply, but success still requires the public continuous validator.
- Exact certificate failure can come from representation, not physical collision. Selective de Casteljau subdivision exposes a curved span's clearance without changing the curve, obstacle, margin, or tolerance.
- Degree-eight endpoint stabilization must share position through jerk. Sharing only through acceleration allowed post-conversion C3 failures.
- Endpoint/continuity stabilization can slightly increase sufficient derivative bounds. Earliest-arrival motions are uniformly time-dilated only by the exact required factor before independent validation; fixed-arrival requests are not given extra time.
- Minimizing travel length inside every alternating iteration approximately doubles unnecessary SOCP work. Alternation now optimizes arrival only, followed by one fixed-arrival length refinement of the retained candidate.
- Explicit `coneprog` linear-solver choices (`normal`, `prodchol`, and `schur`) were all slower on the saved request than MATLAB's automatic augmented-system choice. No solver-option override was retained. MATLAB documents the conic solver and its algorithm at [coneprog](https://www.mathworks.com/help/optim/ug/coneprog.html) and [Cone Programming Algorithm](https://www.mathworks.com/help/optim/ug/cone-programming-algorithm.html).
- The old nonlinear phase repeatedly reached its iteration cap and dominated runtime. General advice for diagnosing slow Optimization Toolbox solves is consistent with profiling the formulation before tuning options; see [When the Solver Takes Too Long](https://www.mathworks.com/help/optim/ug/solver-takes-too-long.html) and [fmincon](https://www.mathworks.com/help/optim/ug/fmincon.html).

## Final benchmark and cleanup lesson

The complete post-cleanup audit is recorded in
[`benchmarks/core_benchmark_2026-09-11.md`](benchmarks/core_benchmark_2026-09-11.md).
All 20 maintained examples passed over three repetitions, all 160 paired random
azimuth cases passed, and all 57 MATLAB tests passed.

The slowest maintained example showed why optimization has to preserve the
representation being checked. Travel refinement already called
`prepareFinalMotion` and continuously certified the corrected, subdivided, and
possibly time-dilated curve. Re-preparing the raw controls immediately afterward
discarded that work and repeated the certificate. Passing that exact prepared
motion and its certificate forward reduced the hard-example median from 46.2960
to 42.8925 seconds while preserving duration and continuous arc length. The
independent public validator remains the final authority.

By contrast, removing unused-looking length-cone variables from time-only SOCPs
changed the numerical problem layout and therefore the active-pair iteration.
It made the alternating slalom slower and measurably lengthened both inspected
paths. That experiment had also been rejected earlier; it was restored rather
than rationalized after the fact. Algebraic redundancy is not sufficient
evidence for changing an iterative conic formulation.

Dead code also needs historical context. The removed Ruckig and clock-guide code
implemented genuine alternate algorithms; BMTP does not replace every standalone
capability they once exposed. They were removed because the current repository
contract has one BMTP planner entry point and repository dependency analysis
found no callers. `PathLengthTimeAllowance_s` had likewise been meaningful, but
its length-delay and jerk-penalty consumers were already gone. Removing its
default and validation makes the active interface honest; old callers now get
the existing unknown-option warning instead of a silently ineffective control.

The next runtime regression was not inside `coneprog`; it was ownership of when
to call it. Two dynamic waiting examples already had an independently certified
analytic delayed chord, but chronological improvement search then launched 176
and 246 trajectory SOCPs respectively, failed every earlier arrival, and returned
the original candidate. An exact initial visibility route plus the optimistic
velocity lower bound `route length / norm(max velocity)` now decides whether an
initial spatial detour could beat the incumbent before those trials run. When it
cannot, the certified wait is returned immediately. When it can, as in the
moving-circle control case, the original chronological search remains active.

This small ownership change reduced the complete maintained-example sum of
medians from 128.8134 to 96.4458 seconds without changing any returned duration
or length. The lesson is broader than this case: before tuning a solver called
hundreds of times, verify that the caller still needs those solves and already
has not computed a validated incumbent that dominates the search family under a
cheap necessary bound.

## Code-retention rule

A suggestion is a hypothesis, not a specification. Core code is retained only when all of the following are true:

- it solves a demonstrated correctness or performance problem;
- it passes the unchanged independent validator;
- it improves identical-input measurements or is required for a regression;
- it applies structurally beyond one fixture;
- it has one clear production call path;
- any superseded implementation, fallback, diagnostic experiment, and temporary output is removed.

This rule is why the final change removes the older static solver instead of keeping both implementations “just in case.” It is also why raw triangle decomposition, four-plane layouts, explicit cone-solver modes, route-point canonicalization, reduced meshes, and horizon retry schedules are absent from production.

## Why the remaining static examples are slower

A focused profile of `exampleUSOutlineExtremeVisibility` attributes the current
47.696-second profiled wall time as follows. These are inclusive timings and
must not be summed across callers:

| Owner | Inclusive time (s) | Calls |
|---|---:|---:|
| Active-pair BMTP | 32.224 | 3 |
| `coneprog` | 24.063 | 2,029 |
| Trajectory steps | 17.100 | 38 |
| Maximum-margin plane steps | 9.507 | 1,991 |
| Final motion certification | 4.889 | 6 |
| Exact visibility graphs | 4.892 | 3 |
| Geographic fixture construction | 5.285 | 3 |

Certificate reuse has already reduced final checking enough that validation is
not the principal remaining owner. The expensive work is inside the conic
solver: both the number of alternating programs and their numerical solution.

The exact Hawaii, Croatia, and Philippines outlines decompose into 278, 340,
and 2,193 convex regions. Only 46, 44, and 322 span-region pairs are eventually
tagged, but a collision-free improvement changes the curve-dependent rows of
every tagged maximum-margin problem. The solver therefore refreshes the full
tagged set before the next trajectory step. This produces 305, 220, and 1,466
plane SOCPs respectively. Those refreshes are optimization work, not duplicate
certification.

The `bmtp-cleanup-codex` branch completed the identical geographic example in
21.888 profiled seconds with 185 conic calls. Its advantage is not generic code
cleanliness: when an outline has more than 64 exact convex regions, that branch
replaces them with eight conservative convex hulls and retains an exact-region
retry. It also routes through older fixed-time excursion and candidate-search
paths. The hulls change the occupied set and the fallback changes the solve
policy, so this is not an implementation-equivalent comparison under the
current exact-geometry, one-method, no-retry contract.

Several targeted experiments explain why the expensive settings remain:

- Longest-shared-edge-first exact face merging increased the region counts to
  299, 343, and 2,216. The default deterministic merge order is better for
  these inputs.
- Replacing maximum-margin planes with analytic supporting axes kept static U
  valid but worsened arrival from 20.8452 to 20.9236 seconds, worsened length
  from 39.3457 to 39.4043 units, and did not reduce wall time. The rotating
  degree-one maximum-margin plane is materially different from a cheap constant
  separator.
- Refreshing only planes with trajectory dual weight above the conic optimality
  tolerance reduced static-U plane solves from 350 to 100. On the Philippines
  outline it reduced plane solves from 1,466 to 403 and total geographic wall
  time from about 42.9 to 36.7 seconds, but arrival regressed from 5.2642 to
  5.9417 seconds and length from 18.8320 to 19.4574 units. A plane that is
  inactive for the current trajectory can constrain the next optimum after
  neighboring planes move.
- Stopping on a coarser arrival plateau and omitting the terminal plane refresh
  saved iterations but changed the final length-refinement corridor. The dense
  outline and static U returned different, sometimes longer paths. The final
  refresh is live optimization input rather than dead end-of-loop work.
- Deferring construction of length-cone objects unused by arrival-only calls
  preserved solver inputs and exact outputs. It reduced trajectory-step profile
  self-time by only 0.089 seconds across 38 calls; geographic wall time changed
  from 47.696 to 47.881 profiled seconds. The setup-only saving is below run
  noise and does not justify another branch in the shared solver.
- An exact-input trace over all 1,991 geographic plane solves found zero
  duplicates when comparing the curve controls, convex-region vertices,
  separation target, and reserve. Repeated span-region identifiers therefore do
  not provide a valid memoization opportunity: each accepted feasible curve
  changes the numerical maximum-margin problem.
- The official Python implementation describes its plane update as an LP, but
  its exact polyhedral formulation also bounds each plane normal with a Lorentz
  cone. It is consequently an SOCP like this implementation. Its important
  execution difference is parallel solution of independent plane programs.
  This MATLAB installation has no process-based Parallel Computing Toolbox, and
  `coneprog` explicitly rejects execution on the available six-worker
  thread-based background pool. Parallel plane solves are therefore not an
  available dependency-free optimization here.
- An axis-aligned shortcut in final certification preserved every maintained
  motion exactly, but a complete three-run example sweep became slightly slower
  overall. It was removed rather than retained on an isolated-case timing win.

## Exact transient plane consolidation

The overlap exists in the trajectory formulation, not in the plane-update
inputs. A degree-one target plane block is implied by one retained block when
the same nonnegative scale relates both endpoint normals and the retained
offset plus arrival reserve dominates at both endpoints. Any floating normal
residual is bounded over the existing workspace box. Because the Bernstein
product is linear, that one proof implies all ten degree-eight product rows.

A diagnostic over the geographic example found 1,002 removable blocks among
4,173 arrival-plane appearances with the one-source proof. It cost 1.29 seconds.
A two-source proof found 1,627 but cost 15.44 seconds, so it was rejected. Static
U had no removable block among 349 arrival-plane appearances.

Production consolidation is deliberately narrower than the proof permits. It
runs only while collision discovery has not yet produced a feasible iterate,
and only when plane rows outnumber the trajectory's control-polygon edges. The
complete plane array and tagged-pair set remain upstream, every scheduled
maximum-margin plane is still recomputed, and all planes return for feasible
improvement and final length refinement. This changes neither exact geometry
nor the mathematical pre-feasibility corridor, although removing redundant
rows can change the finite-precision solution selected by `coneprog`.

On the complete 20-example, three-repetition benchmark, all expected outcomes
remained independently valid. The sum of median wall times fell from 96.446 to
91.933 seconds and the maximum median fell from 41.949 to 36.588 seconds. The
only maintained motion change was the Philippines path, from 18.83204 to
18.84937 units (+0.092%); its arrival changed by about one microsecond in the
faster direction. The 160 deterministic random azimuth cases remained 160/160
valid, but those fixed-arrival dynamic requests do not exercise the static
active-pair consolidation path.

Restoring the complete corridor after first feasibility does not bound the
eventual path perturbation. Removing implied rows can change the finite-precision
first feasible point selected by `coneprog`; that changes later maximum-margin
planes and the local optimum. Runtime is the stated priority, but every observed
motion-quality change must therefore be reported rather than assumed small.

## Held-out static random corpora

The retained reproducer is `benchmarks/benchmarkRandomStaticBmtp.m`; it records
compact validation, motion, graph, BMTP, conic, and removal diagnostics without
retaining full result objects.

The first three fixed-seed corpora were useful negative evidence because they
showed that region count alone does not create a consolidation opportunity:

- seed 20260911: 21 cases comprising six cavities, six separated slaloms, six
  star outlines, and three direct controls. All 18 active BMTP cases and all
  three controls validated, but tagged-pair count stayed at or below 22;
- seed 20260912: eight clouds of 24--36 overlapping rectangles. All validated,
  with 58--109 tagged pairs and 9--21 optimizer spans, but the production
  plane-density guard was not crossed;
- seed 20260913: eight serrated bands with 45--75 teeth, 123--197 exact convex
  regions, 31--46 tagged pairs, and 15 optimizer spans. All validated and the
  transient removal count was zero.

Seed 20260914 deliberately scaled the same structural family to four bands with
250--350 teeth. These cases contained 514, 643, 527, and 626 exact regions and
finally exercised the production proof. Three independent repetitions per
version produced 12 runs per version, 24 total; all succeeded and passed the
public validator.

| Case | Baseline median (s) | Consolidated median (s) | Wall change | Length change | Arrival change | Cumulative removed block appearances |
|---|---:|---:|---:|---:|---:|---:|
| highDensity 1 | 18.5471 | 18.5369 | -0.055% | +0.027735% | +0.160861 us | 100 |
| highDensity 2 | 17.5999 | 17.5439 | -0.318% | -0.002366% | -0.004931 us | 288 |
| highDensity 3 | 14.8474 | 13.9985 | -5.717% | +0.474115% | -0.000114 us | 236 |
| highDensity 4 | 20.7135 | 21.0344 | +1.549% | -0.005380% | -0.001847 us | 134 |

The median sums changed from 71.7079 to 71.1138 seconds, only -0.828%.
Removal count did not predict speedup, and the largest gain accompanied the
largest length increase. This held-out result supports keeping the already
narrow proof because the maintained dense geographic case benefits, but it does
not support a general static-planner speedup claim, a new density threshold, or
a bound on path-quality change. The diagnostic counts cumulative appearances of
removed blocks across trajectory solves, not unique planes or avoided plane
SOCPs.

A corrected one-repetition stage attribution found identical iteration,
trajectory-SOCP, plane-SOCP, and total conic-call counts between versions. The
aggregate BMTP time changed by -0.519% and raw `coneprog` time by -0.912%.
Case 3's wall/BMTP/conic changes were -6.065%, -9.218%, and -10.175%, while
case 4's were +3.266%, +4.436%, and +4.529%. Thus the mixed behavior belongs to
the numerical cost of the changed trajectory programs, not extra retries.

Benchmark isolation mattered. The first follow-up baseline attribution was
launched from the production checkout; MATLAB's current-folder precedence
overrode the detached baseline on the path, and the supposed baseline reported
the current-only removal counter. That run was interrupted and discarded. The
driver now changes to the requested planner root before warming or measuring.
The retained three-repetition baseline files were separately checked to contain
zero removals; the corrected stage run also resolved the old implementation.

## Exact visibility-graph relevance filtering

After consolidation, a focused dense-static profile still assigned about five
seconds to the exact visibility graph. On highDensity 3, an unprofiled graph
took 4.864 seconds for 1,308 nodes and 129,315 collision queries. The profiled
graph took 5.088 seconds inclusive: `segmentIntervals` owned 3.960 seconds,
`inpolygon` 0.558 seconds, and endpoint-cone rejection 0.353 seconds. The hot
lines were the dense edge-by-candidate outer products, divisions, and logical
masks. Shortest-path search, polygon union, and triangulation were negligible.

A six-size fixed-seed microbenchmark confirmed the scaling mechanism. Before
the change, log-log runtime slope was 2.059 against node count and 1.080 against
the number of collision queries.

| Teeth | Nodes | Queries | Before median (s) | Relevant-edge median (s) | Change |
|---:|---:|---:|---:|---:|---:|
| 40 | 246 | 5,535 | 0.1678 | 0.1630 | -2.8% |
| 80 | 486 | 21,062 | 0.4903 | 0.4026 | -17.9% |
| 120 | 726 | 46,937 | 1.0637 | 0.9192 | -13.6% |
| 180 | 846 | 55,193 | 1.4066 | 0.9701 | -31.0% |
| 250 | 1,325 | 132,646 | 4.7139 | 2.6456 | -43.9% |
| 320 | 1,489 | 180,776 | 6.9528 | 3.5729 | -48.6% |

The sum of these medians fell from 14.7952 to 8.6735 seconds (-41.4%). The
implementation still enumerates every node pair and retains the per-candidate
AABB mask. It only omits cross-product arithmetic for a boundary edge when that
edge's AABB is disjoint from every candidate segment in the current bounded
batch. Such an edge cannot intersect any candidate in the batch, so this is an
exact work reduction rather than route pruning. The complete highDensity 3
graph was bit-for-bit equal before and after, including every accepted and
rejected edge.

Increasing the existing temporary-array budget was tested and rejected. On the
same dense graph, median times for budgets 2^17, 2^18, 2^19, and 2^20 elements
were 4.8037, 4.6788, 4.5338, and 4.6182 seconds. Although 2^19 saved 2.4% across
the six sizes, one size regressed 1.6%, the effect was hardware-sensitive, and
it doubled the intended temporary matrix budget. Production retains 2^18.

At whole-planner level, three repetitions of the four high-density cases with
both accepted optimizations reduced the median sum from the original 71.7079
seconds to 58.4761 seconds (-18.45%). Isolating the visibility change against
the already consolidated version gives 71.1138 to 58.4761 seconds (-17.77%),
with per-case reductions of 19.33%, 21.28%, 15.02%, and 15.30%. All 12 filtered
runs passed independent validation, and every arrival time and path length was
numerically unchanged from the same consolidated inputs. The complete MATLAB
suite passed 60/60 after the change.

The complete 20-example, three-repetition rerun was 20/20 valid and preserved
all maintained durations and lengths. Its median-time sum was 91.9071 seconds
versus 91.9332 before filtering (-0.03%), with a 36.5722-second maximum. This is
correctly treated as neutral: the maintained suite contains little of the
high-node-count geometry targeted by the filter, and solver variability hides
its small absolute graph saving in the geographic example. The 160-case random
azimuth rerun also remained 160/160 valid, with median, mean, and maximum wall
times of 0.1542, 0.1696, and 0.8472 seconds.

## Trajectory-SOCP formulation experiments after graph filtering

The post-filter profile moved the optimization target decisively into BMTP. On
the slow US-outline example, the planner spent about 34.9 seconds in planning,
27.8 seconds in BMTP, and 20.4 seconds in 1,609 `coneprog` calls. Trajectory
programs accounted for about 15.1 seconds and maximum-margin plane programs for
7.6 seconds, while the now-filtered visibility graph accounted for about 4.5
seconds. This means another graph-only change cannot provide a large whole-case
gain: most remaining time is the required alternating conic work.

One exact bookkeeping change was retained. The plane table is rectangular, but
most entries are inactive; the trajectory builder formerly visited all 442,296
slots in the profiled run merely to reject the inactive ones. It now computes
the same active mask once and iterates the active region indices in the same
ascending order. The rows, offsets, solver options, and row ordering are
unchanged. The 0.615-second measured slot-visitation hotspot disappeared and
inclusive trajectory-step time fell from 15.145 to 14.474 seconds in the focused
profile, while `solveConic` time remained within noise.

Three repetitions of the four high-density random static cases remained 12/12
valid and preserved every arrival time and path length. Their median wall times
were 14.896, 13.647, 11.780, and 17.798 seconds; the sum was 58.1210 seconds
versus 58.4761 before this loop change (-0.61%). A separate three-repetition
focused US run had a 35.8386-second median versus the prior 36.5722-second
median (-2.01%), with the same 5.264174121-second arrival and
18.849365277-unit length. The complete 20-example rerun was 20/20 valid with
unchanged motion metrics, but its median-time sum was an unfavorable 99.1605
seconds versus 91.9071; the slow US case alone measured 37.6424 seconds in that
run. The direct profile and dense corpus support retaining the exact low-level
work reduction, but the suite result shows that its small benefit is easily
overwhelmed by conic-solver timing variability.

Several larger-looking formulations were tested and rejected:

- Batching 137 independent maximum-margin plane updates into one exact
  block-diagonal SOCP increased median time from about 0.63 seconds for the
  scalar solves to about 1.89 seconds, roughly 200% slower. The global solver
  returned exit flag -7 even though all 137 recovered planes passed their
  per-plane checks, and the selected non-unique normals differed. Grouping only
  by optimizer span was still 0.98% slower and verified only 134 of 137 planes;
  all four nonempty batch solves returned -7. Independent scalar plane solves
  remain both faster and more reliable.
- Reassembling all active plane rows through another sparse staging layer was
  not implemented. After inactive-slot removal, row creation and insertion were
  only about 0.25 seconds of the roughly 34-second profile, below 1%; another
  representation would add complexity with no material whole-run ceiling.
- An exact trace of 37 trajectory programs showed two different cost regimes.
  For nine-span programs, solve time tracked inequality-row count strongly
  (correlation 0.960 within the nonfixed group). For 27-span programs, row count
  did not explain time, while interior-point iterations did (correlation 0.984).
  Across all calls, elapsed time correlated 0.802 with inequality rows and only
  0.182 with active-plane count. Thus “fewer planes” is not itself a sufficient
  optimization target; the induced program dimensions and conditioning matter.
- Positive row normalization was tested on eight captured trajectory programs
  with three counterbalanced repetitions per variant and all residuals checked
  in the original unscaled formulation. The unscaled median was 0.4639 seconds
  at 33 iterations. Equality-only scaling was 0.4741 seconds at 34 iterations,
  inequality-only scaling was 0.6010 seconds at 44 iterations, and scaling both
  was 0.6180 seconds at 45.5 iterations. Row scaling preserves the feasible set
  mathematically but worsened this solver's numerical path, so it was rejected.

These results narrow the credible remaining opportunity. Small exact MATLAB-side
reductions can still recover low-single-digit percentages, but a further
double-digit improvement must reduce the intrinsic trajectory/plane conic work
or improve its conditioning without changing the accepted motion, tolerances,
or independent validation.

## Remaining limitations

- BMTP alternation is biconvex and does not guarantee a globally shortest or globally minimum-time trajectory.
- Sampled collisions drive constraint discovery. The independent exact validator prevents false success, but a missed pair can still produce an expected failure instead of a solution.
- The maintained examples and randomized corpus are deterministic and structurally varied, but remain synthetic 2-D evidence rather than a proof over all scenes.
- Hard curved cavities remain the dominant runtime family.
- The cleanup branch remains faster on the dense outline, so further work should profile mathematically equivalent trajectory and plane SOCP solution on hard curved cavities. Any future optimization must preserve the current maintained and randomized validation results.

## Sources

- [Bounded-Magnitude Trajectory Planning (primary paper)](https://arxiv.org/abs/2608.02834)
- [BMTP project page](https://wernerpe.github.io/bmtp-website/)
- [Official pybmtp reference implementation](https://github.com/wernerpe/pybmtp)
- [Safe Flight Corridors with Bezier trajectory optimization](https://arxiv.org/abs/2310.01190)
- [MathWorks `coneprog`](https://www.mathworks.com/help/optim/ug/coneprog.html)
- [MathWorks cone programming algorithm](https://www.mathworks.com/help/optim/ug/cone-programming-algorithm.html)
- [MathWorks `fmincon`](https://www.mathworks.com/help/optim/ug/fmincon.html)
- [MathWorks: When the Solver Takes Too Long](https://www.mathworks.com/help/optim/ug/solver-takes-too-long.html)
