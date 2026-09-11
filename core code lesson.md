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
valid. This is the accepted runtime/length trade: the user prioritized runtime,
while the complete corridor after first feasibility limits the path change.

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
