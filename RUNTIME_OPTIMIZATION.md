# Runtime optimization after C3 quintic

## Diagnosis

MATLAB profiling found three owners of repeated large computations rather than
one shared hot function. Profiling was used for attribution only; final timing
runs have profiling and display disabled.

| Case | Profiled wall (s) | Main owner |
| --- | ---: | --- |
| Moving/deforming US | 81.15 | Visibility: 36.64 s, including 574 expanded nodes |
| Static U | 60.07 | Nonlinear optimization: 50.58 s; constraint-system factorization: 24.05 s |
| Geographic sequence | 64.94 | Repeated final collision certification: 24.38 s across 18 passes |

Inclusive timers overlap and must not be added. The geographic wall time
includes Hawaii, Croatia, and Philippines.

## One approach applied to the three owners

Avoid recomputing or coupling work whose answer is already determined exactly:

- Visibility keeps optimistic parent edges and verifies an edge only when its
  child reaches the front of A*. A blocked edge raises that child's lower bound
  and returns it to the frontier. Only closed nodes can supply a parent cost;
  the Euclidean heuristic remains admissible. Every returned route edge has
  passed the original contact-partition and polygon checks. Filled triangles
  identify the occupied side of each boundary edge, including hole boundaries.
  Directions strictly entering the occupied vertex cone are rejected before
  the full predicate. Touching, nearly flat, and uncertain vertices use the
  original full predicate.
- Variable-time optimization uses explicit shared knot position, velocity, and
  acceleration states, shared endpoint jerk, and local continuity equations.
  The same integrated quadratic jerk generates quintic position. This replaces
  dense cumulative sensitivities with sparse local derivatives. The physical
  feasible set, objective, 200-iteration budget, and numerical tolerances remain.
  The static-U inequality Jacobian drops from 138,165 to 9,786 nonzeros. The
  local model has 304 variables versus 196; locality, not fewer variables,
  reduces its factorization cost.
- Refinement reuses a complete span's collision certificates only for identical
  controls, geometry, coverage, and clearance settings. Moving geometry also
  requires identical absolute span times. Dynamics checks and the final public
  independent validator still run. The cache lives only inside the current solve.

The visibility mechanism was checked against ordinary shortest-path search on
300 randomized abstract graphs and against MATLAB's original implementation on
the four dense geographic scenes. The latter returned exactly identical route
coordinates and lengths. Moving-US edge checks fell from 326,753 to 60,745;
that isolated graph calculation took 28.12 s before and 16.66 s after.
Adding occupied-cone rejection reduced full collision queries to 1,576. In a
separate paired measurement, the original graph took 32.38 s and the final
graph 2.98 s, again with identical route coordinates and length. The final
Hawaii, Croatia, and Philippines graph calculations took 0.386, 0.013, and
0.445 s respectively, versus 3.193, 0.017, and 1.941 s.

The remaining moving-US cost is not entirely in the planner. Before adding
the cone rejection, its updated profile measured 60.28 s end to end and
33.35 s inside `planner`: about 26.93 s was scenario construction outside the
planner. Visibility still consumed 21.38 s inside that profile. The example
constructs, protects, and prepares the full dense obstacle history, even
though the returned motion arrives at 8.5 s. That source history is unchanged.

## Experiments not retained as the algorithmic fix

MATLAB defaulted to six computational threads. Three baseline repetitions at
six threads were noisy: medians were 82.42, 83.89, and 71.68 seconds respectively.
One thread reduced these to 56.30, 43.71, and 49.19 seconds, but also changed the
static U's local optimum slightly. Thread count is therefore controlled in the
algorithm comparison; the planner does not alter the session thread setting.

Sharing jerk variables alone did not materially reduce static-U wall time
(43.84 s in the trial). Explicit local knot states were needed. Batching 32
optimistic visibility parents checked too many premature edges and was slower
(33.00 s versus 30.04 s for the original moving-US graph), so it was rejected.

## Verification and measured results

All timing comparisons below use identical scenario inputs, one computational
thread, three repetitions, and a cleared planner graph cache before each run.
The constructor, planner, and example-level validation are inside each timer;
additional independent checks and evidence serialization are outside it.

**All 91 regression tests passed, and all 18 unchanged examples passed their
independent scenario validation, including all three geographic subcases.**

| Case | Before median (s) | After median (s) | Speedup | Before range (s) | After range (s) |
| --- | ---: | ---: | ---: | --- | --- |
| Moving/deforming US | 56.301 | 31.273 | 1.80x | 55.345–58.503 | 30.145–34.174 |
| Static U | 43.713 | 12.881 | 3.39x | 41.896–49.357 | 12.678–13.097 |
| Geographic sequence | 49.189 | 30.992 | 1.59x | 47.969–53.737 | 30.242–32.001 |

For the final moving-US runs, median time inside `planner` is **11.99 s**.
Median time outside it, including scenario construction and its extra checks,
is **18.99 s**. These are separately calculated medians; their sum need not
equal the 31.27 s median of whole-example time. The full history remains part
of the benchmark, so accelerating visibility does not eliminate setup cost.

The moving-US duration remains 8.5 s and length remains 40.298297 units. All
three geographic subcases retain their duration and length at the reported
precision. Static-U duration changes from 20.773353 to 20.758067 s and length
from 38.468378 to 39.178563 units: 0.0153 s earlier, with a 1.85% longer path.
That length exceeds the unchanged historical target of 38.678082 units.
Consequently, historical quality-gate passes fall from 10/18 to 9/18; 7/18
still meet all historical gates. Physical validation and its tolerances were
not weakened. The runtime improvement includes this explicitly reported
length tradeoff rather than claiming preserved optimization quality.
These fresh baselines differ from the older C3 table because thread settings,
machine load, and measurement conditions affect runtime; the old table is
not used to calculate the speedups above.

The knot-state Jacobian and exact Hessian passed four finite-difference probes:
maximum relative errors were 9.84e-10 and 1.72e-10, respectively.

The nonlinear solve remains local and can exhaust its iteration budget. The
static U arrived approximately 0.0153 s earlier but followed a longer path;
this is a motion-quality tradeoff, not an unchanged-motion speed comparison.

Reproduce controlled thread experiments with `comparePlannerThreads`, which
alternates conditions and restores the session's previous thread count. Raw
measurements, profiles, and audit scripts remain in ignored `scratch/` and
`benchmarks/` directories.
The before/after timing records are `scratch/slow_one_thread.csv` and
`scratch/slow_final.csv`. The final regression and example results are
`scratch/runtime_cones_tests.mat` and `scratch/runtime_final_examples.csv`.

## Compact plane constraints and bounded path-length refinement (2026-09-09)

This comparison starts from `3c63062`, with the MATLAB path-isolation fix in
both versions. It uses MATLAB R2024b, Optimization Toolbox, the session's six
computational threads, identical default example inputs, one warmup, and three
measured repetitions per example. These are new baselines; the earlier
one-thread measurements above do not measure this change. Profiling was run
separately. Timed examples include construction, planning, and example-level
validation, with graphics and profiling disabled.

The remaining expensive work has different causes. Vietnam's fixed-clock
conic problem has 120 phases and 960 active plane/phase pairs. Static U has
only three exact convex regions; nonlinear timing still exhausts its existing
200-iteration budget. Moving US spends substantial time constructing and
preparing its dense polygon history. Philippines spends time trying an extra
arrival-time allowance whose required path-length improvement is impossible
given its exact static visibility route.

The retained changes address those costs without changing obstacle geometry:

- Before fixed-clock optimization, remove a constant-normal affine plane only
  when a nonnegative combination of retained plane/workspace normals proves
  it redundant at both offset endpoints. Bound normal reconstruction error
  over the workspace and keep uncertain planes. Removal is sequential, so
  proofs cannot depend circularly on other removed planes. Preprocessing is
  bounded to local sets of 3 through 64 planes. The helper is shared by the
  fixed-clock solver and the final fixed-clock quintic repair; initial time
  optimization stays unchanged.
- For fixed-clock formulations with more plane pairs than length-cone
  variables, use one elastic clearance variable per phase, weighted by its
  retained plane count. Smaller problems retain the original formulation.
  Zero slack has the same hard corridor; the elastic search objective changes.
  All original regions still participate in final collision checks and public
  independent validation. Vietnam retains 417 of 960 plane pairs; alternating
  occlusion retains 185 of 864.
- Skip the optional extra-time length trial only when the exact static
  visibility route, with a numerical guard, rules out its existing required
  one-percent gain. The first length polish at the earliest clock still runs.
  Dynamic snapshot routes are not used as geometric lower bounds. At the time of
  this historical experiment, examples forwarded an explicit
  `PathLengthTimeAllowance_s` override; that option and its consumers have since
  been removed from the active core.
- Return immediately when obstacle rings match exactly, and use a bounded
  direct index comparison when merging short convex faces. Ring alignment,
  convex regions, source geometry, margins, and collision predicates retain
  their original meaning and deterministic ordering.

Only one shared production helper was added. No engine dependency, validation
tolerance change, relaxed geometry, or new public planner entry point is needed.

Median times in seconds:

| Case | Planner before | Planner after | Whole example before | Whole example after |
| --- | ---: | ---: | ---: | ---: |
| Vietnam | 6.381 | 4.966 | 6.789 | 5.372 |
| Static U | 12.123 | 11.706 | 12.163 | 11.732 |
| Moving/deforming US | 6.992 | 6.125 | 21.063 | 20.263 |
| Geographic sequence | — | — | 26.831 | 24.845 |
| Philippines within sequence | 14.741 | 13.101 | — | — |
| Alternating occlusion | 2.999 | 1.483 | 3.078 | 1.558 |
| Alternating slalom | 1.158 | 0.846 | 1.214 | 0.876 |
| Dense concave obstacle | 1.211 | 1.001 | 1.252 | 1.037 |
| Opposing U obstacles | 1.396 | 1.196 | 1.425 | 1.225 |

All 18 examples, including the three geographic subcases, kept their success
or expected no-path outcome. Every feasible output passed public independent
validation. Arrival times were unchanged. Static U became 0.00000366 units
shorter; alternating occlusion became 0.000000128 units longer (about 9.4e-9
relative); Vietnam shortened by 3.52e-9 units. Other lengths were unchanged.
Those tiny differences are numerical solve variation, not a newly accepted
arrival/path-length trade. Physical validation tolerances were not changed.
The original `straightpathtaketoolong` and `pathtoolong` inputs also preserved
arrival and length exactly; their median planner-call times were 0.059 to
0.062 seconds and 2.953 to 2.691 seconds, respectively.

### Effect on oscillatory kinematics

Consolidation reduces solver work but does not consistently smooth motion.
The final combined change gives these integrated squared-jerk values:

| Case | Before | After |
| --- | ---: | ---: |
| Vietnam | 151.354 | 152.369 |
| Static U | 14.284 | 14.204 |
| Alternating occlusion | 12.665 | 10.643 |

Vietnam's normalized total jerk variation changes from [27.470, 25.209] to
[27.659, 25.358], so its large oscillations remain and slightly increase.
Alternating occlusion improves from [12.861, 5.757] to [9.792, 4.955]. Static U
is essentially unchanged at [13.381, 11.093]. The jerk comparison plot is
`scratch/holistic/jerk_comparison.png`. C3 continuity and jerk bounds permit
oscillatory motion; changing redundant constraints does not add a smoothness
objective. Treating smoothness as another objective would require a separate
quality/runtime comparison under the arrival-first, length-second policy.

Static U and Philippines remain above five seconds; moving-US construction
still dominates whole-example time. This change reduces those costs without
claiming a global timing optimum. The nonlinear solve can still reach its
iteration limit, and final physical certification, rather than the optimizer
exit flag alone, determines success. Changing the nonlinear subproblem to
factorization was tested and rejected: static U slowed from 12.11 to 13.73
seconds with the same returned motion.

Raw final measurements and comparisons are in ignored
`scratch/holistic/final_baseline.{mat,json}`,
`scratch/holistic/final_candidate_results.{mat,json}`,
`scratch/holistic/final_comparison.{md,json}`, and
`scratch/holistic/final_rogue_pair.{mat,json}`.

The runtime candidate passed all 125 MATLAB tests, including stale-package
graphical validation, independent linear-program checks of removed planes,
moving intervals, endpoint states, direct motion, detours, invalid inputs, and
expected no-path cases. Eight counterbalanced measured pairs resolved the
smaller timing fluctuations: moving circle median whole-example time was
0.3442 to 0.3420 seconds; four accelerating circles 3.4871 to 3.5198 seconds
(median paired difference +0.0056 seconds); target exits 1.2376 to 1.2108
seconds. Every follow-up motion had identical arrival and length. Both versions
show run-to-run variability; this does not establish exact runtime equality.
Records are `scratch/holistic/timing_followup.{json,md}` and
`scratch/holistic/final_tests.mat`.

### Intrinsic generation follow-up

The subsequent generation change reduces artificial phase subdivision and adds
bounded jerk-variation objectives inside the original conic/nonlinear solves.
It uses physical local derivative variables and an exact integration template.
This avoids introducing a separate output smoother. Relative to the runtime
candidate above, median planner time changes from 4.966 to 2.916 seconds for
Vietnam and 11.706 to 9.209 seconds for static U; normalized total jerk variation
falls 69.7% and 30.7%, respectively. Static U arrives 0.197665 seconds later and
is 1.51185% shorter. Vietnam keeps its arrival and increases length by 0.04957%.
Alternating occlusion and target exits improve jerk variation by about 74% at
unchanged arrival, with length increases below 0.065%.

[BMTP_GENERATION.md](BMTP_GENERATION.md) records the formulation, rejected
coarsening and numerical experiments, primary-source papers, full benchmark
scope, remaining limitations, and what profile precomputation can reuse.
