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
