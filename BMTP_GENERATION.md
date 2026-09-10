# Intrinsic jerk variation and profile reuse

This experiment changes motion generation. It does not filter returned samples
or apply a separate smoothing pass. Original geometry, margins, continuity,
endpoint conditions, and independent validation remain in force.

## Causes and formulation

Length alone leaves many nearly equivalent jerk histories. A detailed moving
obstacle clock was subdivided four times per source interval, creating 120
optimizer phases in Vietnam. C3 continuity connects jerk values but does not
penalize repeated changes in jerk. On static variable clocks, a time-only
objective can additionally collapse phases into very short intervals.

The experiment uses natural clock intervals, retaining at least eight phases
when possible and up to four subdivisions per source interval. Static guide
edges use three rather than four initial phases. Compact fixed-clock steering
models retain their original objective; larger models include integrated
squared snap in their conic generation objective. The static variable-clock
objective also penalizes changes of normalized jerk controls.

Fixed-clock optimization uses physical local position, velocity, acceleration,
and quadratic jerk variables. Direct third differences of absolute position
controls amplified cancellation on short phases; adding snap cost in those
coordinates produced invalid proposals. The physical representation uses an
exact unit-time integration template scaled by h, h^2, and h^3. Local rotated
cones are considerably cheaper than a single cone spanning every phase.

For quadratic Bernstein jerk controls j0, j1, j2, define d0=j1-j0 and d1=j2-j1.
The exact phase snap energy is 4/(3h)*(d0^2+d0*d1+d1^2). After normalizing each
axis by its jerk limit, the maximum summed two-axis energy is 32/(3h). This
gives a dimensionless measure bounded by one over the full clock. Its weight
is 0.005 times the start-to-goal distance. It bounds the added *objective*
cost; control-polygon length is not exactly continuous arc length.

In the variable-clock solve, the sum of squared normalized jerk differences,
divided by 16 times the phase count, is at most one. Multiplying it by the
existing PathLengthTimeAllowance_s (default 0.49 seconds) bounds that penalty.
The subsequent fixed-clock repair spends no additional arrival allowance.
Setting the option to zero removes this variable-clock penalty.

These bounded penalties establish trade bounds for globally solved instances
of the same formulation. The nonlinear solve is local, and changing phase
count changes its feasible set. They do not prove that every new request will
stay within 0.5 seconds of the previous implementation's output or of the
global optimum. Arrival, actual arc length, runtime, and exact jerk variation
must therefore be reported independently on identical inputs.

Blanket coarsening was rejected: one or two phases eliminated feasible early
arrivals in moving circle, moving US, and the rotating field. Preserving the
small steering mesh restored those results. Plane consolidation alone had
not consistently reduced oscillation. Neither C3 nor minimum snap guarantees
monotone jerk or eliminates necessary braking reversals.

## Papers

The search found no BMTP-specific published oscillation patch. The original
[BMTP paper](https://arxiv.org/html/2608.02834v1) minimizes time; its example
representation includes degree-eight C4 trajectories and snap constraints.
This branch uses C3 quintics, so those representations should not be conflated.

[Marcucci et al., Fast Path Planning Through Large Collections of Safe Boxes](https://web.stanford.edu/~boyd/papers/pdf/fpp.pdf)
gives fixed-time Bernstein derivative-energy objectives. That objective idea
transfers here without importing its safe-box decomposition or graph.
[Richter, Bry, and Roy](https://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf)
discuss derivative-energy objectives and the conditioning benefit of endpoint
derivative variables. [Caponigro et al.](https://arxiv.org/pdf/1303.5796)
provide bounded-variation regularization results under stated assumptions;
those results are motivation, not a proof that these examples exhibit Fuller
chattering or inherit the paper's convergence guarantees.

## What can be precomputed

Exact algebra can be reused immediately: Bernstein conversion, integration
templates, derivative-energy matrices, and sparsity/index patterns. Numerical
factorizations generally change when times, separating planes, or active
constraints change. The new physical-state integration already uses one
exact template instead of repeatedly constructing it for each phase.

[Ruckig](https://www.roboticsproceedings.org/rss17/p015.pdf) enumerates analytic
jerk-limited profile families for obstacle-free transfer; its piecewise-constant
jerk does not directly satisfy this branch's C3 position contract. Its restored
package remains standalone and is not called by the planner.

## Measured result

MATLAB R2024b, the same physical inputs, default six computational threads,
plotting off, one unmeasured warm-up and three measured repetitions. The
baseline here is the already-verified plane-consolidation/runtime candidate,
not the earlier C2 implementation.

| Example | Planner before / after (s) | Arrival change (s) | Arc-length change | Jerk variation change |
| --- | ---: | ---: | ---: | ---: |
| Vietnam | 4.966 / 2.916 | 0 | +0.04957% | -69.7% |
| Static U | 11.706 / 9.209 | +0.197665 | -1.51185% | -30.7% |
| Alternating occlusion | 1.483 / 0.655 | 0 | +0.06472% | -74.7% |
| Target exits obstacle | 1.199 / 0.619 | 0 | +0.04323% | -74.1% |

Jerk variation integrates absolute snap at every exact quadratic-jerk turning
point, divides each axis by its jerk limit, then sums the axes. It measures
oscillation separately from integrated squared jerk and from continuous arc
length. The regression helper also counts any residual jump at segment joins.

All 18 examples (20 results including geographic subcases) kept their valid
success or expected no-path outcome. Their measured arrival increases stayed
below 0.5 seconds and arc-length increases below 0.5%. All other arrival times
and lengths were unchanged. Moving US retained its 8.5-second arrival and
40.298297432-unit length. The three geographic motions were unchanged; their
planner medians were 4.102, 1.630, and 12.978 seconds. This change does not
eliminate oscillation in the separate monotone-corridor formulation.

Small runtime fluctuations were investigated: eight alternating baseline/new
pairs for the rotating field gave planner medians 0.623079 / 0.631216 seconds,
with identical arrival and length. This does not establish exact runtime
equality. Moving-US planner medians were 6.125 / 6.235 seconds in the full sweep;
its expensive geometry construction was unchanged by the generation edit.

The first full test run exposed an infeasible U initialization with a 7.29
linear constraint violation. Feeding that finite conic result into the
nonlinear optimizer produced many conditioning warnings and wasted about nine
seconds. The solver now checks the original linear constraints and bounds,
at the existing ConstraintTolerance, after its existing endpoint roundoff
correction. An unusable initialization returns before nonlinear refinement.
The valid U initialization has a residual below that same tolerance after
projection and returns the same motion. A trial duration floor was rejected:
it worsened the infeasible case and did not resolve the conditioning problem.

The tests cover independent validation, the energy-cone bound, exact jerk
variation, full endpoint states, transformed coordinates/clocks, invalid
inputs, the saved arrival/path regressions, and expected no-path outcomes.
All 126 MATLAB tests passed. After installing the tested source on `build-core`,
all 18 graphical examples also passed: 19 independently valid motions and the
expected `noVisibilityRoute` result, including all three geographic subcases.
One older test assumed a future occupied starting point made the entire
problem infeasible. The new generator can leave before occupation; that test
now checks independent validity and clearance from the occupied box whenever
a motion is found, while still rejecting an unsafe delayed departure.

Ignored evidence: `scratch/holistic/generator_v2_suite.{mat,json}`,
`generation_comparison.md`, `generation_timing_followup.{mat,json}`,
`generation_jerk_comparison.png`, and the generation test logs. Solver optimality
is not guaranteed: static U still reaches the nonlinear iteration limit, and
physical success is established by independent validation. Static U and
Philippines still take more than five seconds. Profile reuse remains a separate
opportunity to reduce those optimization costs.
