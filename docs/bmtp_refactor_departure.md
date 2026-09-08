# Analytic departure scheduling

The planner now replaces repeated wait trials for eligible exact straight-line
rest-to-rest motions with emptycore's forbidden-departure formulation. It reuses
cleanup's existing analytic direct motion, including its per-axis limit semantics.
It does not introduce another direct-motion generator.

Each protected history interval becomes convex source geometry in space-time.
Static concave shapes, holes, disconnected components, and topology-transition
unions use exact convex decomposition of the prepared source model. Convex
moving intervals use the hull of their endpoint shapes in space-time; this can
conservatively cover extra space when the shape deforms. Moving concavities,
moving-target clocks, nonzero endpoint derivatives, non-straight direct profiles,
and cells with more than 128 vertices retain the broader existing path. The
vertex limit bounds temporary cross-section pairs; it does not reduce obstacles.

The motion's scalar progress intersects those cells. Extrema of forbidden delay
along each cubic phase come from endpoints and quadratic derivative roots.
Occupied waiting positions exclude every delay extending into their activity.
The first available departure is a proposal, never an acceptance certificate.
The complete public motion check must pass, and normal candidate selection still
compares waiting with detours. A disabled wait-refinement option remains disabled.

## Validation and numerical reserve

The first exact-boundary proposals were unresolved by cleanup's adaptive
validator. A one-collision-step departure reserve passed the maintained waiting
examples but failed a sharp-corner regression, including exchanged axes and a
translated time origin. The final proposal uses the larger of the existing
arrival tolerance and collision time-step settings as a departure reserve.
No source geometry, safety margin, acceptance tolerance, or validation limit was
changed. These failed experiments and tests remain in ignored logs.

Ineligible or rejected schedules continue through the existing numerical wait
refinement. The retained numerical loop therefore still has a broader technical
responsibility; it is skipped on accepted eligible schedules. No failure of this
restricted formulation is reported as global infeasibility.

The full MATLAB suite passes **198 tests**. Focused checks cover translating
barriers, exchanged axes, shifted time origins, multiple blocked departure
intervals, unsafe waiting positions, short horizons, nonzero-state and moving-
target bypass, and a detour faster than waiting. A contract test now checks
accepted departure scheduling rather than requiring execution of the superseded
bisection loop; it retains successful independent validation and a reduced wait.
Final naming cleanup, explicit target bypass, and preservation of a non-first
waiting seed's identity passed 40 focused tests. The added seed-identity
regression brings distinct covered tests to 199; the preceding complete suite
contained 198 tests.

## Matched full-request measurements

All 60 records preserve expected outcomes and pass arrival and independently
integrated length gates. Every successful motion passes fresh independent
validation. The other 18 requests retain their arrival and arc length exactly.
The following comparison isolates this stage against commit `6b2f314`:

| Case | Planner median before / after s | Motion duration before / after s | Route / arc length units |
| --- | ---: | ---: | ---: |
| Moving barrier | 0.223737 / 0.120265 | 10.090301514 / 10.091088896 | 10 / 10 |
| Opening U | 0.505405 / 0.404099 | 13.617522354 / 11.585333446 | 10 / 10 |

Both cases terminate with `goalReached`; collision, kinematic, and applicable
certificate checks pass. The barrier is **0.000787382 s later**, within the
unchanged 0.001 s arrival comparison tolerance. Opening U finishes 2.032189 s
earlier. Length differences are below 1.1e-14 units. No universal runtime gain
is claimed: unchanged cases still show run-to-run timing differences. All
individual records, including unfavorable measurements, are in `benchmark.csv`.

Separate profiles attribute the work reduction:

| Case | Direct-motion constructions before / after | Validation-function calls before / after | Wait-refinement inclusive s before / after |
| --- | ---: | ---: | ---: |
| Moving barrier | 18 / 3 | 61 / 31 | 0.145859 / 0.085240 |
| Opening U | 19 / 3 | 87 / 55 | 0.201637 / 0.129705 |

Validation-function counts include empty-record requests as well as actual
motion checks. The new scheduler runs once per case. Its inclusive/self profile
times are 0.072882/0.011480 s for the barrier and 0.114968/0.013071 s for opening
U. The engine's forbidden-interval projection accounts for 0.045997/0.030331 s
and 0.061075/0.039292 s respectively. Do not add these nested times together.
The profile script completed; a subsequent unrelated scratch-corridor launch
had a path error and was restarted using an absolute script path.

Benchmark replay uses the same one-warmup/three-repetition policy as the prior
stages, with the same physical requests and per-axis limits. It excludes example
setup and interception-wrapper search. As previously reported, one warmup does
not guarantee a fully steady JIT state.

This stage adds a conditional formulation and its source adapter. Consolidation,
static corridor/clock replacement, exact visibility migration, and the final
production-size reduction remain unfinished.
