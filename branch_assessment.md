# Planner decisions

## Sampling-independent executable length — 2026-09-08

Motion records, selection, and fixed-clock offset refinement now use adaptive
polynomial arc length. A separate integration checks randomized polynomials,
reversals, large coordinate offsets, and batch boundaries. Coarse/fine public
planner calls select identical polynomials in both timing modes. All 213 tests
pass, as do 60 frozen quality comparisons with unchanged arrivals and measured
arcs. The CSV retains all runtimes, including unfavorable samples; this is an
objective-correctness change with small measured evaluation cost. Production
is 16,477 physical lines in 115 files; final consolidation remains required.
See [length evidence](docs/bmtp_refactor_length.md).

## Static corridor integration — 2026-09-08

The working refactor integrates exact source facets, physical clocks, and
speed quadrature through shared engine helpers and removes the duplicate
motion exporter. All 209 tests pass, as do 60 complete frozen quality records
and 18 affected-request comparisons under a matched three-warmup policy. Runtime
regressions on broader fallbacks were confirmed by one matched rerun and traced
to failed clock attempts. Cheap geometric and reachability checks reduce that
work, and a linear feasibility precheck avoids unresolved speed-cone solves.
Prescribed arrivals keep the general solver. Matched medians are slalom
1.9574/0.3829 s, dense concave 0.8445/0.3588 s, Hawaii 1.3997/1.2505 s, and
target exit 2.6185/2.6282 s. Opposing U remains slower (1.1211/1.2311 s), as
does Philippines (4.1073/4.2335 s); the additional failed attempt is retained
as an explicit cost of the conditional formulation. Production is 16,408
physical lines in 114 files. The complete refactor and size gate are unfinished.
See [corridor evidence and rejected changes](docs/bmtp_refactor_corridors.md).

## Analytic departure scheduling — 2026-09-08

Eligible straight-progress direct waits now use forbidden source-derived
departure intervals from emptycore, reusing cleanup's exact motion generator.
All 198 tests pass and all 60 frozen records pass independent validation,
outcome, arrival, and adaptive-length gates. Barrier planner median falls from
0.223737 to 0.120265 s; opening U falls from 0.505405 to 0.404099 s and arrives
2.032189 s earlier. The barrier's additional 0.000787382 s is within the unchanged
0.001 s arrival tolerance. Direct-motion constructions fall from 18/19 to 3 in
the two profiles. Failed grazing proposals and sharp-corner tests are preserved;
only the construction reserve changed, never the validator or protected geometry.
Broader numerical refinement remains for ineligible or rejected proposals.
See [departure evidence and limitations](docs/bmtp_refactor_departure.md).

## Resumed containment and pinned-reference comparison — 2026-09-08

Batched containment now reuses bounded edge-projection arrays, retaining the
original MATLAB predicate in its near-edge uncertainty band. An initial parity-only
experiment failed exact signed-clearance tests and was corrected without changing
the occupancy tolerance. All 194 tests pass; all 60 frozen planner records retain
exact motions, route/search decisions, certificates, arrivals, and adaptive lengths.
Philippines median runtime is 4.1861 s versus pinned cleanup's 5.5686 s; the matched
profile reduces clearance inclusive time from 2.5516 to 1.1519 s. An initial
deforming-US slowdown did not persist in the one matched rerun. Startup/JIT noise
in millisecond cases remains disclosed, and all individual runs are preserved.
See [containment evidence](docs/bmtp_refactor_containment.md).

The [pinned reference comparison](docs/bmtp_refactor_reference_comparison.md)
also captures all 20 identical physical requests three times. Emptycore loses a
supported fixed-time interception and worsens Hawaii arrival/length/runtime, so
wholesale replacement is rejected. Selective solver replacement continues. The
production count is 108 files / 15,577 physical lines, still above cleanup's
15,528: neither the eight-stage refactor nor its code-reduction gate is complete.

## BMTP refactor started — 2026-09-08

The shared eight-stage plan is imported as `bmtp_refactor.md`, based on `c04f3b2`.
The first retained change batches offset-spline translations while preserving
exact motion outputs. All 188 MATLAB tests pass, and the 18 maintained examples
meet their expected outcomes across three baseline and three candidate runs.
Matched constructor measurements show 1.095–1.622x speedups for 8–30 output
spans; physical production size decreases from 15,528 to 15,519 lines. Full-example
warm-up histories differ, so their raw timing ratios are not controlled speedup
claims. The experimental corridor port remains outside production because it
does not preserve all measured motion-quality gates. The full plan is unfinished.
See [the progress and limitations report](docs/bmtp_refactor_progress.md).

### Resumed geometry verification — 2026-09-08

The refactor now retains source-derived geometry for time-invariant histories;
the public validator rebuilds caller-supplied preparation independently. All
192 MATLAB tests pass. Three matched repetitions of 20 frozen physical requests
preserve exact polynomials, samples, routes, certificates, search records,
arrival times, and adaptively integrated lengths. Every successful motion
passes fresh validation; the expected no-path result remains explicit. The
geographic capture now includes Hawaii, Croatia, and Philippines separately.

An eager moving-history cache was rejected after the matched rerun confirmed a
slowdown and profiles showed geometry calls rise from 242 to 1,430. The retained
static-only change restores 242 calls. Both favorable and unfavorable runs are
recorded in `benchmark.csv`. Full-branch medians improve on dense concave and
Hawaii cases by about 19% and 26%; other cases show little change or small
slowdowns. These are planner-request replays, excluding example setup and
interception-search overhead. The reference-core comparison is still running.
Production currently has 108 MATLAB files and 15,556 physical lines; the final
code-reduction and solver-replacement gates remain unfinished.

## Generic coordinates — 2026-09-07

The repository now uses x/y coordinates and caller-consistent units. Independent WrapX and WrapY flags use each workspace interval width, with stable positive-displacement half-period ties. Unwrapped physics and validation matched exactly across all 18 maintained finite-jerk examples. The shifted-period wrapping regression now reaches the equivalent endpoint in 3.372281 s instead of 6.5 s. All eight wrapping regressions and 27 browser-function checks pass; 172 distinct MATLAB tests pass. One saved-route diagnostic regression reproduces in the untouched baseline, and three pre-existing deleted fixtures block their tests. No validation tolerance or assertion was weakened. Periodic obstacles and moving goals remain unsupported. [Detailed evidence](benchmarks/results/xy_coordinate_migration_20260907.md).

Current implementation: generic coordinate API based on `3fcd62c` plus the existing working changes.
Commit verification: the four fixture regressions pass against schema-only conversions of the committed fixtures, bringing the distinct passing MATLAB checks to 176. The fixture failure and deletions above belong to pre-existing working changes and are excluded from the migration commit.
The earlier review cleanup started at `1843944`. Full recorded CSV history is retained in [benchmark.csv](benchmark.csv)
and [benchmarks](benchmarks/); earlier prose remains in Git history.

| Decision | Reason / boundary |
| --- | --- |
| Use degree-eight Bezier trajectories with MATLAB coneprog. | Adopted by the user; no external solver or MEX dependency. Cold conic-example totals improved by 1.36â€“1.65x in separate single passes, with accepted small arrival/path regressions. A 3x cold speedup was not demonstrated. |
| Keep automatic static-scene detection. | Forcing dynamic search increased Philippines median planner time from 13.437 to 115.169 s and worsened its motion. [Comparison](benchmarks/results/static_dynamic_flag_20260905.md). |
| Prefer earliest arrival; shorten travel at the retained arrival time. | `earliestArrival` and `fixedArrival` are the supported modes. Removed balanced-arrival pricing, migration, and the single-choice trajectory-method field. |
| Prepare obstacles once at the planning boundary; share solver geometry across guesses. | Internal queries and validation reuse prepared histories; public calls still check caller data. Four comparisons preserved motion and validation exactly apart from timing. [Evidence](benchmarks/results/lean_workflow_20260905.md). |
| Return `[result, diagnosis]`. | Keep motion, plotting inputs, and validation evidence in result; put detailed attempts and solver evidence in optional diagnosis. Functions receive only used inputs. |
| Use explicit path-guess stages and preserve failure evidence. | Removed dead work, duplicate timing fields, and discarded diagnostics; retain cold recovery paths proven reachable. Rejected motion remains marked unsuccessful. [Audit](benchmarks/results/planner_execution_audit_20260906.md). |
| Preserve full protected geometry and independent validation. | Apply margins once. Search sampling is not continuous collision proof. A failed search is not proof of infeasibility. |
| Keep input-derived search times and verified safe-wait pruning. | Thinning time layers missed short openings; checking only the earliest traversal missed later safe crossings. Search remains limited by its graph and edge predicate. |
| Keep Ruckig as a utility and explicit bounded fallback. | It is not a selectable planner method. Unsupported timed topology fails by default; the opt-in waypoint fallback remains limited to two segments. |
| Use Ctrl+C to interrupt MATLAB planning. | Removed production cancellation callbacks and sandbox Stop controls. Interrupting the blocking HTTP server requires restarting it. |
| Keep cleanup evidence trustworthy. | Declare expected example outcomes independently, require fresh validation in comparisons, and remove only identified wall-clock fields from physical comparisons. |
| Share defaults and preserve one goal-mode sandbox interface. | Reuse the existing option merger, remove unused private mode routing and optional bundle work, and preserve public state/export fields. |

## Closed experiments

- Fastcone/native replacements were removed at the user's request. Historical
  warm improvements do not establish current cold performance; recovery and
  motion-quality differences remain material. [Archive scope](benchmarks/results/fastcone/README.md).
- Parametric-duration LP was rejected: 23.497 versus 20.616 s on the measured
  static-U request, with 13 coneprog fallbacks in 14 replacements.
- Prepared-boundary occupancy was rejected: 49.983 versus 41.765 s on the saved
  request. Recovery controls varied materially; no speedup was established.
- Native hybrid was rejected despite faster runtime because arrival quality
  missed its gate. Coarse initialization, alternative solvers, and microbenchmarks
  did not justify a general replacement.
- The later coarse moving-obstacle screening experiment was reverted at the
  user's request. No reduced-geometry optimization is adopted from it.

## Verification and remaining limits

The timed route handoff now preserves proposed physical crossing times when
trying different arrivals and reuses the route search's complete input-derived
time grid. It no longer jumps from one velocity-only arrival estimate directly
to the deadline while stretching all interior crossing times. On the unmodified
`inefficientroute.mat` request, arrival improves from 123.870 to 112.500 s
(-9.18%), motion length from 291.311 to 228.213 units (-21.66%), and minimum
y from -89.152 to -24.550 units. Full independent validation passes with
unchanged limits, tolerances, and safety margin. Reported clearance decreases
from 0.894364 to 0.040321 units; the change is not a clearance improvement.

The repository still contains 18 maintained examples. This comparison ran a
subset of eight, plus the two supplied MAT replays, once for warm-up and three
times for measurement in each version (80 actual calls). The other nine compared
cases retained the same recorded motion metrics and validation outcomes. The
supplied case's median end-to-end wall time changed from 15.072 to 13.809 s, but
timing variation on unchanged paths precludes a general speedup claim. The
existing mixed moving-circle/static-U timed-owner tests also pass. Across the
full suite and corrected focused rerun, 155 distinct tests have passing evidence;
three remain blocked by the pre-existing deletions of `failed.mat`,
`pathtoolong.mat`, and `pathtoolong2.mat`. No deleted fixture was restored or
replaced. The remaining ten maintained examples were not rerun for this change.
The correction remains a discrete arrival search and local optimizer, not a
global continuous-time optimum; unsuccessful dense timed searches can do more
work. [Detailed scope, measurements, and limits](benchmarks/results/timed_route_handoff_20260906.md).

The MATLAB built-in replacement audit inspected all 145 maintained MATLAB files
(545 function declarations and one script). Retained substitutions use `conncomp`
for ordinary graph reachability, `discretize` for polynomial segment selection,
and direct logical-to-string conversion for four formatting-only helpers. The
segment lookup matched boundary and out-of-range behavior and reduced the
10,000-query microbenchmark from 45.5--6,950 microseconds to 25.6--85.5
microseconds across 2--1,000 segments. Median graph-connectivity timings improved
from 46.6, 261.9, and 1,124 microseconds to 38.8, 138.5, and 1,003 microseconds
at 20, 100, and 400 nodes. Rejected built-ins either regressed representative
runtime (`minjerkpolytraj`, `bounds`, and `OnOffSwitchState`) or changed protected
geometry/branch-cut behavior (`polybuffer` and `wrapToPi`). The final 154-test
suite and 70 focused tests passed; Code Analyzer reported no new issue.

Repository-wide MATLAB formatting now follows the local style documented in
`AGENTS.md`: main bodies are flush left, local helper bodies use one indent,
unnecessary continuation lines are removed, related assignments are aligned,
and scalar literal report structures use explicit field assignments. Archived
copies under `tmp` were left unchanged. Constructors whose cell values can
control MATLAB struct-array dimensions remain in constructor form to preserve
behavior. MATLAB R2024b parsed all 173 maintained `.m` files without syntax
issues. The full 152-test suite passed 151 tests and reported only the three
expected reviewed-source hash changes; inspection confirmed formatting-only
changes to those physical input sections, their hashes were updated, and the
focused hash test then passed. No planner behavior or runtime improvement is
claimed.

Focused diagnostic cleanup: removed the retired `BarrierSequence` and
`ProgressPolynomial` placeholder reports and their three private constructors
from `tryFixedTimeDetour`. No maintained source consumer reads those fields;
the optional `PathRefinement` table previously exposed them through generic
flattening. New tables omit those obsolete rows. Active search, validation,
and failure reports remain intact; historical saved MAT files are unchanged.
No motion algorithm or runtime improvement is claimed. MATLAB R2024b passed
all five `testPlannerStageTiming` tests and direct zero-input/flattened-table
checks after removal. Source-reference and diff-whitespace checks passed.
The broader example matrix was not rerun for this diagnostic-only change.

The slalom `TrajectoryDuration_s` exception was reproduced by mixing the current
planner with an older engine under `tmp`. Explicit current production paths
passed slalom and obstacle-free examples with independent validation. No planner
change was needed; the user's active MATLAB path remains uninspected.

The current review cleanup passed 149 distinct MATLAB tests and 24 Node tests.
All 18 maintained default finite-jerk examples ran before and after the changes:
17 independently valid motions and the expected `noValidatedSeed` failure.
Complete returned physical records, inputs, and independent validation matched
exactly after excluding measured runtime. The focused timing regression covers
fixed and earliest arrival with two different obstacle shapes.

Corrected the overwritten first-validation timestamp and validation-inclusive
detour timing; replaced positional validation records with named fields. Removed
duplicate option merging, unused private helpers/assignments, unnecessary optional
bundle construction, and tests coupled to internal filenames/call counts. Current
walkthrough/appendix sources now describe BMTP; retired HS3 documents are archived
unchanged. See [verification](verification.md) for all review dispositions.

Across the 112 planner, engine, and sandbox MATLAB files, this pass removes 44
physical source lines (27 nonblank, noncomment lines). The core-only size check
still fails its unchanged historical target: 12,708 versus 11,482. Its core count
increased by 30 noncomment lines, primarily for explicit validation field mapping;
the sandbox reductions outweigh that increase. No runtime speedup is claimed.

The planner remains a bounded search with local trajectory optimization;
completeness and global optimality are not established. Three tests that invoke
examples directly were omitted; their default scenarios ran in the separate
matrix, but their exact assertions and alternate-clearance case were not rerun.
Manual browser interaction, interactive Ctrl+C, and PDF regeneration were not
verified. An extra infinite-jerk example probe was rejected by the existing finite
input requirement before planning; this pass verifies finite jerk limits.

The original benchmark CSV bytes are preserved, with 36 completed example runs
and the unsupported-input attempt appended. User-owned saved MAT files are
unchanged. The obsolete autosave is recoverable under ignored
`tmp/review-cleanup-20260906/recovery/`; profiles, frozen source, and unsuccessful
runner logs also remain outside source control.

## Measured early-detour shortcut (2026-09-06)

Compared frozen current source (`6321743` plus the recorded working changes)
with an isolated copy that skips only the early fixed-time lateral detour.
22 new cases and two maintained-request replays ran once for warm-up and three
more times per mode (192 complete calls, no timeouts). Every claimed success
passed fresh independent validation, with identical physical outcomes across
repeats. The shortcut helped 10/22 new outcomes: nine shorter movements and one
additional safe solution. It materially reduced planning time in 2/22 and
increased it in 8/22, including the additional-solution case. The other cases
had no material timing change under the declared 20% and 0.05 s threshold.
Both modes failed the sealed-wall control. These small generated cases establish
usefulness for some new requests, not a real-world success rate or general
planning speedup. Production behavior is unchanged.

[Full comparison and limits](benchmarks/results/shortcut_benefit_20260906.md).
All raw repeats are retained there; the 16 maintained-request replay records
were also appended to `benchmark.csv` without changing previous history.

## Endpoint-derivative dispatch correction (2026-09-06)

The repository health trace found that the public moving-target interface
accepted requests to match target velocity or acceleration, but the general
planner later sent those non-rest endpoint states to the rest-to-rest BMTP
kernel. The kernel then threw `bmtpEngine:UnsupportedRequest` instead of
returning a planner result. Nonzero endpoint velocity or acceleration now
selects the existing state-to-state Ruckig constructor for the exact direct
attempt and every later route seed. Rest-to-rest requests keep their existing
BMTP path and two-point direct route record.

A deterministic probe passed position-only, velocity-only, acceleration-only,
and combined matching requests without an exception; every request returned a
validated motion. A separate fixed-arrival case exercised nonzero terminal
velocity and acceleration without the moving-target wrapper. MATLAB R2024b
passed all 154 repository tests, including the new contract cases, and Code
Analyzer reported no messages in the four changed files. This establishes the
supported one- and two-segment state-to-state cases exercised here; longer
Ruckig waypoint routes still return their existing stable unsupported result.

## Clearance optimization comparison (2026-09-06)

Four bounded optimization candidates were evaluated against commit `a790424`
on six maintained examples. Each timed candidate used identical inputs, one
warm-up, and three measured repetitions. The accepted candidate reuses prepared
polygon edges during clearance queries and uses exact oriented edge half-spaces
for verified convex single-ring shapes. Its sum of per-example median runtimes
fell from 42.2472 s to 36.7781 s, a 12.9456% reduction (1.1487x speedup), for 23
net added production lines: 0.56285 percentage points or 0.23779 aggregate
seconds saved per added line. An initially slow static-U measurement did not
repeat in a separate five-run confirmation. All compared physical result fields
were exactly unchanged, all six independent validations passed, and MATLAB
R2024b passed all 155 repository tests.

Three candidates were rejected and reverted. Cached Bernstein/binomial matrices
added 29 production lines for only a 1.1229% aggregate reduction and regressed
the dense-concave and accelerating-circle examples by 18.13% and 13.98%.
Shape-comparison prefilters added 21 lines and reduced aggregate time by 10.5591%
but regressed dense-concave by 24.04% and accelerating circles by 9.89%. A
33-line constant separating-plane shortcut failed the correctness gate before
timing because a retained plane invalidated a later trajectory iteration. No
rejected experiment was retained or added to `benchmark.csv`.

## Earliest-arrival exact-clock travel refinement (2026-09-06)

Repository history confirms that `balancedArrival` existed before commit
`4344795`. It ranked validated motions by travel plus a declared 1 units/s
arrival-time exchange rate and also ran travel-weighted BMTP refinements. A
bounded attempt to restore that full trade inside `earliestArrival` was rejected
and reverted: on `inefficientroute.mat`, the best policy-compliant motion reduced
sampled travel from 187.745001194 units to 184.750085715 units, only 1.60%, below the
declared 3% benefit gate. Trials that shortened the path further required more
delay than their saving justified at the historical exchange rate.

The retained narrower change implements the already documented path-length
tie-break at the exact earliest-arrival clock. After BMTP establishes the fastest
feasible homotopy, one fixed-clock travel solve may replace it only with a
shorter solution at the identical duration. On `inefficientroute.mat`, arrival
remained 91.5513221229 s while sampled travel fell by 2.995061021 units, from
187.745001194 units to 184.749940173 units. Independent collision, velocity,
acceleration, jerk, and plane-certificate checks all passed. The structurally
different static-box planner contract passed, as did all affected option and BMTP
engine tests (16/16). The maintained fixed-arrival
`exampleTargetExitsObstacle` remained independently valid at 24 s, with
21.7425467317 units polyline and 21.9321570168 units smoothed length. This change
does not claim or implement a justified arrival-time trade; it records the
initial/final travel and duration so a future bounded Pareto policy can be
evaluated without hiding its cost.

## Circle detour and explicit objective priority (2026-09-06)

The exact `inefficientroutecircle.mat` replay reached the physical arrival floor
but traveled 249.201569069 units. The fixed-clock excursion treated its nominal
peak as an unconstrained spline through-point; y continued rising to
54.920807222 units after the obstacle's approximately 26 units top. An additional
proposal now imposes zero complete-axis velocity at the proposed turn while
leaving acceleration free. Original through-point proposals remain available,
and every retained motion still passes independent continuous validation.

The user explicitly clarified the objective: earliest arrival, then shortest
path, subject to all physical limits. Accordingly, the earlier integrated-jerk
rejection within fixed-clock travel refinement has been removed. Integrated
squared jerk remains diagnostic and may increase; no physical derivative limit,
collision tolerance, protected geometry, or endpoint condition was relaxed.

On the unchanged saved circle request, travel falls to 227.456480998 units
(-8.73%) at the identical 113.691362616 s arrival. Peak y falls to
26.602152360 units, and independently checked clearance is 0.000413923 units.
A structurally different asymmetric rectangle with the other governing axis
and a nonzero start time improves from 112.397597877 to 103.355070722 units
(-8.05%) at the identical 52.966666667 s duration. One warm-up and three measured
calls per implementation and case all passed. Warmed median planning time
increased from 2.067310 to 3.910954 s for the circle and from 0.938971 to
1.629758 s for the rectangle. This is a motion-quality improvement with extra
planning work, not a runtime speedup or a global shortest-path certificate.

All 18 maintained examples ran before and after: 17 independently valid motions
and the expected `noValidatedSeed` failure, with unchanged durations. Three
maintained paths shortened; none lengthened. MATLAB R2024b passed 60 affected
tests, including static/moving obstacles, fixed/earliest arrival, and expected
failure behavior. Code Analyzer reported no messages in the four changed MATLAB
files. The full test suite was not rerun; the two route-economy tests requiring
previously deleted MAT fixtures were excluded. Existing user edits and missing
fixtures were preserved, except for the jerk-cost policy explicitly superseded
by the user's instruction. Production changes add 49 and remove 18 lines
relative to the starting working source, with no new planner option or dependency.

[Complete measurements and all 18 example comparisons](benchmarks/results/circle_route_economy_20260906.md).
Actual accepted comparison and example runs are appended to `benchmark.csv`;
raw MAT results, starting-source copies, and plots remain in ignored `tmp`.

## Combined derivative limits (2026-09-07)

The public planner now accepts velocity, acceleration, and jerk limits either
as three positive finite scalar magnitudes or as three two-element axis vectors.
Mixing those forms raises `planTrajectory:MixedLimitModes`. Each scalar `L` is
the hypotenuse and becomes `[L/sqrt(2), L/sqrt(2)]` internally; this allocation
is fixed, with no transfer of unused capacity between axes. Position intervals
retain their existing bounds and defaults. Resolved limits remain per-axis
vectors and can be reused without another division.

One normalizer owns this contract for planning, moving-target interception,
public validation, MATLAB sandbox overrides, and offline JSON requests. Native
sandbox controls preserve the full converted values on readback. Maintained
examples already use per-axis velocity and acceleration, so their jerk-only
override must also be a two-element vector; scalar jerk is no longer silently
duplicated.

MATLAB R2024b passed 66 selected core checks and 26 sandbox checks (83 distinct
tests, including 11 new limit-contract tests). Evidence covers scalar/explicit
axis motion equivalence in both arrival modes, exact and specified-time
interception, all six mixed forms, endpoint failure, every derivative bound,
an interior velocity violation between samples, defaults, unequal axis limits,
workspace handling, existing static/moving planning, Ruckig fallback, graphics
control readback, JSON, and bundle replay. Code Analyzer reported no issues in
the nine changed MATLAB files. Results remain in ignored
`tmp/combinedLimitChecks.mat` and `tmp/combinedLimitSandboxChecks.mat`.

The full test suite and all 18 maintained examples were not rerun for this
input-contract change; no example runs or benchmark rows were added. The
pre-existing missing `failed.mat` replay fixture remains untouched. No runtime
improvement or additional trajectory optimality is claimed.

## Vietnam keep-out input diagnosis (2026-09-07)

The exact `examples/data/vietnamKeepoutSlewInput.mat` request succeeds on
`5872d456` plus the existing working changes. MATLAB R2024b independently
validated its 30 s motion: polyline 17.297991315814 units, sampled smoothed
length 17.305374620919 units, and reported protected clearance 0.000061818787
units. Collision and continuous kinematic checks pass. The optional plane
certificate is not accepted; adaptive continuous validation resolves all
intervals instead.

At the diagnosis snapshot, the public call took 203.211984 s, with 180.112266 s in route search and
14.083587 s in motion solving. A separate unchanged-search profile attributes
the dominant cost to timed-edge collision queries over 111 positions and 65
time layers. The exact search records 4.82 million rejected transitions;
this count includes cost and timing decisions. This is successful but costly
planning, not evidence of physical infeasibility.

All 248 protected slices repeat their closing vertex despite the documented
open-ring contract. Removing only that duplicate in a geometry-only copy
preserves sampled occupied sets exactly and restores certified interpolation
for seven regions (210 intervals); the main concave region still uses a hull.
A rotating-triangle control reproduces the same representation dependence.
During that diagnosis, no source behavior, input file, or validation tolerance
was changed, and no speedup or corrected-input planning outcome was claimed.
That profile was for attribution; the diagnosis itself did not measure timing
medians or run the full suite. The subsequent optimization is recorded below.

[Complete diagnosis and verification limits](benchmarks/results/vietnam_keepout_diagnosis_20260907.md).
The one actual planner replay is appended to `benchmark.csv`; raw evidence is
retained under ignored `tmp/vietnam-diagnosis-20260907/`.

## 2026-09-07: Vietnam runtime reduction applied

The exact saved request now has a **25.8342754 s median planner runtime**, down
from the frozen working baseline's **198.1828721 s**: **86.9644% less time**
(7.6713x). The final measured repeats were 35.1461561, 25.8342754, and
23.2860196 s; the preliminary run was 39.468021 s. The median meets the requested
30 s target, with visible run-to-run variation rather than a hard deadline.

`timeExpandedVisibilitySearch` now reuses exact occupancy answers within
stationary geometry intervals, batches query positions and candidate entry
construction, and reuses checked endpoints only when both coordinates and time
match exactly. The private lookup allowance is 300 MiB as requested; this
request needs much less. Cache eviction and bypass repeat existing checks.
The protected geometry, all 65 input-derived search layers, physical limits,
13-sample screening semantics, validation tolerances, and candidate order remain
unchanged. Nonfinite query clocks cannot populate finite-time cache entries.

The complete route, polynomial, motion histories, resolved inputs/options,
search record, and fresh independent validation matched exactly after excluding
runtime fields. The selected physical motion remains 30 s long: polyline
17.297991315813945 units, sampled smoothed length 17.305374620918947 units,
`goalReached`, collision-free and kinematically valid. The optional plane
certificate remains unaccepted; continuous adaptive validation resolves safety.

All 18 maintained examples matched the frozen original in full result/search/
validation comparisons, including the expected no-path failure. The affected
suite passed 126/126 tests; an additional overflow-cache control passed on both
implementations, for 127 distinct covered checks. Code Analyzer reported zero
issues in the changed production file and new test file. Verification was
focused, not the entire repository suite; the unrelated convergence-summary
case and user-deleted legacy fixtures were not exercised.

The two production/test files were promoted from byte-identical tested copies.
The supplied input MAT hash and the user's preexisting changes were preserved.
Actual accepted runs are appended to `benchmark.csv`; unsuccessful experiments
and measurement limitations are recorded in the
[runtime report](benchmarks/results/vietnam_keepout_runtime_20260907.md).

## 2026-09-07: Vietnam promoted to maintained example and pruned search

`exampleVietnamKeepoutSlew` now replaces `exampleObstacleAvoidance` in the
maintained 18-example inventory. The source request moved unchanged to
`examples/data/vietnamKeepoutSlewInput.mat`; its SHA-256 remains
`dbd475086da1398e4735887b78d3a5466a6c22deb67174fb889540de392c7cac`.

Fixed-arrival timed search now uses an incumbent goal cost with Euclidean
distance as an admissible completion lower bound. It rejects only strictly worse
candidates and preserves cost ties. For the maintained Vietnam request, motion
edges fall from 56,571 to 29,231 while the exact route, polynomial, and complete
motion histories remain unchanged and independently valid. Candidate enumeration
also batches consecutive source nodes so its temporary logical tensor cannot
exceed 1,048,576 elements; this request remains on the one-batch fast path.

Accepted timed-solver diagnostics have one complete owner, and exact diagnostic
flattening now preallocates and block-collects homogeneous leaf arrays. The
Vietnam solver-detail table falls from 136,333 to 68,275 rows without losing
field paths. An unhelpful convex-occupancy experiment was removed.

The fully integrated CBBC comparison measured a 21.8339201 s baseline median and
17.56002525 s candidate median, 19.5746% lower. All runs returned the exact
frozen physical motion and passed fresh validation. Earlier measurements include
an unfavorable 37.6722 s candidate outlier and are retained in the report.

All 18 maintained examples produced their expected outcome. Across the full
suite and post-fix focused reruns, all 186 tests independent of pre-existing
Rogue Example fixture state passed. Three remaining tests require user-deleted
MAT files; the fourth fails identically under the frozen implementation against
the user-modified `inefficientroute.mat`. Code Analyzer and `git diff --check`
are clean.

[Complete follow-up, runtime evidence, and verification limits](benchmarks/results/vietnam_maintained_example_runtime_20260907.md).
