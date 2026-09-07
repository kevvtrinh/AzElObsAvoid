# Planner decisions

Current implementation: review cleanup based on `1843944`. This summary replaces
the chronological research log. Full recorded CSV history is retained in [benchmark.csv](benchmark.csv)
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
(-9.18%), motion length from 291.311 to 228.213 deg (-21.66%), and minimum
elevation from -89.152 to -24.550 deg. Full independent validation passes with
unchanged limits, tolerances, and safety margin. Reported clearance decreases
from 0.894364 to 0.040321 deg; the change is not a clearance improvement.

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
