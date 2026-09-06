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
