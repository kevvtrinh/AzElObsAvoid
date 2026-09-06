# Planner decisions

Current implementation: execution/data-flow cleanup based on `3877017`. This summary replaces the chronological
research log. Full recorded CSV history is retained in [benchmark.csv](benchmark.csv)
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

The cleanup passed 143 distinct MATLAB tests across the broad run and focused
corrections, 24 Node tests, and all 18 maintained default finite-jerk examples:
17 independently valid motions and the expected no-path failure. Successful
motions and arrival times matched the frozen baseline exactly. Shared geometry
removed repeated preparation; matched warm timings do not establish a speedup.
See [verification](verification.md) for scope and corrections.

The follow-up consolidation moved two single-owner helpers into their callers
and shortened repeated help text: 139 fewer production lines in this pass,
31 fewer overall versus `3877017`. All 49 focused MATLAB tests passed; fixed-target
and saved static-request checks preserved motion exactly and passed validation.

The planner is a bounded search with local trajectory optimization; completeness
and global optimality are not established. Dense moving scenes can remain costly.
Manual browser interaction and interactive Ctrl+C were not verified in the latest
refactor; PDF sources were updated but PDFs were not regenerated.

The 2026-09-06 execution audit and cleanup append actual runs to the full CSV
history. Three example-invoking tests were replaced by the separate example
matrix; their exact assertions were not rerun. The repository-health skill now
requires execution coverage and data-flow checks before classifying code as dead.
