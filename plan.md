# Plan 325 corridor-only completion checkpoint

## Objective And Outcome

Complete `325-less-nlp` as a deterministic corridor-quintic planner without
HS3/NLP, while preserving independently validated trajectories, stable failure
diagnostics, maintained benchmark paths, and hard repository size limits.

The latest bounded task is complete locally. Fixed random moving-circle cases
3, 4, 6, and 8 now solve with at most three seeds. The complete deterministic
sweep improves from pushed baseline 4/8 to final 8/8 independently validated
solutions, without changing any maintained example path metric.

## Retained Design

1. Generate deterministic topology from obstacle, state, limit, and option
   inputs under the existing bounded-work policy.
2. Preserve the established seed portfolio on connected graphs.
3. If the sparse visibility graph remains disconnected after its third
   offset/exhaustive retry, test the four input workspace corners as ordinary
   visibility nodes and preserve one spatial-diversity seed opportunity.
4. Reject blocked boundary edges normally; the workspace-spanning wall remains
   an expected diagnosable failure.
5. Construct, retime, and independently validate the complete C3
   corridor-quintic motion. No obstacle name, scenario name, expected route,
   hidden waypoint, tolerance relaxation, or HS3 fallback affects behavior.

## Final Evidence — 2026-08-22

- Pushed baseline: `0c8bf66`, 4/8 fixed-circle coverage.
- Final sweep: 8/8 independently valid in `232.089424 s`; formerly failing
  cases 3, 4, 6, and 8 have durations `21.1728041688`, `21.2089164576`,
  `21.1748286081`, and `20.9089388561 s`.
- Tightest final sweep clearance is `0.000355731152304 deg`; all are positive.
- Focused dynamic suite: 7/7 in `95.3418714 s`, including the four new cases,
  prior collision feedback, and static no-path wall.
- Full suite: 59/59 in `134.081193 s`.
- Code Analyzer: zero messages across 66 nonscratch MATLAB files.
- Maintained examples: 17 validated successes and one validated expected
  failure in fresh serial processes. Every successful polyline, smoothed
  length, and duration is exact to pushed evidence within `1e-6`.
- Fresh example wall sum is an unfavorable `205.6452420 s`; no speedup is
  claimed. Exact rows are in `benchmark.csv` under
  `working-tree-boundary-support-final`.
- Visible success created three figures and 522 objects. Visible expected
  failure created two returned-diagnostic figures and 342 objects before a
  Windows graphics teardown fault; the fault is recorded in `verification.md`.
- Core production remains exactly 7,500 lines excluding the 565-line plotter;
  maintained MATLAB excluding examples/scratch is 10,392 lines; largest
  production file is 887 lines.

## Current Limitations

- The improvement is focused coverage, not completeness or global optimality.
- Several random-case solutions use workspace-boundary detours and retain small
  but independently positive clearances.
- Dynamic route scaling and the moving/deforming and extreme-outline wall times
  remain unfavorable.
- Production is at the exact user-authorized 7,500-line ceiling.
- The Windows GUI teardown fault remains an environment issue; planner tests,
  example validation, and figure construction completed before it occurred.

## Repository State

Production, regression test, `verification.md`, `branch_assessment.md`,
`benchmark.csv`, and this checkpoint contain the retained local change and its
evidence. Temporary harnesses are removed. Twelve pre-existing untracked
benchmark artifacts remain untouched. No commit or push has been requested.
