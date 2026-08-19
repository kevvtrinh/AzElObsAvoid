# Plan 502 branch assessment

## Evidence scope

This assessment applies to branch `plan-502-implementation` at source commit
`6c6b948`. It uses the 18 maintained example runs, 25 focused tests, MATLAB
Code Analyzer results, visible and headless plot checks, line counts, and the
repository-wide legacy-code and call-site review.

## Biggest strength

The largest strength is one correctness-focused planning path with independent
evidence. Static obstacles, moving obstacles, deforming obstacles, waiting,
moving targets, fixed arrival, earliest arrival, concave geometry, and dense
geographic geometry use the same public planner. Each successful example
passed independent collision and kinematic validation.

The planner also keeps failure evidence. A no-path or time-limit result retains
the explored nodes, accepted and rejected transitions, frontier, best partial
route, seed summaries, and termination reason. General `Verbose` output reports
the same planner events without enabling an optimizer iteration table.

This is stronger than an example-only success claim. The result states that it
is the earliest validated local HS3 solution from the attempted deterministic
seed set. It does not claim global completeness or global time optimality.

## Biggest weaknesses

### 1. Finite local-search coverage

The planner can miss a feasible topology. HS3 is a local nonlinear optimizer,
and the topology generator returns a bounded seed set. A conservative cluster,
dense swept envelope, or frozen local corridor can remove a narrow useful
initialization even when an exact feasible path exists. Independent validation
prevents false success, but it cannot make the seed set complete.

### 2. Runtime on difficult geometry

The slowest verified examples were:

- `exampleUSOutlineExtremeVisibility`: 333.423481 s.
- `exampleFourAcceleratingCircles`: 176.305066 s.
- `exampleOpeningUShapedAzElTimeSpace`: 121.012455 s.

Continuous collision resolution, repeated local optimization, and dense
geometry protection improve correctness but increase runtime. The branch has
not established a worst-case runtime bound.

### 3. Small remaining size margin

Production MATLAB code has 5,997 physical lines. The target is 6,000 lines.
`generateAzElTopologySeeds.m` and `solveAzElHs3.m` each have 900 physical
lines. This leaves little space for a new feature without simplification.
There are 22 production files, which exceeds the 8-to-16 file-count target,
although the line-count target and hard limits pass.

### 4. Conservative seed geometry is an explicit tradeoff

Nearby obstacle clustering and the coarse directional dense envelope reduce
topology-graph work. They do not replace the exact obstacle history used by
HS3 and validation. However, a large clustering distance or coarse convex
envelope can remove a narrow seed corridor and cause a false planning failure.

## Current judgment

The branch is suitable as a compact correctness reference and as a general
deterministic planner for the verified scenario families. Its main development
need is lower runtime and broader seed coverage without adding a second planner,
weakening continuous validation, or exceeding the size limits.
