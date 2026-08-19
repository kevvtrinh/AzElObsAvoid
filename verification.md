# HS3 diagnostic refactor verification

## Environment

- Branch: `hs3-diagnostic-refactor`.
- Starting commit: `e46ccae6343a8127d303211a0d1134754a847bc2`.
- Runtime: MATLAB R2024b Update 4.
- Solver: Optimization Toolbox `fmincon`.
- Random seed: zero for all default planner calls.

## Final size

| Scope | Files | MATLAB lines |
| --- | ---: | ---: |
| Production | 17 | 5,215 |
| Examples and private example helpers | 23 | 2,873 |
| Focused tests | 1 | 519 |
| Complete maintained MATLAB repository | 41 | 8,607 |

The baseline had 18,090 production lines and 24,436 total MATLAB lines. The
final result is below the 6,000-line production target and the 10,500-line
repository target.

The target was 8 to 16 production files. The result has 17. The additional
file, `+azElInternal/searchAzElTimeExpandedGraph.m`, owns the complete bounded
time-layer search. Keeping this function separate prevents the seed generator
from exceeding the 900-line file limit and prevents time-search state from
entering the HS3 solver.

Three production files exceed 700 lines. All are below 900 lines:

- `+azElInternal/generateAzElTopologySeeds.m`: 837 lines. It owns spatial
  visibility seeds, conservative swept geometry, seed diversity, and graph
  diagnostics.
- `+azElInternal/solveAzElHs3.m`: 809 lines. It owns the separated
  jerk-controlled HS3 model, local obstacle corridor, nonlinear solve, and
  candidate reconstruction.
- `planAzElMotion.m`: 744 lines. It owns input normalization, seed scheduling,
  independent candidate validation, result selection, and the stable schema.

These responsibilities share internal state that would need a new duplicate
representation if they were divided again. The selected split keeps the graph,
the mathematical kernel, and public orchestration separate.

## Final headless example results

All runs used jerk constraints. `Planner` is the planner success state.
`Validation` is the independent example-validation state. Lengths are degrees.
Duration and runtime are seconds.

| Example or region | Planner | Validation | Polyline | Smoothed | Duration | Collision | Kinematic | Runtime | Reason |
| --- | --- | --- | ---: | ---: | ---: | --- | --- | ---: | --- |
| `exampleObstacleFreeAzElMotion` | pass | pass | 4.472136 | 4.472136 | 8 | pass | pass | 4.465437 | `goalReached` |
| `exampleAzElPlanning` | pass | pass | 11.152120 | 11.567722 | 12 | pass | pass | 41.228741 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | pass | pass | 13.098463 | 13.795538 | 16 | pass | pass | 11.035922 | `goalReached` |
| `exampleMovingBarrierWait` | pass | pass | 11.769860 | 10.321701 | 12 | pass | pass | 12.016864 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | pass | pass | 12 | 12.012667 | 15 | pass | pass | 14.703396 | `goalReached` |
| `exampleAlternatingSlalom` | pass | pass | 16.019320 | 16.384571 | 22 | pass | pass | 16.920260 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | pass | pass | 10.097524 | 7.342217 | 6.275806 | pass | pass | 10.227419 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | pass | pass | 13.831635 | 13.036816 | 15 | pass | pass | 43.488484 | `goalReached` |
| `exampleNoPathAzElMotion` | expected fail | pass | `NaN` | `NaN` | `NaN` | not available | not available | 5.377993 | `noValidatedSeed` |
| `exampleFortyMovingCircleGrid` | pass | pass | 111.354775 | 118.866473 | 200 | pass | pass | 135.786504 | `goalReached` |
| `exampleFourAcceleratingCircles` | pass | pass | 25.164826 | 20.588787 | 22 | pass | pass | 50.204030 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | pass | pass | 9.538941 | 9.538941 | 12 | pass | pass | 4.657692 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | pass | pass | 71.930983 | 470.884620 | 300 | pass | pass | 245.673700 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | pass | pass | 10 | 10.153312 | 12.304983 | pass | pass | 50.441460 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | pass | pass | 13.556661 | 14.423757 | 20.869565 | pass | pass | 44.173038 | `goalReached` |
| `exampleTargetExitsObstacle` | pass | pass | 24.461148 | 20.598601 | 24 | pass | pass | 26.046134 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | pass | pass | 23.853721 | 24.296649 | 22.875112 | pass | pass | 62.204455 | `goalReached` |
| `exampleUSOutlineExtremeVisibility/Hawaii` | pass | pass | 14.304462 | 13.188632 | 120 | pass | pass | see note | `goalReached` |
| `exampleUSOutlineExtremeVisibility/Croatia` | pass | pass | 6.700000 | 7.931866 | 120 | pass | pass | see note | `goalReached` |
| `exampleUSOutlineExtremeVisibility/Philippines` | pass | pass | 24.711660 | 19.677931 | 120 | pass | pass | see note | `goalReached` |

The three-region geographic sequence took 178.617265 seconds. The 40-circle
grid and moving/deforming U.S. outline used the documented conservative swept
bounding-box envelope. Every successful result passed the public continuous
collision and kinematic validator.

## Automated and visual checks

- Code Analyzer: 41 files, zero messages.
- Focused tests: 21 passed, zero failed, zero incomplete, 54.932075 seconds.
- Legacy MATLAB dependency search: zero matches.
- `git diff --check`: passed.
- Visible success check: three figures were created and all three were visible.
- Hidden failure check: two figures were created. A title included
  `noValidatedSeed`.

The focused tests cover the analytic jerk chain, fixed and earliest arrival,
endpoint state, mesh refinement, opposite-side seeds, translating and
deforming polygons, waiting, safety-margin provenance, between-node
collision and kinematic violations, deterministic repetition, planning-time
failure, no-path diagnostics, and moving-target adaptation.

## Removed stack and retained limits

The refactor removes the SIPP and safe-interval search, snapshot visibility
graphs, event selection, alternate space-time forwarding planner, old direct
collocation path, parallel multi-seed machinery, old solver cascades, and
their exclusive options, tests, benchmarks, and wrappers.

The maintained planner has one production path. A finite deterministic seed
set can miss a feasible topology. HS3 is a local nonlinear optimizer. The
local corridor can reject a feasible route. More than 24 sampled shapes use a
conservative bounding-box envelope for swept spatial seeds. This envelope can
reject a feasible inner route. It cannot admit a route through protected
geometry. Independent continuous validation remains the authority for every
success result.

The planner claims only the earliest independently validated local HS3 result
from the deterministic seeds that it attempted. It does not claim global time
optimality or global path completeness.
