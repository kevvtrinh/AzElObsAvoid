# HS3 refactor verification

## Environment

- Branch: `hs3-refactor`
- Starting commit: `e46ccae6343a8127d303211a0d1134754a847bc2`
- Runtime: MATLAB R2024b Update 4
- Solver: Optimization Toolbox `fmincon`

## Size checkpoints

| Checkpoint | Production | Examples | Tests | Other | Total |
| --- | ---: | ---: | ---: | ---: | ---: |
| Baseline | 18,090 | 2,989 | 2,521 | 836 | 24,436 |
| Legacy removal checkpoint | 4,069 | 897 | 370 | 0 | 5,336 |
| Compact HS3 commit | 4,135 | 922 | 486 | 0 | 5,543 |
| Visibility correction | 4,576 | 922 | 496 | 0 | 5,994 |

The final repository has 16 production MATLAB files. The target was 8 to 12.
The four-file exception preserves the small public obstacle constructors,
normalizer, inflation function, and query interface. Merging these interfaces
into the planner would reduce the file count but would mix obstacle ownership
with optimization. All hard line limits are satisfied.

Two production files exceed the 700-line target. Both are below the 900-line
hard limit. `+azElInternal/solveAzElHs3.m` has 726 lines. It owns one decision
layout, the integrated third-order chain, continuous polynomial bounds, the
frozen local corridor, and candidate reconstruction. The corrected
`+azElInternal/generateAzElTopologySeeds.m` has 877 lines. It owns the shared
protected-boundary candidates, exact spatial visibility edges, distinct-route
search, forward time layers, motion edges, wait edges, and bounded search
diagnostics. Splitting either file would add another internal representation
and could make reconstruction or graph diagnostics inconsistent.

## Final headless example results

All examples used jerk constraints. `vmax`, `amax`, and `jmax` are maximum
absolute sampled component values. Continuous polynomial certificates also
passed for each successful result.

| Example | Planner / validation | Seed | Duration (s) | Seed length (deg) | Motion length (deg) | vmax | amax | jmax | Collision / kinematic | Runtime (s) | Reason |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | --- |
| `exampleObstacleFreeAzElMotion` | success / pass | 1/1 | 8 | 4.472136 | 4.472136 | 0.937934 | 0.360067 | 0.461151 | pass / pass | 2.013279 | `goalReached` |
| `exampleAzElPlanning` | success / pass | 2/3 | 12 | 11.152120 | 11.714168 | 1.563224 | 0.465244 | 0.493145 | pass / pass | 30.387430 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | success / pass | 2/3 | 16 | 13.098463 | 14.035227 | 1.406249 | 0.290373 | 0.266976 | pass / pass | 35.581630 | `goalReached` |
| `exampleMovingBarrierWait` | success / pass | 2/2 | 12 | 10 | 10.162733 | 1.730429 | 0.557482 | 0.612760 | pass / pass | 34.789296 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | success / pass | 1/3 | 15 | 12 | 12.021127 | 1.495556 | 0.309236 | 0.214327 | pass / pass | 35.307706 | `goalReached` |
| `exampleAlternatingSlalom` | success / pass | 2/3 | 22 | 16.019320 | 16.690230 | 1.156965 | 0.400538 | 0.406357 | pass / pass | 41.595010 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | success / pass | 1/1 | 6.275806 | 10.097524 | 7.342217 | 1.993137 | 1 | 2 | pass / pass | 5.407495 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | success / pass | 2/3 | 15 | 12.700722 | 13.090750 | 1.500732 | 0.317475 | 0.226989 | pass / pass | 35.537637 | `goalReached` |
| `exampleNoPathAzElMotion` | failure / pass | 0/1 | `NaN` | `NaN` | `NaN` | `NaN` | `NaN` | `NaN` | not available | 13.115984 | `noValidatedSeed` |

The moving-wait example selected the time-layer `directWait` seed. The expected
failure created a hidden diagnostic figure. The figure title contained
`noValidatedSeed` and the visibility expansion count of 3. A visible obstacle-free
run created the workspace, kinematic, and animation figures.

## Automated and static checks

- Code Analyzer: 28 files, 0 issues.
- Focused tests: 19 passed, 0 failed, 0 incomplete, 52.002326 seconds.
- Legacy MATLAB dependency search: 0 matches.
- Expected no-path result: stable failure without an exception.
- `git diff --check`: passed.

The tests cover the analytic jerk chain, fixed and earliest arrival, nonzero
endpoint state, mesh refinement, opposite-side seeds, selection order, static,
translating, and deforming polygons, moving-time queries, waiting-seed freedom,
between-node collision and velocity violations, safety-margin idempotence,
azimuth wrapping on and off, deterministic repetition, planning-time failure,
no-path diagnostics, and moving-target adaptation.

## Removed stack

The change deletes 33 files. The deleted responsibilities are the snapshot
visibility graphs, visibility-event selection, safe-interval and SIPP search,
the space-time visibility forwarding graph, the old direct-collocation solver,
packed moving-obstacle queries, the old collision certifier, parallel mode,
two obsolete benchmarks, redundant examples and geographic helpers, three
legacy test suites, scratch visualization, and compatibility validation and
animation layers.

Removed option families include visibility resolution and snapshots, event
detection, space-time graph layers and candidates, SIPP intervals and search,
direct-collocation seed budgets, parallel execution, solver cascades, old
corridor-repair counts, and compatibility validation tolerances. The remaining
options describe the goal policy, workspace, bounded seed set, one HS3 mesh,
one solver, independent collision resolution, and display-independent runtime.

## Limits and claim

- The seed generator uses exact spatial visibility checks on protected swept
  boundaries. Moving-obstacle seeds use at most 17 forward time layers,
  straight motion edges, wait edges, conservative per-axis velocity bounds,
  and 13 trajectory-time occupancy samples per motion edge. The independent
  continuous validator remains the authority for a returned trajectory.
- A finite seed set can miss a feasible obstacle topology.
- The local frozen corridor can reject a feasible route. Independent validation
  never converts this rejection to success.
- Adjacent slices with different topology use a conservative union.
- No replacement performance benchmark was added because the deleted
  benchmarks measured removed algorithms.
- Octave was not used. MATLAB is the behavioral reference.

The result is the earliest independently validated local HS3 solution found
from the finite deterministic seed set that was attempted. The planner does
not claim global time optimality, global path completeness, or a proof that a
failed seed set means that no feasible trajectory exists.
