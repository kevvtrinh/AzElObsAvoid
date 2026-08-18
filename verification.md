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
| Final | 4,135 | 922 | 486 | 0 | 5,543 |

The final repository has 16 production MATLAB files. The target was 8 to 12.
The four-file exception preserves the small public obstacle constructors,
normalizer, inflation function, and query interface. Merging these interfaces
into the planner would reduce the file count but would mix obstacle ownership
with optimization. All hard line limits are satisfied.

Only `+azElInternal/solveAzElHs3.m` exceeds 700 lines. It has 726 lines. It
owns one decision layout, the exact integrated third-order chain, continuous
polynomial bounds, the frozen local corridor, and candidate reconstruction.
These parts use one coefficient convention. Splitting them would create an
additional internal data interface and a risk of inconsistent reconstruction.
The file is below the 900-line approval limit.

## Final headless example results

All examples used jerk constraints. `vmax`, `amax`, and `jmax` are maximum
absolute sampled component values. Continuous polynomial certificates also
passed for each successful result.

| Example | Planner / validation | Seed | Duration (s) | Seed length (deg) | Motion length (deg) | vmax | amax | jmax | Collision / kinematic | Runtime (s) | Reason |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | --- |
| `exampleObstacleFreeAzElMotion` | success / pass | 1/1 | 8 | 4.472136 | 4.472136 | 0.937934 | 0.360067 | 0.461151 | pass / pass | 2.455096 | `goalReached` |
| `exampleAzElPlanning` | success / pass | 3/3 | 12 | 12.151626 | 11.759649 | 1.563224 | 0.476183 | 0.573459 | pass / pass | 30.954293 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | success / pass | 3/3 | 16 | 14.201201 | 14.038283 | 1.406250 | 0.291488 | 0.290995 | pass / pass | 36.290954 | `goalReached` |
| `exampleMovingBarrierWait` | success / pass | 2/2 | 12 | 10 | 10.014993 | 1.729848 | 0.557633 | 0.613415 | pass / pass | 35.334758 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | success / pass | 1/3 | 15 | 12 | 12.021262 | 1.500562 | 0.310125 | 0.214260 | pass / pass | 35.950592 | `goalReached` |
| `exampleAlternatingSlalom` | success / pass | 2/3 | 22 | 18.113616 | 16.372897 | 1.111631 | 0.314600 | 0.402326 | pass / pass | 42.592004 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | success / pass | 1/1 | 6.275806 | 10.097524 | 7.342217 | 1.993137 | 1 | 2 | pass / pass | 5.990194 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | success / pass | 2/3 | 15 | 14.116797 | 13.089912 | 1.500098 | 0.314971 | 0.223232 | pass / pass | 36.373486 | `goalReached` |
| `exampleNoPathAzElMotion` | failure / pass | 0/1 | `NaN` | `NaN` | `NaN` | `NaN` | `NaN` | `NaN` | not available | 13.941313 | `noValidatedSeed` |

The moving-wait example selected the `directWait` seed. The expected failure
created a hidden diagnostic figure with 713 explored nodes. The figure title
contained `noValidatedSeed`. A visible obstacle-free run created the workspace,
kinematic, and animation figures.

## Automated and static checks

- Code Analyzer: 28 files, 0 issues.
- Focused tests: 19 passed, 0 failed, 0 incomplete, 50.091938 seconds.
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

- The coarse seed graph uses a swept spatial occupancy grid. It is not a full
  time-expanded reachability graph. One input-driven waiting variant supplies
  the bounded temporal alternative.
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
