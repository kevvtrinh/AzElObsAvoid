# HS3 plan completion verification

## Environment

- Worktree: `hs3-plan-complete`
- Branch: `hs3-plan-complete`
- Base: `origin/hs3-refactor` at `895eb1a`
- Runtime: MATLAB R2024b Update 4
- Solvers and data: Optimization Toolbox and Mapping Toolbox

This worktree is separate from the existing `hs3-refactor` worktree.

## Final size

| Category | Files | MATLAB lines |
| --- | ---: | ---: |
| Production | 16 | 4,660 |
| Examples and private example helpers | 23 | 2,987 |
| Tests | 1 | 494 |
| Total | 40 | 8,141 |

Both hard limits pass. Production is below 5,000 lines. The complete maintained
MATLAB repository is below 9,000 lines. Two production files exceed 700 lines,
but both are below the 900-line hard limit:

- `+azElInternal/generateAzElTopologySeeds.m`: 869 lines;
- `+azElInternal/solveAzElHs3.m`: 735 lines.

The seed generator owns one bounded graph, seed ordering, and search
diagnostics. The solver owns one HS3 decision layout, corridor constraints,
polynomial reconstruction, and solver diagnostics. Splitting either file would
add another internal representation.

## Final headless example results

All runs used a finite jerk constraint. `P/V` means planner success and
independent example-validation pass. Failed plans use `NaN` for unavailable
motion metrics. Runtime includes example setup and independent validation.

| Example | P/V | Seed | Polyline (deg) | Motion (deg) | Duration (s) | Collision / kinematic | Runtime (s) | Reason |
| --- | --- | ---: | ---: | ---: | ---: | --- | ---: | --- |
| `exampleObstacleFreeAzElMotion` | 1/1 | 1/1 | 4.472136 | 4.472136 | 8 | pass / pass | 3.711466 | `goalReached` |
| `exampleAzElPlanning` | 1/1 | 2/3 | 11.152120 | 11.637799 | 12 | pass / pass | 31.825646 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | 1/1 | 2/3 | 13.098463 | 13.907652 | 16 | pass / pass | 37.178248 | `goalReached` |
| `exampleMovingBarrierWait` | 1/1 | 1/2 | 10 | 10.955590 | 12 | pass / pass | 19.165248 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | 1/1 | 1/3 | 12 | 12.026889 | 15 | pass / pass | 66.268918 | `goalReached` |
| `exampleAlternatingSlalom` | 1/1 | 2/3 | 16.019320 | 16.401997 | 22 | pass / pass | 185.549365 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | 1/1 | 1/1 | 10.097524 | 7.342217 | 6.275806 | pass / pass | 20.399519 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | 1/1 | 2/3 | 12.700722 | 13.021914 | 15 | pass / pass | 80.164244 | `goalReached` |
| `exampleNoPathAzElMotion` | 0/1 | 0/1 | `NaN` | `NaN` | `NaN` | not available | 22.382117 | `planningTimeLimit` |
| `exampleInterceptMovingTargetAtSetTime` | 1/1 | 1/1 | 9.538941 | 9.538941 | 12 | pass / pass | 5.366669 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | 1/1 | 2/5 | 24.035785 | 24.323592 | 22.875112 | pass / pass | 122.871669 | `goalReached` |
| `exampleTargetExitsObstacle` | 1/1 | 1/1 | 19.824387 | 23.173633 | 24 | pass / pass | 67.319569 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | 1/1 | 1/1 | 13.341664 | 15.491997 | 20.869565 | pass / pass | 120.961669 | `goalReached` |
| `exampleFourAcceleratingCircles` | 1/1 | 1/1 | 20 | 21.065053 | 22 | pass / pass | 48.037667 | `goalReached` |
| `exampleFortyMovingCircleGrid` | 0/1 | 0/2 | `NaN` | `NaN` | `NaN` | not available | 39.677951 | `planningTimeLimit` |
| `exampleMovingDeformingUSOutlineVisibility` | 0/1 | 0/1 | `NaN` | `NaN` | `NaN` | not available | 216.335725 | `planningTimeLimit` |
| `exampleOpeningUShapedAzElTimeSpace` | 1/1 | 1/2 | 10 | 10 | 18 | pass / pass | 137.611569 | `goalReached` |
| `exampleUSOutlineExtremeVisibility/Hawaii` | 0/1 | 0/1 | `NaN` | `NaN` | `NaN` | not available | shared | `planningTimeLimit` |
| `exampleUSOutlineExtremeVisibility/Croatia` | 1/1 | 1/1 | 6.7 | 7.959419 | 120 | pass / pass | shared | `goalReached` |
| `exampleUSOutlineExtremeVisibility/Philippines` | 0/1 | 0/1 | `NaN` | `NaN` | `NaN` | not available | shared | `planningTimeLimit` |

The geographic sequence runtime was 141.716411 s. Hawaii and the Philippines
are declared bounded-failure stress cases. The 40-circle and moving U.S. cases
are also declared bounded-failure stress cases. These results do not prove that
no feasible path exists.

The moving-barrier trajectory crossed the barrier azimuth at 7.25 s, after the
barrier started to open. The timed-opening example independently verified that
the nominal direct timing collided and that the returned path used the later
opening.

## Plot verification

- Hidden failure run: two figures. The workspace and 3-D visibility figures
  existed. No motion-only figure existed. The title contained
  `planningTimeLimit`.
- Explicit visible success run: four visible figures. Workspace, 3-D
  visibility, kinematics, and animation handles were valid.
- Zero-input default success run: four figures with plotting enabled and
  `FigureVisible="on"`.

The workspace plot shows original and protected obstacles, accepted and
rejected graph edges, explored nodes, all topology seeds, the selected seed,
and the returned trajectory. The 3-D plot shows azimuth, elevation, time,
obstacle slices, graph nodes, and seed timing. The kinematic plot shows
position, velocity, acceleration, jerk, and applicable limits. Failure plots
show the reason, counts, and best partial seed when data is available.

## Automated checks

- Code Analyzer: 40 files, 0 issues.
- Focused tests: 20 passed, 0 failed, 0 incomplete, 69.891874 s.
- Legacy SIPP, snapshot, safe-interval, and parallel-seed search: 0 matches.
- `git diff --check`: passed before the documentation update and is rerun at
  final audit.

The focused tests cover analytic jerk integration, terminal equalities,
between-knot motion limits, earliest arrival, topology diversity, selection
order, static and moving geometry, waiting, between-point collision,
safety-margin provenance, deterministic behavior, stable failure diagnostics,
and topology change at an exact source time.

## Existing-file growth above 50 added lines

- `generateAzElTopologySeeds.m`: +54, -31, net +23. Added bounded graph
  diagnostics, direct-wait early search, and capped time sampling.
- `solveAzElHs3.m`: +51, -11, net +40. Added input-driven corridor density,
  topology-change support, obstacle-motion clearance, and a bounded seed-rate
  guard.
- `resolveAzElExampleOptions.m`: +99, -21, net +78. Centralized uniform
  example and plot controls and documented aliases.
- `plotAzElMotion.m`: +283, -53, net +230. Added reusable workspace,
  visibility-time, kinematic, animation, and failure diagnostic plots.

## Limits and claim

The result is the earliest independently validated local HS3 solution found
from the finite deterministic seed set that was attempted. The planner does not
claim global time optimality, global path completeness, or a proof that a
failed finite seed set means that no feasible motion exists.

The Mapping Toolbox supplied the geographic source files. MATLAB, Optimization
Toolbox, and graphics were available. Octave was not used because MATLAB is the
behavioral reference.
