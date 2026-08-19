# Plan 502 HS3 verification

## Environment

- Branch: `plan-502-implementation`
- Starting commit: `e46ccae6343a8127d303211a0d1134754a847bc2`
- Runtime: MATLAB R2024b Update 4
- Solver: Optimization Toolbox `fmincon`

## Size checkpoints

| Checkpoint | Production | Examples | Tests | Total |
| --- | ---: | ---: | ---: | ---: |
| Baseline | 18,090 | 2,989 | 2,521 | 24,436 |
| Plan 502 final, nonblank lines | 5,766 | 2,878 | 622 | 9,266 |
| Plan 502 final, physical lines | 5,997 | 3,222 | 656 | 9,875 |

The final repository has 22 production MATLAB files. The production target is
6,000 physical lines. The complete target is 10,500 physical lines. Both
targets pass. The public obstacle constructors, normalizer, inflation, query,
planner, validator, and plotter keep separate responsibilities. Six small
internal files own the continuous seed-envelope and clustering invariants.

Three production files exceed the 700-line target. No production file exceeds
the 900-line limit. `+azElInternal/solveAzElHs3.m` and
`+azElInternal/generateAzElTopologySeeds.m` each have 900 physical lines.
`planAzElMotion.m` has 702 physical lines. The solver owns one decision layout,
the integrated third-order chain, continuous polynomial bounds, the frozen
local corridor, and candidate reconstruction. The seed generator owns one
bounded topology graph, spatial and timed routes, and search diagnostics. The
public planner owns input resolution, candidate selection, refinement, verbose
reporting, and the stable result contract. Splitting these three execution
paths would duplicate state and diagnostic assembly.

## Final headless example results

All examples used jerk constraints. Polyline length is the selected seed
length. Smoothed length is the final motion length. Each successful result
passed the collision and kinematic certificates.

| Example | Planner / validation | Polyline (deg) | Smoothed (deg) | Minimum duration (s) | Collision / kinematic | Runtime (s) | Reason |
| --- | --- | ---: | ---: | ---: | --- | ---: | --- |
| `exampleObstacleFreeAzElMotion` | success / pass | 4.472136 | 4.472136 | 8 | pass / pass | 7.421851 | `goalReached` |
| `exampleAzElPlanning` | success / pass | 11.152120 | 11.430274 | 12 | pass / pass | 6.832057 | `goalReached` |
| `exampleUShapedAzElTimeSpace` | success / pass | 13.098463 | 13.633792 | 16 | pass / pass | 12.883694 | `goalReached` |
| `exampleMovingBarrierWait` | success / pass | 10 | 10 | 12 | pass / pass | 19.674199 | `goalReached` |
| `exampleMovingCircleNoAzimuthWrap` | success / pass | 12 | 12.076383 | 15 | pass / pass | 14.929877 | `goalReached` |
| `exampleAlternatingSlalom` | success / pass | 21.593158 | 22.843229 | 22 | pass / pass | 29.413146 | `goalReached` |
| `exampleInterceptMovingTargetEarliest` | success / pass | 10.097524 | 7.342218 | 6.275808 | pass / pass | 10.547406 | `goalReached` |
| `exampleDenseConcaveAzElMotion` | success / pass | 12.700722 | 12.922625 | 15 | pass / pass | 9.870478 | `goalReached` |
| `exampleNoPathAzElMotion` | expected failure / pass | `NaN` | `NaN` | `NaN` | not available | 20.872299 | `planningTimeLimit` |
| `exampleFourAcceleratingCircles` | success / pass | 20 | 23.014367 | 22 | pass / pass | 176.305066 | `goalReached` |
| `exampleFortyMovingCircleGrid` | success / pass | 110.719602 | 119.913575 | 200 | pass / pass | 14.448237 | `goalReached` |
| `exampleInterceptMovingTargetAtSetTime` | success / pass | 9.538941 | 9.538941 | 12 | pass / pass | 4.775197 | `goalReached` |
| `exampleMovingDeformingUSOutlineVisibility` | success / pass | 63.084805 | 69.932965 | 300 | pass / pass | 53.478524 | `goalReached` |
| `exampleOpeningUShapedAzElTimeSpace` | success / pass | 10 | 10 | 15.000998 | pass / pass | 121.012455 | `goalReached` |
| `exampleStraightTargetAlternatingOcclusion` | success / pass | 20.285321 | 14.030278 | 20.869565 | pass / pass | 63.571625 | `goalReached` |
| `exampleTargetExitsObstacle` | success / pass | 19.824387 | 22.884476 | 24 | pass / pass | 20.278459 | `goalReached` |
| `exampleTwoOpposingUVisibilityGraph` | success / pass | 24.035785 | 24.298600 | 22.876125 | pass / pass | 91.218125 | `goalReached` |
| `exampleUSOutlineExtremeVisibility` | success / pass | 22.239464 | 26.834575 | 7.239888 | pass / pass | 333.423481 | `goalReached` |

Fixed-arrival checks passed in `exampleAzElPlanning`,
`exampleMovingBarrierWait`, `exampleInterceptMovingTargetAtSetTime`, and the
moving U.S. outline example. Each terminal-time error was zero. Earliest-arrival
checks passed in `exampleAzElPlanning`, `exampleMovingBarrierWait`, and
`exampleInterceptMovingTargetEarliest`. The moving-wait example selected a
causal `directWait` seed.

The expected failure created two diagnostic figures. They showed the original
and protected wall, explored nodes, accepted and collision-rejected edges, the
frontier, the best partial route, and the `planningTimeLimit` reason. A visible
obstacle-free run created three figures for workspace, kinematics, and motion.

## Automated and static checks

- Code Analyzer: 46 files, 0 issues.
- Focused tests: 25 passed, 0 failed, 0 incomplete, approximately 44 seconds.
- Legacy MATLAB dependency search: 0 matches.
- Expected no-path result: stable failure without an exception.
- Direct public `Verbose = true` call: planner graph, seed, validation,
  arrival, violation, reason, and final result tokens present. The `fmincon`
  iteration table was absent.
- Fixed and earliest arrival: passed on different static, moving-obstacle, and
  moving-target examples.
- `git diff --check`: passed after the final documentation update.

The tests cover the analytic jerk chain, fixed and earliest arrival, nonzero
endpoint state, mesh refinement, opposite-side seeds, static, translating, and
deforming polygons, topology-change unions, moving-time queries, causal waiting
seeds, continuous seed corridors, dense-envelope containment, nearby-obstacle
clustering, between-node collision and velocity violations, safety-margin
idempotence, azimuth wrapping, deterministic repetition, planning-time failure,
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
  and 13 trajectory-time occupancy samples per motion edge. Timed seeds retain
  a causal lower-bound schedule. The independent continuous validator remains
  the authority for a returned trajectory.
- A dense swept history can use a conservative coarse directional support
  polygon for seed generation. A connected group of at least three nearby
  swept regions can use a conservative convex hull when
  `SeedClusterDistance_deg` is positive. HS3 and validation always use the
  original protected obstacle history.
- Seed-only convex envelopes use a continuous half-plane corridor and an
  independent containment and polynomial-clearance certificate.
- A finite seed set can miss a feasible obstacle topology.
- The local frozen corridor can reject a feasible route. Independent validation
  never converts this rejection to success.
- Adjacent slices with different topology use a stationary conservative union
  inside the source interval. Source event times split corridor checks.
- No replacement performance benchmark was added because the deleted
  benchmarks measured removed algorithms.
- Octave was not used. MATLAB is the behavioral reference.

The result is the earliest independently validated local HS3 solution found
from the finite deterministic seed set that was attempted. The planner does
not claim global time optimality, global path completeness, or a proof that a
failed seed set means that no feasible trajectory exists.
