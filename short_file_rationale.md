# Why short production MATLAB files remain separate

Production code is organized into the `+obstacleAvoidance/` namespace and two
independent packages under `trajectory/`. Tests, examples, benchmarks, the
interactive sandbox, and user data are outside this ownership audit.

| Package | Single responsibility |
| --- | --- |
| `+obstacleAvoidance/+input` | Normalize public requests and options. |
| `+obstacleAvoidance/+obstacles` | Create, combine, prepare, and query obstacles. |
| `+obstacleAvoidance/+geometry` | Interpret boundaries and perform geometry operations. |
| `+obstacleAvoidance/+search` | Create route candidates and certify corridors. |
| `+obstacleAvoidance/+planner` | Select an engine, adapt constrained routes, and assemble results. |
| `+obstacleAvoidance/+plotting` | Render results without rerunning planning. |
| `trajectory/+hs3Engine/+polynomial` | Own HS3 trajectory-polynomial mathematics. |
| `trajectory/+hs3Engine/+constraints` | Own HS3 continuous constraints. |
| `trajectory/+hs3Engine` | Own fixed- and free-time collocation. |
| `trajectory/+ruckigEngine` | Own exact jerk-switching motion. |
| `trajectory/+ruckigEngine/+internal` | Own Ruckig input and result validation. |

Each engine owns its ordinary `solve` and `defaultOptions` names. MATLAB package
qualification makes their ownership explicit without artificial filename
prefixes.

## Engine boundaries

`hs3Engine` is an independent numerical engine. Its state, limits, polynomial,
and path-constraint formats use caller-defined coordinates and units. It
contains no Az/El, obstacle, visibility, topology, plotting, or scenario
knowledge.

`+obstacleAvoidance/+planner/solveRouteCandidate.m` adapts a route corridor to
the dimension-neutral HS3 optimization problem. Numerical optimization uses
`hs3Engine.optimize`; polynomial creation and evaluation use
`hs3Engine.polynomial.createTrajectoryPolynomial` and
`hs3Engine.polynomial.evaluateTrajectoryPolynomial`. The adapter restores
unit-bearing coordinate fields without duplicating engine mathematics.

`ruckigEngine` independently owns exact switching profiles, input
normalization, polynomial evaluation, and result validation. It contains no
HS3, optimizer, Az/El, obstacle, visibility, or topology dependency.

## Stable public boundaries

`obstacleAvoidance.planTrajectory` is the single public obstacle-avoidance
planner entry. Obstacle construction and queries are qualified
`obstacleAvoidance.obstacles.*` calls, plotting uses
`obstacleAvoidance.plotting.plotTrajectory`, and direct dimension-neutral calls
use either `hs3Engine.solve` or `ruckigEngine.solve`.

## Small internal files

Short files remain when they own one shared requirement or mathematical
invariant:

- input functions own defaults, logical normalization, endpoint checks, and
  request normalization;
- obstacle and geometry functions own canonical shape interpretation;
- search functions own time layers, topology generation, corridor construction,
  and independent corridor certificates;
- planner functions own result formats, stage timing, engine selection, Az/El
  translation, and post-solve validation;
- HS3 functions own sensitivities, constraint matrices, optimization,
  reconstruction, evaluation, and validation;
- Ruckig functions own switching families, synchronization, reconstruction,
  and their independent validation.

Merging these files would combine distinct ownership or make a shared
requirement local to one caller. File length alone is not a reason to merge
them.
