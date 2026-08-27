# Why short production MATLAB files remain separate

Production code is organized into the `+obstacleAvoidance/` namespace and the
`hs3/` engine folder. The obstacle-avoidance product contains six responsibility
subpackages; HS3 contains one public entry and three neutral implementation
subpackages. Tests, examples, benchmarks, the interactive sandbox, and user
data are outside this ownership audit.

| Package | Single responsibility |
| --- | --- |
| `+obstacleAvoidance/+input` | Normalize public requests and options. |
| `+obstacleAvoidance/+obstacles` | Create, combine, prepare, and query obstacles. |
| `+obstacleAvoidance/+geometry` | Interpret boundaries and perform geometry operations. |
| `+obstacleAvoidance/+search` | Create route candidates and certify corridors. |
| `+obstacleAvoidance/+planner` | Adapt requests to HS3 and assemble results. |
| `+obstacleAvoidance/+plotting` | Render results without rerunning planning. |
| `hs3/+hs3Internal/+polynomial` | Own neutral trajectory-polynomial mathematics. |
| `hs3/+hs3Internal/+constraints` | Own neutral continuous constraints. |
| `hs3/+hs3Internal/+solver` | Own fixed- and free-time optimization. |

Production MATLAB basenames are unique, so a responsibility cannot be reached
through duplicate implementations.

## The HS3 boundary

`hs3/` is an independent numerical engine. Its state, limits,
polynomial, and path-constraint schemas use caller-defined coordinates and
units. It contains no Az/El, obstacle, visibility, topology, plotting, or
scenario knowledge.

`+obstacleAvoidance/+planner/solveRouteCandidate.m` owns the adapter from a
route corridor to the dimension-neutral HS3 optimization problem. Numerical
optimization is invoked through `hs3Internal.solver.optimize`; exact
polynomial creation and evaluation use
`hs3Internal.polynomial.createTrajectoryPolynomial` and
`hs3Internal.polynomial.evaluateTrajectoryPolynomial`. The adapter restores
unit-bearing coordinate fields for the stable result without duplicating
engine mathematics.

## Stable public boundaries

`obstacleAvoidance.planTrajectory` is the single public obstacle-avoidance
planner entry. Obstacle construction and queries are qualified
`obstacleAvoidance.obstacles.*` calls, plotting uses
`obstacleAvoidance.plotting.plotTrajectory`, and `hs3/solveTrajHS3.m` remains
the single public neutral engine entry.

## Small internal files

Short files remain when they own one shared contract or mathematical invariant:

- input functions own defaults, logical normalization, endpoint checks, and
  request normalization;
- obstacle and geometry functions own canonical shape interpretation;
- search functions own time layers, topology generation, corridor construction,
  and independent corridor certificates;
- planner functions own result schemas, stage timing, Az/El constraint
  translation, and post-solve validation;
- HS3 functions own generic sensitivities, constraint matrices, optimization,
  reconstruction, evaluation, and validation.

Merging these files would combine distinct ownership or make a public contract
local to one caller. File length alone is not a reason to merge them.
