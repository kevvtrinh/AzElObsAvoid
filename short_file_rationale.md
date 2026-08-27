# Why short production MATLAB files remain separate

Production code is organized into the `planAzElMotion/` and `hs3/` product
folders. The Az/El product contains six flat packages; the HS3 product contains
one public entry and one neutral internal package. Tests, examples, benchmarks,
the interactive sandbox, and user data are outside this ownership audit.

| Package | Single responsibility |
| --- | --- |
| `+azElInput` | Normalize and validate public planner requests and options. |
| `+azElObstacles` | Construct, combine, prepare, and query obstacles. |
| `+azElGeometry` | Interpret boundaries and perform geometry operations. |
| `+azElSearch` | Generate topology seeds and certify seed corridors. |
| `+azElPlanner` | Adapt Az/El requests to HS3 and assemble planner results. |
| `+azElPlotting` | Render returned results without rerunning planning. |
| `hs3/+hs3Internal` | Implement dimension-neutral polynomial motion. |

Each product has only one package level. Production MATLAB basenames are
unique, so a responsibility cannot be reached through duplicate
implementations.

## The HS3 boundary

`hs3/` is an independent numerical engine. Its state, limits,
polynomial, and path-constraint schemas use caller-defined coordinates and
units. It contains no Az/El, obstacle, visibility, topology, plotting, or
scenario knowledge.

`+azElPlanner/solveSeed.m` owns the adapter from an Az/El seed corridor to the
dimension-neutral HS3 optimization problem. Numerical optimization is invoked
only through `hs3Internal.optimize`; exact polynomial reconstruction and
evaluation are invoked through `hs3Internal.reconstructPolynomial` and
`hs3Internal.evaluatePolynomial`.
The adapter restores unit-bearing Az/El field names for the stable planner
result. It does not duplicate engine mathematics.

## Stable public boundaries

`planAzElMotion/planAzElMotion.m` remains the single public Az/El planner entry
point. Obstacle construction and queries are qualified `azElObstacles.*`
package calls, plotting uses `azElPlotting.plotMotion`, and
`hs3/solveTrajHS3.m` is the single public neutral engine entry.

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
