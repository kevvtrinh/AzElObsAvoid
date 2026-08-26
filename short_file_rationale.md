# Why short production MATLAB files remain separate

Production code is organized into six flat, high-level Az/El packages plus the
frozen, dimension-neutral `+hs3` engine. Root MATLAB files are stable public
entry points. Tests, examples, benchmarks, the interactive sandbox, and user
data are outside this ownership audit.

| Package | Single responsibility |
| --- | --- |
| `+azElInput` | Normalize and validate public planner requests and options. |
| `+azElObstacles` | Prepare and query static or time-varying obstacles. |
| `+azElGeometry` | Interpret boundaries and perform geometry operations. |
| `+azElSearch` | Generate topology seeds and certify seed corridors. |
| `+azElPlanner` | Adapt Az/El requests to HS3 and assemble planner results. |
| `+azElPlotting` | Render returned results without rerunning planning. |
| `+hs3` | Solve and validate dimension-neutral polynomial motion problems. |

No production package contains another directory. Production MATLAB basenames
are unique, so a responsibility cannot be reached through duplicate package
implementations.

## The HS3 boundary

`+hs3` is frozen as an independent numerical engine. Its state, limits,
polynomial, and path-constraint schemas use caller-defined coordinates and
units. It contains no Az/El, obstacle, visibility, topology, plotting, or
scenario knowledge.

`+azElPlanner/solveSeed.m` owns the adapter from an Az/El seed corridor to the
dimension-neutral HS3 optimization problem. Numerical optimization is invoked
only through `hs3.optimize`; exact polynomial reconstruction and evaluation
are invoked through `hs3.reconstructPolynomial` and `hs3.evaluatePolynomial`.
The adapter restores unit-bearing Az/El field names for the stable planner
result. It does not duplicate engine mathematics.

## Stable public boundaries

Root functions remain separate because they are the documented API for
obstacle construction, planning, interception, validation, plotting, and
queries. `planAzElMotion.m` remains the single public planner entry point.
`plotAzElMotion.m` remains a compatibility facade over
`azElPlotting.plotMotion`, which keeps graphics out of search and solver code.

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
