# Compact corridor replacement closeout

## Objective

Replace the legacy corridor motion stack with one compact, input-driven C3/C4
implementation, support nonzero endpoint velocity and acceleration, and meet
or beat the frozen maintained-example and repeated-turn/hairpin results within
the user-authorized 1,200 non-comment-line limit.

## Implemented state

- Obstacle-path ownership is 1,023 nonblank, noncomment MATLAB lines across
  `solveCompactC3`, `solveCompactC3Candidate`, `runCorridorPlanner`,
  `buildFixedDurationAffineModel`, and `expandRouteClearance`.
- Static straight requests retain the small exact analytic quintic. Other
  static, moving, deforming, and timed-hold routes use compact C3/C4 motion.
- Endpoint position, velocity, and acceleration are enforced for both initial
  and terminal states. A structurally different detour regression covers
  nonzero derivatives.
- Duration retries reuse one affine basis. Diagnostics aggregate every trial,
  QP, basis build, and validation attempt.
- `solveCorridorQuintic`, `retimeDynamicRoute`, `optimizeExactTraversal`, and
  `spanTimeDemand` are deleted with no remaining executable callers.
- Both scaling benchmarks call the same production compact-candidate adapter
  and reject route truncation rather than benchmarking a changed topology.

## Verification state

- The serial 18-example gate passed all 18 outcomes: 17 independently
  validated successes and the expected validated no-path result. Every
  successful duration met or beat the frozen legacy duration.
- The final-source 1/5/10/20-turn and 12-hairpin gate passed independent
  validation and beat every frozen legacy duration.
- The first full-suite attempt exposed two duplicate-basis accounting failures.
  The redundant rebuild was removed without deleting the two-bracket behavior;
  both affected test files then passed 16/16.
- A fresh complete test run passed 133/133. The final 18-row maintained-example
  CSV capture, 99-file Code Analyzer pass, and success/failure graphics smokes
  also passed. Only final staging inspection and commit remain.

## Boundaries and next phase

The compact search remains finite and supplies no global optimality or
completeness certificate. Runtime is reported per case and is not represented
as a uniform speedup. Unrelated untracked `docs/` and `tmp/` content remains
untouched.

After this compact checkpoint is committed, begin a separate bounded HS3
replacement: at most 1,200 HS3-owned noncomment lines, identical inputs and
independent validation, and per-example arrival/runtime comparison against the
committed compact baseline.
