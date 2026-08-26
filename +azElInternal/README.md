# Shared planner internals

`azElInternal` contains method-neutral implementation details used by the HS3
planner. It is the single owner of canonical request normalization, endpoint
checks, obstacle preparation and interpolation, boundary traversal, signed
clearance, option merging, logical normalization, goal interpolation,
polynomial evaluation, power-to-Bernstein conversion, and the sub-interval
hull maps that bound a polynomial continuously between constraint times.

Shared ownership also includes:

- bounded topology and time-expanded seed generation;
- conservative dense-history envelopes and endpoint-safe clustering;
- convex decomposition, seed-corridor construction, certification, and
  obstacle-envelope containment;
- stable success/failure result schemas; and
- exact polynomial jerk integration and reusable comparison utilities.

Root `validateAzElTrajectory` is the final trajectory validator.
`azElPlannerMethods.hs3.validateTrajectory` forwards to it and does not own an
alternate rule set.

The remaining geometry is intentionally narrow:

- `geometry`: canonical boundary conversion, deterministic edge traversal,
  and vectorized signed polygon clearance;
- `obstacles`: immutable preparation and shape-at-time interpolation used by
  planning, querying, validation, and plotting.

The public dispatcher calls `azElPlannerMethods.hs3.plan`. HS3 consumes the
neutral contracts in this package while its Hermite-Simpson transcription and
nonlinear solve remain in the method package.
