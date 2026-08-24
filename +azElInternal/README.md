# Shared planner internals

`azElInternal` contains method-neutral implementation details shared by the
standalone compact corridor and Hermite-Simpson planners. It is the single
owner of canonical request normalization, endpoint checks, obstacle preparation
and interpolation, boundary traversal, signed clearance, option merging,
logical normalization, goal interpolation, polynomial evaluation, and
power-to-Bernstein conversion.

Shared planner ownership also includes:

- bounded topology and time-expanded seed generation;
- conservative dense-history envelopes and endpoint-safe clustering;
- convex decomposition, seed-corridor construction, certification, and
  obstacle-envelope containment;
- stable success/failure result schemas; and
- exact polynomial jerk integration and reusable comparison utilities.

Root `validateAzElTrajectory` is the one final trajectory validator. The
method-qualified validation functions forward to it and do not own alternate
rules.

The remaining geometry is intentionally narrow:

- `geometry`: canonical boundary conversion, deterministic edge traversal, and
  vectorized signed polygon clearance;
- `obstacles`: immutable preparation and shape-at-time interpolation used by
  planning, querying, validation, and plotting.

The public dispatcher calls either `azElPlannerMethods.corridor.plan` or
`azElPlannerMethods.hs3.plan` directly. Both consume the neutral contracts in
this package, but neither invokes the other. Compact C3/C4 construction stays
in the corridor package; Hermite-Simpson transcription and nonlinear solving
stay in the HS3 package.
