# Shared public-utility internals

`azElInternal` contains neutral implementation details shared by root public
utilities and the current compact/HS3 composition. It is the single owner of
canonical obstacle preparation and interpolation, boundary traversal, signed
clearance, option merging, logical normalization, goal interpolation,
polynomial evaluation, and power-to-Bernstein conversion.

Shared planner ownership now also includes:

- bounded topology and time-expanded seed generation;
- conservative dense-history envelopes and endpoint-safe clustering;
- convex decomposition, seed-corridor construction, certification, and
  obstacle-envelope containment;
- the stable empty planner/result schemas;
- exact polynomial jerk integration and monotone improvement comparison.

Root `validateAzElTrajectory` is the one final trajectory validator. The
method-qualified validation functions forward to it and do not own alternate
rules.

The remaining geometry is intentionally narrow:

- `geometry`: canonical boundary conversion, deterministic edge traversal, and
  vectorized signed polygon clearance;
- `obstacles`: immutable preparation and shape-at-time interpolation used by
  planning, querying, validation, and plotting.

Small option, logical, goal-position, polynomial, search, corridor, result, and
comparison helpers remain here because compact construction and HS3 improvement
must interpret the same evidence. Compact C3/C4 construction stays in the
corridor package; HS3 collocation and nonlinear solving stay in the HS3
package.

This neutral ownership does not make the current HS3 execution closure
independent of corridor: the root dispatcher still obtains the immutable
compact baseline from `azElPlannerMethods.corridor.plan` before calling
`azElPlannerMethods.hs3.improve`. Neutral extraction of that baseline remains
future work.
