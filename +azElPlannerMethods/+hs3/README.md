# Bounded HS3 composite improver

This folder descends from the Plan-325 HS3 snapshot at commit
`5a067112a9f880d015f52fb97538a99010871478`, but it is no longer a complete
isolated planner. The public `PlannerMethod="hs3"` path first builds an
immutable compact corridor baseline, then passes that result and resolved HS3
options to this package for optional bounded nonlinear work.

## Current entry points

- `plan.m` preserves the method-qualified compatibility call and defaults
  request; planning forwards to the root composition.
- `resolvePlannerOptions.m` owns all HS3 defaults and validation and projects
  only common fields into the compact baseline call.
- `improve.m` records bounded attempts, validates every returned trajectory
  through `validateAzElTrajectory`, and accepts only strict monotone improvement
  over the immutable baseline.
- `validateTrajectory.m` is a compatibility facade over the canonical root
  validator.

There is no method-local moving-target adapter, topology search, stop-at-
waypoint motion family, corridor-certificate stack, result builder, or final
validator. Neutral topology, geometry, corridor, result, and comparison helpers
live in `+azElInternal`; interception and validation live at the root.

## Selection and result source

`planAzElMotion("hs3")` returns HS3-specific defaults with
`EnableHs3Improvement=false`. In this default mode the composition returns the
compact trajectory unchanged, sets `Options.PlannerMethod="hs3"`, and retains
`SelectedMotionSource="corridorQuintic"`.

With improvement enabled, HS3 uses collocation and `fmincon` on deterministic
compact seeds. Invalid, equal, slower, rougher, or timed-out attempts remain in
`CompositionDiagnostics.Hs3.Attempts` and cannot displace a valid baseline.
Only a canonically validated strict improvement sets
`SelectedMotionSource="hs3"`.

`MaximumMeshRefinementPasses` is resolved and echoed for compatibility, but
the current improver explicitly reports `RefinementSupported=false` and does
not perform mesh refinement. The solver timeout is cooperative: setup checks
and the `fmincon` output callback stop future work, but one active function
evaluation can return after the requested deadline, so measured elapsed time
may exceed `MaximumHs3ImprovementTime_s`.

## Size and dependency boundary

The HS3 package has a hard 1,200-noncomment-line ownership cap. The current
recount is **1,200 noncomment lines** across its facade, option owner, improver,
validation facade, and motion internals. This count excludes tests and also
excludes the executed compact baseline and neutral shared dependencies. It is
therefore not a claim that the complete HS3 execution closure is within 1,200
lines.

The current root composition transitively depends on `+corridor`. HS3 cannot
yet be deployed or verified by deleting that folder; neutral extraction of the
compact baseline remains future work. Optimization Toolbox is required only
when HS3 improvement is enabled.
