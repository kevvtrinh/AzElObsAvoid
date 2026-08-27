# Standalone HS3 engine

This normal folder is the dimension-neutral kinematic engine extracted from
the azimuth/elevation planner. Its single public entry is:

```matlab
trajectory = solveTrajHS3( ...
    initialState, terminalState, limits, options, pathConstraints);
```

The engine owns quadratic jerk controls, exact polynomial integration and
sampling, continuous kinematic bounds, fixed-time QP solving, free-time NLP
solving, and HS3 diagnostics. `+hs3Internal/` contains those neutral
implementation functions. The public entry may consume optional
coordinate-space affine point constraints or exact single-segment interval
constraints through Bernstein hulls.

It must not depend on azimuth/elevation semantics, obstacle records, visibility
graphs, homology, seed generation or ranking, moving-target interpretation,
planner result assembly, validation outside the kinematic contract, examples,
or plotting.

The production azimuth/elevation adapter retains ownership of obstacle,
topology, corridor, and result assembly. It delegates the dimension-neutral
optimization and polynomial mathematics to this package. The extraction was
accepted against frozen commit `4827e47` only after the three-run behavioral,
numeric, and runtime parity gate reported zero mismatches.
