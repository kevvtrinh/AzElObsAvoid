# BMTP engine map

`bmtpEngine.solve` is the engine entry point. The nested MATLAB packages group
the implementation by the stage that owns each responsibility:

- `+pipeline` builds the solve request, warm start, prepared motion, and public
  motion output. It also owns timed refinement between the initial solve and
  final proof.
- `+motion` constructs and evaluates jerk-limited Bezier/power-polynomial
  motion, including endpoint controls, subdivision, and duration estimates.
- `+optimization` assembles and runs the conic trajectory programs. It owns
  solver diagnostics and the static, active-pair, and timed solve variants.
- `+separation` constructs, selects, solves, and verifies separating planes
  against static or moving obstacle regions.
- `+validation` computes shared coordinate tolerances and independently checks
  the final prepared motion.

## Pipeline flow

```text
bmtpEngine.solve
  -> pipeline.createSolveRequest
  -> pipeline.createWarmStart
  -> optimization solve variants
       -> motion primitives
       -> separation planes
  -> pipeline.prepareFinalMotion
  -> validation.checkFinalMotion
  -> pipeline.createMotionOutput
```

Folder names begin with `+` because they are MATLAB packages. Call internal
functions with their full names, for example
`bmtpEngine.motion.evaluatePolynomial`.
