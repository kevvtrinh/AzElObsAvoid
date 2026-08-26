# Planner Measurement Protocol

Use this protocol only after a focused candidate has shown enough promise to
justify formal measurement.

## Freeze Equivalence

Hold constant the branch baseline, MATLAB/toolbox versions, hardware, seed,
parallel state, plotting state, obstacle data, safety margin, initial and goal
states, goal-time policy, limits, unrelated options, and independent validator.
Interleave baseline and candidate runs when environment drift is plausible.

Invalid, unsupported, no-path, fallback, and timeout results remain in the
evidence. A faster invalid or easier problem is not an improvement.

## Measure The Owning Work

Start with end-to-end wall time, then attribute only enough detail to identify
the owner. Use exclusive top-level stage times where possible and label nested
or cross-cutting timers as non-additive. Include fallback and final validation
time.

Connect measurements to their driving scale, such as obstacle vertices,
history slices, topology states, candidate routes, spline spans, decision
variables, constraint rows, solver evaluations, collision queries, or repeated
solves. Test increasing sizes when the change can alter asymptotic behavior.

For minimum-arrival planning, final independently validated arrival is the
primary quality measure unless the user declares another objective. Preserve
collision, workspace, velocity, acceleration, jerk, endpoint, and goal-policy
gates. Compare complete planners separately from isolated motion builders.

## Statistics And Decision

Record the first run and repeated median, minimum, maximum, and worst case when
practical. Predeclare cases and random seeds. Do not select only favorable
cases, average away failures, or call a sub-tolerance change meaningful.

For a production method decision, report per case:

```text
Method, Valid, TerminationReason, ArrivalTime_s, QualityMetrics,
TotalWallTime_s, StageTimes, FallbackUsed, InputScale, Notes
```

Also report added and removed production lines, options, dependencies,
artifacts, and failure modes. Apply the current `AGENTS.md` size rules using the
smallest verified improvement in the declared representative set.

Conclude with accept or reject, the primary metric, worst regression, validity
result, evidence limits, and exact recovery performed for a rejection.
