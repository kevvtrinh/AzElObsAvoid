---
name: bounded-planner-change
description: Evaluate an uncertain planner algorithm, tuning, optimization, performance refactor, or formal method choice against a working baseline, and retain it only when a declared benefit is verified without unacceptable regression.
---

# Bounded Planner Change

Use this skill for proposed improvements to working planner behavior. Use
`planner-failure-diagnosis` first when the task begins with an unexplained
failure rather than an improvement hypothesis.

## Declare The Retention Gate

Before editing, record:

- the exact baseline revision, dirty state, command, seed, options, and result;
- one primary benefit to improve and the measurement that represents it;
- correctness, physical-validity, interface, diagnostic, and quality invariants;
- the input scale that could change the runtime growth rate;
- the experiment-owned files or hunks that can be removed safely;
- a proof budget of one hypothesis, one isolated implementation, one focused
  comparison, and one structurally different verification case.

Do not treat more code, more metrics, a solver exit flag, or a favorable plot as
an improvement. Sub-tolerance differences and ordinary wall-clock noise are
equivalent results.

## Protect Completeness From Heuristics

Classify every heuristic before retaining it. A heuristic may order work,
propose a warm start, or provide a lower bound. It may not discard a seed,
shorten the admissible time horizon, stop route exploration, or authorize a
no-path result unless the rejected set is excluded by a documented conservative
bound or an authoritative validator.

In particular, a duration estimate is not a feasibility certificate. If an
estimated-time attempt fails construction or validation, preserve the original
horizon and continue the non-pruned attempt. Do not bisect or otherwise infer a
feasibility boundary without proving the required monotonicity.

When the user supplies a diagnosis bundle or other exact failure artifact:

- add its unmodified replay to the focused regression suite before the repair;
- assert planner success and independent validation when the request is known
  to be feasible, rather than merely asserting a changed termination reason;
- retain the artifact test after the fix so the same false negative cannot
  silently return; and
- verify the same invariant on one structurally different case before claiming
  that the fix is general.

Prefer removing an unproven pruning heuristic over adding thresholds or
exceptions. Reintroduce it only through the retention gate with measured runtime
benefit and unchanged completeness on the focused and structurally different
checks.

## Evidence Ladder

1. Run the smallest end-to-end baseline that measures the declared benefit.
2. Implement only the mechanism needed to test the hypothesis.
3. Run the same focused case with identical inputs, validation, and environment.
4. Stop and recover immediately if correctness fails or benefit is absent.
5. If the focused gate passes, run a structurally different case that exercises
   the same input-driven invariant.
6. Only after both gates pass, run the broader tests and maintained examples
   required by `AGENTS.md` for an accepted change.

Read [references/measurement-protocol.md](references/measurement-protocol.md)
for a speedup claim, formal method comparison, scaling claim, or production
growth justified by performance.

## Retention Decision

Keep the change only when the declared benefit is demonstrated above the
measurement noise floor, every applicable independent validity check passes,
and no representative case has an unacceptable quality or worst-case runtime
regression. A correctness fix or requested new capability may justify runtime
cost only when the tradeoff is measured and reported explicitly.

If the gate fails, remove all experiment-only code, options, tests, diagnostic
fields, scripts, and artifacts. Restore only experiment-owned changes; never
use a broad reset that could overwrite unrelated work. Verify the original
baseline command again.

Do not rescue a failed candidate with scenario branches, hidden fallbacks,
special thresholds, additional tuning sweeps, or a new user option whose only
purpose is to avoid the unfavorable result.

## Diagnostic And Record Discipline

Retain instrumentation only when it supports a recurring public diagnostic or
engineering decision and its runtime and memory cost are acceptable. Remove
temporary tracing after the measurement.

Do not append failed experimental measurements to the current benchmark record
or leave production code behind merely to document an attempt. Report the
negative result concisely in chat; preserve one short durable note only when it
prevents a likely repeated mistake.
