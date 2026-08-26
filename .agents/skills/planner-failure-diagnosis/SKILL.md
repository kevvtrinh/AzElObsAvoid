---
name: planner-failure-diagnosis
description: Diagnose an unexplained Az/El planner failure, no-path result, invalid trajectory, or validation disagreement by locating the earliest broken stage before changing planner behavior.
---

# Planner Failure Diagnosis

Find the earliest transformation whose valid input becomes invalid output. A
later solver, validator, or plotting symptom does not establish root cause.

## Focused Workflow

1. Reproduce the smallest deterministic failure and preserve the exact request,
   revision, dirty state, seed, resolved options, result, termination reason,
   and independent validation.
2. Inspect returned diagnostics before adding instrumentation.
3. Trace only the applicable stages in execution order:

   ```text
   input -> obstacle preparation -> topology/seed -> geometric route
   -> motion construction -> timing -> kinematics -> collision
   -> endpoint -> candidate ranking -> result/plotting
   ```

4. Compare both sides of each suspected transformation quantitatively. Use
   `UNKNOWN` until evidence identifies the first violated contract.
5. Before editing, state the failure class, earliest failing stage, violated
   input-driven invariant, and a structurally different verification case.

Read [references/stage-protocol.md](references/stage-protocol.md) only when the
focused pass cannot isolate the cause, the failure crosses several stages, the
user requests a comprehensive diagnosis, or a behavior change is likely.

## Fix And Verification

Fix the common generating invariant, not the named example. Do not add hidden
waypoints, preferred turns, scenario checks, widened tolerances, clipped state
histories, easier geometry, or an unreported fallback.

After a requested fix, run static checks, the original failure, and one
structurally different case. Expand to affected success, expected-failure,
static/moving, motion-profile, and maintained-example coverage only after the
focused fix passes and as required by `AGENTS.md`.

Report quantitative evidence, root cause, changed owner, checks, remaining
limits, and `Root cause: not yet established` when unresolved.
