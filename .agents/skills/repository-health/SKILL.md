---
name: repository-health
description: Audit repository health or perform an explicitly requested cleanup by prioritizing correctness, invariant ownership, duplication, diagnostic integrity, verified performance, tests, and repository hygiene without retaining neutral churn.
---

# Repository Health

Use focused scope unless the user explicitly requests a repository-wide audit
or final production cleanup.

## Modes And Authority

- **Audit:** read-only. Identify and prioritize evidence-backed findings.
- **Cleanup:** edit or delete only when the user requested implementation or
  cleanup. Preserve public behavior and unrelated work.
- **Production readiness:** use only when explicitly requested as a final gate;
  report blockers rather than redesigning the planner.

An audit does not authorize refactoring, deletion, commits, or pushes.

## Investigation

Freeze repository identity and dirty state. Read applicable `AGENTS.md`, public
contracts, callers, tests, examples, diagnostics, assessments, and benchmark
evidence. Trace at least one real behavior through generation, validation,
result assembly, and expected failure.

Prioritize:

1. invalid success or inconsistent success/failure contracts;
2. scenario-specific behavior and duplicated invariant ownership;
3. missing or misleading diagnostics and performance attribution;
4. repeated work, dead private paths, stale options, and proven artifacts;
5. documentation, organization, and style issues with actual consequence.

Do not infer that code is unused from filenames or text search alone. Inspect
dynamic dispatch, package/private lookup, function handles, tests, examples,
and compatibility behavior.

## Change Gate

For cleanup implementation, define the observable benefit before editing:
removed reachable complexity, one source of truth, fewer production lines,
less repeated work, corrected documentation, or repaired behavior. Preserve
deterministic outputs and independent validity within declared tolerances.

Run focused checks after each coherent change. Do not launch the full example
matrix until the candidate passes its focused behavior or equivalence gate.
Restore cleanup-owned changes when safety or equivalence cannot be shown.

Do not retain wrappers, options, comments, instrumentation, or evidence files
that merely make the diff larger. A smaller file count or more recorded data is
not itself an improvement.

## Findings And Completion

For each material finding, report severity (`blocker`, `high`, `medium`, or
`low`), confidence (`observed`, `inferred`, or `unknown`), evidence and affected
path, triggering condition, smallest proposal, risks, and confirming check.

At completion, state the inspected scope, healthy properties preserved,
changes made, focused and broader checks actually run, runtime or size effects,
unresolved candidates, untested paths, and final repository status. Do not call
the repository production-ready while a known required blocker remains.
