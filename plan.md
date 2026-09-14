# Plan: finish the one-profile earliest-arrival core

## Objective

Keep one public planner and the smallest general decision tree that returns
truthful BMTP motions. Exercise every retained decision with deterministic
inputs, remove provenance-driven profiles and compensating retry machinery, and
preserve earliest-arrival behavior.

Acceptance requires unchanged physical inputs and independent validation. No
arrival or path-length regression may exceed 1% on the retained corpus. Runtime
is judged at its actual scale: roughly 1 s to 2 s and 15 s to 20 s are
acceptable; 15 s to 60 s is not.

## Factory rule

Every stage must emit a truthful input for the next stage. When a proposal is
bad, reproduce the geometry, clock, kinematics, clearance, certificate, and
selection decisions by hand and remove the first source of falseness. Do not
send a known-bad seed downstream and add retries, filters, alternate profiles,
or cleanup solvers around it. `AGENTS.md` records this rule for future work.

## Completed work

- Mapped and tested the retained public flows: direct, static detour, expected
  no path, fixed dynamic initial snapshot, distinct arrival snapshot, timed
  dynamic guide, zero-delay earliest chord, delayed-chord incumbent, timed
  homotopy that beats the incumbent, moving target/non-rest chronological
  search, and honest search exhaustion.
- Removed solver degree, subdivision, and formulation choices driven by
  `Source`/`SearchKind`. A regression proves diagnostic labels cannot alter the
  manufactured motion.
- Found the first bad earliest-arrival clock: a timed guide ending at 10.5 s was
  stretched to 48.4974 s before moving-obstacle planes were built. Timed warm
  starts now retain the supplied physical clock and BMTP owns the variable
  duration.
- Fixed the zero-length visibility predicate that falsely rejected every
  stationary wait in scenes containing a static obstacle.
- Exact-certified each static obstacle over its own active sub-interval instead
  of using a common lifetime or 13 samples.
- Replaced the 13-sample moving-edge oracle with a complete-interval affine
  point-versus-convex-cell test. Quadratic half-space roots partition every
  cell clock, so between-sample contacts are rejected before BMTP.
- Removed the near-goal wait preference and alternate final-transition
  tie-breaker. Temporal selection is now earliest reachable layer followed by
  shortest spatial ancestry, with deterministic first selection on exact ties.
- Hand-audited the timed proposal offset attempts. In the saved moving fixture,
  attempts 1 through 6 produced no temporal route; attempt 7 alone worked. The
  split was compensating for capped, order-sensitive staging nodes, not a BMTP
  or clearance failure.
- Replaced the offset retry schedule, Delaunay-first graph, connectivity
  recovery, and exhaustive fallback with one deterministic staging-node helper
  and one temporal search. The single offset is derived from derivative limits
  and bounded by the supplied workspace scale.
- Increased the source-independent variable-clock mesh floor from 16 to 20
  spans. The 180-second static-wall case now arrives at 64.6330611 s versus the
  64 s exact static reference (+0.989%), with length 120.0057758.
- Corrected regressions that demanded slower historical paths: a valid curved
  homotopy arrives at 8.55 s instead of waiting for the 10.76 s direct chord,
  and a collision-free slow direct edge no longer receives a preferred wait.
- Updated the example harness so the two unsupported continuous-deformation
  cases count as their documented stable failure and added the previously
  omitted spinning-U example.

## Current measurements

- Complete MATLAB suite before the final workspace-scale adjustment: 121/121
  passed in 184.7 s.
- Random fixed-arrival corpus: 160/160 successful and independently valid;
  total wall time 82.44 s, maximum request 4.17 s. The prior candidate record
  was 86.95 s and production was 99.79 s.
- Earliest representative results, all independently valid:
  - moving barrier: 10.1400889 s, length 10;
  - moving circle: 8.5732124 s, length 12.1018884;
  - opening U: 11.6133889 s, length 10;
  - random case 1: 70.7396341 s, length 134.1418419;
  - saved moving detour: 118.6665293 s, length 232.6950111;
  - static wall: 64.6330611 s, length 120.0057758.

## Final verification

- Final complete MATLAB suite: 121/121 passed in 183.5 s.
- Final maintained examples: 21/21 documented outcomes valid, including
  spinning U and both expected unsupported-deformation results.
- Small-workspace gate: rotating field 9.1376249 s / 20.4272801; spinning U
  24 s / 16.2756944. The latter is +0.822% in path length versus the Claude
  branch before cleanup and remains within the gate.
- The random fixed-arrival and earliest-arrival measurements above remain
  within the stated quality/runtime gates.

## Adoption status

The verified tree has been transferred to `build-core`. Final static analysis
reported zero `checkcode` messages and a clean `git diff --check`. The adoption
commit and removal of task-created worktrees/CSV/scratch artifacts complete this
plan; user Rogue Cases and unrelated sandboxes remain untouched.
