# Plan: Improve Obstacle-Planner Performance and Robustness

## Goal

Correct the unsafe or ambiguous moving-obstacle behaviors first, then reduce
repeated geometry construction, validation, and search work without weakening
the independent public validator. Keep changes general-purpose and retain
`coneprog` during this work so solver replacement does not confound the result.

## Implementation List

1. Create a dedicated branch when implementation begins:

   ```bash
   git switch -c obstacle-performance-robustness
   ```

2. Record the current commit, MATLAB version, hardware, maintained benchmark
   results, and a profiler baseline. Add counters for validation calls,
   collision intervals, unresolved intervals, `shapeAtTime` calls, polyshape
   constructions, occupancy queries, route states, and `coneprog` calls.

3. Add failing regression tests before changing moving-obstacle semantics:

   - cyclically shifted and reversed polygon vertices between samples;
   - a rigidly rotating rectangle, if rigid interpolation is a supported
     contract;
   - a sparse translation with deliberately nonmatching topology and a query
     in the between-sample swept gap;
   - a 0.1-second sampled obstacle that must retain timed-search proposals;
   - history samples before and after the request horizon that must not enlarge
     the request projection;
   - multiple disjoint feasible wait and moving-target interception windows;
   - visible/hidden status transitions after the status meaning is defined.

4. Define one documented obstacle-history contract before optimizing it:

   - state whether vertices move linearly or represent sampled rigid motion;
   - define ring orientation, vertex correspondence, and hole semantics;
   - define whether `status ~= "visible"` means inactive or is metadata only;
   - define conservative behavior when correspondence cannot be proven.

5. Make dynamic interpolation conservative under that contract. Do not infer
   correspondence from only array size and finite masks. Canonicalize and verify
   rings when possible; otherwise use a proven swept enclosure or return a
   clear unsupported-model result. Do not treat a separated endpoint union as
   stationary geometry with zero speed.

6. Restrict proposal geometry to the active request horizon. Include shapes at
   the exact horizon endpoints plus source samples and applicable interval
   geometry inside the horizon. Apply the same horizon policy to static
   projections, dense swept envelopes, seed-corridor containment, broad-phase
   bounds, and moving-target trials.

7. Apply the low-risk call reductions first:

   - cap fixed-clock boundary refinement and use a documented physical
     amplitude tolerance instead of approximately `1e-6` degree resolution;
   - reuse each candidate's retained validation instead of validating it again
     during selection;
   - create invariant static planning projections once per request, outside the
     seed loop;
   - compute static convex decompositions once per authoritative occupied shape;
   - retain validator independence by caching source-derived geometry, not by
     trusting a solver-owned certificate.

8. Introduce one prepared-obstacle collection owned by the request. Normalize,
   protect, and prepare each obstacle once; pass the prepared value downward.
   Attach a version or immutable-source check so modified public fields cannot
   reuse stale preparation. Cache sample shapes, interval bounds, and boundary
   edges needed by repeated clearance queries.

9. Optimize geometry queries without changing their semantics. Use cached edges
   for static/sample shapes and operate directly on interpolated boundary arrays
   only after tests prove equivalence for multiple regions, holes, NaN-separated
   rings, boundary points, and orientation. Keep the current polyshape path as a
   correctness fallback.

10. Strengthen and accelerate continuous collision certification:

    - derive a conservative velocity bound for each polynomial subinterval by
      subdivision of its polynomial or Bernstein representation;
    - use per-obstacle, per-time-interval swept bounds instead of whole-history
      boxes;
    - report the last unresolved interval, path and obstacle speed bounds,
      required certifiable clearance, and minimum-time-step limit;
    - preserve fail-closed behavior and do not replace proof with dense sampling.

11. Improve continuous bound checking with a certified fast path. Use Bernstein
    range tests to prove easy intervals, recursively subdivide ambiguous cases,
    and retain a reliable stationary-point fallback. Do not treat a single
    Bernstein coefficient test as an exact rejection test.

12. Restore bounding-box acceleration in occupancy queries:

    - for occupancy or first-blocker outputs, reject boxes that cannot contain
      the query point;
    - for minimum-clearance diagnostics, use box distance as a lower bound and
      skip an obstacle only when it cannot improve the best exact clearance.

13. Improve the time-expanded search while preserving exact decision rules:

    - reserve endpoints and hard activity/topology event times first, then fill
      remaining layers uniformly;
    - vectorize per-source duration and target-layer calculations;
    - preallocate explored-state diagnostics;
    - replace implicit `uint16` assumptions with explicit state/work limits or
      wider indices;
    - represent "move at a feasible rate, then wait" separately from a motion
      stretched uniformly across an entire layer gap.

14. Reduce spatial-homology overhead. Replace formatted string state keys with
    an overflow-checked numeric encoding, compute cleanup trees once from the
    start and goal, and benchmark a priority queue against the current linear
    frontier scan. Keep an optimization only when the measured planner time
    improves.

15. Change wait and intercept refinement so it does not assume global monotone
    feasibility. Search event-aware/coarse intervals for every feasible window,
    refine each fail/pass boundary locally, and choose the earliest validated
    result. Reuse immutable obstacle preparation across trials, but key any
    horizon-dependent projection by the trial horizon.

16. Refactor `planCorridorQuintic` only after behavior is protected by tests.
    Extract one `solveOneSeed` helper with the current fallback order, refine
    direct waits in balanced-arrival mode when objective-relevant, and add a
    safe early exit only when a validated result reaches a proven request-wide
    lower bound. Evaluate `parfor` for independent seeds separately and never
    nest it around parallel solver work.

17. Make timing-accounting disagreement diagnostic rather than plan-fatal.
    Return a `TimingAccountingValid` flag and residual while preserving the
    valid trajectory result. Continue to throw for malformed, negative, or
    nonfinite timing inputs that indicate corrupted program state.

18. Consolidate diagnostics through shared templates and explicit named field
    assignments. Document the reason for `MaximumSeedCount = 9`, and replace the
    nominal 65,535-layer allowance with a bound on estimated states and
    transitions.

## Benchmark Matrix

Run at least these deterministic families:

| Family | Required coverage |
| --- | --- |
| Direct | Obstacle-free static and moving-target requests |
| Static | Convex, concave/U-shaped, dense outline, expected no path |
| Dynamic | Translation, rotation, topology change, status transition |
| Timed | Wait, fast-cross-then-wait, balanced arrival, event-heavy history |
| Numerical | Zero-margin near grazing and minimum-step unresolved result |
| Search | Many visibility nodes, homology classes, and time layers |

Warm up once, discard that run, and perform at least three measured repetitions
in one MATLAB session. Record minimum and median wall time, validation time,
collision checks, polyshape constructions, route states, `coneprog` calls,
arrival time, path length, kinematic peaks, and termination reason.

## Acceptance and Rollback Rules

- Every successful result must pass the unchanged independent public validator
  for endpoints, workspace, collision, velocity, acceleration, jerk, and
  applicable certificates.
- Expected no-path and unsupported-model results must remain explicit; never
  convert them into fabricated trajectories or hidden fallbacks.
- New dynamic-obstacle behavior must be conservative relative to the documented
  interpolation model.
- Do not loosen tolerances, safety margins, geometry, or work limits to obtain a
  passing benchmark.
- Implement each numbered group in a separate reviewable commit. Revert only
  the group that regresses correctness, trajectory quality, or diagnostics.
- Retain a performance change only when affected cases show either a material
  exact call-count reduction or at least a 10 percent median wall-time
  improvement without a material regression elsewhere.
- Run focused tests after each group. After the accepted groups, run every
  maintained example serially and headlessly, then one visible example and one
  expected-failure diagnostic plot.
- Record measured evidence in `verification.md` and `benchmark.csv`. Do not
  claim a speedup or correctness fix for cases that were not executed.

## Recommended Order

Implement items 3-6 first for modeling correctness, items 7-12 for the largest
low-risk runtime reductions, items 13-15 for search quality and scaling, and
items 16-18 for controlled refactoring. Revisit conic-solver replacement only
after these changes establish how much runtime still belongs to `coneprog`.
