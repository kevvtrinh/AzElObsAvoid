# Reduce Repeated Cone-Program Calls

## Source

The supplied shared-chat snapshot exposes `plan_coneprog.md`, not a separate
artifact named `planreduceconeprogcall`. This plan applies the snapshot's
explicit follow-up recommendation to reduce repeated SOCP construction and
solver invocation overhead.

## Fixed Gate

Retain the change only if all of the following hold:

- median end-to-end runtime improves by at least 10% on the fixed
  `exampleObstacleAvoidance` benchmark;
- planner success, independent validation, collision validation, kinematic
  validation, and the plane certificate remain passed;
- selected polyline length, smoothed-path length, and arrival quality do not
  regress beyond existing tolerances;
- a structurally different obstacle case passes the same correctness checks;
- focused tests, the full test suite, every maintained headless example, one
  visible success, and one no-path diagnostic pass.

## Candidate 1: Batch Independent Plane Problems

Maximum-margin plane problems within one planner stage are independent. Their
objectives and constraints can therefore be assembled as one block-diagonal
SOCP and solved by one `coneprog` invocation without coupling their decisions.
Extracted planes remain subject to the existing direct numerical verifier.

Result: rejected and removed. A comparable cold run was 8.20 s versus 7.97 s
for restored `coneprog`, so the candidate was slower before formal timing.

## Candidate 2: Certify The First Collision-Free Iterate

The first collision-free biconvex iterate already satisfies the nonlinear
collision requirement. Retain it immediately and rely on the unchanged exact
plane certificate plus independent trajectory validation, avoiding the next
maximum-margin-plane refit and trajectory SOCP. If certification fails, return
the existing explicit `planeCertificateUnavailable` failure.

Result: rejected and removed. The warmed 11-run median improved from
2.2773948 s to 1.8804262 s (17.4%), and all validation passed, but the smoothed
path grew from 11.4118614 deg to 11.9926831 deg (5.1%). That violated the fixed
quality gate despite the earlier 7.5586369 s arrival.

## Implementation

1. Replace per-pair calls in alternating-plane updates with one batched call.
2. Replace per-pair calls in travel refinement with one batched call.
3. Replace per-pair final-certificate calls with one batched call.
4. Preserve plane-pair counts and add explicit cone-program invocation counts
   so diagnostics do not hide the batching effect.
5. Reject and remove the candidate completely if the fixed gate fails.

## Status

Completed. Neither candidate passed the fixed gate, so production behavior
remains unchanged.
