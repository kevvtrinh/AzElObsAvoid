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

## Candidate 3: QOCO For Trajectory SOCPs

Fresh baseline at `e9af134e5cd1ff36fd92067f4b26ea589f548396`:

- command: three warmups plus eleven headless `exampleObstacleAvoidance` runs;
- median 2.3941869 s, minimum 2.3739192 s, maximum 2.6600484 s;
- 7.57454176632 s arrival, 11.152119519 deg selected polyline, and
  11.4118613877 deg smoothed path;
- planner, independent validation, collision, kinematic, and plane
  certificates passed;
- profile: eight trajectory SOCP calls owned 1.764811327 s, while 49 plane
  solve scopes owned 0.310989022 s.

Hypothesis: the BSD-3-Clause QOCO interior-point solver may improve the larger
trajectory SOCPs even though its rejected plane-only adapter was slower. Keep
all maximum-margin and final-certificate planes on `coneprog`. Retain only for
at least 10% warmed end-to-end improvement with unchanged physical metrics,
validation, and failure behavior.

Result: rejected and removed at the focused correctness gate. The first
trajectory subproblem solved, but both obstacle-avoiding seeds failed on their
second trajectory SOCP after separating planes were introduced. The planner
returned `noValidatedSeed`; no tolerance or iteration tuning was attempted.

## Candidate 4: Fixed-Time LP Bisection

Hypothesis: for `earliestArrival`, fixing the common segment time turns every
trajectory constraint into a linear constraint. Feasibility is monotone in
time, so bisection plus `linprog` can replace the two time-power cones while
leaving plane construction, final certification, and public validation
unchanged. Retain only for at least 10% warmed end-to-end improvement with no
arrival, motion-quality, certificate, or failure regression.

Result: rejected and removed at the focused quality gate. The planner,
independent validator, and plane certificate passed, but arrival regressed from
7.57454176632 s to 7.66333007813 s and the smoothed path grew from
11.4118613877 deg to 11.9272675751 deg. No secondary objective was added to
rescue the failed formulation.

## Candidate 5: Clarabel For Trajectory SOCPs

Hypothesis: the modern Apache-2.0 Clarabel interior-point core could replace
`coneprog` for the dominant trajectory SOCPs while `coneprog` retained all
separating-plane and final-certificate programs. The public GPL-3.0
`iFR-OFC/Clarabel.m` adapter at
`fd731d74d77cadc0049b4c3b1cb0b22405d166fb` was evaluated against official
Clarabel C++/Rust sources at
`25540f559592068d0c8a80e46ded1b21760212a1`. Its stale unconditional FAER and
SDP settings were repaired only in an isolated temporary checkout. Its LP,
QP, and SOCP examples then solved with small residuals before planner wiring.

Focused result: the three-warmup, eleven-run median improved from 2.3941869 s
to 1.0663089 s (55.5%). All focused planner, collision, kinematic, independent
validation, and 18-of-18 plane-certificate checks passed. Arrival improved to
7.52479579758 s and smoothed length to 11.4065391737 deg.

Structural result: rejected and removed. The concave static-U case remained
feasible and independently valid, and all 504 plane pairs certified, but
arrival regressed from 20.7814508253 s to 36.2565015075 s and smoothed length
from 39.4001427062 deg to 48.2844877484 deg. No solver tuning or hybrid
fallback was added to rescue the failed quality gate.

## Candidate 6: ECOS For Trajectory SOCPs

Hypothesis: ECOS could be used only for the dominant trajectory SOCPs, avoiding
the plane-certificate precision failure that rejected its earlier plane-only
trial. All plane and final-certificate programs remained on `coneprog`.

Result: rejected and removed at the first focused correctness gate. ECOS failed
the initial trajectory SOCP for all three seeds, before any separating-plane
solve, and the planner returned `noValidatedSeed`. No tolerance or iteration
tuning was attempted.

## Candidate 7: Exact Constant-Plane Fast Path

Hypothesis: when the obstacle polygon and the complete Bezier control hull are
linearly separable, a shared unit-normal plane can be selected from the finite
2-D polytope support directions, checked by the existing direct verifier, and
used without a cone-program call. The variable affine-plane SOCP remained the
explicit fallback when no such certificate existed.

Result: rejected and removed at the focused runtime gate. The analytic path
certified all 18 final plane pairs without `coneprog`, but its weaker valid
linearizations increased each successful seed from three to five trajectory
SOCPs. The cold smoke grew to 8.7553808 s, while the retained solver took
5.2025991 s in the comparable retained headless record. Arrival was
7.52785401154 s and smoothed length was 11.4146921369 deg. This shows that
plane strength, not only plane-call count, controls total runtime.

## Final Decision

Retain the existing `coneprog` backend and call sequence. Clarabel produced the
largest focused speedup, but its structurally different motion-quality
regression violated the declared gate. Every candidate implementation and
adapter was removed; only reproducible negative evidence remains.
