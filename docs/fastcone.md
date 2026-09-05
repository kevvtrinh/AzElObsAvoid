# Fastcone in BMTP

Both `bmtpEngine.solveTrajectoryStep` and `bmtpEngine.solveSeparatingLine`
call `fastcone.solve` with their existing nine optimization arguments.
Fastcone is implemented entirely in MATLAB. It combines certified plane
contact equations with an iterative conic predictor-corrector. It is not a
universal closed-form motion solver or a complete replacement for coneprog.

## Use

```matlab
addpath(repositoryRoot, fullfile(repositoryRoot, 'trajectory'));
[x, objective, flag, output] = fastcone.solve( ...
    f, cones, A, b, Aeq, beq, lb, ub, options);
```

MATLAB and Optimization Toolbox are required. No compiler, MEX, Python, or
third-party numerical library is required. MATLAB R2024b Update 4 has been
exercised; other releases and platforms have not been verified.

`options` is the existing `optimoptions('coneprog',...)` object. Fastcone
supports this project's nine-argument interface, not every optional coneprog
interface or output. Acceptance requires original-unit feasibility and an
independent weak-dual objective bound. Unsupported cone dimensions, numerical
failure, iteration limits, and uncertified profiles recover through coneprog
with identical inputs. Its exit flag and numerical behavior are preserved.
An unresolved attempt never establishes infeasibility. A separately checked
original-input dual certificate may return -2 without reference recovery.
That certifies this conic program, not the absence of a planner route. The public planner
independently validates every returned motion, including recovered solutions.

## Executed-method diagnostics

Solver outputs retain `Method`, `FallbackUsed`, `Prototype`, `PrototypeTime_s`
and `TotalTime_s`. `MatlabAccepted` identifies accepted iterative conic solves;
analytical plane acceptance is a separate method. Legacy `NativeAvailable` and
`NativeAccepted` are always false. `Prototype` reports certificates or the
reason a candidate remained unresolved. Total time includes recovery.

`SolverDiagnostics.ConicSolver` aggregates construction and refinement calls;
final certificates have a separate `PlaneCertificate.ConicSolver` record.
The counts are `CallCount`, `MatlabAcceptedCount`, `AnalyticalPlaneCount`,
`CertifiedInfeasibleCount`, `RecoveryCount` and the compatibility field
`NativeAcceptedCount` (zero). `CertifiedInfeasible` marks that solver outcome.
Timing and last-method/recovery fields remain available. Do not recursively
sum duplicated seed summaries or add conic time to planner time: it is already
included in the planner total.

## Plane contact equations

Each Bernstein product row supplies the weak bound
`target - alpha*distance(q0, polygon) - beta*distance(q1, polygon)`.
Polygon projections use edge/vertex equations. A single-contact candidate is
accepted when its feasible objective meets this bound. A free endpoint uses
a small Newton centering solve on the optimal face.

For multiple contacts, `contactPlanes` first proposes triples using the
single-contact solution and its most violated row. Equal contact values give
`n1 = M*n0`; intersecting this relation with two unit circles reduces to
quadratic/trigonometric equations. Nonnegative contact weights give a dual
certificate. Four-contact profiles use explicit 3-by-3 determinants for the
remaining normal direction and dual weights.

`exchangeContacts` solves a small working set and inserts the most violated
original row. A certified contact basis can replace the previous working set;
every original row remains in the global objective and feasibility checks.
Previous support vertices order proposals, with a full support-pair attempt
when necessary. Two-contact subproblems use safeguarded roots of analytical
polygon-distance derivatives. Exhausted or singular proposals return
unresolved; they do not prune planner routes or authorize no-path results.

A certified maximum-margin plane can still make a later alternating trajectory
step infeasible. When that happens after direct multiple-contact planes, BMTP
recomputes those planes through `fastcone.reference` from the same stored
source curves and retries once. `ContactPlaneRecoveryCount` and ordinary conic
recovery counts disclose this work. The existing permitted warm-start horizon
expansion precedes the retry when applicable; the original deadline and full
independent validation still constrain the returned motion.

## General conic equations

`solveConic` eliminates fixed variables and exact equalities, recognizes
constant/one-tail norm profiles, scales the program, and calls `coneKernel`.
The degree-7/16 C3 Bezier equality map is used only after checking its exact
input pattern and null-space identities. Other equalities use QR reduction.

For Lorentz blocks, with `a=(a0,av)` and `b=(b0,bv)`:

```text
a o b = (a0*b0 + av'*bv, a0*bv + b0*av)
L(a)*u = b:
u0 = (a0*b0 - av'*bv) / (a0^2 - av'*av)
uv = (bv - av*u0) / a0
```

Scalar blocks use multiplication and division. Cone boundary step lengths
come from the first nonnegative quadratic root and head positivity. MATLAB
sparse Cholesky solves the predictor-corrector systems. Equal-width row
products are batched across Lorentz blocks. A sparsity-pattern
cache reuses product indexing, recomputing coefficients from the current input.

For `G*u+s=h`, cone-feasible `z`, and inherited variable bounds, the weak-dual
objective bound is

```text
-h'*z + min_{lower <= u <= upper} (cost + G'*z)'*u.
```

The box minimum selects endpoints by coefficient sign. Infinite endpoints
remain explicit. The physical residual and this bound are rechecked after
the kernel returns. A positive bound on the zero objective, with a floating
point allowance, can end a diverging iteration. To avoid reference recovery,
`infeasibilityCertificate` must also find a positive bound using the original
G/h and E/d arrays and a cone-feasible dual witness. It expands the original
variable box outward by ConstraintTolerance, subtracts the corresponding
scalar/cone-head and equality tolerance terms, and subtracts an arithmetic
error allowance. Only a positive remaining margin returns -2. The raw witness,
margin, tolerance and allowance are retained in solver output. Unbounded boxes
or unavailable certificates retain explicit reference recovery. No physical
limits or validation tolerances are relaxed.

## Verification and measurements

Run `runtests(fullfile(repositoryRoot,'tests'))` for maintained regressions.
`verifyFastconeExample` records maintained example runs and their independent
validation. Existing `benchmarkFastcone` compares identical captured programs,
including setup and explicit recovery. Benchmark artifacts stay under output.

The two previously slow captured plane requests improved from 67.04/65.31 ms
to 3.77/3.27 ms in an isolated MATLAB prototype, with original certificates.
The tight-U planner replay, including explicit plane recovery, measured 2.03 s
versus 3.30 s native; the moving-occlusion replay measured 0.567 s versus 0.522 s
native after batched preparation. These results do not establish uniform
speedups. Production passed all 151 tests, matched the prototype on 200 saved
requests, and passed the expected outcomes of all 18 maintained examples. See
[the current assessment](../branch_assessment.md) for exact measurements,
production verification status, and retained unfavorable evidence. Older
[research measurements](../benchmarks/results/fastcone/README.md) describe the
superseded native implementation and must not be attributed to this solver.
