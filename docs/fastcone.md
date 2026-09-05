# Fastcone in BMTP

Both `bmtpEngine.solveTrajectoryStep` and `bmtpEngine.solveSeparatingLine`
call `fastcone.solve` with their existing optimization inputs and options.
The adapter combines certified geometric plane profiles with a native
primal-dual solver for scalar and three-dimensional Lorentz cones. It is an
iterative numerical engine with analytical block equations, not a universal
closed-form motion-profile solver or a complete replacement for coneprog.

## Build and use

```matlab
addpath(repositoryRoot, fullfile(repositoryRoot, 'trajectory'));
fastcone.build; % uses the configured C++ MEX compiler
% Windows also supports an explicit installed MinGW g++ path:
% fastcone.build('C:\msys64\mingw64\bin\g++.exe');

[x, objective, flag, output] = fastcone.solve( ...
    f, cones, A, b, Aeq, beq, lb, ub, options);
```

`options` is the same `optimoptions('coneprog',...)` object used by BMTP.
The interface supports the nine positional arguments used by this project;
it does not implement every optional coneprog interface or output. A positive
native acceptance requires original-unit primal checks and an independent
weak-dual objective bound. Every unresolved attempt, including unsupported
cone dimensions, missing native code, numerical failure, and iteration limit,
calls original `coneprog` with the identical nine arguments. Its exit flag is
returned unchanged. An unresolved attempt is never labeled infeasible by
fastcone itself. Recovery retains coneprog's original numerical behavior;
the public planner still independently validates any returned trajectory.

MATLAB R2024b / Optimization Toolbox with Windows MinGW and Microsoft Visual
C++ 2022 have been exercised. Configured MEX compiler support on other
platforms is provided but untested.
The builder does not download dependencies and never runs during planning.
Eigen 3.4.0 is pinned in `trajectory/+fastcone/third_party/eigen`; its original
notices and license files are retained. The locally built
`trajectory/+fastcone/core.mex*` is ignored by Git. After pulling this branch
onto another machine, build it there.
Without the binary, geometric plane profiles can still run, while the native
path reports recovery. Full native regression tests require a successful build.

## Executed-method diagnostics

Each solver output retains `Method`, `FallbackUsed`, `NativeAvailable`,
`NativeAccepted`, `Prototype`, `PrototypeTime_s`, and `TotalTime_s`.
`Prototype` contains the candidate's checks or the reason it was unresolved.
`TotalTime_s` includes the failed attempt and any coneprog recovery.

For BMTP seed summaries, `SolverDiagnostics.ConicSolver` aggregates trajectory,
alternation, and travel-refinement calls. Final certificate solves appear
separately in `SolverDiagnostics.PlaneCertificate.ConicSolver`. Each records
`CallCount`, `NativeAcceptedCount`, `AnalyticalPlaneCount`, `RecoveryCount`,
`TotalTime_s`, `LastMethod`, and `LastRecoveryReason`. These fields describe
completed conic calls; they do not change candidate selection. Results may
repeat seed summaries in several places, so do not sum recursively duplicated
diagnostics. Conic time is already contained in total planner time.

## Analytical components

For a Lorentz block, write `a = (a0, av)` and `b = (b0, bv)`.
The Jordan product and inverse action are evaluated directly:

```text
a o b = (a0*b0 + av'*bv, a0*bv + b0*av)
L(a)*u = b:
u0 = (a0*b0 - av'*bv) / (a0^2 - av'*av)
uv = (bv - av*u0) / a0
```

Scalar blocks use ordinary multiplication and division. Feasible step lengths
come from the first nonnegative boundary root of
`(a0+t*b0)^2 - ||av+t*bv||^2 = 0`, together with head positivity. The native
kernel uses explicit cone scaling, predictor-corrector steps, and prepared
sparse Gram products. It reuses block structure and symbolic factorization
when the Newton-matrix sparsity pattern matches. It does not invoke another
optimization package; Eigen supplies matrix algebra and Cholesky.

For equal-duration, equal-degree Bezier spans, C3 continuation is local:

```text
[Q0 Q1 Q2 Q3]' = [ 0  0   0  1;
                   0  0  -1  2;
                   0  1  -4  4;
                  -1  6 -12  8 ] * [P(n-3) P(n-2) P(n-1) Pn]'
```

The degree-7/16 rest-boundary equality pattern is recognized exactly before
using this sparse reduction. Both `E*basis == 0` and `E*base == d` are checked.
Other equalities use a general numerical reduction. Input-derived scalar,
one-tail, and constant-tail cones reduce to their equivalent affine bounds.

For canonical plane programs, each Bernstein product row gives a weak bound
`target - alpha*distance(q0, polygon) - beta*distance(q1, polygon)`.
Polygon projections are computed from exact edge/vertex formulas. A proposed
single-contact solution is accepted only when its feasible objective meets
the strongest bound within the requested tolerance. A free endpoint normal
uses an explicit three-variable log-barrier gradient and Hessian to select a
center on the optimal face. That center is a Newton solve; it is not claimed
to duplicate coneprog's choice on a nonunique face. Multiple-contact cases
that are not certified use recovery.

For `G*u+s=h`, `s` in the product cone, and cone-feasible `z`, the independent
objective lower bound is

```text
-h'*z + min_{lower <= u <= upper} (cost + G'*z)'*u.
```

The last term selects a box endpoint by each residual coefficient's sign.
Infinite endpoints remain explicit; an unavailable finite bound cannot
certify acceptance. MATLAB rechecks this bound and the original primal
residual after the native return. No physical constraints are clipped,
relaxed, or removed to accept a trajectory.

## Verification and runtime

```matlab
addpath(repositoryRoot, fullfile(repositoryRoot,'trajectory'), ...
    fullfile(repositoryRoot,'benchmarks'));
r = runtests(fullfile(repositoryRoot,'tests'));
assert(all([r.Passed]) && ~any([r.Incomplete]));
% Run each maintained example in its own MATLAB process:
verifyFastconeExample('exampleObstacleAvoidance');
% Compare exact captured programs, with three or more interleaved repeats:
results = benchmarkFastcone(cases,3); % cases(k).Name and nine-argument .Args
```

The benchmark includes setup, unsuccessful attempts, and recovery. Its saved
MAT file retains inputs, repeated timings, flags, primal residuals, and solver
outputs. Its per-case CSV retains unfavorable results rather than filtering
them. See the [recorded research measurements](../benchmarks/results/fastcone/README.md)
for full-planner comparisons, numerical-quality differences, and limits.

This integration selects the measured `hsSolveBlocks` / `solveNativeBlocks`
algorithm from the CONEPROG GO FAST research workspace. Those functions become
`fastcone.solve` / `fastcone.solveNative`; the native blocks source becomes
`fastcone_core.cpp`. It excludes the later ordered-kernel and plane-box
experiments. The original 10x overall target was not reached; the user accepted
the current measured implementation and requested this branch integration.
