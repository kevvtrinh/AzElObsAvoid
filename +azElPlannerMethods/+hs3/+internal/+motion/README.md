# HS3 motion

This module owns Hermite-Simpson collocation, finite-jerk propagation,
constraints, objective derivatives, reconstruction, diagnostics, and the
`fmincon` solve used by standalone HS3.

`solveHs3` consumes prepared obstacles and a neutral topology proposal. For
fixed arrival it uses exact affine boundary and kinematic matrices; earliest
arrival retains a nonlinear final-time decision with exact jerk-variable
derivatives and a bounded one-sided time derivative. Complete polynomial
motion is returned to `hs3.plan` for canonical independent validation.

The planner may rebuild collision linearizations around an HS3 candidate and
increase collocation segments within configured mesh limits.

The deadline is cooperative: setup and the solver output callback stop future
work, while an active function evaluation can finish after its requested time.
Optimizer feasibility is never accepted as planner success without canonical
trajectory validation.
