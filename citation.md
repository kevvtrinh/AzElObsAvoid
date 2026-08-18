# References

## Time-optimal path parameterization

Pham, H., and Pham, Q.-C. (2018). A New Approach to Time-Optimal Path
Parameterization Based on Reachability Analysis. *IEEE Transactions on
Robotics*, 34(3), 645-659.
https://doi.org/10.1109/TRO.2018.2819195

The acceleration-only retimer uses the paper's squared-path-speed state and
spatial reachability structure. Its continuous derivative certificates are
more conservative than point collocation. The configurable maximum cell
length controls this discretization. A smaller cell length can reduce the
conservatism, but it increases the number of reachability stages.

## Continuous-jerk motion profiles

Fang, Y., Qi, J., Hu, J., Wang, W., and Peng, Y. (2020). An approach for
jerk-continuous trajectory generation of robotic manipulators with
kinematical constraints. *Mechanism and Machine Theory*, 153, 103957.
https://doi.org/10.1016/j.mechmachtheory.2020.103957

The finite-jerk retimer adapts the paper's sinusoidal jerk family to each
scalar path-speed transition. Analytic integration makes velocity,
acceleration, and jerk continuous while the existing derivative envelopes
continue to certify Cartesian speed, acceleration, and jerk. The planner
does not claim global time optimality across all continuous-jerk functions.

## Modified sigmoid jerk smoothing

Fang, Y., Hu, J., Liu, W., Shao, Q., Qi, J., and Peng, Y. (2019).
"Smooth and time-optimal S-curve trajectory planning for automated robots
and machines." *Mechanism and Machine Theory*, 137, 127-153.
https://doi.org/10.1016/j.mechmachtheory.2019.03.019

The `continuousSigmoid` retimer option uses the paper's modified logistic
jerk law from Eq. (5), its recommended variation parameter `sqrt(3)/2`, and
the peak-snap relation in Eq. (19). The scalar path-speed transition uses the
paper's snap-first constraint hierarchy to select varying-jerk, constant-jerk,
and constant-acceleration intervals. Fixed Gauss-Legendre quadrature evaluates
the non-elementary phase integrals. The implementation applies this law to a
curved path coordinate. It does not reproduce the paper's separate multi-axis
point-to-point synchronization method.
