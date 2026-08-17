# References

The RP retimer uses Bernstein-form Bezier curves, de Casteljau degree
elevation and subdivision, and convex-hull derivative bounds. See:

- Farouki, R. T. (2012). The Bernstein polynomial basis: A centennial
  retrospective. *Computer Aided Geometric Design*, 29(6), 379-419.
  https://doi.org/10.1016/j.cagd.2012.03.001

The deterministic spatial forward/backward retiming follows the standard
path-velocity propagation problem described in:

- Bobrow, J. E., Dubowsky, S., and Gibson, J. S. (1985). Time-optimal
  control of robotic manipulators along specified paths. *The International
  Journal of Robotics Research*, 4(3), 3-17.
  https://doi.org/10.1177/027836498500400301
- Shin, K. G., and McKay, N. D. (1985). Minimum-time control of robotic
  manipulators with geometric path constraints. *IEEE Transactions on
  Automatic Control*, 30(6), 531-541.
  https://doi.org/10.1109/TAC.1985.1104009

The nonlinear timed-curve collision certificate uses the linear
interpolation remainder bound `M * h^2 / 8` for a twice differentiable
function whose second derivative magnitude is at most `M`. See:

- Atkinson, K. E. (1989). *An Introduction to Numerical Analysis*, 2nd ed.,
  Section 3.1. Wiley. ISBN 978-0-471-62489-9.

The optional continuous learned proposer uses the DDPG algorithm. It has no
authority to accept geometry or motion:

- Lillicrap, T. P., Hunt, J. J., Pritzel, A., Heess, N., Erez, T., Tassa,
  Y., Silver, D., and Wierstra, D. (2016). Continuous control with deep
  reinforcement learning. *International Conference on Learning
  Representations*. https://arxiv.org/abs/1509.02971
