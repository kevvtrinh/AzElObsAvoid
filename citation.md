# References

## Homology-signature graph search

- Bhattacharya, S., Likhachev, M., and Kumar, V. (2012). "Search-Based Path
  Planning with Homotopy Class Constraints in 3D." *Proceedings of the
  Twenty-Sixth AAAI Conference on Artificial Intelligence*, 2097-2099.
  https://doi.org/10.1609/aaai.v26i1.8435

  The spatial visibility search augments each graph node with a bounded 2-D
  homology signature. The signature integrates principal angle changes about
  one interior representative of each connected sampled obstacle region. This
  preserves distinct route classes during Dijkstra search. It is a 2-D
  adaptation. It is not the paper's 3-D electromagnetic construction and is
  not a continuous Az/El/time homotopy certificate.

## Third-order direct collocation

- Moreno-Martin, S., Ros, L., and Celaya, E. (2024). "Collocation Methods
  for Second and Higher Order Systems." *Autonomous Robots*, 48, Article 2.
  https://doi.org/10.1007/s10514-023-10155-z

  The compact optimizer uses a separated third-order chain. Quadratic jerk
  ordinates are integrated once to produce acceleration, velocity, and
  position polynomials. This prevents independent state splines from
  violating the shared dynamics.

## Quintic stop profile

- Perlin, K. (2002). "Improving Noise." *Proceedings of SIGGRAPH 2002*,
  681-682. https://doi.org/10.1145/566570.566636

  The analytic first-motion constructor uses the quintic blend
  `10*u^3 - 15*u^4 + 6*u^5`. Its endpoint velocity and acceleration are
  zero. The planner scales each segment duration from analytic derivative
  bounds and reports each mandatory waypoint stop.

## Bernstein polynomial bounds

- Farouki, R. T. (2012). "The Bernstein Polynomial Basis: A Centennial
  Retrospective." *Computer Aided Geometric Design*, 29(6), 379-419.
  https://doi.org/10.1016/j.cagd.2012.03.001

  The solver and independent validator convert segment-local power
  coefficients to Bernstein form. They use the convex-hull property to
  bound complete polynomial intervals instead of checking samples only.
