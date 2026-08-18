# References

## Third-order direct collocation

- Moreno-Martin, S., Ros, L., and Celaya, E. (2024). "Collocation Methods
  for Second and Higher Order Systems." *Autonomous Robots*, 48, Article 2.
  https://doi.org/10.1007/s10514-023-10155-z

  The compact optimizer uses a separated third-order chain. Quadratic jerk
  ordinates are integrated once to produce acceleration, velocity, and
  position polynomials. This prevents independent state splines from
  violating the shared dynamics.
