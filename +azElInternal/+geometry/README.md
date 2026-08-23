# Geometry

Shared deterministic polygon primitives live here. These functions know
nothing about planner options, seed policy, or candidate ranking. Keeping
polyshape construction and boundary traversal in one module prevents search,
collision clearance, and obstacle preparation from interpreting the same
geometry differently.
