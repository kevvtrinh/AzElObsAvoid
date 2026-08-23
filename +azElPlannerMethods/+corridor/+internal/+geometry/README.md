# Corridor geometry

Deterministic polygon primitives for the corridor snapshot live here. These
functions know nothing about planner options, seed policy, or candidate
ranking. Keeping polyshape construction and boundary traversal in one module
prevents search, clearance checks, and obstacle preparation from interpreting
the same geometry differently.

The helpers are method-local even when HS3 contains a similar operation. Do
not create a cross-method dependency merely to remove duplicate code.
