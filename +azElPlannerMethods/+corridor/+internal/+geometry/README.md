# Corridor geometry

Corridor-only polygon construction and convex-region decomposition live here.
Canonical boundary traversal and signed clearance are shared through
`azElInternal.geometry`, while these remaining helpers stay local because they
serve corridor seed and certificate construction.
