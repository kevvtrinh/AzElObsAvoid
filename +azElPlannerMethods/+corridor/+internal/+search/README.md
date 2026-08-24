# Corridor search

This method consumes the neutral bounded visibility/homology and time-expanded
seed portfolio owned by `azElInternal.generateTopologySeeds`. Only the compact
motion path's dynamic-route expansion adapter remains local. Search outputs are
proposals only: they never certify motion, relax protected geometry, or declare
planner success.

The proposal portfolio is finite and deterministic. Motion construction and
the independent validator, not graph connectivity alone, decide success.
