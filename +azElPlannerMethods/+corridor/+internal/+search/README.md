# Corridor search

This module builds bounded visibility/homology graphs and time-expanded seed
proposals. Its outputs are proposals only: search never certifies a returned
motion, relaxes protected geometry, or declares planner success.

The proposal portfolio is finite and deterministic. Motion construction and
the independent validator, not graph connectivity alone, decide success.
