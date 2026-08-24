# Corridor obstacles

Canonical obstacle preparation and shape-at-time queries live in
`azElInternal.obstacles`. This module builds corridor-specific envelope
boundaries from that shared immutable cached geometry; it never applies a
safety margin or selects a route.

Safety margins must already be present in the canonical public obstacle input.
