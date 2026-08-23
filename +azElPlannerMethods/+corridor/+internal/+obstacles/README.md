# Corridor obstacles

This module converts canonical obstacle histories into immutable cached
geometry and answers shape-at-time requests. It preserves original and
protected geometry; it never applies a safety margin or selects a route.

The resulting cache is consumed only by the corridor snapshot. Safety margins
must already be present in the canonical public obstacle input.
