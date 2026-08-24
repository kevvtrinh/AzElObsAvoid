# Corridor geometry ownership

No corridor-local geometry source remains. Convex decomposition is owned by
`azElInternal.convexPolygonRegions`; canonical boundary construction, traversal,
and signed clearance live under `azElInternal.geometry`. Compact motion consumes
those neutral results without redefining them.
