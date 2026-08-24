# Corridor motion

This module turns geometric and timed seeds into continuous C3 quintic motion,
retimes candidates, and selects the earliest independently valid result. It
consumes prepared obstacles and validation certificates; it does not construct
scenario geometry.

Straight static requests use the exact endpoint quintic. Multi-segment and
dynamic requests use the compact C3/C4 spline path, including nonzero endpoint
velocity and acceleration. Neither path invokes HS3 or `fmincon`.
