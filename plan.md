# Plan: solve the supplied Vietnam boundary history

Source: https://chatgpt.com/share/6aa1a94f-9544-83e8-a195-7357aba47093

This implementation plan restates the coordinate messages and latest
requirement summaries recovered from **Find STK Area Target Code**. It is not
a byte-for-byte download of the chat's plan attachment. The earlier broad
projected-area preparation plan is superseded for this fixture.

1. Preserve all 842 supplied coordinate pairs in
   `examples/data/vietnamBoundaryPoints.csv`, with columns
   `time_s,vertex_index,az_deg,el_deg`. Retain decimal precision, order, and
   explicit closing copies at 2910 and 3000 s. Keep the distinct last vertex
   at 2770 s.
2. Build a deterministic fixture from the three snapshots. Remove only exact
   closing duplicates for computation, leaving 280 vertices per anchor.
   Interpolate by supplied vertex index, separately over both intervals, at
   0.25 s spacing from 2770 to 3000 s: exactly 921 slices of 280 vertices.
   Preserve all anchors and original geometry. This is a declared model,
   not measured intermediate physical motion.
3. Use the public planner with start `[80,0]` at 2770 s, goal `[0,80]` at
   3000 s, workspace `[-180,180]` by `[-90,90]`, and per-axis velocity
   `[2,2]`, acceleration `[0.75,0.75]`, and jerk `[2,2]`. Apply no extra
   margin. First solve the chat's fixed-arrival request; keep earliest arrival
   available as an explicit public override. Distinguish discrete arrival
   search from a proof of globally earliest arrival.
4. Diagnose interval models and timed-search eligibility. Equal counts alone
   do not certify corresponding physical motion. The preparer currently
   supports verified translation/convex deformation; unsupported concave
   deformation retains its explicitly labeled conservative interval model.
   Do not weaken geometry or silently substitute a different history.
5. Optimize one complete BMTP trajectory, carrying velocity and acceleration
   through intermediate joins with continuous jerk. Do not impose waypoint
   stops or concatenate independently stopped motions. Any legitimate wait
   needs feasible braking and departure. Check internal speeds and polynomial
   continuity in the returned motion.
6. Profile preparation, search, optimization, and independent validation.
   Retain only general runtime improvements supported by identical-input
   measurements without correctness or motion-quality regression. Preserve
   all source slices and vertices; add no route heuristics.
7. Add source-fidelity/interpolation and complete-motion regressions, plus a
   structurally different regression for any general optimization. Run MATLAB
   core tests and independently validate motions. Report runtime, arrival,
   path length, continuity, selected interval models, and limitations.
8. Commit and push the CSV and this plan, then the reviewed implementation,
   tests, example, and benchmark report. Keep generated profiling artifacts
   out of source control.
