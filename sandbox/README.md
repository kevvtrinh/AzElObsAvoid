# Interactive Az/El sandbox

`azElInteractiveSandbox` is a persistent manual workspace for building and
planning Az/El scenes. Launch it from the repository root:

```matlab
addpath("sandbox");
sandboxState = azElInteractiveSandbox();
```

The Goal Mode tab automatically guides scene input in this order:

1. Click the start position.
2. Click the goal position.
3. Draw the first obstacle stroke.
4. Click **Add Obstacle** before drawing each additional obstacle.
5. Click **Run** when the scene is ready.

The Free Mode tab uses the same sequence, except the second click supplies the
first free-route endpoint. **Add Segment** remains available for optional
later endpoints. A drawn line becomes a buffered path obstacle; closing a
stroke near its starting point creates a polygon obstacle.

Planning controls use three compact groups—Workspace, Kinematic limits, and
Timing and obstacle geometry. Paired values share column headings so each
setting occupies one row and remains readable under Windows display scaling.

The sandbox uses the HS3 planner for Goal Mode and for every Free Mode segment
and arrival-time candidate. `PlannerOptions` accepts partial HS3 overrides;
`PlannerMethod`, when supplied, must be `"hs3"`. Resolved controls and exported
diagnosis bundles retain that method explicitly.

The first obstacle step begins automatically. After a valid obstacle is
stored, mouse drawing returns to idle so tab changes cannot accidentally add
geometry. Use **Add Obstacle** whenever another obstacle is needed.

The legend identifies requested points, requested routes, solved motion, and
failure routes. Obstacle fills, outlines, centerlines, and safety boundaries
remain visible on the canvas but are intentionally omitted from the legend.

**Reset** removes the retained scene and every visible or hidden graphics
object, then restarts the guided sequence at the start-position click.
**Diagnostics** opens the planner diagnostics for the most recent result.

After any planner call, **Export Bundle** saves a diagnosis-ready MAT file for
the active tab. The bundle contains the raw drawn geometry, canonical protected
obstacles, exact planner inputs and resolved options, retained segment results,
latest result, independent validation, planner log, environment metadata, and
copyable reproduction commands. It omits figure handles and callbacks. Send
that MAT file when asking for failure diagnosis; both successful and failed
planner calls can be exported.
