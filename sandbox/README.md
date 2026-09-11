# MATLAB sandbox

Ported from `bmtp-cleanup-codex` commit `c04f3b2` and adapted to `build-core`.
From this checkout's root, run:

```matlab
addpath(fullfile(pwd, 'sandbox'));
ui = obstacleAvoidanceSandbox();
```

Place the start and goal, draw obstacles, set limits and mission time, then
select **Run**. Polygon, circle, square, and hand-drawn line tools are retained.
**Set Motion** assigns a translation profile to a polygon. **Diagnostics**
opens the core plotter's workspace, visibility graph, and kinematic views.
Successful plans can play automatically; disable that with `AnimateOnRun=false`.

```matlab
ui = obstacleAvoidanceSandbox(struct('AnimateOnRun', false, ...
    'PlannerOptions', struct('GoalTimeMode', 'fixedArrival')));
state = ui.ReadState();
state.ExportBundle(fullfile(pwd, 'scratch', 'sandbox.mat'), 'goal');
```

Create the export folder first. Bundles can be saved before planning when the
scene is complete, or after success/failure. They retain the original request,
canonical obstacle geometry, the full core result, and independent validation.
The initial `ui` snapshot is not updated by later interaction; use `ReadState()`.

Replay in MATLAB:

```matlab
loaded = load(fullfile(pwd, 'scratch', 'sandbox.mat'), 'diagnosisBundle');
bundle = loaded.diagnosisBundle;
inputs = bundle.PlannerInputs;
result = planner(inputs.obstacles, inputs.initialState, inputs.goalState, ...
    inputs.limits, bundle.PlannerOptions);
validation = obstacleAvoidance.validateTrajectory(result);
```

The sandbox defaults to earliest arrival. The public planner still owns all
motion construction and validation, with its unchanged C3 tolerances. A
geometric guide shown after failure is not a validated motion. The cleanup
branch's Ruckig fallback selector and seed-count overrides are removed because
this core does not implement them. Wrapping supports obstacle-free,
fixed-position goals only. See the [core README](../README.md) for arrival-search
and moving-obstacle limitations.

The [HTML sandbox](../offlinesandbox/README.md) can also replay these MAT bundles.
