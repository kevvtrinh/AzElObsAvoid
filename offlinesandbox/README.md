# HTML planner sandbox

Ported from `bmtp-cleanup-codex` commit `c04f3b2` and adapted to `build-core`.
The standalone `xy_planner_sandbox.html` contains its own styles and Canvas 2D
renderer. MATLAB owns planning, obstacle protection, and independent validation.
No package installation, build step, or external web service is required.

## Run with MATLAB

From this checkout's root:

```matlab
addpath(fullfile(pwd, 'offlinesandbox'));
offlineSandbox.serveSandbox();
```

Open **http://127.0.0.1:52731/** in a browser. The page should show
**LIVE · MATLAB CONNECTED**. The server runs synchronously in that MATLAB
session. Press Ctrl+C there, or create the exact stop file printed at startup,
to stop it. A custom port can be supplied, for example `serveSandbox(52732)`;
open that server's printed URL. The server binds only to IPv4 loopback.
MATLAB with Java and Optimization Toolbox is required.

## Build and inspect a scene

- Place endpoints on the canvas or enter exact x/y coordinates and apply them.
- Draw polygons, rectangles, circles, or freehand obstacles. Select a shape to
  move it, edit its vertices, resize, rotate, copy, or delete it.
- Set physical limits, workspace bounds, mission time, and safety margins.
  **Earliest arrival** is the initial UI choice. **Arrive at mission time**
  requests the fixed-arrival core problem.
- Select **Plan in MATLAB**. A successful response displays the trajectory,
  velocity, acceleration, jerk, final status, and independent validation.
- Play or scrub the mission. After arrival, the vehicle marker stays at its
  terminal position while obstacle playback continues; this extension is a
  display convention, not an additional certified trajectory.
- Enable the graph/guide overlays, or use **Start replay** and **Super deep
  dive** to inspect the current core's preparation, graph, motion, and checks.

**Set motion** exposes a final-pose ghost. Move its body, drag a corner to
stretch it, or use its rotation handle. Numeric final-pose controls are also
available. Translation profiles include constant velocity, zero-start
acceleration, trapezoidal, and out-and-back motion. The speed slider spans
0–10 coordinate units/s; a configured turn spans −360° to 360°.
**Preview obstacles** runs in the browser without endpoints or MATLAB.

Preview and export use the same keyframes. Translation/stretch uses at least
20 intervals; rotation adds samples to keep angular steps at most 5°. Between
keyframes, corresponding vertices move linearly, so rotation is a sampled
approximation. The translation-speed limit does not bound the speed of every
rotating or stretching boundary vertex. Moving shapes can leave the workspace.
Circle and freehand tools emit ordinary polygons (24 and at most 40 vertices).
MATLAB applies each margin once and returns original and protected histories.

## Save and replay diagnosis bundles

**Save diagnosis bundle** opens MATLAB's native **Save As** dialog for the
matching completed live request. Choose a folder and filename there; switch to
MATLAB if the dialog is behind the browser. The page reports the actual saved
path only after MATLAB confirms the write. Cancel leaves the plan available. **Load & run diagnosis bundle** sends a saved HTML or MATLAB sandbox
bundle to the local server and reruns its canonical request on the current core.
The result is a fresh solve, not a playback of the old solver's trajectory.
Unknown cleanup-only options follow the core's normal warning/ignore policy.

Bundles retain `PlannerInputs`, `PlannerOptions`, the full core `Result`,
`IndependentValidation`, scene data, and reproduction commands in the
`obstacleAvoidanceSandboxDiagnosis-v2` format. The original horizon and supplied
goal are preserved even when the solver chooses an earlier temporal trial.
Reconstruction uses original obstacle vertices and applies the saved margin
once. `Result.VisibilityGraph`, `Result.SolverDiagnostics`, and optional
`Result.TemporalSearch` contain the core evidence. Its compatibility `Diagnosis`
output is empty.

Programmatic replay:

```matlab
addpath(fullfile(pwd, 'offlinesandbox'));
[response, bundle] = offlineSandbox.replayDiagnosisBundle( ...
    'saved-sandbox.mat', 'replayed-result.json');
```

## Offline JSON handoff

Open `xy_planner_sandbox.html` directly. When no default-port server is available,
it shows **Offline · file handoff**. Build a scene and download its request JSON.
Run in MATLAB, using the actual downloaded paths:

```matlab
addpath(fullfile(pwd, 'offlinesandbox'));
requestFile = fullfile(getenv('USERPROFILE'), 'Downloads', 'x-y-request.json');
resultFile = fullfile(getenv('USERPROFILE'), 'Downloads', 'x-y-result.json');
[response, diagnosisBundle] = offlineSandbox.runPlanningRequest(requestFile, resultFile);
```

Select **Load result JSON** in the page. The adapter discovers the repository
from its own location, calls `planner(...)`, independently validates successful
motion, and atomically writes the result file. The destination folder must
exist and the input/output files must differ. Invalid input does not replace
an existing result. The optional second output supplies the full MAT bundle.

## Wire records and graph interpretation

Requests use `offlineSandboxRequest/v1` with `requestId`, `obstacles`,
`initialState`, `goalState`, `limits`, and `options`. Each obstacle has `name`,
`safetyMargin_units`, and ordered `keyframes` containing `time_s` and finite
N-by-2 `vertices_units`. Endpoints contain time and position; omitted velocity
and acceleration default to zero in the core. The browser emits per-axis
derivative limits. External requests may use all scalars or all pairs, following
the core's combined-magnitude contract.

Responses use `offlineSandboxResult/v1`:

```text
requestId, generatedAtUtc
result
  Success, Message, TerminationReason, Options, Inputs
  Request (original endpoints, limits, options), RequestedLimits
  Route_units, time_s, position_units, velocity_units_s
  acceleration_units_s2, jerk_units_s3
  ArrivalTime_s, TrajectoryDuration_s, MotionLength_units, ElapsedTime_s
diagnosis
  Planner = "build-core"
  Search
    Nodes_units, AcceptedEdges_units, RejectedEdges_units
    NodeCount, AcceptedEdgeCount, ExpandedCount, RejectedTransitionCount
    CollisionQueryCount, SearchKind, GraphIsFullyEnumerated
    TraceDownsampleRule
  SolverDiagnostics, TemporalSearch (when present)
validation (public independent-validation record)
obstacles[]
  Name, time_s, status, SafetyMargin_units
  OriginalVerticesByTime_units, ProtectedVerticesByTime_units
```

The graph overlay contains actual returned nodes and examined connections.
Unexamined connections remain implicit; they are not classified as blocked.
The core does not record expansion order or frontier identities. For moving
scenes the spatial guide alone cannot certify collision freedom over time.
The geometric guide can remain available after a failed solve. No cleanup seed
counts, candidate-selection histories, or exclusive stage times are fabricated.
The walkthrough reports the actual C3, endpoint, derivative, history, and plane
checks. A false validation flag may indicate an earlier check stopped validation.
Unavailable numeric diagnostics become JSON `null`, displayed as unavailable.

## Transport and limitations

The local HTTP server supports `GET /`, `GET /health`, `POST /plan`,
`POST /bundle`, `POST /save-bundle`, and `POST /run-bundle`, plus their browser preflight requests.
It validates request framing and origins; bundle downloads require the matching
cached request ID. `/save-bundle` uses a native chooser and never accepts a
destination path from HTTP; `/bundle` retains the raw MAT download API. Cache and adapter temporary files are removed when the server
stops. `Server-Timing` and the `X-Offline-Sandbox-*-Time-s` headers report the
planner and server durations. Expected no-path outcomes remain HTTP 200 results
with `Success=false`; malformed requests return bounded error responses.

- C3 continuity and all core validation tolerances are unchanged. The cleanup
  Ruckig fallback and seed-count settings are not offered by this UI.
- Wrapping supports obstacle-free, fixed-position goals only. Earliest-arrival
  searches do not prove a global optimum. Temporal trials can leave unsearched
  gaps or exhaust their explicit budget; see the [core README](../README.md).
- A result loaded without its matching editable request is inspection-only
  when it contains obstacle histories. Reset to author a fresh scene.
- External topology-changing histories may be rendered using the nearer
  keyframe. MATLAB's protected geometry and independent certification remain
  authoritative; browser interpolation does not validate collision freedom.
- The editor sets zero endpoint velocity and acceleration. Full endpoint states
  and moving-target histories can be supplied through JSON or replay bundles.
- Offline mode requires explicit file download/load. MAT decoding and bundle
  replay require MATLAB. Clipboard access from a local file can be restricted
  by the browser; the command can also be copied manually.
- Interrupt a long live plan in MATLAB with Ctrl+C. There is no HTTP cancel
  endpoint. A page opened as a file discovers only the default port.
