# Az/El Planner Sandbox

This folder provides a dependency-free browser front end for the maintained
Az/El planner. MATLAB can serve the page and planning API itself through its
shipped JVM, or the original request/result file handoff can run with no server
and no network. Neither mode needs Node, Python, a package manager, a CDN, or a
new toolbox.

The page always displays one explicit mode:

- **Live · MATLAB connected** sends a request directly to MATLAB and loads the
  returned result automatically.
- **Offline · file handoff** preserves the numbered download, MATLAB command,
  and result-file selection steps.

## Scene editor

The light workspace follows three steps: build your scene, plan in MATLAB,
and inspect the returned motion. Choose a drawing tool above the canvas;
the active tool is highlighted in blue and the canvas hint explains its action.
Exact endpoints and physical limits are visible in the controls. Expand
**Workspace bounds (deg)** or **Planner options** for additional settings.
The connection badge stays visible on small screens, and offline handoff
labels each of its three steps, including the MATLAB command.

Choose **Rectangle** (or press R), then drag between opposite corners to add a
four-vertex polygon. Drawing works in any direction and stops at workspace
bounds. Escape or a canceled pointer gesture discards the draft. Rectangles
use the same editing, motion, safety-margin, and export behavior as polygons.

New obstacles are selected immediately and show **transform handles**. Drag
the body to move it, a corner of the outline to resize it, or the round handle
to rotate it. These actions require no tool-button selection. Initial-shape
edits must remain within the workspace; exact vertex editing remains available.

## Live mode: MATLAB serves the page and planner

In MATLAB, add this folder's parent to the path and start the blocking server:

```matlab
repositoryRoot = "C:\path\to\the\repository";
addpath(fullfile(repositoryRoot, "offlinesandbox"));
offlineSandbox.serveSandbox();
```

MATLAB prints the URL to open, normally `http://127.0.0.1:52731/`, and the
path of its stop file. Open that printed URL, create the scene, and select
**Plan in MATLAB**. The page sends its existing `offlineSandboxRequest/v1`
JSON to `POST /plan` and passes the returned `offlineSandboxResult/v1` object
to the same result loader used by offline mode.

Disconnected HTTP clients are reported as undeliverable responses. In
particular, a browser health probe that times out during planning must not
abort the planner through its cancellation callback. Socket write failures
are contained at the response boundary; other errors still propagate and a
valid cancellation remains accepted even if its acknowledgement cannot be sent.

After a live plan completes, select **Save diagnosis bundle** to download a
MAT file for the exact displayed result. The file contains the same versioned
`diagnosisBundle` workflow used by the MATLAB sandbox: canonical planner
inputs, resolved options, the unprojected success or failure result,
independent validation, original browser geometry, environment metadata, and
reproduction commands. MATLAB-only cancellation callbacks are removed. The
button is enabled only while the matching live result remains current; editing
the request or loading an unrelated result disables it.

Select **Load & run diagnosis bundle** to upload a v2 diagnosis MAT file to
the loopback server. MATLAB reconstructs its canonical initial state, goal,
limits, original obstacle histories, safety margins, and resolved planner
options, then runs the current planner. The displayed result is therefore a
fresh reproduction, not the result stored in the bundle. Replayed bundles may
be saved again, and cooperative cancellation remains available while they run.

Select a polygon to open its floating **Copy**, **Rotate**, **Set motion**, and
**Delete** actions. **Set motion** rewinds to mission start and displays a purple
ghost of its final pose. Edit the ghost directly using its outline; no move,
stretch, or rotate tool selection is needed. Its small floating menu contains
only **Preview** and the green **Finish final pose** checkmark:

- **Move:** grab the ghost body or center, or expand **Exact final pose values** in the obstacle
  sidebar to enter its final center. This sets constant translation to reach that center at mission end.
  Destinations requiring more than the existing 10 deg/s center-speed limit are
  refused; increase mission time or choose a nearer destination.
- **Stretch:** drag an always-visible purple corner to change width and height about the
  ghost's center, along its rotated local axes. Numeric width/height factors
  are also available; 1× retains the original size and 0.01× is the minimum.
- **Rotate:** drag the round handle extending from the ghost, or enter **Turn from start (deg)**.
  Positive turns are counter-clockwise, negative turns clockwise, with a
  supported range of -360 to 360 degrees. Pure rotation needs no translation.

Select the green checkmark, press Escape, or start mission playback to leave final-pose editing.
The original polygon remains the mission-start shape; turn and scale progress
over the mission. Copy preserves these settings. **Make stationary** clears
translation, turn, and stretch. Rotation and scale edits preserve the selected
translation profile; changing destination explicitly selects constant velocity.
The **Obstacle speed** slider overlays the lower-right corner of the canvas
when an editable obstacle is selected. It controls commanded translation speed from
0 to 10 deg/s, with a live numeric readout. It preserves direction through zero
and retains the selected motion profile; starting a new stationary obstacle
defaults to constant motion toward +Az. Drag the ghost to choose another
direction. Exact velocity fields are under **Direction components**.
The velocity fields and arrow handle remain available for zero-start,
trapezoidal, and out-and-back translation. Changing mission duration retains
the commanded velocity, so the ghost destination updates accordingly.

The ghost's play icon and **Preview obstacles** below the canvas run obstacle
animation locally, without MATLAB, start/goal points, or a planner result.
The preview includes translation, rotation, and stretch. Each obstacle's remaining
centroid path appears as a dashed line and its final shape stays visible as a
purple ghost throughout playback, including for imported result histories.
The line disappears behind the obstacle and is gone at the end; scrubbing
backward restores the remaining path for that earlier time.
Use the timeline to pause or scrub, then **End preview** to return to editing.
Preview preserves any existing planner result, while
hiding its path and kinematic charts until the preview ends. It does not run
planning or collision validation. Exact numeric pose fields are collapsed in
the sidebar rather than displayed in a bar above the canvas.

Preview and export use the same sampled motion. Translation or stretch uses at
least 20 intervals; rotation adds intervals to keep each angular step at most
5 degrees. Between samples, corresponding vertices follow straight segments,
not exact rigid rotation arcs. This is the existing polygon-history model in
`obstacle_history_contract.md`. MATLAB may conservatively enclose unsupported
history intervals; the response retains its protected geometry. Safety margins
are still applied only by MATLAB, and rotation/scale do not inherit the 10 deg/s
translation limit as a bound on every boundary vertex's speed.

Select **Cancel** to request cooperative cancellation. The server accepts that
request out of band and supplies a trusted MATLAB-only `CancellationCheckFcn`
to the existing adapter. The public planner stops at its next safe checkpoint;
the callback is never accepted from JSON and is not returned on the wire.

To use another port, pass one integer from 1024 through 65535 and open the URL
MATLAB prints:

```matlab
offlineSandbox.serveSandbox(52732);
```

A local HTML file can discover only the default port. Opening the printed URL
is therefore required for a custom port. Stop the server with Ctrl-C or create
the exact stop file whose path MATLAB printed. The accept loop polls every
250 ms, so an idle server releases the socket promptly.

The server binds `java.net.ServerSocket` to `127.0.0.1` explicitly, never
`0.0.0.0`. It is not reachable from the local network. Browser API requests
are accepted only from the served loopback origin or the local-file origin;
non-browser scripted clients may omit `Origin`.

Browsers represent a local `file://` page with the opaque `Origin: null` value;
sandboxed remote documents can use that same opaque value. The server accepts
it deliberately so the unchanged local HTML file can enter live mode. This
loopback utility is not an authentication boundary: run it only while using
the sandbox and stop it when finished.

## Offline mode: unchanged file handoff

1. Double-click `az_el_planner_sandbox.html`. No server or installation is
   needed. The initial loopback probe times out promptly and the page plainly
   displays **Offline · file handoff**.
2. Place the start and goal by clicking the plot or entering their exact
   coordinates, draw any polygon obstacles, edit the limits, and select
   **Download request JSON**. The browser saves
   `az-el-request.json` in its configured download folder.
3. In MATLAB, run this command block after replacing `repositoryRoot` with this
   checkout's root if necessary:

   ```matlab
   repositoryRoot = "C:\path\to\the\repository";
   addpath(fullfile(repositoryRoot, "offlinesandbox"));
   requestFile = fullfile( ...
       getenv("USERPROFILE"), "Downloads", "az-el-request.json");
   resultFile = fullfile( ...
       getenv("USERPROFILE"), "Downloads", "az-el-result.json");
   offlineSandbox.runPlanningRequest(requestFile, resultFile);
   ```

   The function finds the repository from its own location, adds the two
   production parents, constructs obstacles through
   `obstacleAvoidance.obstacles.createObstacle`, calls the unchanged public
   `obstacleAvoidance.planTrajectory` entry point once, and independently
   validates a successful result with
   `obstacleAvoidance.validateTrajectory`.
4. Return to the page and select **Load result JSON**. Choose
   `az-el-result.json`.

The two-argument `offlineSandbox.runPlanningRequest(requestFile, resultFile)`
path is unchanged. It still owns JSON validation, canonical obstacle creation,
the one public planner call, independent validation, projection, and atomic
result-file replacement.

The page uses Canvas 2D. It provides direct control of equal Az/El scale,
degree ticks, the grid, dense search traces, and animation while keeping all
rendering code inside the single HTML file.

## Loopback HTTP transport

The MATLAB server implements a small HTTP/1.1 subset directly over
`java.net.ServerSocket`:

- `GET /` returns `az_el_planner_sandbox.html` as UTF-8.
- `GET /health` identifies the local transport and lets the page select live
  mode. The page probes only while no plan is active and checks again every
  three seconds, so a stopped server changes the UI to offline mode visibly.
- `POST /plan` accepts the exact request JSON documented below. MATLAB writes
  it to an adapter-owned temporary file, calls
  `offlineSandbox.runPlanningRequest`, and returns that adapter's exact result
  JSON bytes. No request or result schema is duplicated in the server.
- `POST /cancel` is serviced by the planner's cooperative cancellation callback
  while the main MATLAB thread is planning.
- `POST /bundle` returns the server-cached MAT diagnosis bundle only when the
  supplied request identifier matches the latest completed live plan. The
  cache is deleted when the server stops.
- `POST /run-bundle` accepts a diagnosis MAT file, reconstructs its canonical
  request, runs the current planner, and returns `offlineSandboxResult/v1`.

Every connection has bounded headers, a 16 MiB body limit (128 MiB only for
the MAT replay route), a read timeout, an exact `Content-Length`,
`Connection: close`, and cleanup on normal or
exceptional exit. Unsupported paths, methods, framing, and malformed requests
receive bounded JSON errors rather than terminating the accept loop.

Successful planning responses expose `Server-Timing`,
`X-Offline-Sandbox-Planner-Time-s`, and
`X-Offline-Sandbox-Server-Time-s` headers. The first custom value is the
planner-owned elapsed time already returned in the result; the second measures
server work through adapter validation, planning, projection, and result-file
reading immediately before socket transport.

If MATLAB exits or the listener disappears during a request, the fetch rejects,
request editing is restored, and the page switches explicitly to offline mode.
When a page was opened from the printed HTTP URL, the health response retains
the checkout's sandbox path for the fallback MATLAB command. If the server
vanishes before that path is learned, reopen the local HTML file before using
file handoff.

## Request JSON: `offlineSandboxRequest/v1`

The browser writes this shape:

```json
{
  "schemaVersion": "offlineSandboxRequest/v1",
  "requestId": "az-el-20260830...",
  "obstacles": [
    {
      "name": "Obstacle 1",
      "safetyMargin_deg": 0.2,
      "keyframes": [
        {
          "time_s": 0,
          "vertices_deg": [[-8, -3], [-2, -3], [-2, 3], [-8, 3]]
        },
        {
          "time_s": 20,
          "vertices_deg": [[-8, -3], [-2, -3], [-2, 3], [-8, 3]]
        }
      ]
    }
  ],
  "initialState": {
    "time_s": 0,
    "position_deg": [-15, 0],
    "velocity_deg_s": [0, 0],
    "acceleration_deg_s2": [0, 0]
  },
  "goalState": {
    "time_s": 20,
    "position_deg": [15, 0],
    "velocity_deg_s": [0, 0],
    "acceleration_deg_s2": [0, 0]
  },
  "limits": {
    "maxVelocity_deg_s": [2, 2],
    "maxAcceleration_deg_s2": [0.75, 0.75],
    "maxJerk_deg_s3": [2.5, 2.5],
    "azimuthInterval_deg": [-180, 180],
    "elevationInterval_deg": [-90, 90]
  },
  "options": {
    "GoalTimeMode": "earliestArrival",
    "AllowAzimuthWrapping": false
  }
}
```

- All positions and polygon rows are `[azimuth, elevation]` in degrees.
- Time is seconds. Derivatives use `deg/s`, `deg/s^2`, and `deg/s^3`.
- `obstacles` may be `[]`. Each nonempty obstacle has a nonnegative margin and
  one or more strictly increasing keyframes. Every `vertices_deg` value is a
  finite N-by-2 array with at least three rows.
- The page repeats a static polygon at mission start and end. Translation and
  stretch use at least 21 keyframes. Rotation uses additional samples as needed
  to limit angular steps to 5 degrees; see the final-pose controls above.
- `options` is a partial public planner-options structure. JSON callbacks are
  prohibited; in particular, `CancellationCheckFcn` is not accepted. Live
  cancellation is injected only as a trusted MATLAB argument after this check.
- The MATLAB constructor owns safety inflation. The page never preinflates
  request geometry.

## Result JSON: `offlineSandboxResult/v1`

The MATLAB function writes this wrapper:

```text
schemaVersion       "offlineSandboxResult/v1"
requestId           copied from the request
generatedAtUtc      UTC timestamp
result
  Success, Message, TerminationReason
  Options
  Inputs
    initialState, goalState, limits
  SelectedSeedIndex, SelectedSeed_deg
  time_s, position_deg, velocity_deg_s
  acceleration_deg_s2, jerk_deg_s3
  ArrivalTime_s, TrajectoryDuration_s, GoalHorizon_s
  ElapsedPlanningTime_s
  SearchDiagnostics
    TerminationReason, AttemptedSeedCount, ValidatedCandidateCount
    BestPartialSeedIndex, FirstValidatedMotionTime_s
    SeedGenerationElapsedTime_s, SeedSummaries, StageTiming
    Grid
      Bounds_deg, AcceptedEdges_deg, RejectedEdges_deg
      ExploredNodes_deg, FrontierNodes_deg, BestPartialRoute_deg
      Start_deg, Goal_deg, NodeCount, ExpandedCount
      RejectedTransitionCount, GeneratedSeedCount, TraceDownsampleRule
validation            public independent-validation record
obstacles[]
  Name, time_s, status, SafetyMargin_deg
  OriginalVerticesByTime_deg, ProtectedVerticesByTime_deg
```

`SeedSummaries` retains the public summary fields through `Message`, while
nested solver internals are intentionally not placed on the browser wire.
`validation` is the complete stable record returned by
`obstacleAvoidance.validateTrajectory` on success, or the planner's stable
failure validation record on an expected planning failure.

Unavailable MATLAB `NaN` and `Inf` values are encoded as JSON `null`. The page
displays them as unavailable and never converts them to zero. Expected no-path
and work-limit outcomes still produce a result file with `Success=false`, the
termination reason, seed summaries, timing, and any retained search geometry.
Invalid JSON or invalid planner requirements raise an identified MATLAB error
in the file handoff and become a bounded HTTP 400 error in live mode.

## Deliberate offline and display limitations

- In offline mode, browser security requires the explicit download and
  file-input steps. The page cannot discover where a download was saved,
  launch MATLAB, or reload the result automatically. The displayed command
  assumes the usual Windows `Downloads` folder; edit the two paths if the
  browser uses another folder.
- MAT bundle download and replay require live mode because MATLAB owns the
  unprojected result and MAT decoding. Offline request/result JSON remains
  intentionally bounded and cannot reconstruct omitted solver diagnostics.
- The page does not plan, inflate geometry, check collisions, or certify
  dynamics. Those responsibilities remain in the unchanged MATLAB code.
  Protected geometry becomes visible only after a result is loaded.
- When a result containing obstacle histories is loaded without the matching
  request having first been downloaded in that page session, the scene is
  deliberately inspection-only. Returned histories are planner output, not
  lossless editable request keyframes. Select **Reset** to create a new request.
- Page-created moving polygons preserve vertex topology, so their returned
  keyframes interpolate exactly. For an externally authored result whose
  adjacent obstacle keyframes change topology, the dependency-free display
  selects the nearer keyframe instead of reproducing MATLAB's conservative
  polygon union. This is only a rendering limitation; MATLAB planning and
  validation use the canonical geometry.
- The scrubber spans the mission horizon. After an earliest-arrival trajectory
  ends, the displayed vehicle remains at its returned terminal position while
  obstacle playback continues.
- This focused mirror implements required polygon drawing. The MATLAB GUI's
  circle, square, freehand-capsule, and direct in-process plotting are not
  duplicated. Live **Cancel** is cooperative; offline file handoff has no
  in-flight browser cancellation.
- Some browsers restrict clipboard access for `file://`. If **Copy MATLAB
  command** is denied, select the visible command manually.

No external network access, package manager, build step, web font, external
script, or external stylesheet is used. Live mode's only runtime connection is
the explicit `127.0.0.1` HTTP transport; offline mode makes no connection.
