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

After a live plan completes, select **Save diagnosis bundle** to download a
MAT file for the exact displayed result. The file contains the same versioned
`diagnosisBundle` workflow used by the MATLAB sandbox: canonical planner
inputs, resolved options, the unprojected success or failure result,
independent validation, original browser geometry, environment metadata, and
reproduction commands. MATLAB-only cancellation callbacks are removed. The
button is enabled only while the matching live result remains current; editing
the request or loading an unrelated result disables it.

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

Every connection has bounded headers, a 16 MiB body limit, a read timeout, an
exact UTF-8 `Content-Length`, `Connection: close`, and cleanup on normal or
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
    "AllowAzimuthWrapping": false,
    "Verbose": false
  }
}
```

- All positions and polygon rows are `[azimuth, elevation]` in degrees.
- Time is seconds. Derivatives use `deg/s`, `deg/s^2`, and `deg/s^3`.
- `obstacles` may be `[]`. Each nonempty obstacle has a nonnegative margin and
  one or more strictly increasing keyframes. Every `vertices_deg` value is a
  finite N-by-2 array with at least three rows.
- The page repeats a static polygon at mission start and end. Moving polygons
  use 21 keyframes with the same profiles as the MATLAB sandbox.
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
- MAT bundle download requires live mode because only MATLAB retains the full,
  unprojected planner result. Offline request/result JSON remains intentionally
  bounded for file handoff and cannot reconstruct omitted solver diagnostics.
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
