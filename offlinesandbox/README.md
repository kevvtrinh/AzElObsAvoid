# Az/El Offline Planner Sandbox

This folder provides a dependency-free browser front end for the maintained
Az/El planner. It uses a deliberate file handoff because a page opened with
`file://` cannot call MATLAB or read arbitrary local files.

## Open and run

1. Double-click `az_el_planner_sandbox.html`. No server or installation is
   needed.
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

The page uses Canvas 2D. It provides direct control of equal Az/El scale,
degree ticks, the grid, dense search traces, and animation while keeping all
rendering code inside the single HTML file.

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
  prohibited; in particular, `CancellationCheckFcn` is not accepted.
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
Invalid JSON or invalid planner requirements raise an identified MATLAB error.

## Deliberate offline limitations

- Browser security requires the explicit download and file-input steps. The
  page cannot discover where a download was saved, launch MATLAB, or reload the
  result automatically. The displayed command assumes the usual Windows
  `Downloads` folder; edit the two paths if the browser uses another folder.
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
  circle, square, freehand-capsule, stop button, and direct in-process plotting
  are not duplicated.
- Some browsers restrict clipboard access for `file://`. If **Copy MATLAB
  command** is denied, select the visible command manually.

No runtime network access, package manager, build step, web font, external
script, or external stylesheet is used.
