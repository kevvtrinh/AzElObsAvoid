# Az/El Web Sandbox

This sandbox uses a local file-watch transport. MATLAB R2024b has HTTP client
support but no supported server API, and this installation has no Parallel
Computing Toolbox: its solver-capable planner cannot run on `backgroundPool`.
The small local Node bridge remains curl-able while one MATLAB process runs the
unchanged public planner and watches JSON files.

Start the bridge (Node 22+):

```powershell
node websandbox/bridge.mjs
```

In a second terminal, start the MATLAB worker:

```matlab
addpath(fullfile(pwd, "websandbox", "matlab"));
webSandbox.fileProtocol.serve(fullfile(pwd, "websandbox", "runtime"));
```

The bridge exposes `GET /api/health`, `POST /api/plan`,
`GET /api/jobs/{jobId}`, and `POST /api/jobs/{jobId}/cancel`. `POST /api/plan`
accepts the schema in `fixtures/trajectory-request.json`; obstacle `slices`
provide time-indexed `vertices_deg` polygons. The response retains public
motion histories plus termination, validation, stage timing, and seed summaries.

The browser can load and save `obstacleAvoidanceSandboxDiagnosis-v2` MAT
bundles through the local worker. Import keeps canonical original polygon
geometry, obstacle time histories, and the one stored safety margin. Export
writes the same v2 `diagnosisBundle` top-level record used by
`sandbox/exportSandboxDiagnosis.m`; it represents a replayable pre-run goal
request. The web editor supports one finite polygon per obstacle slice.

Run the browser client in a third terminal:

```powershell
npm --prefix websandbox run dev
```

The bridge resolves its runtime directory relative to `bridge.mjs`, so it may
be started from either the repository root or `websandbox/`. Vite proxies its
local `/api` requests to the bridge. The client polls every 100 ms and displays
click-to-render latency, MATLAB planner time, and the residual transport/UI
time after each completed job.

For a curl round trip, post the fixture, then poll the returned job ID:

```powershell
curl.exe -H "Content-Type: application/json" --data-binary "@websandbox/fixtures/trajectory-request.json" http://127.0.0.1:42831/api/plan
curl.exe http://127.0.0.1:42831/api/jobs/job_example
```

`processOnce` supports headless test execution. The worker writes responses
atomically and passes a per-job cancellation-file check through the planner's
existing `CancellationCheckFcn`; cancellation remains cooperative.
