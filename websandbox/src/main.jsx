import React, { useCallback, useEffect, useRef, useState } from 'react';
import { createRoot } from 'react-dom/client';
import * as THREE from 'three';
import './styles.css';

const initialScene = { obstacles: [], initialState: { time_s: 0, position_deg: [-8, -5], velocity_deg_s: [0, 0], acceleration_deg_s2: [0, 0] }, goalState: { time_s: 20, position_deg: [8, 5], velocity_deg_s: [0, 0], acceleration_deg_s2: [0, 0] }, limits: { maxVelocity_deg_s: [2, 2], maxAcceleration_deg_s2: [0.75, 0.75], maxJerk_deg_s3: [2.5, 2.5], azimuthInterval_deg: [-180, 180], elevationInterval_deg: [-90, 90] }, plannerOptions: { GoalTimeMode: 'fixedArrival', MaximumSeedCount: 3 } };
const sceneBounds = (scene) => ({ left: scene.limits.azimuthInterval_deg[0], right: scene.limits.azimuthInterval_deg[1], bottom: scene.limits.elevationInterval_deg[0], top: scene.limits.elevationInterval_deg[1] });

const clamp = (value, low, high) => Math.min(Math.max(value, low), high);

function createViewport(bounds, width_px, height_px) {
  const centerAzimuth_deg = (bounds.left + bounds.right) / 2;
  const centerElevation_deg = (bounds.bottom + bounds.top) / 2;
  const workspaceWidth_deg = bounds.right - bounds.left;
  const workspaceHeight_deg = bounds.top - bounds.bottom;
  const canvasAspect = width_px / Math.max(height_px, 1);
  let viewportWidth_deg = workspaceWidth_deg * 1.08;
  let viewportHeight_deg = workspaceHeight_deg * 1.12;
  if (viewportWidth_deg / viewportHeight_deg > canvasAspect) {
    viewportHeight_deg = viewportWidth_deg / canvasAspect;
  } else {
    viewportWidth_deg = viewportHeight_deg * canvasAspect;
  }
  return {
    left: centerAzimuth_deg - viewportWidth_deg / 2,
    right: centerAzimuth_deg + viewportWidth_deg / 2,
    bottom: centerElevation_deg - viewportHeight_deg / 2,
    top: centerElevation_deg + viewportHeight_deg / 2,
  };
}

function createTicks(minimum_deg, maximum_deg) {
  const desiredStep_deg = (maximum_deg - minimum_deg) / 8;
  const exponent = 10 ** Math.floor(Math.log10(desiredStep_deg));
  const multiplier = [1, 2, 5, 10].find((value) => value * exponent >= desiredStep_deg);
  const step_deg = (multiplier || 10) * exponent;
  const firstTick_deg = Math.ceil(minimum_deg / step_deg) * step_deg;
  const ticks = [];
  for (let tick_deg = firstTick_deg; tick_deg <= maximum_deg + step_deg * 1e-8; tick_deg += step_deg) {
    ticks.push(Math.abs(tick_deg) < step_deg * 1e-8 ? 0 : +tick_deg.toFixed(8));
  }
  return ticks;
}

function degreeLabel(value_deg) {
  return `${Number.isInteger(value_deg) ? value_deg : value_deg.toFixed(1)}°`;
}

function formatPosition(position_deg) {
  return `(${position_deg[0].toFixed(2)}°, ${position_deg[1].toFixed(2)}°)`;
}

function CoordinateFrame({ bounds, viewport, initialPosition_deg, goalPosition_deg }) {
  if (!viewport) return null;
  const toLeftPercent = (value_deg) => (value_deg - viewport.left) / (viewport.right - viewport.left) * 100;
  const toBottomPercent = (value_deg) => (value_deg - viewport.bottom) / (viewport.top - viewport.bottom) * 100;
  const azimuthTicks_deg = createTicks(bounds.left, bounds.right);
  const elevationTicks_deg = createTicks(bounds.bottom, bounds.top);
  return <div className="coordinate-frame" aria-hidden="true">
    {azimuthTicks_deg.map((tick_deg) => <span className="azimuth-tick-label" key={`az-${tick_deg}`} style={{ left: `${toLeftPercent(tick_deg)}%` }}>{degreeLabel(tick_deg)}</span>)}
    {elevationTicks_deg.map((tick_deg) => <span className="elevation-tick-label" key={`el-${tick_deg}`} style={{ bottom: `${toBottomPercent(tick_deg)}%` }}>{degreeLabel(tick_deg)}</span>)}
    <span className="marker-label start-marker-label" style={{ left: `${toLeftPercent(initialPosition_deg[0])}%`, bottom: `${toBottomPercent(initialPosition_deg[1])}%` }}>Start {formatPosition(initialPosition_deg)}</span>
    <span className="marker-label goal-marker-label" style={{ left: `${toLeftPercent(goalPosition_deg[0])}%`, bottom: `${toBottomPercent(goalPosition_deg[1])}%` }}>Goal {formatPosition(goalPosition_deg)}</span>
    <span className="azimuth-axis-label">Azimuth (deg)</span>
    <span className="elevation-axis-label">Elevation (deg)</span>
  </div>;
}

function Plane({ scene, result, time, mode, onPoint, onVertexMove, onCursorMove }) {
  const mount = useRef();
  const [viewport, setViewport] = useState();
  useEffect(() => {
    const bounds = sceneBounds(scene);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    const width_px = mount.current.clientWidth;
    const height_px = mount.current.clientHeight;
    const currentViewport = createViewport(bounds, width_px, height_px);
    const camera = new THREE.OrthographicCamera(
      currentViewport.left, currentViewport.right, currentViewport.top,
      currentViewport.bottom, 0.1, 100,
    );
    camera.position.z = 10;
    const world = new THREE.Scene(); world.background = new THREE.Color('#101827');
    const drawLine = (points, color, closed = false) => { if (!points?.length) return; const geometry = new THREE.BufferGeometry().setFromPoints((closed ? [...points, points[0]] : points).map(([x, y]) => new THREE.Vector3(x, y, 0))); world.add(new THREE.Line(geometry, new THREE.LineBasicMaterial({ color }))); };
    const drawVertices = (points, color) => { if (!points?.length) return; const geometry = new THREE.BufferGeometry().setFromPoints(points.map(([x, y]) => new THREE.Vector3(x, y, 0.1))); world.add(new THREE.Points(geometry, new THREE.PointsMaterial({ color, size: 8, sizeAttenuation: false }))); };
    const drawGrid = () => {
      const azimuthTicks_deg = createTicks(bounds.left, bounds.right);
      const elevationTicks_deg = createTicks(bounds.bottom, bounds.top);
      for (const azimuth_deg of azimuthTicks_deg) {
        drawLine([[azimuth_deg, bounds.bottom], [azimuth_deg, bounds.top]],
          azimuth_deg === 0 ? 0x64748b : 0x26354d);
      }
      for (const elevation_deg of elevationTicks_deg) {
        drawLine([[bounds.left, elevation_deg], [bounds.right, elevation_deg]],
          elevation_deg === 0 ? 0x64748b : 0x26354d);
      }
      drawLine([
        [bounds.left, bounds.bottom], [bounds.right, bounds.bottom],
        [bounds.right, bounds.top], [bounds.left, bounds.top],
      ], 0x94a3b8, true);
    };
    drawGrid();
    const activeSlices = scene.obstacles.map((obstacle) => { const index = Math.max(0, obstacle.slices.findLastIndex((_, sliceIndex) => (obstacle.time_s?.[sliceIndex] ?? 0) <= time)); const vertices = obstacle.slices[index]?.vertices_deg ?? []; drawLine(vertices, 0xee7d72, true); if (mode === 'edit') drawVertices(vertices, 0xfbbf24); return index; });
    drawVertices([scene.initialState.position_deg], 0x4ade80); drawVertices([scene.goalState.position_deg], 0xf87171); if (result?.position_deg?.length) drawLine(result.position_deg, 0x60a5fa);
    renderer.setSize(width_px, height_px); mount.current.appendChild(renderer.domElement); renderer.render(world, camera);
    setViewport(currentViewport);
    const toPosition = (event) => { const box = renderer.domElement.getBoundingClientRect(); const azimuth_deg = currentViewport.left + (event.clientX - box.left) / box.width * (currentViewport.right - currentViewport.left); const elevation_deg = currentViewport.top - (event.clientY - box.top) / box.height * (currentViewport.top - currentViewport.bottom); return [clamp(azimuth_deg, bounds.left, bounds.right), clamp(elevation_deg, bounds.bottom, bounds.top)]; };
    let drag = null;
    const findVertex = (event) => { const position = toPosition(event); const box = renderer.domElement.getBoundingClientRect(); const threshold_deg = 12 / box.width * (bounds.right - bounds.left); let nearest = null; scene.obstacles.forEach((obstacle, obstacleIndex) => obstacle.slices[activeSlices[obstacleIndex]]?.vertices_deg.forEach((vertex, vertexIndex) => { const distance = Math.hypot(vertex[0] - position[0], vertex[1] - position[1]); if (distance <= threshold_deg && (!nearest || distance < nearest.distance)) nearest = { obstacleIndex, vertexIndex, distance }; })); return nearest; };
    const pointerDown = (event) => { if (mode === 'edit') drag = findVertex(event); }; const pointerMove = (event) => { const position_deg = toPosition(event); onCursorMove(position_deg); if (drag) onVertexMove(drag, position_deg); }; const pointerLeave = () => onCursorMove(null); const pointerUp = (event) => { if (drag) { drag = null; return; } if (mode !== 'edit') onPoint(toPosition(event)); };
    renderer.domElement.addEventListener('pointerdown', pointerDown); renderer.domElement.addEventListener('pointermove', pointerMove); renderer.domElement.addEventListener('pointerup', pointerUp);
    renderer.domElement.addEventListener('pointerleave', pointerLeave);
    return () => { renderer.domElement.removeEventListener('pointerdown', pointerDown); renderer.domElement.removeEventListener('pointermove', pointerMove); renderer.domElement.removeEventListener('pointerup', pointerUp); renderer.domElement.removeEventListener('pointerleave', pointerLeave); renderer.dispose(); mount.current?.replaceChildren(); };
  }, [scene, result, time, mode, onPoint, onVertexMove, onCursorMove]);
  const bounds = sceneBounds(scene);
  return <div className="plane"><div className="canvas-mount" ref={mount} /><CoordinateFrame bounds={bounds} viewport={viewport} initialPosition_deg={scene.initialState.position_deg} goalPosition_deg={scene.goalState.position_deg} /></div>;
}

function App() {
  const [scene, setScene] = useState(initialScene); const [mode, setMode] = useState('polygon'); const [draft, setDraft] = useState([]); const [job, setJob] = useState(null); const [response, setResponse] = useState(); const [time, setTime] = useState(0); const [latency, setLatency] = useState(); const [cursorPosition_deg, setCursorPosition_deg] = useState(null);
  const point = useCallback((position_deg) => { if (mode === 'start') setScene((current) => ({ ...current, initialState: { ...current.initialState, position_deg } })); else if (mode === 'goal') setScene((current) => ({ ...current, goalState: { ...current.goalState, position_deg } })); else if (mode === 'polygon') setDraft((current) => [...current, position_deg]); }, [mode]);
  const moveVertex = useCallback(({ obstacleIndex, vertexIndex }, position_deg) => setScene((current) => ({ ...current, obstacles: current.obstacles.map((obstacle, index) => index !== obstacleIndex ? obstacle : { ...obstacle, slices: obstacle.slices.map((slice) => slice.vertices_deg.length > vertexIndex ? { ...slice, vertices_deg: slice.vertices_deg.map((vertex, index) => index === vertexIndex ? position_deg : vertex) } : slice) }) })), []);
  const updateCursor = useCallback((position_deg) => setCursorPosition_deg(position_deg), []);
  const finishPolygon = () => { if (draft.length >= 3) setScene((current) => ({ ...current, obstacles: [...current.obstacles, { name: `polygon ${current.obstacles.length + 1}`, time_s: [current.initialState.time_s, current.goalState.time_s], safetyMargin_deg: 0, slices: [{ vertices_deg: draft }, { vertices_deg: draft }] }] })); setDraft([]); };
  const queue = async (url, options, type) => { setResponse(); setLatency(); const startedAt_ms = performance.now(); const result = await fetch(url, options); const payload = await result.json(); if (!payload.Success) throw new Error(payload.Error?.Message || 'The bridge rejected the request.'); setJob({ id: payload.JobId, type, startedAt_ms }); };
  const run = () => queue('/api/plan', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ ...scene, mode: 'trajectory' }) }, 'plan');
  const importBundle = async (event) => { const [bundle] = event.target.files; if (!bundle) return; await queue('/api/bundles/import', { method: 'POST', headers: { 'Content-Type': 'application/octet-stream', 'X-Bundle-Name': bundle.name }, body: bundle }, 'import'); event.target.value = ''; };
  const exportBundle = () => queue('/api/bundles/export', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ ...scene, bundleFileName: 'az_el_web_sandbox.mat' }) }, 'export');
  useEffect(() => { if (!job) return undefined; const timer = setInterval(async () => { const poll = await fetch(`/api/jobs/${job.id}`); const payload = await poll.json(); if (payload.Status !== 'completed') return; clearInterval(timer); if (job.type === 'import' && payload.Response.Scene) setScene(payload.Response.Scene); setResponse(payload.Response); requestAnimationFrame(() => setLatency({ clickToRender_s: (performance.now() - job.startedAt_ms) / 1000, planner_s: payload.Response.ElapsedWallTime_s })); }, 100); return () => clearInterval(timer); }, [job]);
  const cancel = () => fetch(`/api/jobs/${job.id}/cancel`, { method: 'POST' }); const duration = response?.Result?.TrajectoryDuration_s || scene.goalState.time_s; const busy = job && !response; const transport_s = latency ? Math.max(0, latency.clickToRender_s - (latency.planner_s || 0)) : undefined;
  return <main><header><h1>Az/El Planner Sandbox</h1><span>{response?.Result?.TerminationReason || response?.Error?.Identifier || (busy ? 'planning' : 'ready')}</span></header><section className="workspace"><section><div className="position-readout"><span>Cursor: {cursorPosition_deg ? formatPosition(cursorPosition_deg) : 'outside workspace'}</span><span>Start: {formatPosition(scene.initialState.position_deg)}</span><span>Goal: {formatPosition(scene.goalState.position_deg)}</span></div><Plane scene={scene} result={response?.Result} time={time} mode={mode} onPoint={point} onVertexMove={moveVertex} onCursorMove={updateCursor} /></section><aside><button onClick={() => setMode('polygon')}>Draw polygon</button><button onClick={finishPolygon}>Finish polygon ({draft.length})</button><button onClick={() => setMode('edit')}>Edit vertices</button><button onClick={() => setMode('start')}>Place start</button><button onClick={() => setMode('goal')}>Place goal</button><button disabled={busy} onClick={run}>Run planner</button><button disabled={!busy || job.type !== 'plan'} onClick={cancel}>Cancel</button><label>Load sandbox bundle <input aria-label="Load sandbox bundle" type="file" accept=".mat" disabled={busy} onChange={importBundle} /></label><button disabled={busy} onClick={exportBundle}>Save sandbox bundle</button><label>Mission time <input value={scene.goalState.time_s} type="number" onChange={(event) => setScene((current) => ({ ...current, goalState: { ...current.goalState, time_s: +event.target.value } }))} /></label><label>Velocity az/el <input value={scene.limits.maxVelocity_deg_s.join(',')} onChange={(event) => setScene((current) => ({ ...current, limits: { ...current.limits, maxVelocity_deg_s: event.target.value.split(',').map(Number) } }))} /></label>{latency && <output>Click to render: {latency.clickToRender_s.toFixed(3)} s; planner: {latency.planner_s.toFixed(3)} s; transport/UI: {transport_s.toFixed(3)} s</output>}{response?.DownloadUrl && <a href={response.DownloadUrl}>Download saved MATLAB bundle</a>}<pre>{JSON.stringify(response?.Diagnostics || response?.Error || { draftVertices: draft.length, obstacleCount: scene.obstacles.length, editMode: mode === 'edit' }, null, 2)}</pre></aside></section><input aria-label="Trajectory time" className="scrub" type="range" min={scene.initialState.time_s} max={duration} step="0.01" value={Math.min(time, duration)} onChange={(event) => setTime(+event.target.value)} /></main>;
}
createRoot(document.getElementById('root')).render(<App />);
