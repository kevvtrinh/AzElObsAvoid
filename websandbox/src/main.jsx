import React, { useCallback, useEffect, useRef, useState } from 'react';
import { createRoot } from 'react-dom/client';
import * as THREE from 'three';
import './styles.css';
import {
  coordinateToFramePixel,
  coordinateToFramePercent,
  createTicks,
  createViewport,
} from './coordinateFrame.js';

const initialScene = { obstacles: [], initialState: { time_s: 0, position_deg: [-8, -5], velocity_deg_s: [0, 0], acceleration_deg_s2: [0, 0] }, goalState: { time_s: 20, position_deg: [8, 5], velocity_deg_s: [0, 0], acceleration_deg_s2: [0, 0] }, limits: { maxVelocity_deg_s: [2, 2], maxAcceleration_deg_s2: [0.75, 0.75], maxJerk_deg_s3: [2.5, 2.5], azimuthInterval_deg: [-180, 180], elevationInterval_deg: [-90, 90] }, plannerOptions: { GoalTimeMode: 'fixedArrival', MaximumSeedCount: 3 } };
const sceneBounds = (scene) => ({ left: scene.limits.azimuthInterval_deg[0], right: scene.limits.azimuthInterval_deg[1], bottom: scene.limits.elevationInterval_deg[0], top: scene.limits.elevationInterval_deg[1] });

const clamp = (value, low, high) => Math.min(Math.max(value, low), high);

function degreeLabel(value_deg) {
  return `${Number.isInteger(value_deg) ? value_deg : value_deg.toFixed(1)}°`;
}

function finitePolygonPaths(vertices_deg) {
  const paths = [];
  let path = [];
  for (const vertex_deg of vertices_deg || []) {
    if (Number.isFinite(vertex_deg[0]) && Number.isFinite(vertex_deg[1])) {
      path.push(vertex_deg);
    } else if (path.length >= 3) {
      paths.push(path);
      path = [];
    }
  }
  if (path.length >= 3) paths.push(path);
  return paths;
}

function formatPosition(position_deg) {
  return `(${position_deg[0].toFixed(2)}°, ${position_deg[1].toFixed(2)}°)`;
}

function formatTime(time_s) {
  return `${time_s.toFixed(2)} s`;
}

function CoordinateFrame({ bounds, frame, initialPosition_deg, goalPosition_deg, initialTime_s, goalTime_s, result }) {
  if (!frame) return null;
  const { viewport, width_px, height_px } = frame;
  const toFramePercent = (position_deg) => coordinateToFramePercent(position_deg, viewport);
  const toFramePixel = (position_deg) => coordinateToFramePixel(
    position_deg, viewport, width_px, height_px,
  );
  const azimuthTicks_deg = createTicks(bounds.left, bounds.right);
  const elevationTicks_deg = createTicks(bounds.bottom, bounds.top);
  const trajectoryTime_s = result?.time_s || [];
  const trajectoryPosition_deg = result?.position_deg || [];
  const hasTrajectory = trajectoryTime_s.length > 0 && trajectoryPosition_deg.length > 0;
  const startTime_s = hasTrajectory ? trajectoryTime_s[0] : initialTime_s;
  const arrivalTime_s = hasTrajectory ? trajectoryTime_s[trajectoryTime_s.length - 1] : goalTime_s;
  const startPosition_deg = hasTrajectory ? trajectoryPosition_deg[0] : initialPosition_deg;
  const endPosition_deg = hasTrajectory ? trajectoryPosition_deg[trajectoryPosition_deg.length - 1] : goalPosition_deg;
  const workspaceTopLeft = toFramePixel([bounds.left, bounds.top]);
  const workspaceBottomRight = toFramePixel([bounds.right, bounds.bottom]);
  const startFramePercent = toFramePercent(startPosition_deg);
  const endFramePercent = toFramePercent(endPosition_deg);
  return <div className="coordinate-frame" aria-hidden="true">
    <svg className="coordinate-grid" viewBox={`0 0 ${width_px} ${height_px}`} preserveAspectRatio="none">
      {azimuthTicks_deg.map((tick_deg) => { const point = toFramePixel([tick_deg, bounds.bottom]); return <React.Fragment key={`az-${tick_deg}`}><line className={tick_deg === 0 ? 'zero-gridline' : 'gridline'} x1={point.x} x2={point.x} y1={workspaceTopLeft.y} y2={workspaceBottomRight.y} /><text className="azimuth-tick-label" x={point.x} y={point.y + 5}>{degreeLabel(tick_deg)}</text></React.Fragment>; })}
      {elevationTicks_deg.map((tick_deg) => { const point = toFramePixel([bounds.left, tick_deg]); return <React.Fragment key={`el-${tick_deg}`}><line className={tick_deg === 0 ? 'zero-gridline' : 'gridline'} x1={workspaceTopLeft.x} x2={workspaceBottomRight.x} y1={point.y} y2={point.y} /><text className="elevation-tick-label" x={point.x - 5} y={point.y}>{degreeLabel(tick_deg)}</text></React.Fragment>; })}
      <rect className="workspace-boundary" x={workspaceTopLeft.x} y={workspaceTopLeft.y} width={workspaceBottomRight.x - workspaceTopLeft.x} height={workspaceBottomRight.y - workspaceTopLeft.y} />
    </svg>
    <span className="marker-label start-marker-label" style={{ left: `${startFramePercent.left}%`, top: `${startFramePercent.top}%` }}>Start t = {formatTime(startTime_s)} {formatPosition(startPosition_deg)}</span>
    <span className="marker-label goal-marker-label" style={{ left: `${endFramePercent.left}%`, top: `${endFramePercent.top}%` }}>Goal t = {formatTime(arrivalTime_s)} {formatPosition(endPosition_deg)}</span>
    <span className="obstacle-legend"><i className="protected-legend" />Protected / planner geometry<i className="original-legend" />Original polygon</span>
    <span className="azimuth-axis-label">Azimuth (deg)</span>
    <span className="elevation-axis-label">Elevation (deg)</span>
  </div>;
}

function Plane({ scene, result, time, mode, onPoint, onVertexMove, onCursorMove }) {
  const mount = useRef();
  const [frame, setFrame] = useState();
  useEffect(() => {
    const bounds = sceneBounds(scene);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    let currentViewport;
    const camera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.1, 100);
    camera.position.z = 10;
    const world = new THREE.Scene(); world.background = new THREE.Color('#101827');
    const drawLine = (points, color, closed = false, z = 0) => { if (!points?.length) return; const geometry = new THREE.BufferGeometry().setFromPoints((closed ? [...points, points[0]] : points).map(([x, y]) => new THREE.Vector3(x, y, z))); world.add(new THREE.Line(geometry, new THREE.LineBasicMaterial({ color }))); };
    const drawVertices = (points, color) => { if (!points?.length) return; const geometry = new THREE.BufferGeometry().setFromPoints(points.map(([x, y]) => new THREE.Vector3(x, y, 0.1))); world.add(new THREE.Points(geometry, new THREE.PointsMaterial({ color, size: 8, sizeAttenuation: false }))); };
    const drawPolygon = (vertices_deg, fillColor, fillOpacity, lineColor, z) => {
      for (const path of finitePolygonPaths(vertices_deg)) {
        const shape = new THREE.Shape(path.map(([x, y]) => new THREE.Vector2(x, y)));
        const fill = new THREE.Mesh(new THREE.ShapeGeometry(shape), new THREE.MeshBasicMaterial({ color: fillColor, transparent: true, opacity: fillOpacity, depthWrite: false }));
        fill.position.z = z;
        world.add(fill);
        drawLine(path, lineColor, true, z + 0.01);
      }
    };
    const drawArrowhead = (from_deg, to_deg, color) => {
      const direction = new THREE.Vector2(
        to_deg[0] - from_deg[0], to_deg[1] - from_deg[1],
      );
      if (direction.lengthSq() === 0) return;
      direction.normalize();
      const normal = new THREE.Vector2(-direction.y, direction.x);
      const size_deg = Math.min(
        bounds.right - bounds.left, bounds.top - bounds.bottom,
      ) * 0.018;
      const tip = new THREE.Vector2(to_deg[0], to_deg[1]);
      const base = tip.clone().addScaledVector(direction, -size_deg);
      const shape = new THREE.Shape([
        tip,
        base.clone().addScaledVector(normal, size_deg * 0.48),
        base.clone().addScaledVector(normal, -size_deg * 0.48),
      ]);
      const arrow = new THREE.Mesh(new THREE.ShapeGeometry(shape), new THREE.MeshBasicMaterial({ color }));
      arrow.position.z = 0.08;
      world.add(arrow);
    };
    const drawTrajectory = (position_deg) => {
      if (!position_deg?.length) return;
      drawLine(position_deg, 0x60a5fa, false, 0.06);
      const arrowCount = Math.min(4, Math.max(0, position_deg.length - 1));
      for (let arrowIndex = 1; arrowIndex <= arrowCount; arrowIndex += 1) {
        const endIndex = Math.round(arrowIndex * (position_deg.length - 1) / (arrowCount + 1));
        drawArrowhead(position_deg[Math.max(0, endIndex - 1)], position_deg[endIndex], 0x93c5fd);
      }
    };
    const activeSlices = scene.obstacles.map((obstacle) => { const index = Math.max(0, obstacle.slices.findLastIndex((_, sliceIndex) => (obstacle.time_s?.[sliceIndex] ?? 0) <= time)); const slice = obstacle.slices[index] || {}; const originalVertices_deg = slice.vertices_deg ?? []; const protectedVertices_deg = slice.protectedVertices_deg ?? originalVertices_deg; drawPolygon(protectedVertices_deg, 0xf97316, 0.26, 0xfb923c, 0); drawPolygon(originalVertices_deg, 0xef4444, 0.20, 0xfca5a5, 0.03); if (mode === 'edit') drawVertices(originalVertices_deg, 0xfbbf24); return index; });
    drawVertices([scene.initialState.position_deg], 0x4ade80); drawVertices([scene.goalState.position_deg], 0xf87171); drawTrajectory(result?.position_deg);
    mount.current.appendChild(renderer.domElement);
    const resize = (width_px, height_px) => {
      if (width_px <= 0 || height_px <= 0) return;
      currentViewport = createViewport(bounds, width_px, height_px);
      camera.left = currentViewport.left;
      camera.right = currentViewport.right;
      camera.top = currentViewport.top;
      camera.bottom = currentViewport.bottom;
      camera.updateProjectionMatrix();
      renderer.setSize(width_px, height_px, false);
      renderer.render(world, camera);
      setFrame({ viewport: currentViewport, width_px, height_px });
    };
    const resizeObserver = new ResizeObserver(([entry]) => {
      resize(entry.contentRect.width, entry.contentRect.height);
    });
    resizeObserver.observe(mount.current);
    resize(mount.current.clientWidth, mount.current.clientHeight);
    const toPosition = (event) => { const box = renderer.domElement.getBoundingClientRect(); const azimuth_deg = currentViewport.left + (event.clientX - box.left) / box.width * (currentViewport.right - currentViewport.left); const elevation_deg = currentViewport.top - (event.clientY - box.top) / box.height * (currentViewport.top - currentViewport.bottom); return [clamp(azimuth_deg, bounds.left, bounds.right), clamp(elevation_deg, bounds.bottom, bounds.top)]; };
    let drag = null;
    const findVertex = (event) => { const position = toPosition(event); const box = renderer.domElement.getBoundingClientRect(); const threshold_deg = 12 / box.width * (bounds.right - bounds.left); let nearest = null; scene.obstacles.forEach((obstacle, obstacleIndex) => obstacle.slices[activeSlices[obstacleIndex]]?.vertices_deg.forEach((vertex, vertexIndex) => { const distance = Math.hypot(vertex[0] - position[0], vertex[1] - position[1]); if (distance <= threshold_deg && (!nearest || distance < nearest.distance)) nearest = { obstacleIndex, vertexIndex, distance }; })); return nearest; };
    const pointerDown = (event) => { if (mode === 'edit') drag = findVertex(event); }; const pointerMove = (event) => { const position_deg = toPosition(event); onCursorMove(position_deg); if (drag) onVertexMove(drag, position_deg); }; const pointerLeave = () => onCursorMove(null); const pointerUp = (event) => { if (drag) { drag = null; return; } if (mode !== 'edit') onPoint(toPosition(event)); };
    renderer.domElement.addEventListener('pointerdown', pointerDown); renderer.domElement.addEventListener('pointermove', pointerMove); renderer.domElement.addEventListener('pointerup', pointerUp);
    renderer.domElement.addEventListener('pointerleave', pointerLeave);
    return () => { resizeObserver.disconnect(); renderer.domElement.removeEventListener('pointerdown', pointerDown); renderer.domElement.removeEventListener('pointermove', pointerMove); renderer.domElement.removeEventListener('pointerup', pointerUp); renderer.domElement.removeEventListener('pointerleave', pointerLeave); renderer.dispose(); mount.current?.replaceChildren(); };
  }, [scene, result, time, mode, onPoint, onVertexMove, onCursorMove]);
  const bounds = sceneBounds(scene);
  return <div className="plane"><div className="canvas-mount" ref={mount} /><CoordinateFrame bounds={bounds} frame={frame} initialPosition_deg={scene.initialState.position_deg} goalPosition_deg={scene.goalState.position_deg} initialTime_s={scene.initialState.time_s} goalTime_s={scene.goalState.time_s} result={result} /></div>;
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
  useEffect(() => { if (!job) return undefined; const timer = setInterval(async () => { const poll = await fetch(`/api/jobs/${job.id}`); const payload = await poll.json(); if (payload.Status !== 'completed') return; clearInterval(timer); if (job.type === 'import' && payload.Response.Scene) { setScene(payload.Response.Scene); setTime(payload.Response.Scene.initialState.time_s); } setResponse(payload.Response); requestAnimationFrame(() => setLatency({ clickToRender_s: (performance.now() - job.startedAt_ms) / 1000, planner_s: payload.Response.ElapsedWallTime_s })); }, 100); return () => clearInterval(timer); }, [job]);
  const cancel = () => fetch(`/api/jobs/${job.id}/cancel`, { method: 'POST' }); const trajectoryTime_s = response?.Result?.time_s; const duration = trajectoryTime_s?.length ? trajectoryTime_s[trajectoryTime_s.length - 1] : scene.goalState.time_s; const currentTime_s = clamp(time, scene.initialState.time_s, duration); const busy = job && !response; const transport_s = latency ? Math.max(0, latency.clickToRender_s - (latency.planner_s || 0)) : undefined;
  return <main><header><h1>Az/El Planner Sandbox</h1><span>{response?.Result?.TerminationReason || response?.Error?.Identifier || (busy ? 'planning' : 'ready')}</span></header><section className="workspace"><section><div className="position-readout"><span>Cursor: {cursorPosition_deg ? formatPosition(cursorPosition_deg) : 'outside workspace'}</span><span>Start: {formatPosition(scene.initialState.position_deg)}</span><span>Goal: {formatPosition(scene.goalState.position_deg)}</span></div><Plane scene={scene} result={response?.Result} time={currentTime_s} mode={mode} onPoint={point} onVertexMove={moveVertex} onCursorMove={updateCursor} /></section><aside><button onClick={() => setMode('polygon')}>Draw polygon</button><button onClick={finishPolygon}>Finish polygon ({draft.length})</button><button onClick={() => setMode('edit')}>Edit vertices</button><button onClick={() => setMode('start')}>Place start</button><button onClick={() => setMode('goal')}>Place goal</button><button disabled={busy} onClick={run}>Run planner</button><button disabled={!busy || job.type !== 'plan'} onClick={cancel}>Cancel</button><label>Load sandbox bundle <input aria-label="Load sandbox bundle" type="file" accept=".mat" disabled={busy} onChange={importBundle} /></label><button disabled={busy} onClick={exportBundle}>Save sandbox bundle</button><label>Mission time <input value={scene.goalState.time_s} type="number" onChange={(event) => setScene((current) => ({ ...current, goalState: { ...current.goalState, time_s: +event.target.value } }))} /></label><label>Velocity az/el <input value={scene.limits.maxVelocity_deg_s.join(',')} onChange={(event) => setScene((current) => ({ ...current, limits: { ...current.limits, maxVelocity_deg_s: event.target.value.split(',').map(Number) } }))} /></label>{latency && <output>Click to render: {latency.clickToRender_s.toFixed(3)} s; planner: {latency.planner_s.toFixed(3)} s; transport/UI: {transport_s.toFixed(3)} s</output>}{response?.DownloadUrl && <a href={response.DownloadUrl}>Download saved MATLAB bundle</a>}<pre>{JSON.stringify(response?.Diagnostics || response?.Error || { draftVertices: draft.length, obstacleCount: scene.obstacles.length, editMode: mode === 'edit' }, null, 2)}</pre></aside></section><label className="scrub-label" htmlFor="trajectory-time">Mission time: {formatTime(currentTime_s)}<input id="trajectory-time" aria-label="Trajectory time in seconds" className="scrub" type="range" min={scene.initialState.time_s} max={duration} step="0.01" value={currentTime_s} onChange={(event) => setTime(+event.target.value)} /></label></main>;
}
createRoot(document.getElementById('root')).render(<App />);
