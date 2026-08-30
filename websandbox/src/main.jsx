import React, { useEffect, useRef, useState } from 'react';
import { createRoot } from 'react-dom/client';
import * as THREE from 'three';
import './styles.css';

const initialScene = { obstacles: [], initialState: { time_s: 0, position_deg: [-8, -5], velocity_deg_s: [0, 0], acceleration_deg_s2: [0, 0] }, goalState: { time_s: 20, position_deg: [8, 5], velocity_deg_s: [0, 0], acceleration_deg_s2: [0, 0] }, limits: { maxVelocity_deg_s: [2, 2], maxAcceleration_deg_s2: [.75, .75], maxJerk_deg_s3: [2.5, 2.5], azimuthInterval_deg: [-20, 20], elevationInterval_deg: [-10, 10] }, plannerOptions: { GoalTimeMode: 'fixedArrival' } };

function Plane({ scene, result, time, onPoint }) {
  const mount = useRef();
  useEffect(() => {
    const renderer = new THREE.WebGLRenderer({ antialias: true }); const camera = new THREE.OrthographicCamera(-20, 20, 10, -10, 0.1, 100); camera.position.z = 10;
    const world = new THREE.Scene(); world.background = new THREE.Color('#101827');
    const draw = (points, color, closed = false) => { const geometry = new THREE.BufferGeometry().setFromPoints(points.map(([x, y]) => new THREE.Vector3(x, y, 0))); world.add(new THREE.Line(geometry, new THREE.LineBasicMaterial({ color }))); if (closed) world.children.at(-1).geometry.setFromPoints([...points, points[0]].map(([x, y]) => new THREE.Vector3(x, y, 0))); };
    scene.obstacles.forEach((obstacle) => { const slices = obstacle.slices; const index = Math.min(slices.length - 1, slices.findLastIndex((_, i) => (obstacle.time_s?.[i] ?? 0) <= time)); draw(slices[Math.max(0, index)].vertices_deg, 0xee7d72, true); });
    draw([scene.initialState.position_deg], 0x4ade80); draw([scene.goalState.position_deg], 0xf87171);
    if (result?.position_deg?.length) draw(result.position_deg, 0x60a5fa);
    renderer.setSize(mount.current.clientWidth, mount.current.clientHeight); mount.current.appendChild(renderer.domElement); renderer.render(world, camera);
    const click = (event) => { const box = renderer.domElement.getBoundingClientRect(); onPoint([(event.clientX - box.left) / box.width * 40 - 20, 10 - (event.clientY - box.top) / box.height * 20]); }; renderer.domElement.addEventListener('click', click);
    return () => { renderer.domElement.removeEventListener('click', click); renderer.dispose(); mount.current?.replaceChildren(); };
  }, [scene, result, time, onPoint]);
  return <div className="plane" ref={mount} />;
}

function App() {
  const [scene, setScene] = useState(initialScene), [mode, setMode] = useState('polygon'), [draft, setDraft] = useState([]), [jobId, setJobId] = useState(), [response, setResponse] = useState(), [time, setTime] = useState(0);
  const point = (position_deg) => { if (mode === 'start') setScene({ ...scene, initialState: { ...scene.initialState, position_deg } }); else if (mode === 'goal') setScene({ ...scene, goalState: { ...scene.goalState, position_deg } }); else setDraft([...draft, position_deg]); };
  const finishPolygon = () => { if (draft.length >= 3) setScene({ ...scene, obstacles: [...scene.obstacles, { name: `polygon ${scene.obstacles.length + 1}`, time_s: [0, scene.goalState.time_s], safetyMargin_deg: 0, slices: [{ vertices_deg: draft }, { vertices_deg: draft }] }] }); setDraft([]); };
  const run = async () => { const r = await fetch('/api/plan', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ ...scene, mode: 'trajectory' }) }); const j = await r.json(); setJobId(j.JobId); };
  useEffect(() => { if (!jobId) return; const timer = setInterval(async () => { const j = await (await fetch(`/api/jobs/${jobId}`)).json(); if (j.Status === 'completed') { setResponse(j.Response); clearInterval(timer); } }, 250); return () => clearInterval(timer); }, [jobId]);
  const cancel = () => fetch(`/api/jobs/${jobId}/cancel`, { method: 'POST' });
  const duration = response?.Result?.TrajectoryDuration_s || scene.goalState.time_s;
  return <main><header><h1>Az/El Planner Sandbox</h1><span>{response?.Result?.TerminationReason || (jobId ? 'planning' : 'ready')}</span></header><section className="workspace"><Plane scene={scene} result={response?.Result} time={time} onPoint={point} /><aside><button onClick={() => setMode('polygon')}>Draw polygon</button><button onClick={finishPolygon}>Finish polygon</button><button onClick={() => setMode('start')}>Place start</button><button onClick={() => setMode('goal')}>Place goal</button><button onClick={run}>Run planner</button><button disabled={!jobId || response} onClick={cancel}>Cancel</button><label>Mission time <input value={scene.goalState.time_s} type="number" onChange={e => setScene({ ...scene, goalState: { ...scene.goalState, time_s: +e.target.value } })} /></label><label>Velocity az/el <input value={scene.limits.maxVelocity_deg_s.join(',')} onChange={e => setScene({ ...scene, limits: { ...scene.limits, maxVelocity_deg_s: e.target.value.split(',').map(Number) } })} /></label><pre>{JSON.stringify(response?.Diagnostics || { draftVertices: draft.length }, null, 2)}</pre></aside></section><input className="scrub" type="range" min="0" max={duration} step="0.01" value={time} onChange={e => setTime(+e.target.value)} /></main>;
}
createRoot(document.getElementById('root')).render(<App />);
