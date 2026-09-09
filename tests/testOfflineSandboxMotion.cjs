// Run with: node --test tests/testOfflineSandboxMotion.cjs
// Exercise the standalone page's actual functions without a browser or MATLAB.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const { test } = require('node:test');
const html = fs.readFileSync(path.join(__dirname,
  '../offlinesandbox/xy_planner_sandbox.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];

function harness() {
  const inputs = new Map();
  const context = vm.createContext({
    state: { obstacles: [], selectedObstacleIndex: 0, currentTime_s: 0,
      nextObstacleNumber: 1, mode: 'motion', isPlanning: false,
      importedSceneIsReadOnly: false, responseObstacles: [], obstaclePreview: false,
      plotRect: { width: 360 } },
    defaultControls: { missionTime: 100 }, MAXIMUM_OBSTACLE_SPEED_UNITS_S: 10,
    HANDLE_RADIUS_PX: 10, workspace: () => ({ x: [-180, 180], y: [-90, 90] }),
    worldToScreen: (point) => point,
    element: (id) => {
      if (!inputs.has(id)) inputs.set(id, { value: '', textContent: '', setAttribute() {} });
      return inputs.get(id);
    },
    finiteNumber: Number.isFinite,
    readControls: () => ({ defaultMargin: 0.2 }),
    setStatus: () => {}, invalidateResult: () => {},
    renderObstacleEditor: () => {}, updateActionAvailability: () => {},
    draw: () => {}, updateMotionInputs: () => {},
    canvasHint: { textContent: '' }, document: { querySelectorAll: () => [] },
    playButton: { textContent: '', setAttribute() {} }, timeSlider: {}, timeReadout: {},
    performance: { now: () => 0 }, requestAnimationFrame: () => 1, cancelAnimationFrame: () => {},
    stopPlayback: () => {}, syncTimelineControl: () => {},
    activateMode: (mode) => { context.state.mode = mode; },
    strokePath: () => {}, drawPoint: () => {}
  });
  const names = ['centroid', 'numberFromInput', 'motionMissionTime',
    'motionOffset_units', 'obstaclePoseAtTime', 'obstacleMotionIntervals',
    'buildObstacleKeyframes', 'interpolateRows', 'obstacleVerticesAtCurrentTime',
    'setFinalPoseCenter', 'applyFinalPoseControls', 'syncFinalPoseControls',
    'addObstacle', 'deleteSelectedObstacle', 'drawFinalPoseGhost',
    'activateMotionDrawing', 'finalPoseCorners', 'finalPoseRotationHandle',
    'finalPoseTargetAt', 'screenDistance', 'drawObstacleMotionGuides', 'splitRegions', 'asArray',
    'updateActiveDrag', 'pointsClose', 'pointInPolygon', 'selectObstacleAt',
    'clearObstaclePreview', 'previewObstacleMotion', 'finishMotionEditing',
    'stopPlayback', 'startPlayback', 'advancePlayback', 'timelineBounds',
    'configureTimeline', 'syncTimelineControl', 'updateObstacleSpeed',
    'updateMotionInputs', 'updateMotionSpeedReadout', 'updateSpeedOverlay',
    'rectangleVertices', 'finishActiveDrag', 'polygonArea', 'clampToWorkspace',
    'originalTransformGeometry', 'originalTransformTargetAt', 'beginSceneDrag',
    'pointInsideWorkspace', 'markDragChanged', 'wrapPoint', 'nearestWrappedGoal', 'wrappedSegments'];
  const functions = [...script.matchAll(/^      function (\w+)\(/gm)];
  for (const name of names) {
    const index = functions.findIndex((entry) => entry[1] === name);
    assert.ok(index >= 0, name);
    vm.runInContext(script.slice(functions[index].index,
      functions[index + 1].index), context);
  }
  context.element('missionTime').value = '100';
  return context;
}

const triangle = [[0, 0], [6, 0], [0, 3]];

test('independent wrapping uses each shifted workspace period', () => {
  const c = harness();
  for (let mask = 0; mask < 4; mask += 1) {
    c.readControls = () => ({ xMin: -5, xMax: 5, yMin: 100, yMax: 120,
      wrapX: Boolean(mask & 1), wrapY: Boolean(mask & 2) });
    near(c.nearestWrappedGoal([4, 119], [-4, 101]),
      [-4 + 10 * Boolean(mask & 1), 101 + 20 * Boolean(mask & 2)]);
    near(c.wrapPoint([6, 121]),
      [6 - 10 * Boolean(mask & 1), 121 - 20 * Boolean(mask & 2)]);
  }
});

test('periodic paths split simultaneous seams and preserve the disabled axis', () => {
  const c = harness();
  c.readControls = () => ({ xMin: -5, xMax: 5, yMin: 100, yMax: 120,
    wrapX: true, wrapY: true });
  near(c.wrappedSegments([[4, 118], [6, 122]]),
    [[[4, 118], [5, 120]], [[-5, 100], [-4, 102]]]);
  near(c.nearestWrappedGoal([0, 110], [-5, 100]), [5, 120]);
  near(c.nearestWrappedGoal([0, 110], [5, 120]), [5, 120]);
  c.readControls = () => ({ xMin: -5, xMax: 5, yMin: 100, yMax: 120,
    wrapX: false, wrapY: true });
  near(c.wrappedSegments([[4, 118], [6, 122]]),
    [[[4, 118], [5, 120]], [[5, 100], [6, 102]]]);
});

test('result display reads separate diagnosis and the compact route', () => {
  const context = harness();
  const functions = [...script.matchAll(/      function (\w+)\(/g)];
  for (const name of ['renderDiagnostics', 'drawSelectedSeed']) {
    const index = functions.findIndex((entry) => entry[1] === name);
    assert.ok(index >= 0);
    vm.runInContext(script.slice(functions[index].index,
      functions[index + 1].index), context);
  }
  const validation = { Passed: true, InterSegmentContinuous: true };
  const search = { NodeCount: 12, AcceptedEdgeCount: 8, ExpandedCount: 7, RejectedTransitionCount: 3 };
  context.state.result = { Route_units: [[0, 0], [3, 2]] };
  context.state.diagnosis = { Search: search };
  context.state.validation = validation;
  context.renderCoreEvidence = (value) => assert.equal(value, context.state.diagnosis);
  context.renderValidationChecks = (value) => assert.equal(value, validation);
  context.updateOverlayControls = (value) => assert.equal(value, search);
  context.renderDiagnostics();
  assert.equal(context.element('attemptedSeedCount').textContent, '12');
  assert.equal(context.element('expandedStateCount').textContent, '7');
  context.state.showSelectedSeed = true;
  context.matrix = (value) => value;
  context.wrappedSegments = (value) => [value];
  context.walkthroughViewIs = () => false;
  let drawn;
  context.strokePath = (value) => { drawn = value; };
  context.drawSelectedSeed();
  assert.deepEqual(drawn, context.state.result.Route_units);
});

const concave = [[-3, -2], [4, -2], [4, 0], [0, 0], [0, 5], [-3, 5]];
function obstacle(vertices = triangle, angle = 0, profile = 'stationary') {
  return { name: 'Test polygon', vertices_units: vertices.map((p) => p.slice()),
    finalRotation_deg: angle, motionProfile: profile,
    motionVelocity_units_s: [0, 0], safetyMargin_units: 0.2 };
}
function near(actual, expected, tolerance = 1e-10) {
  if (Array.isArray(expected)) {
    assert.equal(actual.length, expected.length);
    actual.forEach((value, index) => near(value, expected[index], tolerance));
  } else assert.ok(Math.abs(actual - expected) <= tolerance,
    `${actual} differs from ${expected}`);
}

test('page syntax, unique IDs, and contextual Delete wiring', () => {
  new vm.Script(script);
  const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map((m) => m[1]);
  assert.equal(new Set(ids).size, ids.length);
  assert.match(html, /id="obstacleActionMenu"[\s\S]*id="deletePolygonButton"[\s\S]*id="coordinateReadout"/);
  assert.match(script, /element\("deletePolygonButton"\)\.addEventListener\("click", deleteSelectedObstacle\)/);
});

test('bundle Save As confirms the written path and distinguishes cancel and failure', async () => {
  const start = script.indexOf('      async function downloadLiveBundle()');
  const end = script.indexOf('      function asArray(', start);
  for (const outcome of [
    { Saved: true, FilePath: 'C:\\chosen\\scene.mat' },
    { Saved: false, Cancelled: true },
    { Saved: false },
    new Error('Save failed')
  ]) {
    const messages = [];
    const c = vm.createContext({
      state: { connectionMode: 'live', isPlanning: false, bundleRequestId: 'current-plan',
        serverOrigin: 'http://127.0.0.1:52731' },
      updatePlanningControls() {}, setStatus: (...args) => messages.push(args),
      requestJson: async (url, request) => {
        assert.equal(url, 'http://127.0.0.1:52731/save-bundle');
        assert.equal(JSON.parse(request.body).requestId, 'current-plan');
        if (outcome instanceof Error) throw outcome;
        return outcome;
      }
    });
    vm.runInContext(script.slice(start, end), c);
    await c.downloadLiveBundle();
    assert.equal(c.state.isPlanning, false);
    assert.match(messages[0][0], /Save As dialog/);
    const [message, tone] = messages.at(-1);
    if (outcome.Saved) {
      assert.equal(tone, 'success');
      assert.match(message, /C:\\chosen\\scene\.mat/);
    } else if (outcome.Cancelled) {
      assert.equal(tone, 'info');
      assert.match(message, /cancelled/);
    } else assert.equal(tone, 'error');
  }
});

test('deep walkthrough uses core C3 checks without inventing candidate results', () => {
  const stages = script.slice(script.indexOf('      const WALKTHROUGH_STAGES'),
    script.indexOf('      const FREEHAND_MAXIMUM_VERTICES'));
  const context = harness();
  vm.runInContext(stages, context);
  context.state.result = { Success: false, Message: 'No route', TerminationReason: 'noVisibilityRoute' };
  context.state.diagnosis = { Search: { NodeCount: 3, RejectedEdges_units: [[0,0,1,1]] } };
  context.state.validation = { Passed: false };
  context.displayNumber = (value) => Number.isFinite(value) ? String(value) : '—';
  context.matrix = (value) => value || [];
  const functions = [...script.matchAll(/^      function (\w+)\(/gm)];
  for (const name of ['deepDiveStage', 'checkDescription', 'checkDecision', 'buildDeepDiveStages']) {
    const index = functions.findIndex(entry => entry[1] === name);
    vm.runInContext(script.slice(functions[index].index, functions[index+1].index), context);
  }
  const result = context.buildDeepDiveStages();
  const continuity = result.find(stage => stage.title === 'C3 continuity including jerk');
  assert.equal(continuity.decision, 'Inspect');
  assert.match(continuity.description, /not available/);
  assert.ok(!result.some(stage => /candidate pruning|motion seeds/i.test(stage.title)));
  assert.equal(result.at(-1).decision, 'Pruned');
});

test('rotation-only motion preserves initial and final geometry for different shapes', () => {
  const c = harness();
  for (const shape of [triangle, concave]) {
    for (const angle of [-90, 90, 360]) {
      const o = obstacle(shape, angle);
      const frames = c.buildObstacleKeyframes(o, 100);
      assert.ok(frames.length > 2);
      near(frames[0].vertices_units, shape);
      const center = c.centroid(shape);
      const final = frames.at(-1).vertices_units;
      near(c.centroid(final), center);
      shape.forEach((p, i) => {
        const x = p[0] - center[0], y = p[1] - center[1];
        near(final[i], angle === 90 ? [center[0] - y, center[1] + x] :
          angle === -90 ? [center[0] + y, center[1] - x] : p);
      });
      assert.ok(frames.every((f, i) => i === 0 || f.time_s > frames[i - 1].time_s));
    }
  }
});

test('combined translation and rotation reaches the requested pose; excessive speed is refused', () => {
  const c = harness();
  const o = obstacle(triangle, 90);
  assert.equal(c.setFinalPoseCenter(o, [42, -19]), true);
  near(c.centroid(c.buildObstacleKeyframes(o, 100).at(-1).vertices_units), [42, -19]);
  near(o.motionVelocity_units_s, [0.4, -0.2]);
  const saved = JSON.stringify(o);
  assert.equal(c.setFinalPoseCenter(o, [9999, 9999]), false);
  assert.equal(c.setFinalPoseCenter(o, [NaN, 0]), false);
  assert.equal(JSON.stringify(o), saved);
});

test('preview matches exported vertex interpolation for every translation profile', () => {
  const c = harness();
  for (const profile of ['stationary', 'nonzeroVelocity', 'zeroStart', 'trapezoidal', 'oscillating']) {
    const o = obstacle(concave, -135, profile);
    o.motionVelocity_units_s = [0.35, -0.1];
    const frames = c.buildObstacleKeyframes(o, 100);
    for (const time of [0, 0.17, 12.3, 25, 51.2, 99.9, 100]) {
      c.state.currentTime_s = time;
      const index = Math.min(frames.length - 2,
        frames.findIndex((f, i) => i < frames.length - 1 && time <= frames[i + 1].time_s));
      const first = frames[index], next = frames[index + 1];
      const fraction = (time - first.time_s) / (next.time_s - first.time_s);
      const expected = first.vertices_units.map((p, i) => p.map((v, axis) =>
        v + fraction * (next.vertices_units[i][axis] - v)));
      near(c.obstacleVerticesAtCurrentTime(o), expected);
    }
  }
});

test('angle editing preserves translation profile, rejects invalid input, and honors edit locks', () => {
  const c = harness(); const o = obstacle(triangle, 0, 'oscillating');
  o.motionVelocity_units_s = [0.4, 0.2]; c.state.obstacles = [o];
  c.syncFinalPoseControls();
  c.element('finalPoseRotation').value = '90';
  c.applyFinalPoseControls({ target: { id: 'finalPoseRotation' } });
  assert.equal(o.finalRotation_deg, 90);
  assert.equal(o.motionProfile, 'oscillating');
  for (const value of ['361', '-361', 'NaN', '']) {
    c.element('finalPoseRotation').value = value;
    c.applyFinalPoseControls({ target: { id: 'finalPoseRotation' } });
    assert.equal(o.finalRotation_deg, 90);
  }
  for (const lock of ['isPlanning', 'importedSceneIsReadOnly']) {
    c.state[lock] = true;
    c.element('finalPoseRotation').value = '45';
    c.applyFinalPoseControls({ target: { id: 'finalPoseRotation' } });
    assert.equal(o.finalRotation_deg, 90);
    c.deleteSelectedObstacle(); assert.equal(c.state.obstacles.length, 1);
    c.state[lock] = false;
  }
});

test('ghost drag keeps grab offset and original geometry; copying preserves the motion', () => {
  const c = harness(); const o = obstacle(triangle, 45); c.state.obstacles = [o];
  c.state.activeDrag = { type: 'finalPose', obstacleIndex: 0, grabOffset: [-1, 2] };
  c.updateActiveDrag([31, 18]);
  near(c.centroid(c.buildObstacleKeyframes(o, 100).at(-1).vertices_units), [30, 20]);
  near(o.vertices_units, triangle);
  c.addObstacle(o.vertices_units, o);
  const copy = c.state.obstacles[1];
  assert.equal(copy.finalRotation_deg, 45);
  near(copy.motionVelocity_units_s, o.motionVelocity_units_s);
  copy.vertices_units[0][0] = -10; near(o.vertices_units, triangle);
  c.deleteSelectedObstacle();
  assert.equal(c.state.obstacles.length, 1);
  assert.equal(c.state.selectedObstacleIndex, -1);
  assert.equal(c.state.mode, 'select');
});

test('ghost shows mission-end pose and Set motion rewinds playback', () => {
  const c = harness(); const o = obstacle(concave, 90, 'nonzeroVelocity');
  o.motionVelocity_units_s = [0.1, 0.2]; c.state.obstacles = [o];
  c.state.currentTime_s = 33;
  let outline;
  c.strokePath = (vertices, style) => { if (style.close && style.width === 2) outline = vertices; };
  c.drawFinalPoseGhost();
  near(outline, c.buildObstacleKeyframes(o, 100).at(-1).vertices_units);
  c.activateMotionDrawing();
  assert.equal(c.state.currentTime_s, 0);
  assert.equal(c.state.mode, 'motion');
});

test('stretching a rotated final pose preserves its center and leaves the initial polygon intact', () => {
  const c = harness(); const o = obstacle(concave, 90); c.state.obstacles = [o];
  const center = c.centroid(o.vertices_units);
  const corner = c.finalPoseCorners(o)[2];
  c.state.activeDrag = { type: 'finalStretch', obstacleIndex: 0, center, local: corner.local };
  // At +90 coordinate units, scaled local coordinates (x,y) map to world (-y,x).
  c.updateActiveDrag([center[0] - corner.local[1] * 0.5,
    center[1] + corner.local[0] * 2]);
  near(o.finalScale, [2, 0.5]);
  near(o.vertices_units, concave);
  const final = c.buildObstacleKeyframes(o, 100).at(-1).vertices_units;
  near(c.centroid(final), center);
  final.forEach((point, index) => near(point, [
    center[0] - (concave[index][1] - center[1]) * 0.5,
    center[1] + (concave[index][0] - center[0]) * 2]));
  c.addObstacle(o.vertices_units, o);
  near(c.state.obstacles[1].finalScale, [2, 0.5]);
  c.state.obstacles[1].finalScale[0] = 3;
  near(o.finalScale, [2, 0.5]);
});

test('rotation drag crosses the angle wrap smoothly and enforces the full-turn bound', () => {
  const c = harness(); const o = obstacle(triangle, 170); c.state.obstacles = [o];
  const center = c.centroid(o.vertices_units);
  c.state.activeDrag = { type: 'finalRotate', obstacleIndex: 0, center,
    lastAngle: 170 * Math.PI / 180, turn_deg: 170 };
  for (const angle_deg of [-170, -90, 0]) {
    const angle = angle_deg * Math.PI / 180;
    c.updateActiveDrag([center[0] + 10 * Math.cos(angle), center[1] + 10 * Math.sin(angle)]);
  }
  near(o.finalRotation_deg, 360);
  c.updateActiveDrag([center[0], center[1] + 10]);
  near(o.finalRotation_deg, 360);
});

test('stretch plus rotation playback uses the exported keyframes and rejects degenerate scale', () => {
  const c = harness(); const o = obstacle(triangle, -90, 'nonzeroVelocity');
  o.finalScale = [1.8, 0.4]; o.motionVelocity_units_s = [0.2, 0.1];
  c.state.obstacles = [o];
  const frames = c.buildObstacleKeyframes(o, 100);
  c.state.currentTime_s = 12.5;
  near(c.obstacleVerticesAtCurrentTime(o),
    frames[2].vertices_units.map((p, i) => p.map((v, axis) =>
      (v + frames[3].vertices_units[i][axis]) / 2)));
  c.syncFinalPoseControls();
  for (const value of ['0', '-1', 'NaN', '']) {
    c.element('finalPoseScaleX').value = value;
    c.element('finalPoseRotation').value = '-90';
    c.applyFinalPoseControls({ target: { id: 'finalPoseScaleX' } });
    near(o.finalScale, [1.8, 0.4]);
  }
});

test('click selection follows the translated and rotated polygon after playback', () => {
  const c = harness(); const o = obstacle(triangle, 90, 'nonzeroVelocity');
  o.motionVelocity_units_s = [0.5, 0.2]; c.state.obstacles = [o];
  c.state.currentTime_s = 100;
  c.selectObstacleAt([52, 21]);
  assert.equal(c.state.selectedObstacleIndex, 0);
  c.selectObstacleAt([2, 1]);
  assert.equal(c.state.selectedObstacleIndex, -1);
});

test('ghost retains only preview/confirm icons, with no transform buttons or top bar', () => {
  const menu = html.match(/id="finalPoseActionMenu"[\s\S]*?<\/div>/)[0];
  const buttons = [...menu.matchAll(/<button[\s\S]*?<\/button>/g)];
  assert.equal(buttons.length, 2);
  for (const [button] of buttons) {
    assert.match(button, /aria-label=/);
    assert.match(button, /title=/);
    assert.match(button, /<svg/);
  }
  assert.match(menu, /id="finishMotionButton"/);
  assert.doesNotMatch(html, /data-pose-tool|finalPoseTool/);
  assert.ok(html.indexOf('id="motionPoseControls"') > html.indexOf('id="obstacleEditor"'));
  assert.doesNotMatch(html, /class="motion-pose-controls"/);
});

test('browser preview runs without endpoints, a result, or MATLAB and returns to ghost editing', () => {
  const c = harness(); const o = obstacle(triangle, 90, 'nonzeroVelocity');
  o.motionVelocity_units_s = [0.2, 0.1]; o.finalScale = [2, 1];
  c.state.obstacles = [o];
  c.state.start = null; c.state.goal = null; c.state.result = null;
  c.state.connectionMode = 'offline';
  c.fetch = () => { throw Error('Preview must not use a network request'); };
  c.previewObstacleMotion();
  assert.equal(c.state.playing, true); assert.equal(c.state.obstaclePreview, true);
  assert.equal(c.state.mode, 'select'); near(c.timelineBounds(), [0, 100]);
  assert.equal(c.element('motionKinematicsPanel').hidden, true);
  c.advancePlayback(50000);
  near(c.state.currentTime_s, 50);
  near(c.obstacleVerticesAtCurrentTime(o), c.buildObstacleKeyframes(o, 100)[10].vertices_units);
  c.advancePlayback(150000);
  near(c.state.currentTime_s, 100); assert.equal(c.state.playing, false);
  c.previewObstacleMotion();
  assert.equal(c.state.obstaclePreview, false); assert.equal(c.state.mode, 'motion');
  near(c.state.currentTime_s, 0);
  assert.equal(c.element('motionKinematicsPanel').hidden, false);
  assert.equal(c.state.result, null);
});

test('preview preserves an existing planner result and the checkmark finishes editing', () => {
  const c = harness(); c.state.obstacles = [obstacle()];
  const result = { Success: true, Inputs: { initialState: { time_s: 5 }, goalState: { time_s: 50 } } };
  c.state.result = result;
  c.previewObstacleMotion(); near(c.timelineBounds(), [0, 100]);
  c.previewObstacleMotion(); assert.equal(c.state.result, result);
  c.finishMotionEditing(); assert.equal(c.state.mode, 'select');
  c.state.mode = 'motion'; c.state.isPlanning = true;
  c.finishMotionEditing(); assert.equal(c.state.mode, 'motion');
});

test('corners, rotation handle, and ghost body are directly draggable without a selected tool', () => {
  const c = harness(); const o = obstacle([[0, 0], [60, 0], [0, 30]], 45);
  for (const corner of c.finalPoseCorners(o)) {
    assert.equal(c.finalPoseTargetAt(o, corner.point, corner.point).type, 'finalStretch');
  }
  const handle = c.finalPoseRotationHandle(o);
  assert.equal(c.finalPoseTargetAt(o, handle, handle).type, 'finalRotate');
  const center = c.centroid(c.obstaclePoseAtTime(o, 100, 100));
  assert.equal(c.finalPoseTargetAt(o, center, center).type, 'finalPose');
  assert.equal(c.finalPoseTargetAt(o, [1000, 1000], [1000, 1000]), null);
});

test('preview removes traveled path segments and retains the final ghost', () => {
  const c = harness(); const o = obstacle(concave, 90, 'oscillating');
  o.finalScale = [1.5, 0.5]; o.motionVelocity_units_s = [0.4, 0.2];
  c.state.obstacles = [o];
  const frames = c.buildObstacleKeyframes(o, 100);
  for (const time of [0, 12.5, 25, 100, 10]) {
    const paths = []; c.state.currentTime_s = time;
    c.strokePath = (points, style) => paths.push({ points, style });
    c.drawObstacleMotionGuides();
    const line = paths.find((p) => !p.style.close);
    if (time === 100) assert.equal(line, undefined);
    else {
      near(line.points[0], c.centroid(c.obstacleVerticesAtCurrentTime(o)));
      near(line.points.slice(1), frames.filter((f) => f.time_s > time).map((f) => c.centroid(f.vertices_units)));
    }
    near(paths.find((p) => p.style.close).points, frames.at(-1).vertices_units);
  }
  c.state.obstacles = [];
  c.state.responseObstacles = [{ time_s: frames.map((f) => f.time_s),
    OriginalVerticesByTime_units: frames.map((f) => f.vertices_units) }];
  const paths = []; c.strokePath = (points, style) => paths.push({ points, style });
  c.drawObstacleMotionGuides();
  near(paths.find((p) => p.style.close).points, frames.at(-1).vertices_units);
  near(paths.find((p) => !p.style.close).points[0], c.centroid(frames[2].vertices_units));
});

test('speed slider preserves direction and motion profile, including across zero', () => {
  const c = harness(); const o = obstacle(triangle, 90, 'trapezoidal');
  o.motionVelocity_units_s = [-0.6, 0.8]; c.state.obstacles = [o];
  for (const speed of [10, 0, 0.5]) {
    c.element('obstacleSpeedSlider').value = String(speed);
    c.updateObstacleSpeed();
    near(o.motionVelocity_units_s, [-0.6 * speed, 0.8 * speed]);
    assert.equal(o.motionProfile, 'trapezoidal');
    assert.equal(o.finalRotation_deg, 90);
  }
  c.addObstacle(o.vertices_units, o);
  near(c.state.obstacles[1].motionDirection, [-0.6, 0.8]);
});

test('slider starts new motion predictably and honors bounds and editing locks', () => {
  const c = harness(); const o = obstacle(); c.state.obstacles = [o];
  c.element('obstacleSpeedSlider').value = '1'; c.updateObstacleSpeed();
  near(o.motionVelocity_units_s, [1, 0]); assert.equal(o.motionProfile, 'nonzeroVelocity');
  for (const speed of ['10.01', '-1', 'NaN']) {
    c.element('obstacleSpeedSlider').value = speed; c.updateObstacleSpeed();
    near(o.motionVelocity_units_s, [1, 0]);
  }
  for (const lock of ['isPlanning', 'importedSceneIsReadOnly']) {
    c.state[lock] = true; c.element('obstacleSpeedSlider').value = '2'; c.updateObstacleSpeed();
    near(o.motionVelocity_units_s, [1, 0]); c.state[lock] = false;
  }
  assert.match(html, /id="obstacleSpeedSlider" type="range" min="0" max="10"/);
});

test('rectangle drags normalize every direction, clamp to workspace, and reject zero area', () => {
  const c = harness();
  for (const end of [[20, 10], [-20, 10], [20, -10], [-20, -10]]) {
    c.state.activeDrag = { type: 'rectangle', startPoint: [0, 0] };
    c.updateActiveDrag(end);
    assert.equal(c.state.rectanglePreview.length, 4);
    near(c.polygonArea(c.state.rectanglePreview), 200);
    c.finishActiveDrag();
    assert.equal(c.state.rectanglePreview, null);
  }
  assert.equal(c.state.obstacles.length, 4);
  c.state.activeDrag = { type: 'rectangle', startPoint: [0, 0] };
  c.updateActiveDrag([999, 999]);
  near(c.state.rectanglePreview, [[0, 0], [180, 0], [180, 90], [0, 90]]);
  c.state.activeDrag = { type: 'rectangle', startPoint: [0, 0] };
  c.updateActiveDrag([0, 20]); c.finishActiveDrag();
  assert.equal(c.state.obstacles.length, 4);
});

test('speed slider is a selected-obstacle canvas overlay with editing locks', () => {
  const c = harness(); c.state.obstacles = [obstacle()];
  c.updateSpeedOverlay(); assert.equal(c.element('obstacleSpeedOverlay').hidden, false);
  c.state.mode = 'rectangle'; c.updateSpeedOverlay();
  assert.equal(c.element('obstacleSpeedOverlay').hidden, true);
  c.state.mode = 'select'; c.state.isPlanning = true; c.updateSpeedOverlay();
  assert.equal(c.element('obstacleSpeedSlider').disabled, true);
  c.state.importedSceneIsReadOnly = true; c.updateSpeedOverlay();
  assert.equal(c.element('obstacleSpeedOverlay').hidden, true);
  assert.ok(html.indexOf('id="obstacleSpeedSlider"') > html.indexOf('id="canvasWrap"'));
  assert.ok(html.indexOf('id="obstacleSpeedSlider"') < html.indexOf('id="coordinateReadout"'));
  assert.match(html, /data-mode="rectangle"/);
});

test('new obstacles expose original resize and rotation handles immediately', () => {
  const c = harness(); c.addObstacle([[-20, -10], [20, -10], [20, 10], [-20, 10]]);
  assert.equal(c.state.selectedObstacleIndex, 0);
  const o = c.state.obstacles[0], handles = c.originalTransformGeometry(o);
  for (const corner of handles.corners) {
    assert.equal(c.originalTransformTargetAt(o, corner.point).type, 'originalStretch');
  }
  assert.equal(c.originalTransformTargetAt(o, handles.rotation).type, 'originalRotate');
  c.beginSceneDrag({ type: 'originalStretch', obstacleIndex: 0, local: [20, 10] }, [20, 10], 1);
  c.updateActiveDrag([40, 5]);
  near(o.vertices_units, [[-40, -5], [40, -5], [40, 5], [-40, 5]]);
  const saved = JSON.stringify(o.vertices_units);
  c.updateActiveDrag([1000, 1000]); assert.equal(JSON.stringify(o.vertices_units), saved);
});

test('original rotation preserves size and updates geometry about its center', () => {
  const c = harness(); c.addObstacle(triangle);
  const o = c.state.obstacles[0], center = c.centroid(triangle);
  c.beginSceneDrag({ type: 'originalRotate', obstacleIndex: 0 }, [center[0] + 20, center[1]], 1);
  c.updateActiveDrag([center[0], center[1] + 20]);
  near(o.vertices_units, triangle.map((p) => [center[0] - (p[1] - center[1]),
    center[1] + p[0] - center[0]]));
  near(c.polygonArea(o.vertices_units), c.polygonArea(triangle));
});

test('arrival controls offer only earliest and fixed timing', () => {
  const select = html.match(/<select id="goalTimeMode">([\s\S]*?)<\/select>/)[1];
  assert.equal((select.match(/<option /g) || []).length, 2);
  assert.match(select, /value="earliestArrival" selected/);
  assert.doesNotMatch(html, /minimumTravelSavingsRate|balancedArrival/);
  new vm.Script(script);
});

test('walkthrough keeps accepted edges visible and confines tree scrolling', () => {
  assert.match(script, /walkthroughHasReachedView\("search"\)/);
  assert.match(script, /tree\.scrollTop/);
  assert.doesNotMatch(script, /scrollIntoView/);
});
