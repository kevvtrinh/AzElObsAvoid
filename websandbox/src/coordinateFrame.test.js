import assert from 'node:assert/strict';
import test from 'node:test';
import {
  coordinateToFramePixel,
  coordinateToFramePercent,
  createTicks,
  createViewport,
} from './coordinateFrame.js';

const defaultBounds = {
  left: -180,
  right: 180,
  bottom: -90,
  top: 90,
};

test('default azimuth ticks cover the full workspace every 45 degrees', () => {
  assert.deepEqual(createTicks(defaultBounds.left, defaultBounds.right), [
    -180, -135, -90, -45, 0, 45, 90, 135, 180,
  ]);
});

test('default azimuth ticks map across the measured plot width', () => {
  const width_px = 919;
  const height_px = 502;
  const viewport = createViewport(defaultBounds, width_px, height_px);
  const tickPixels = [-180, 0, 180].map((azimuth_deg) => (
    coordinateToFramePixel(
      [azimuth_deg, defaultBounds.bottom], viewport, width_px, height_px,
    ).x
  ));

  assert.ok(Math.abs(tickPixels[0] - 34.037037037037045) < 1e-12);
  assert.equal(tickPixels[1], width_px / 2);
  assert.ok(Math.abs(tickPixels[2] - 884.9629629629629) < 1e-12);
});

test('default elevation ticks cover the full workspace', () => {
  assert.deepEqual(createTicks(defaultBounds.bottom, defaultBounds.top), [
    -90, -67.5, -45, -22.5, 0, 22.5, 45, 67.5, 90,
  ]);
});

test('known coordinate uses the same projection as ticks and gridlines', () => {
  const viewport = createViewport(defaultBounds, 1000, 500);
  const knownPoint = coordinateToFramePercent([90, 45], viewport);
  const azimuthTick = coordinateToFramePercent([90, 0], viewport);
  const elevationTick = coordinateToFramePercent([0, 45], viewport);

  assert.ok(Math.abs(viewport.left + 201.6) < 1e-12);
  assert.ok(Math.abs(viewport.right - 201.6) < 1e-12);
  assert.ok(Math.abs(viewport.bottom + 100.8) < 1e-12);
  assert.ok(Math.abs(viewport.top - 100.8) < 1e-12);
  assert.equal(knownPoint.left, azimuthTick.left);
  assert.equal(knownPoint.top, elevationTick.top);
  assert.ok(Math.abs(knownPoint.left - 72.32142857142857) < 1e-12);
  assert.ok(Math.abs(knownPoint.top - 27.67857142857143) < 1e-12);
});
