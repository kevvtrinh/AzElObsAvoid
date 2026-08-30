export function createViewport(bounds, width_px, height_px) {
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

export function createTicks(minimum_deg, maximum_deg, intervalCount = 8) {
  const step_deg = (maximum_deg - minimum_deg) / intervalCount;
  return Array.from({ length: intervalCount + 1 }, (_, tickIndex) => {
    const tick_deg = tickIndex === intervalCount
      ? maximum_deg
      : minimum_deg + tickIndex * step_deg;
    return Math.abs(tick_deg) < Math.abs(step_deg) * 1e-10
      ? 0
      : +tick_deg.toFixed(8);
  });
}

export function coordinateToFramePercent(position_deg, viewport) {
  return {
    left: (position_deg[0] - viewport.left)
      / (viewport.right - viewport.left) * 100,
    top: (viewport.top - position_deg[1])
      / (viewport.top - viewport.bottom) * 100,
  };
}
