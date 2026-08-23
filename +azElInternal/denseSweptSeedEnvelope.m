function [envelopeShape, usedEnvelope] = denseSweptSeedEnvelope( ...
        obstacles, sampleTimes_s, endpointPosition_deg, vertexWorkBudget)
% Replace excessive seed-only swept-union work with a conservative hull.
%% Section 1: Estimate The Swept Boolean Work
validateattributes(sampleTimes_s, {'numeric'}, {'real', 'finite', 'vector'});
validateattributes(endpointPosition_deg, {'numeric'}, ...
    {'real', 'finite', 'size', [2 2]});
validateattributes(vertexWorkBudget, {'numeric'}, ...
    {'real', 'finite', 'positive', 'scalar'});
maximumVerticesPerLayer = 0;
for obstacleIndex = 1:numel(obstacles)
    maximumVertexCount = 0;
    for sampleIndex = 1:numel(obstacles(obstacleIndex).az_deg)
        maximumVertexCount = max(maximumVertexCount, ...
            numel(obstacles(obstacleIndex).az_deg{sampleIndex}));
    end
    maximumVerticesPerLayer = ...
        maximumVerticesPerLayer + maximumVertexCount;
end
estimatedVertexWork = numel(sampleTimes_s) * maximumVerticesPerLayer;
envelopeShape = polyshape();
usedEnvelope = false;
if estimatedVertexWork <= vertexWorkBudget
    return;
end
%% Section 2: Collect All Protected History Vertices
historyVertices_deg = zeros(0, 2);
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    for sampleIndex = 1:numel(obstacle.az_deg)
        position_deg = [obstacle.az_deg{sampleIndex}(:), ...
            obstacle.el_deg{sampleIndex}(:)];
        position_deg = position_deg(all(isfinite(position_deg), 2), :);
        if isempty(position_deg)
            continue;
        end
        historyVertices_deg = [historyVertices_deg; position_deg]; %#ok<AGROW>
    end
end
if size(historyVertices_deg, 1) < 3
    return;
end
%% Section 3: Build A Conservative Directional Support Hull
% A fixed set of support directions limits graph size. More directions are
% tried only when a coarse hull captures an endpoint that the exact convex
% hull does not capture.
directionCounts = [16 24 32 48 64];
trialShape = polyshape();
for directionCount = directionCounts
    candidateShape = directionalSupportHull( ...
        historyVertices_deg, directionCount);
    if endpointsAreOutside(candidateShape, endpointPosition_deg)
        trialShape = candidateShape;
        break;
    end
end
if isempty(trialShape.Vertices)
    hullIndex = convhull( ...
        historyVertices_deg(:, 1), historyVertices_deg(:, 2));
    trialShape = polyshape( ...
        historyVertices_deg(hullIndex, :), "Simplify", true);
end
if ~endpointsAreOutside(trialShape, endpointPosition_deg)
    return;
end
envelopeShape = trialShape;
usedEnvelope = true;
end
%% Section 4: Local Functions
function envelopeShape = directionalSupportHull(vertices_deg, directionCount)
% PURPOSE
%   - Clip a bounding polygon by ordered supports to make a coarse hull.
angles_rad = (0:directionCount - 1).' * (2 * pi / directionCount);
normal = [cos(angles_rad), sin(angles_rad)];
minimum_deg = min(vertices_deg, [], 1);
maximum_deg = max(vertices_deg, [], 1);
% This small outward pad makes vertex containment stable at support lines.
supportPadding_deg = 1e-7 * max(1, max(maximum_deg - minimum_deg));
offset_deg = max(vertices_deg * normal.', [], 1).' + supportPadding_deg;
center_deg = 0.5 * (minimum_deg + maximum_deg);
halfWidth_deg = 2 * max(1, max(maximum_deg - minimum_deg));
envelopeVertices_deg = center_deg + halfWidth_deg * ...
    [-1 -1; 1 -1; 1 1; -1 1];
for directionIndex = 1:directionCount
    envelopeVertices_deg = clipToSupportHalfPlane( ...
        envelopeVertices_deg, normal(directionIndex, :), ...
        offset_deg(directionIndex));
end
hullIndex = convhull( ...
    envelopeVertices_deg(:, 1), envelopeVertices_deg(:, 2));
envelopeShape = polyshape( ...
    envelopeVertices_deg(hullIndex(1:end - 1), :), "Simplify", false);
end
function clippedVertices_deg = clipToSupportHalfPlane( ...
        vertices_deg, normal, offset_deg)
% PURPOSE
%   - Clip one ordered convex polygon without duplicate intersections.
clippedVertices_deg = zeros(0, 2);
previousPoint_deg = vertices_deg(end, :);
previousInside = previousPoint_deg * normal.' <= offset_deg + 1e-12;
for vertexIndex = 1:size(vertices_deg, 1)
    currentPoint_deg = vertices_deg(vertexIndex, :);
    currentInside = currentPoint_deg * normal.' <= offset_deg + 1e-12;
    if currentInside ~= previousInside
        edge_deg = currentPoint_deg - previousPoint_deg;
        fraction = (offset_deg - previousPoint_deg * normal.') / ...
            (edge_deg * normal.');
        intersection_deg = previousPoint_deg + fraction * edge_deg;
        clippedVertices_deg = appendDistinctPoint( ...
            clippedVertices_deg, intersection_deg);
    end
    if currentInside
        clippedVertices_deg = appendDistinctPoint( ...
            clippedVertices_deg, currentPoint_deg);
    end
    previousPoint_deg = currentPoint_deg;
    previousInside = currentInside;
end
if size(clippedVertices_deg, 1) > 1 && ...
        norm(clippedVertices_deg(1, :) - ...
        clippedVertices_deg(end, :)) <= 1e-7
    clippedVertices_deg(end, :) = [];
end
end
function vertices_deg = appendDistinctPoint(vertices_deg, point_deg)
% PURPOSE
%   - Omit consecutive duplicate vertices created at support intersections.
if isempty(vertices_deg) || norm(vertices_deg(end, :) - point_deg) > 1e-7
    vertices_deg(end + 1, :) = point_deg;
end
end
function areOutside = endpointsAreOutside(envelopeShape, endpointPosition_deg)
% PURPOSE
%   - Reject a seed envelope that contains or touches a request endpoint.
if isempty(envelopeShape.Vertices)
    areOutside = false;
    return;
end
endpointGuard_deg = 1e-9;
guardedShape = polybuffer(envelopeShape, endpointGuard_deg);
areOutside = ~any(isinterior( ...
    guardedShape, endpointPosition_deg(:, 1), endpointPosition_deg(:, 2)));
end
