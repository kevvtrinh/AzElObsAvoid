function [envelopeShape, usedEnvelope] = denseSweptEnvelope( ...
        obstacles, sampleTimes_s, endpointPosition_deg, vertexWorkBudget)
%% Section 0: Header & Readme
% SYNTAX
%   [envelopeShape, usedEnvelope] = azElSearch.denseSweptEnvelope( ...
%       obstacles, sampleTimes_s, endpointPosition_deg, vertexWorkBudget)
%**************************************************************************
% PURPOSE
%   - Replace an unaffordable exact swept union with a conservative
%     directional support hull for seed generation only.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle struct array)
%       All protected history vertices define the envelope bounds.
%   - sampleTimes_s (numeric vector)
%       Seed-union times used to estimate Boolean vertex work.
%   - endpointPosition_deg (2-by-2 numeric array)
%       Start and goal positions in [azimuth elevation] order.
%   - vertexWorkBudget (positive numeric scalar)
%       Maximum estimated vertices for the ordinary swept union.
%**************************************************************************
% OUTPUTS
%   - envelopeShape (scalar polyshape)
%       Conservative seed-only coarse hull, or an empty shape when unused.
%   - usedEnvelope (logical scalar)
%       True only when the coarse hull is safe for both endpoints.
%**************************************************************************
% UNITS
%   - Position is degrees. Time is seconds. The work budget is a count.
%**************************************************************************

%% Section 1: Estimate The Swept Boolean Work

validateattributes(sampleTimes_s, {'numeric'}, ...
    {'real', 'finite', 'vector'});
validateattributes(endpointPosition_deg, {'numeric'}, ...
    {'real', 'finite', 'size', [2 2]});
validateattributes(vertexWorkBudget, {'numeric'}, ...
    {'real', 'finite', 'positive', 'scalar'});
maximumVerticesPerLayer = 0;

% Use the largest stored slice from each obstacle so the estimate cannot be
% optimistic about work at a sampled layer.
for obstacleIndex = 1:numel(obstacles)
    maximumVertexCount = 0;
    for sampleIndex = 1:numel(obstacles(obstacleIndex).az_deg)
        maximumVertexCount = max(maximumVertexCount, ...
            numel(obstacles(obstacleIndex).az_deg{sampleIndex}));
    end
    maximumVerticesPerLayer = maximumVerticesPerLayer + ...
        maximumVertexCount;
end
estimatedVertexWork = numel(sampleTimes_s) * maximumVerticesPerLayer;
envelopeShape = polyshape();
usedEnvelope = false;
if estimatedVertexWork <= vertexWorkBudget
    return;
end

%% Section 2: Build One Conservative Envelope Per Obstacle

% Separate envelopes cannot bridge free space between unrelated histories.
directionCounts = [16 24 32 48 64];
obstacleEnvelopes = cell(numel(obstacles), 1);
envelopeCount = 0;
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    historyVerticesBySlice_deg = cell(numel(obstacle.az_deg), 1);
    for sampleIndex = 1:numel(obstacle.az_deg)
        position_deg = [obstacle.az_deg{sampleIndex}(:), ...
            obstacle.el_deg{sampleIndex}(:)];
        historyVerticesBySlice_deg{sampleIndex} = position_deg( ...
            all(isfinite(position_deg), 2), :);
    end
    historyVertices_deg = vertcat(historyVerticesBySlice_deg{:});
    if size(historyVertices_deg, 1) < 3
        continue;
    end
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
    envelopeCount = envelopeCount + 1;
    obstacleEnvelopes{envelopeCount} = trialShape;
end
if envelopeCount == 0
    return;
end
envelopeShape = union([obstacleEnvelopes{1:envelopeCount}]);
usedEnvelope = true;
end

function envelopeShape = directionalSupportHull(vertices_deg, directionCount)
% Clip a bounding polygon by ordered supports to make a coarse hull.
angles_rad = (0:directionCount - 1).' * (2 * pi / directionCount);
normal = [cos(angles_rad), sin(angles_rad)];
minimum_deg = min(vertices_deg, [], 1);
maximum_deg = max(vertices_deg, [], 1);
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
% Clip one ordered convex polygon without duplicate intersections.
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
% Omit consecutive duplicate vertices created at support intersections.
if isempty(vertices_deg) || ...
        norm(vertices_deg(end, :) - point_deg) > 1e-7
    vertices_deg(end + 1, :) = point_deg;
end
end

function areOutside = endpointsAreOutside( ...
        envelopeShape, endpointPosition_deg)
% Reject a seed envelope that contains or touches a request endpoint.
if isempty(envelopeShape.Vertices)
    areOutside = false;
    return;
end
endpointGuard_deg = 1e-9;
guardedShape = polybuffer(envelopeShape, endpointGuard_deg);
areOutside = ~any(isinterior( ...
    guardedShape, endpointPosition_deg(:, 1), endpointPosition_deg(:, 2)));
end
