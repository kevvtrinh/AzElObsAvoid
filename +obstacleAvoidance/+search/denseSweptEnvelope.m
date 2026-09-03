function [envelopeShape, usedEnvelope, estimatedVertexWork] = ...
        denseSweptEnvelope( ...
        obstacles, sampleTimes_s, endpointPosition_deg, vertexWorkBudget)
%% Section 0: Header & Readme
% SYNTAX
%   [envelopeShape, usedEnvelope] = ...
%       obstacleAvoidance.search.denseSweptEnvelope( ...
%       obstacles, sampleTimes_s, endpointPosition_deg, vertexWorkBudget)
%   [envelopeShape, usedEnvelope, estimatedVertexWork] = ...
%       obstacleAvoidance.search.denseSweptEnvelope( ...
%       obstacles, sampleTimes_s, endpointPosition_deg, vertexWorkBudget)
%**************************************************************************
% PURPOSE
%   - Replace an unaffordable sampled union with one conservative convex
%     history envelope per obstacle for topology proposals only.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle struct array)
%       Complete stored histories whose vertices define each envelope.
%   - sampleTimes_s (numeric vector)
%       Times used to estimate the ordinary sampled-union work.
%   - endpointPosition_deg (2-by-2 numeric array)
%       Start and goal in [azimuth elevation] order.
%   - vertexWorkBudget (positive numeric scalar)
%       Maximum estimated sampled-union vertex work.
%**************************************************************************
% OUTPUTS
%   - envelopeShape (scalar polyshape)
%       Separate conservative history hulls, or empty when unused.
%   - usedEnvelope (logical scalar)
%       True only when dense fallback was needed and protects both endpoints.
%   - estimatedVertexWork (nonnegative integer scalar)
%       Sample-time count times the maximum stored vertices per obstacle.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; work is a vertex count.
%**************************************************************************

%% Section 1: Detect Dense History Work

validateattributes(sampleTimes_s, {'numeric'}, {'real', 'finite', 'vector'});
validateattributes(endpointPosition_deg, {'numeric'}, {'real', 'finite', 'size', [2 2]});
validateattributes(vertexWorkBudget, {'numeric'}, {'real', 'finite', 'positive', 'scalar'});
verticesPerLayer = 0;
for obstacleIndex = 1:numel(obstacles)
    maximumVertexCount = 0;
    for sampleIndex = 1:numel(obstacles(obstacleIndex).az_deg)
        maximumVertexCount = max(maximumVertexCount, ...
            numel(obstacles(obstacleIndex).az_deg{sampleIndex}));
    end
    verticesPerLayer = verticesPerLayer + maximumVertexCount;
end
envelopeShape = polyshape();
usedEnvelope = false;
estimatedVertexWork = numel(sampleTimes_s) * verticesPerLayer;
if estimatedVertexWork <= vertexWorkBudget
    return;
end

%% Section 2: Enclose Every Complete Stored History

% Linear corresponding-vertex motion lies in the convex hull of its endpoint
% vertices. A topology-change query uses endpoint geometry, which is contained
% by the same hull. Separate hulls do not bridge unrelated obstacles.
envelopes = cell(numel(obstacles), 1);
envelopeCount = 0;
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    vertices_deg = zeros(0, 2);
    for sampleIndex = 1:numel(obstacle.az_deg)
        sample_deg = [obstacle.az_deg{sampleIndex}(:), obstacle.el_deg{sampleIndex}(:)];
        vertices_deg = [vertices_deg; sample_deg(all(isfinite(sample_deg), 2), :)]; %#ok<AGROW>
    end
    vertices_deg = unique(vertices_deg, "rows", "stable");
    if size(vertices_deg, 1) < 3
        continue;
    end
    hullIndex = convhull(vertices_deg(:, 1), vertices_deg(:, 2));
    trialShape = polyshape(vertices_deg(hullIndex(1:end - 1), :), ...
        "Simplify", false, "KeepCollinearPoints", true);
    guardedShape = polybuffer(trialShape, 1e-9);
    if any(isinterior(guardedShape, endpointPosition_deg(:, 1), endpointPosition_deg(:, 2)))
        envelopeShape = polyshape();
        return;
    end
    envelopeCount = envelopeCount + 1;
    envelopes{envelopeCount} = trialShape;
end
if envelopeCount > 0
    envelopeShape = union([envelopes{1:envelopeCount}]);
    usedEnvelope = true;
end
end
