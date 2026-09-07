function [envelopeShape, usedEnvelope, estimatedVertexWork] = denseSweptEnvelope(obstacles, sampleTimes_s, endpointPosition_units, vertexWorkBudget)
%% Section 0: Header & Readme
% SYNTAX
%   [envelopeShape, usedEnvelope] = ...
%       obstacleAvoidance.search.denseSweptEnvelope( ...
%       obstacles, sampleTimes_s, endpointPosition_units, vertexWorkBudget)
%   [envelopeShape, usedEnvelope, estimatedVertexWork] = ...
%       obstacleAvoidance.search.denseSweptEnvelope( ...
%       obstacles, sampleTimes_s, endpointPosition_units, vertexWorkBudget)
%
% PURPOSE
%   - Replace an unaffordable sampled union with one conservative convex
%     history envelope per obstacle for topology proposals only.
%
% INPUTS
%   - obstacles (canonical protected obstacle struct array)
%       Complete stored histories whose vertices define each envelope.
%   - sampleTimes_s (numeric vector)
%       Times used to estimate the ordinary sampled-union work.
%   - endpointPosition_units (2-by-2 numeric array)
%       Start and goal in [x y] order.
%   - vertexWorkBudget (positive numeric scalar)
%       Maximum estimated sampled-union vertex work.
%
% OUTPUTS
%   - envelopeShape (scalar polyshape)
%       Separate conservative history hulls, or empty when unused.
%   - usedEnvelope (logical scalar)
%       True only when dense fallback was needed and protects both endpoints.
%   - estimatedVertexWork (nonnegative integer scalar)
%       Sample-time count times the maximum stored vertices per obstacle.
%
% UNITS
%   - Position is coordinate units; time is seconds; work is a vertex count.
%

%% Section 1: Detect Dense History Work

validateattributes(sampleTimes_s, {'numeric'}, {'real', 'finite', 'vector'});
validateattributes(endpointPosition_units, {'numeric'}, {'real', 'finite', 'size', [2 2]});
validateattributes(vertexWorkBudget, {'numeric'}, {'real', 'finite', 'positive', 'scalar'});
verticesPerLayer = 0;
% Evaluate each obstacle against the current geometry or motion.
for obstacleIndex = 1:numel(obstacles)
    maximumVertexCount = 0;
    % Process each sample in temporal order and accumulate its result.
    for sampleIndex = 1:numel(obstacles(obstacleIndex).x_units)
        maximumVertexCount = max(maximumVertexCount, numel(obstacles(obstacleIndex).x_units{sampleIndex}));
    end
    verticesPerLayer = verticesPerLayer + maximumVertexCount;
end
envelopeShape       = polyshape();
usedEnvelope        = false;
estimatedVertexWork = numel(sampleTimes_s) * verticesPerLayer;
if estimatedVertexWork <= vertexWorkBudget
    return;
end

%% Section 2: Enclose Every Complete Stored History

% Endpoint convex hulls cover linear vertex motion and topology changes.
% Use a separate hull for each obstacle to avoid joining unrelated shapes.
envelopes     = cell(numel(obstacles), 1);
envelopeCount = 0;
% Evaluate each obstacle against the current geometry or motion.
for obstacleIndex = 1:numel(obstacles)
    obstacle     = obstacles(obstacleIndex);
    vertices_units = zeros(0, 2);
    % Process each sample in temporal order and accumulate its result.
    for sampleIndex = 1:numel(obstacle.x_units)
        sample_units   = [obstacle.x_units{sampleIndex}(:), obstacle.y_units{sampleIndex}(:)];
        vertices_units = [vertices_units; sample_units(all(isfinite(sample_units), 2), :)]; %#ok<AGROW>
    end
    vertices_units = unique(vertices_units, "rows", "stable");
    if size(vertices_units, 1) < 3
        continue;
    end
    hullIndex    = convhull(vertices_units(:, 1), vertices_units(:, 2));
    trialShape   = polyshape(vertices_units(hullIndex(1:end - 1), :), "Simplify", false, "KeepCollinearPoints", true);
    guardedShape = polybuffer(trialShape, 1e-9);
    if any(isinterior(guardedShape, endpointPosition_units(:, 1), endpointPosition_units(:, 2)))
        envelopeShape = polyshape();
        return;
    end
    envelopeCount = envelopeCount + 1;
    envelopes{envelopeCount} = trialShape;
end
if envelopeCount > 0
    envelopeShape = union([envelopes{1:envelopeCount}]);
    usedEnvelope  = true;
end
end
