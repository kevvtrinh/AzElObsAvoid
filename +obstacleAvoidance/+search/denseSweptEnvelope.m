function [envelopeShape, usedEnvelope] = denseSweptEnvelope( ...
        obstacles, sampleTimes_s, endpointPosition_deg, vertexWorkBudget)
%% Section 0: Header & Readme
% SYNTAX
%   [envelopeShape, usedEnvelope] = ...
%       obstacleAvoidance.search.denseSweptEnvelope( ...
%       obstacles, sampleTimes_s, endpointPosition_deg, vertexWorkBudget)
%**************************************************************************
% PURPOSE
%   - Replace an unaffordable sampled union with one conservative convex
%     history envelope per obstacle for topology proposals only.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle struct array)
%       Only geometry applicable to the sampleTimes_s horizon is enclosed.
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
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; work is a vertex count.
%**************************************************************************

%% Section 1: Detect Dense History Work

validateattributes(sampleTimes_s, {'numeric'}, ...
    {'real', 'finite', 'vector', 'nonempty'});
validateattributes(endpointPosition_deg, {'numeric'}, {'real', 'finite', 'size', [2 2]});
validateattributes(vertexWorkBudget, {'numeric'}, {'real', 'finite', 'positive', 'scalar'});
verticesPerLayer = 0;
horizonStart_s = min(sampleTimes_s);
horizonEnd_s = max(sampleTimes_s);
horizonGeometry = cell(numel(obstacles), 1);
for obstacleIndex = 1:numel(obstacles)
    horizonGeometry{obstacleIndex} = ...
        obstacleAvoidance.obstacles.queryHorizonGeometry( ...
        obstacles(obstacleIndex), horizonStart_s, horizonEnd_s);
    verticesPerLayer = verticesPerLayer + ...
        size(horizonGeometry{obstacleIndex}.SweepShape.Vertices, 1);
end
envelopeShape = polyshape();
usedEnvelope = false;
if numel(sampleTimes_s) * verticesPerLayer <= vertexWorkBudget
    return;
end

%% Section 2: Enclose Every Complete Stored History

% The shared horizon query already encloses corresponding-vertex motion and
% every applicable fallback interval. Separate hulls do not bridge obstacles.
envelopes = cell(numel(obstacles), 1);
envelopeCount = 0;
for obstacleIndex = 1:numel(obstacles)
    trialShape = horizonGeometry{obstacleIndex}.SweepShape;
    if isempty(trialShape.Vertices)
        continue;
    end
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
