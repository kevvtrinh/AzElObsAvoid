function [minimumClearance_deg, sampleCount, sampleClearance_deg] = ...
        sampleAzElObstacleClearance(obstacleField, time_s, position_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [minimumClearance_deg, sampleCount, sampleClearance_deg] = ...
%       azElInternal.sampleAzElObstacleClearance( ...
%       obstacleField, time_s, position_deg)
%**************************************************************************
% PURPOSE
%   - Measure dense-sample signed obstacle clearance for diagnostics.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed-obstacle struct)
%       Canonical original or safety-adjusted obstacle geometry.
%   - time_s (N-by-1 numeric vector)
%       Absolute sample times.
%   - position_deg (N-by-2 numeric matrix)
%       Azimuth and elevation sample positions.
%**************************************************************************
% OUTPUTS
%   - minimumClearance_deg (numeric scalar)
%       Minimum signed sample clearance. Negative values are inside.
%   - sampleCount (nonnegative integer scalar)
%       Number of evaluated path samples.
%   - sampleClearance_deg (N-by-1 numeric vector)
%       Signed minimum clearance at each sample.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees. Time is seconds.
%**************************************************************************

%% Section 1: Evaluate Each Dense Sample

sampleCount = numel(time_s);
sampleClearance_deg = inf(sampleCount, 1);
if isempty(obstacleField.Obstacles)
    minimumClearance_deg = Inf;
    return;
end
edgeSets_deg = azElInternal.interpolateAzElObstacleEdges( ...
    obstacleField, time_s);
for sampleIndex = 1:sampleCount
    for obstacleIndex = 1:numel(obstacleField.Obstacles)
        edges_deg = edgeSets_deg{sampleIndex, obstacleIndex};
        if isempty(edges_deg)
            continue;
        end
        [~, nearestPoint_deg] = nearestEdge( ...
            position_deg(sampleIndex, :), edges_deg);
        clearance_deg = norm( ...
            position_deg(sampleIndex, :) - nearestPoint_deg);
        if pointInsideEdges(position_deg(sampleIndex, :), edges_deg)
            clearance_deg = -clearance_deg;
        end
        sampleClearance_deg(sampleIndex) = min( ...
            sampleClearance_deg(sampleIndex), clearance_deg);
    end
end
minimumClearance_deg = min(sampleClearance_deg);
end

%% Section 2: Local Functions

function [edgeIndex, nearestPoint_deg] = nearestEdge(point_deg, edges_deg)
% PURPOSE
%   - Select the deterministic closest edge and its projection.
edgeStart_deg = edges_deg(:, 1:2);
edgeDelta_deg = edges_deg(:, 3:4) - edgeStart_deg;
edgeLengthSquared_deg2 = sum(edgeDelta_deg .^ 2, 2);
projectionFraction = sum((point_deg - edgeStart_deg) .* ...
    edgeDelta_deg, 2) ./ max(edgeLengthSquared_deg2, eps);
projectionFraction = min(1, max(0, projectionFraction));
projection_deg = edgeStart_deg + projectionFraction .* edgeDelta_deg;
distanceSquared_deg2 = sum((projection_deg - point_deg) .^ 2, 2);
[~, edgeIndex] = min(distanceSquared_deg2);
nearestPoint_deg = projection_deg(edgeIndex, :);
end

function inside = pointInsideEdges(point_deg, edges_deg)
% PURPOSE
%   - Classify one point for signed diagnostic clearance.
y0 = edges_deg(:, 2);
y1 = edges_deg(:, 4);
crossesElevation = (y0 > point_deg(2)) ~= (y1 > point_deg(2));
crossingAzimuth_deg = edges_deg(:, 1) + ...
    (point_deg(2) - y0) .* (edges_deg(:, 3) - edges_deg(:, 1)) ./ ...
    max(abs(y1 - y0), eps) .* sign(y1 - y0 + eps);
inside = mod(nnz(crossesElevation & ...
    crossingAzimuth_deg > point_deg(1)), 2) == 1;
end
