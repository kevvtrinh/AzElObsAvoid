function [clearance_deg, nearest] = azElObstacleClearance( ...
        obstacles, point_deg, queryTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [clearance_deg, nearest] = azElObstacleClearance( ...
%       obstacles, point_deg, queryTime_s, options)
%**************************************************************************
% PURPOSE
%   - Compute signed point-to-union clearance from canonical polygons.
%**************************************************************************
% INPUTS
%   - obstacles (cell column)
%       Normalized canonical obstacles.
%   - point_deg (1-by-2 numeric)
%       Continuous unwrapped azimuth and elevation.
%   - queryTime_s (finite scalar)
%       Absolute query time.
%   - options (scalar struct)
%       temporalPadding_s, azimuthWrap, and azimuthDisplayRange_deg.
%**************************************************************************
% OUTPUTS
%   - clearance_deg (scalar)
%       Positive outside, zero on a boundary, and negative inside.
%   - nearest (scalar struct)
%       Source record for the region attaining minimum clearance.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees; time is seconds.

[regionRecords, motionSpeedBound_deg_s] = ...
    azElObstacleRegionsAtTime(obstacles, queryTime_s, ...
        options.temporalPadding_s);
clearance_deg = Inf;
nearest = struct( ...
    "obstacleIndex", 0, ...
    "regionIndex", 0, ...
    "targetName", "", ...
    "motionSpeedBound_deg_s", motionSpeedBound_deg_s);

for recordIndex = 1:numel(regionRecords)
    vertices_deg = regionRecords(recordIndex).vertices_deg;
    candidateClearance_deg = signedPointPolygonDistance( ...
        point_deg, vertices_deg);
    if options.azimuthWrap
        displaySpan_deg = diff(options.azimuthDisplayRange_deg);
        meanAzimuth_deg = mean(vertices_deg(:, 1));
        centerShiftCount = round( ...
            (point_deg(1) - meanAzimuth_deg) ./ displaySpan_deg);
        for shiftOffset = -1:1
            shiftedVertices_deg = vertices_deg;
            shiftedVertices_deg(:, 1) = shiftedVertices_deg(:, 1) + ...
                (centerShiftCount + shiftOffset) .* displaySpan_deg;
            candidateClearance_deg = min(candidateClearance_deg, ...
                signedPointPolygonDistance(point_deg, shiftedVertices_deg));
        end
    end
    if candidateClearance_deg < clearance_deg
        clearance_deg = candidateClearance_deg;
        nearest.obstacleIndex = regionRecords(recordIndex).obstacleIndex;
        nearest.regionIndex = regionRecords(recordIndex).regionIndex;
        nearest.targetName = regionRecords(recordIndex).targetName;
    end
end
end
function distance_deg = signedPointPolygonDistance(point_deg, vertices_deg)
%% Section 0: Header & Readme
% SYNTAX
%   distance_deg = signedPointPolygonDistance(point_deg, vertices_deg)
%**************************************************************************
% PURPOSE
%   - Compute Euclidean distance to polygon edges with an interior sign.
%**************************************************************************
% INPUTS
%   - point_deg (1-by-2 numeric)
%   - vertices_deg (N-by-2 numeric)
%**************************************************************************
% OUTPUTS
%   - distance_deg (scalar)
%**************************************************************************
% UNITS
%   - Coordinates and distance are degrees.

vertexCount = size(vertices_deg, 1);
minimumEdgeDistance_deg = Inf;
for vertexIndex = 1:vertexCount
    nextIndex = mod(vertexIndex, vertexCount) + 1;
    edgeStart_deg = vertices_deg(vertexIndex, :);
    edgeVector_deg = vertices_deg(nextIndex, :) - edgeStart_deg;
    edgeLengthSquared_deg2 = dot(edgeVector_deg, edgeVector_deg);
    if edgeLengthSquared_deg2 == 0
        projection_deg = edgeStart_deg;
    else
        projectionFraction = dot(point_deg - edgeStart_deg, ...
            edgeVector_deg) ./ edgeLengthSquared_deg2;
        projectionFraction = min(max(projectionFraction, 0), 1);
        projection_deg = edgeStart_deg + ...
            projectionFraction .* edgeVector_deg;
    end
    minimumEdgeDistance_deg = min(minimumEdgeDistance_deg, ...
        norm(point_deg - projection_deg));
end
[isInside, isOnBoundary] = inpolygon(point_deg(1), point_deg(2), ...
    vertices_deg(:, 1), vertices_deg(:, 2));
if isOnBoundary
    distance_deg = 0;
elseif isInside
    distance_deg = -minimumEdgeDistance_deg;
else
    distance_deg = minimumEdgeDistance_deg;
end
end
