function [clearance_deg, nearestPoint_deg, edgeIndex] = pointPolygonClearance(shape, point_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [clearance_deg, nearestPoint_deg, edgeIndex] = obstacleAvoidance.geometry.pointPolygonClearance(shape, point_deg)
%**************************************************************************
% PURPOSE
%   - Compute signed Euclidean clearance from points to one polyshape.
%**************************************************************************
% INPUTS
%   - shape (scalar polyshape)
%       Occupied polygon geometry.
%   - point_deg (N-by-2 finite numeric array)
%       Query points in [azimuth elevation] order.
%**************************************************************************
% OUTPUTS
%   - clearance_deg (N-by-1 vector)
%       Positive outside, zero on the boundary, and negative inside.
%   - nearestPoint_deg (N-by-2 array)
%       Closest boundary point, or [NaN NaN] for empty geometry.
%   - edgeIndex (N-by-1 positive integer or zero)
%       One-based edge index in deterministic boundary traversal order.
%**************************************************************************
% UNITS
%   - Point, clearance, and nearest boundary position are degrees.
%**************************************************************************

%% Section 1: Validate Inputs

if ~isa(shape, "polyshape") || ~isscalar(shape)
    error("pointPolygonClearance:InvalidShape", ...
        "shape must be a scalar polyshape.");
end
validateattributes(point_deg, {'numeric'}, {'real', 'finite', '2d', 'ncols', 2, 'nonempty'});
point_deg = double(point_deg);
queryCount = size(point_deg, 1);
nearestPoint_deg = nan(queryCount, 2);
edgeIndex = zeros(queryCount, 1);
if isempty(shape.Vertices)
    clearance_deg = inf(queryCount, 1);
    return;
end

%% Section 2: Traverse Boundary Edges

% Edge order is stable and shared with visibility tests, so the returned edge
% index remains meaningful diagnostic information.
[edgeStart_deg, edgeEnd_deg] = obstacleAvoidance.geometry.boundaryToEdges(shape, 0);

%% Section 3: Project Query Blocks And Apply The Occupancy Sign

% Projection onto every segment gives exact Euclidean boundary distance.
% Processing 64 queries at a time bounds the temporary query-by-edge matrices
% for dense validation histories.
edgeDelta_deg = edgeEnd_deg - edgeStart_deg;
edgeLengthSquared_deg2 = sum(edgeDelta_deg .^ 2, 2);
nonzeroEdge = edgeLengthSquared_deg2 > 0;
edgeLengthSquared_deg2(~nonzeroEdge) = 1;
clearance_deg = zeros(queryCount, 1);

% Bound temporary query-by-edge matrices by projecting 64 query points at a time.
for blockStart = 1:64:queryCount
    selectedQuery = blockStart:min(queryCount, blockStart + 63);
    azimuthOffset_deg = point_deg(selectedQuery, 1) - edgeStart_deg(:, 1).';
    elevationOffset_deg = point_deg(selectedQuery, 2) - edgeStart_deg(:, 2).';
    projectionFraction = (azimuthOffset_deg .* edgeDelta_deg(:, 1).' + ...
        elevationOffset_deg .* edgeDelta_deg(:, 2).') ./ edgeLengthSquared_deg2.';
    projectionFraction(:, ~nonzeroEdge) = 0;
    projectionFraction = min(1, max(0, projectionFraction));
    projectedAzimuth_deg = edgeStart_deg(:, 1).' + projectionFraction .* edgeDelta_deg(:, 1).';
    projectedElevation_deg = edgeStart_deg(:, 2).' + projectionFraction .* edgeDelta_deg(:, 2).';
    distanceSquared_deg2 = (point_deg(selectedQuery, 1) - projectedAzimuth_deg) .^ 2 + ...
        (point_deg(selectedQuery, 2) - projectedElevation_deg) .^ 2;
    [minimumDistanceSquared_deg2, selectedEdgeIndex] = min(distanceSquared_deg2, [], 2);
    selectedLinearIndex = sub2ind(size(projectionFraction), (1:numel(selectedQuery)).', selectedEdgeIndex);
    selectedFraction = projectionFraction(selectedLinearIndex);
    nearestPoint_deg(selectedQuery, :) = edgeStart_deg(selectedEdgeIndex, :) + ...
        selectedFraction .* edgeDelta_deg(selectedEdgeIndex, :);
    edgeIndex(selectedQuery) = selectedEdgeIndex;
    clearance_deg(selectedQuery) = sqrt(max(0, minimumDistanceSquared_deg2));
end
isInside = isinterior(shape, point_deg(:, 1), point_deg(:, 2));
% Distance magnitude alone cannot distinguish safe exterior points from
% occupied interior points; the sign supplies that semantic distinction.
clearance_deg(isInside) = -clearance_deg(isInside);
coordinateScale_deg = max(1, max(abs(point_deg), [], 2));
clearance_deg(abs(clearance_deg) <= 1e-12 * coordinateScale_deg) = 0;
end
