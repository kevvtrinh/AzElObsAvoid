function [clearance_deg, nearestPoint_deg, edgeIndex] = pointPolygonClearance(shape, point_deg, geometry)
%% Section 0: Header & Readme
% SYNTAX
%   [clearance_deg, nearestPoint_deg, edgeIndex] = ...
%       obstacleAvoidance.geometry.pointPolygonClearance(shape, point_deg)
%
% PURPOSE
%   - Compute signed Euclidean clearance from points to one polyshape.
%
% INPUTS
%   - shape (scalar polyshape)
%       Occupied polygon geometry.
%   - point_deg (N-by-2 finite numeric array)
%       Query points in [azimuth elevation] order.
%   - geometry (optional prepared boundary record)
%       Supplies cached edges and verified convex-ring classification.
%
% OUTPUTS
%   - clearance_deg (N-by-1 vector)
%       Positive outside, zero on the boundary, and negative inside.
%   - nearestPoint_deg (N-by-2 array)
%       Closest boundary point, or [NaN NaN] for empty geometry.
%   - edgeIndex (N-by-1 positive integer or zero)
%       One-based edge index in deterministic boundary traversal order.
%
% UNITS
%   - Point, clearance, and nearest boundary position are degrees.
%

%% Section 1: Validate Inputs

% Check polygon and query-point dimensions.

if ~isa(shape, "polyshape") || ~isscalar(shape)
    error("pointPolygonClearance:InvalidShape", "shape must be a scalar polyshape.");
end
validateattributes(point_deg, {'numeric'}, {'real', 'finite', '2d', 'ncols', 2, 'nonempty'});
point_deg        = double(point_deg);
queryCount       = size(point_deg, 1);
nearestPoint_deg = nan(queryCount, 2);
edgeIndex        = zeros(queryCount, 1);
if isempty(shape.Vertices)
    % Empty geometry has infinite clearance.
    clearance_deg = inf(queryCount, 1);
    return;
end

%% Section 2: Traverse Boundary Edges

% Measure distance to edges, not just vertices.

usePreparedGeometry = nargin >= 3 && isstruct(geometry) && isscalar(geometry) && all(isfield(geometry, {'EdgeStart_deg', 'EdgeEnd_deg', 'HasOrderedSingleRegion', 'IsConvex', 'OutwardSign'}));
if usePreparedGeometry
    edgeStart_deg = geometry.EdgeStart_deg;
    edgeEnd_deg   = geometry.EdgeEnd_deg;
else
    [edgeStart_deg, edgeEnd_deg] = obstacleAvoidance.geometry.boundaryToEdges(shape, 0);
end

%% Section 3: Project Query Blocks And Apply The Occupancy Sign

% Project onto each finite edge; use a negative distance inside the polygon.

edgeDelta_deg          = edgeEnd_deg - edgeStart_deg;
edgeLengthSquared_deg2 = sum(edgeDelta_deg .^ 2, 2);
nonzeroEdge            = edgeLengthSquared_deg2 > 0;
% Treat zero-length edges as points and avoid division by zero.
edgeLengthSquared_deg2(~nonzeroEdge) = 1;
clearance_deg = zeros(queryCount, 1);

% Target 512 KiB per double projection matrix, permitting one complete edge
% row for larger polygons. Small polygons can process more queries together.
maximumProjectionElementCount = 65536;
blockQueryCount               = max(1, floor(maximumProjectionElementCount / size(edgeStart_deg, 1)));
% Process each block start needed to complete point polygon clearance.
for blockStart = 1:blockQueryCount:queryCount
    selectedQuery       = blockStart:min(queryCount, blockStart + blockQueryCount - 1);
    azimuthOffset_deg   = point_deg(selectedQuery, 1) - edgeStart_deg(:, 1).';
    elevationOffset_deg = point_deg(selectedQuery, 2) - edgeStart_deg(:, 2).';
    projectionFraction  = (azimuthOffset_deg .* edgeDelta_deg(:, 1).' + elevationOffset_deg .* edgeDelta_deg(:, 2).') ./ edgeLengthSquared_deg2.';
    projectionFraction(:, ~nonzeroEdge) = 0;
    projectionFraction = min(1, max(0, projectionFraction));
    % Clamp projections to the finite segment.
    projectedAzimuth_deg   = edgeStart_deg(:, 1).' + projectionFraction .* edgeDelta_deg(:, 1).';
    projectedElevation_deg = edgeStart_deg(:, 2).' + projectionFraction .* edgeDelta_deg(:, 2).';
    distanceSquared_deg2   = (point_deg(selectedQuery, 1) - projectedAzimuth_deg) .^ 2 + (point_deg(selectedQuery, 2) - projectedElevation_deg) .^ 2;
    [minimumDistanceSquared_deg2, selectedEdgeIndex] = min(distanceSquared_deg2, [], 2);
    selectedLinearIndex = sub2ind(size(projectionFraction), (1:numel(selectedQuery)).', selectedEdgeIndex);
    selectedFraction    = projectionFraction(selectedLinearIndex);
    nearestPoint_deg(selectedQuery, :) = edgeStart_deg(selectedEdgeIndex, :) + selectedFraction .* edgeDelta_deg(selectedEdgeIndex, :);
    edgeIndex(selectedQuery) = selectedEdgeIndex;
    clearance_deg(selectedQuery) = sqrt(max(0, minimumDistanceSquared_deg2));
end
if usePreparedGeometry && geometry.HasOrderedSingleRegion && geometry.IsConvex
    outwardCross = geometry.OutwardSign * ((point_deg(:, 1) - edgeStart_deg(:, 1).') .* edgeDelta_deg(:, 2).' - (point_deg(:, 2) - edgeStart_deg(:, 2).') .* edgeDelta_deg(:, 1).');
    isInside = all(outwardCross >= 0, 2);
else
    isInside = isinterior(shape, point_deg(:, 1), point_deg(:, 2));
end
% Make clearance negative inside the polygon.
clearance_deg(isInside) = -clearance_deg(isInside);
coordinateScale_deg = max(1, max(abs(point_deg), [], 2));
% Snap boundary noise below 1e-12 of the coordinate scale to zero.
clearance_deg(abs(clearance_deg) <= 1e-12 * coordinateScale_deg) = 0;
end
