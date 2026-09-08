function [clearance_units, nearestPoint_units, edgeIndex] = pointPolygonClearance(shape, point_units, geometry)
%% Section 0: Header & Readme
% SYNTAX
%   [clearance_units, nearestPoint_units, edgeIndex] = ...
%       obstacleAvoidance.geometry.pointPolygonClearance(shape, point_units)
% PURPOSE
%   - Compute signed Euclidean clearance from points to one polyshape.
% INPUTS
%   - shape (scalar polyshape)
%       Occupied polygon geometry.
%   - point_units (N-by-2 finite numeric array)
%       Query points in [x y] order.
%   - geometry (optional prepared boundary record)
%       Supplies cached edges and verified convex-ring classification.
% OUTPUTS
%   - clearance_units (N-by-1 vector)
%       Positive outside, zero on the boundary, and negative inside.
%   - nearestPoint_units (N-by-2 array)
%       Closest boundary point, or [NaN NaN] for empty geometry.
%   - edgeIndex (N-by-1 positive integer or zero)
%       One-based edge index in deterministic boundary traversal order.
% UNITS
%   - Point, clearance, and nearest boundary position are coordinate units.

%% Section 1: Validate Inputs

% Check polygon and query-point dimensions.

if ~isa(shape, "polyshape") || ~isscalar(shape)
    error("pointPolygonClearance:InvalidShape", "shape must be a scalar polyshape.");
end
validateattributes(point_units, {'numeric'}, {'real', 'finite', '2d', 'ncols', 2, 'nonempty'});
point_units        = double(point_units);
queryCount       = size(point_units, 1);
nearestPoint_units = nan(queryCount, 2);
edgeIndex        = zeros(queryCount, 1);
if isempty(shape.Vertices)
    % Empty geometry has infinite clearance.
    clearance_units = inf(queryCount, 1);
    return;
end

%% Section 2: Traverse Boundary Edges

% Measure distance to edges, not just vertices.

usePreparedGeometry = nargin >= 3 && isstruct(geometry) && isscalar(geometry) && all(isfield(geometry, {'EdgeStart_units', 'EdgeEnd_units', 'HasOrderedSingleRegion', 'IsConvex', 'OutwardSign'}));
if usePreparedGeometry
    edgeStart_units = geometry.EdgeStart_units;
    edgeEnd_units   = geometry.EdgeEnd_units;
else
    [edgeStart_units, edgeEnd_units] = obstacleAvoidance.geometry.boundaryToEdges(shape, 0);
end

%% Section 3: Project Query Blocks And Apply The Occupancy Sign

% Project onto each finite edge; use a negative distance inside the polygon.

edgeDelta_units          = edgeEnd_units - edgeStart_units;
edgeLengthSquared_units2 = sum(edgeDelta_units .^ 2, 2);
nonzeroEdge            = edgeLengthSquared_units2 > 0;
% Treat zero-length edges as points and avoid division by zero.
edgeLengthSquared_units2(~nonzeroEdge) = 1;
clearance_units = zeros(queryCount, 1);
isInside = false(queryCount, 1);

% Target 512 KiB per double projection matrix, permitting one complete edge
% row for larger polygons. Small polygons can process more queries together.
maximumProjectionElementCount = 65536;
blockQueryCount               = max(1, floor(maximumProjectionElementCount / size(edgeStart_units, 1)));
for blockStart = 1:blockQueryCount:queryCount
    selectedQuery       = blockStart:min(queryCount, blockStart + blockQueryCount - 1);
    xOffset_units   = point_units(selectedQuery, 1) - edgeStart_units(:, 1).';
    yOffset_units = point_units(selectedQuery, 2) - edgeStart_units(:, 2).';
    projectionFraction  = (xOffset_units .* edgeDelta_units(:, 1).' + yOffset_units .* edgeDelta_units(:, 2).') ./ edgeLengthSquared_units2.';
    projectionFraction(:, ~nonzeroEdge) = 0;
    projectionFraction = min(1, max(0, projectionFraction));
    % Clamp projections to the finite segment.
    projectedX_units   = edgeStart_units(:, 1).' + projectionFraction .* edgeDelta_units(:, 1).';
    projectedY_units = edgeStart_units(:, 2).' + projectionFraction .* edgeDelta_units(:, 2).';
    distanceSquared_units2   = (point_units(selectedQuery, 1) - projectedX_units) .^ 2 + (point_units(selectedQuery, 2) - projectedY_units) .^ 2;
    [minimumDistanceSquared_units2, selectedEdgeIndex] = min(distanceSquared_units2, [], 2);
    selectedLinearIndex = sub2ind(size(projectionFraction), (1:numel(selectedQuery)).', selectedEdgeIndex);
    selectedFraction    = projectionFraction(selectedLinearIndex);
    nearestPoint_units(selectedQuery, :) = edgeStart_units(selectedEdgeIndex, :) + selectedFraction .* edgeDelta_units(selectedEdgeIndex, :);
    edgeIndex(selectedQuery) = selectedEdgeIndex;
    clearance_units(selectedQuery) = sqrt(max(0, minimumDistanceSquared_units2));
    % A half-open horizontal ray counts each shared vertex once. Parity over
    % every boundary ring preserves holes and disconnected occupied regions.
    % Reuse the bounded projection block; no dense all-query/edge array is needed.
    crossesHeight = (edgeStart_units(:, 2).' > point_units(selectedQuery, 2)) ~= (edgeEnd_units(:, 2).' > point_units(selectedQuery, 2));
    crossProduct_units2 = xOffset_units .* edgeDelta_units(:, 2).' - yOffset_units .* edgeDelta_units(:, 1).';
    crossesRight = (edgeDelta_units(:, 2).' > 0 & crossProduct_units2 < 0) | (edgeDelta_units(:, 2).' < 0 & crossProduct_units2 > 0);
    if usePreparedGeometry && geometry.HasOrderedSingleRegion && geometry.IsConvex
        isInside(selectedQuery) = all(geometry.OutwardSign * crossProduct_units2 >= 0, 2);
    else
        isInside(selectedQuery) = mod(sum(crossesHeight & crossesRight, 2), 2) ~= 0;
    end
end
% MATLAB's polyshape predicate includes a coordinate-scaled determinant
% tolerance near edges. Preserve that policy by querying it in the uncertainty
% band; ray parity alone would change signed gaps for near-boundary points.
if ~(usePreparedGeometry && geometry.HasOrderedSingleRegion && geometry.IsConvex)
    boundsMinimum_units = min(edgeStart_units, [], 1);
    boundsMaximum_units = max(edgeStart_units, [], 1);
    shapeScale_units = max(abs([boundsMinimum_units boundsMaximum_units]));
    determinantTolerance_units2 = (shapeScale_units + max(boundsMaximum_units - boundsMinimum_units)) * 1e-12;
    shortestEdge_units = sqrt(min(edgeLengthSquared_units2(nonzeroEdge)));
    uncertainty_units = determinantTolerance_units2 / shortestEdge_units + 64 * eps(max(1, max(shapeScale_units, max(abs(point_units), [], 2))));
    uncertain = clearance_units <= uncertainty_units;
    if any(uncertain)
        isInside(uncertain) = isinterior(shape, point_units(uncertain, 1), point_units(uncertain, 2));
    end
end
% Make clearance negative inside the polygon.
clearance_units(isInside) = -clearance_units(isInside);
coordinateScale_units = max(1, max(abs(point_units), [], 2));
% Snap boundary noise below 1e-12 of the coordinate scale to zero.
clearance_units(abs(clearance_units) <= 1e-12 * coordinateScale_units) = 0;
end
