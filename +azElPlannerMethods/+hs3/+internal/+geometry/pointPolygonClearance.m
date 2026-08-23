function [clearance_deg, nearestPoint_deg, edgeIndex] = pointPolygonClearance( ...
        shape, point_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [clearance_deg, nearestPoint_deg, edgeIndex] = azElPlannerMethods.hs3.internal.geometry.pointPolygonClearance( ...
%       shape, point_deg)
%**************************************************************************
% PURPOSE
%   - Compute signed Euclidean clearance from one point to one polyshape.
%**************************************************************************
% INPUTS
%   - shape (scalar polyshape)
%       Occupied polygon geometry.
%   - point_deg (1-by-2 finite numeric row)
%       Query point in [azimuth elevation] order.
%**************************************************************************
% OUTPUTS
%   - clearance_deg (scalar)
%       Positive outside, zero on the boundary, and negative inside.
%   - nearestPoint_deg (1-by-2 row)
%       Closest boundary point, or [NaN NaN] for empty geometry.
%   - edgeIndex (positive integer or zero)
%       One-based edge index in deterministic boundary traversal order.
%**************************************************************************
% UNITS
%   - Point, clearance, and nearest boundary position are degrees.
%**************************************************************************

%% Section 1: Validate Inputs

if ~isa(shape, "polyshape") || ~isscalar(shape)
    error("azElInternal:pointPolygonClearance:InvalidShape", ...
        "shape must be a scalar polyshape.");
end
validateattributes(point_deg, {'numeric'}, ...
    {'real', 'finite', 'size', [1 2]});
point_deg = double(point_deg);
nearestPoint_deg = [NaN NaN];
edgeIndex = 0;
if isempty(shape.Vertices)
    clearance_deg = Inf;
    return;
end

%% Section 2: Traverse Boundary Edges

[boundaryAzimuth_deg, boundaryElevation_deg] = boundary(shape);
boundaryPosition_deg = [ ...
    double(boundaryAzimuth_deg(:)), double(boundaryElevation_deg(:))];
finiteRows = all(isfinite(boundaryPosition_deg), 2);
regionChanges = diff([false; finiteRows; false]);
regionStarts = find(regionChanges == 1);
regionStops = find(regionChanges == -1) - 1;
minimumDistanceSquared_deg2 = Inf;
traversedEdgeCount = 0;

% Measure every closed boundary region so disconnected rings and holes all
% contribute to the signed clearance result.
for regionIndex = 1:numel(regionStarts)
    vertices_deg = boundaryPosition_deg( ...
        regionStarts(regionIndex):regionStops(regionIndex), :);
    if size(vertices_deg, 1) < 2
        continue;
    end
    if all(vertices_deg(1, :) == vertices_deg(end, :))
        vertices_deg(end, :) = [];
    end
    edgeStart_deg = vertices_deg;
    edgeEnd_deg = vertices_deg([2:end 1], :);
    edgeDelta_deg = edgeEnd_deg - edgeStart_deg;
    edgeLengthSquared_deg2 = sum(edgeDelta_deg.^2, 2);
    pointOffset_deg = point_deg - edgeStart_deg;
    projectionFraction = zeros(size(edgeLengthSquared_deg2));
    nonzeroEdges = edgeLengthSquared_deg2 > 0;
    projectionFraction(nonzeroEdges) = sum( ...
        pointOffset_deg(nonzeroEdges, :) .* ...
        edgeDelta_deg(nonzeroEdges, :), 2) ./ ...
        edgeLengthSquared_deg2(nonzeroEdges);
    projectionFraction = min(1, max(0, projectionFraction));
    projectedPoint_deg = edgeStart_deg + ...
        projectionFraction .* edgeDelta_deg;
    distanceSquared_deg2 = sum((point_deg - projectedPoint_deg).^2, 2);
    [regionMinimum_deg2, regionEdgeIndex] = min(distanceSquared_deg2);
    if regionMinimum_deg2 < minimumDistanceSquared_deg2
        minimumDistanceSquared_deg2 = regionMinimum_deg2;
        nearestPoint_deg = projectedPoint_deg(regionEdgeIndex, :);
        edgeIndex = traversedEdgeCount + regionEdgeIndex;
    end
    traversedEdgeCount = traversedEdgeCount + size(edgeStart_deg, 1);
end

%% Section 3: Apply The Occupancy Sign

unsignedClearance_deg = sqrt(max(0, minimumDistanceSquared_deg2));
isInside = isinterior(shape, point_deg(1), point_deg(2));
if isInside
    clearance_deg = -unsignedClearance_deg;
else
    clearance_deg = unsignedClearance_deg;
end
coordinateScale_deg = max(1, max(abs(point_deg)));
boundaryTolerance_deg = 1e-12 * coordinateScale_deg;
if unsignedClearance_deg <= boundaryTolerance_deg
    clearance_deg = 0;
end
end
