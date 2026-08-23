function convexRegions = convexPolygonRegions(shape)
%% Section 0: Header & Readme
% SYNTAX
%   convexRegions = azElPlannerMethods.corridor.internal.geometry.convexPolygonRegions(shape)
%**************************************************************************
% PURPOSE
%   - Decompose occupied polygon geometry into exact convex regions for
%     local supporting-half-space corridor construction and certification.
%**************************************************************************
% INPUTS
%   - shape (scalar polyshape)
%       Possibly disconnected or nonconvex occupied geometry.
%**************************************************************************
% OUTPUTS
%   - convexRegions (column polyshape array)
%       Nonoverlapping convex polygons whose union equals shape.
%**************************************************************************
% UNITS
%   - Geometry coordinates retain the input shape's angular degree units.
%**************************************************************************

%% Section 1: Validate And Decompose Every Connected Region

% A convex region needs only one supporting half-space for a query direction.
% Nonconvex regions are triangulated so corridor construction never assumes a
% concave polygon has a globally valid single support plane.
if ~isa(shape, "polyshape") || ~isscalar(shape)
    error("convexPolygonRegions:InvalidShape", "shape must be a scalar polyshape.");
end
connectedRegions = regions(shape);
convexRegions = polyshape.empty(0, 1);

% Process disconnected occupied regions independently so none are lost.
for regionIndex = 1:numel(connectedRegions)
    connectedRegion = connectedRegions(regionIndex);
    finiteVertex_deg = connectedRegion.Vertices;
    finiteVertex_deg = finiteVertex_deg( all(isfinite(finiteVertex_deg), 2), :);
    hullIndex = convhull(finiteVertex_deg(:, 1), finiteVertex_deg(:, 2));
    hull = polyshape(finiteVertex_deg(hullIndex(1:end - 1), :), "Simplify", false, "KeepCollinearPoints", true);
    areaTolerance_deg2 = 256 * eps(max(1, area(hull)));
    if abs(area(hull) - area(connectedRegion)) <= areaTolerance_deg2
        % Preserve an already-convex region instead of replacing its exact
        % boundary with a numerically reconstructed hull.
        convexRegions(end + 1, 1) = connectedRegion; %#ok<AGROW>
        continue;
    end
    regionTriangulation = triangulation(connectedRegion);
    point_deg = regionTriangulation.Points;
    triangleIndex = regionTriangulation.ConnectivityList;

    % Retain every positive-area triangle as an exact convex corridor region.
    for triangleIndexRow = 1:size(triangleIndex, 1)
        triangle_deg = point_deg(triangleIndex(triangleIndexRow, :), :);
        triangle = polyshape( triangle_deg(:, 1), triangle_deg(:, 2), "Simplify", false);
        if area(triangle) > 0
            convexRegions(end + 1, 1) = triangle; %#ok<AGROW>
        end
    end
end
end
