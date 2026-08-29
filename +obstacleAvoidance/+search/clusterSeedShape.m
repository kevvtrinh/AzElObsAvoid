function [clusteredShape, record] = clusterSeedShape( ...
        sweptShape, clusterDistance_deg, protectedPoints_deg, invalidShapeIdentifier)
%% Section 0: Header & Readme
% SYNTAX
%   [clusteredShape, record] = ...
%       obstacleAvoidance.search.clusterSeedShape( ...
%       sweptShape, clusterDistance_deg, protectedPoints_deg, ...
%       invalidShapeIdentifier)
%**************************************************************************
% PURPOSE
%   - Conservatively replace connected groups of at least three nearby
%     proposal regions with endpoint-safe convex hulls.
%**************************************************************************
% INPUTS
%   - sweptShape (scalar polyshape)
%       Protected proposal geometry.
%   - clusterDistance_deg (nonnegative numeric scalar)
%       Maximum connected gap; zero disables clustering.
%   - protectedPoints_deg (N-by-2 numeric array)
%       Request points that no hull may contain or touch.
%   - invalidShapeIdentifier (string scalar)
%       Caller-owned invalid-shape error identifier.
%**************************************************************************
% OUTPUTS
%   - clusteredShape (scalar polyshape)
%       Seed-only conservative geometry.
%   - record (scalar struct)
%       Source, group, absorbed-region, and boundary diagnostics.
%**************************************************************************
% UNITS
%   - Geometry and distance are degrees.
%**************************************************************************

%% Section 1: Find Endpoint-Safe Connected Hulls

if ~isa(sweptShape, "polyshape") || ~isscalar(sweptShape)
    error(invalidShapeIdentifier, "sweptShape must be one scalar polyshape.");
end
validateattributes(clusterDistance_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(protectedPoints_deg, {'numeric'}, {'real', 'finite', '2d', 'ncols', 2});
sourceRegions = regions(sweptShape);
record = struct("Distance_deg", clusterDistance_deg, ...
    "SourceRegionCount", numel(sourceRegions), "ClusterGroupCount", 0, ...
    "ClusteredRegionCount", 0, "ClusterBoundary_deg", zeros(0, 2));
clusteredShape = sweptShape;
if clusterDistance_deg == 0 || numel(sourceRegions) < 3
    return;
end
expandedRegions = regions(polybuffer(sweptShape, clusterDistance_deg / 2));
isClustered = false(numel(sourceRegions), 1);
hulls = cell(numel(expandedRegions), 1);
for expandedIndex = 1:numel(expandedRegions)
    isMember = false(numel(sourceRegions), 1);
    for sourceIndex = find(~isClustered).'
        vertices_deg = sourceRegions(sourceIndex).Vertices;
        vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
        isMember(sourceIndex) = any(isinterior( ...
            expandedRegions(expandedIndex), vertices_deg(:, 1), vertices_deg(:, 2)));
    end
    members = find(isMember);
    if numel(members) < 3
        continue;
    end
    vertices_deg = vertcat(sourceRegions(members).Vertices);
    vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
    hullIndex = convhull(vertices_deg(:, 1), vertices_deg(:, 2));
    hull = polyshape(vertices_deg(hullIndex(1:end - 1), :), "Simplify", false);
    if any(obstacleAvoidance.geometry.pointPolygonClearance( ...
            hull, protectedPoints_deg) <= 1e-12)
        continue;
    end
    record.ClusterGroupCount = record.ClusterGroupCount + 1;
    hulls{record.ClusterGroupCount} = hull;
    isClustered(members) = true;
end
if record.ClusterGroupCount == 0
    return;
end
parts = [hulls(1:record.ClusterGroupCount); num2cell(sourceRegions(~isClustered))];
clusteredShape = union([parts{:}]);
hullShape = union([hulls{1:record.ClusterGroupCount}]);
record.ClusteredRegionCount = nnz(isClustered);
record.ClusterBoundary_deg = hullShape.Vertices;
end
