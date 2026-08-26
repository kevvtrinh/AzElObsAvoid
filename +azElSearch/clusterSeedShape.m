function [clusteredShape, record] = clusterSeedShape( ...
        sweptShape, clusterDistance_deg, protectedPoints_deg, ...
        invalidShapeIdentifier)
%% Section 0: Header & Readme
% SYNTAX
%   [clusteredShape, record] = azElSearch.clusterSeedShape( ...
%       sweptShape, clusterDistance_deg, protectedPoints_deg, ...
%       invalidShapeIdentifier)
%**************************************************************************
% PURPOSE
%   - Replace nearby groups of at least three swept obstacle regions with
%     conservative convex hulls for topology-seed generation only.
%**************************************************************************
% INPUTS
%   - sweptShape (scalar polyshape)
%       Union of protected obstacle geometry sampled across planning time.
%   - clusterDistance_deg (nonnegative numeric scalar)
%       Maximum connected region gap. Zero disables clustering.
%   - protectedPoints_deg (N-by-2 finite numeric array)
%       Start and goal points that a cluster hull must not contain.
%   - invalidShapeIdentifier (string scalar)
%       Caller-owned compatibility identifier for an invalid swept shape.
%**************************************************************************
% OUTPUTS
%   - clusteredShape (scalar polyshape)
%       Seed-only geometry. Physical obstacle records are not changed.
%   - record (scalar struct)
%       Distance, region counts, group counts, and cluster boundaries.
%**************************************************************************
% UNITS
%   - Polygon vertices, distances, and protected points are degrees.
%**************************************************************************

%% Section 1: Validate Inputs & Initialize Diagnostics

if ~isa(sweptShape, "polyshape") || ~isscalar(sweptShape)
    error(invalidShapeIdentifier, ...
        "sweptShape must be one scalar polyshape.");
end
validateattributes(clusterDistance_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(protectedPoints_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2});
sourceRegions = regions(sweptShape);
sourceRegionCount = numel(sourceRegions);
record = struct( ...
    "Distance_deg", double(clusterDistance_deg), ...
    "SourceRegionCount", sourceRegionCount, ...
    "ClusterGroupCount", 0, ...
    "ClusteredRegionCount", 0, ...
    "ClusterBoundary_deg", zeros(0, 2));
clusteredShape = sweptShape;
if clusterDistance_deg == 0 || sourceRegionCount < 3
    return;
end

%% Section 2: Find Connected Groups Of Nearby Regions

expandedShape = polybuffer(sweptShape, clusterDistance_deg / 2);
expandedRegions = regions(expandedShape);
regionIsClustered = false(sourceRegionCount, 1);
clusterShapes = cell(numel(expandedRegions), 1);
clusterGroupCount = 0;

% Examine each connected expanded component as one possible conservative
% cluster group.
for expandedRegionIndex = 1:numel(expandedRegions)
    isMember = false(sourceRegionCount, 1);

    % Test every source region that has not already been committed to an
    % earlier cluster against this expanded component.
    for sourceRegionIndex = 1:sourceRegionCount
        if regionIsClustered(sourceRegionIndex)
            continue;
        end
        vertices_deg = sourceRegions(sourceRegionIndex).Vertices;
        vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
        isMember(sourceRegionIndex) = any(isinterior( ...
            expandedRegions(expandedRegionIndex), ...
            vertices_deg(:, 1), vertices_deg(:, 2)));
    end
    memberRegionIndices = find(isMember);
    if numel(memberRegionIndices) < 3
        continue;
    end
    memberVertices = cell(numel(memberRegionIndices), 1);

    % Collect finite vertices before forming their conservative convex hull.
    for memberIndex = 1:numel(memberRegionIndices)
        vertices_deg = sourceRegions( ...
            memberRegionIndices(memberIndex)).Vertices;
        memberVertices{memberIndex} = vertices_deg( ...
            all(isfinite(vertices_deg), 2), :);
    end
    memberVertices_deg = vertcat(memberVertices{:});
    hullIndex = convhull( ...
        memberVertices_deg(:, 1), memberVertices_deg(:, 2));
    hullShape = polyshape( ...
        memberVertices_deg(hullIndex, 1), ...
        memberVertices_deg(hullIndex, 2), "Simplify", false);
    hullContainsProtectedPoint = false;

    % Reject a hull that would cover either protected endpoint.
    for pointIndex = 1:size(protectedPoints_deg, 1)
        clearance_deg = azElGeometry.pointPolygonClearance( ...
            hullShape, protectedPoints_deg(pointIndex, :));
        if clearance_deg <= 1e-12
            hullContainsProtectedPoint = true;
            break;
        end
    end
    if hullContainsProtectedPoint
        continue;
    end
    clusterGroupCount = clusterGroupCount + 1;
    clusterShapes{clusterGroupCount} = hullShape;
    regionIsClustered(memberRegionIndices) = true;
end

%% Section 3: Assemble The Conservative Seed Geometry

if clusterGroupCount == 0
    return;
end
unclusteredRegionIndices = find(~regionIsClustered);
partCount = clusterGroupCount + numel(unclusteredRegionIndices);
shapeParts = cell(partCount, 1);
shapeParts(1:clusterGroupCount) = clusterShapes(1:clusterGroupCount);

% Retain every region that was not absorbed into a cluster.
for unclusteredIndex = 1:numel(unclusteredRegionIndices)
    shapeParts{clusterGroupCount + unclusteredIndex} = sourceRegions( ...
        unclusteredRegionIndices(unclusteredIndex));
end
clusteredShape = union([shapeParts{:}]);
clusterHulls = union([clusterShapes{1:clusterGroupCount}]);
record.ClusterGroupCount = clusterGroupCount;
record.ClusteredRegionCount = nnz(regionIsClustered);
record.ClusterBoundary_deg = clusterHulls.Vertices;
end
