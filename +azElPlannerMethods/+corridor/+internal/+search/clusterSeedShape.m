function [clusteredShape, record] = clusterSeedShape(sweptShape, clusterDistance_deg, protectedPoints_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [clusteredShape, record] = azElPlannerMethods.corridor.internal.search.clusterSeedShape( ...
%       sweptShape, clusterDistance_deg, protectedPoints_deg)
%**************************************************************************
% PURPOSE
%   - Merge groups of nearby swept regions into conservative hulls used only
%     to reduce seed-search complexity. Endpoint guards prevent a merge from
%     erasing the start or goal's apparent free-space component.
%**************************************************************************
% INPUTS
%   - sweptShape (scalar polyshape), protected swept obstacle geometry.
%   - clusterDistance_deg (nonnegative scalar), maximum merge distance.
%   - protectedPoints_deg (N-by-2 array), points clustering may not capture.
%**************************************************************************
% OUTPUTS
%   - clusteredShape (scalar polyshape), seed-only reduced geometry.
%   - record (scalar struct), source and retained cluster diagnostics.
%**************************************************************************
% UNITS
%   - Geometry and cluster distance are degrees.
%**************************************************************************

%% Section 1: Validate Inputs & Initialize Diagnostics

if ~isa(sweptShape, "polyshape") || ~isscalar(sweptShape)
    error("azElInternal:clusterSeedShape:InvalidShape", "sweptShape must be one scalar polyshape.");
end
validateattributes(clusterDistance_deg, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(protectedPoints_deg, {'numeric'}, {'real', 'finite', '2d', 'ncols', 2});
sourceRegions = regions(sweptShape);
sourceRegionCount = numel(sourceRegions);
record = struct( ...
    "Distance_deg", double(clusterDistance_deg), ...
    "SourceRegionCount", sourceRegionCount, ...
    "ClusterGroupCount", 0, "ClusteredRegionCount", 0, "ClusterBoundary_deg", zeros(0, 2));
clusteredShape = sweptShape;
if clusterDistance_deg == 0 || sourceRegionCount < 3
    return;
end

%% Section 2: Find Connected Groups Of Nearby Regions

% Buffering by half the requested distance turns geometric proximity into
% connected components. A group needs at least three source regions; smaller
% groups are left exact because merging them offers little graph reduction.
expandedShape = polybuffer(sweptShape, clusterDistance_deg / 2);
expandedRegions = regions(expandedShape);
regionIsClustered = false(sourceRegionCount, 1);
clusterShapes = cell(numel(expandedRegions), 1);
clusterGroupCount = 0;

% Treat each connected buffered region as one possible proximity cluster.
for expandedRegionIndex = 1:numel(expandedRegions)
    isMember = false(sourceRegionCount, 1);

    % Determine which unassigned source regions contributed to this buffered component.
    for sourceRegionIndex = 1:sourceRegionCount
        if regionIsClustered(sourceRegionIndex)
            continue;
        end
        vertices_deg = sourceRegions(sourceRegionIndex).Vertices;
        vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
        isMember(sourceRegionIndex) = any(isinterior( ...
            expandedRegions(expandedRegionIndex), vertices_deg(:, 1), vertices_deg(:, 2)));
    end
    memberRegionIndices = find(isMember);
    if numel(memberRegionIndices) < 3
        continue;
    end
    memberVertices = cell(numel(memberRegionIndices), 1);

    % Gather complete member boundaries before computing their conservative hull.
    for memberIndex = 1:numel(memberRegionIndices)
        vertices_deg = sourceRegions( memberRegionIndices(memberIndex)).Vertices;
        memberVertices{memberIndex} = vertices_deg( all(isfinite(vertices_deg), 2), :);
    end
    memberVertices_deg = vertcat(memberVertices{:});
    hullIndex = convhull( memberVertices_deg(:, 1), memberVertices_deg(:, 2));
    hullShape = polyshape( memberVertices_deg(hullIndex, 1), memberVertices_deg(hullIndex, 2), "Simplify", false);
    hullContainsProtectedPoint = false;

    % Reject a merge that would place the start or goal inside the reduced obstacle hull.
    for pointIndex = 1:size(protectedPoints_deg, 1)
        clearance_deg = azElInternal.geometry.pointPolygonClearance( hullShape, protectedPoints_deg(pointIndex, :));
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

% Replace only accepted groups and preserve every unclustered source region.
% This geometry may remove seed choices, but it is never used for final motion
% validation or for claiming continuous collision freedom.
if clusterGroupCount == 0
    return;
end
unclusteredRegionIndices = find(~regionIsClustered);
partCount = clusterGroupCount + numel(unclusteredRegionIndices);
shapeParts = cell(partCount, 1);
shapeParts(1:clusterGroupCount) = clusterShapes(1:clusterGroupCount);

% Append every region that was deliberately left at its original geometry.
for unclusteredIndex = 1:numel(unclusteredRegionIndices)
    shapeParts{clusterGroupCount + unclusteredIndex} = sourceRegions(unclusteredRegionIndices(unclusteredIndex));
end
clusteredShape = union([shapeParts{:}]);
clusterHulls = union([clusterShapes{1:clusterGroupCount}]);
record.ClusterGroupCount = clusterGroupCount;
record.ClusteredRegionCount = nnz(regionIsClustered);
record.ClusterBoundary_deg = clusterHulls.Vertices;
end
