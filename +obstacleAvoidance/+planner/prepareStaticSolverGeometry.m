function geometry = prepareStaticSolverGeometry(obstacles, startTime_s, endTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   geometry = prepareStaticSolverGeometry(obstacles, startTime_s, endTime_s)
% PURPOSE
%   Prepare a static exclusion representation once for all path guesses.
% INPUTS
%   Prepared obstacles and the inclusive planning horizon.
% OUTPUTS
%   Exact regions, solver regions, grouping, and coverage provenance.
% UNITS
%   Geometry is coordinate units; times are seconds.

%% Section 1: Prepare The Exclusion Regions
[obstaclesRemainStatic, occupiedShape] = obstacleAvoidance.obstacles.queryStaticHorizon(obstacles, startTime_s, endTime_s);
if ~obstaclesRemainStatic
    error("solveStaticBmtpTrajectory:UnsupportedDynamicObstacle", "Every obstacle must be static and active over the horizon.");
end
[exactRegions_units, coverage] = createExactRegions(occupiedShape, numel(obstacles));
[regions_units, grouping]      = createSolverRegions(exactRegions_units);
coverage.SolverRegionCount    = numel(regions_units);
coverage.ConservativeGrouping = grouping;

geometry = struct("ExactRegions_units", {exactRegions_units}, ...
    "Regions_units", {regions_units}, "Grouping", grouping, "Coverage", coverage);
end

%% Section 2: Local Functions
function [regions_units, coverage] = createExactRegions(occupiedShape, obstacleCount)
    % Split protected geometry into convex regions.
    exactRegions = obstacleAvoidance.geometry.convexPolygonRegions(occupiedShape);
    regions_units  = cell(numel(exactRegions), 1);
    % Process each geometric region while constructing or checking the region topology.
    for regionIndex = 1:numel(exactRegions)
        vertices_units = exactRegions(regionIndex).Vertices;
        regions_units{regionIndex} = vertices_units(all(isfinite(vertices_units), 2), :);
    end
    coverage = struct("Passed", obstacleCount == 0 || ~isempty(regions_units), ...
        "ObstacleCount", obstacleCount, ...
        "RegionCount", numel(regions_units), ...
        "ExactRegionCount", numel(regions_units), ...
        "AuthoritativeCoverageCheck", "publicValidation");
end

function [groupedRegions_units, record] = createSolverRegions(regions_units)
    % Group complex outlines to limit separating-plane solves.
    maximumExactRegionCount = 64;
    targetGroupCount        = 8;
    regionCount             = numel(regions_units);
    record                  = struct("Applied", false, "ExactRegionCount", regionCount, ...
        "SolverRegionCount", regionCount, ...
        "MaximumExactRegionCount", maximumExactRegionCount, ...
        "TargetGroupCount", targetGroupCount, ...
        "RelationToExactGeometry", "equal", ...
        "GroupMemberIndices", {num2cell((1:regionCount).')});
    groupedRegions_units = regions_units;
    if regionCount <= maximumExactRegionCount
        return;
    end

    centroid_units = zeros(regionCount, 2);
    % Process each geometric region while constructing or checking the region topology.
    for regionIndex = 1:regionCount
        centroid_units(regionIndex, :) = mean(regions_units{regionIndex}, 1);
    end
    groups = cell(targetGroupCount, 1);
    groups{1} = (1:regionCount).';
    activeGroupCount = 1;
    % Merge compatible region groups until the requested group count is reached.
    while activeGroupCount < targetGroupCount
        groupSizes = zeros(activeGroupCount, 1);
        % Process each geometric group while constructing or checking the region topology.
        for groupIndex = 1:activeGroupCount
            groupSizes(groupIndex) = numel(groups{groupIndex});
        end
        groupSizes(groupSizes < 2) = 0;
        [largestGroupSize, splitGroupIndex] = max(groupSizes);
        if largestGroupSize < 2
            break;
        end
        memberIndex = groups{splitGroupIndex};
        spread_units  = max(centroid_units(memberIndex, :), [], 1) - min(centroid_units(memberIndex, :), [], 1);
        [~, splitAxisIndex] = max(spread_units);
        ordering    = sortrows([centroid_units(memberIndex, splitAxisIndex), memberIndex], [1 2]);
        middleIndex = floor(numel(memberIndex) / 2);
        groups{splitGroupIndex} = ordering(1:middleIndex, 2);
        activeGroupCount = activeGroupCount + 1;
        groups{activeGroupCount} = ordering(middleIndex + 1:end, 2);
    end
    groups           = groups(1:activeGroupCount);
    firstRegionIndex = zeros(activeGroupCount, 1);
    % Process each geometric group while constructing or checking the region topology.
    for groupIndex = 1:activeGroupCount
        firstRegionIndex(groupIndex) = min(groups{groupIndex});
    end
    [~, groupOrder] = sort(firstRegionIndex);
    groups             = groups(groupOrder);
    groupedRegions_units = cell(activeGroupCount, 1);
    % Process each geometric group while constructing or checking the region topology.
    for groupIndex = 1:activeGroupCount
        vertices_units = vertcat(regions_units{groups{groupIndex}});
        hullIndex    = convhull(vertices_units(:, 1), vertices_units(:, 2));
        groupedRegions_units{groupIndex} = vertices_units(hullIndex(1:end - 1), :);
    end
    record.Applied                 = true;
    record.SolverRegionCount       = activeGroupCount;
    record.RelationToExactGeometry = "conservativeSuperset";
    record.GroupMemberIndices      = groups;
end
