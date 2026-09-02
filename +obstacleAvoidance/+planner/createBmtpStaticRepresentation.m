function representation = createBmtpStaticRepresentation( ...
        obstacles, startTime_s, endTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   representation = ...
%       obstacleAvoidance.planner.createBmtpStaticRepresentation( ...
%       obstacles, startTime_s, endTime_s)
%**************************************************************************
% PURPOSE
%   - Create one request-owned static exclusion representation for all BMTP
%     seed solves over the same authoritative obstacle geometry.
%**************************************************************************
% INPUTS
%   - obstacles (canonical or prepared obstacle struct array)
%       Protected geometry must remain static over the request horizon.
%   - startTime_s, endTime_s (finite numeric scalars)
%       Inclusive request horizon with endTime_s not before startTime_s.
%**************************************************************************
% OUTPUTS
%   - representation (scalar struct)
%       Source-derived convex regions, conservative grouping evidence, and
%       horizon metadata. Independent public validation remains authoritative.
%**************************************************************************
% UNITS
%   - Position and boundary coordinates are degrees; time is seconds.
%**************************************************************************

%% Section 1: Normalize The Static Request

validateattributes(startTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
validateattributes(endTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', '>=', startTime_s});
if isempty(obstacles) || ~isfield(obstacles, "InternalPreparation")
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
end
obstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacles);
[hasStaticHorizon, occupiedShape] = ...
    obstacleAvoidance.obstacles.queryStaticHorizon( ...
    obstacles, startTime_s, endTime_s);
if ~hasStaticHorizon
    error("createBmtpStaticRepresentation:UnsupportedDynamicObstacle", ...
        "Every obstacle must be static and active over the horizon.");
end

%% Section 2: Decompose The Authoritative Occupied Shape Once

[exactRegions_deg, coverage] = createExactRegions( ...
    occupiedShape, numel(obstacles));
[regions_deg, grouping] = createSolverRegions(exactRegions_deg);
coverage.SolverRegionCount = numel(regions_deg);
coverage.ConservativeGrouping = grouping;

%% Section 3: Assemble Reusable Source-Derived Evidence

representation = struct( ...
    "StartTime_s", startTime_s, ...
    "EndTime_s", endTime_s, ...
    "ObstacleCount", numel(obstacles), ...
    "Regions_deg", {regions_deg}, ...
    "Coverage", coverage);
end

%% Section 4: Local Functions

function [regions_deg, coverage] = createExactRegions( ...
        occupiedShape, obstacleCount)
% Decompose the protected union; public validation owns final acceptance.
exactRegions = obstacleAvoidance.geometry.convexPolygonRegions(occupiedShape);
regions_deg = cell(numel(exactRegions), 1);
for regionIndex = 1:numel(exactRegions)
    vertices_deg = exactRegions(regionIndex).Vertices;
    regions_deg{regionIndex} = ...
        vertices_deg(all(isfinite(vertices_deg), 2), :);
end
coverage = struct( ...
    "Passed", obstacleCount == 0 || ~isempty(regions_deg), ...
    "ObstacleCount", obstacleCount, ...
    "RegionCount", numel(regions_deg), ...
    "ExactRegionCount", numel(regions_deg), ...
    "AuthoritativeCoverageCheck", "publicValidation");
end

function [groupedRegions_deg, record] = createSolverRegions(regions_deg)
% Bound separator work with conservative hulls only for complex outlines.
maximumExactRegionCount = 64;
targetGroupCount = 8;
regionCount = numel(regions_deg);
record = struct( ...
    "Applied", false, "ExactRegionCount", regionCount, ...
    "SolverRegionCount", regionCount, ...
    "MaximumExactRegionCount", maximumExactRegionCount, ...
    "TargetGroupCount", targetGroupCount, ...
    "RelationToExactGeometry", "equal", ...
    "GroupMemberIndices", {num2cell((1:regionCount).')});
groupedRegions_deg = regions_deg;
if regionCount <= maximumExactRegionCount
    return;
end

centroid_deg = zeros(regionCount, 2);
for regionIndex = 1:regionCount
    centroid_deg(regionIndex, :) = mean(regions_deg{regionIndex}, 1);
end
groups = cell(targetGroupCount, 1);
groups{1} = (1:regionCount).';
activeGroupCount = 1;
while activeGroupCount < targetGroupCount
    groupSizes = zeros(activeGroupCount, 1);
    for groupIndex = 1:activeGroupCount
        groupSizes(groupIndex) = numel(groups{groupIndex});
    end
    groupSizes(groupSizes < 2) = 0;
    [largestGroupSize, splitGroupIndex] = max(groupSizes);
    if largestGroupSize < 2
        break;
    end
    memberIndex = groups{splitGroupIndex};
    spread_deg = max(centroid_deg(memberIndex, :), [], 1) - ...
        min(centroid_deg(memberIndex, :), [], 1);
    [~, splitAxisIndex] = max(spread_deg);
    ordering = sortrows( ...
        [centroid_deg(memberIndex, splitAxisIndex), memberIndex], [1 2]);
    middleIndex = floor(numel(memberIndex) / 2);
    groups{splitGroupIndex} = ordering(1:middleIndex, 2);
    activeGroupCount = activeGroupCount + 1;
    groups{activeGroupCount} = ordering(middleIndex + 1:end, 2);
end
groups = groups(1:activeGroupCount);
firstRegionIndex = zeros(activeGroupCount, 1);
for groupIndex = 1:activeGroupCount
    firstRegionIndex(groupIndex) = min(groups{groupIndex});
end
[~, groupOrder] = sort(firstRegionIndex);
groups = groups(groupOrder);
groupedRegions_deg = cell(activeGroupCount, 1);
for groupIndex = 1:activeGroupCount
    vertices_deg = vertcat(regions_deg{groups{groupIndex}});
    hullIndex = convhull(vertices_deg(:, 1), vertices_deg(:, 2));
    groupedRegions_deg{groupIndex} = ...
        vertices_deg(hullIndex(1:end - 1), :);
end
record.Applied = true;
record.SolverRegionCount = activeGroupCount;
record.RelationToExactGeometry = "conservativeSuperset";
record.GroupMemberIndices = groups;
end
