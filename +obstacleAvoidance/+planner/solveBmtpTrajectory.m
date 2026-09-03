function [candidate, diagnostics] = solveBmtpTrajectory( ...
        seed, obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.solveBmtpTrajectory( ...
%       seed, obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Adapt one static obstacle-planner seed to the independent BMTP engine.
%   - Own protected-geometry coverage while the engine owns trajectory math.
%**************************************************************************
% INPUTS
%   - seed (scalar struct)
%       position_deg is N-by-2 and tau increases from zero through one.
%   - obstacles (canonical or prepared obstacle struct array)
%       Protected geometry must remain static over the request horizon.
%   - initialState, goalState, limits, options (resolved scalar structs)
%       Normalized planner request and fully resolved planner options.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       BMTP motion or stable expected-failure record for public validation.
%   - diagnostics (scalar struct)
%       Engine timing, convergence, motion, and plane-certificate evidence.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivatives use deg/s,
%     deg/s^2, and deg/s^3. Histories are N-by-2.
%**************************************************************************

%% Section 1: Create The Static Exclusion Representation

if isempty(obstacles) || ~isfield(obstacles, "InternalPreparation")
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
end
obstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacles);
[hasStaticHorizon, occupiedShape] = ...
    obstacleAvoidance.obstacles.queryStaticHorizon( ...
    obstacles, initialState.time_s, goalState.time_s);
if ~hasStaticHorizon
    error("solveBmtpTrajectory:UnsupportedDynamicObstacle", ...
        "Every obstacle must be static and active over the horizon.");
end
[exactRegions_deg, coverage] = createExactRegions( ...
    occupiedShape, numel(obstacles));
[regions_deg, grouping] = createSolverRegions(exactRegions_deg);
coverage.SolverRegionCount = numel(regions_deg);
coverage.ConservativeGrouping = grouping;

%% Section 2: Generate The Motion In The Independent Engine

[candidate, diagnostics] = bmtpEngine.solve( ...
    seed, regions_deg, coverage, initialState, goalState, limits, options);
fallback = struct( ...
    "Attempted", false, ...
    "PrimaryTerminationReason", candidate.TerminationReason, ...
    "Outcome", "notApplicable", ...
    "ExactRegionCount", numel(exactRegions_deg), ...
    "PrimarySolverDiagnostics", struct());
if grouping.Applied
    fallback.Outcome = "groupedAttemptAccepted";
end
if grouping.Applied && ~candidate.Success
    % Conservative hulls are useful only as a first attempt. They can bridge
    % free gaps, so a grouped failure cannot reject the exact request.
    fallback.Attempted = true;
    fallback.Outcome = "exactRegionAttemptFailed";
    fallback.PrimarySolverDiagnostics = diagnostics;
    exactCoverage = coverage;
    exactCoverage.SolverRegionCount = numel(exactRegions_deg);
    exactGrouping = grouping;
    exactGrouping.Applied = false;
    exactGrouping.SolverRegionCount = numel(exactRegions_deg);
    exactGrouping.RelationToExactGeometry = "equal";
    exactGrouping.GroupMemberIndices = ...
        num2cell((1:numel(exactRegions_deg)).');
    exactCoverage.ConservativeGrouping = exactGrouping;
    [candidate, diagnostics] = bmtpEngine.solve( ...
        seed, exactRegions_deg, exactCoverage, initialState, goalState, ...
        limits, options);
    if candidate.Success
        fallback.Outcome = "exactRegionAttemptAccepted";
    end
end
diagnostics.ExactRegionFallback = fallback;
candidate.SolverDiagnostics = diagnostics;
end

%% Section 3: Local Functions

function [regions_deg, coverage] = createExactRegions( ...
        occupiedShape, obstacleCount)
% Decompose the protected union; public validation owns coverage acceptance.
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
