function [seeds, diagnostics] = generateAzElTopologySeeds( ...
        obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [seeds, diagnostics] = azElInternal.generateAzElTopologySeeds( ...
%       obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Generate one direct seed and a bounded deterministic set of coarse
%     obstacle-side or waiting seeds for HS3 initialization.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected obstacle struct array)
%   - initialState, goalState (normalized scalar state structs)
%   - limits (normalized physical limits struct)
%   - options (resolved planner options)
%**************************************************************************
% OUTPUTS
%   - seeds (column struct array)
%       Geometry, normalized time law, source, and duration estimate.
%   - diagnostics (scalar struct)
%       Coarse-grid bounds, resolution, counts, and bounded search trace.
%**************************************************************************
% UNITS
%   - Position and grid resolution are degrees. Duration is seconds.
%**************************************************************************

%% Section 1: Create The Direct Seed

start_deg = initialState.position_deg;
goal_deg = goalPositionAtHorizon(goalState);
if options.AllowAzimuthWrapping
    azimuthTurns = round((start_deg(1) - goal_deg(1)) / 360);
    goal_deg(1) = goal_deg(1) + 360 * azimuthTurns;
end
availableDuration_s = goalState.time_s - initialState.time_s;
directDuration_s = min(availableDuration_s, max(1e-3, ...
    norm(goal_deg - start_deg) / max(limits.maxVelocity_deg_s)));
seedTemplate = struct( ...
    "Index", 0, "Source", "", ...
    "position_deg", zeros(0, 2), "tau", zeros(0, 1), ...
    "EstimatedDuration_s", NaN, "Length_deg", NaN);
directSeed = seedTemplate;
directSeed.Index = 1;
directSeed.Source = "direct";
directSeed.position_deg = [start_deg; goal_deg];
directSeed.tau = [0; 1];
directSeed.EstimatedDuration_s = directDuration_s;
directSeed.Length_deg = norm(goal_deg - start_deg);
seeds = directSeed;
diagnostics = emptyDiagnostics(start_deg, goal_deg);
if options.DirectSeedOnly || options.MaximumSeedCount == 1 || ...
        isempty(obstacles)
    diagnostics.GeneratedSeedCount = numel(seeds);
    return;
end

%% Section 2: Build One Coarse Swept-Geometry Grid

sampleTimes_s = obstacleSampleTimes( ...
    obstacles, initialState.time_s, goalState.time_s);
[azimuthGrid_deg, elevationGrid_deg, gridResolution_deg, ...
    bounds_deg] = buildGrid( ...
    obstacles, start_deg, goal_deg, options);
[azimuthMesh_deg, elevationMesh_deg] = ndgrid( ...
    azimuthGrid_deg, elevationGrid_deg);
occupied = false(size(azimuthMesh_deg));
for sampleTimeIndex = 1:numel(sampleTimes_s)
    sampleTime_s = sampleTimes_s(sampleTimeIndex);
    for obstacleIndex = 1:numel(obstacles)
        shape = azElInternal.obstacleShapeAtTime( ...
            obstacles(obstacleIndex), sampleTime_s);
        if ~isempty(shape.Vertices)
            occupiedAtTime = isinterior( ...
                shape, azimuthMesh_deg(:), elevationMesh_deg(:));
            occupied = occupied | reshape( ...
                occupiedAtTime, size(occupied));
        end
    end
end
[~, startAzimuthIndex] = min(abs(azimuthGrid_deg - start_deg(1)));
[~, startElevationIndex] = min(abs(elevationGrid_deg - start_deg(2)));
[~, goalAzimuthIndex] = min(abs(azimuthGrid_deg - goal_deg(1)));
[~, goalElevationIndex] = min(abs(elevationGrid_deg - goal_deg(2)));
occupied(startAzimuthIndex, startElevationIndex) = false;
occupied(goalAzimuthIndex, goalElevationIndex) = false;
diagnostics.Bounds_deg = bounds_deg;
diagnostics.Resolution_deg = gridResolution_deg;
diagnostics.SampleTimes_s = sampleTimes_s;
diagnostics.NodeCount = numel(occupied);

%% Section 3: Search Both Sides Of The Direct Line

sideModes = [0 1 -1];
for sideModeIndex = 1:numel(sideModes)
    if numel(seeds) >= options.MaximumSeedCount
        break;
    end
    allowed = ~occupied;
    if sideModes(sideModeIndex) ~= 0
        sideValue = signedSide( ...
            azimuthMesh_deg, elevationMesh_deg, start_deg, goal_deg);
        sideTolerance_deg2 = gridResolution_deg * ...
            max(gridResolution_deg, norm(goal_deg - start_deg)) * 0.05;
        if sideModes(sideModeIndex) > 0
            allowed = allowed & sideValue >= -sideTolerance_deg2;
        else
            allowed = allowed & sideValue <= sideTolerance_deg2;
        end
        allowed(startAzimuthIndex, startElevationIndex) = true;
        allowed(goalAzimuthIndex, goalElevationIndex) = true;
    end
    [indexPath, searchRecord] = gridAStar( ...
        azimuthGrid_deg, elevationGrid_deg, allowed, ...
        [startAzimuthIndex, startElevationIndex], ...
        [goalAzimuthIndex, goalElevationIndex]);
    diagnostics.ExpandedCount = diagnostics.ExpandedCount + ...
        searchRecord.ExpandedCount;
    diagnostics.RejectedTransitionCount = ...
        diagnostics.RejectedTransitionCount + ...
        searchRecord.RejectedTransitionCount;
    diagnostics.ExploredNodes_deg = appendBoundedTrace( ...
        diagnostics.ExploredNodes_deg, searchRecord.ExploredNodes_deg, ...
        2000);
    if isempty(indexPath)
        continue;
    end
    route_deg = [ ...
        azimuthGrid_deg(indexPath(:, 1)), ...
        elevationGrid_deg(indexPath(:, 2))];
    route_deg(1, :) = start_deg;
    route_deg(end, :) = goal_deg;
    route_deg = removeCollinearPoints(route_deg);
    if routeDuplicates(route_deg, seeds, gridResolution_deg)
        continue;
    end
    seed = seedTemplate;
    seed.Index = numel(seeds) + 1;
    seed.Source = "coarseTopologyGraph";
    seed.position_deg = route_deg;
    [seed.tau, seed.Length_deg] = routeTau(route_deg);
    seed.EstimatedDuration_s = min(availableDuration_s, max( ...
        directDuration_s, seed.Length_deg / ...
        max(limits.maxVelocity_deg_s)));
    seeds(end + 1, 1) = seed; %#ok<AGROW>
end

%% Section 4: Add One Input-Driven Waiting Variant

if numel(seeds) < options.MaximumSeedCount
    waitFraction = findClearDirectWait( ...
        obstacles, start_deg, goal_deg, initialState.time_s, ...
        goalState.time_s, sampleTimes_s);
    if isfinite(waitFraction) && waitFraction > 0 && waitFraction < 1
        waitSeed = seedTemplate;
        waitSeed.Index = numel(seeds) + 1;
        waitSeed.Source = "directWait";
        waitSeed.position_deg = [start_deg; start_deg; goal_deg];
        waitSeed.tau = [0; waitFraction; 1];
        waitSeed.EstimatedDuration_s = availableDuration_s;
        waitSeed.Length_deg = norm(goal_deg - start_deg);
        seeds(end + 1, 1) = waitSeed;
    end
end

diagnostics.GeneratedSeedCount = numel(seeds);
end

%% Section 5: Local Functions

function [azimuthGrid_deg, elevationGrid_deg, resolution_deg, bounds_deg] = ...
        buildGrid(obstacles, start_deg, goal_deg, options)
% PURPOSE
%   - Bound one coarse grid by inputs and a fixed maximum axis count.
allPosition_deg = [start_deg; goal_deg];
for obstacleIndex = 1:numel(obstacles)
    for sampleIndex = 1:numel(obstacles(obstacleIndex).time_s)
        finiteRows = isfinite(obstacles(obstacleIndex).az_deg{sampleIndex}) & ...
            isfinite(obstacles(obstacleIndex).el_deg{sampleIndex});
        allPosition_deg = [allPosition_deg; ...
            obstacles(obstacleIndex).az_deg{sampleIndex}(finiteRows), ...
            obstacles(obstacleIndex).el_deg{sampleIndex}(finiteRows)]; ...
            %#ok<AGROW>
    end
end
minimum_deg = min(allPosition_deg, [], 1);
maximum_deg = max(allPosition_deg, [], 1);
span_deg = maximum_deg - minimum_deg;
baseResolution_deg = max(0.25, min(2, ...
    max(norm(goal_deg - start_deg), 1) / 18));
padding_deg = max(2 * baseResolution_deg, 0.15 * max(span_deg));
minimum_deg = max(minimum_deg - padding_deg, ...
    [options.AzimuthInterval_deg(1), options.ElevationInterval_deg(1)]);
maximum_deg = min(maximum_deg + padding_deg, ...
    [options.AzimuthInterval_deg(2), options.ElevationInterval_deg(2)]);
maximumAxisCount = 45;
resolution_deg = max([baseResolution_deg, ...
    (maximum_deg - minimum_deg) / (maximumAxisCount - 1)]);
azimuthGrid_deg = gridAxis(minimum_deg(1), maximum_deg(1), ...
    resolution_deg, start_deg(1), goal_deg(1));
elevationGrid_deg = gridAxis(minimum_deg(2), maximum_deg(2), ...
    resolution_deg, start_deg(2), goal_deg(2));
bounds_deg = [azimuthGrid_deg(1), azimuthGrid_deg(end), ...
    elevationGrid_deg(1), elevationGrid_deg(end)];
end

function axis_deg = gridAxis(minimum_deg, maximum_deg, resolution_deg, ...
        start_deg, goal_deg)
% PURPOSE
%   - Create a bounded uniform axis that includes endpoint neighborhoods.
axisCount = max(2, ceil((maximum_deg - minimum_deg) / resolution_deg) + 1);
axis_deg = linspace(minimum_deg, maximum_deg, axisCount).';
axis_deg = unique([axis_deg; start_deg; goal_deg]);
end

function sampleTimes_s = obstacleSampleTimes(obstacles, startTime_s, endTime_s)
% PURPOSE
%   - Use source times and interval midpoints without snapshot event logic.
sampleTimes_s = [startTime_s; endTime_s];
for obstacleIndex = 1:numel(obstacles)
    time_s = obstacles(obstacleIndex).time_s(:);
    midTime_s = (time_s(1:end - 1) + time_s(2:end)) / 2;
    sampleTimes_s = [sampleTimes_s; time_s; midTime_s]; %#ok<AGROW>
end
sampleTimes_s = unique(sampleTimes_s( ...
    sampleTimes_s >= startTime_s & sampleTimes_s <= endTime_s));
end

function sideValue = signedSide(azimuth_deg, elevation_deg, start_deg, goal_deg)
% PURPOSE
%   - Measure deterministic side of the input-defined start-to-goal line.
direction_deg = goal_deg - start_deg;
sideValue = direction_deg(1) * (elevation_deg - start_deg(2)) - ...
    direction_deg(2) * (azimuth_deg - start_deg(1));
end

function [indexPath, record] = gridAStar( ...
        azimuthGrid_deg, elevationGrid_deg, allowed, startIndex, goalIndex)
% PURPOSE
%   - Find one shortest eight-neighbor route on the bounded topology grid.
gridSize = size(allowed);
startLinearIndex = sub2ind(gridSize, startIndex(1), startIndex(2));
goalLinearIndex = sub2ind(gridSize, goalIndex(1), goalIndex(2));
costToCome = Inf(gridSize);
estimatedCost = Inf(gridSize);
parent = zeros(gridSize, "uint32");
open = false(gridSize);
closed = false(gridSize);
costToCome(startLinearIndex) = 0;
estimatedCost(startLinearIndex) = heuristic( ...
    startIndex, goalIndex, azimuthGrid_deg, elevationGrid_deg);
open(startLinearIndex) = true;
expandedCount = 0;
rejectedCount = 0;
exploredNodes_deg = zeros(0, 2);
neighborOffset = [ ...
    -1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];
while any(open, "all")
    openIndex = find(open);
    [~, localIndex] = min(estimatedCost(openIndex));
    currentLinearIndex = openIndex(localIndex);
    open(currentLinearIndex) = false;
    if currentLinearIndex == goalLinearIndex
        break;
    end
    closed(currentLinearIndex) = true;
    expandedCount = expandedCount + 1;
    [currentAzimuthIndex, currentElevationIndex] = ...
        ind2sub(gridSize, currentLinearIndex);
    exploredNodes_deg(end + 1, :) = [ ...
        azimuthGrid_deg(currentAzimuthIndex), ...
        elevationGrid_deg(currentElevationIndex)]; %#ok<AGROW>
    for neighborIndex = 1:size(neighborOffset, 1)
        nextIndex = [currentAzimuthIndex, currentElevationIndex] + ...
            neighborOffset(neighborIndex, :);
        insideGrid = all(nextIndex >= 1) && ...
            nextIndex(1) <= gridSize(1) && ...
            nextIndex(2) <= gridSize(2);
        if ~insideGrid
            rejectedCount = rejectedCount + 1;
            continue;
        end
        nextLinearIndex = sub2ind( ...
            gridSize, nextIndex(1), nextIndex(2));
        if ~allowed(nextLinearIndex) || closed(nextLinearIndex)
            rejectedCount = rejectedCount + 1;
            continue;
        end
        edgeLength_deg = hypot( ...
            azimuthGrid_deg(nextIndex(1)) - ...
            azimuthGrid_deg(currentAzimuthIndex), ...
            elevationGrid_deg(nextIndex(2)) - ...
            elevationGrid_deg(currentElevationIndex));
        trialCost = costToCome(currentLinearIndex) + edgeLength_deg;
        if trialCost < costToCome(nextLinearIndex)
            costToCome(nextLinearIndex) = trialCost;
            parent(nextLinearIndex) = uint32(currentLinearIndex);
            estimatedCost(nextLinearIndex) = trialCost + heuristic( ...
                nextIndex, goalIndex, azimuthGrid_deg, elevationGrid_deg);
            open(nextLinearIndex) = true;
        end
    end
end
indexPath = zeros(0, 2);
if isfinite(costToCome(goalLinearIndex))
    reversePath = goalLinearIndex;
    while reversePath(end) ~= startLinearIndex
        reversePath(end + 1, 1) = ...
            double(parent(reversePath(end))); %#ok<AGROW>
    end
    reversePath = flipud(reversePath(:));
    [azimuthIndex, elevationIndex] = ind2sub(gridSize, reversePath);
    indexPath = [azimuthIndex, elevationIndex];
end
record = struct( ...
    "ExpandedCount", expandedCount, ...
    "RejectedTransitionCount", rejectedCount, ...
    "ExploredNodes_deg", exploredNodes_deg);
end

function value = heuristic(index, goalIndex, azimuthGrid_deg, elevationGrid_deg)
% PURPOSE
%   - Use Euclidean distance as an admissible spatial-grid heuristic.
value = hypot( ...
    azimuthGrid_deg(index(1)) - azimuthGrid_deg(goalIndex(1)), ...
    elevationGrid_deg(index(2)) - elevationGrid_deg(goalIndex(2)));
end

function route_deg = removeCollinearPoints(route_deg)
% PURPOSE
%   - Remove grid samples that do not change route direction.
if size(route_deg, 1) <= 2
    return;
end
firstDelta = diff(route_deg, 1, 1);
turn_deg2 = firstDelta(1:end - 1, 1) .* firstDelta(2:end, 2) - ...
    firstDelta(1:end - 1, 2) .* firstDelta(2:end, 1);
keep = [true; abs(turn_deg2) > 1e-10; true];
route_deg = route_deg(keep, :);
end

function duplicate = routeDuplicates(route_deg, seeds, resolution_deg)
% PURPOSE
%   - Reject geometrically indistinguishable routes at grid resolution.
duplicate = false;
sampleTau = linspace(0, 1, 101).';
[parameterizedTau, ~] = routeTau(route_deg);
sampledRoute_deg = interp1( ...
    parameterizedTau, route_deg, sampleTau, "linear");
for seedIndex = 1:numel(seeds)
    sampledSeed_deg = interp1( ...
        seeds(seedIndex).tau, seeds(seedIndex).position_deg, ...
        sampleTau, "linear");
    if max(vecnorm(sampledRoute_deg - sampledSeed_deg, 2, 2)) <= ...
            0.5 * resolution_deg
        duplicate = true;
        return;
    end
end
end

function [tau, length_deg] = routeTau(route_deg)
% PURPOSE
%   - Parameterize a route by normalized cumulative Euclidean length.
cumulativeLength_deg = [0; cumsum(vecnorm(diff(route_deg, 1, 1), 2, 2))];
length_deg = cumulativeLength_deg(end);
if length_deg <= 0
    tau = linspace(0, 1, size(route_deg, 1)).';
else
    tau = cumulativeLength_deg / length_deg;
end
end

function waitFraction = findClearDirectWait(obstacles, start_deg, goal_deg, ...
        startTime_s, endTime_s, candidateTimes_s)
% PURPOSE
%   - Add waiting only when a source-time delay clears the direct time law.
duration_s = endTime_s - startTime_s;
candidateWait_s = unique([0; candidateTimes_s(:) - startTime_s]);
candidateWait_s = candidateWait_s( ...
    candidateWait_s >= 0 & candidateWait_s < duration_s);
sampleTau = linspace(0, 1, 61).';
sampleTime_s = startTime_s + sampleTau * duration_s;
waitFraction = NaN;
for waitIndex = 1:numel(candidateWait_s)
    wait_s = candidateWait_s(waitIndex);
    progress = max(0, sampleTime_s - startTime_s - wait_s) / ...
        max(duration_s - wait_s, eps);
    position_deg = start_deg + progress .* (goal_deg - start_deg);
    occupied = queryAzElTimeObstacle( ...
        obstacles, position_deg(:, 1), position_deg(:, 2), sampleTime_s);
    if ~any(occupied)
        waitFraction = wait_s / duration_s;
        return;
    end
end
end

function values = appendBoundedTrace(values, additions, maximumCount)
% PURPOSE
%   - Retain a deterministic prefix while preserving complete counts.
remainingCount = maximumCount - size(values, 1);
if remainingCount > 0
    values = [values; additions(1:min(remainingCount, ...
        size(additions, 1)), :)];
end
end

function position_deg = goalPositionAtHorizon(goalState)
% PURPOSE
%   - Select the fixed or sampled target position at the planning horizon.
if isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s)
    position_deg = interp1( ...
        goalState.targetTime_s, goalState.targetPosition_deg, ...
        goalState.time_s, goalState.InterpolationMethod);
else
    position_deg = goalState.position_deg;
end
end

function diagnostics = emptyDiagnostics(start_deg, goal_deg)
% PURPOSE
%   - Define stable bounded search diagnostics before a grid is required.
diagnostics = struct( ...
    "Bounds_deg", [NaN NaN NaN NaN], ...
    "Resolution_deg", NaN, ...
    "SampleTimes_s", zeros(0, 1), ...
    "NodeCount", 0, ...
    "ExpandedCount", 0, ...
    "RejectedTransitionCount", 0, ...
    "GeneratedSeedCount", 1, ...
    "ExploredNodes_deg", zeros(0, 2), ...
    "Start_deg", start_deg, ...
    "Goal_deg", goal_deg, ...
    "TraceDownsampleRule", "First 2000 expanded nodes in search order");
end
