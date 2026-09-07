function [route_units, routeTime_s, record] = timeExpandedVisibilitySearch(nodePosition_units, edgeCost_units, obstacles, initialState, goalState, limits, sampleTimes_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [route_units, routeTime_s, record] = ...
%       obstacleAvoidance.search.timeExpandedVisibilitySearch(nodePosition_units, ...
%       edgeCost_units, obstacles, initialState, goalState, limits, sampleTimes_s, options)
%
% PURPOSE
%   - Search forward reachability using waits and moving edges at every
%     supplied planning time.
%
% INPUTS
%   - nodePosition_units (N-by-2 numeric matrix)
%       Nodes with start first and goal second.
%   - edgeCost_units (N-by-N numeric matrix)
%       Finite entries enable motion edges.
%   - obstacles (canonical protected obstacle struct array)
%   - initialState, goalState, limits, options (scalar structs)
%   - sampleTimes_s (numeric vector)
%       Candidate times retained exactly as temporal search layers.
%
% OUTPUTS
%   - route_units (M-by-2 numeric matrix), routeTime_s (M-by-1 numeric vector)
%       Selected timed route, or documented empty arrays on exhaustion.
%   - record (scalar struct)
%       Search counts, frontier, and best partial ancestry.
%
% UNITS
%   - Position and edge cost are coordinate units; time is seconds.
%
%% Section 1: Propagate The Reachability Frontier
layerTimes_s = unique([initialState.time_s; sampleTimes_s(:); goalState.time_s]);
layerTimes_s = layerTimes_s(layerTimes_s >= initialState.time_s & layerTimes_s <= goalState.time_s);
layerCount   = numel(layerTimes_s);
nodeCount    = size(nodePosition_units, 1);
[geometryTimes_s, stationaryTimeCell] = stationaryGeometryCells(obstacles);
% Cache unknown/free/occupied as 0/1/2 within 300 MiB. Eviction only repeats
% authoritative queries; it never removes a search candidate.
bytesPerGeometry = 13 * nodeCount ^ 2;
maximumCacheBytes = 300 * 1024 ^ 2;
batchPositions_units = zeros(0, 2);
batchPointIndices = zeros(0, 1);
hasStationarySpan = any(stationaryTimeCell(3:2:end - 2));
if hasStationarySpan && 24 * bytesPerGeometry <= maximumCacheBytes / 2
    % Every edge uses the same 13 spatial fractions regardless of its clock.
    % Keep exact arithmetic and merge only numerically identical positions.
    [firstNode, secondNode] = ndgrid(1:nodeCount, 1:nodeCount);
    firstPosition_units = nodePosition_units(firstNode(:), :);
    secondPosition_units = nodePosition_units(secondNode(:), :);
    batchPositions_units = zeros(bytesPerGeometry, 2);
    fractions = linspace(0, 1, 13);
    for fractionIndex = 1:13
        batchIndices = (fractionIndex - 1) * nodeCount ^ 2 + (1:nodeCount ^ 2);
        batchPositions_units(batchIndices, :) = firstPosition_units + fractions(fractionIndex) .* (secondPosition_units - firstPosition_units);
    end
    [batchPositions_units, ~, batchPointIndices] = unique(batchPositions_units, 'rows');
end
lookupBytes = 8 * (numel(batchPositions_units) + numel(batchPointIndices));
cacheSlotCount = min(numel(stationaryTimeCell), floor((maximumCacheBytes - lookupBytes) / max(1, bytesPerGeometry)));
occupancyCache = cell(cacheSlotCount, 1);
occupancyCacheKey = zeros(cacheSlotCount, 1);
nodeIsFree   = false(layerCount, nodeCount);
% Process each layer needed to complete time expanded visibility search.
for layerIndex = 1:layerCount
    nodeIsFree(layerIndex, :) = ~obstacleAvoidance.obstacles. queryPreparedObstacles(obstacles, nodePosition_units(:, 1), nodePosition_units(:, 2), repmat(layerTimes_s(layerIndex), nodeCount, 1)).';
end
waitIsClear = false(max(0, layerCount - 1), nodeCount);
% Process each layer needed to complete time expanded visibility search.
for layerIndex = 1:layerCount - 1
    candidateNodeIndices = find(nodeIsFree(layerIndex, :) & nodeIsFree(layerIndex + 1, :));
    % Test a stationary wait only at nodes that are free in both adjacent layers; all other waits remain unavailable.
    if ~isempty(candidateNodeIndices)
        waitIsClear(layerIndex, candidateNodeIndices) = edgeIsClear(candidateNodeIndices, candidateNodeIndices, layerTimes_s(layerIndex), layerTimes_s(layerIndex + 1));
    end
end

% Within a clear-wait interval, an earlier arrival can wait to match a later one.
isWaitComponentStart = nodeIsFree;
isWaitComponentStart(2:end, :) = nodeIsFree(2:end, :) & ~waitIsClear;
waitComponentFinalLayerIndex = repmat(uint32((1:layerCount).'), 1, nodeCount);
% Process each layer needed to complete time expanded visibility search.
for layerIndex = layerCount - 1:-1:1
    continuingNodeIndices = find(waitIsClear(layerIndex, :));
    waitComponentFinalLayerIndex(layerIndex, continuingNodeIndices) = waitComponentFinalLayerIndex(layerIndex + 1, continuingNodeIndices);
end
motionEdgeExists = isfinite(edgeCost_units);
motionEdgeExists(1:nodeCount + 1:end) = false;
minimumEdgeDuration_s = zeros(nodeCount, nodeCount);
motionEdgeLengths_units = zeros(nodeCount, nodeCount);
for sourceIndex = 1:nodeCount
    displacement_units = nodePosition_units - nodePosition_units(sourceIndex, :);
    minimumEdgeDuration_s(sourceIndex, :) = max(abs(displacement_units) ./ limits.maxVelocity_units_s, [], 2).';
    for targetIndex = reshape(find(motionEdgeExists(sourceIndex, :)), 1, [])
        motionEdgeLengths_units(sourceIndex, targetIndex) = norm(displacement_units(targetIndex, :));
    end
end
reachable        = false(layerCount, nodeCount);
spatialCost_units  = Inf(layerCount, nodeCount);
parentLayerIndex = zeros(layerCount, nodeCount, "uint32");
parentNodeIndex  = zeros(layerCount, nodeCount, "uint16");
reachable(1, 1) = nodeIsFree(1, 1);
spatialCost_units(1, 1) = 0;
[waitCount, motionCount, rejectedCount, expandedCount] = deal(0);
exploredNodes_units = zeros(0, 2);
% Process each layer needed to complete time expanded visibility search.
for layerIndex = 1:layerCount - 1
    % Continue searching only until the first reachable goal layer in earliest-arrival mode; fixed-arrival mode must evaluate its prescribed horizon.
    if options.GoalTimeMode == "earliestArrival" && reachable(layerIndex, 2)
        break;
    end
    currentNodeIndices          = find(reachable(layerIndex, :));
    % Process each current node needed to complete time expanded visibility search.
    for currentNodeIndex = reshape(currentNodeIndices, 1, [])
        expandedCount = expandedCount + 1;
        exploredNodes_units(end + 1, :) = nodePosition_units(currentNodeIndex, :); %#ok<AGROW>
        % Add the same-node transition only when the obstacle sweep permits waiting through the full layer interval.
        if waitIsClear(layerIndex, currentNodeIndex)
            waitCount = waitCount + 1;
            [reachable, spatialCost_units, parentLayerIndex, ...
                parentNodeIndex] = updateTemporalState(reachable, spatialCost_units, parentLayerIndex, parentNodeIndex, layerIndex, currentNodeIndex, layerIndex + 1, currentNodeIndex, 0);
        else
            rejectedCount = rejectedCount + 1;
        end
    end
    [motionCandidates, candidateRejections] = buildLayerCandidates(currentNodeIndices, layerTimes_s(layerIndex), layerTimes_s, motionEdgeExists, minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, isWaitComponentStart, waitComponentFinalLayerIndex);
    rejectedCount = rejectedCount + candidateRejections;
    motionCandidateCount = size(motionCandidates, 1);
    pendingMotion    = true(motionCandidateCount, 1);

    % Keep the first clear entry per wait interval; later entries can be reached by waiting.
    while any(pendingMotion)
        queriedTargetLayers = unique(motionCandidates(pendingMotion, 3));
        % Process each target layer needed to complete time expanded visibility search.
        for targetLayerIndex = reshape(queriedTargetLayers, 1, [])
            queryIndices = find(pendingMotion & motionCandidates(:, 3) == targetLayerIndex);
            % Use the prescribed final layer for fixed-arrival requests; earliest-arrival selection was resolved during forward search.
            if options.GoalTimeMode ~= "earliestArrival"
                trialCost_units  = reshape(spatialCost_units(layerIndex, motionCandidates(queryIndices, 1)), [], 1) + motionCandidates(queryIndices, 5);
                storedCost_units = reshape(spatialCost_units(targetLayerIndex, motionCandidates(queryIndices, 2)), [], 1);
                % Preserve cheaper arrivals and cost ties that can propagate through safe waits,
                % including frontier states that an early exit would otherwise omit.
                isDominated = trialCost_units > storedCost_units + 1e-12;
                pendingMotion(queryIndices(isDominated)) = false;
                rejectedCount = rejectedCount + nnz(isDominated);
                queryIndices  = queryIndices(~isDominated);
            end
            if isempty(queryIndices)
                continue;
            end
            queryIsClear = edgeIsClear(motionCandidates(queryIndices, 1), motionCandidates(queryIndices, 2), layerTimes_s(layerIndex), layerTimes_s(targetLayerIndex));
            clearIndices = queryIndices(queryIsClear);
            motionCount  = motionCount + numel(clearIndices);
            % Process each motion needed to complete time expanded visibility search.
            for motionIndex = reshape(clearIndices, 1, [])
                [reachable, spatialCost_units, parentLayerIndex, ...
                    parentNodeIndex] = updateTemporalState(reachable, spatialCost_units, parentLayerIndex, parentNodeIndex, layerIndex, motionCandidates(motionIndex, 1), motionCandidates(motionIndex, 3), motionCandidates(motionIndex, 2), motionCandidates(motionIndex, 5));
            end
            rejectedCount = rejectedCount + nnz(~queryIsClear);
            pendingMotion(queryIndices) = false;
            advanceIndices = queryIndices(~queryIsClear & motionCandidates(queryIndices, 3) < motionCandidates(queryIndices, 4));
            motionCandidates(advanceIndices, 3) = motionCandidates(advanceIndices, 3) + 1;
            pendingMotion(advanceIndices) = true;
        end
    end
end
%% Section 2: Reconstruct Goal And Best-Partial Routes
deepestLayerIndex = find(any(reachable, 2), 1, "last");
[frontier_units, bestPartial_units] = deal(zeros(0, 2));
% Report no partial timed route when even the start layer has no reachable state.
if ~isempty(deepestLayerIndex)
    frontierNodeIndices = find(reachable(deepestLayerIndex, :));
    frontier_units        = nodePosition_units(frontierNodeIndices, :);
    [~, bestIndex]       = min(vecnorm(frontier_units - nodePosition_units(2, :), 2, 2));
    [bestPartial_units, ~] = reconstructTimedRoute(nodePosition_units, layerTimes_s, parentLayerIndex, parentNodeIndex, deepestLayerIndex, frontierNodeIndices(bestIndex));
end
% Continue searching only until the first reachable goal layer in earliest-arrival mode; fixed-arrival mode must evaluate its prescribed horizon.
if options.GoalTimeMode == "earliestArrival"
    goalLayerIndex = find(reachable(:, 2), 1, "first");
else
    goalLayerIndex = find(reachable(:, 2) & (1:layerCount).' == layerCount, 1, "first");
end
[route_units, routeTime_s] = reconstructTimedRoute(nodePosition_units, layerTimes_s, parentLayerIndex, parentNodeIndex, goalLayerIndex, 2);
record = struct("LayerTimes_s", layerTimes_s, ...
    "CandidateLayerCount", layerCount, "NodeCount", nodeCount, ...
    "WaitEdgeCount", waitCount, "MotionEdgeCount", motionCount, ...
    "RejectedTransitionCount", rejectedCount, "ExpandedCount", expandedCount, ...
    "ExploredNodes_units", exploredNodes_units, "FrontierNodes_units", frontier_units, ...
    "BestPartialRoute_units", bestPartial_units, ...
    "SelectedGoalLayerIndex", goalLayerIndex, ...
    "ReachableGoalLayerCount", nnz(reachable(:, 2)));
function clear = edgeIsClear(firstNodeIndices, secondNodeIndices, first_s, second_s)
    % A blocked sample rejects the edge. Check the midpoint first, then keep
    % all remaining samples for edges that could still be clear.
    fraction     = linspace(0, 1, 13).';
    firstNodeIndices = firstNodeIndices(:);
    secondNodeIndices = secondNodeIndices(:);
    first_units = nodePosition_units(firstNodeIndices, :);
    second_units = nodePosition_units(secondNodeIndices, :);
    edgeCount    = numel(firstNodeIndices);
    time_s       = first_s + fraction * (second_s - first_s);
    middleIndex  = ceil(numel(fraction) / 2);
    sampleOrder  = [middleIndex, 1:middleIndex - 1, middleIndex + 1:numel(fraction)];
    clear        = true(edgeCount, 1);
    for sampleIndex = sampleOrder
        candidate = find(clear);
        if isempty(candidate)
            break;
        end
        if sampleIndex == 1 || sampleIndex == numel(fraction)
            % Reachability already checked both endpoint nodes. Reuse that
            % result only when the sampled arithmetic reaches the same point and time.
            sampledEndpoint_units = first_units(candidate, :) + fraction(sampleIndex) .* (second_units(candidate, :) - first_units(candidate, :));
            if sampleIndex == 1
                checkedEndpoint_units = first_units(candidate, :);
                checkedTime_s = first_s;
            else
                checkedEndpoint_units = second_units(candidate, :);
                checkedTime_s = second_s;
            end
            candidate = candidate(~(all(sampledEndpoint_units == checkedEndpoint_units, 2) & time_s(sampleIndex) == checkedTime_s));
            if isempty(candidate), continue; end
        end
        geometryKey = 1 + 2 * nnz(geometryTimes_s < time_s(sampleIndex)) + any(geometryTimes_s == time_s(sampleIndex));
        useCache = cacheSlotCount > 0 && isfinite(time_s(sampleIndex)) && stationaryTimeCell(geometryKey);
        if useCache
            cacheSlot = 1 + mod(geometryKey - 1, cacheSlotCount);
            if occupancyCacheKey(cacheSlot) ~= geometryKey
                if ~isempty(batchPointIndices) && mod(geometryKey, 2) == 1
                    % Populate stationary intervals in one query. Exact sample
                    % times retain lazy entries; moving intervals bypass reuse.
                    batchOccupied = obstacleAvoidance.obstacles.queryPreparedObstacles(obstacles, batchPositions_units(:, 1), batchPositions_units(:, 2), time_s(sampleIndex));
                    occupancyCache{cacheSlot} = reshape(1 + uint8(batchOccupied(batchPointIndices)), nodeCount ^ 2, 13);
                else
                    occupancyCache{cacheSlot} = zeros(nodeCount ^ 2, 13, 'uint8');
                end
                occupancyCacheKey(cacheSlot) = geometryKey;
            end
            cacheIndices = firstNodeIndices(candidate) + nodeCount * (secondNodeIndices(candidate) - 1) + nodeCount ^ 2 * (sampleIndex - 1);
            priorOccupancy = occupancyCache{cacheSlot}(cacheIndices);
            clear(candidate(priorOccupancy == 2)) = false;
            candidate = candidate(priorOccupancy == 0);
            cacheIndices = cacheIndices(priorOccupancy == 0);
        end
        if isempty(candidate), continue; end
        position_units = first_units(candidate, :) + fraction(sampleIndex) .* (second_units(candidate, :) - first_units(candidate, :));
        clear(candidate) = ~obstacleAvoidance.obstacles.queryPreparedObstacles(obstacles, position_units(:, 1), position_units(:, 2), time_s(sampleIndex));
        if useCache
            occupancyCache{cacheSlot}(cacheIndices) = 2 - uint8(clear(candidate));
        end
    end
end
end
%% Section 3: Local Functions
function [candidates, rejectedCount] = buildLayerCandidates(sourceNodes, sourceTime_s, layerTimes_s, motionEdgeExists, minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, isWaitComponentStart, waitComponentFinalLayerIndex)
    % Enumerate [entry layer, target node, source node] in the original order.
    % Through-moving nodes retain the velocity-only duration lower bound.
    candidates = zeros(0, 5);
    rejectedCount = 0;
    if isempty(sourceNodes), return; end
    layerCount = numel(layerTimes_s);
    nodeCount = size(nodeIsFree, 2);
    sourceCount = numel(sourceNodes);
    earliestTime_s = sourceTime_s + minimumEdgeDuration_s(sourceNodes, :).' - 1e-12;
    firstFeasibleLayers = 1 + sum(reshape(layerTimes_s, [], 1, 1) <= reshape(earliestTime_s, 1, nodeCount, sourceCount), 1);
    firstFeasibleLayers(isnan(earliestTime_s)) = layerCount + 1;
    starts = reshape(isWaitComponentStart, layerCount, nodeCount, 1) & (reshape((1:layerCount).', [], 1, 1) >= firstFeasibleLayers);
    feasiblePairs = find(firstFeasibleLayers <= layerCount);
    firstEntries = firstFeasibleLayers(feasiblePairs) + layerCount * (feasiblePairs - 1);
    targetNodes = 1 + mod(feasiblePairs - 1, nodeCount);
    firstStates = firstFeasibleLayers(feasiblePairs) + layerCount * (targetNodes - 1);
    starts(firstEntries) = nodeIsFree(firstStates);
    enabledEdges = motionEdgeExists(sourceNodes, :).';
    starts = starts & reshape(enabledEdges, 1, nodeCount, sourceCount);
    rejectedCount = nnz(enabledEdges & ~reshape(any(starts, 1), nodeCount, sourceCount));
    % Column-major enumeration matches the old source/target/layer loops.
    [entryLayers, targetNodes, sourceOffsets] = ind2sub([layerCount, nodeCount, sourceCount], find(starts));
    selectedSources = reshape(sourceNodes(sourceOffsets), [], 1);
    finalStates = entryLayers + layerCount * (targetNodes - 1);
    finalLayers = double(waitComponentFinalLayerIndex(finalStates));
    edgeIndices = selectedSources + nodeCount * (targetNodes - 1);
    candidates = [selectedSources, targetNodes, entryLayers, finalLayers, motionEdgeLengths_units(edgeIndices)];
end

function [geometryTimes_s, stationaryTimeCell] = stationaryGeometryCells(obstacles)
    % Exact history samples and open intervals have distinct geometry keys.
    geometryTimes_s = zeros(0, 1);
    for obstacleIndex = 1:numel(obstacles)
        geometryTimes_s = [geometryTimes_s; double(obstacles(obstacleIndex).time_s(:))]; %#ok<AGROW>
    end
    geometryTimes_s = unique(geometryTimes_s);
    stationaryTimeCell = true(2 * numel(geometryTimes_s) + 1, 1);
    for obstacleIndex = 1:numel(obstacles)
        obstacle = obstacles(obstacleIndex);
        preparation = obstacle.InternalPreparation;
        movingIntervals = find(preparation.MatchingTopology & preparation.IntervalSpeedBound_units_s > 0);
        for intervalIndex = reshape(movingIntervals, 1, [])
            inInterval = geometryTimes_s >= obstacle.time_s(intervalIndex) & geometryTimes_s < obstacle.time_s(intervalIndex + 1);
            stationaryTimeCell(2 * find(inInterval) + 1) = false;
        end
    end
end
function [reachable, spatialCost_units, parentLayerIndex, parentNodeIndex] = updateTemporalState(reachable, spatialCost_units, parentLayerIndex, parentNodeIndex, sourceLayerIndex, sourceNodeIndex, targetLayerIndex, targetNodeIndex, edgeLength_units)
    % Keep the shortest spatial cost and break final-layer ties consistently.
    trialCost_units          = spatialCost_units(sourceLayerIndex, sourceNodeIndex) + edgeLength_units;
    storedCost_units         = spatialCost_units(targetLayerIndex, targetNodeIndex);
    costIsEqual            = abs(trialCost_units - storedCost_units) <= 1e-12;
    isLaterFinalTransition = targetLayerIndex == size(reachable, 1) && edgeLength_units > 0 && sourceLayerIndex > double(parentLayerIndex(targetLayerIndex, targetNodeIndex));
    % Discard transitions that are more expensive than the stored state, while allowing the designated equal-cost final transition.
    if trialCost_units > storedCost_units + 1e-12 || (costIsEqual && ~isLaterFinalTransition)
        return;
    end
    reachable(targetLayerIndex, targetNodeIndex) = true;
    spatialCost_units(targetLayerIndex, targetNodeIndex) = trialCost_units;
    parentLayerIndex(targetLayerIndex, targetNodeIndex) = uint32(sourceLayerIndex);
    parentNodeIndex(targetLayerIndex, targetNodeIndex) = uint16(sourceNodeIndex);
end
function [route_units, routeTime_s] = reconstructTimedRoute(nodePosition_units, layerTimes_s, parentLayerIndex, parentNodeIndex, goalLayerIndex, goalNodeIndex)
    % Follow temporal parents backward, including waits.
    route_units   = zeros(0, 2);
    routeTime_s = zeros(0, 1);
    if isempty(goalLayerIndex)
        return;
    end
    layerPath = goalLayerIndex;
    nodePath  = goalNodeIndex;
    % Continue iterating until the stopping condition for complete reconstruct timed route is satisfied.
    while ~(layerPath(1) == 1 && nodePath(1) == 1)
        priorLayerIndex = double(parentLayerIndex(layerPath(1), nodePath(1)));
        priorNodeIndex  = double(parentNodeIndex(layerPath(1), nodePath(1)));
        % Stop backtracking at the recorded start sentinel; encountering it earlier would indicate corrupt parent data.
        if priorLayerIndex == 0 || priorNodeIndex == 0
            route_units   = zeros(0, 2);
            routeTime_s = zeros(0, 1);
            return;
        end
        layerPath = [priorLayerIndex; layerPath]; %#ok<AGROW>
        nodePath  = [priorNodeIndex; nodePath]; %#ok<AGROW>
    end
    route_units   = nodePosition_units(nodePath, :);
    routeTime_s = layerTimes_s(layerPath);
end
