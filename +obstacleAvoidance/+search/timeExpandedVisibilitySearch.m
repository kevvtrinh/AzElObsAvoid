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
% The local obstacle snapshot stays unchanged throughout this search.
obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles,[initialState.time_s,goalState.time_s]);
[geometryTimes_s, stationaryTimeCell] = stationaryGeometryCells(obstacles);

% Cache unknown/free/occupied as 0/1/2 within 300 MiB. Eviction only repeats
% authoritative queries; it never removes a search candidate.
bytesPerGeometry = 13 * nodeCount ^ 2;
maximumCacheBytes = 300 * 1024 ^ 2;
batchPositions_units = zeros(0, 2);
batchPointIndices = zeros(0, 1);
hasStationarySpan = any(stationaryTimeCell(3:2:end - 2));
% Cache directed-edge samples only for exactly unchanged source boundaries.
staticObstacles = obstacles([]);
dynamicObstacles = obstacles;
staticEdgeCache = zeros(0,1,'uint8');
staticTimeRange_s = [-Inf,Inf];
if ~hasStationarySpan && nodeCount^2 <= maximumCacheBytes
    isStatic = false(size(obstacles));
    for j = 1:numel(obstacles)
        obstacle = obstacles(j);
        isStatic(j) = obstacle.InternalPreparation.SamplesExactlyEqual;
        if isStatic(j) && numel(obstacle.time_s)>1
            staticTimeRange_s = [max(staticTimeRange_s(1),obstacle.time_s(1)), ...
                min(staticTimeRange_s(2),obstacle.time_s(end))];
        end
    end
    staticObstacles = obstacles(isStatic);
    dynamicObstacles = obstacles(~isStatic);
    if any(isStatic,'all'), staticEdgeCache = zeros(nodeCount^2,1,'uint8'); end
end
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
    nodeIsFree(layerIndex, :) = ~obstacleAvoidance.obstacles.queryPreparedOccupancy( ...
        obstacles,nodePosition_units(:,1),nodePosition_units(:,2),layerTimes_s(layerIndex),false).';
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
distanceToGoal_units = vecnorm(nodePosition_units - nodePosition_units(2, :), 2, 2);
goalCanWaitToFinal = nodeIsFree(:, 2) & double(waitComponentFinalLayerIndex(:, 2)) == layerCount;
reachable        = false(layerCount, nodeCount);
spatialCost_units  = Inf(layerCount, nodeCount);
parentLayerIndex = zeros(layerCount, nodeCount, "uint32");
parentNodeIndex  = zeros(layerCount, nodeCount, "uint16");
reachable(1, 1) = nodeIsFree(1, 1);
spatialCost_units(1, 1) = 0;
[waitCount, motionCount, rejectedCount, expandedCount, goalBoundRejectionCount, candidateBatchSplitCount] = deal(0);
goalCostBound_units = Inf;
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
    goalCostBound_units = min([goalCostBound_units; spatialCost_units(goalCanWaitToFinal, 2)]);
    [motionCandidates, candidateRejections, batchSplitCount] = buildLayerCandidates(currentNodeIndices, layerTimes_s(layerIndex), layerTimes_s, motionEdgeExists, minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, isWaitComponentStart, waitComponentFinalLayerIndex);
    rejectedCount = rejectedCount + candidateRejections;
    candidateBatchSplitCount = candidateBatchSplitCount + batchSplitCount;
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
                trialCost_units = trialCost_units(~isDominated);
                % A goal state that can wait to the final layer supplies an
                % admissible completion bound. Retain cost ties because they
                % participate in the existing deterministic parent policy.
                cannotImproveGoal = trialCost_units + distanceToGoal_units(motionCandidates(queryIndices, 2)) > goalCostBound_units + 1e-12;
                pendingMotion(queryIndices(cannotImproveGoal)) = false;
                rejectedCount = rejectedCount + nnz(cannotImproveGoal);
                goalBoundRejectionCount = goalBoundRejectionCount + nnz(cannotImproveGoal);
                queryIndices = queryIndices(~cannotImproveGoal);
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
            clearGoalIndices = clearIndices(motionCandidates(clearIndices, 2) == 2 & goalCanWaitToFinal(motionCandidates(clearIndices, 3)));
            if ~isempty(clearGoalIndices)
                clearGoalLayers = motionCandidates(clearGoalIndices, 3);
                goalCostBound_units = min([goalCostBound_units; spatialCost_units(clearGoalLayers, 2)]);
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
    "RejectedTransitionCount", rejectedCount, ...
    "GoalCostBoundRejectionCount", goalBoundRejectionCount, ...
    "CandidateBatchSplitCount", candidateBatchSplitCount, ...
    "ExpandedCount", expandedCount, ...
    "ExploredNodes_units", exploredNodes_units, "FrontierNodes_units", frontier_units, ...
    "BestPartialRoute_units", bestPartial_units, ...
    "SelectedGoalLayerIndex", goalLayerIndex, ...
    "ReachableGoalLayerCount", nnz(reachable(:, 2)));
function clear = edgeIsClear(firstNodeIndices, secondNodeIndices, first_s, second_s)
    % A blocked sample rejects the edge. Check interior samples first, then keep
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
    if ~hasStationarySpan
        % A cached edge retains all thirteen original sample checks. Only use
        % it while every cached obstacle is active at every sampled time.
        queryObstacles = obstacles;
        if ~isempty(staticEdgeCache) && all(time_s>=staticTimeRange_s(1) & time_s<=staticTimeRange_s(2))
            cacheKeys = firstNodeIndices + nodeCount*(secondNodeIndices-1);
            unknown = find(staticEdgeCache(cacheKeys)==0);
            batchSize = max(1,floor(2^18/numel(fraction)));
            for batchStart = 1:batchSize:numel(unknown)
                indices = unknown(batchStart:min(numel(unknown),batchStart+batchSize-1));
                x_units = first_units(indices,1) + fraction.' .* (second_units(indices,1)-first_units(indices,1));
                y_units = first_units(indices,2) + fraction.' .* (second_units(indices,2)-first_units(indices,2));
                occupied = obstacleAvoidance.obstacles.queryPreparedOccupancy( ...
                    staticObstacles,x_units,y_units,repmat(time_s.',numel(indices),1),false);
                staticEdgeCache(cacheKeys(indices)) = 1+uint8(any(occupied,2));
            end
            clear = staticEdgeCache(cacheKeys)==1;
            queryObstacles = dynamicObstacles;
        end
        % Test the quarter, midpoint, and three-quarter samples together.
        % Reject blocked edges before batching the remaining original samples.
        for sampleGroup = {[4,7,10],[1:3,5:6,8:9,11:13]}
            samples = sampleGroup{1};
            candidates = find(clear);
            edgeBatchSize = max(1,floor(2^18/numel(samples)));
            for batchStart = 1:edgeBatchSize:numel(candidates)
                indices = candidates(batchStart:min(numel(candidates),batchStart+edgeBatchSize-1));
                x_units = first_units(indices,1) + fraction(samples).' .* ...
                    (second_units(indices,1)-first_units(indices,1));
                y_units = first_units(indices,2) + fraction(samples).' .* ...
                    (second_units(indices,2)-first_units(indices,2));
                occupied = obstacleAvoidance.obstacles.queryPreparedOccupancy( ...
                    queryObstacles,x_units,y_units,repmat(time_s(samples).',numel(indices),1),false);
                clear(indices) = ~any(occupied,2);
            end
        end
        return;
    end
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
                    batchOccupied = obstacleAvoidance.obstacles.queryPreparedOccupancy( ...
                        obstacles,batchPositions_units(:,1),batchPositions_units(:,2),time_s(sampleIndex),false);
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
        clear(candidate) = ~obstacleAvoidance.obstacles.queryPreparedOccupancy( ...
            obstacles,position_units(:,1),position_units(:,2),time_s(sampleIndex),false);
        if useCache
            occupancyCache{cacheSlot}(cacheIndices) = 2 - uint8(clear(candidate));
        end
    end
end
end
%% Section 3: Local Functions
function [candidates, rejectedCount, batchSplitCount] = buildLayerCandidates(sourceNodes, sourceTime_s, layerTimes_s, motionEdgeExists, minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, isWaitComponentStart, waitComponentFinalLayerIndex)
    % Enumerate [entry layer, target node, source node] in the original order.
    % Bound the temporary logical tensor while retaining the vectorized path
    % for ordinary inputs and the velocity-only duration lower bound.
    candidates = zeros(0, 5);
    rejectedCount = 0;
    batchSplitCount = 0;
    if isempty(sourceNodes), return; end
    layerCount = numel(layerTimes_s);
    nodeCount = size(nodeIsFree, 2);
    sourceCount = numel(sourceNodes);
    maximumCandidateTensorElements = 1024 ^ 2;
    sourceBatchSize = max(1, floor(maximumCandidateTensorElements / max(1, layerCount * nodeCount)));
    batchCount = ceil(sourceCount / sourceBatchSize);
    batchSplitCount = batchCount - 1;
    candidateBlocks = cell(batchCount, 1);
    for batchIndex = 1:batchCount
        firstSourceOffset = 1 + (batchIndex - 1) * sourceBatchSize;
        finalSourceOffset = min(sourceCount, batchIndex * sourceBatchSize);
        batchSources = sourceNodes(firstSourceOffset:finalSourceOffset);
        [candidateBlocks{batchIndex}, batchRejectedCount] = buildCandidateBatch(batchSources, sourceTime_s, layerTimes_s, motionEdgeExists, minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, isWaitComponentStart, waitComponentFinalLayerIndex);
        rejectedCount = rejectedCount + batchRejectedCount;
    end
    candidates = vertcat(candidateBlocks{:});
end

function [candidates, rejectedCount] = buildCandidateBatch(sourceNodes, sourceTime_s, layerTimes_s, motionEdgeExists, minimumEdgeDuration_s, motionEdgeLengths_units, nodeIsFree, isWaitComponentStart, waitComponentFinalLayerIndex)
    % Vectorize one bounded block and preserve source/target/layer ordering.
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
    % Column-major enumeration matches the original source/target/layer loops.
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
