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
        waitIsClear(layerIndex, candidateNodeIndices) = edgeIsClear(obstacles, nodePosition_units(candidateNodeIndices, :), nodePosition_units(candidateNodeIndices, :), layerTimes_s(layerIndex), layerTimes_s(layerIndex + 1));
    end
end

% Within a clear-wait interval, an earlier arrival can wait to match a later one.
isWaitComponentStart = nodeIsFree;
isWaitComponentStart(2:end, :) = nodeIsFree(2:end, :) & ~waitIsClear;
waitComponentCount           = sum(isWaitComponentStart, 1);
waitComponentFinalLayerIndex = repmat(uint32((1:layerCount).'), 1, nodeCount);
% Process each layer needed to complete time expanded visibility search.
for layerIndex = layerCount - 1:-1:1
    continuingNodeIndices = find(waitIsClear(layerIndex, :));
    waitComponentFinalLayerIndex(layerIndex, continuingNodeIndices) = waitComponentFinalLayerIndex(layerIndex + 1, continuingNodeIndices);
end
motionEdgeExists = isfinite(edgeCost_units);
motionEdgeExists(1:nodeCount + 1:end) = false;
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
    maximumMotionCandidateCount = max(1, sum(motionEdgeExists(currentNodeIndices, :) * waitComponentCount.'));
    % Columns are source node, target node, current target layer, final
    % layer in the same safe-wait interval, and spatial length in coordinate units.
    motionCandidates     = zeros(maximumMotionCandidateCount, 5);
    motionCandidateCount = 0;
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
        targets = find(motionEdgeExists(currentNodeIndex, :));
        % Process each target node needed to complete time expanded visibility search.
        for targetNodeIndex = reshape(targets, 1, [])
            displacement_units = nodePosition_units(targetNodeIndex, :) - nodePosition_units(currentNodeIndex, :);
            % Intermediate nodes need not stop. Use a velocity-only time lower bound;
            % a rest-to-rest estimate could incorrectly exclude a feasible transition.
            minimumDuration_s       = max(abs(displacement_units) ./ limits.maxVelocity_units_s);
            earliestTime_s          = layerTimes_s(layerIndex) + minimumDuration_s - 1e-12;
            firstFeasibleLayerIndex = find(layerTimes_s > earliestTime_s, 1, "first");
            % Drop transitions that cannot reach any later time layer even at the velocity limit.
            if isempty(firstFeasibleLayerIndex)
                rejectedCount = rejectedCount + 1;
                continue;
            end
            isCandidateStart = isWaitComponentStart(firstFeasibleLayerIndex:end, targetNodeIndex);
            isCandidateStart(1) = nodeIsFree(firstFeasibleLayerIndex, targetNodeIndex);
            firstLayerIndices = find(isCandidateStart) + firstFeasibleLayerIndex - 1;
            if isempty(firstLayerIndices)
                rejectedCount = rejectedCount + 1;
                continue;
            end
            newMotionCandidateCount = numel(firstLayerIndices);
            motionIndices           = motionCandidateCount + (1:newMotionCandidateCount);
            finalLayerIndices       = double(waitComponentFinalLayerIndex(firstLayerIndices, targetNodeIndex));
            motionCandidates(motionIndices, :) = [repmat([currentNodeIndex, targetNodeIndex], ...
                newMotionCandidateCount, 1), firstLayerIndices, ...
                finalLayerIndices, repmat(norm(displacement_units), ...
                newMotionCandidateCount, 1)];
            motionCandidateCount = motionCandidateCount + newMotionCandidateCount;
        end
    end
    motionCandidates = motionCandidates(1:motionCandidateCount, :);
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
            queryIsClear = edgeIsClear(obstacles, nodePosition_units(motionCandidates(queryIndices, 1), :), nodePosition_units(motionCandidates(queryIndices, 2), :), layerTimes_s(layerIndex), layerTimes_s(targetLayerIndex));
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
end
%% Section 3: Local Functions
function clear = edgeIsClear(obstacles, first_units, second_units, first_s, second_s)
    % Batch edges with the same layer times; keep all 13 sample points.
    fraction     = linspace(0, 1, 13).';
    edgeCount    = size(first_units, 1);
    position_units = zeros(13 * edgeCount, 2);
    time_s       = first_s + fraction * (second_s - first_s);
    % Process each geometric edge while constructing or checking the region topology.
    for edgeIndex = 1:edgeCount
        sampleIndices = (edgeIndex - 1) * 13 + (1:13);
        position_units(sampleIndices, :) = first_units(edgeIndex, :) + fraction .* (second_units(edgeIndex, :) - first_units(edgeIndex, :));
    end
    occupied = obstacleAvoidance.obstacles.queryPreparedObstacles(obstacles, position_units(:, 1), position_units(:, 2), repmat(time_s, edgeCount, 1));
    clear    = ~any(reshape(occupied, 13, edgeCount), 1).';
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
