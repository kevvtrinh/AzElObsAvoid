function [route_deg, routeTime_s, record] = ...
        timeExpandedVisibilitySearch(nodePosition_deg, edgeCost_deg, ...
        obstacles, initialState, goalState, limits, sampleTimes_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [route_deg, routeTime_s, record] = ...
%       obstacleAvoidance.search.timeExpandedVisibilitySearch(nodePosition_deg, ...
%       edgeCost_deg, obstacles, initialState, goalState, limits, sampleTimes_s, options)
%**************************************************************************
% PURPOSE
%   - Search bounded forward reachability using waits and moving edges.
%**************************************************************************
% INPUTS
%   - nodePosition_deg (N-by-2 numeric matrix)
%       Nodes with start first and goal second.
%   - edgeCost_deg (N-by-N numeric matrix)
%       Finite entries enable motion edges.
%   - obstacles (canonical protected obstacle struct array)
%   - initialState, goalState, limits, options (scalar structs)
%   - sampleTimes_s (numeric vector)
%       Candidate times before applying MaximumTimeLayerCount.
%**************************************************************************
% OUTPUTS
%   - route_deg (M-by-2 numeric matrix), routeTime_s (M-by-1 numeric vector)
%       Selected timed route, or documented empty arrays on exhaustion.
%   - record (scalar struct)
%       Search counts, frontier, and best partial ancestry.
%**************************************************************************
% UNITS
%   - Position and edge cost are degrees; time is seconds.
%**************************************************************************
%% Section 1: Propagate The Reachability Frontier
[layerTimes_s, layerLimitApplied, candidateLayerCount] = ...
    obstacleAvoidance.search.boundedTimeLayers(sampleTimes_s, ...
    initialState.time_s, goalState.time_s, options.MaximumTimeLayerCount);
layerCount = numel(layerTimes_s);
nodeCount = size(nodePosition_deg, 1);
nodeIsFree = false(layerCount, nodeCount);
for layerIndex = 1:layerCount
    obstacleAvoidance.input.throwIfCancellationRequested(options);
    nodeIsFree(layerIndex, :) = ~obstacleAvoidance.obstacles. ...
        queryObstacleOccupancyAtTime( ...
        obstacles, nodePosition_deg(:, 1), nodePosition_deg(:, 2), ...
        repmat(layerTimes_s(layerIndex), nodeCount, 1)).';
end
reachable = false(layerCount, nodeCount);
spatialCost_deg = Inf(layerCount, nodeCount);
[parentLayerIndex, parentNodeIndex] = deal(zeros(layerCount, nodeCount, "uint16"));
reachable(1, 1) = nodeIsFree(1, 1);
spatialCost_deg(1, 1) = 0;
[waitCount, motionCount, rejectedCount, expandedCount] = deal(0);
exploredNodes_deg = zeros(0, 2);
for layerIndex = 1:layerCount - 1
    obstacleAvoidance.input.throwIfCancellationRequested(options);
    if options.GoalTimeMode == "earliestArrival" && reachable(layerIndex, 2)
        break;
    end
    currentNodeIndices = find(reachable(layerIndex, :));
    maximumTransitionCount = numel(currentNodeIndices) * nodeCount;
    transitionSourceNodeIndex = zeros(maximumTransitionCount, 1, "uint16");
    transitionTargetNodeIndex = zeros(maximumTransitionCount, 1, "uint16");
    transitionTargetLayerIndex = zeros(maximumTransitionCount, 1, "uint16");
    transitionLength_deg = zeros(maximumTransitionCount, 1);
    transitionIsWait = false(maximumTransitionCount, 1);
    transitionNeedsQuery = false(maximumTransitionCount, 1);
    transitionCount = 0;
    for currentNodeIndex = reshape(currentNodeIndices, 1, [])
        expandedCount = expandedCount + 1;
        if mod(expandedCount - 1, 8) == 0
            obstacleAvoidance.input.throwIfCancellationRequested(options);
        end
        exploredNodes_deg(end + 1, :) = nodePosition_deg(currentNodeIndex, :); %#ok<AGROW>
        transitionCount = transitionCount + 1;
        transitionSourceNodeIndex(transitionCount) = uint16(currentNodeIndex);
        transitionTargetNodeIndex(transitionCount) = uint16(currentNodeIndex);
        transitionTargetLayerIndex(transitionCount) = uint16(layerIndex + 1);
        transitionIsWait(transitionCount) = true;
        transitionNeedsQuery(transitionCount) = ...
            nodeIsFree(layerIndex + 1, currentNodeIndex);
        targets = find(isfinite(edgeCost_deg(currentNodeIndex, :)));
        targets(targets == currentNodeIndex) = [];
        for targetNodeIndex = reshape(targets, 1, [])
            displacement_deg = nodePosition_deg(targetNodeIndex, :) - ...
                nodePosition_deg(currentNodeIndex, :);
            minimumDuration_s = max([abs(displacement_deg) ./ ...
                limits.maxVelocity_deg_s, 2 * sqrt(abs(displacement_deg) ./ ...
                limits.maxAcceleration_deg_s2)]);
            earliestTime_s = layerTimes_s(layerIndex) + minimumDuration_s - 1e-12;
            targetLayerIndex = find(layerTimes_s > earliestTime_s, 1, "first");
            transitionCount = transitionCount + 1;
            transitionSourceNodeIndex(transitionCount) = uint16(currentNodeIndex);
            transitionTargetNodeIndex(transitionCount) = uint16(targetNodeIndex);
            transitionLength_deg(transitionCount) = norm(displacement_deg);
            if ~isempty(targetLayerIndex)
                transitionTargetLayerIndex(transitionCount) = ...
                    uint16(targetLayerIndex);
                transitionNeedsQuery(transitionCount) = ...
                    nodeIsFree(targetLayerIndex, targetNodeIndex);
            end
        end
    end
    transitionSourceNodeIndex = transitionSourceNodeIndex(1:transitionCount);
    transitionTargetNodeIndex = transitionTargetNodeIndex(1:transitionCount);
    transitionTargetLayerIndex = transitionTargetLayerIndex(1:transitionCount);
    transitionLength_deg = transitionLength_deg(1:transitionCount);
    transitionIsWait = transitionIsWait(1:transitionCount);
    transitionNeedsQuery = transitionNeedsQuery(1:transitionCount);
    transitionIsClear = false(transitionCount, 1);
    queriedTargetLayers = unique(transitionTargetLayerIndex(transitionNeedsQuery));
    for targetLayerIndex = reshape(queriedTargetLayers, 1, [])
        queryIndices = find(transitionNeedsQuery & ...
            transitionTargetLayerIndex == targetLayerIndex);
        transitionIsClear(queryIndices) = edgeIsClear(obstacles, ...
            nodePosition_deg(double(transitionSourceNodeIndex(queryIndices)), :), ...
            nodePosition_deg(double(transitionTargetNodeIndex(queryIndices)), :), ...
            layerTimes_s(layerIndex), layerTimes_s(targetLayerIndex));
    end
    for transitionIndex = 1:transitionCount
        if ~transitionIsClear(transitionIndex)
            rejectedCount = rejectedCount + 1;
            continue;
        end
        sourceNodeIndex = double(transitionSourceNodeIndex(transitionIndex));
        targetNodeIndex = double(transitionTargetNodeIndex(transitionIndex));
        targetLayerIndex = double(transitionTargetLayerIndex(transitionIndex));
        edgeLength_deg = transitionLength_deg(transitionIndex);
        if transitionIsWait(transitionIndex)
            waitCount = waitCount + 1;
        else
            motionCount = motionCount + 1;
        end
            [reachable, spatialCost_deg, parentLayerIndex, parentNodeIndex] = ...
                updateTemporalState(reachable, spatialCost_deg, parentLayerIndex, ...
                parentNodeIndex, layerIndex, sourceNodeIndex, targetLayerIndex, ...
                targetNodeIndex, edgeLength_deg);
    end
end
%% Section 2: Reconstruct Goal And Best-Partial Routes
deepestLayerIndex = find(any(reachable, 2), 1, "last");
[frontier_deg, bestPartial_deg] = deal(zeros(0, 2));
if ~isempty(deepestLayerIndex)
    frontierNodeIndices = find(reachable(deepestLayerIndex, :));
    frontier_deg = nodePosition_deg(frontierNodeIndices, :);
    [~, bestIndex] = min(vecnorm(frontier_deg - nodePosition_deg(2, :), 2, 2));
    [bestPartial_deg, ~] = reconstructTimedRoute(nodePosition_deg, layerTimes_s, ...
        parentLayerIndex, parentNodeIndex, deepestLayerIndex, ...
        frontierNodeIndices(bestIndex));
end
if options.GoalTimeMode == "earliestArrival"
    goalLayerIndex = find(reachable(:, 2), 1, "first");
elseif options.GoalTimeMode == "balancedArrival"
    candidateGoalLayers = find(reachable(:, 2));
    if isempty(candidateGoalLayers)
        goalLayerIndex = [];
    else
        travelCost_deg = spatialCost_deg(candidateGoalLayers, 2);
        delayCost_deg = options.MinimumTravelSavingsRate_deg_s * ...
            (layerTimes_s(candidateGoalLayers) - initialState.time_s);
        [~, selectedGoalIndex] = min(travelCost_deg + delayCost_deg);
        goalLayerIndex = candidateGoalLayers(selectedGoalIndex);
    end
else
    goalLayerIndex = find(reachable(:, 2) & (1:layerCount).' == layerCount, 1, "first");
end
[route_deg, routeTime_s] = reconstructTimedRoute(nodePosition_deg, layerTimes_s, ...
    parentLayerIndex, parentNodeIndex, goalLayerIndex, 2);
record = struct("LayerTimes_s", layerTimes_s, ...
    "LayerLimitApplied", layerLimitApplied, ...
    "CandidateLayerCount", candidateLayerCount, "NodeCount", nodeCount, ...
    "WaitEdgeCount", waitCount, "MotionEdgeCount", motionCount, ...
    "RejectedTransitionCount", rejectedCount, "ExpandedCount", expandedCount, ...
    "ExploredNodes_deg", exploredNodes_deg, "FrontierNodes_deg", frontier_deg, ...
    "BestPartialRoute_deg", bestPartial_deg, ...
    "SelectedGoalLayerIndex", goalLayerIndex, ...
    "ReachableGoalLayerCount", nnz(reachable(:, 2)));
end
%% Section 3: Local Functions
function clear = edgeIsClear(obstacles, first_deg, second_deg, first_s, second_s)
% Batch edges sharing layer times without changing their 13 sample points.
fraction = linspace(0, 1, 13).';
edgeCount = size(first_deg, 1);
position_deg = zeros(13 * edgeCount, 2);
time_s = first_s + fraction * (second_s - first_s);
for edgeIndex = 1:edgeCount
    sampleIndices = (edgeIndex - 1) * 13 + (1:13);
    position_deg(sampleIndices, :) = first_deg(edgeIndex, :) + ...
        fraction .* (second_deg(edgeIndex, :) - first_deg(edgeIndex, :));
end
occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacles, position_deg(:, 1), position_deg(:, 2), ...
    repmat(time_s, edgeCount, 1));
clear = ~any(reshape(occupied, 13, edgeCount), 1).';
end
function [reachable, spatialCost_deg, parentLayerIndex, parentNodeIndex] = ...
        updateTemporalState(reachable, spatialCost_deg, parentLayerIndex, ...
        parentNodeIndex, sourceLayerIndex, sourceNodeIndex, ...
        targetLayerIndex, targetNodeIndex, edgeLength_deg)
% Retain shortest spatial cost with deterministic final-layer tie breaking.
trialCost_deg = spatialCost_deg(sourceLayerIndex, sourceNodeIndex) + edgeLength_deg;
storedCost_deg = spatialCost_deg(targetLayerIndex, targetNodeIndex);
costIsEqual = abs(trialCost_deg - storedCost_deg) <= 1e-12;
isLaterFinalTransition = targetLayerIndex == size(reachable, 1) && ...
    edgeLength_deg > 0 && sourceLayerIndex > ...
    double(parentLayerIndex(targetLayerIndex, targetNodeIndex));
if trialCost_deg > storedCost_deg + 1e-12 || (costIsEqual && ~isLaterFinalTransition)
    return;
end
reachable(targetLayerIndex, targetNodeIndex) = true;
spatialCost_deg(targetLayerIndex, targetNodeIndex) = trialCost_deg;
parentLayerIndex(targetLayerIndex, targetNodeIndex) = uint16(sourceLayerIndex);
parentNodeIndex(targetLayerIndex, targetNodeIndex) = uint16(sourceNodeIndex);
end
function [route_deg, routeTime_s] = reconstructTimedRoute(nodePosition_deg, ...
        layerTimes_s, parentLayerIndex, parentNodeIndex, goalLayerIndex, goalNodeIndex)
% Walk temporal parent pointers backward without dropping wait states.
route_deg = zeros(0, 2);
routeTime_s = zeros(0, 1);
if isempty(goalLayerIndex)
    return;
end
layerPath = goalLayerIndex;
nodePath = goalNodeIndex;
while ~(layerPath(1) == 1 && nodePath(1) == 1)
    priorLayerIndex = double(parentLayerIndex(layerPath(1), nodePath(1)));
    priorNodeIndex = double(parentNodeIndex(layerPath(1), nodePath(1)));
    if priorLayerIndex == 0 || priorNodeIndex == 0
        route_deg = zeros(0, 2);
        routeTime_s = zeros(0, 1);
        return;
    end
    layerPath = [priorLayerIndex; layerPath]; %#ok<AGROW>
    nodePath = [priorNodeIndex; nodePath]; %#ok<AGROW>
end
route_deg = nodePosition_deg(nodePath, :);
routeTime_s = layerTimes_s(layerPath);
end
