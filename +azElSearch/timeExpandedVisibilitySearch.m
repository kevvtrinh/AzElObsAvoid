function [route_deg, routeTime_s, record] = timeExpandedVisibilitySearch( ...
        nodePosition_deg, edgeCost_deg, obstacles, initialState, goalState, ...
        limits, sampleTimes_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [route_deg, routeTime_s, record] = ...
%       azElSearch.timeExpandedVisibilitySearch( ...
%       nodePosition_deg, edgeCost_deg, obstacles, initialState, goalState, ...
%       limits, sampleTimes_s, options)
%**************************************************************************
% PURPOSE
%   - Search a shared forward time-layer visibility graph with waiting and
%     collision-checked motion edges.
%**************************************************************************
% INPUTS
%   - nodePosition_deg (N-by-2 numeric matrix)
%       Visibility-graph nodes with start first and goal second.
%   - edgeCost_deg (N-by-N numeric matrix)
%       Finite entries identify spatially visible neighbor edges.
%   - obstacles (canonical protected obstacle array)
%       Static or time-varying geometry queried at trajectory time.
%   - initialState, goalState, limits, options (scalar structs)
%       Resolved request, physical limits, and goal-time policy.
%   - sampleTimes_s (numeric vector)
%       Source-aware candidate time layers.
%**************************************************************************
% OUTPUTS
%   - route_deg (M-by-2 numeric matrix)
%       Timed route geometry, or zeros(0, 2) when no route is found.
%   - routeTime_s (M-by-1 numeric vector)
%       Absolute time associated with each route point.
%   - record (scalar struct)
%       Complete temporal-search counts and bounded diagnostic geometry.
%**************************************************************************
% UNITS
%   - Position and edge costs are degrees. Time is seconds.
%**************************************************************************

%% Section 1: Search Time Layers
layerTimes_s = azElSearch.boundedTimeLayers( ...
    sampleTimes_s, initialState.time_s, goalState.time_s, 17);
layerCount = numel(layerTimes_s);
nodeCount = size(nodePosition_deg, 1);
nodeIsFree = false(layerCount, nodeCount);

% Precompute whether every graph node is free at every retained time layer.
for layerIndex = 1:layerCount
    queryTime_s = repmat(layerTimes_s(layerIndex), nodeCount, 1);
    nodeIsFree(layerIndex, :) = ~queryAzElTimeObstacle(obstacles, ...
        nodePosition_deg(:, 1), nodePosition_deg(:, 2), queryTime_s).';
end
reachable = false(layerCount, nodeCount);
spatialCost_deg = Inf(layerCount, nodeCount);
parentLayerIndex = zeros(layerCount, nodeCount, "uint16");
parentNodeIndex = zeros(layerCount, nodeCount, "uint16");
reachable(1, 1) = nodeIsFree(1, 1);
spatialCost_deg(1, 1) = 0;
waitEdgeCount = 0;
motionEdgeCount = 0;
rejectedTransitionCount = 0;
expandedCount = 0;
exploredNodes_deg = zeros(0, 2);

% Propagate reachability forward only; timed routes may wait but never travel backward in time.
for layerIndex = 1:layerCount - 1
    % Earliest-arrival search can stop at the first time layer that reaches the goal.
    if options.GoalTimeMode == "earliestArrival" && reachable(layerIndex, 2)
        break;
    end
    currentNodeIndices = find(reachable(layerIndex, :));

    % Expand every node reached at the current time layer.
    for currentNodeIndex = reshape(currentNodeIndices, 1, [])
        expandedCount = expandedCount + 1;
        exploredNodes_deg(end + 1, :) = nodePosition_deg(currentNodeIndex, :); %#ok<AGROW>
        if nodeIsFree(layerIndex + 1, currentNodeIndex) && ...
                motionEdgeIsClear( ...
                obstacles, nodePosition_deg(currentNodeIndex, :), ...
                nodePosition_deg(currentNodeIndex, :), ...
                layerTimes_s(layerIndex), layerTimes_s(layerIndex + 1))
            [reachable, spatialCost_deg, parentLayerIndex, ...
                parentNodeIndex] = updateTemporalState( ...
                reachable, spatialCost_deg, parentLayerIndex, ...
                parentNodeIndex, layerIndex, currentNodeIndex, layerIndex + 1, currentNodeIndex, 0);
            waitEdgeCount = waitEdgeCount + 1;
        else
            rejectedTransitionCount = rejectedTransitionCount + 1;
        end
        visibleNeighbor = find(isfinite(edgeCost_deg(currentNodeIndex, :)));
        targetNodeIndices = unique([1, 2, visibleNeighbor]);
        targetNodeIndices(targetNodeIndices == currentNodeIndex) = [];

        % The same-node wait edge is handled separately.
        for targetNodeIndex = reshape(targetNodeIndices, 1, [])
            displacement_deg = nodePosition_deg(targetNodeIndex, :) - ...
                nodePosition_deg(currentNodeIndex, :);
            speedDuration_s = abs(displacement_deg) ./ limits.maxVelocity_deg_s;
            accelerationDuration_s = 2 * sqrt( ...
                abs(displacement_deg) ./ limits.maxAcceleration_deg_s2);
            minimumDuration_s = max([speedDuration_s, accelerationDuration_s]);
            earliestTargetTime_s = layerTimes_s(layerIndex) + ...
                minimumDuration_s - 1e-12;
            targetLayerIndex = find( ...
                layerTimes_s > earliestTargetTime_s, 1, "first");
            % Reject unavailable destinations before paying for a swept-edge collision query.
            if isempty(targetLayerIndex) || ~nodeIsFree(targetLayerIndex, targetNodeIndex)
                rejectedTransitionCount = rejectedTransitionCount + 1;
                continue;
            end
            isClear = motionEdgeIsClear( ...
                obstacles, nodePosition_deg(currentNodeIndex, :), ...
                nodePosition_deg(targetNodeIndex, :), ...
                layerTimes_s(layerIndex), layerTimes_s(targetLayerIndex));
            if ~isClear
                rejectedTransitionCount = rejectedTransitionCount + 1;
                continue;
            end
            [reachable, spatialCost_deg, parentLayerIndex, ...
                parentNodeIndex] = updateTemporalState( ...
                reachable, spatialCost_deg, parentLayerIndex, ...
                parentNodeIndex, layerIndex, currentNodeIndex, ...
                targetLayerIndex, targetNodeIndex, norm(displacement_deg));
            motionEdgeCount = motionEdgeCount + 1;
        end
    end
end
deepestLayerIndex = find(any(reachable, 2), 1, "last");
frontierNodes_deg = zeros(0, 2);
if ~isempty(deepestLayerIndex)
    frontierNodes_deg = nodePosition_deg(reachable(deepestLayerIndex, :), :);
end
if options.GoalTimeMode == "earliestArrival"
    goalLayerIndex = find(reachable(:, 2), 1, "first");
else
    goalLayerIndex = layerCount;
    if ~reachable(goalLayerIndex, 2)
        goalLayerIndex = [];
    end
end
[route_deg, routeTime_s] = reconstructTimedRoute( ...
    nodePosition_deg, layerTimes_s, parentLayerIndex, parentNodeIndex, goalLayerIndex);
record = struct("LayerTimes_s", layerTimes_s, "NodeCount", nodeCount, ...
    "WaitEdgeCount", waitEdgeCount, "MotionEdgeCount", motionEdgeCount, ...
    "RejectedTransitionCount", rejectedTransitionCount, ...
    "ExpandedCount", expandedCount, ...
    "ExploredNodes_deg", exploredNodes_deg, ...
    "FrontierNodes_deg", frontierNodes_deg);
end

%% Section 2: Local Functions

function clear = motionEdgeIsClear( ...
        obstacles, firstPosition_deg, secondPosition_deg, ...
        firstTime_s, secondTime_s)
% Check a moving seed edge at deterministic trajectory-time samples.
sampleCount = 13;
sampleFraction = linspace(0, 1, sampleCount).';
samplePosition_deg = firstPosition_deg + sampleFraction .* (secondPosition_deg - firstPosition_deg);
sampleTime_s = firstTime_s + sampleFraction * (secondTime_s - firstTime_s);
occupied = queryAzElTimeObstacle( ...
    obstacles, samplePosition_deg(:, 1), ...
    samplePosition_deg(:, 2), sampleTime_s);
clear = ~any(occupied);
end

function [reachable, spatialCost_deg, parentLayerIndex, ...
        parentNodeIndex] = updateTemporalState( ...
        reachable, spatialCost_deg, parentLayerIndex, parentNodeIndex, ...
        sourceLayerIndex, sourceNodeIndex, targetLayerIndex, targetNodeIndex, edgeLength_deg)
% Apply deterministic shortest-distance tie breaking at one timed state.
trialCost_deg = spatialCost_deg( sourceLayerIndex, sourceNodeIndex) + edgeLength_deg;
storedCost_deg = spatialCost_deg(targetLayerIndex, targetNodeIndex);
costIsEqual = abs(trialCost_deg - storedCost_deg) <= 1e-12;
isLaterFinalTransition = targetLayerIndex == size(reachable, 1) && edgeLength_deg > 0 && ...
    sourceLayerIndex > double(parentLayerIndex( targetLayerIndex, targetNodeIndex));
% Retain the cheaper state, with a deterministic exception for a later final transition.
if trialCost_deg > storedCost_deg + 1e-12 || (costIsEqual && ~isLaterFinalTransition)
    return;
end
reachable(targetLayerIndex, targetNodeIndex) = true;
spatialCost_deg(targetLayerIndex, targetNodeIndex) = trialCost_deg;
parentLayerIndex(targetLayerIndex, targetNodeIndex) = uint16(sourceLayerIndex);
parentNodeIndex(targetLayerIndex, targetNodeIndex) = uint16(sourceNodeIndex);
end

function [route_deg, routeTime_s] = reconstructTimedRoute( ...
        nodePosition_deg, layerTimes_s, parentLayerIndex, parentNodeIndex, goalLayerIndex)
% Reconstruct one forward time-layer path without losing wait states.
route_deg = zeros(0, 2);
routeTime_s = zeros(0, 1);
% An empty goal layer is the normal bounded-search no-path outcome.
if isempty(goalLayerIndex)
    return;
end
layerPath = goalLayerIndex;
nodePath = 2;

% Follow temporal parent pointers backward until the start state is reached.
while ~(layerPath(1) == 1 && nodePath(1) == 1)
    priorLayerIndex = double(parentLayerIndex( layerPath(1), nodePath(1)));
    priorNodeIndex = double(parentNodeIndex( layerPath(1), nodePath(1)));
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
