function [route_deg, routeTime_s, record] = ...
        timeExpandedVisibilitySearch(nodePosition_deg, edgeCost_deg, ...
        obstacles, initialState, goalState, limits, sampleTimes_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [route_deg, routeTime_s, record] = ...
%       obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
%       nodePosition_deg, edgeCost_deg, obstacles, initialState, ...
%       goalState, limits, sampleTimes_s, options)
%**************************************************************************
% PURPOSE
%   - Search deterministic forward reachability over complete input-derived
%     time layers using waits and exact-history motion queries.
%**************************************************************************
% INPUTS
%   - nodePosition_deg (N-by-2 numeric matrix)
%       Search nodes with start first and goal second.
%   - edgeCost_deg (N-by-N numeric matrix)
%       Finite entries enable candidate motion edges.
%   - obstacles (canonical protected obstacle struct array)
%       Exact prepared histories queried at trajectory time.
%   - initialState, goalState, limits, options (scalar structs)
%       Resolved request, limits, and goal-time policy.
%   - sampleTimes_s (numeric vector)
%       Complete source-aware request times; no layer cap is applied.
%**************************************************************************
% OUTPUTS
%   - route_deg (M-by-2 numeric matrix)
%       Selected timed route, or zeros(0,2) on exhaustion.
%   - routeTime_s (M-by-1 numeric vector)
%       Absolute route times.
%   - record (scalar struct)
%       Generated edges, reached states, frontier, and best partial ancestry.
%**************************************************************************
% UNITS
%   - Position and edge cost are degrees; time is seconds.
%**************************************************************************

%% Section 1: Create The Forward Layer Graph

layerTimes_s = unique([initialState.time_s; sampleTimes_s(:); ...
    linspace(initialState.time_s, goalState.time_s, 9).'; goalState.time_s]);
layerTimes_s = layerTimes_s(layerTimes_s >= initialState.time_s & ...
    layerTimes_s <= goalState.time_s);
layerCount = numel(layerTimes_s);
nodeCount = size(nodePosition_deg, 1);
nodeIsFree = false(layerCount, nodeCount);
for layerIndex = 1:layerCount
    nodeIsFree(layerIndex, :) = ...
        ~obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        obstacles, nodePosition_deg(:, 1), nodePosition_deg(:, 2), ...
        repmat(layerTimes_s(layerIndex), nodeCount, 1)).';
end
sourceState = zeros(0, 1);
targetState = zeros(0, 1);
edgeLength_deg = zeros(0, 1);
waitCount = 0;
motionCount = 0;
rejectedCount = 0;
for layerIndex = 1:layerCount - 1
    for sourceNode = reshape(find(nodeIsFree(layerIndex, :)), 1, [])
        nextState = sub2ind([layerCount nodeCount], layerIndex + 1, sourceNode);
        waitIsClear = nodeIsFree(layerIndex + 1, sourceNode) && ...
            edgeIsClear(obstacles, nodePosition_deg(sourceNode, :), ...
            nodePosition_deg(sourceNode, :), layerTimes_s(layerIndex), ...
            layerTimes_s(layerIndex + 1));
        if waitIsClear
            [sourceState, targetState, edgeLength_deg] = appendEdge( ...
                sourceState, targetState, edgeLength_deg, ...
                sub2ind([layerCount nodeCount], layerIndex, sourceNode), ...
                nextState, 0);
            waitCount = waitCount + 1;
        else
            rejectedCount = rejectedCount + 1;
        end
        targets = find(isfinite(edgeCost_deg(sourceNode, :)));
        targets(targets == sourceNode) = [];
        for targetNode = reshape(targets, 1, [])
            displacement_deg = nodePosition_deg(targetNode, :) - ...
                nodePosition_deg(sourceNode, :);
            minimumDuration_s = max([abs(displacement_deg) ./ ...
                limits.maxVelocity_deg_s, 2 * sqrt(abs(displacement_deg) ./ ...
                limits.maxAcceleration_deg_s2)]);
            targetLayer = find(layerTimes_s > layerTimes_s(layerIndex) + ...
                minimumDuration_s - 1e-12, 1, "first");
            motionIsClear = ~isempty(targetLayer) && nodeIsFree(targetLayer, targetNode);
            if motionIsClear
                motionIsClear = edgeIsClear(obstacles, nodePosition_deg(sourceNode, :), ...
                    nodePosition_deg(targetNode, :), layerTimes_s(layerIndex), ...
                    layerTimes_s(targetLayer));
            end
            if ~motionIsClear
                rejectedCount = rejectedCount + 1;
                continue;
            end
            [sourceState, targetState, edgeLength_deg] = appendEdge( ...
                sourceState, targetState, edgeLength_deg, ...
                sub2ind([layerCount nodeCount], layerIndex, sourceNode), ...
                sub2ind([layerCount nodeCount], targetLayer, targetNode), ...
                norm(displacement_deg));
            motionCount = motionCount + 1;
        end
    end
end
searchGraph = digraph(sourceState, targetState, edgeLength_deg, layerCount * nodeCount);

%% Section 2: Select Goal And Best-Partial Paths

reachedState = zeros(0, 1);
if nodeIsFree(1, 1)
    reachedState = bfsearch(searchGraph, 1);
end
goalStates = sub2ind([layerCount nodeCount], (1:layerCount).', 2 * ones(layerCount, 1));
reachedGoal = find(ismember(goalStates, reachedState));
if options.GoalTimeMode == "earliestArrival" && ~isempty(reachedGoal)
    goalStateIndex = goalStates(reachedGoal(1));
elseif options.GoalTimeMode ~= "earliestArrival" && ...
        ismember(goalStates(end), reachedState)
    goalStateIndex = goalStates(end);
else
    goalStateIndex = [];
end
[route_deg, routeTime_s] = statePath(searchGraph, goalStateIndex, ...
    nodePosition_deg, layerTimes_s);
[reachedLayer, reachedNode] = ind2sub([layerCount nodeCount], reachedState);
frontier_deg = zeros(0, 2);
bestPartial_deg = zeros(0, 2);
if ~isempty(reachedState)
    deepestLayer = max(reachedLayer);
    frontierState = reachedState(reachedLayer == deepestLayer);
    [~, frontierNode] = ind2sub([layerCount nodeCount], frontierState);
    frontier_deg = nodePosition_deg(frontierNode, :);
    [~, bestIndex] = min(vecnorm(frontier_deg - nodePosition_deg(2, :), 2, 2));
    [bestPartial_deg, ~] = statePath(searchGraph, frontierState(bestIndex), ...
        nodePosition_deg, layerTimes_s);
end
record = struct("LayerTimes_s", layerTimes_s, "NodeCount", nodeCount, ...
    "WaitEdgeCount", waitCount, "MotionEdgeCount", motionCount, ...
    "RejectedTransitionCount", rejectedCount, ...
    "ExpandedCount", numel(reachedState), ...
    "ExploredNodes_deg", nodePosition_deg(reachedNode, :), ...
    "FrontierNodes_deg", frontier_deg, "BestPartialRoute_deg", bestPartial_deg);
end

%% Section 3: Local Functions

function [source, target, length_deg] = appendEdge( ...
        source, target, length_deg, newSource, newTarget, newLength_deg)
% Append one generated transition without changing deterministic loop order.
source(end + 1, 1) = newSource;
target(end + 1, 1) = newTarget;
length_deg(end + 1, 1) = newLength_deg;
end

function clear = edgeIsClear(obstacles, first_deg, second_deg, first_s, second_s)
% Sample proposal edges; final public validation remains adaptive.
fraction = linspace(0, 1, 13).';
position_deg = first_deg + fraction .* (second_deg - first_deg);
time_s = first_s + fraction * (second_s - first_s);
clear = ~any(obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacles, position_deg(:, 1), position_deg(:, 2), time_s));
end

function [route_deg, routeTime_s] = statePath(graph, targetState, positions_deg, times_s)
% Map one stored graph path back to position/time samples.
route_deg = zeros(0, 2);
routeTime_s = zeros(0, 1);
if isempty(targetState)
    return;
end
state = shortestpath(graph, 1, targetState);
[layer, node] = ind2sub([numel(times_s) size(positions_deg, 1)], state);
route_deg = positions_deg(node, :);
routeTime_s = times_s(layer);
end
