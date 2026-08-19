function [route_deg, routeTime_s, record] = ...
        searchAzElTimeExpandedGraph( ...
        nodePosition_deg, edgeCost_deg, obstacles, initialState, ...
        goalState, limits, sampleTimes_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [route_deg, routeTime_s, record] = ...
%       azElInternal.searchAzElTimeExpandedGraph( ...
%       nodePosition_deg, edgeCost_deg, obstacles, initialState, ...
%       goalState, limits, sampleTimes_s, options)
%**************************************************************************
% PURPOSE
%   - Search bounded forward time layers with motion and waiting edges.
%**************************************************************************
% INPUTS
%   - nodePosition_deg (N-by-2 finite numeric matrix)
%       Candidate positions in [azimuth elevation] order.
%   - edgeCost_deg (N-by-N nonnegative numeric matrix)
%       Finite entries identify candidate spatial edges.
%   - obstacles (canonical protected obstacle struct array)
%   - initialState, goalState (normalized scalar state structs)
%   - limits (normalized physical limits struct)
%   - sampleTimes_s (finite numeric vector)
%       Obstacle source and midpoint times retained when capacity permits.
%   - options (resolved planner options struct)
%**************************************************************************
% OUTPUTS
%   - route_deg (M-by-2 numeric matrix)
%       Empty on search failure. Otherwise, one timed seed route.
%   - routeTime_s (M-by-1 numeric vector)
%       Strictly increasing layer times for route_deg.
%   - record (scalar struct)
%       Complete counts and a bounded explored-node trace.
%**************************************************************************
% UNITS
%   - Position and edge cost are degrees. Time is seconds.
%**************************************************************************

%% Section 1: Build The Bounded Time Layers

layerTimes_s = boundedTimeLayers( ...
    sampleTimes_s, initialState.time_s, goalState.time_s, 17);
layerCount = numel(layerTimes_s);
nodeCount = size(nodePosition_deg, 1);
nodeIsFree = true(layerCount, nodeCount);
for layerIndex = 1:layerCount
    queryTime_s = repmat(layerTimes_s(layerIndex), nodeCount, 1);
    nodeIsFree(layerIndex, :) = ~queryAzElTimeObstacle( ...
        obstacles, nodePosition_deg(:, 1), ...
        nodePosition_deg(:, 2), queryTime_s).';
end

%% Section 2: Propagate Motion And Waiting Edges

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
for layerIndex = 1:layerCount - 1
    currentNodeIndices = find(reachable(layerIndex, :));
    for currentNodeIndex = reshape(currentNodeIndices, 1, [])
        expandedCount = expandedCount + 1;
        exploredNodes_deg(end + 1, :) = ...
            nodePosition_deg(currentNodeIndex, :); %#ok<AGROW>
        if nodeIsFree(layerIndex + 1, currentNodeIndex) && ...
                motionEdgeIsClear( ...
                obstacles, nodePosition_deg(currentNodeIndex, :), ...
                nodePosition_deg(currentNodeIndex, :), ...
                layerTimes_s(layerIndex), ...
                layerTimes_s(layerIndex + 1))
            [reachable, spatialCost_deg, parentLayerIndex, ...
                parentNodeIndex] = updateTemporalState( ...
                reachable, spatialCost_deg, parentLayerIndex, ...
                parentNodeIndex, layerIndex, currentNodeIndex, ...
                layerIndex + 1, currentNodeIndex, 0);
            waitEdgeCount = waitEdgeCount + 1;
        else
            rejectedTransitionCount = rejectedTransitionCount + 1;
        end
        visibleNeighbor = find(isfinite( ...
            edgeCost_deg(currentNodeIndex, :)));
        visibleNeighbor(visibleNeighbor == currentNodeIndex) = [];
        [~, neighborOrder] = sort( ...
            edgeCost_deg(currentNodeIndex, visibleNeighbor), "ascend");
        maximumNeighborCount = min(8, numel(neighborOrder));
        visibleNeighbor = visibleNeighbor( ...
            neighborOrder(1:maximumNeighborCount));
        for targetNodeIndex = reshape(visibleNeighbor, 1, [])
            displacement_deg = nodePosition_deg(targetNodeIndex, :) - ...
                nodePosition_deg(currentNodeIndex, :);
            minimumDuration_s = max( ...
                abs(displacement_deg) ./ limits.maxVelocity_deg_s);
            candidateLayerIndices = find( ...
                layerTimes_s > layerTimes_s(layerIndex) + ...
                minimumDuration_s - 1e-12);
            if numel(candidateLayerIndices) > 3
                candidateLayerIndices = candidateLayerIndices( ...
                    unique(round(linspace( ...
                    1, numel(candidateLayerIndices), 3))));
            end
            for targetLayerIndex = reshape(candidateLayerIndices, 1, [])
                if ~nodeIsFree(targetLayerIndex, targetNodeIndex) || ...
                        ~motionEdgeIsClear( ...
                        obstacles, nodePosition_deg(currentNodeIndex, :), ...
                        nodePosition_deg(targetNodeIndex, :), ...
                        layerTimes_s(layerIndex), ...
                        layerTimes_s(targetLayerIndex))
                    rejectedTransitionCount = ...
                        rejectedTransitionCount + 1;
                    continue;
                end
                [reachable, spatialCost_deg, parentLayerIndex, ...
                    parentNodeIndex] = updateTemporalState( ...
                    reachable, spatialCost_deg, parentLayerIndex, ...
                    parentNodeIndex, layerIndex, currentNodeIndex, ...
                    targetLayerIndex, targetNodeIndex, ...
                    norm(displacement_deg));
                motionEdgeCount = motionEdgeCount + 1;
                break;
            end
        end
    end
end

%% Section 3: Reconstruct The Earliest Or Final-Layer Route

if options.GoalTimeMode == "earliestArrival"
    goalLayerIndex = find(reachable(:, 2), 1, "first");
else
    goalLayerIndex = layerCount;
    if ~reachable(goalLayerIndex, 2)
        goalLayerIndex = [];
    end
end
[route_deg, routeTime_s] = reconstructTimedRoute( ...
    nodePosition_deg, layerTimes_s, parentLayerIndex, ...
    parentNodeIndex, goalLayerIndex);
record = struct( ...
    "LayerTimes_s", layerTimes_s, ...
    "NodeCount", nodeCount, ...
    "NodePosition_deg", nodePosition_deg, ...
    "WaitEdgeCount", waitEdgeCount, ...
    "MotionEdgeCount", motionEdgeCount, ...
    "RejectedTransitionCount", rejectedTransitionCount, ...
    "ExpandedCount", expandedCount, ...
    "ExploredNodes_deg", exploredNodes_deg);
end

%% Section 4: Local Functions

function layerTimes_s = boundedTimeLayers( ...
        sampleTimes_s, startTime_s, endTime_s, maximumLayerCount)
% PURPOSE
%   - Retain source timing and an early-time base in one bounded layer set.
layerFraction = linspace(0, 1, maximumLayerCount).'.^2;
uniformTime_s = startTime_s + ...
    (endTime_s - startTime_s) * layerFraction;
candidateTime_s = unique([startTime_s; sampleTimes_s(:); ...
    uniformTime_s; endTime_s]);
candidateTime_s = candidateTime_s( ...
    candidateTime_s >= startTime_s & candidateTime_s <= endTime_s);
if numel(candidateTime_s) <= maximumLayerCount
    layerTimes_s = candidateTime_s;
    return;
end
selectedIndex = zeros(maximumLayerCount, 1);
for targetIndex = 1:maximumLayerCount
    [~, selectedIndex(targetIndex)] = min( ...
        abs(candidateTime_s - uniformTime_s(targetIndex)));
end
selectedIndex = unique([1; selectedIndex; numel(candidateTime_s)]);
layerTimes_s = candidateTime_s(selectedIndex);
end

function clear = motionEdgeIsClear( ...
        obstacles, firstPosition_deg, secondPosition_deg, ...
        firstTime_s, secondTime_s)
% PURPOSE
%   - Check a moving seed edge at deterministic trajectory-time samples.
% A bounded 2-degree seed step resolves useful topology without claiming a
% collision certificate. The independent validator remains authoritative.
sampleCount = max(5, min(33, 2 * ...
    ceil(norm(secondPosition_deg - firstPosition_deg) / 4) + 1));
sampleFraction = linspace(0, 1, sampleCount).';
samplePosition_deg = firstPosition_deg + sampleFraction .* ...
    (secondPosition_deg - firstPosition_deg);
sampleTime_s = firstTime_s + sampleFraction * ...
    (secondTime_s - firstTime_s);
occupied = queryAzElTimeObstacle( ...
    obstacles, samplePosition_deg(:, 1), ...
    samplePosition_deg(:, 2), sampleTime_s);
clear = ~any(occupied);
end

function [reachable, spatialCost_deg, parentLayerIndex, ...
        parentNodeIndex] = updateTemporalState( ...
        reachable, spatialCost_deg, parentLayerIndex, parentNodeIndex, ...
        sourceLayerIndex, sourceNodeIndex, targetLayerIndex, ...
        targetNodeIndex, edgeLength_deg)
% PURPOSE
%   - Apply deterministic shortest-distance tie breaking at one timed state.
trialCost_deg = spatialCost_deg( ...
    sourceLayerIndex, sourceNodeIndex) + edgeLength_deg;
if trialCost_deg >= spatialCost_deg( ...
        targetLayerIndex, targetNodeIndex) - 1e-12
    return;
end
reachable(targetLayerIndex, targetNodeIndex) = true;
spatialCost_deg(targetLayerIndex, targetNodeIndex) = trialCost_deg;
parentLayerIndex(targetLayerIndex, targetNodeIndex) = ...
    uint16(sourceLayerIndex);
parentNodeIndex(targetLayerIndex, targetNodeIndex) = ...
    uint16(sourceNodeIndex);
end

function [route_deg, routeTime_s] = reconstructTimedRoute( ...
        nodePosition_deg, layerTimes_s, parentLayerIndex, ...
        parentNodeIndex, goalLayerIndex)
% PURPOSE
%   - Reconstruct one forward time-layer path without losing wait states.
route_deg = zeros(0, 2);
routeTime_s = zeros(0, 1);
if isempty(goalLayerIndex)
    return;
end
layerPath = goalLayerIndex;
nodePath = 2;
while ~(layerPath(1) == 1 && nodePath(1) == 1)
    priorLayerIndex = double(parentLayerIndex( ...
        layerPath(1), nodePath(1)));
    priorNodeIndex = double(parentNodeIndex( ...
        layerPath(1), nodePath(1)));
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
