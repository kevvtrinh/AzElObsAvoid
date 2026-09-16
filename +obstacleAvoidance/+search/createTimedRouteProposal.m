function [route_units, routeTime_s, record] = createTimedRouteProposal(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [route_units, routeTime_s, record] = obstacleAvoidance.search.createTimedRouteProposal( ...
%       obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Build a deterministic time-expanded route proposal for BMTP.
%**************************************************************************
% INPUTS
%   - obstacles (canonical or prepared obstacle array)
%       Protected geometry over the request horizon.
%   - initialState (scalar struct)
%       Normalized initial endpoint state.
%   - goalState (scalar struct)
%       Normalized goal endpoint state.
%   - limits (scalar struct)
%       Normalized workspace and derivative limits.
%   - options (scalar struct)
%       Normalized timed-search options.
%**************************************************************************
% OUTPUTS
%   - route_units (N-by-2 numeric array)
%       Timed route positions, or an empty array after search exhaustion.
%   - routeTime_s (N-by-1 numeric array)
%       Route knot times, or an empty array after search exhaustion.
%   - record (scalar struct)
%       Timed-search evidence. Exhaustion is an ordinary planning outcome;
%       invalid input throws an error.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Build The Sampled Swept Proposal

obstacles = obstacleAvoidance.obstacles.prepareObstacles( ...
    obstacles, [initialState.time_s, goalState.time_s]);
sampleTimes_s  = obstacleAvoidance.search.createTimeLayers(obstacles, initialState.time_s, goalState.time_s);
shapeParts     = cell(numel(sampleTimes_s) * numel(obstacles), 1);
shapePartCount = 0;
for timeIndex = 1:numel(sampleTimes_s)
    for obstacleIndex = 1:numel(obstacles)
        shape = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacles(obstacleIndex), sampleTimes_s(timeIndex));
        if isempty(shape.Vertices)
            continue;
        end
        shapePartCount             = shapePartCount + 1;
        shapeParts{shapePartCount} = shape;
    end
end
proposalShape = polyshape();
if shapePartCount > 0
    proposalShape = unionShapeParts(shapeParts(1:shapePartCount));
end

%% Section 2: Build The Exact-Boundary Node Set

allPositions_units     = [initialState.position_units; goalState.position_units; proposalShape.Vertices];
coordinateScale_units = bmtpEngine.createCoordinateTolerances(allPositions_units);
% Nodes need room for a bounded-speed path to reverse velocity, but that
% clearance cannot consume a material fraction of a small workspace. Use
% one scale derived from both physical limits and the supplied domain.
turnScale_units       = max(limits.maxVelocity_units_s.^2 ./ limits.maxAcceleration_units_s2);
workspaceScale_units  = max([diff(limits.xInterval_units), diff(limits.yInterval_units)]) / 64;
candidateOffset_units = max([1e-3, 256 * eps(coordinateScale_units), ...
    min(turnScale_units, workspaceScale_units)]);
nodes_units = obstacleAvoidance.search.createTimedVisibilityNodes( ...
    proposalShape, initialState.position_units, goalState.position_units, limits, candidateOffset_units);
stationaryNodes_units = createStationaryIntervalNodes( ...
    obstacles, initialState.time_s, goalState.time_s, limits, candidateOffset_units);
nodes_units = unique([nodes_units(1:2, :); stationaryNodes_units; nodes_units(3:end, :)], ...
    "rows", "stable");

%% Section 3: Search Physical Time Layers

% The staging nodes propose a corridor only. Complete-interval collision
% checks guard every temporal edge before BMTP receives the route.
timedCost_units = hypot(nodes_units(:, 1) - nodes_units(:, 1).', nodes_units(:, 2) - nodes_units(:, 2).');
[route_units, routeTime_s, timedRecord] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
    nodes_units, timedCost_units, obstacles, initialState, goalState, limits, sampleTimes_s, options);
record = struct( ...
    'Nodes_units', nodes_units, ...
    'TimedSearch', timedRecord);
end

%% Section 4: Local Functions

function combinedShape = unionShapeParts(shapeParts)
    % Union the identical exhaustive part set in bounded batches. Polyshape's
    % vector boolean allocates work superlinearly when thousands of mutually
    % overlapping samples are supplied at once; associativity lets a balanced
    % reduction keep each exact boolean operation small.
    maximumBatchSize = 32;
    while numel(shapeParts) > maximumBatchSize
        batchCount = ceil(numel(shapeParts) / maximumBatchSize);
        combinedBatches = cell(batchCount, 1);
        for batchIndex = 1:batchCount
            firstPartIndex = 1 + (batchIndex - 1) * maximumBatchSize;
            finalPartIndex = min(numel(shapeParts), batchIndex * maximumBatchSize);
            combinedBatches{batchIndex} = union([shapeParts{firstPartIndex:finalPartIndex}]);
        end
        shapeParts = combinedBatches;
    end
    combinedShape = union([shapeParts{:}]);
end

function nodes_units = createStationaryIntervalNodes( ...
        obstacles, startTime_s, endTime_s, limits, candidateOffset_units)
    % A swept union can hide a persistent obstacle boundary inside another
    % obstacle's swept area. Retain every exact stationary-interval boundary
    % so the time-expanded graph can enumerate those later-visible detours.
    nodes_units = zeros(0, 2);
    for obstacleIndex = 1:numel(obstacles)
        obstacle   = obstacles(obstacleIndex);
        preparation = obstacle.InternalPreparation;
        if isscalar(obstacle.time_s)
            sampleIndices = 1;
        else
            intervalOverlapsRequest = obstacle.time_s(1:end - 1) < endTime_s & ...
                obstacle.time_s(2:end) > startTime_s;
            intervalIsStationary = preparation.IntervalIsStationary | ...
                (preparation.MatchingTopology & ...
                preparation.IntervalSpeedBound_units_s == 0);
            stationaryIntervalIndices = find( ...
                preparation.IntervalPrepared & intervalIsStationary & intervalOverlapsRequest);
            sampleIndices = unique(stationaryIntervalIndices);
        end
        for sampleIndex = reshape(sampleIndices, 1, [])
            shape = preparation.SampleShapes{sampleIndex};
            if isempty(shape.Vertices)
                continue;
            end
            candidateShape = polybuffer( ...
                shape, candidateOffset_units, "JointType", "miter");
            candidates_units = candidateShape.Vertices;
            insideWorkspace = candidates_units(:, 1) >= limits.xInterval_units(1) & ...
                candidates_units(:, 1) <= limits.xInterval_units(2) & ...
                candidates_units(:, 2) >= limits.yInterval_units(1) & ...
                candidates_units(:, 2) <= limits.yInterval_units(2);
            nodes_units = [nodes_units; candidates_units(insideWorkspace, :)]; %#ok<AGROW>
        end
    end
    nodes_units = unique(nodes_units, "rows", "stable");
end
