function [route_units, routeTime_s, timedRouteDetails] = createTimedRouteProposal( ...
    obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [route_units, routeTime_s, timedRouteDetails] = ...
%       obstacleAvoidance.search.createTimedRouteProposal( ...
%       obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Propose route points and times around moving obstacles. The search
%     checks where the vehicle can travel as the obstacle shapes change.
%     BMTP then uses the route as a starting point for complete vehicle motion.
%**************************************************************************
% INPUTS
%   - obstacles (prepared obstacle array)
%       Protected geometry already prepared over at least the request
%       time range. The timed search prepares its own working copy.
%   - initialState (scalar struct)
%       Normalized initial endpoint state.
%   - goalState (scalar struct)
%       Normalized goal endpoint state.
%   - limits (scalar struct)
%       Checked workspace, speed, acceleration, and jerk limits.
%   - options (scalar struct)
%       Normalized timed-search options.
%**************************************************************************
% OUTPUTS
%   - route_units (N-by-2 numeric array)
%       Route positions, or an empty array if the search finds no route.
%   - routeTime_s (N-by-1 numeric array)
%       Time at each route point, or an empty array if the search finds no route.
%   - timedRouteDetails (scalar struct)
%       Candidate nodes and search details. Finding no route is a normal outcome;
%       invalid input throws an error.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Combine Obstacle Shapes At The Selected Times

% Use obstacle sample times, interval midpoints, and times across the
% requested trip to collect shapes for choosing candidate route positions.
searchTimes_s = obstacleAvoidance.search.createTimeLayers( ...
    obstacles, initialState.time_s, goalState.time_s);
sampleShapes     = cell(numel(searchTimes_s) * numel(obstacles), 1);
sampleShapeCount = 0;
for timeIndex = 1:numel(searchTimes_s)
    for obstacleIndex = 1:numel(obstacles)
        obstacleShape = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
            obstacles(obstacleIndex), searchTimes_s(timeIndex));
        if isempty(obstacleShape.Vertices)
            continue;
        end
        sampleShapeCount = sampleShapeCount + 1;
        sampleShapes{sampleShapeCount} = obstacleShape;
    end
end
combinedSampleShape = polyshape();
if sampleShapeCount > 0
    combinedSampleShape = unionShapeParts(sampleShapes(1:sampleShapeCount));
end

%% Section 2: Place Route Points Around Obstacle Boundaries

allPositions_units    = [initialState.position_units; goalState.position_units; combinedSampleShape.Vertices];
coordinateScale_units = bmtpEngine.validation.createCoordinateTolerances(allPositions_units);
% Place candidate points away from corners to give BMTP room to turn.
% turning distance scale = maximum speed^2 / maximum acceleration.
% Limit that contribution using the workspace size, with a small minimum
% offset for numerical separation. This places points; it does not change
% the protected obstacle shapes used for collision checks.
turningDistanceScale_units = max(limits.maxVelocity_units_s .^ 2 ./ limits.maxAcceleration_units_s2);
workspaceOffsetScale_units = max([diff(limits.xInterval_units), diff(limits.yInterval_units)]) / 64;
candidateOffset_units      = max([1e-3, 256 * eps(coordinateScale_units), ...
    min(turningDistanceScale_units, workspaceOffsetScale_units)]);
nodes_units = obstacleAvoidance.search.createTimedVisibilityNodes( ...
    combinedSampleShape, initialState.position_units, goalState.position_units, limits, candidateOffset_units);
% The combined shapes can hide a stationary boundary that becomes useful
% after another obstacle moves away. Include its candidate points as well.
stationaryNodes_units = createStationaryIntervalNodes( ...
    obstacles, initialState.time_s, goalState.time_s, limits, candidateOffset_units);
nodes_units = unique([nodes_units(1:2, :); stationaryNodes_units; nodes_units(3:end, :)], ...
    "rows", "stable");

%% Section 3: Find Clear Connections At Successive Times

% The search checks connections for collisions throughout their travel time.
% A found route gives BMTP positions and times to work from; BMTP and the
% independent validator still determine whether the complete motion is valid.
% Matrix entry (i, j) is the straight-line distance from node i to node j.
connectionLength_units = hypot( ...
    nodes_units(:, 1) - nodes_units(:, 1).', nodes_units(:, 2) - nodes_units(:, 2).');
[route_units, routeTime_s, timedSearchDetails] = obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
    nodes_units, connectionLength_units, obstacles, initialState, goalState, limits, searchTimes_s, options);
timedRouteDetails = struct( ...
    'Nodes_units', nodes_units, ...
    'TimedSearch', timedSearchDetails);
end

%% Section 4: Local Functions

function combinedShape = unionShapeParts(sampleShapes)
    % Combine all shapes in small batches to limit temporary memory use.
    % Every supplied shape is included; only the grouping of unions changes.
    maximumBatchSize = 32;
    while numel(sampleShapes) > maximumBatchSize
        batchCount      = ceil(numel(sampleShapes) / maximumBatchSize);
        combinedBatches = cell(batchCount, 1);
        for batchIndex = 1:batchCount
            firstPartIndex = 1 + (batchIndex - 1) * maximumBatchSize;
            finalPartIndex = min(numel(sampleShapes), batchIndex * maximumBatchSize);
            combinedBatches{batchIndex} = union([sampleShapes{firstPartIndex:finalPartIndex}]);
        end
        sampleShapes = combinedBatches;
    end
    combinedShape = union([sampleShapes{:}]);
end

function nodes_units = createStationaryIntervalNodes( ...
        obstacles, startTime_s, endTime_s, limits, candidateOffset_units)
    % Combining shapes across time can hide a stationary boundary behind
    % another obstacle's occupied area at a different time. Add points around
    % that boundary because those routes may become clear later in the trip.
    nodes_units = zeros(0, 2);
    for obstacleIndex = 1:numel(obstacles)
        obstacle    = obstacles(obstacleIndex);
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
            % A stationary interval keeps its starting sample's shape.
            sampleIndices = unique(stationaryIntervalIndices);
        end
        for sampleIndex = reshape(sampleIndices, 1, [])
            obstacleShape = preparation.SampleShapes{sampleIndex};
            if isempty(obstacleShape.Vertices)
                continue;
            end
            % Offset only the candidate points; keep the obstacle geometry
            % unchanged for the later collision checks.
            offsetShape = polybuffer( ...
                obstacleShape, candidateOffset_units, "JointType", "miter");
            candidatePositions_units   = offsetShape.Vertices;
            candidateIsInsideWorkspace = candidatePositions_units(:, 1) >= limits.xInterval_units(1) & ...
                candidatePositions_units(:, 1) <= limits.xInterval_units(2) & ...
                candidatePositions_units(:, 2) >= limits.yInterval_units(1) & ...
                candidatePositions_units(:, 2) <= limits.yInterval_units(2);
            nodes_units = [nodes_units; candidatePositions_units(candidateIsInsideWorkspace, :)]; %#ok<AGROW>
        end
    end
    nodes_units = unique(nodes_units, "rows", "stable");
end
