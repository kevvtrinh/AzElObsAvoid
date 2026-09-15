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
    proposalShape = union([shapeParts{1:shapePartCount}]);
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
