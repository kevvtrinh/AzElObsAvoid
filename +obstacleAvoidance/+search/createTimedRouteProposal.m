function [route_units, routeTime_s, record] = createTimedRouteProposal(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX: [route_units,routeTime_s,record] = obstacleAvoidance.search.createTimedRouteProposal(
%   obstacles,initialState,goalState,limits,options)
% PURPOSE: Build a deterministic time-expanded route proposal for moving obstacles. The returned
%   route initializes BMTP and cannot approve a final motion.
% INPUTS: Prepared obstacles, normalized endpoint states, limits, and options.
% OUTPUTS: Timed route arrays, or empty arrays on exhaustion, plus search evidence.
% UNITS: Position is coordinate units and time is seconds.

%% Section 1: Build The Sampled Swept Proposal
obstacles = obstacleAvoidance.obstacles.prepareObstacles( ...
    obstacles,[initialState.time_s,goalState.time_s]);
sampleTimes_s = obstacleAvoidance.search.createTimeLayers( ...
    obstacles, initialState.time_s, goalState.time_s);
parts = cell(numel(sampleTimes_s) * numel(obstacles), 1);
partCount = 0;
for timeIndex = 1:numel(sampleTimes_s)
    for obstacleIndex = 1:numel(obstacles)
        shape = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
            obstacles(obstacleIndex), sampleTimes_s(timeIndex));
        if isempty(shape.Vertices), continue; end
        partCount = partCount + 1;
        parts{partCount} = shape;
    end
end
proposalShape = polyshape();
if partCount > 0
    proposalShape = union([parts{1:partCount}]);
end

%% Section 2: Build The Exact-Boundary Node Set
allPositions_units = [initialState.position_units; ...
    goalState.position_units; proposalShape.Vertices];
coordinateScale_units = bmtpEngine.createCoordinateTolerances(allPositions_units);
% Nodes need room for a bounded-speed path to reverse velocity, but that
% clearance cannot consume a material fraction of a small workspace. Use
% one scale derived from both physical limits and the supplied domain.
turnScale_units=max(limits.maxVelocity_units_s.^2 ./ ...
    limits.maxAcceleration_units_s2);
workspaceScale_units=max([diff(limits.xInterval_units), ...
    diff(limits.yInterval_units)])/64;
candidateOffset_units=max([1e-3,256*eps(coordinateScale_units), ...
    min(turnScale_units,workspaceScale_units)]);
nodes_units = obstacleAvoidance.search.createTimedVisibilityNodes( ...
    proposalShape, initialState.position_units, goalState.position_units, ...
    limits, candidateOffset_units);

%% Section 3: Search Physical Time Layers
% The staging nodes propose a corridor only. Complete-interval collision
% checks guard every temporal edge before BMTP receives the route.
timedCost_units = hypot(nodes_units(:,1) - nodes_units(:,1).', ...
    nodes_units(:,2) - nodes_units(:,2).');
[route_units,routeTime_s,timedRecord] = ...
    obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
    nodes_units,timedCost_units,obstacles,initialState,goalState, ...
    limits,sampleTimes_s,options);
record = struct('Nodes_units',nodes_units,'TimedSearch',timedRecord);
end
