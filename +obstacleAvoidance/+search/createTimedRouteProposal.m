function [route_units, routeTime_s, record] = createTimedRouteProposal(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [route_units,routeTime_s,record] = ...
%       obstacleAvoidance.search.createTimedRouteProposal( ...
%       obstacles,initialState,goalState,limits,options)
% PURPOSE
%   Build a deterministic time-expanded route proposal for moving obstacles.
%   The returned route initializes BMTP and cannot approve a final motion.
% INPUTS
%   Prepared obstacles, normalized endpoint states, limits, and options.
% OUTPUTS
%   Timed route arrays, or empty arrays on exhaustion, plus search evidence.
% UNITS
%   Position is coordinate units and time is seconds.

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

%% Section 2: Find A Connected Offset Node Set
allPositions_units = [initialState.position_units; ...
    goalState.position_units; proposalShape.Vertices];
coordinateScale_units = bmtpEngine.createCoordinateTolerances(allPositions_units);
candidateOffset_units = max(1e-3, 256 * eps(coordinateScale_units));
maximumOffset_units = max([diff(limits.xInterval_units), diff(limits.yInterval_units)]);
workBudget = 1e6;
offsetRetryCount = 0;
while true
    attempt = obstacleAvoidance.search.createVisibilityAttempt( ...
        proposalShape, initialState.position_units, goalState.position_units, ...
        limits, candidateOffset_units, offsetRetryCount, workBudget);
    if offsetRetryCount == 0
        attempts = attempt;
    else
        attempts(end + 1, 1) = attempt; %#ok<AGROW>
    end
    if attempt.IsConnected || candidateOffset_units >= maximumOffset_units
        break;
    end
    candidateOffset_units = min(4 * candidateOffset_units, maximumOffset_units);
    offsetRetryCount = offsetRetryCount + 1;
end

%% Section 3: Search Physical Time Layers
nodes_units = attempts(end).Nodes.Positions_units;
timedCost_units = hypot(nodes_units(:,1) - nodes_units(:,1).', ...
    nodes_units(:,2) - nodes_units(:,2).');
[route_units,routeTime_s,timedRecord] = ...
    obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
    nodes_units,timedCost_units,obstacles,initialState,goalState,limits,sampleTimes_s,options);
record = struct('ProposalShape',proposalShape,'SampleTimes_s',sampleTimes_s, ...
    'Attempts',attempts,'CandidateOffset_units',candidateOffset_units, ...
    'OffsetRetryCount',offsetRetryCount,'TimedSearch',timedRecord);
end
