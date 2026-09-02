function routeSet = searchRoutes(scene, request, proposal, visibilityGraph)
%% Section 0: Header & Readme
% SYNTAX
%   routeSet = obstacleAvoidance.search.searchRoutes( ...
%       scene, request, proposal, visibilityGraph)
%**************************************************************************
% PURPOSE
%   - Coordinate timed route search and distinct spatial route search.
%   - Return route suggestions and complete search records before seeding.
%**************************************************************************
% INPUTS
%   - scene (scalar prepared-scene struct)
%       Prepared obstacle histories and request horizon.
%   - request (scalar planning-request struct)
%       Normalized states, limits, and resolved planner options.
%   - proposal (scalar proposal-geometry struct)
%       Spatial route-guidance geometry and sample times.
%   - visibilityGraph (scalar visibility-graph struct)
%       Final nodes, edge costs, and obstacle reference points.
%**************************************************************************
% OUTPUTS
%   - routeSet (scalar struct)
%       Timed and spatial routes, route-class patterns, search records,
%       selected search modes, and coverage details. Routes are suggestions
%       and cannot approve a completed obstacle-avoidance motion.
%**************************************************************************
% UNITS
%   - Positions and route lengths are degrees; physical times are seconds.
%**************************************************************************

%% Section 1: Search Complete Input-Derived Time Layers

% Changing obstacle histories may admit waits or time-dependent passages that
% a spatial union hides. Search the exact histories unless proposal work was
% already reduced; that conservative reduction remains explicit in coverage.

obstacles = scene.preparedObstacles;
initialState = request.initialState;
goalState = request.goalState;
limits = request.limits;
options = request.options;
nodePosition_deg = visibilityGraph.NodePosition_deg;
timedRoute_deg = zeros(0, 2);
timedRouteTime_s = zeros(0, 1);
timedRecord = struct();
timedSearchOptions = options;
timedSearchAttempted = false;
timedSearchSuppressionReason = "staticObstacleHistory";
hasChangingHistory = obstacleAvoidance.obstacles.hasChangingHistory( ...
    obstacles, initialState.time_s, goalState.time_s);
if hasChangingHistory && proposal.usedDenseEnvelope
    timedSearchSuppressionReason = "timedQueryWorkLimit";
elseif hasChangingHistory
    timedSearchAttempted = true;
    timedSearchSuppressionReason = "";
    timedCost_deg = hypot( ...
        nodePosition_deg(:, 1) - nodePosition_deg(:, 1).', ...
        nodePosition_deg(:, 2) - nodePosition_deg(:, 2).');
    if options.GoalTimeMode == "balancedArrival"
        % The spatial candidates supply the fast end of the later selection.
        % Preserve final-horizon ancestry here, then remove irrelevant dwell
        % at the goal so this route represents obstacle-dependent timing.
        timedSearchOptions.GoalTimeMode = "fixedArrival";
    end
    [timedRoute_deg, timedRouteTime_s, timedRecord] = ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodePosition_deg, timedCost_deg, obstacles, initialState, ...
        goalState, limits, proposal.sampleTimes_s, timedSearchOptions);
    if options.GoalTimeMode == "balancedArrival"
        [timedRoute_deg, timedRouteTime_s] = trimTerminalGoalDwell( ...
            timedRoute_deg, timedRouteTime_s, proposal.goal_deg);
    end
end

%% Section 2: Search Distinct Spatial Route Classes

% Spatial visibility can suggest obstacle-passing classes that timed search
% did not retain. Reserve seed capacity for a usable timed route, then search
% only the remaining class count using the exact graph and visibility rule.

hasTimedRoute = ~isempty(timedRoute_deg) && ...
    timedRouteTime_s(end) > timedRouteTime_s(1);
maximumClassCount = max( ...
    0, options.MaximumSeedCount - 1 - double(hasTimedRoute));
visibilityFunction = @(first_deg, second_deg) ...
    obstacleAvoidance.search.checkVisibilitySegments( ...
    first_deg, second_deg, proposal.shape, ...
    proposal.edgeStart_deg, proposal.edgeEnd_deg);
[spatialRoutes_deg, routeClassPattern, spatialSearchRecord] = ...
    obstacleAvoidance.search.searchDistinctSpatialRoutes( ...
    visibilityGraph.EdgeCost_deg, nodePosition_deg, ...
    visibilityGraph.ObstacleReferencePoints_deg, maximumClassCount, ...
    visibilityFunction, options);

%% Section 3: Assemble The Route Set

% Keep the search records beside their routes. Seed creation can then remain
% a deterministic conversion stage, while diagnostics and plotters inspect
% the actual search decisions without rerunning either algorithm.

routeSet = struct( ...
    "TimedRoute_deg", timedRoute_deg, ...
    "TimedRouteTime_s", timedRouteTime_s, ...
    "TimedSearchRecord", timedRecord, ...
    "TimedSearchOptions", timedSearchOptions, ...
    "TimedSearchAttempted", timedSearchAttempted, ...
    "TimedSearchSuppressionReason", timedSearchSuppressionReason, ...
    "SpatialRoutes_deg", {spatialRoutes_deg}, ...
    "RouteClassPattern", routeClassPattern, ...
    "SpatialSearchRecord", spatialSearchRecord, ...
    "MaximumSpatialClassCount", maximumClassCount, ...
    "ObstacleReferencePoints_deg", ...
    visibilityGraph.ObstacleReferencePoints_deg, ...
    "UsesReducedGeometry", proposal.usedDenseEnvelope);
end

%% Section 4: Local Functions

function [route_deg, routeTime_s] = trimTerminalGoalDwell( ...
        route_deg, routeTime_s, goal_deg)
% Remove only repeated goal occupancy after the first verified arrival.
if isempty(route_deg)
    return;
end
coordinateScale_deg = bmtpEngine.createCoordinateTolerances( ...
    route_deg, goal_deg);
goalTolerance_deg = 256 * eps(coordinateScale_deg);
firstGoalIndex = find(vecnorm(route_deg - goal_deg, 2, 2) <= ...
    goalTolerance_deg, 1, "first");
if isempty(firstGoalIndex)
    return;
end
route_deg = route_deg(1:firstGoalIndex, :);
routeTime_s = routeTime_s(1:firstGoalIndex);
end
