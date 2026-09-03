function routeSet = searchRoutes( ...
        scene, request, proposal, visibilityGraph, priorRouteSet)
%% Section 0: Header & Readme
% SYNTAX
%   routeSet = obstacleAvoidance.search.searchRoutes( ...
%       scene, request, proposal, visibilityGraph)
%   routeSet = obstacleAvoidance.search.searchRoutes( ...
%       scene, request, proposal, visibilityGraph, priorRouteSet)
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
%   - priorRouteSet (scalar route-set struct, optional)
%       Initial deferred result to resume with exact timed search. Its
%       spatial routes and search record are reused without recomputation.
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
% a spatial union hides. Defer the expensive exact-history search for a dense
% proposal until the caller confirms that every cheap motion attempt failed.
% A fifth-input call resumes only that timed work and returns before spatial
% search, so recovery cannot repeat graph or route-class work.

isTimedRecovery = nargin >= 5 && ~isempty(priorRouteSet);
if isTimedRecovery && (~isstruct(priorRouteSet) || ...
        ~isscalar(priorRouteSet) || ...
        ~isfield(priorRouteSet, "TimedSearchDeferred") || ...
        ~priorRouteSet.TimedSearchDeferred)
    error("searchRoutes:InvalidRecoveryState", ...
        "priorRouteSet must be a deferred scalar route-set record.");
end

obstacles = scene.preparedObstacles;
initialState = request.initialState;
goalState = request.goalState;
options = request.options;
nodePosition_deg = visibilityGraph.NodePosition_deg;
timedRoute_deg = zeros(0, 2);
timedRouteTime_s = zeros(0, 1);
timedRecord = struct();
timedSearchOptions = options;
timedSearchAttempted = false;
timedSearchDeferred = false;
timedSearchSuppressionReason = "staticObstacleHistory";
hasChangingHistory = obstacleAvoidance.obstacles.hasChangingHistory( ...
    obstacles, initialState.time_s, goalState.time_s);
if hasChangingHistory && proposal.usedDenseEnvelope && ~isTimedRecovery
    timedSearchDeferred = true;
    timedSearchSuppressionReason = "deferredDenseTimedSearch";
elseif hasChangingHistory
    timedSearchAttempted = true;
    timedSearchSuppressionReason = "";
    timedCost_deg = hypot( ...
        nodePosition_deg(:, 1) - nodePosition_deg(:, 1).', ...
        nodePosition_deg(:, 2) - nodePosition_deg(:, 2).');
    if options.GoalTimeMode == "balancedArrival"
        % Preserve final-horizon ancestry, then remove goal dwell so this
        % route represents obstacle-dependent timing rather than waiting.
        timedSearchOptions.GoalTimeMode = "fixedArrival";
    end
    [timedRoute_deg, timedRouteTime_s, timedRecord] = ...
        obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
        nodePosition_deg, timedCost_deg, obstacles, initialState, ...
        goalState, request.limits, proposal.sampleTimes_s, ...
        timedSearchOptions);
    if options.GoalTimeMode == "balancedArrival"
        [timedRoute_deg, timedRouteTime_s] = trimTerminalGoalDwell( ...
            timedRoute_deg, timedRouteTime_s, proposal.goal_deg);
    end
end
if isTimedRecovery
    routeSet = priorRouteSet;
    routeSet.TimedRoute_deg = timedRoute_deg;
    routeSet.TimedRouteTime_s = timedRouteTime_s;
    routeSet.TimedSearchRecord = timedRecord;
    routeSet.TimedSearchOptions = timedSearchOptions;
    routeSet.TimedSearchAttempted = true;
    routeSet.TimedSearchRecoveryAttempted = true;
    routeSet.TimedSearchSuppressionReason = "";
    return;
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
    "TimedSearchDeferred", timedSearchDeferred, ...
    "TimedSearchRecoveryAttempted", false, ...
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
