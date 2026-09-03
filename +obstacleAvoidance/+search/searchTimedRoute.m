function timedSearch = searchTimedRoute( ...
        scene, request, proposal, visibilityGraph)
%% Section 0: Header & Readme
% SYNTAX
%   timedSearch = obstacleAvoidance.search.searchTimedRoute( ...
%       scene, request, proposal, visibilityGraph)
%**************************************************************************
% PURPOSE
%   - Search exact obstacle histories over the retained input-derived time
%     layers and visibility nodes.
%**************************************************************************
% INPUTS
%   - scene (scalar prepared-scene struct)
%       Prepared obstacle histories and request horizon.
%   - request (scalar planning-request struct)
%       Normalized states, limits, and resolved planner options.
%   - proposal (scalar proposal-geometry struct)
%       Input-derived time samples and resolved goal position.
%   - visibilityGraph (scalar visibility-graph struct)
%       Retained node positions used by the timed graph.
%**************************************************************************
% OUTPUTS
%   - timedSearch (scalar struct)
%       Timed route, absolute route times, search record, resolved timed
%       options. The route remains only a proposal.
%**************************************************************************
% UNITS
%   - Positions and edge costs are degrees; physical times are seconds.
%**************************************************************************

%% Section 1: Search The Exact Obstacle Histories

% Proposal geometry supplies reusable nodes and event times only. Timed edges
% query the prepared histories directly, so reduced spatial geometry cannot
% weaken collision meaning in this search.

obstacles = scene.preparedObstacles;
initialState = request.initialState;
goalState = request.goalState;
timedOptions = request.options;
nodePosition_deg = visibilityGraph.NodePosition_deg;
timedCost_deg = hypot( ...
    nodePosition_deg(:, 1) - nodePosition_deg(:, 1).', ...
    nodePosition_deg(:, 2) - nodePosition_deg(:, 2).');
if timedOptions.GoalTimeMode == "balancedArrival"
    % Preserve final-horizon ancestry, then remove goal dwell so this route
    % represents obstacle-dependent timing rather than artificial waiting.
    timedOptions.GoalTimeMode = "fixedArrival";
end
[route_deg, routeTime_s, record] = ...
    obstacleAvoidance.search.timeExpandedVisibilitySearch( ...
    nodePosition_deg, timedCost_deg, obstacles, initialState, goalState, ...
    request.limits, proposal.sampleTimes_s, timedOptions);
if request.options.GoalTimeMode == "balancedArrival"
    [route_deg, routeTime_s] = trimTerminalGoalDwell( ...
        route_deg, routeTime_s, proposal.goal_deg);
end

%% Section 2: Assemble Timed Search Evidence

timedSearch = struct( ...
    "Route_deg", route_deg, ...
    "RouteTime_s", routeTime_s, ...
    "Record", record, ...
    "Options", timedOptions);
end

%% Section 3: Local Functions

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
