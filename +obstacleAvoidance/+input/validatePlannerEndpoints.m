function [feasible,message,reason] = validatePlannerEndpoints(obstacles,initialState,goalState,limits,options)
%% Section 0: Header & Readme
% SYNTAX: [ok,message,reason] = obstacleAvoidance.input.validatePlannerEndpoints(obstacles,initial,goal,limits,options)
% PURPOSE: Reject physical endpoint violations at their actual physical times.
% INPUTS: Prepared geometry, normalized full states, limits, and arrival policy.
% OUTPUTS: Feasibility and stable expected-failure description.
% UNITS: Coordinate units, seconds, and physical derivatives.

%% Section 1: Check Occupancy At The Requested Endpoint Times
feasible = false; message = ""; reason = "";
positions = initialState.position_units;
times = initialState.time_s;
if options.GoalTimeMode=="fixedArrival"
    positions(2,:) = goalState.position_units; times(2,1) = goalState.time_s;
end
if any(obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(obstacles,positions(:,1),positions(:,2),times))
    message = "A protected obstacle occupies the initial or fixed terminal state.";
    reason = "endpointBlocked"; return;
end

%% Section 2: Check Physical States And Necessary Travel Time
checkGoal = options.GoalTimeMode=="fixedArrival" || isempty(goalState.targetMotion);
states = initialState;
if checkGoal
    states(2).position_units = goalState.position_units;
    states(2).velocity_units_s = goalState.velocity_units_s;
    states(2).acceleration_units_s2 = goalState.acceleration_units_s2;
end
intervals = [limits.xInterval_units;limits.yInterval_units];
for k = 1:numel(states)
    if any(abs(states(k).velocity_units_s)>limits.maxVelocity_units_s) || ...
            any(abs(states(k).acceleration_units_s2)>limits.maxAcceleration_units_s2)
        message = "An endpoint derivative exceeds its physical limit.";
        reason = "dynamicEndpointInfeasible"; return;
    end
    p = states(k).position_units;
    if any(p<intervals(:,1).' | p>intervals(:,2).')
        message = "An endpoint lies outside the workspace.";
        reason = "endpointOutsideWorkspace"; return;
    end
end
if checkGoal && max(abs(goalState.position_units-initialState.position_units)./limits.maxVelocity_units_s)> ...
        goalState.time_s-initialState.time_s+options.ArrivalTimeTolerance_s
    message = "The displacement exceeds the velocity-limited travel distance within the horizon.";
    reason = "timeWindowInfeasible"; return;
end
feasible = true;
end
