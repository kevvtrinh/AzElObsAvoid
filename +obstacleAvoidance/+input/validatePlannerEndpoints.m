function [feasible, message, reason] = validatePlannerEndpoints( ...
    obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [feasible, message, reason] = ...
%       obstacleAvoidance.input.validatePlannerEndpoints( ...
%       obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Reject endpoint geometry, dynamics, timing, or workspace failures
%     before route search or trajectory optimization begins.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected-obstacle array)
%   - initialState, goalState, limits (normalized scalar structs)
%   - options (scalar struct)
%       Requires GoalTimeMode, ArrivalTimeTolerance_s, and
%       AllowAzimuthWrapping.
%**************************************************************************
% OUTPUTS
%   - feasible (logical scalar)
%   - message, reason (string scalars)
%       Empty on success; otherwise actionable and machine-readable failure.
%**************************************************************************
% UNITS
%   - Position and workspace intervals are degrees; time is seconds;
%     derivatives use deg/s and deg/s^2.
%**************************************************************************

%% Section 1: Check Protected Endpoint Geometry

goalPosition_deg = obstacleAvoidance.input.goalPositionAtTime( ...
    goalState, goalState.time_s);
startIsBlocked = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacles, initialState.position_deg(1), initialState.position_deg(2), ...
    initialState.time_s);
terminalIsBlocked = false;
if options.GoalTimeMode == "fixedArrival"
    terminalIsBlocked = ...
        obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        obstacles, goalPosition_deg(1), goalPosition_deg(2), ...
        goalState.time_s);
end
if startIsBlocked || terminalIsBlocked
    [feasible, message, reason] = failure( ...
        "The protected geometry contains the start or fixed terminal point.", ...
        "endpointBlocked");
    return;
end

%% Section 2: Check Endpoint Dynamics And Timing

derivatives = [initialState.velocity_deg_s; goalState.velocity_deg_s];
accelerations = [initialState.acceleration_deg_s2; ...
    goalState.acceleration_deg_s2];
if any(abs(derivatives) > limits.maxVelocity_deg_s, "all") || ...
        any(abs(accelerations) > limits.maxAcceleration_deg_s2, "all")
    [feasible, message, reason] = failure( ...
        "An endpoint derivative exceeds its physical limit.", ...
        "dynamicEndpointInfeasible");
    return;
end
hasMovingGoal = isfield(goalState, "targetTime_s") && ...
    ~isempty(goalState.targetTime_s);
checkTerminal = options.GoalTimeMode == "fixedArrival" || ~hasMovingGoal;
availableDuration_s = goalState.time_s - initialState.time_s;
minimumVelocityDuration_s = max(abs(goalPosition_deg - ...
    initialState.position_deg) ./ limits.maxVelocity_deg_s);
if checkTerminal && minimumVelocityDuration_s > ...
        availableDuration_s + options.ArrivalTimeTolerance_s
    message = sprintf( ...
        "The time window is too short for the endpoint displacement " + ...
        "at the configured velocity limits (minimum %.6g s, " + ...
        "available %.6g s). Increase goalState.time_s or the " + ...
        "velocity limits.", minimumVelocityDuration_s, availableDuration_s);
    [feasible, message, reason] = failure(message, "timeWindowInfeasible");
    return;
end

%% Section 3: Check The Workspace

endpointPosition_deg = initialState.position_deg;
if checkTerminal
    endpointPosition_deg(2, :) = goalPosition_deg;
end
positionWithinBounds = all( ...
    endpointPosition_deg(:, 2) >= limits.elevationInterval_deg(1) & ...
    endpointPosition_deg(:, 2) <= limits.elevationInterval_deg(2));
if ~options.AllowAzimuthWrapping
    positionWithinBounds = positionWithinBounds && all( ...
        endpointPosition_deg(:, 1) >= limits.azimuthInterval_deg(1) & ...
        endpointPosition_deg(:, 1) <= limits.azimuthInterval_deg(2));
end
if ~positionWithinBounds
    [feasible, message, reason] = failure( ...
        "An endpoint is outside the configured workspace.", ...
        "endpointOutsideWorkspace");
    return;
end
feasible = true;
message = "";
reason = "";
end

%% Section 4: Local Functions

function [feasible, message, reason] = failure(message, reason)
% Return the stable expected-failure triple without throwing.
feasible = false;
end
