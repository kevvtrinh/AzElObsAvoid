function [feasible, message, reason] = validatePlannerEndpoints( ...
    obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [feasible, message, reason] = ...
%       azElInput.validatePlannerEndpoints( ...
%       obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Reject endpoint geometry, dynamics, timing, or workspace failures
%     before a motion method performs search or optimization.
%**************************************************************************
% INPUTS
%   - obstacles (canonical protected-obstacle array)
%   - initialState (normalized scalar struct)
%   - goalState (normalized scalar struct)
%   - limits (normalized scalar struct)
%   - options (scalar struct)
%       Requires GoalTimeMode, ArrivalTimeTolerance_s, and
%       AllowAzimuthWrapping.
%**************************************************************************
% OUTPUTS
%   - feasible (logical scalar)
%       True only when every shared endpoint check passes.
%   - message (string scalar)
%       Empty on success; otherwise the established actionable failure.
%   - reason (string scalar)
%       Empty on success; otherwise the established machine-readable reason.
%**************************************************************************
% UNITS
%   - Position and workspace intervals are degrees; time is seconds;
%     derivatives use deg/s and deg/s^2.
%**************************************************************************

%% Section 1: Check Protected Endpoint Geometry

goalPosition_deg = azElInput.goalPositionAtTime( ...
    goalState, goalState.time_s);
startIsBlocked = azElObstacles.queryAzElTimeObstacle( ...
    obstacles, initialState.position_deg(1), ...
    initialState.position_deg(2), initialState.time_s);
fixedTerminalIsBlocked = false;
if options.GoalTimeMode == "fixedArrival"
    fixedTerminalIsBlocked = azElObstacles.queryAzElTimeObstacle( ...
        obstacles, goalPosition_deg(1), goalPosition_deg(2), ...
        goalState.time_s);
end
if startIsBlocked || fixedTerminalIsBlocked
    feasible = false;
    message = ...
        "The protected geometry contains the start or fixed terminal point.";
    reason = "endpointBlocked";
    return;
end

%% Section 2: Check Endpoint Dynamics And Timing

derivativesWithinLimits = ...
    all(abs(initialState.velocity_deg_s) <= limits.maxVelocity_deg_s) && ...
    all(abs(goalState.velocity_deg_s) <= limits.maxVelocity_deg_s) && ...
    all(abs(initialState.acceleration_deg_s2) <= ...
    limits.maxAcceleration_deg_s2) && ...
    all(abs(goalState.acceleration_deg_s2) <= ...
    limits.maxAcceleration_deg_s2);
if ~derivativesWithinLimits
    feasible = false;
    message = "An endpoint derivative exceeds its physical limit.";
    reason = "dynamicEndpointInfeasible";
    return;
end
hasMovingGoal = isfield(goalState, "targetTime_s") && ...
    ~isempty(goalState.targetTime_s);
availableDuration_s = goalState.time_s - initialState.time_s;
if options.GoalTimeMode == "fixedArrival" || ~hasMovingGoal
    endpointDisplacement_deg = ...
        abs(goalPosition_deg - initialState.position_deg);
    minimumVelocityDuration_s = ...
        max(endpointDisplacement_deg ./ limits.maxVelocity_deg_s);
    if minimumVelocityDuration_s > ...
            availableDuration_s + options.ArrivalTimeTolerance_s
        feasible = false;
        message = sprintf( ...
            "The time window is too short for the endpoint displacement " + ...
            "at the configured velocity limits (minimum %.6g s, " + ...
            "available %.6g s). Increase goalState.time_s or the " + ...
            "velocity limits.", ...
            minimumVelocityDuration_s, availableDuration_s);
        reason = "timeWindowInfeasible";
        return;
    end
end

%% Section 3: Check The Workspace

endpointPosition_deg = initialState.position_deg;
if options.GoalTimeMode == "fixedArrival" || ~hasMovingGoal
    endpointPosition_deg(end + 1, :) = goalPosition_deg;
end
positionWithinBounds = ...
    all(endpointPosition_deg(:, 2) >= limits.elevationInterval_deg(1)) && ...
    all(endpointPosition_deg(:, 2) <= limits.elevationInterval_deg(2));
if ~options.AllowAzimuthWrapping
    positionWithinBounds = positionWithinBounds && ...
        all(endpointPosition_deg(:, 1) >= ...
        limits.azimuthInterval_deg(1)) && ...
        all(endpointPosition_deg(:, 1) <= ...
        limits.azimuthInterval_deg(2));
end
if ~positionWithinBounds
    feasible = false;
    message = "An endpoint is outside the configured workspace.";
    reason = "endpointOutsideWorkspace";
    return;
end
feasible = true;
message = "";
reason = "";
end
