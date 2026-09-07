function [feasible, message, reason] = validatePlannerEndpoints(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [feasible, message, reason] = ...
%       obstacleAvoidance.input.validatePlannerEndpoints( ...
%       obstacles, initialState, goalState, limits, options)
%
% PURPOSE
%   - Reject endpoint geometry, dynamics, timing, or workspace failures
%     before route search or trajectory optimization begins.
%
% INPUTS
%   - obstacles (canonical protected-obstacle array)
%   - initialState, goalState, limits (normalized scalar structs)
%   - options (scalar struct)
%       Requires GoalTimeMode, ArrivalTimeTolerance_s, and
%       WrapX and WrapY.
%
% OUTPUTS
%   - feasible (logical scalar)
%   - message, reason (string scalars)
%       Empty on success; otherwise actionable and machine-readable failure.
%
% UNITS
%   - Position and workspace intervals are coordinate units; time is seconds;
%     derivatives use units/s and units/s^2.
%

%% Section 1: Check Protected Endpoint Geometry

goalPosition_units  = obstacleAvoidance.input.goalPositionAtTime(goalState, goalState.time_s);
startIsBlocked    = obstacleAvoidance.obstacles.queryPreparedObstacles(obstacles, initialState.position_units(1), initialState.position_units(2), initialState.time_s);
terminalIsBlocked = false;
if options.GoalTimeMode == "fixedArrival"
    terminalIsBlocked = obstacleAvoidance.obstacles.queryPreparedObstacles(obstacles, goalPosition_units(1), goalPosition_units(2), goalState.time_s);
end
if startIsBlocked || terminalIsBlocked
    [feasible, message, reason] = failure("The protected geometry contains the start or fixed terminal point.", "endpointBlocked");
    return;
end

%% Section 2: Check Endpoint Dynamics And Timing

derivatives   = [initialState.velocity_units_s; goalState.velocity_units_s];
accelerations = [initialState.acceleration_units_s2; ...
    goalState.acceleration_units_s2];
if any(abs(derivatives) > limits.maxVelocity_units_s, "all") || any(abs(accelerations) > limits.maxAcceleration_units_s2, "all")
    [feasible, message, reason] = failure("An endpoint derivative exceeds its physical limit.", "dynamicEndpointInfeasible");
    return;
end
hasMovingGoal             = isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s);
checkTerminal             = options.GoalTimeMode == "fixedArrival" || ~hasMovingGoal;
availableDuration_s       = goalState.time_s - initialState.time_s;
minimumVelocityDuration_s = max(abs(goalPosition_units - initialState.position_units) ./ limits.maxVelocity_units_s);
if checkTerminal && minimumVelocityDuration_s > availableDuration_s + options.ArrivalTimeTolerance_s
    message = sprintf("The time window is too short for the endpoint displacement " + "at the configured velocity limits (minimum %.6g s, " + "available %.6g s). Increase goalState.time_s or the " + "velocity limits.", minimumVelocityDuration_s, availableDuration_s);
    [feasible, message, reason] = failure(message, "timeWindowInfeasible");
    return;
end

%% Section 3: Check The Workspace

endpointPosition_units = initialState.position_units;
if checkTerminal
    endpointPosition_units(2, :) = goalPosition_units;
end
positionWithinBounds = true;
wrapAxes = [options.WrapX, options.WrapY];
intervals_units = [limits.xInterval_units; limits.yInterval_units];
for axisIndex = find(~wrapAxes)
    positionWithinBounds = positionWithinBounds && all(endpointPosition_units(:, axisIndex) >= intervals_units(axisIndex, 1) & endpointPosition_units(:, axisIndex) <= intervals_units(axisIndex, 2));
end
if ~positionWithinBounds
    [feasible, message, reason] = failure("An endpoint is outside the configured workspace.", "endpointOutsideWorkspace");
    return;
end
feasible = true;
message  = "";
reason   = "";
end

%% Section 4: Local Functions

function [feasible, message, reason] = failure(message, reason)
    % Return an expected failure without throwing an error.
    feasible = false;
end
