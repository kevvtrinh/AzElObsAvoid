function [feasible, message, reason] = validatePlannerEndpoints(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [feasible, message, reason] = ...
%       obstacleAvoidance.input.validatePlannerEndpoints(obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Reject physical endpoint violations at their actual times.
%**************************************************************************
% INPUTS
%   - obstacles (prepared obstacle array)
%       Protected geometry over the request horizon.
%   - initialState (scalar struct)
%       Normalized initial endpoint state.
%   - goalState (scalar struct)
%       Normalized goal endpoint state.
%   - limits (scalar struct)
%       Normalized workspace and derivative limits.
%   - options (scalar struct)
%       Normalized arrival and wrapping policy.
%**************************************************************************
% OUTPUTS
%   - feasible (logical scalar)
%       True when all endpoint checks pass; expected physical failures
%       return false and invalid input throws an error.
%   - message (string scalar)
%       Stable expected-failure description.
%   - reason (string scalar)
%       Stable expected-failure identifier.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Check Occupancy At The Requested Endpoint Times

feasible                = false;
message                 = "";
reason                  = "";
endpointPositions_units = initialState.position_units;
endpointTimes_s           = initialState.time_s;
arrivalIsFixed            = options.GoalTimeMode == "fixedArrival";
if arrivalIsFixed
    endpointPositions_units(2, :) = goalState.position_units;
    endpointTimes_s(2, 1)         = goalState.time_s;
end
endpointIsOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacles, endpointPositions_units(:, 1), endpointPositions_units(:, 2), endpointTimes_s);
if any(endpointIsOccupied)
    message = "A protected obstacle occupies the initial or fixed terminal state.";
    reason  = "endpointBlocked";
    return
end
reachabilityShouldBeChecked = arrivalIsFixed && ~options.WrapX && ~options.WrapY;
if reachabilityShouldBeChecked && terminalReachabilityIsBlocked(obstacles, initialState, goalState, limits)
    message = "Every jerk-limited approach to the fixed terminal state intersects a protected obstacle.";
    reason  = "terminalReachabilityBlocked";
    return
end

%% Section 2: Check Physical States And Necessary Travel Time

goalShouldBeChecked = arrivalIsFixed || isempty(goalState.targetMotion);
endpointStates      = initialState;
if goalShouldBeChecked
    endpointStates(2).position_units        = goalState.position_units;
    endpointStates(2).velocity_units_s      = goalState.velocity_units_s;
    endpointStates(2).acceleration_units_s2 = goalState.acceleration_units_s2;
end
intervals = [limits.xInterval_units; limits.yInterval_units];
for stateIndex = 1:numel(endpointStates)
    velocityExceedsLimit = any(abs(endpointStates(stateIndex).velocity_units_s) > limits.maxVelocity_units_s);
    derivativeExceedsLimit = velocityExceedsLimit;
    if ~derivativeExceedsLimit
        accelerationExceedsLimit = any( ...
            abs(endpointStates(stateIndex).acceleration_units_s2) > limits.maxAcceleration_units_s2);
        derivativeExceedsLimit = accelerationExceedsLimit;
    end
    if derivativeExceedsLimit
        message = "An endpoint derivative exceeds its physical limit.";
        reason  = "dynamicEndpointInfeasible";
        return
    end
    position_units             = endpointStates(stateIndex).position_units;
    positionIsOutsideWorkspace = position_units < intervals(:, 1).' | position_units > intervals(:, 2).';
    if any(positionIsOutsideWorkspace)
        message = "An endpoint lies outside the workspace.";
        reason  = "endpointOutsideWorkspace";
        return
    end
    localVelocity_units_s = endpointStates(stateIndex).velocity_units_s;
    if stateIndex > 1
        % At a terminal state, inspect the trajectory backward from arrival.
        localVelocity_units_s = -localVelocity_units_s;
    end
    positionIsOnLowerBound   = position_units == intervals(:, 1).';
    positionIsOnUpperBound   = position_units == intervals(:, 2).';
    velocityLeavesWorkspace  = (positionIsOnLowerBound & localVelocity_units_s < 0) | ...
        (positionIsOnUpperBound & localVelocity_units_s > 0);
    boundaryVelocityIsZero   = localVelocity_units_s == 0;
    acceleration_units_s2    = endpointStates(stateIndex).acceleration_units_s2;
    accelerationLeavesWorkspace = boundaryVelocityIsZero & ...
        ((positionIsOnLowerBound & acceleration_units_s2 < 0) | ...
        (positionIsOnUpperBound & acceleration_units_s2 > 0));
    if any(velocityLeavesWorkspace | accelerationLeavesWorkspace)
        message = "An endpoint derivative points outside the workspace.";
        reason  = "dynamicEndpointInfeasible";
        return
    end
end
if goalShouldBeChecked
    minimumDuration_s   = obstacleAvoidance.input.minimumTravelTime(initialState, goalState, limits);
    availableDuration_s = goalState.time_s - initialState.time_s + options.ArrivalTimeTolerance_s;
    if minimumDuration_s > availableDuration_s
        message = "The horizon is below a necessary travel-time bound.";
        reason  = "timeWindowInfeasible";
        return
    end
end
feasible = true;
end

%% Section 3: Local Functions

function reachabilityIsBlocked = terminalReachabilityIsBlocked(obstacles, initialState, goalState, limits)
    % A convex obstacle containing the complete backward-reachable box proves infeasibility.
    reachabilityIsBlocked = false;
    finalTime_s         = goalState.time_s;
    previousEventTime_s = initialState.time_s;
    for obstacleIndex = 1:numel(obstacles)
        priorTimes_s = obstacles(obstacleIndex).time_s( ...
            obstacles(obstacleIndex).time_s < finalTime_s);
        if ~isempty(priorTimes_s)
            previousEventTime_s = max(previousEventTime_s, max(priorTimes_s));
        end
    end
    localDuration_s = finalTime_s - previousEventTime_s;
    if localDuration_s <= 0
        return
    end
    candidateDurations_s = unique([localDuration_s * 2.^-(0:16), ...
        linspace(localDuration_s / 256, localDuration_s, 256)]);
    for duration_s = reshape(candidateDurations_s, 1, [])
        center_units = goalState.position_units - goalState.velocity_units_s * duration_s + ...
            0.5 * goalState.acceleration_units_s2 * duration_s^2;
        radius_units = backwardPositionRadius(duration_s, goalState.acceleration_units_s2, ...
            limits.maxAcceleration_units_s2, limits.maxJerk_units_s3);
        corners_units = center_units + [-radius_units(1), -radius_units(2); -radius_units(1), radius_units(2); ...
            radius_units(1), -radius_units(2); radius_units(1), radius_units(2)];
        scene = obstacleAvoidance.obstacles.snapshot(obstacles, finalTime_s - duration_s);
        for sceneIndex = 1:numel(scene)
            for regionIndex = 1:numel(scene(sceneIndex).Regions_units)
                region_units = scene(sceneIndex).Regions_units{regionIndex};
                [inside, onBoundary] = inpolygon(corners_units(:, 1), corners_units(:, 2), ...
                    region_units(:, 1), region_units(:, 2));
                if all(inside | onBoundary)
                    reachabilityIsBlocked = true;
                    return
                end
            end
        end
    end
end

function radius_units = backwardPositionRadius(duration_s, finalAcceleration_units_s2, accelerationLimit_units_s2, jerkLimit_units_s3)
    % Ignore velocity limits to retain a sound outer bound; acceleration tightens long intervals.
    rampDuration_s         = accelerationLimit_units_s2 ./ jerkLimit_units_s3;
    radius_units           = jerkLimit_units_s3 * duration_s^3 / 6;
    accelerationIsLimited  = finalAcceleration_units_s2 == 0 & duration_s > rampDuration_s;
    limitedAcceleration_units_s2 = accelerationLimit_units_s2(accelerationIsLimited);
    limitedJerk_units_s3          = jerkLimit_units_s3(accelerationIsLimited);
    radius_units(accelerationIsLimited) = 0.5 * limitedAcceleration_units_s2 * duration_s^2 - ...
        limitedAcceleration_units_s2.^2 * duration_s ./ (2 * limitedJerk_units_s3) + ...
        limitedAcceleration_units_s2.^3 ./ (6 * limitedJerk_units_s3.^2);
end
