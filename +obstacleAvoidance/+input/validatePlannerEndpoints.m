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
    % A convex obstacle region containing the complete backward-reachable box
    % at its time proves that no jerk-limited approach can reach the goal.
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

    % --- Backward-Reachable Boxes For Every Candidate Duration ---
    durations_s   = reshape(unique([localDuration_s * 2.^-(0:16), ...
        linspace(localDuration_s / 256, localDuration_s, 256)]), [], 1);
    centers_units = goalState.position_units - goalState.velocity_units_s .* durations_s + ...
        0.5 * goalState.acceleration_units_s2 .* durations_s.^2;
    radii_units   = backwardPositionRadius(durations_s, goalState.acceleration_units_s2, ...
        limits.maxAcceleration_units_s2, limits.maxJerk_units_s3);
    cornerSigns   = [-1, -1; -1, 1; 1, -1; 1, 1];
    corners_units = reshape(centers_units, [], 1, 2) + ...
        reshape(cornerSigns, 1, 4, 2) .* reshape(radii_units, [], 1, 2);
    reachableMinimum_units = reshape(min(corners_units, [], 2), [], 2);
    reachableMaximum_units = reshape(max(corners_units, [], 2), [], 2);

    % --- Obstacles Whose Complete History Can Contain A Box ---
    % Every geometry the exact test below can read comes from the protected
    % samples, the interval union shapes, or the interval start regions, so
    % their joint extents, gathered once, name the only obstacles whose
    % regions can contain a box at any time.
    obstacleCount        = numel(obstacles);
    historyMinimum_units = Inf(obstacleCount, 2);
    historyMaximum_units = -Inf(obstacleCount, 2);
    for obstacleIndex = 1:obstacleCount
        vertices_units = historyVertices(obstacles(obstacleIndex));
        if ~isempty(vertices_units)
            historyMinimum_units(obstacleIndex, :) = min(vertices_units, [], 1);
            historyMaximum_units(obstacleIndex, :) = max(vertices_units, [], 1);
        end
    end
    boxIsCoverable = ...
        all(reshape(historyMinimum_units, 1, [], 2) <= reshape(reachableMinimum_units, [], 1, 2), 3) & ...
        all(reshape(historyMaximum_units, 1, [], 2) >= reshape(reachableMaximum_units, [], 1, 2), 3);
    [durationIndices, obstacleIndices] = find(boxIsCoverable);

    % --- Exact Containment Test For Each Surviving Duration And Obstacle ---
    for pairIndex = 1:numel(durationIndices)
        durationIndex     = durationIndices(pairIndex);
        scene             = obstacleAvoidance.obstacles.snapshot( ...
            obstacles(obstacleIndices(pairIndex)), finalTime_s - durations_s(durationIndex));
        regions_units     = vertcat(scene.Regions_units);
        pairCorners_units = reshape(corners_units(durationIndex, :, :), 4, 2);
        for regionIndex = 1:numel(regions_units)
            region_units = regions_units{regionIndex};
            [inside, onBoundary] = inpolygon(pairCorners_units(:, 1), pairCorners_units(:, 2), ...
                region_units(:, 1), region_units(:, 2));
            if all(inside | onBoundary)
                reachabilityIsBlocked = true;
                return
            end
        end
    end
end

function vertices_units = historyVertices(obstacle)
    % Gather every finite vertex the prepared obstacle can present at any
    % time: protected samples, interval union shapes, and interval start regions.
    preparation = obstacle.InternalPreparation;
    sampleSets  = cellfun(@(x, y) [double(x(:)), double(y(:))], ...
        obstacle.x_units(:), obstacle.y_units(:), 'UniformOutput', false);
    unionSets   = cellfun(@intervalShapeVertices, preparation.IntervalUnionShapes(:), ...
        'UniformOutput', false);
    regionSets  = vertcat(preparation.IntervalStartRegions_units{:});
    if isempty(regionSets)
        regionSets = cell(0, 1);
    end
    vertices_units = vertcat(sampleSets{:}, unionSets{:}, regionSets{:});
    vertices_units = vertices_units(all(isfinite(vertices_units), 2), :);
end

function vertices_units = intervalShapeVertices(shape)
    % An interval without a union model holds an empty placeholder.
    vertices_units = zeros(0, 2);
    if isa(shape, 'polyshape')
        vertices_units = shape.Vertices;
    end
end

function radius_units = backwardPositionRadius(durations_s, finalAcceleration_units_s2, accelerationLimit_units_s2, jerkLimit_units_s3)
    % Ignore velocity limits to retain a sound outer bound; acceleration tightens long intervals.
    rampDuration_s        = accelerationLimit_units_s2 ./ jerkLimit_units_s3;
    radius_units          = jerkLimit_units_s3 .* durations_s.^3 / 6;
    accelerationIsLimited = finalAcceleration_units_s2 == 0 & durations_s > rampDuration_s;
    limitedRadius_units   = 0.5 * accelerationLimit_units_s2 .* durations_s.^2 - ...
        accelerationLimit_units_s2.^2 .* durations_s ./ (2 * jerkLimit_units_s3) + ...
        accelerationLimit_units_s2.^3 ./ (6 * jerkLimit_units_s3.^2);
    radius_units(accelerationIsLimited) = limitedRadius_units(accelerationIsLimited);
end
