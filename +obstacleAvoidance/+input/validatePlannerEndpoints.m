function [feasible, message, reason] = validatePlannerEndpoints(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [feasible, message, reason] = ...
%       obstacleAvoidance.input.validatePlannerEndpoints(obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Check start and goal states for obstacle collisions, workspace bounds,
%     motion limits, and insufficient travel time before planning a route.
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
%       True when these endpoint checks pass. This alone does not prove that
%       a complete motion exists. Expected physical failures return false;
%       invalid input throws an error.
%   - message (string scalar)
%       Stable expected-failure description.
%   - reason (string scalar)
%       Stable expected-failure identifier.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Check Occupancy At The Requested Endpoint Times

feasible = false;
message  = "";
reason   = "";

% An earliest-arrival goal may be blocked at the deadline but clear earlier.
% Check its obstacle occupancy here only when the arrival time is fixed.
endpointPositions_units = initialState.position_units;
endpointTimes_s         = initialState.time_s;
arrivalIsFixed          = options.GoalTimeMode == "fixedArrival";
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

% A clear goal can still be impossible to approach without a collision.
% Wrapped requests get this check when planning each unwrapped goal copy.
goalApproachShouldBeChecked = arrivalIsFixed && ~options.WrapX && ~options.WrapY;
if goalApproachShouldBeChecked && allGoalApproachesAreBlocked(obstacles, initialState, goalState, limits)
    message = "Every jerk-limited approach to the fixed terminal state intersects a protected obstacle.";
    reason  = "terminalReachabilityBlocked";
    return
end

%% Section 2: Check Endpoint Motion Limits And Workspace Bounds

% A moving target's goal state depends on the chosen arrival time. For an
% earliest intercept, check that state later in each fixed-arrival trial.
goalShouldBeChecked = arrivalIsFixed || isempty(goalState.targetMotion);
endpointStates      = initialState;
if goalShouldBeChecked
    endpointStates(2).position_units        = goalState.position_units;
    endpointStates(2).velocity_units_s      = goalState.velocity_units_s;
    endpointStates(2).acceleration_units_s2 = goalState.acceleration_units_s2;
end
workspaceIntervals_units = [limits.xInterval_units; limits.yInterval_units];
for stateIndex = 1:numel(endpointStates)
    velocityExceedsLimit   = any(abs(endpointStates(stateIndex).velocity_units_s) > limits.maxVelocity_units_s);
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
    positionIsOutsideWorkspace = position_units < workspaceIntervals_units(:, 1).' | ...
        position_units > workspaceIntervals_units(:, 2).';
    if any(positionIsOutsideWorkspace)
        message = "An endpoint lies outside the workspace.";
        reason  = "endpointOutsideWorkspace";
        return
    end

    % At a workspace edge, motion must stay inside immediately next to the
    % endpoint. Look forward from the start and backward from the goal.
    velocityIntoTrip_units_s = endpointStates(stateIndex).velocity_units_s;
    if stateIndex > 1
        % Looking backward reverses velocity; acceleration keeps its sign.
        velocityIntoTrip_units_s = -velocityIntoTrip_units_s;
    end
    positionIsOnLowerBound  = position_units == workspaceIntervals_units(:, 1).';
    positionIsOnUpperBound  = position_units == workspaceIntervals_units(:, 2).';
    velocityLeavesWorkspace = (positionIsOnLowerBound & velocityIntoTrip_units_s < 0) | ...
        (positionIsOnUpperBound & velocityIntoTrip_units_s > 0);

    % When velocity is zero, acceleration determines which way motion starts.
    velocityIsZero              = velocityIntoTrip_units_s == 0;
    acceleration_units_s2       = endpointStates(stateIndex).acceleration_units_s2;
    accelerationLeavesWorkspace = velocityIsZero & ...
        ((positionIsOnLowerBound & acceleration_units_s2 < 0) | ...
        (positionIsOnUpperBound & acceleration_units_s2 > 0));
    if any(velocityLeavesWorkspace | accelerationLeavesWorkspace)
        message = "An endpoint derivative points outside the workspace.";
        reason  = "dynamicEndpointInfeasible";
        return
    end
end

%% Section 3: Check That Enough Travel Time Is Available

% Even without obstacles, motion limits require at least this travel time.
% Allow only the configured arrival-time tolerance when comparing times.
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

%% Section 4: Local Functions

function allApproachesAreBlocked = allGoalApproachesAreBlocked(obstacles, initialState, goalState, limits)
    % Work backward from arrival to enclose every possible earlier position
    % in a box. The box may include extra positions, but must not exclude a
    % possible one. If an obstacle covers the whole box at that time, every
    % allowed approach to the goal is blocked.
    allApproachesAreBlocked  = false;
    finalTime_s              = goalState.time_s;
    approachCheckStartTime_s = initialState.time_s;

    % Look just before arrival, back to the latest obstacle sample or trip start.
    for obstacleIndex = 1:numel(obstacles)
        earlierObstacleTimes_s = obstacles(obstacleIndex).time_s( ...
            obstacles(obstacleIndex).time_s < finalTime_s);
        if ~isempty(earlierObstacleTimes_s)
            approachCheckStartTime_s = max(approachCheckStartTime_s, max(earlierObstacleTimes_s));
        end
    end
    approachCheckDuration_s = finalTime_s - approachCheckStartTime_s;
    if approachCheckDuration_s <= 0
        return
    end

    % Check times across the final interval, with extra checks close to arrival.
    timeBeforeArrival_s = reshape(unique([approachCheckDuration_s * 2 .^ -(0:16), ...
        linspace(approachCheckDuration_s / 256, approachCheckDuration_s, 256)]), [], 1);

    % With dt = time before arrival, place the box center at:
    % goal position - goal velocity x dt + goal acceleration x dt^2 / 2.
    % The half-width allows for the extra displacement caused by jerk.
    boxCenters_units = goalState.position_units - goalState.velocity_units_s .* timeBeforeArrival_s + ...
        0.5 * goalState.acceleration_units_s2 .* timeBeforeArrival_s .^ 2;
    boxHalfWidths_units = calculateApproachBoxHalfWidths(timeBeforeArrival_s, goalState.acceleration_units_s2, ...
        limits.maxAcceleration_units_s2, limits.maxJerk_units_s3);

    % Build four [x y] corners for each possible-position box.
    cornerSigns              = [-1, -1; -1, 1; 1, -1; 1, 1];
    approachBoxCorners_units = reshape(boxCenters_units, [], 1, 2) + ...
        reshape(cornerSigns, 1, 4, 2) .* reshape(boxHalfWidths_units, [], 1, 2);
    boxMinimum_units = reshape(min(approachBoxCorners_units, [], 2), [], 2);
    boxMaximum_units = reshape(max(approachBoxCorners_units, [], 2), [], 2);

    % Use each obstacle's full recorded extent to skip obstacles that
    % cannot contain a whole box. The remaining pairs get an exact check.
    obstacleCount        = numel(obstacles);
    historyMinimum_units = Inf(obstacleCount, 2);
    historyMaximum_units = -Inf(obstacleCount, 2);
    for obstacleIndex = 1:obstacleCount
        vertices_units = collectObstacleHistoryVertices(obstacles(obstacleIndex));
        if ~isempty(vertices_units)
            historyMinimum_units(obstacleIndex, :) = min(vertices_units, [], 1);
            historyMaximum_units(obstacleIndex, :) = max(vertices_units, [], 1);
        end
    end
    obstacleCanCoverApproachBox = ...
        all(reshape(historyMinimum_units, 1, [], 2) <= reshape(boxMinimum_units, [], 1, 2), 3) & ...
        all(reshape(historyMaximum_units, 1, [], 2) >= reshape(boxMaximum_units, [], 1, 2), 3);
    [checkTimeIndices, obstacleIndices] = find(obstacleCanCoverApproachBox);

    % Check whether the obstacle actually covers the box at that time.
    % Each prepared region is convex: containing all four corners means
    % the region contains the entire box, including its edges and interior.
    for pairIndex = 1:numel(checkTimeIndices)
        checkTimeIndex   = checkTimeIndices(pairIndex);
        obstacleSnapshot = obstacleAvoidance.obstacles.snapshot( ...
            obstacles(obstacleIndices(pairIndex)), finalTime_s - timeBeforeArrival_s(checkTimeIndex));
        regions_units    = vertcat(obstacleSnapshot.Regions_units);
        boxCorners_units = reshape(approachBoxCorners_units(checkTimeIndex, :, :), 4, 2);
        for regionIndex = 1:numel(regions_units)
            region_units = regions_units{regionIndex};

            [cornerIsInside, cornerIsOnBoundary] = inpolygon(boxCorners_units(:, 1), boxCorners_units(:, 2), ...
                region_units(:, 1), region_units(:, 2));
            if all(cornerIsInside | cornerIsOnBoundary)
                allApproachesAreBlocked = true;
                return
            end
        end
    end
end

function vertices_units = collectObstacleHistoryVertices(obstacle)
    % Gather stored vertices used to bound the obstacle's extent: protected
    % samples, shapes covering intervals, and regions at interval starts.
    preparation = obstacle.InternalPreparation;

    sampleVertices_units = cellfun(@(x, y) [double(x(:)), double(y(:))], ...
        obstacle.x_units(:), obstacle.y_units(:), 'UniformOutput', false);
    intervalUnionVertices_units = cellfun(@intervalShapeVertices, preparation.IntervalUnionShapes(:), ...
        'UniformOutput', false);
    intervalStartRegions_units = vertcat(preparation.IntervalStartRegions_units{:});
    if isempty(intervalStartRegions_units)
        intervalStartRegions_units = cell(0, 1);
    end
    vertices_units = vertcat(sampleVertices_units{:}, intervalUnionVertices_units{:}, intervalStartRegions_units{:});
    % NaN rows separate polygon rings; they do not contribute to the extent.
    vertices_units = vertices_units(all(isfinite(vertices_units), 2), :);
end

function vertices_units = intervalShapeVertices(shape)
    % An interval without a union model holds an empty placeholder.
    vertices_units = zeros(0, 2);
    if isa(shape, 'polyshape')
        vertices_units = shape.Vertices;
    end
end

function boxHalfWidths_units = calculateApproachBoxHalfWidths( ...
        timeBeforeArrival_s, finalAcceleration_units_s2, accelerationLimit_units_s2, jerkLimit_units_s3)
    % Maximum displacement from the box center = jerk limit x dt^3 / 6.
    % Use that distance on each side of the box center. Ignoring speed limits
    % can enlarge the box, but does not exclude any possible position.
    timeToAccelerationLimit_s = accelerationLimit_units_s2 ./ jerkLimit_units_s3;
    boxHalfWidths_units       = jerkLimit_units_s3 .* timeBeforeArrival_s .^ 3 / 6;

    % With zero goal acceleration, the acceleration limit gives a smaller box
    % after dt exceeds acceleration limit / jerk limit. Keep the wider jerk-only
    % box for nonzero goal acceleration or shorter times.
    useAccelerationLimitedWidth         = finalAcceleration_units_s2 == 0 & timeBeforeArrival_s > timeToAccelerationLimit_s;
    accelerationLimitedHalfWidths_units = 0.5 * accelerationLimit_units_s2 .* timeBeforeArrival_s .^ 2 - ...
        accelerationLimit_units_s2 .^ 2 .* timeBeforeArrival_s ./ (2 * jerkLimit_units_s3) + ...
        accelerationLimit_units_s2 .^ 3 ./ (6 * jerkLimit_units_s3 .^ 2);
    boxHalfWidths_units(useAccelerationLimitedWidth) = accelerationLimitedHalfWidths_units(useAccelerationLimitedWidth);
end
