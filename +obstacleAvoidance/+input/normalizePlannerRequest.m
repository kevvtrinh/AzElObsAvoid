function [obstacles, initialState, goalState, limits] = normalizePlannerRequest(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [obstacles, initialState, goalState, limits] = ...
%       obstacleAvoidance.input.normalizePlannerRequest( ...
%       obstacles, initialState, goalState, limits, options)
%
% PURPOSE
%   - Normalize one public planning request before search or motion work.
%
% INPUTS
%   - obstacles (supported obstacle input or [])
%       Combined into the canonical protected-obstacle array.
%   - initialState, goalState (scalar structs)
%       Require time_s and 1-by-2 position_units. Missing endpoint
%       derivatives default to zero. goalState may contain a sampled target.
%   - limits (scalar struct)
%       Velocity, acceleration, and jerk must all be combined scalars or all
%       be two-axis vectors. Combined magnitudes are divided by sqrt(2) per
%       axis. Omitted workspace intervals receive the public defaults.
%   - options (scalar struct)
%       Requires WrapX and WrapY. Wrapping is limited to an
%       obstacle-free fixed-position request.
%
% OUTPUTS
%   - obstacles (canonical protected-obstacle array)
%   - initialState, goalState, limits (normalized scalar structs)
%       Numeric vectors are double rows. Invalid requirements throw the
%       established planTrajectory errors.
%
% UNITS
%   - Position and intervals are coordinate units; time is seconds; derivatives use
%     units/s, units/s^2, and units/s^3.
%

%% Section 1: Normalize Obstacles And Endpoint States

obstacles    = obstacleAvoidance.obstacles.combineObstacles(obstacles);
initialState = obstacleAvoidance.input.normalizePlannerState(initialState, "initialState");
goalState    = obstacleAvoidance.input.normalizePlannerState(goalState, "goalState");

%% Section 2: Normalize A Sampled Moving Goal

hasTargetTime     = isfield(goalState, "targetTime_s");
hasTargetPosition = isfield(goalState, "targetPosition_units");
if xor(hasTargetTime, hasTargetPosition)
    error("planTrajectory:IncompleteMovingGoal", "targetTime_s and targetPosition_units must be supplied together.");
end
if hasTargetTime
    validateattributes(goalState.targetTime_s, {'numeric'}, {'real', 'finite', 'vector', 'increasing'});
    goalState.targetTime_s = double(goalState.targetTime_s(:));
    if numel(goalState.targetTime_s) < 2
        error("planTrajectory:MovingGoalHistoryTooShort", "targetTime_s must contain at least two increasing samples.");
    end
    validateattributes(goalState.targetPosition_units, {'numeric'}, {'real', 'finite', '2d', 'ncols', 2, 'nrows', numel(goalState.targetTime_s)});
    goalState.targetPosition_units = double(goalState.targetPosition_units);
    if goalState.time_s < goalState.targetTime_s(1) || goalState.time_s > goalState.targetTime_s(end)
        error("planTrajectory:MovingGoalHorizonOutsideHistory", "goalState.time_s must be inside targetTime_s.");
    end
    if ~isfield(goalState, "InterpolationMethod") || isempty(goalState.InterpolationMethod)
        goalState.InterpolationMethod = "linear";
    end
    goalState.InterpolationMethod = string(goalState.InterpolationMethod);
    if ~isscalar(goalState.InterpolationMethod) || ~any(goalState.InterpolationMethod == ["linear", "pchip"])
        error("planTrajectory:InvalidGoalInterpolation", "InterpolationMethod must be 'linear' or 'pchip'.");
    end
end

%% Section 3: Normalize Physical And Workspace Limits

limits = obstacleAvoidance.input.normalizePlannerLimits(limits);

%% Section 4: Validate Time And Wrapping Compatibility

if goalState.time_s <= initialState.time_s
    error("planTrajectory:InvalidTimeWindow", "goalState.time_s must be greater than initialState.time_s.");
end
hasMovingGoal = hasTargetTime && ~isempty(goalState.targetTime_s);
if (options.WrapX || options.WrapY) && (~isempty(obstacles) || hasMovingGoal)
    error("planTrajectory:UnsupportedWrappedGeometry", "WrapX and WrapY are supported only for obstacle-free fixed-position goals. Disable wrapping for this request.");
end
goalState.position_units = obstacleAvoidance.input.resolveWrappedGoal(initialState.position_units, goalState.position_units, limits, options);
end
