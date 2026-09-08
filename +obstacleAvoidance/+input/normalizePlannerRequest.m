function [obstacles, initialState, goalState, limits] = normalizePlannerRequest(obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX: [obstacles, initialState, goalState, limits] = normalizePlannerRequest(obstacles, initialState, goalState, limits, options)
% PURPOSE: Normalize one request before geometry, search, or motion computation.
% INPUTS: Supported obstacle input or []; states require time_s and 1-by-2 position_units.
%   Missing derivatives default zero. Goals may include sampled target history.
%   Physical limits must all be combined scalars or all axis vectors; combined
%   magnitudes allocate 1/sqrt(2) to each axis. Missing workspace intervals use defaults.
%   Options require WrapX/WrapY; periodic requests require an empty scene and fixed goal.
% OUTPUTS: Canonical protected obstacles and normalized states/limits with double row vectors.
%   Invalid requests throw identified planner errors.
% UNITS: Coordinate units, seconds, and physical derivatives.

%% Section 1: Normalize Obstacles And Endpoint States

obstacles    = obstacleAvoidance.obstacles.combineObstacles(obstacles);
initialState = obstacleAvoidance.input.normalizePlannerState(initialState, "initialState");
goalState    = obstacleAvoidance.input.normalizePlannerState(goalState, "goalState");

%% Section 2: Normalize A Sampled Moving Goal

hasTargetTime     = isfield(goalState, "targetTime_s");
hasTargetPosition = isfield(goalState, "targetPosition_units");
if xor(hasTargetTime, hasTargetPosition)
    error("planner:IncompleteMovingGoal", "targetTime_s and targetPosition_units must be supplied together.");
end
if hasTargetTime
    validateattributes(goalState.targetTime_s, {'numeric'}, {'real', 'finite', 'vector', 'increasing'});
    goalState.targetTime_s = double(goalState.targetTime_s(:));
    if numel(goalState.targetTime_s) < 2
        error("planner:MovingGoalHistoryTooShort", "targetTime_s must contain at least two increasing samples.");
    end
    validateattributes(goalState.targetPosition_units, {'numeric'}, {'real', 'finite', '2d', 'ncols', 2, 'nrows', numel(goalState.targetTime_s)});
    goalState.targetPosition_units = double(goalState.targetPosition_units);
    if goalState.time_s < goalState.targetTime_s(1) || goalState.time_s > goalState.targetTime_s(end)
        error("planner:MovingGoalHorizonOutsideHistory", "goalState.time_s must be inside targetTime_s.");
    end
    if ~isfield(goalState, "InterpolationMethod") || isempty(goalState.InterpolationMethod)
        goalState.InterpolationMethod = "linear";
    end
    goalState.InterpolationMethod = string(goalState.InterpolationMethod);
    if ~isscalar(goalState.InterpolationMethod) || ~any(goalState.InterpolationMethod == ["linear", "pchip"])
        error("planner:InvalidGoalInterpolation", "InterpolationMethod must be 'linear' or 'pchip'.");
    end
end

%% Section 3: Normalize Physical And Workspace Limits

limits = obstacleAvoidance.input.normalizePlannerLimits(limits);

%% Section 4: Validate Time And Wrapping Compatibility

if goalState.time_s <= initialState.time_s
    error("planner:InvalidTimeWindow", "goalState.time_s must be greater than initialState.time_s.");
end
hasMovingGoal = hasTargetTime && ~isempty(goalState.targetTime_s);
if (options.WrapX || options.WrapY) && (~isempty(obstacles) || hasMovingGoal)
    error("planner:UnsupportedWrappedGeometry", "WrapX and WrapY are supported only for obstacle-free fixed-position goals. Disable wrapping for this request.");
end
goalState.position_units = obstacleAvoidance.input.resolveWrappedGoal(initialState.position_units, goalState.position_units, limits, options);
end
