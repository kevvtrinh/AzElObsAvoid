function [obstacles, initialState, goalState, limits] = ...
    normalizePlannerRequest( ...
    obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [obstacles, initialState, goalState, limits] = ...
%       obstacleAvoidance.input.normalizePlannerRequest( ...
%       obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Normalize the planner request once for every motion method while
%     preserving the public planTrajectory input rules and error behavior.
%**************************************************************************
% INPUTS
%   - obstacles (supported obstacle input or [])
%       Combined into the canonical protected-obstacle array.
%   - initialState (scalar struct)
%       Requires time_s and 1-by-2 position_deg. Missing or empty endpoint
%       derivatives default to zero.
%   - goalState (scalar struct)
%       Supports the fixed endpoint fields and an optional sampled moving
%       target with targetTime_s and targetPosition_deg.
%   - limits (scalar struct)
%       Requires two-axis velocity, acceleration, and jerk limits. Missing
%       workspace intervals receive the public defaults.
%   - options (scalar struct)
%       Requires AllowAzimuthWrapping. Wrapping is compatible only with an
%       obstacle-free fixed-position goal.
%**************************************************************************
% OUTPUTS
%   - obstacles (canonical protected-obstacle array)
%   - initialState (normalized scalar struct)
%   - goalState (normalized scalar struct)
%   - limits (normalized scalar struct)
%       Numeric vectors have stable orientation and double precision.
%       Invalid contracts throw the established planTrajectory errors.
%**************************************************************************
% UNITS
%   - Position and workspace intervals are degrees; time is seconds;
%     derivatives use deg/s, deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Normalize Obstacles And Endpoint States

% Convert public inputs to one internal format before planning starts. Make
% state vectors use one row per physical axis. Check sizes and finite values
% here so later errors do not appear inside search or optimization.

% All supported obstacle container forms are flattened before any planner
% method sees them. Endpoint processing then uses the same loop for the start
% and goal. Thus, both states use the same normalization rules.
obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
states = {initialState, goalState};
stateLabels = ["initialState", "goalState"];
derivativeNames = ["velocity_deg_s", "acceleration_deg_s2"];
for stateIndex = 1:2
    % Validate the fields needed to locate an endpoint in space and time before
    % reading optional derivative fields. Report invalid input before an
    % algorithm uses it.
    state = states{stateIndex};
    if ~isstruct(state) || ~isscalar(state) || ...
            ~all(isfield(state, {'time_s', 'position_deg'}))
        error("planTrajectory:InvalidState", ...
            "%s must be a scalar struct with time_s and position_deg.", ...
            stateLabels(stateIndex));
    end
    validateattributes(state.time_s, ...
        {'numeric'}, {'real', 'finite', 'scalar'});
    validateattributes(state.position_deg, ...
        {'numeric'}, {'real', 'finite', 'vector', 'numel', 2});
    state.time_s = double(state.time_s);
    state.position_deg = double(state.position_deg(:).');
    for derivativeName = derivativeNames
        % Zero is the default for a missing endpoint derivative. Thus, the
        % mechanism starts or stops at rest. The function converts given values to
        % 1-by-2 double rows in [azimuth elevation] order.
        if ~isfield(state, derivativeName) || isempty(state.(derivativeName))
            state.(derivativeName) = [0 0];
        else
            validateattributes(state.(derivativeName), {'numeric'}, ...
                {'real', 'finite', 'vector', 'numel', 2});
            state.(derivativeName) = ...
                double(state.(derivativeName)(:).');
        end
    end
    states{stateIndex} = state;
end
initialState = states{1};
goalState = states{2};

%% Section 2: Normalize A Sampled Moving Goal

% A moving goal uses a strictly increasing time list and one position at each
% time. Sort no data here because sorting could hide a caller error. A failure
% in this section usually means that goal times and position rows do not match.

% The two history fields describe one time series. Each position needs one time.
% The function rejects incomplete moving-goal data.
if isfield(goalState, "targetTime_s") || ...
        isfield(goalState, "targetPosition_deg")
    if ~all(isfield(goalState, ...
            {'targetTime_s', 'targetPosition_deg'}))
        error("planTrajectory:IncompleteMovingGoal", ...
            "targetTime_s and targetPosition_deg must be supplied together.");
    end
    validateattributes(goalState.targetTime_s, ...
        {'numeric'}, {'real', 'finite', 'vector', 'increasing'});
    goalState.targetTime_s = double(goalState.targetTime_s(:));
    if numel(goalState.targetTime_s) < 2
        error("planTrajectory:MovingGoalHistoryTooShort", ...
            "targetTime_s must contain at least two increasing samples.");
    end
    validateattributes(goalState.targetPosition_deg, {'numeric'}, ...
        {'real', 'finite', '2d', 'ncols', 2, ...
        'nrows', numel(goalState.targetTime_s)});
    goalState.targetPosition_deg = double(goalState.targetPosition_deg);
    outsideMovingGoalHistory = ...
        goalState.time_s < goalState.targetTime_s(1) || ...
        goalState.time_s > goalState.targetTime_s(end);
    if outsideMovingGoalHistory
        % The planner must be able to evaluate the target at the latest allowed
        % arrival time without extrapolating beyond user-supplied information.
        error("planTrajectory:MovingGoalHorizonOutsideHistory", ...
            "goalState.time_s must be inside targetTime_s.");
    end
    if ~isfield(goalState, "InterpolationMethod") || ...
            isempty(goalState.InterpolationMethod)
        goalState.InterpolationMethod = "linear";
    end
    goalState.InterpolationMethod = string(goalState.InterpolationMethod);
    validInterpolation = isscalar(goalState.InterpolationMethod) && ...
        any(goalState.InterpolationMethod == ["linear", "pchip"]);
    if ~validInterpolation
        error("planTrajectory:InvalidGoalInterpolation", ...
            "InterpolationMethod must be 'linear' or 'pchip'.");
    end
end

%% Section 3: Normalize Physical And Workspace Limits

% Physical limits bound velocity, acceleration, and jerk. Workspace intervals
% bound allowed azimuth and elevation positions. Keep these two meanings
% separate when debugging an endpoint or continuous-bound failure.

% Each angular axis has its own velocity, acceleration, and jerk bound. A
% two-element row keeps element 1 aligned with azimuth and element 2 aligned
% with elevation throughout vectorized limit calculations.
requiredFields = [ ...
    "maxVelocity_deg_s", ...
    "maxAcceleration_deg_s2", ...
    "maxJerk_deg_s3"];
if ~isstruct(limits) || ~isscalar(limits) || ...
        ~all(isfield(limits, cellstr(requiredFields)))
    error("planTrajectory:InvalidLimits", ...
        "limits must contain velocity, acceleration, and jerk limits.");
end

% Physical limits must be finite and positive. Zero prevents motion on an axis.
% A negative limit does not have a physical meaning.
for name = requiredFields
    validateattributes(limits.(name), {'numeric'}, ...
        {'real', 'finite', 'positive', 'vector', 'numel', 2});
    limits.(name) = double(limits.(name)(:).');
end
intervalDefaults = struct( ...
    "azimuthInterval_deg", [-180 180], ...
    "elevationInterval_deg", [-90 90]);
intervalNames = string(fieldnames(intervalDefaults));

% Workspace intervals are inclusive [minimum maximum] pairs. Defaults cover a
% conventional full azimuth turn and elevation from nadir to zenith.
for name = reshape(intervalNames, 1, [])
    if ~isfield(limits, name) || isempty(limits.(name))
        limits.(name) = intervalDefaults.(name);
    end
    validateattributes(limits.(name), {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 2, 'increasing'}, ...
        "planTrajectory", name);
    limits.(name) = double(limits.(name)(:).');
end

%% Section 4: Validate Time And Wrapping Compatibility

% Confirm that start, goal, obstacle history, and planning horizon use a
% compatible time range. Azimuth wrapping changes position meaning. Reject it
% when obstacle geometry does not use the same periodic interpretation.

% Positive duration is required by every trajectory and interpolation method.
if goalState.time_s <= initialState.time_s
    error("planTrajectory:InvalidTimeWindow", ...
        "goalState.time_s must be greater than initialState.time_s.");
end
hasMovingGoal = isfield(goalState, "targetTime_s") && ...
    ~isempty(goalState.targetTime_s);
if options.AllowAzimuthWrapping && (~isempty(obstacles) || hasMovingGoal)
    % A full azimuth turn gives the same direction for a fixed goal. Obstacles
    % and moving goals make this change unsafe. The changed path can cross a
    % different occupied region.
    error("planTrajectory:UnsupportedWrappedGeometry", ...
        "AllowAzimuthWrapping is supported only for obstacle-free " + ...
        "fixed-position goals. Disable wrapping for this request.");
end
if options.AllowAzimuthWrapping
    % Choose the equivalent goal azimuth nearest the initial azimuth. round
    % finds the number of 360-degree turns to add. This reduces the rotation
    % length and does not change the physical direction.
    turnCount = round((initialState.position_deg(1) - ...
        goalState.position_deg(1)) / 360);
    goalState.position_deg(1) = goalState.position_deg(1) + ...
        360 * turnCount;
    if isfield(goalState, "targetPosition_deg") && ...
            ~isempty(goalState.targetPosition_deg)
        goalState.targetPosition_deg(:, 1) = ...
            goalState.targetPosition_deg(:, 1) + 360 * turnCount;
    end
end
end
