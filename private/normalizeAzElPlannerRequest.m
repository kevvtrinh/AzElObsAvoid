function request = normalizeAzElPlannerRequest(request)
%% Section 0: Header & Readme
% SYNTAX
%   request = normalizeAzElPlannerRequest(request)
%**************************************************************************
% PURPOSE
%   - Validate and normalize the one public planning request contract.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       obstacles, initialState, goal, limits, and optional options fields.
%**************************************************************************
% OUTPUTS
%   - request (scalar struct)
%       Normalized private request with canonical obstacle cells and
%       resolved public options.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

%% Section 1: Validate Top-Level Fields
if ~isstruct(request) || ~isscalar(request)
    error("normalizeAzElPlannerRequest:InvalidRequest", ...
        "request must be one scalar structure.");
end
requiredFields = ["obstacles", "initialState", "goal", "limits"];
for fieldIndex = 1:numel(requiredFields)
    if ~isfield(request, requiredFields(fieldIndex))
        error("normalizeAzElPlannerRequest:MissingField", ...
            "request is missing required field '%s'.", ...
            requiredFields(fieldIndex));
    end
end
if ~isfield(request, "options") || isempty(request.options)
    request.options = struct();
end

%% Section 2: Normalize Obstacles & Boundary States
request.obstacles = normalizeObstacleCollection(request.obstacles);
request.initialState = normalizeBoundaryState( ...
    request.initialState, "initialState", true);
request.goal = normalizeGoal(request.goal);
request.limits = normalizeLimits(request.limits);
request.options = resolveOptions(request.options);
request.options.azimuthDisplayRange_deg = request.limits.azimuth_deg;

%% Section 3: Resolve Time Horizon & Wrapped Coordinates
startTime_s = request.initialState.time_s;
if request.goal.type == "fixed"
    if isnan(request.options.deadline_s) && isfinite(request.goal.time_s)
        request.options.deadline_s = request.goal.time_s;
    end
    request.goal.unwrappedPosition_deg = request.goal.position_deg;
    if request.options.azimuthWrap
        request.goal.unwrappedPosition_deg(1) = nearestEquivalentAzimuth( ...
            request.goal.position_deg(1), ...
            request.initialState.position_deg(1), ...
            diff(request.limits.azimuth_deg));
    end
else
    request.goal.unwrappedPosition_deg = request.goal.position_deg;
    if request.options.azimuthWrap
        request.goal.unwrappedPosition_deg(:, 1) = ...
            unwrapGoalAzimuth(request.goal.position_deg(:, 1), ...
                request.initialState.position_deg(1), ...
                diff(request.limits.azimuth_deg));
    end
    if isnan(request.options.deadline_s)
        request.options.deadline_s = request.goal.time_s(end) - ...
            request.options.trailingDuration_s;
    end
end

if isnan(request.options.deadline_s)
    request.options.deadline_s = deriveDefaultDeadline(request);
end
if request.options.deadline_s <= startTime_s
    error("normalizeAzElPlannerRequest:InvalidDeadline", ...
        "deadline_s must be later than initialState.time_s.");
end
if request.goal.type == "moving"
    latestCaptureTime_s = request.goal.time_s(end) - ...
        request.options.trailingDuration_s;
    request.options.deadline_s = min( ...
        request.options.deadline_s, latestCaptureTime_s);
    if request.options.deadline_s < request.goal.time_s(1)
        error("normalizeAzElPlannerRequest:GoalOutsideDeadline", ...
            "The moving goal has no sample inside the capture horizon.");
    end
end

%% Section 4: Validate Physical Boundary Conditions
validateBoundaryAgainstLimits(request.initialState, request.limits, ...
    request.options, "initialState");
if request.goal.type == "fixed"
    validateBoundaryAgainstLimits(request.goal, request.limits, ...
        request.options, "goal");
else
    for sampleIndex = 1:numel(request.goal.time_s)
        sampledGoal = struct( ...
            "position_deg", request.goal.position_deg(sampleIndex, :), ...
            "velocity_deg_s", request.goal.velocity_deg_s(sampleIndex, :), ...
            "acceleration_deg_s2", ...
                request.goal.acceleration_deg_s2(sampleIndex, :));
        validateBoundaryAgainstLimits(sampledGoal, request.limits, ...
            request.options, "moving goal sample");
    end
end
request.isNormalizedInternal = true;
end

function obstacles = normalizeObstacleCollection(rawObstacles)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = normalizeObstacleCollection(rawObstacles)
%**************************************************************************
% PURPOSE
%   - Convert empty, struct-array, or cell obstacle input to scalar cells.
%**************************************************************************
% INPUTS
%   - rawObstacles (empty, struct array, or cell collection)
%**************************************************************************
% OUTPUTS
%   - obstacles (cell column)
%**************************************************************************
% UNITS
%   - Canonical obstacle fields state their units.

if isempty(rawObstacles)
    obstacles = cell(0, 1);
elseif isstruct(rawObstacles)
    obstacles = cell(numel(rawObstacles), 1);
    for obstacleIndex = 1:numel(rawObstacles)
        obstacles{obstacleIndex} = rawObstacles(obstacleIndex);
    end
elseif iscell(rawObstacles)
    obstacles = reshape(rawObstacles, [], 1);
else
    error("normalizeAzElPlannerRequest:InvalidObstacles", ...
        "obstacles must be empty, a struct array, or a cell collection.");
end
for obstacleIndex = 1:numel(obstacles)
    obstacles{obstacleIndex} = ...
        normalizeAzElTimeObstacleData(obstacles{obstacleIndex});
end
end

function state = normalizeBoundaryState(state, fieldName, requireTime)
%% Section 0: Header & Readme
% SYNTAX
%   state = normalizeBoundaryState(state, fieldName, requireTime)
%**************************************************************************
% PURPOSE
%   - Normalize one complete position, velocity, and acceleration state.
%**************************************************************************
% INPUTS
%   - state (scalar struct)
%   - fieldName (scalar text)
%   - requireTime (logical scalar)
%**************************************************************************
% OUTPUTS
%   - state (scalar struct)
%**************************************************************************
% UNITS
%   - Field names state their units.

if ~isstruct(state) || ~isscalar(state)
    error("normalizeAzElPlannerRequest:InvalidBoundaryState", ...
        "%s must be one scalar structure.", fieldName);
end
requiredFields = ["position_deg", "velocity_deg_s", ...
    "acceleration_deg_s2"];
if requireTime
    requiredFields = ["time_s", requiredFields];
elseif ~isfield(state, "time_s")
    state.time_s = NaN;
end
for requiredIndex = 1:numel(requiredFields)
    if ~isfield(state, requiredFields(requiredIndex))
        error("normalizeAzElPlannerRequest:MissingStateField", ...
            "%s is missing required field '%s'.", fieldName, ...
            requiredFields(requiredIndex));
    end
end
if requireTime
    validateattributes(state.time_s, "numeric", ...
        ["scalar", "real", "finite"], mfilename, fieldName + ".time_s");
else
    validateattributes(state.time_s, "numeric", ...
        ["scalar", "real"], mfilename, fieldName + ".time_s");
end
state.time_s = double(state.time_s);
state.position_deg = validatedRow(state.position_deg, ...
    fieldName + ".position_deg");
state.velocity_deg_s = validatedRow(state.velocity_deg_s, ...
    fieldName + ".velocity_deg_s");
state.acceleration_deg_s2 = validatedRow( ...
    state.acceleration_deg_s2, fieldName + ".acceleration_deg_s2");
end

function goal = normalizeGoal(goal)
%% Section 0: Header & Readme
% SYNTAX
%   goal = normalizeGoal(goal)
%**************************************************************************
% PURPOSE
%   - Normalize fixed and sampled moving complete-state goals.
%**************************************************************************
% INPUTS
%   - goal (scalar struct)
%**************************************************************************
% OUTPUTS
%   - goal (scalar struct)
%**************************************************************************
% UNITS
%   - Field names state their units.

if ~isstruct(goal) || ~isscalar(goal)
    error("normalizeAzElPlannerRequest:InvalidGoal", ...
        "goal must be one scalar structure.");
end
if isfield(goal, "type") && ~isempty(goal.type)
    goalType = lower(string(goal.type));
else
    if isfield(goal, "time_s") && numel(goal.time_s) > 1
        goalType = "moving";
    else
        goalType = "fixed";
    end
end
if ~ismember(goalType, ["fixed", "moving"])
    error("normalizeAzElPlannerRequest:InvalidGoalType", ...
        "goal.type must be 'fixed' or 'moving'.");
end

if goalType == "fixed"
    goal = normalizeBoundaryState(goal, "goal", false);
    goal.type = "fixed";
    return;
end

requiredFields = ["time_s", "position_deg", "velocity_deg_s", ...
    "acceleration_deg_s2"];
for fieldIndex = 1:numel(requiredFields)
    if ~isfield(goal, requiredFields(fieldIndex))
        error("normalizeAzElPlannerRequest:MissingMovingGoalField", ...
            "Moving goal is missing required field '%s'.", ...
            requiredFields(fieldIndex));
    end
end
goal.time_s = double(goal.time_s(:));
validateattributes(goal.time_s, "numeric", ...
    ["real", "finite", "nonempty", "increasing"], mfilename, ...
    "goal.time_s");
sampleCount = numel(goal.time_s);
goal.position_deg = validatedHistory(goal.position_deg, sampleCount, ...
    "goal.position_deg");
goal.velocity_deg_s = validatedHistory(goal.velocity_deg_s, sampleCount, ...
    "goal.velocity_deg_s");
goal.acceleration_deg_s2 = validatedHistory( ...
    goal.acceleration_deg_s2, sampleCount, ...
    "goal.acceleration_deg_s2");
goal.type = "moving";
end

function limits = normalizeLimits(limits)
%% Section 0: Header & Readme
% SYNTAX
%   limits = normalizeLimits(limits)
%**************************************************************************
% PURPOSE
%   - Validate the two-axis physical position and motion limits.
%**************************************************************************
% INPUTS
%   - limits (scalar struct)
%**************************************************************************
% OUTPUTS
%   - limits (scalar struct)
%**************************************************************************
% UNITS
%   - Field names state their units.

if ~isstruct(limits) || ~isscalar(limits)
    error("normalizeAzElPlannerRequest:InvalidLimits", ...
        "limits must be one scalar structure.");
end
requiredFields = ["azimuth_deg", "elevation_deg", ...
    "maxVelocity_deg_s", "maxAcceleration_deg_s2"];
for fieldIndex = 1:numel(requiredFields)
    if ~isfield(limits, requiredFields(fieldIndex))
        error("normalizeAzElPlannerRequest:MissingLimitField", ...
            "limits is missing required field '%s'.", ...
            requiredFields(fieldIndex));
    end
end
limits.azimuth_deg = validatedRow(limits.azimuth_deg, ...
    "limits.azimuth_deg");
limits.elevation_deg = validatedRow(limits.elevation_deg, ...
    "limits.elevation_deg");
if limits.azimuth_deg(2) <= limits.azimuth_deg(1) || ...
        limits.elevation_deg(2) <= limits.elevation_deg(1)
    error("normalizeAzElPlannerRequest:InvalidPositionLimits", ...
        "Position limit pairs must be strictly increasing.");
end
limits.maxVelocity_deg_s = validatedRow( ...
    limits.maxVelocity_deg_s, "limits.maxVelocity_deg_s");
limits.maxAcceleration_deg_s2 = validatedRow( ...
    limits.maxAcceleration_deg_s2, ...
    "limits.maxAcceleration_deg_s2");
if any(limits.maxVelocity_deg_s <= 0) || ...
        any(limits.maxAcceleration_deg_s2 <= 0)
    error("normalizeAzElPlannerRequest:InvalidMotionLimits", ...
        "Velocity and acceleration limits must be positive.");
end
end

function options = resolveOptions(providedOptions)
%% Section 0: Header & Readme
% SYNTAX
%   options = resolveOptions(providedOptions)
%**************************************************************************
% PURPOSE
%   - Merge partial mission options with the single defaults authority.
%**************************************************************************
% INPUTS
%   - providedOptions (scalar struct)
%**************************************************************************
% OUTPUTS
%   - options (scalar struct)
%**************************************************************************
% UNITS
%   - Field names state their units.

if ~isstruct(providedOptions) || ~isscalar(providedOptions)
    error("normalizeAzElPlannerRequest:InvalidOptions", ...
        "options must be one scalar structure.");
end
options = azElPlannerDefaults();
providedNames = string(fieldnames(providedOptions));
knownNames = string(fieldnames(options));
for fieldIndex = 1:numel(providedNames)
    fieldName = providedNames(fieldIndex);
    if ismember(fieldName, knownNames)
        if ~isempty(providedOptions.(fieldName))
            options.(fieldName) = providedOptions.(fieldName);
        end
    else
        warning("planAzElAvoidance:UnknownOption", ...
            "Unknown mission option '%s' was ignored.", fieldName);
    end
end

nonnegativeFields = ["safetyMargin_deg", "temporalPadding_s", ...
    "trailingDuration_s", "positionTolerance_deg", ...
    "velocityTolerance_deg_s", "accelerationTolerance_deg_s2", ...
    "clearanceTolerance_deg", "arrivalTolerance_s"];
for fieldIndex = 1:numel(nonnegativeFields)
    fieldName = nonnegativeFields(fieldIndex);
    validateattributes(options.(fieldName), "numeric", ...
        ["scalar", "real", "finite", "nonnegative"], mfilename, ...
        "options." + fieldName);
    options.(fieldName) = double(options.(fieldName));
end
validateattributes(options.planningWallTime_s, "numeric", ...
    ["scalar", "real", "finite", "positive"], mfilename, ...
    "options.planningWallTime_s");
validateattributes(options.deadline_s, "numeric", ...
    ["scalar", "real"], mfilename, "options.deadline_s");
options.planningWallTime_s = double(options.planningWallTime_s);
options.deadline_s = double(options.deadline_s);
options.azimuthWrap = logical(options.azimuthWrap);
if ~isscalar(options.azimuthWrap)
    error("normalizeAzElPlannerRequest:InvalidWrapPolicy", ...
        "options.azimuthWrap must be scalar logical.");
end
end

function validateBoundaryAgainstLimits(state, limits, options, fieldName)
%% Section 0: Header & Readme
% SYNTAX
%   validateBoundaryAgainstLimits(state, limits, options, fieldName)
%**************************************************************************
% PURPOSE
%   - Reject boundary states that violate physical position or motion limits.
%**************************************************************************
% INPUTS
%   - state (scalar struct)
%   - limits (scalar struct)
%   - options (scalar struct)
%   - fieldName (scalar text)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Field names state their units.

position_deg = state.position_deg;
if ~options.azimuthWrap && (position_deg(1) < limits.azimuth_deg(1) || ...
        position_deg(1) > limits.azimuth_deg(2))
    error("normalizeAzElPlannerRequest:BoundaryPositionLimit", ...
        "%s azimuth is outside its position limits.", fieldName);
end
if position_deg(2) < limits.elevation_deg(1) || ...
        position_deg(2) > limits.elevation_deg(2)
    error("normalizeAzElPlannerRequest:BoundaryPositionLimit", ...
        "%s elevation is outside its position limits.", fieldName);
end
if any(abs(state.velocity_deg_s) > limits.maxVelocity_deg_s)
    error("normalizeAzElPlannerRequest:BoundaryVelocityLimit", ...
        "%s velocity exceeds a physical limit.", fieldName);
end
if any(abs(state.acceleration_deg_s2) > ...
        limits.maxAcceleration_deg_s2)
    error("normalizeAzElPlannerRequest:BoundaryAccelerationLimit", ...
        "%s acceleration exceeds a physical limit.", fieldName);
end
end

function value = validatedRow(value, argumentName)
%% Section 0: Header & Readme
% SYNTAX
%   value = validatedRow(value, argumentName)
%**************************************************************************
% PURPOSE
%   - Validate one finite real two-component row vector.
%**************************************************************************
% INPUTS
%   - value (numeric vector)
%   - argumentName (scalar text)
%**************************************************************************
% OUTPUTS
%   - value (1-by-2 double)
%**************************************************************************
% UNITS
%   - Units are identified by argumentName.

validateattributes(value, "numeric", ...
    {'real', 'finite', 'vector', 'numel', 2}, mfilename, argumentName);
value = reshape(double(value), 1, 2);
end

function history = validatedHistory(history, sampleCount, argumentName)
%% Section 0: Header & Readme
% SYNTAX
%   history = validatedHistory(history, sampleCount, argumentName)
%**************************************************************************
% PURPOSE
%   - Validate one finite N-by-2 moving-state history.
%**************************************************************************
% INPUTS
%   - history (numeric matrix)
%   - sampleCount (positive integer)
%   - argumentName (scalar text)
%**************************************************************************
% OUTPUTS
%   - history (N-by-2 double)
%**************************************************************************
% UNITS
%   - Units are identified by argumentName.

history = double(history);
if ~isequal(size(history), [sampleCount, 2]) || ...
        any(~isfinite(history), "all") || ~isreal(history)
    error("normalizeAzElPlannerRequest:InvalidGoalHistory", ...
        "%s must be a finite real %d-by-2 array.", ...
        argumentName, sampleCount);
end
end

function deadline_s = deriveDefaultDeadline(request)
%% Section 0: Header & Readme
% SYNTAX
%   deadline_s = deriveDefaultDeadline(request)
%**************************************************************************
% PURPOSE
%   - Derive a conservative mission horizon when the caller supplies none.
%**************************************************************************
% INPUTS
%   - request (normalized scalar struct)
%**************************************************************************
% OUTPUTS
%   - deadline_s (finite scalar)
%**************************************************************************
% UNITS
%   - Time is seconds.

startPosition_deg = request.initialState.position_deg;
goalPosition_deg = request.goal.position_deg;
if size(goalPosition_deg, 1) > 1
    goalPosition_deg = goalPosition_deg(1, :);
end
positionDelta_deg = abs(goalPosition_deg - startPosition_deg);
rateTime_s = max(positionDelta_deg ./ request.limits.maxVelocity_deg_s);
accelerationTime_s = max(sqrt( ...
    2 .* positionDelta_deg ./ request.limits.maxAcceleration_deg_s2));
nominalDuration_s = max([rateTime_s, accelerationTime_s, 1]);
deadline_s = request.initialState.time_s + max(30, 6 .* nominalDuration_s);
end

function unwrappedAzimuth_deg = nearestEquivalentAzimuth( ...
        azimuth_deg, referenceAzimuth_deg, span_deg)
%% Section 0: Header & Readme
% SYNTAX
%   unwrappedAzimuth_deg = nearestEquivalentAzimuth( ...
%       azimuth_deg, referenceAzimuth_deg, span_deg)
%**************************************************************************
% PURPOSE
%   - Choose the seam-equivalent azimuth nearest a continuous reference.
%**************************************************************************
% INPUTS
%   - azimuth_deg (scalar)
%   - referenceAzimuth_deg (scalar)
%   - span_deg (positive scalar)
%**************************************************************************
% OUTPUTS
%   - unwrappedAzimuth_deg (scalar)
%**************************************************************************
% UNITS
%   - Azimuth is degrees.

shiftCount = round((referenceAzimuth_deg - azimuth_deg) ./ span_deg);
unwrappedAzimuth_deg = azimuth_deg + shiftCount .* span_deg;
end

function unwrappedAzimuth_deg = unwrapGoalAzimuth( ...
        azimuth_deg, referenceAzimuth_deg, span_deg)
%% Section 0: Header & Readme
% SYNTAX
%   unwrappedAzimuth_deg = unwrapGoalAzimuth( ...
%       azimuth_deg, referenceAzimuth_deg, span_deg)
%**************************************************************************
% PURPOSE
%   - Build a continuous moving-goal azimuth history across the seam.
%**************************************************************************
% INPUTS
%   - azimuth_deg (numeric column)
%   - referenceAzimuth_deg (scalar)
%   - span_deg (positive scalar)
%**************************************************************************
% OUTPUTS
%   - unwrappedAzimuth_deg (numeric column)
%**************************************************************************
% UNITS
%   - Azimuth is degrees.

azimuth_deg = azimuth_deg(:);
unwrappedAzimuth_deg = zeros(size(azimuth_deg));
unwrappedAzimuth_deg(1) = nearestEquivalentAzimuth( ...
    azimuth_deg(1), referenceAzimuth_deg, span_deg);
for sampleIndex = 2:numel(azimuth_deg)
    unwrappedAzimuth_deg(sampleIndex) = nearestEquivalentAzimuth( ...
        azimuth_deg(sampleIndex), unwrappedAzimuth_deg(sampleIndex - 1), ...
        span_deg);
end
end
