function request = prepareRequest(obstacles, initialState, goalState, limits, options, parentRequest)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.planning.prepareRequest()
%   request = obstacleAvoidance.planning.prepareRequest( ...
%       obstacles, initialState, goalState, limits, options, parentRequest)
%**************************************************************************
% PURPOSE
%   - Check inputs, fill defaults, match target motion, and unwrap coordinates.
%     Return the prepared request without searching routes or generating motion.
%**************************************************************************
% INPUTS
%   - obstacles (struct array)
%       Static or time-varying polygons; [] requests obstacle-free planning.
%   - initialState (scalar struct)
%       Initial time, position, velocity, and acceleration.
%   - goalState (scalar struct)
%       Goal time, state, and optional target motion.
%   - limits (scalar struct)
%       Workspace intervals and motion limits.
%   - options (scalar struct)
%       Planner option overrides.
%   - parentRequest (scalar struct or [])
%       Original request details passed to a planning trial, built by
%       obstacleAvoidance.planning.createParentRequest; [] for a public call.
%**************************************************************************
% OUTPUTS
%   - request (scalar struct)
%       Checked states, limits, options, obstacles, saved inputs, and parent request.
%       Invalid input throws an error.
%   - options (scalar struct, zero-input call)
%       Fully resolved planner defaults.
%**************************************************************************
% UNITS
%   - Positions are coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Check Inputs And Fill Defaults

% With no inputs, return the available options and their default values.
if nargin == 0
    [~, ~, ~, defaultOptions] = createDefaults();
    request = resolveOptions(struct(), defaultOptions);
    return
end

[defaultInitialState, defaultGoalState, defaultLimits, defaultOptions] = createDefaults();

% Empty input structures select defaults. Partially filled structures are
% completed and checked by the normalization functions below.
if isempty(initialState)
    initialState = defaultInitialState;
end
if isempty(goalState)
    goalState = defaultGoalState;
end
if isempty(limits)
    limits = defaultLimits;
end
if isempty(options)
    options = struct();
end

% Keep the supplied fields so later checks can distinguish an explicit
% user constraint from a missing field filled in during normalization.
% Entirely empty input structures have already been replaced with defaults.
suppliedLimits    = limits;
suppliedGoalState = goalState;

initialState = normalizeState(initialState, defaultInitialState, "initialState", true);
goalState    = normalizeState(goalState, defaultGoalState, "goalState", false);

requestTimeIsInvalid = goalState.time_s <= initialState.time_s;

% Calculate the target's position at the requested goal time.
% For fixed arrival, this is the required intercept position.
% For earliest arrival, this is the target's position at the deadline;
% the planner searches for earlier intercepts later.
if ~isempty(goalState.targetMotion)
    try
        targetPosition_units = obstacleAvoidance.input.targetPositionAtTime(goalState.targetMotion, goalState.time_s);
    catch exception
        % If an invalid arrival time also falls outside the target history,
        % report the time-order error below first. Preserve other errors.
        historyErrorComesFromInvalidRequest = requestTimeIsInvalid && ...
            exception.identifier == "planner:TargetTimeOutsideHistory";
        if ~historyErrorComesFromInvalidRequest
            rethrow(exception)
        end
    end
end

if requestTimeIsInvalid
    error("planTrajectory:InvalidTimeOrder", "goalState.time_s must be greater than initialState.time_s.");
end

if ~isempty(goalState.targetMotion)
    goalState.position_units = targetPosition_units;
end
limits  = normalizeLimits(limits, defaultLimits);
options = resolveOptions(options, defaultOptions);

% Save these normalized values before target matching or unwrapping changes them.
requestedLimits    = limits;
requestedGoalState = goalState;

%% Section 2: Unwrap Periodic Coordinates

% Convert wrapped axes to continuous coordinates so crossing an end does
% not look like a large jump. For example, 359 -> 1 becomes 359 -> 361 on
% a 360-unit x axis. The planner can then use ordinary coordinate differences.
% A y copy over an end is a pole copy: y is mirrored and x turns half a turn.
wrapModes = [options.WrapX options.WrapY];
if any(wrapModes ~= "false")
    intervalFieldNames       = ["xInterval_units", "yInterval_units"];
    workspaceIntervals_units = [limits.xInterval_units; limits.yInterval_units];
    if ~isempty(goalState.targetMotion)
        % Anchor the target path near the start, then choose each sample's
        % equivalent copy nearest the preceding unwrapped sample.
        goalState.targetMotion   = obstacleAvoidance.input.unwrapTargetPath( ...
            goalState.targetMotion, initialState.position_units, workspaceIntervals_units, wrapModes);
        % Read the arrival position from the continuous target path.
        goalState.position_units = obstacleAvoidance.input.targetPositionAtTime( ...
            goalState.targetMotion, goalState.time_s);
    end
    for axisIndex = find(wrapModes ~= "false")
        wrapMode            = wrapModes(axisIndex);
        interval_units      = workspaceIntervals_units(axisIndex, :);
        wrapLength_units    = diff(interval_units);
        startPosition_units = initialState.position_units(axisIndex);
        goalPosition_units  = goalState.position_units(axisIndex);
        % Maximum travel distance = maximum speed x available time.
        % Acceleration limits and obstacles may reduce how far the vehicle can reach.
        maxDisplacement_units = limits.maxVelocity_units_s(axisIndex) * (goalState.time_s - initialState.time_s);
        if isempty(goalState.targetMotion) && axisIndex == 1
            % Shift the goal by whole loops to place it closest to the start.
            % Example: start = 350, goal = 10, loop length = 360.
            % Use goal = 10 + 360 = 370, so the distance is 20 instead of 340.
            % If two copies are equally close, choose the higher coordinate.
            goalWrapCount = floor((startPosition_units - goalPosition_units) / wrapLength_units + 0.5);
            % "forward" may not pass the lower end and "backward" may not pass
            % the upper end, so keep the copy on the allowed side. Example:
            % start = 10, goal = 350: the nearest copy -10 is below 0, so
            % "forward" keeps 350.
            if wrapMode == "forward"
                goalWrapCount = max(goalWrapCount, ceil((interval_units(1) - goalPosition_units) / wrapLength_units));
            elseif wrapMode == "backward"
                goalWrapCount = min(goalWrapCount, floor((interval_units(2) - goalPosition_units) / wrapLength_units));
            end
            goalState.position_units(axisIndex) = goalPosition_units + wrapLength_units * goalWrapCount;
        end
        % A fixed goal keeps its y here. Its pole copies change x and y
        % together, so planWrappedMotion lists every goal copy in the range
        % and tries the nearest first: from (10, 89), goal (190, 89) is
        % tried first as its pole copy (10, 91).

        % Search from the start out to the maximum travel distance, beyond the
        % interval ends. "forward" stops at the lower end and "backward" at
        % the upper end. Example on [0 360], start = 350, reach = 30: "both"
        % and "forward" give [320 380]; "backward" gives [320 360].
        planningRange_units = startPosition_units + [-maxDisplacement_units maxDisplacement_units];
        if wrapMode == "forward"
            planningRange_units(1) = max(planningRange_units(1), interval_units(1));
        elseif wrapMode == "backward"
            planningRange_units(2) = min(planningRange_units(2), interval_units(2));
        end
        limits.(intervalFieldNames(axisIndex)) = planningRange_units;
    end

    % Over a pole, y runs the other way, so the sign of a y velocity or
    % acceleration depends on which side of the pole the target is met, and
    % that time is not known yet for earliest arrival. A matched value is
    % read from the continuous target path at the actual arrival, so it is
    % always right. Require any other y value to be zero, which has no sign,
    % and do not accept a supplied value beside a matched one: the supplied
    % value is on the sphere, the matched one on the continuous copy.
    if wrapModes(2) ~= "false" && ~isempty(goalState.targetMotion)
        derivativeIsMatched   = [options.MatchTargetVelocity, options.MatchTargetAcceleration];
        suppliedYValues       = [goalState.velocity_units_s(2), goalState.acceleration_units_s2(2)];
        derivativeWasSupplied = [isfield(suppliedGoalState, 'velocity_units_s') && ...
            ~isempty(suppliedGoalState.velocity_units_s), ...
            isfield(suppliedGoalState, 'acceleration_units_s2') && ...
            ~isempty(suppliedGoalState.acceleration_units_s2)];
        if any(~derivativeIsMatched & suppliedYValues ~= 0) || any(derivativeIsMatched & derivativeWasSupplied)
            error("planner:UnsupportedPoleTargetDerivative", ...
                "With WrapY on, a moving target's goal y velocity and acceleration must be " + ...
                "zero, or matched to the target (MatchTargetVelocity, MatchTargetAcceleration) " + ...
                "and not also supplied.");
        end
    end
end

%% Section 3: Match The Requested Target Velocity Or Acceleration

% Read the target's velocity and acceleration from its continuous path, so
% a path that crosses a wrapped end gives its true rate, not a jump.
if options.MatchTargetVelocity || options.MatchTargetAcceleration
    if isempty(goalState.targetMotion)
        error('planner:MissingTarget', ...
            ['MatchTargetVelocity or MatchTargetAcceleration is enabled, ' ...
            'but goalState.targetMotion is empty. Provide a target trajectory ' ...
            'or disable both matching options.']);
    end
    derivativeFieldNames  = ["velocity_units_s", "acceleration_units_s2"];
    matchTargetDerivative = [options.MatchTargetVelocity, options.MatchTargetAcceleration];
    [~, targetVelocity_units_s, targetAcceleration_units_s2] = ...
        obstacleAvoidance.input.targetPositionAtTime(goalState.targetMotion, goalState.time_s);
    targetDerivativeValues = [targetVelocity_units_s; targetAcceleration_units_s2];

    % Each row holds one derivative; process only the requested matches.
    for derivativeIndex = find(matchTargetDerivative)
        derivativeFieldName   = derivativeFieldNames(derivativeIndex);
        targetDerivativeValue = targetDerivativeValues(derivativeIndex, :);

        goalDerivativeWasSupplied = isfield(suppliedGoalState, derivativeFieldName) && ...
            ~isempty(suppliedGoalState.(derivativeFieldName));
        if goalDerivativeWasSupplied
            % Compare any supplied goal velocity or acceleration with the target:
            % x with x and y with y. Reject differences larger than the tolerance.
            goalTargetDerivativeDifference = abs(goalState.(derivativeFieldName) - targetDerivativeValue);
            if any(goalTargetDerivativeDifference > options.ConstraintTolerance)
                error('planner:ConflictingTargetDerivative', 'Explicit and matched target derivatives conflict.');
            end
        end
        goalState.(derivativeFieldName) = targetDerivativeValue;
    end
end

%% Section 4: Package The Request And Check For Distinct Endpoints

% Keep obstacles and the parent request beside the working states and limits.
% Include the inputs saved before wrapping or target matching so later steps
% can check the result against the original requirements.
request = struct( ...
    'initialState',   initialState, ...
    'goalState',      goalState, ...
    'limits',         limits, ...
    'options',        options, ...
    'obstacles',      {obstacles}, ...
    'parentRequest',  {parentRequest}, ...
    'originalInputs', struct( ...
        'suppliedLimits',     suppliedLimits, ...
        'requestedLimits',    requestedLimits, ...
        'suppliedGoalState',  suppliedGoalState, ...
        'requestedGoalState', requestedGoalState));

% For a fixed goal or fixed arrival time, reject a goal at the start position.
% For earliest arrival, the target could be at the start at the deadline
% but still be intercepted somewhere else earlier. Allow that case.
goalIsRequiredEndpoint = isempty(request.goalState.targetMotion) || ...
    request.options.GoalTimeMode == "fixedArrival";
endpointsCoincide = norm(request.goalState.position_units - ...
    request.initialState.position_units) <= request.options.ConstraintTolerance;
if goalIsRequiredEndpoint && endpointsCoincide
    error("planTrajectory:CoincidentEndpoints", "Initial and goal positions must be distinct.");
end

end

%% Section 5: Local Functions

function [initialState, goalState, limits, options] = createDefaults()
    % Set default start and goal states, workspace limits, and planner options.
    % By default, the vehicle starts and finishes with velocity = 0 and acceleration = 0.
    initialState = struct( ...
        "time_s",                0, ...
        "position_units",        [-4 0], ...
        "velocity_units_s",      [0 0], ...
        "acceleration_units_s2", [0 0]);

    goalState = struct( ...
        "time_s",                12, ...
        "position_units",        [4 0], ...
        "velocity_units_s",      [0 0], ...
        "acceleration_units_s2", [0 0], ...
        "targetMotion",          []);

    limits = struct( ...
        "xInterval_units",         [-180 180], ...
        "yInterval_units",         [-90 90], ...
        "maxVelocity_units_s",      [2 2], ...
        "maxAcceleration_units_s2", [2 2], ...
        "maxJerk_units_s3",         [4 4]);

    options = struct( ...
        "GoalTimeMode",                      "fixedArrival", ...
        "SampleTime_s",                      0.05, ...
        "ConstraintTolerance",               1e-8, ...
        "CollisionClearanceTolerance_units", 1e-7, ...
        "ArrivalTimeTolerance_s",            1e-8, ...
        "WrapX",                             "false", ...
        "WrapY",                             "false", ...
        "MatchTargetVelocity",               false, ...
        "MatchTargetAcceleration",           false, ...
        "TemporalResolution_s",              0.5, ...
        "SpatialProbeIterationLimit",        2, ...
        "BestSoFarRefinementTrialLimit",     0, ...
        "MaxArrivalTrials",                  100, ...
        "MaxArrivalCandidates",              4096);
end

function state = normalizeState(state, defaults, argumentName, evaluateTargetPosition)
    % Fill missing state fields with defaults and check the supplied values.

    if ~isstruct(state) || ~isscalar(state)
        error("planTrajectory:InvalidState", "%s must be a scalar struct.", argumentName);
    end

    % Report unknown field names so a typing mistake does not go unnoticed.
    allowedStateFieldNames = string(fieldnames(defaults));
    unknownFieldNames      = setdiff(string(fieldnames(state)), allowedStateFieldNames);

    if ~isempty(unknownFieldNames)
        error("planTrajectory:UnsupportedStateField", "%s contains unsupported fields: %s.", ...
            argumentName, strjoin(unknownFieldNames, ", "));
    end

    % Use defaults for missing or empty fields; keep values the caller supplied.
    for fieldName = reshape(allowedStateFieldNames, 1, [])
        if ~isfield(state, fieldName) || isempty(state.(fieldName))
            state.(fieldName) = defaults.(fieldName);
        end
    end

    % Check time before using it to look up the target position.
    validateattributes(state.time_s, {'numeric'}, {'real', 'finite', 'scalar'});
    hasTargetMotion = isfield(state, 'targetMotion') && ~isempty(state.targetMotion);

    if hasTargetMotion && evaluateTargetPosition
        state.position_units = obstacleAvoidance.input.targetPositionAtTime(state.targetMotion, state.time_s);
    end

    % When position is deferred, the main function calculates it from the target path.
    positionIsDeferred = hasTargetMotion && ~evaluateTargetPosition;
    stateFieldNames    = ["position_units", "velocity_units_s", "acceleration_units_s2"];

    for fieldName = stateFieldNames
        if fieldName == "position_units" && positionIsDeferred
            continue
        end

        stateValue        = state.(fieldName);
        stateValueIsValid = isnumeric(stateValue) && isreal(stateValue) && ...
            isvector(stateValue) && numel(stateValue) == 2 && all(isfinite(stateValue));

        if ~stateValueIsValid
            error("planTrajectory:InvalidState", ...
                "%s.%s must contain two finite, real numeric values.", argumentName, fieldName);
        end

        % Accept [x y] or [x; y], then store both as [x y] so later calculations
        % compare x with x and y with y.
        state.(fieldName) = reshape(double(stateValue), 1, 2);
    end

    state.time_s = double(state.time_s);
end

function limits = normalizeLimits(limits, defaults)
    % Fill missing limits with defaults and check workspace, speed, acceleration, and jerk limits.

    if ~isstruct(limits) || ~isscalar(limits)
        error("planTrajectory:InvalidLimits", "limits must be a scalar struct.");
    end

    limitFieldNames   = string(fieldnames(defaults));
    unknownFieldNames = setdiff(string(fieldnames(limits)), limitFieldNames);

    if ~isempty(unknownFieldNames)
        error("planTrajectory:UnsupportedLimitField", ...
            "Unsupported limit fields: %s.", strjoin(unknownFieldNames, ", "));
    end

    % Use defaults for missing or empty limits.
    for fieldName = reshape(limitFieldNames, 1, [])
        if ~isfield(limits, fieldName) || isempty(limits.(fieldName))
            limits.(fieldName) = defaults.(fieldName);
        end
    end

    % Each workspace interval must be [minimum maximum], with minimum < maximum.
    for fieldName = ["xInterval_units", "yInterval_units"]
        interval_units  = limits.(fieldName);
        intervalIsValid = isnumeric(interval_units) && isreal(interval_units) && ...
            isvector(interval_units) && numel(interval_units) == 2 && ...
            all(isfinite(interval_units)) && interval_units(2) > interval_units(1);

        if ~intervalIsValid
            error("planTrajectory:InvalidWorkspace", ...
                "%s must contain two finite, real numeric values with minimum < maximum.", fieldName);
        end

        limits.(fieldName) = reshape(double(interval_units), 1, 2);
    end

    % Specify speed, acceleration, and jerk limits in the same format:
    % one number for each limit, or an [x y] pair for each. Do not mix formats.
    derivativeLimitFieldNames = ["maxVelocity_units_s", "maxAcceleration_units_s2", "maxJerk_units_s3"];
    limitElementCounts        = arrayfun(@(limitName) numel(limits.(limitName)), derivativeLimitFieldNames);

    if any(limitElementCounts ~= limitElementCounts(1))
        error('planTrajectory:MixedLimitModes', ...
            ['Velocity, acceleration, and jerk limits must all be scalars ' ...
            'or all be two-element vectors.']);
    end

    for fieldName = derivativeLimitFieldNames
        limitValue        = limits.(fieldName);
        limitValueIsValid = isnumeric(limitValue) && isreal(limitValue) && ...
            isvector(limitValue) && any(numel(limitValue) == [1 2]) && ...
            all(isfinite(limitValue)) && all(limitValue > 0);

        if ~limitValueIsValid
            error("planTrajectory:InvalidDerivativeLimit", ...
                "%s must contain one or two finite, real numeric values > 0.", fieldName);
        end

        limitValue = double(limitValue);
        if isscalar(limitValue)
            % For a single combined limit: axis limit = combined limit / sqrt(2).
            % Example: a speed limit of 10 gives about 7.07 on each axis.
            % This keeps combined speed <= 10, but also limits motion along
            % just one axis to 7.07.
            limitValue = [limitValue limitValue] / sqrt(2);
        end

        % A very small combined limit must still give positive axis limits.
        if any(limitValue <= 0)
            error("planTrajectory:InvalidDerivativeLimit", ...
                "%s must remain > 0 after conversion to axis limits.", fieldName);
        end

        limits.(fieldName) = reshape(limitValue, 1, 2);
    end
end

function options = resolveOptions(options, defaults)
    % Fill missing planner options with defaults and check their values.

    if ~isstruct(options) || ~isscalar(options)
        error("planTrajectory:InvalidOptions", "options must be a scalar struct.");
    end

    % Use the supplied options where present and defaults for the rest.
    % Warn about unknown option names and ignore them.
    [options, unknownFieldNames] = obstacleAvoidance.input.resolveOptions(defaults, options);

    if ~isempty(unknownFieldNames)
        warning("planTrajectory:UnknownOptions", ...
            "Ignoring unknown option fields: %s.", strjoin(unknownFieldNames, ", "));
    end

    % Fixed arrival means arrive at the requested time. Earliest arrival tries
    % to minimize arrival time, using the requested time as the deadline.
    options.GoalTimeMode = string(options.GoalTimeMode);
    allowedGoalTimeModes = ["fixedArrival", "earliestArrival"];
    goalTimeModeIsValid  = isscalar(options.GoalTimeMode) && any(options.GoalTimeMode == allowedGoalTimeModes);

    if ~goalTimeModeIsValid
        error("planner:UnsupportedGoalTimeMode", "GoalTimeMode must be fixedArrival or earliestArrival.");
    end

    % Each time step or tolerance must be one finite number > 0.
    numericOptionNames = ["SampleTime_s", "ConstraintTolerance", ...
        "CollisionClearanceTolerance_units", "ArrivalTimeTolerance_s", ...
        "TemporalResolution_s"];
    positiveScalarAttributes = {'real', 'finite', 'scalar', 'positive'};

    for optionName = numericOptionNames
        validateattributes(options.(optionName), {'numeric'}, positiveScalarAttributes);
        options.(optionName) = double(options.(optionName));
    end

    % Each wrap option is "false", "both", "forward", or "backward". For
    % compatibility, true means "both" and false means "false".
    allowedWrapModes = ["false", "both", "forward", "backward"];
    for optionName = ["WrapX", "WrapY"]
        wrapMode = options.(optionName);
        if (islogical(wrapMode) || isnumeric(wrapMode)) && isscalar(wrapMode) && ...
                any(wrapMode == [0, 1])
            wrapMode = allowedWrapModes(1 + logical(wrapMode));
        elseif (ischar(wrapMode) && isrow(wrapMode)) || (isstring(wrapMode) && isscalar(wrapMode))
            wrapMode = lower(string(wrapMode));
        end
        if ~(isstring(wrapMode) && isscalar(wrapMode) && any(wrapMode == allowedWrapModes))
            error("planner:InvalidWrapOption", ...
                "%s must be false, both, forward, or backward.", optionName);
        end
        options.(optionName) = wrapMode;
    end

    % Check each on/off option and store it as a single true or false value.
    logicalOptionNames = ["MatchTargetVelocity", "MatchTargetAcceleration"];

    for optionName = logicalOptionNames
        options.(optionName) = obstacleAvoidance.input.normalizeLogicalScalar( ...
            options.(optionName), optionName, "planner:InvalidLogicalOption");
    end

    % Trial and iteration limits must be whole numbers > 0.
    integerOptionNames = ["SpatialProbeIterationLimit", ...
        "MaxArrivalTrials", "MaxArrivalCandidates"];

    for optionName = integerOptionNames
        validateattributes(options.(optionName), {'numeric'}, ...
            {'scalar', 'finite', 'integer', 'positive'});
        options.(optionName) = double(options.(optionName));
    end

    % The extra refinement limit can be 0 to skip those extra trials.
    validateattributes(options.BestSoFarRefinementTrialLimit, {'numeric'}, ...
        {'scalar', 'finite', 'integer', 'nonnegative'});
    options.BestSoFarRefinementTrialLimit = ...
        double(options.BestSoFarRefinementTrialLimit);

    % The spatial probe can use at most 35 BMTP iterations.
    if options.SpatialProbeIterationLimit > 35
        error("planner:InvalidSpatialProbeIterationLimit", ...
            "SpatialProbeIterationLimit must not exceed the full BMTP limit of 35.");
    end
end

