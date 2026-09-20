function result = planRequest(obstacles, initialState, goalState, limits, options, outerRequest)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.planning.planRequest()
%   result = obstacleAvoidance.planning.planRequest( ...
%       obstacles, initialState, goalState, limits, options, outerRequest)
%**************************************************************************
% PURPOSE
%   - Normalize one planner request and dispatch it through periodic-image
%     planning or the named nonperiodic planning core.
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
%   - outerRequest (scalar struct or [])
%       Private provenance for a derived request, built by
%       obstacleAvoidance.planning.createOuterRequest; [] for a public call.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Stable planner result. Expected planning failure returns
%       Success = false; invalid input throws an error.
%   - options (scalar struct, zero-input call)
%       Fully resolved planner defaults.
%**************************************************************************
% UNITS
%   - Positions are coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Normalize And Dispatch The Request

if nargin == 0
    [~, ~, ~, defaultOptions] = createDefaults();
    result = resolveOptions(struct(), defaultOptions);
    return
end

[defaultInitialState, defaultGoalState, defaultLimits, defaultOptions] = createDefaults();
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
suppliedLimits     = limits;
suppliedGoalState  = goalState;
initialState       = normalizeState(initialState, defaultInitialState, "initialState", true);
goalState          = normalizeState(goalState, defaultGoalState, "goalState", false);
requestTimeIsInvalid = goalState.time_s <= initialState.time_s;
if ~isempty(goalState.targetMotion)
    try
        targetPosition_units = obstacleAvoidance.input.targetPositionAtTime( ...
            goalState.targetMotion, goalState.time_s);
    catch exception
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
limits             = normalizeLimits(limits, defaultLimits);
options            = resolveOptions(options, defaultOptions);
requestedLimits    = limits;
requestedGoalState = goalState;

% Matched goal derivatives come from the target motion at the goal time.
if options.MatchTargetVelocity || options.MatchTargetAcceleration
    if isempty(goalState.targetMotion)
        error('planner:MissingTarget', 'Derivative matching requires goalState.targetMotion.');
    end
    derivativeNames     = ["velocity_units_s", "acceleration_units_s2"];
    matches             = [options.MatchTargetVelocity, options.MatchTargetAcceleration];
    [~, tgtVel, tgtAcc] = obstacleAvoidance.input.targetPositionAtTime(goalState.targetMotion, goalState.time_s);
    derivatives         = [tgtVel; tgtAcc];
    for derivativeIndex = find(matches)
        derivativeName   = derivativeNames(derivativeIndex);
        targetDerivative = derivatives(derivativeIndex, :);

        hasExplicitDerivative = isfield(suppliedGoalState, derivativeName) && ...
            ~isempty(suppliedGoalState.(derivativeName));
        if hasExplicitDerivative
            % Compare the normalized 1-by-2 row so a supplied column is not
            % broadcast into a 2-by-2 residual.
            derivativeResidual = abs(goalState.(derivativeName) - targetDerivative);
            if any(derivativeResidual > options.ConstraintTolerance)
                error('planner:ConflictingTargetDerivative', 'Explicit and matched target derivatives conflict.');
            end
        end
        goalState.(derivativeName) = targetDerivative;
    end
end

% A wrapped axis is planned in the unwrapped frame. A moving target is lifted
% by continuity from the initial position, a fixed goal takes its nearest
% image, and each wrapped interval becomes the reach band of the request.
wrapAxes = [options.WrapX options.WrapY];
if any(wrapAxes)
    intervalNames   = ["xInterval_units", "yInterval_units"];
    intervals_units = [limits.xInterval_units; limits.yInterval_units];
    if ~isempty(goalState.targetMotion)
        goalState.targetMotion   = obstacleAvoidance.input.liftPeriodicTarget( ...
            goalState.targetMotion, initialState.position_units, intervals_units, wrapAxes);
        goalState.position_units = obstacleAvoidance.input.targetPositionAtTime( ...
            goalState.targetMotion, goalState.time_s);
    end
    for axisIndex = find(wrapAxes)
        period_units = diff(intervals_units(axisIndex, :));
        start_units  = initialState.position_units(axisIndex);
        goal_units   = goalState.position_units(axisIndex);
        reach_units  = limits.maxVelocity_units_s(axisIndex) * (goalState.time_s - initialState.time_s);
        if isempty(goalState.targetMotion)
            periodOffset = floor((start_units - goal_units) / period_units + 0.5);
            goalState.position_units(axisIndex) = goal_units + period_units * periodOffset;
        end
        limits.(intervalNames(axisIndex)) = start_units + [-reach_units reach_units];
    end
end

request = struct( ...
    'initialState', initialState, ...
    'goalState',    goalState, ...
    'limits',       limits, ...
    'options',      options);
requestContext = struct( ...
    'obstacles',          {obstacles}, ...
    'suppliedLimits',     suppliedLimits, ...
    'requestedLimits',    requestedLimits, ...
    'suppliedGoalState',  suppliedGoalState, ...
    'requestedGoalState', requestedGoalState, ...
    'outerRequest',       {outerRequest});

% A moving target's deadline position only bounds the earliest-arrival
% search; a fixed goal or a fixed-arrival intercept is a required endpoint.
goalIsRequiredEndpoint = isempty(request.goalState.targetMotion) || ...
    request.options.GoalTimeMode == "fixedArrival";
endpointsCoincide = norm(request.goalState.position_units - ...
    request.initialState.position_units) <= request.options.ConstraintTolerance;
if goalIsRequiredEndpoint && endpointsCoincide
    error("planTrajectory:CoincidentEndpoints", "Initial and goal positions must be distinct.");
end

% Every goal image inside the band is planned as a plain request in the
% unwrapped frame and accepted against this periodic request.
if any(wrapAxes)
    result = obstacleAvoidance.planning.planPeriodicRequest(request, requestContext);
    return
end

result = obstacleAvoidance.planning.planNormalizedRequest(request, requestContext);
end


function [initialState, goalState, limits, options] = createDefaults()
    % Provide fallback state, limit, and option values for the public planner.
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
        "WrapX",                             false, ...
        "WrapY",                             false, ...
        "MatchTargetVelocity",               false, ...
        "MatchTargetAcceleration",           false, ...
        "TemporalResolution_s",              0.5, ...
        "SpatialProbeIterationLimit",        2, ...
        "IncumbentRefinementTrialLimit",     0, ...
        "MaxArrivalTrials",                  100, ...
        "MaxArrivalCandidates",              4096);
end

function state = normalizeState(state, defaults, argumentName, evaluateTarget)
    % Resolve omitted rest-to-rest fields and reject unsupported state data.
    if ~isstruct(state) || ~isscalar(state)
        error("planTrajectory:InvalidState", "%s must be a scalar struct.", argumentName);
    end
    allowedStateFields = string(fieldnames(defaults));
    unknownFields      = setdiff(string(fieldnames(state)), allowedStateFields);
    if ~isempty(unknownFields)
        error("planTrajectory:UnsupportedStateField", "%s contains unsupported fields: %s.", ...
            argumentName, strjoin(unknownFields, ", "));
    end
    for fieldName = reshape(allowedStateFields, 1, [])
        if ~isfield(state, fieldName) || isempty(state.(fieldName))
            state.(fieldName) = defaults.(fieldName);
        end
    end
    validateattributes(state.time_s, {'numeric'}, {'real', 'finite', 'scalar'});
    hasTargetMotion = isfield(state, 'targetMotion') && ~isempty(state.targetMotion);
    if hasTargetMotion && evaluateTarget
        state.position_units = obstacleAvoidance.input.targetPositionAtTime(state.targetMotion, state.time_s);
    end
    for fieldName = ["position_units", "velocity_units_s", "acceleration_units_s2"]
        if fieldName == "position_units" && hasTargetMotion && ~evaluateTarget
            continue
        end
        value = double(state.(fieldName));
        valueIsValid = isnumeric(state.(fieldName)) && isreal(value) && isvector(value) && ...
            numel(value) == 2 && all(isfinite(value));
        if ~valueIsValid
            error("planTrajectory:InvalidState", "%s.%s must be a finite 1-by-2 row.", argumentName, fieldName);
        end
        state.(fieldName) = reshape(value, 1, 2);
    end
    state.time_s = double(state.time_s);
end

function limits = normalizeLimits(limits, defaults)
    % Resolve the small fixed set of workspace and derivative limits.
    if ~isstruct(limits) || ~isscalar(limits)
        error("planTrajectory:InvalidLimits", "limits must be a scalar struct.");
    end
    limitNames    = string(fieldnames(defaults));
    unknownFields = setdiff(string(fieldnames(limits)), limitNames);
    if ~isempty(unknownFields)
        error("planTrajectory:UnsupportedLimitField", "Unsupported limit fields: %s.", strjoin(unknownFields, ", "));
    end
    for fieldName = reshape(limitNames, 1, [])
        if ~isfield(limits, fieldName) || isempty(limits.(fieldName))
            limits.(fieldName) = defaults.(fieldName);
        end
    end
    for fieldName = ["xInterval_units", "yInterval_units"]
        interval = double(limits.(fieldName));
        intervalIsValid = isnumeric(limits.(fieldName)) && isreal(interval) && isvector(interval) && ...
            numel(interval) == 2 && all(isfinite(interval)) && interval(2) > interval(1);
        if ~intervalIsValid
            error("planTrajectory:InvalidWorkspace", "%s must be a finite increasing 1-by-2 row.", fieldName);
        end
        limits.(fieldName) = reshape(interval, 1, 2);
    end
    derivativeLimitNames = ["maxVelocity_units_s", "maxAcceleration_units_s2", "maxJerk_units_s3"];
    limitSizes           = arrayfun(@(limitName) numel(limits.(limitName)), derivativeLimitNames);
    if any(limitSizes ~= limitSizes(1))
        error('planTrajectory:MixedLimitModes', ...
            ['Velocity, acceleration, and jerk limits must all be scalars ' ...
            'or all be two-element vectors.']);
    end
    for fieldName = derivativeLimitNames
        value = double(limits.(fieldName));
        if isscalar(value)
            value = [value value] / sqrt(2);
        end
        valueIsValid = isnumeric(limits.(fieldName)) && isreal(value) && isvector(value) && ...
            numel(value) == 2 && all(isfinite(value)) && all(value > 0);
        if ~valueIsValid
            error("planTrajectory:InvalidDerivativeLimit", "%s must be a positive scalar or finite 1-by-2 row.", fieldName);
        end
        limits.(fieldName) = reshape(value, 1, 2);
    end
end

function options = resolveOptions(options, defaults)
    % Resolve all BMTP controls in one place and warn once about unknown fields.
    if ~isstruct(options) || ~isscalar(options)
        error("planTrajectory:InvalidOptions", "options must be a scalar struct.");
    end

    [options, unknownFields] = obstacleAvoidance.input.resolveOptions(defaults, options);
    if ~isempty(unknownFields)
        warning("planTrajectory:UnknownOptions", "Ignoring unknown option fields: %s.", strjoin(unknownFields, ", "));
    end

    options.GoalTimeMode  = string(options.GoalTimeMode);
    allowedGoalTimeModes = ["fixedArrival", "earliestArrival"];
    goalTimeModeIsValid  = isscalar(options.GoalTimeMode) && any(options.GoalTimeMode == allowedGoalTimeModes);
    if ~goalTimeModeIsValid
        error("planner:UnsupportedGoalTimeMode", "GoalTimeMode must be fixedArrival or earliestArrival.");
    end

    numericOptionNames = ["SampleTime_s", "ConstraintTolerance", ...
        "CollisionClearanceTolerance_units", "ArrivalTimeTolerance_s", ...
        "TemporalResolution_s"];
    positiveScalarAttributes = {'real', 'finite', 'scalar', 'positive'};
    for fieldName = numericOptionNames
        validateattributes(options.(fieldName), {'numeric'}, positiveScalarAttributes);
        options.(fieldName) = double(options.(fieldName));
    end

    logicalOptionNames = ["WrapX", "WrapY", "MatchTargetVelocity", "MatchTargetAcceleration"];
    for optionName = logicalOptionNames
        options.(optionName) = obstacleAvoidance.input.normalizeLogicalScalar( ...
            options.(optionName), optionName, "planner:InvalidLogicalOption");
    end

    integerOptionNames = ["SpatialProbeIterationLimit", ...
        "MaxArrivalTrials", "MaxArrivalCandidates"];
    for optionName = integerOptionNames
        validateattributes(options.(optionName), {'numeric'}, ...
            {'scalar', 'finite', 'integer', 'positive'});
        options.(optionName) = double(options.(optionName));
    end
    validateattributes(options.IncumbentRefinementTrialLimit, {'numeric'}, ...
        {'scalar', 'finite', 'integer', 'nonnegative'});
    options.IncumbentRefinementTrialLimit = ...
        double(options.IncumbentRefinementTrialLimit);
    if options.SpatialProbeIterationLimit > 35
        error("planner:InvalidSpatialProbeIterationLimit", ...
            "SpatialProbeIterationLimit must not exceed the full BMTP limit of 35.");
    end
end

