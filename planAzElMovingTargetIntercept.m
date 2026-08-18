function [result, diagnostics] = planAzElMovingTargetIntercept( ...
        obstacles, initialState, targetMotion, limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = planAzElMovingTargetIntercept()
%   result = planAzElMovingTargetIntercept( ...
%       initialState, targetMotion, limits, optionOverrides)
%   result = planAzElMovingTargetIntercept( ...
%       obstacles, initialState, targetMotion, limits, optionOverrides)
%   [result, diagnostics] = planAzElMovingTargetIntercept( ...
%       obstacles, initialState, targetMotion, limits, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Resolve a sampled or constant-velocity target into the earliest
%     feasible or a specified intercept state, then use planAzElMotion.
%   - Support interception through static or moving obstacles without
%     introducing a second search or retiming implementation.
%**************************************************************************
% INPUTS
%   - obstacles (canonical obstacle data, nested cell array, or [])
%       Static or time-varying protected geometry. The four-input legacy
%       call omits this argument and remains obstacle-free.
%   - initialState (scalar struct)
%       time_s, position_deg, and optional velocity/acceleration fields.
%   - targetMotion (scalar struct)
%       Either sampled motion with strictly increasing time_s (N-by-1),
%       position_deg (N-by-2 [azimuth elevation]), and optional
%       InterpolationMethod ("pchip", default, or "linear"); or the
%       deprecated-compatible constant-velocity form referenceTime_s,
%       referencePosition_deg, and velocity_deg_s. Sampled motion is never
%       extrapolated and must span initialState.time_s through interception.
%   - limits (scalar struct)
%       Limits accepted by planAzElMotion.
%   - optionOverrides (scalar struct, optional; default struct())
%       .InterceptMode             "earliestArrival" or "specifiedTime"
%       .SpecifiedInterceptTime_s  required for specifiedTime
%       .MaximumSearchDuration_s   earliest-intercept horizon (60)
%       .InitialSearchStep_s       maximum arbitrary-path scan step (0.5)
%       .SearchTimeTolerance_s     earliest-time tolerance (1e-4)
%       .MaximumSearchIterations   bisection limit (60)
%       .MatchTargetVelocity       match target angular rate (true for the
%                                  constant model, false for sampled motion)
%       .TargetTrackSampleCount    plot samples (121)
%       .PlannerOptions            planAzElMotion option overrides
%     Earliest mode numerically scans arbitrary sampled paths at the target
%     knots and at no more than InitialSearchStep_s between knots, then
%     bisects the first detected feasibility transition. With obstacles,
%     a velocity-only displacement bound safely removes times that are
%     impossible, and a point query removes times when the target is inside
%     protected geometry, before the full planner and continuous collision
%     query; the interpolated first proposal is always rechecked. Search
%     stops when the target first leaves PlannerOptions.AzimuthInterval_deg
%     or the physical elevation interval [-90, 90] degrees.
%     EarliestCertified
%     is true only for the obstacle-free constant-velocity receding-ray model;
%     arbitrary-path results report EarliestNumericallyResolved instead. A
%     stationary initial
%     state is required because fixed-arrival slack may use an initial hold.
%     Velocity matching requires the instantaneous target velocity to be
%     tangent to the direct start-to-intercept vector. Use false for a
%     position-only intercept of a general or obstacle-avoiding path.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Compact planner result with the intercept state and validation.
%       Expected planner infeasibility is returned by planAzElMotion;
%       invalid target contracts throw. If the target leaves the az/el
%       frame before a feasible intercept, Success is false and
%       TerminationReason is "targetLeftAzElFrame".
%   - diagnostics (scalar struct, optional)
%       Full planner diagnostics, intercept search history, normalized
%       target motion, and target-track data for expert inspection.
%**************************************************************************
% UNITS
%   - Angles are degrees; time is seconds; velocity is deg/s.
%**************************************************************************

%% Section 1: Resolve Options & Validate Inputs

defaultOptions = struct( ...
    "InterceptMode", "earliestArrival", ...
    "SpecifiedInterceptTime_s", NaN, ...
    "MaximumSearchDuration_s", 60, ...
    "InitialSearchStep_s", 0.5, ...
    "SearchTimeTolerance_s", 1e-4, ...
    "MaximumSearchIterations", 60, ...
    "MatchTargetVelocity", true, ...
    "TargetTrackSampleCount", 121, ...
    "PlannerOptions", struct());
if nargin == 0
    result = defaultOptions;
    diagnostics = struct();
    return;
end
interceptPlanningTimer = tic;

% The original obstacle-free interface placed initialState first. Detect
% that stable record shape before validating either form so existing
% callers continue to use the same implementation without an adapter.
usesObstacleFreeCall = isstruct(obstacles) && isscalar(obstacles) && ...
    isfield(obstacles, "position_deg") && isfield(obstacles, "time_s");
if usesObstacleFreeCall
    legacyInitialState = obstacles;
    legacyTargetMotion = initialState;
    legacyLimits = targetMotion;
    legacyOptions = struct();
    if nargin >= 4 && ~isempty(limits)
        legacyOptions = limits;
    end
    obstacles = [];
    initialState = legacyInitialState;
    targetMotion = legacyTargetMotion;
    limits = legacyLimits;
    optionOverrides = legacyOptions;
elseif nargin < 4
    error("planAzElMovingTargetIntercept:NotEnoughInputs", ...
        "Provide obstacles, initialState, targetMotion, and limits.");
elseif nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
options = resolveInterceptOptions(defaultOptions, optionOverrides);
initialState = validateInterceptInitialState(initialState);
targetMotion = validateTargetMotion(targetMotion);
if targetMotion.ModelType == "sampledTrajectory" && ...
        (~isfield(optionOverrides, "MatchTargetVelocity") || ...
        isempty(optionOverrides.MatchTargetVelocity))
    options.MatchTargetVelocity = false;
end
if targetMotion.ModelType == "sampledTrajectory" && ...
        (initialState.time_s < targetMotion.DomainTime_s(1) || ...
        initialState.time_s > targetMotion.DomainTime_s(2))
    error("planAzElMovingTargetIntercept:InitialTimeOutsideTargetDomain", ...
        "initialState.time_s must lie within targetMotion.time_s so the " + ...
        "target is defined throughout the planned intercept.");
end

%% Section 2: Resolve The Concrete Intercept State

interceptMode = lower(string(options.InterceptMode));
searchDiagnostics = emptyInterceptSearchDiagnostics();
if interceptMode == "specifiedtime"
    interceptTime_s = options.SpecifiedInterceptTime_s;
    validateattributes(interceptTime_s, {'numeric'}, ...
        {'real', 'finite', 'scalar', '>', initialState.time_s});
    [frameExitTime_s, targetLeavesFrame] = targetFrameExitTime( ...
        targetMotion, initialState.time_s, interceptTime_s, options);
    targetLeftFrameBeforeIntercept = targetLeavesFrame && ...
        interceptTime_s > frameExitTime_s + options.SearchTimeTolerance_s;
    if targetLeftFrameBeforeIntercept
        interceptTime_s = frameExitTime_s;
        searchDiagnostics.Success = false;
        searchDiagnostics.TerminationReason = "targetLeftAzElFrame";
        searchDiagnostics.Message = ...
            "Cannot catch up to the target before it leaves the az/el frame.";
    else
        searchDiagnostics.Message = ...
            "The caller supplied the intercept time; no search was required.";
    end
else
    validateEarliestInitialVelocity(initialState);
    requestedSearchEndTime_s = initialState.time_s + ...
        options.MaximumSearchDuration_s;
    [frameExitTime_s, targetLeavesFrame] = targetFrameExitTime( ...
        targetMotion, initialState.time_s, requestedSearchEndTime_s, options);
    targetAtInitial_deg = evaluateTargetMotion( ...
        targetMotion, initialState.time_s);
    if norm(targetAtInitial_deg - initialState.position_deg) <= 1e-10
        error("planAzElMovingTargetIntercept:InitialCoincidence", ...
            "The target is already at initialState.position_deg.");
    end
    searchOptions = options;
    if targetLeavesFrame
        searchOptions.MaximumSearchDuration_s = min( ...
            options.MaximumSearchDuration_s, ...
            frameExitTime_s - initialState.time_s);
    end
    [interceptTime_s, searchDiagnostics] = ...
        searchEarliestInterceptTime(obstacles, initialState, ...
        targetMotion, limits, searchOptions);
    targetLeftFrameBeforeIntercept = targetLeavesFrame && ...
        ~searchDiagnostics.Success;
    if targetLeftFrameBeforeIntercept
        searchDiagnostics.TerminationReason = "targetLeftAzElFrame";
        searchDiagnostics.Message = ...
            "Cannot catch up to the target before it leaves the az/el frame.";
    end
end
[interceptPosition_deg, targetVelocityAtIntercept_deg_s] = ...
    evaluateTargetMotion(targetMotion, interceptTime_s);
validateVelocityMatchGeometry(initialState, interceptPosition_deg, ...
    targetVelocityAtIntercept_deg_s, options.MatchTargetVelocity);
goalVelocity_deg_s = [0 0];
if options.MatchTargetVelocity
    goalVelocity_deg_s = targetVelocityAtIntercept_deg_s;
end
goalState = struct( ...
    "time_s", interceptTime_s, ...
    "position_deg", interceptPosition_deg, ...
    "velocity_deg_s", goalVelocity_deg_s, ...
    "acceleration_deg_s2", [0 0]);

%% Section 3: Run The Maintained Planner

searchDiagnostics.ElapsedPlanningTime_s = toc(interceptPlanningTimer);
plannerOptions = options.PlannerOptions;
plannerOptions.GoalTimeMode = "fixedArrival";
if searchDiagnostics.CandidatePlannerGoalTimeMode == ...
        "analyticresttorestthenfixedarrival"
    plannerOptions = requireMeshRefinementPasses(plannerOptions, 2);
end
[result, plannerDiagnostics] = planAzElMotion( ...
    obstacles, initialState, goalState, limits, plannerOptions);
plannerSucceeded = result.Success;
plannerValidation = result.Validation;
plannerSearchDiagnostics = plannerDiagnostics.Search;

trackTime_s = targetTrackTimes(targetMotion, initialState.time_s, ...
    interceptTime_s, options.TargetTrackSampleCount);
if plannerSucceeded
    trackTime_s = unique([trackTime_s; result.time_s]);
end
trackPosition_deg = evaluateTargetMotion(targetMotion, trackTime_s);

%% Section 4: Validate The Intercept Contract

targetPositionAtIntercept_deg = interceptPosition_deg;
positionError_deg = Inf;
targetVelocityError_deg_s = Inf;
requestedVelocityError_deg_s = Inf;
timeError_s = Inf;
positionMatched = false;
targetVelocityMatched = false;
requestedVelocityMatched = false;
timeMatched = false;
hasTimedEndpoint = plannerSucceeded && ...
    ~isempty(result.position_deg) && ...
    ~isempty(result.velocity_deg_s) && ...
    isfinite(result.goalLineInterceptTime_s);
if hasTimedEndpoint
    [targetPositionAtIntercept_deg, actualTargetVelocity_deg_s] = ...
        evaluateTargetMotion( ...
        targetMotion, result.goalLineInterceptTime_s);
    actualTerminalPosition_deg = ...
        result.position_deg(end, :);
    actualTerminalVelocity_deg_s = ...
        result.velocity_deg_s(end, :);
    positionError_deg = norm( ...
        actualTerminalPosition_deg - targetPositionAtIntercept_deg);
    targetVelocityError_deg_s = norm( ...
        actualTerminalVelocity_deg_s - actualTargetVelocity_deg_s);
    requestedVelocityError_deg_s = norm( ...
        actualTerminalVelocity_deg_s - goalVelocity_deg_s);
    timeError_s = abs( ...
        result.goalLineInterceptTime_s - interceptTime_s);
    positionMatched = positionError_deg <= 1e-7;
    targetVelocityMatched = targetVelocityError_deg_s <= 1e-7;
    requestedVelocityMatched = requestedVelocityError_deg_s <= 1e-7;
    timeMatched = timeError_s <= ...
        max(1e-7, options.SearchTimeTolerance_s);
end
velocityRequirementSatisfied = requestedVelocityMatched;
earliestResolved = interceptMode == "specifiedtime" || ...
    searchDiagnostics.Success;
interceptPassed = result.Success && positionMatched && ...
    velocityRequirementSatisfied && timeMatched && earliestResolved;
validationMessage = "Moving-target intercept validation passed.";
if ~interceptPassed
    validationMessage = "Moving-target intercept validation failed.";
end
interceptValidation = struct( ...
    "Passed", interceptPassed, ...
    "PlannerPassed", result.Success, ...
    "TargetPositionMatched", positionMatched, ...
    "VelocityMatchRequired", options.MatchTargetVelocity, ...
    "TargetVelocityMatched", targetVelocityMatched, ...
    "RequestedVelocityMatched", requestedVelocityMatched, ...
    "VelocityRequirementSatisfied", velocityRequirementSatisfied, ...
    "InterceptTimeMatched", timeMatched, ...
    "EarliestCertified", searchDiagnostics.EarliestCertified, ...
    "EarliestNumericallyResolved", ...
        searchDiagnostics.EarliestNumericallyResolved, ...
    "PositionError_deg", positionError_deg, ...
    "VelocityError_deg_s", targetVelocityError_deg_s, ...
    "RequestedVelocityError_deg_s", requestedVelocityError_deg_s, ...
    "TimeError_s", timeError_s, ...
    "Message", validationMessage);
result.Success = interceptPassed;
if targetLeftFrameBeforeIntercept
    result.TerminationReason = "targetLeftAzElFrame";
    result.Message = searchDiagnostics.Message;
elseif ~searchDiagnostics.Success
    result.TerminationReason = searchDiagnostics.TerminationReason;
    result.Message = searchDiagnostics.Message;
elseif ~interceptPassed
    if plannerSucceeded
        result.TerminationReason = "interceptValidationFailed";
    end
    result.Message = string(result.Message) + " " + validationMessage;
end
if ~interceptPassed
    result.Validation.Passed = false;
    result.Validation.Message = validationMessage + " " + ...
        string(result.Validation.Message);
    result.Validation.Issues = unique([ ...
        string(result.Validation.Issues(:)); "interceptContract"], ...
        "stable");
end

%% Section 5: Assemble Intercept Diagnostics

result.InterceptMode = options.InterceptMode;
result.RequestedInterceptTime_s = options.SpecifiedInterceptTime_s;
result.InterceptTime_s = interceptTime_s;
result.InterceptPosition_deg = interceptPosition_deg;
result.TargetPositionAtIntercept_deg = targetPositionAtIntercept_deg;
result.TargetVelocityAtIntercept_deg_s = targetVelocityAtIntercept_deg_s;
result.TargetFrameExitTime_s = frameExitTime_s;
result.TargetLeftFrameBeforeIntercept = targetLeftFrameBeforeIntercept;
if targetLeftFrameBeforeIntercept
    result.InterceptTime_s = NaN;
    result.InterceptPosition_deg = [NaN NaN];
    result.TargetPositionAtIntercept_deg = [NaN NaN];
    result.TargetVelocityAtIntercept_deg_s = [NaN NaN];
end
result.InterceptValidation = interceptValidation;
plannerOptionsResolved = result.Options;
result.Options = options;
finalPlannerElapsedPlanningTime_s = result.ElapsedPlanningTime_s;
result.ElapsedPlanningTime_s = toc(interceptPlanningTimer);

combinedSearchDiagnostics = mergeSearchDiagnostics( ...
    searchDiagnostics, plannerSearchDiagnostics);
combinedSearchDiagnostics.Success = result.Success;
combinedSearchDiagnostics.Message = result.Message;
combinedSearchDiagnostics.TerminationReason = result.TerminationReason;
combinedSearchDiagnostics.FinalPlannerElapsedPlanningTime_s = ...
    finalPlannerElapsedPlanningTime_s;
combinedSearchDiagnostics.ElapsedPlanningTime_s = ...
    result.ElapsedPlanningTime_s;
diagnostics = struct( ...
    "Planner", plannerDiagnostics, ...
    "PlannerOptions", plannerOptionsResolved, ...
    "PlannerValidation", plannerValidation, ...
    "InterceptSearch", searchDiagnostics, ...
    "Search", combinedSearchDiagnostics, ...
    "TargetMotion", targetMotion, ...
    "TargetTrack", struct( ...
        "time_s", trackTime_s, ...
        "position_deg", trackPosition_deg), ...
    "FinalPlannerElapsedPlanningTime_s", ...
    finalPlannerElapsedPlanningTime_s);
end

%% Section 6: Local Functions

function options = resolveInterceptOptions(defaultOptions, overrides)
% PURPOSE
%   - Merge and validate public moving-target intercept controls.
if ~isstruct(overrides) || ~isscalar(overrides)
    error("planAzElMovingTargetIntercept:InvalidOptions", ...
        "options must be a scalar struct.");
end

[options, unknownFields] = azElInternal.resolveOptions( ...
    defaultOptions, overrides);
if ~isempty(unknownFields)
    warning("planAzElMovingTargetIntercept:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownFields, ", "));
end
options.InterceptMode = lower(string(options.InterceptMode));
if ~isscalar(options.InterceptMode) || ~any( ...
        options.InterceptMode == ["earliestarrival" "specifiedtime"])
    error("planAzElMovingTargetIntercept:InvalidMode", ...
        "InterceptMode must be earliestArrival or specifiedTime.");
end
validateattributes(options.MaximumSearchDuration_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
validateattributes(options.InitialSearchStep_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
validateattributes(options.SearchTimeTolerance_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
if options.SearchTimeTolerance_s > options.MaximumSearchDuration_s
    error("planAzElMovingTargetIntercept:InvalidSearchTolerance", ...
        "SearchTimeTolerance_s cannot exceed MaximumSearchDuration_s.");
end
validateattributes(options.MaximumSearchIterations, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(options.TargetTrackSampleCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', '>=', 2});
options.MatchTargetVelocity = azElInternal.normalizeLogicalScalar( ...
    options.MatchTargetVelocity, "MatchTargetVelocity", ...
    "planAzElMovingTargetIntercept:InvalidMatchTargetVelocity");
if ~isstruct(options.PlannerOptions) || ~isscalar(options.PlannerOptions)
    error("planAzElMovingTargetIntercept:InvalidPlannerOptions", ...
        "PlannerOptions must be a scalar struct.");
end
end

function combined = mergeSearchDiagnostics(interceptSearch, plannerSearch)
% PURPOSE
%   - Keep intercept fields while adding the uniform planner diagnostics.
combined = interceptSearch;
plannerFieldNames = fieldnames(plannerSearch);
for fieldIndex = 1:numel(plannerFieldNames)
    fieldName = plannerFieldNames{fieldIndex};
    if ~isfield(combined, fieldName)
        combined.(fieldName) = plannerSearch.(fieldName);
    end
end
end

function [frameExitTime_s, targetLeavesFrame] = targetFrameExitTime( ...
        targetMotion, startTime_s, requestedEndTime_s, options)
% PURPOSE
%   - Find the first time the target leaves the configured azimuth interval
%     or the physical elevation interval.
plannerDefaults = planAzElMotion();
azimuthInterval_deg = plannerDefaults.AzimuthInterval_deg;
if isfield(options.PlannerOptions, "AzimuthInterval_deg") && ...
        ~isempty(options.PlannerOptions.AzimuthInterval_deg)
    azimuthInterval_deg = options.PlannerOptions.AzimuthInterval_deg;
end
validateattributes(azimuthInterval_deg, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2, 'increasing'});
azimuthInterval_deg = reshape(double(azimuthInterval_deg), 1, 2);
elevationInterval_deg = [-90 90];

endTime_s = requestedEndTime_s;
if targetMotion.ModelType == "sampledTrajectory"
    endTime_s = min(endTime_s, targetMotion.DomainTime_s(2));
    knotIsRelevant = targetMotion.time_s > startTime_s & ...
        targetMotion.time_s < endTime_s;
    inspectionTime_s = [startTime_s; ...
        targetMotion.time_s(knotIsRelevant); endTime_s];
else
    inspectionTime_s = [startTime_s; endTime_s];
end
inspectionTime_s = unique(inspectionTime_s);
inspectionPosition_deg = evaluateTargetMotion( ...
    targetMotion, inspectionTime_s);
positionIsInFrame = ...
    inspectionPosition_deg(:, 1) >= azimuthInterval_deg(1) & ...
    inspectionPosition_deg(:, 1) <= azimuthInterval_deg(2) & ...
    inspectionPosition_deg(:, 2) >= elevationInterval_deg(1) & ...
    inspectionPosition_deg(:, 2) <= elevationInterval_deg(2);

firstOutsideIndex = find(~positionIsInFrame, 1, "first");
targetLeavesFrame = ~isempty(firstOutsideIndex);
frameExitTime_s = Inf;
if ~targetLeavesFrame
    return;
end
if firstOutsideIndex == 1
    frameExitTime_s = inspectionTime_s(1);
    return;
end

% PCHIP and linear target segments remain within their endpoint ranges.
% Bisection therefore isolates the first inside-to-outside crossing without
% introducing an approximate target model or extrapolating the trajectory.
lowerTime_s = inspectionTime_s(firstOutsideIndex - 1);
upperTime_s = inspectionTime_s(firstOutsideIndex);
for iterationIndex = 1:60
    middleTime_s = 0.5 * (lowerTime_s + upperTime_s);
    middlePosition_deg = evaluateTargetMotion(targetMotion, middleTime_s);
    middleIsInFrame = ...
        middlePosition_deg(1) >= azimuthInterval_deg(1) && ...
        middlePosition_deg(1) <= azimuthInterval_deg(2) && ...
        middlePosition_deg(2) >= elevationInterval_deg(1) && ...
        middlePosition_deg(2) <= elevationInterval_deg(2);
    if middleIsInFrame
        lowerTime_s = middleTime_s;
    else
        upperTime_s = middleTime_s;
    end
end
frameExitTime_s = lowerTime_s;
end

function initialState = validateInterceptInitialState(initialState)
% PURPOSE
%   - Validate and normalize the state required by intercept planning.
if ~isstruct(initialState) || ~isscalar(initialState) || ...
        ~all(isfield(initialState, ["time_s" "position_deg"]))
    error("planAzElMovingTargetIntercept:InvalidInitialState", ...
        "initialState must contain time_s and position_deg.");
end
validateattributes(initialState.time_s, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
validateattributes(initialState.position_deg, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2});
initialState.time_s = double(initialState.time_s);
initialState.position_deg = reshape( ...
    double(initialState.position_deg), 1, 2);
end

function targetMotion = validateTargetMotion(targetMotion)
% PURPOSE
%   - Normalize sampled trajectories and the compatible linear model.
if ~isstruct(targetMotion) || ~isscalar(targetMotion)
    error("planAzElMovingTargetIntercept:InvalidTargetMotion", ...
        "targetMotion must be a scalar struct.");
end
hasSampledFields = all(isfield(targetMotion, ["time_s" "position_deg"]));
linearFields = ["referenceTime_s" ...
    "referencePosition_deg" "velocity_deg_s"];
hasLinearFields = all(isfield(targetMotion, linearFields));
if hasSampledFields && hasLinearFields
    error("planAzElMovingTargetIntercept:AmbiguousTargetMotion", ...
        "targetMotion must use either sampled time_s/position_deg or " + ...
        "constant-velocity reference fields, not both.");
end
if hasSampledFields
    validateattributes(targetMotion.time_s, {'numeric'}, ...
        {'real', 'finite', 'vector'});
    time_s = double(targetMotion.time_s(:));
    if numel(time_s) < 2 || any(diff(time_s) <= 0)
        error("planAzElMovingTargetIntercept:InvalidTargetTime", ...
            "targetMotion.time_s must contain at least two strictly " + ...
            "increasing finite samples.");
    end
    validateattributes(targetMotion.position_deg, {'numeric'}, ...
        {'real', 'finite', '2d', 'ncols', 2});
    position_deg = double(targetMotion.position_deg);
    if size(position_deg, 1) ~= numel(time_s)
        error("planAzElMovingTargetIntercept:TargetSizeMismatch", ...
            "targetMotion.position_deg must have one N-by-2 row for " + ...
            "each targetMotion.time_s sample.");
    end
    interpolationMethod = "pchip";
    if isfield(targetMotion, "InterpolationMethod") && ...
            ~isempty(targetMotion.InterpolationMethod)
        interpolationMethod = lower(string( ...
            targetMotion.InterpolationMethod));
    end
    if ~isscalar(interpolationMethod) || ~any( ...
            interpolationMethod == ["pchip" "linear"])
        error("planAzElMovingTargetIntercept:InvalidInterpolation", ...
            "targetMotion.InterpolationMethod must be pchip or linear.");
    end
    targetMotion = struct( ...
        "ModelType", "sampledTrajectory", ...
        "time_s", time_s, ...
        "position_deg", position_deg, ...
        "InterpolationMethod", interpolationMethod, ...
        "DomainTime_s", time_s([1 end]).');
    return;
end
if ~hasLinearFields
    error("planAzElMovingTargetIntercept:InvalidTargetMotion", ...
        "targetMotion must contain time_s and position_deg, or " + ...
        "referenceTime_s, referencePosition_deg, and velocity_deg_s.");
end
validateattributes(targetMotion.referenceTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
validateattributes(targetMotion.referencePosition_deg, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2});
validateattributes(targetMotion.velocity_deg_s, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2});
targetMotion = struct( ...
    "ModelType", "constantVelocity", ...
    "referenceTime_s", double(targetMotion.referenceTime_s), ...
    "referencePosition_deg", reshape( ...
        double(targetMotion.referencePosition_deg), 1, 2), ...
    "velocity_deg_s", reshape( ...
        double(targetMotion.velocity_deg_s), 1, 2), ...
    "InterpolationMethod", "linear", ...
    "DomainTime_s", [-Inf Inf]);
end

function [position_deg, velocity_deg_s] = evaluateTargetMotion( ...
        targetMotion, queryTime_s)
% PURPOSE
%   - Evaluate target position and instantaneous velocity without extrapolating.
validateattributes(queryTime_s, {'numeric'}, {'real', 'finite'});
queryTime_s = double(queryTime_s(:));
if targetMotion.ModelType == "constantVelocity"
    position_deg = targetMotion.referencePosition_deg + ...
        (queryTime_s - targetMotion.referenceTime_s) .* ...
        targetMotion.velocity_deg_s;
    velocity_deg_s = repmat( ...
        targetMotion.velocity_deg_s, numel(queryTime_s), 1);
    return;
end
domainTime_s = targetMotion.DomainTime_s;
timeTolerance_s = 64 * eps(max(1, max(abs(domainTime_s))));
if any(queryTime_s < domainTime_s(1) - timeTolerance_s) || ...
        any(queryTime_s > domainTime_s(2) + timeTolerance_s)
    error("planAzElMovingTargetIntercept:TargetTimeOutsideDomain", ...
        "Requested target time must lie in the sampled interval " + ...
        "[%.9g, %.9g] s.", domainTime_s(1), domainTime_s(2));
end
queryTime_s = min(max(queryTime_s, domainTime_s(1)), domainTime_s(2));
if targetMotion.InterpolationMethod == "linear"
    position_deg = interp1(targetMotion.time_s, ...
        targetMotion.position_deg, queryTime_s, "linear");
    intervalIndex = sum(queryTime_s >= ...
        targetMotion.time_s(2:end).', 2) + 1;
    intervalIndex = min(intervalIndex, numel(targetMotion.time_s) - 1);
    segmentDuration_s = diff(targetMotion.time_s);
    segmentVelocity_deg_s = diff(targetMotion.position_deg, 1, 1) ./ ...
        segmentDuration_s;
    velocity_deg_s = segmentVelocity_deg_s(intervalIndex, :);
    return;
end
position_deg = zeros(numel(queryTime_s), 2);
velocity_deg_s = zeros(numel(queryTime_s), 2);
for axisIndex = 1:2
    positionPolynomial = pchip(targetMotion.time_s, ...
        targetMotion.position_deg(:, axisIndex));
    velocityPolynomial = differentiatePiecewisePolynomial( ...
        positionPolynomial);
    position_deg(:, axisIndex) = ppval( ...
        positionPolynomial, queryTime_s);
    velocity_deg_s(:, axisIndex) = ppval( ...
        velocityPolynomial, queryTime_s);
end
end

function derivativePolynomial = differentiatePiecewisePolynomial( ...
        polynomial)
% PURPOSE
%   - Differentiate a scalar MATLAB piecewise polynomial without a toolbox.
polynomialOrder = polynomial.order;
if polynomialOrder <= 1
    derivativeCoefficients = zeros(polynomial.pieces, 1);
else
    powers = polynomialOrder - 1:-1:1;
    derivativeCoefficients = polynomial.coefs(:, 1:end - 1) .* powers;
end
derivativePolynomial = mkpp( ...
    polynomial.breaks, derivativeCoefficients, polynomial.dim);
end

function certifiable = targetHasCertifiedRecedingRay( ...
        initialState, targetMotion)
% PURPOSE
%   - Recognize the constant-velocity monotone geometry proved by bisection.
certifiable = false;
if targetMotion.ModelType ~= "constantVelocity"
    return;
end
[targetAtStart_deg, targetVelocity_deg_s] = evaluateTargetMotion( ...
    targetMotion, initialState.time_s);
lineOfSight_deg = targetAtStart_deg - initialState.position_deg;
lineOfSightLength_deg = norm(lineOfSight_deg);
if lineOfSightLength_deg <= 1e-10
    error("planAzElMovingTargetIntercept:InitialCoincidence", ...
        "The target is already at the initial position.");
end
targetSpeed_deg_s = norm(targetVelocity_deg_s);
crossProduct_deg2_s = ...
    lineOfSight_deg(1) * targetVelocity_deg_s(2) - ...
    lineOfSight_deg(2) * targetVelocity_deg_s(1);
collinearityScale = max(1, ...
    lineOfSightLength_deg * max(1, targetSpeed_deg_s));
isCollinear = abs(crossProduct_deg2_s) <= 1e-10 * collinearityScale;
isReceding = dot(lineOfSight_deg, targetVelocity_deg_s) >= -1e-10;
certifiable = isCollinear && isReceding;
end

function trackTime_s = targetTrackTimes( ...
        targetMotion, startTime_s, endTime_s, sampleCount)
% PURPOSE
%   - Retain both regular display samples and supplied trajectory knots.
trackTime_s = linspace(startTime_s, endTime_s, sampleCount).';
if targetMotion.ModelType == "sampledTrajectory"
    suppliedTime_s = targetMotion.time_s( ...
        targetMotion.time_s >= startTime_s & ...
        targetMotion.time_s <= endTime_s);
    trackTime_s = unique([trackTime_s; suppliedTime_s]);
end
end

function candidateTimes_s = recedingRaySearchTimes( ...
        startTime_s, endTime_s, initialStep_s)
% PURPOSE
%   - Create exponentially expanding candidates for a monotone residual.
candidateTimes_s = startTime_s;
elapsedTime_s = initialStep_s;
while candidateTimes_s(end) < endTime_s
    candidateTimes_s(end + 1, 1) = min( ...
        endTime_s, startTime_s + elapsedTime_s); %#ok<AGROW>
    elapsedTime_s = 2 * elapsedTime_s;
end
candidateTimes_s = unique(candidateTimes_s);
end

function candidateTimes_s = arbitraryTargetSearchTimes( ...
        targetMotion, startTime_s, endTime_s, maximumStep_s)
% PURPOSE
%   - Scan a general path at every supplied knot and a bounded time step.
regularTimes_s = (startTime_s:maximumStep_s:endTime_s).';
candidateTimes_s = unique([regularTimes_s; startTime_s; endTime_s]);
if targetMotion.ModelType == "sampledTrajectory"
    suppliedTime_s = targetMotion.time_s( ...
        targetMotion.time_s >= startTime_s & ...
        targetMotion.time_s <= endTime_s);
    candidateTimes_s = unique([candidateTimes_s; suppliedTime_s]);
end
end

function message = earliestSearchMessage(certifiedRecedingRay)
% PURPOSE
%   - Describe whether a first bracket has an analytic monotonicity proof.
if certifiedRecedingRay
    message = "Earliest feasible intercept was certified by a monotone " + ...
        "receding-ray bracket and bisection.";
else
    message = "First sampled feasibility transition on the arbitrary " + ...
        "target path was bracketed and bisected numerically.";
end
end

function method = conditionalSearchMethod(certifiedRecedingRay)
% PURPOSE
%   - Return a machine-readable earliest-search method identifier.
if certifiedRecedingRay
    method = "certifiedRecedingRayBisection";
else
    method = "sampledPathFirstTransitionBisection";
end
end

function validateEarliestInitialVelocity(initialState)
% PURPOSE
%   - Reject unsupported moving initial states before earliest search.
initialVelocity_deg_s = [0 0];
if isfield(initialState, "velocity_deg_s") && ...
        ~isempty(initialState.velocity_deg_s)
    validateattributes(initialState.velocity_deg_s, {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 2});
    initialVelocity_deg_s = reshape( ...
        double(initialState.velocity_deg_s), 1, 2);
end
if norm(initialVelocity_deg_s) > 1e-12
    error("planAzElMovingTargetIntercept:EarliestRequiresInitialRest", ...
        "Earliest intercept currently requires zero initial velocity " + ...
        "because final scheduling may use a short initial hold.");
end
end

function validateVelocityMatchGeometry( ...
        initialState, interceptPosition_deg, targetVelocity_deg_s, ...
        matchVelocity)
% PURPOSE
%   - Validate terminal-velocity compatibility with the direct route.
% The compatible velocity-matching mode uses the direct start-to-target
% direction. Obstacle-avoiding routes should request position-only capture.
if ~matchVelocity || norm(targetVelocity_deg_s) <= 1e-12
    return;
end
routeVector_deg = interceptPosition_deg - initialState.position_deg;
routeLength_deg = norm(routeVector_deg);
if routeLength_deg <= 1e-12
    error("planAzElMovingTargetIntercept:VelocityMatchNeedsRoute", ...
        "A zero-length intercept cannot match nonzero target velocity.");
end
crossProduct_deg2_s = ...
    routeVector_deg(1) * targetVelocity_deg_s(2) - ...
    routeVector_deg(2) * targetVelocity_deg_s(1);
alignmentScale = max(1, ...
    routeLength_deg * max(1, norm(targetVelocity_deg_s)));
isTangent = abs(crossProduct_deg2_s) <= 1e-10 * alignmentScale;
isForward = dot(routeVector_deg, targetVelocity_deg_s) >= -1e-12;
if ~isTangent || ~isForward
    error("planAzElMovingTargetIntercept:TargetVelocityNotTangent", ...
        "Target velocity must be forward-tangent to the direct route " + ...
        "when MatchTargetVelocity=true. Set MatchTargetVelocity=false " + ...
        "for a position-only intercept.");
end
end

function [interceptTime_s, diagnostics] = searchEarliestInterceptTime( ...
        obstacles, initialState, targetMotion, limits, options)
% PURPOSE
%   - Find the first numerically feasible intercept on a target track.
% Arbitrary paths are scanned chronologically. Exponential bracketing is
% retained only for the receding-ray case where monotonicity is proved.
searchStartTime_s = initialState.time_s;
searchEndTime_s = initialState.time_s + options.MaximumSearchDuration_s;
if targetMotion.ModelType == "sampledTrajectory"
    searchStartTime_s = max(searchStartTime_s, ...
        targetMotion.DomainTime_s(1));
    searchEndTime_s = min(searchEndTime_s, ...
        targetMotion.DomainTime_s(2));
end
if searchEndTime_s <= searchStartTime_s
    error("planAzElMovingTargetIntercept:EmptyTargetSearchDomain", ...
        "The target trajectory does not overlap the requested search horizon.");
end
certifiedRecedingRay = isempty(obstacles) && ...
    targetHasCertifiedRecedingRay(initialState, targetMotion);
if certifiedRecedingRay
    candidateTimes_s = recedingRaySearchTimes( ...
        searchStartTime_s, searchEndTime_s, options.InitialSearchStep_s);
else
    candidateTimes_s = arbitraryTargetSearchTimes( ...
        targetMotion, searchStartTime_s, searchEndTime_s, ...
        options.InitialSearchStep_s);
end
targetCandidateCount = numel(candidateTimes_s);
candidateGoalTimeMode = candidatePlannerGoalTimeMode( ...
    obstacles, initialState, options);

prefilter = emptyKinematicPrefilterDiagnostics(candidateTimes_s);
if ~isempty(obstacles)
    prefilter = kinematicInterceptPrefilter(candidateTimes_s, obstacles, ...
        initialState, targetMotion, limits);
    candidateTimes_s = prefilter.FullValidationTimes_s;
end

if prefilter.HasKnownLowerBound
    lowerTime_s = prefilter.LowerInfeasibleTime_s;
    lowerResidual_s = prefilter.LowerResidual_s;
    lowerArrivalTime_s = prefilter.LowerMinimumArrivalTime_s;
    firstCandidateIndex = 1;
    bracketEvaluationCount = 0;
else
    lowerTime_s = candidateTimes_s(1);
    [lowerResidual_s, lowerArrivalTime_s, lowerStatus] = ...
        interceptResidual( ...
        lowerTime_s, obstacles, initialState, ...
        targetMotion, limits, options);
    firstCandidateIndex = 2;
    bracketEvaluationCount = 1;
end
numericallyUnknownEvaluationCount = 0;
if exist("lowerStatus", "var") && lowerStatus == "numericallyUnknown"
    numericallyUnknownEvaluationCount = 1;
end
upperTime_s = lowerTime_s;
upperResidual_s = lowerResidual_s;
upperArrivalTime_s = lowerArrivalTime_s;
for candidateIndex = firstCandidateIndex:numel(candidateTimes_s)
    if upperResidual_s <= 0
        break;
    end
    upperTime_s = candidateTimes_s(candidateIndex);
    [upperResidual_s, upperArrivalTime_s, upperStatus] = ...
        interceptResidual( ...
        upperTime_s, obstacles, initialState, ...
        targetMotion, limits, options);
    bracketEvaluationCount = bracketEvaluationCount + 1;
    numericallyUnknownEvaluationCount = ...
        numericallyUnknownEvaluationCount + ...
        double(upperStatus == "numericallyUnknown");
    if upperResidual_s > 0 && upperStatus ~= "numericallyUnknown"
        lowerTime_s = upperTime_s;
        lowerResidual_s = upperResidual_s;
        lowerArrivalTime_s = upperArrivalTime_s;
    end
end
if upperResidual_s > 0
    interceptTime_s = searchEndTime_s;
    diagnostics = emptyInterceptSearchDiagnostics();
    diagnostics.Success = false;
    diagnostics.Message = "No feasible intercept was found " + ...
        "before the target search ended.";
    diagnostics.TerminationReason = "noInterceptInHorizon";
    if numericallyUnknownEvaluationCount > 0
        diagnostics.Message = "The intercept search was " + ...
            "inconclusive because one or more planner evaluations had a " + ...
            "numerical or time-limit failure.";
        diagnostics.TerminationReason = "interceptSearchInconclusive";
    end
    diagnostics.LowerInfeasibleTime_s = lowerTime_s;
    diagnostics.LowerResidual_s = lowerResidual_s;
    diagnostics.LowerMinimumArrivalTime_s = lowerArrivalTime_s;
    diagnostics.BracketEvaluationCount = bracketEvaluationCount;
    diagnostics.KinematicPrefilterApplied = prefilter.Applied;
    diagnostics.KinematicPrefilterEvaluationCount = ...
        prefilter.EvaluationCount;
    diagnostics.KinematicPrunedCandidateCount = ...
        prefilter.PrunedCandidateCount;
    diagnostics.InterpolatedCandidateTime_s = ...
        prefilter.InterpolatedCandidateTime_s;
    diagnostics.TargetCandidateCount = targetCandidateCount;
    diagnostics.FullValidationCandidateCount = numel(candidateTimes_s);
    diagnostics.FullPlannerEvaluationCount = plannerEvaluationCount( ...
        bracketEvaluationCount, candidateGoalTimeMode);
    diagnostics.NumericallyUnknownEvaluationCount = ...
        numericallyUnknownEvaluationCount;
    diagnostics.CandidatePlannerGoalTimeMode = candidateGoalTimeMode;
    diagnostics.EarliestCertified = false;
    diagnostics.EarliestNumericallyResolved = false;
    diagnostics.SearchMethod = conditionalSearchMethod(certifiedRecedingRay);
    if prefilter.Applied
        diagnostics.SearchMethod = ...
            "interpolatedKinematicPrefilterAndValidatedScan";
    end
    diagnostics.SearchStartTime_s = searchStartTime_s;
    diagnostics.SearchEndTime_s = searchEndTime_s;
    diagnostics.MaximumScanStep_s = options.InitialSearchStep_s;
    return;
end

iterationCount = 0;
while upperTime_s - lowerTime_s > ...
        options.SearchTimeTolerance_s && ...
        iterationCount < options.MaximumSearchIterations
    iterationCount = iterationCount + 1;
    middleTime_s = 0.5 * (lowerTime_s + upperTime_s);
    [middleResidual_s, middleArrivalTime_s, middleStatus] = ...
        interceptResidual( ...
        middleTime_s, obstacles, initialState, ...
        targetMotion, limits, options);
    numericallyUnknownEvaluationCount = ...
        numericallyUnknownEvaluationCount + ...
        double(middleStatus == "numericallyUnknown");
    if middleResidual_s <= 0
        upperTime_s = middleTime_s;
        upperResidual_s = middleResidual_s;
        upperArrivalTime_s = middleArrivalTime_s;
    else
        lowerTime_s = middleTime_s;
        lowerResidual_s = middleResidual_s;
        lowerArrivalTime_s = middleArrivalTime_s;
    end
end
fixedPlannerEvaluationCount = 0;
fixedBoundaryFound = true;
if candidateGoalTimeMode == "analyticresttorest"
    [lowerTime_s, lowerResidual_s, lowerArrivalTime_s, ...
        upperTime_s, upperResidual_s, upperArrivalTime_s, ...
        fixedScanCount, fixedBisectionCount, fixedUnknownCount, ...
        fixedBoundaryFound] = resolveAnalyticHs3Boundary( ...
        lowerTime_s, upperTime_s, searchEndTime_s, obstacles, ...
        initialState, targetMotion, limits, options);
    fixedPlannerEvaluationCount = ...
        fixedScanCount + fixedBisectionCount;
    bracketEvaluationCount = bracketEvaluationCount + fixedScanCount;
    iterationCount = iterationCount + fixedBisectionCount;
    numericallyUnknownEvaluationCount = ...
        numericallyUnknownEvaluationCount + fixedUnknownCount;
    candidateGoalTimeMode = ...
        "analyticresttorestthenfixedarrival";
end
interceptTime_s = upperTime_s;
if candidateGoalTimeMode == ...
        "analyticresttorestthenfixedarrival"
    fullPlannerEvaluationCount = fixedPlannerEvaluationCount;
else
    fullPlannerEvaluationCount = plannerEvaluationCount( ...
        bracketEvaluationCount + iterationCount, candidateGoalTimeMode);
end
searchMethod = conditionalSearchMethod(certifiedRecedingRay);
message = earliestSearchMessage(certifiedRecedingRay);
if prefilter.Applied
    searchMethod = "interpolatedKinematicPrefilterAndValidatedScan";
    message = message + " The interpolated target prefilter discarded " + ...
        string(prefilter.PrunedCandidateCount) + ...
        " dynamically impossible candidate times before full obstacle checks.";
end
terminationReason = "interceptFound";
if ~fixedBoundaryFound
    terminationReason = "noInterceptInHorizon";
    message = "No HS-3 fixed-arrival intercept was found before the " + ...
        "target search ended.";
elseif numericallyUnknownEvaluationCount > 0
    terminationReason = "interceptSearchInconclusive";
    message = "A feasible planner sample was found, but the " + ...
        "earliest intercept was not resolved because an earlier planner " + ...
        "evaluation had a numerical or time-limit failure.";
end
diagnostics = struct( ...
    "Success", numericallyUnknownEvaluationCount == 0 && ...
    lowerResidual_s > 0 && upperResidual_s <= 0 && ...
    upperTime_s - lowerTime_s <= options.SearchTimeTolerance_s, ...
    "Message", message, ...
    "TerminationReason", terminationReason, ...
    "LowerInfeasibleTime_s", lowerTime_s, ...
    "UpperFeasibleTime_s", upperTime_s, ...
    "LowerResidual_s", lowerResidual_s, ...
    "UpperResidual_s", upperResidual_s, ...
    "LowerMinimumArrivalTime_s", lowerArrivalTime_s, ...
    "UpperMinimumArrivalTime_s", upperArrivalTime_s, ...
    "BracketEvaluationCount", bracketEvaluationCount, ...
    "BisectionIterationCount", iterationCount, ...
    "KinematicPrefilterApplied", prefilter.Applied, ...
    "KinematicPrefilterEvaluationCount", prefilter.EvaluationCount, ...
    "KinematicPrunedCandidateCount", prefilter.PrunedCandidateCount, ...
    "InterpolatedCandidateTime_s", prefilter.InterpolatedCandidateTime_s, ...
    "TargetCandidateCount", targetCandidateCount, ...
    "FullValidationCandidateCount", numel(candidateTimes_s), ...
    "FullPlannerEvaluationCount", fullPlannerEvaluationCount, ...
    "NumericallyUnknownEvaluationCount", ...
    numericallyUnknownEvaluationCount, ...
    "CandidatePlannerGoalTimeMode", candidateGoalTimeMode, ...
    "EarliestCertified", certifiedRecedingRay && ...
    numericallyUnknownEvaluationCount == 0 && ...
    lowerResidual_s > 0 && ...
    upperResidual_s <= 0 && upperTime_s - lowerTime_s <= ...
    options.SearchTimeTolerance_s, ...
    "EarliestNumericallyResolved", ...
    numericallyUnknownEvaluationCount == 0 && lowerResidual_s > 0 && ...
    upperResidual_s <= 0 && upperTime_s - lowerTime_s <= ...
    options.SearchTimeTolerance_s, ...
    "SearchMethod", searchMethod, ...
    "SearchStartTime_s", searchStartTime_s, ...
    "SearchEndTime_s", searchEndTime_s, ...
    "MaximumScanStep_s", options.InitialSearchStep_s);
end

function diagnostics = kinematicInterceptPrefilter(candidateTimes_s, ...
        obstacles, initialState, targetMotion, limits)
% PURPOSE
%   - Discard target times forbidden by velocity or target-point occupancy.
%   - Interpolate the first safe lower-bound zero crossing for validation.
diagnostics = emptyKinematicPrefilterDiagnostics(candidateTimes_s);
diagnostics.Applied = true;
residual_s = inf(size(candidateTimes_s));
arrivalTime_s = inf(size(candidateTimes_s));
firstPotentialIndex = 0;
obstacleField = buildAzElTimeObstacleField(obstacles);

maximumVelocity_deg_s = limits.maxVelocity_deg_s;
validateattributes(maximumVelocity_deg_s, {'numeric'}, ...
    {'real', 'positive', 'nonempty'});
maximumVelocity_deg_s = reshape(double(maximumVelocity_deg_s), 1, []);
if isscalar(maximumVelocity_deg_s)
    maximumVelocity_deg_s = repmat(maximumVelocity_deg_s, 1, 2);
elseif numel(maximumVelocity_deg_s) ~= 2
    error("planAzElMovingTargetIntercept:InvalidVelocityLimit", ...
        "limits.maxVelocity_deg_s must be scalar or two-element.");
end

for candidateIndex = 1:numel(candidateTimes_s)
    targetPosition_deg = evaluateTargetMotion( ...
        targetMotion, candidateTimes_s(candidateIndex));
    displacement_deg = abs( ...
        targetPosition_deg - initialState.position_deg);
    minimumDuration_s = max( ...
        displacement_deg ./ maximumVelocity_deg_s);
    arrivalTime_s(candidateIndex) = ...
        initialState.time_s + minimumDuration_s;
    residual_s(candidateIndex) = arrivalTime_s(candidateIndex) - ...
        candidateTimes_s(candidateIndex);
    targetIsOccupied = queryAzElTimeObstacle( ...
        obstacleField, targetPosition_deg(1), targetPosition_deg(2), ...
        candidateTimes_s(candidateIndex));
    if targetIsOccupied
        residual_s(candidateIndex) = Inf;
        arrivalTime_s(candidateIndex) = Inf;
    end
    diagnostics.EvaluationCount = candidateIndex;
    if residual_s(candidateIndex) <= 0
        firstPotentialIndex = candidateIndex;
        break;
    end
end

if firstPotentialIndex == 0
    diagnostics.HasKnownLowerBound = true;
    diagnostics.LowerInfeasibleTime_s = candidateTimes_s(end);
    diagnostics.LowerResidual_s = residual_s(end);
    diagnostics.LowerMinimumArrivalTime_s = arrivalTime_s(end);
    diagnostics.PrunedCandidateCount = numel(candidateTimes_s);
    diagnostics.FullValidationTimes_s = candidateTimes_s(end);
    return;
end
if firstPotentialIndex == 1
    diagnostics.FullValidationTimes_s = candidateTimes_s;
    return;
end

lowerIndex = firstPotentialIndex - 1;
lowerTime_s = candidateTimes_s(lowerIndex);
upperTime_s = candidateTimes_s(firstPotentialIndex);
lowerResidual_s = residual_s(lowerIndex);
upperResidual_s = residual_s(firstPotentialIndex);
interpolatedTime_s = upperTime_s;
if isfinite(lowerResidual_s) && isfinite(upperResidual_s) && ...
        lowerResidual_s > 0 && upperResidual_s <= 0
    interpolationFraction = lowerResidual_s / ...
        (lowerResidual_s - upperResidual_s);
    interpolatedTime_s = lowerTime_s + interpolationFraction * ...
        (upperTime_s - lowerTime_s);
end

diagnostics.HasKnownLowerBound = true;
diagnostics.LowerInfeasibleTime_s = lowerTime_s;
diagnostics.LowerResidual_s = lowerResidual_s;
diagnostics.LowerMinimumArrivalTime_s = arrivalTime_s(lowerIndex);
diagnostics.PrunedCandidateCount = lowerIndex;
diagnostics.InterpolatedCandidateTime_s = interpolatedTime_s;
diagnostics.FullValidationTimes_s = unique([ ...
    interpolatedTime_s; candidateTimes_s(firstPotentialIndex:end)]);
end

function diagnostics = emptyKinematicPrefilterDiagnostics(candidateTimes_s)
% PURPOSE
%   - Return the stable internal target-prefilter diagnostics record.
diagnostics = struct( ...
    "Applied", false, ...
    "EvaluationCount", 0, ...
    "PrunedCandidateCount", 0, ...
    "HasKnownLowerBound", false, ...
    "LowerInfeasibleTime_s", NaN, ...
    "LowerResidual_s", NaN, ...
    "LowerMinimumArrivalTime_s", NaN, ...
    "InterpolatedCandidateTime_s", NaN, ...
    "FullValidationTimes_s", candidateTimes_s(:));
end

function [residual_s, arrivalTime_s, evaluationStatus] = ...
        interceptResidual( ...
        candidateTime_s, obstacles, initialState, ...
        targetMotion, limits, options)
% PURPOSE
%   - Test one target time with an exact fixed-arrival planner request.
evaluationStatus = "numericallyUnknown";
goalVelocity_deg_s = [0 0];
[targetPosition_deg, targetVelocity_deg_s] = evaluateTargetMotion( ...
    targetMotion, candidateTime_s);
if candidateTime_s <= initialState.time_s
    residual_s = Inf;
    arrivalTime_s = Inf;
    evaluationStatus = "kinematicallyInfeasible";
    return;
end
candidateGoalTimeMode = candidatePlannerGoalTimeMode( ...
    obstacles, initialState, options);
if candidateGoalTimeMode == "analyticresttorest"
    minimumDuration_s = minimumRestToRestDuration( ...
        targetPosition_deg - initialState.position_deg, limits);
    arrivalTime_s = initialState.time_s + minimumDuration_s;
    residual_s = arrivalTime_s - candidateTime_s;
    if residual_s <= 0
        evaluationStatus = "feasible";
    else
        evaluationStatus = "kinematicallyInfeasible";
    end
    return;
end
if options.MatchTargetVelocity
    try
        validateVelocityMatchGeometry(initialState, targetPosition_deg, ...
            targetVelocity_deg_s, true);
    catch exception
        if exception.identifier == ...
                "planAzElMovingTargetIntercept:TargetVelocityNotTangent" || ...
                exception.identifier == ...
                "planAzElMovingTargetIntercept:VelocityMatchNeedsRoute"
            residual_s = Inf;
            arrivalTime_s = Inf;
            evaluationStatus = "kinematicallyInfeasible";
            return;
        end
        rethrow(exception);
    end
    goalVelocity_deg_s = targetVelocity_deg_s;
end
plannerGoalTime_s = candidateTime_s;
if candidateGoalTimeMode == "earliestarrival"
    plannerGoalTime_s = initialState.time_s + ...
        options.MaximumSearchDuration_s;
end
goalState = struct( ...
    "time_s", plannerGoalTime_s, ...
    "position_deg", targetPosition_deg, ...
    "velocity_deg_s", goalVelocity_deg_s, ...
    "acceleration_deg_s2", [0 0]);
plannerOptions = options.PlannerOptions;
plannerOptions.GoalTimeMode = candidateGoalTimeMode;
plannerOptions.Verbose = false;
trialResult = planAzElMotion( ...
    obstacles, initialState, goalState, limits, plannerOptions);
if ~trialResult.Success
    residual_s = Inf;
    arrivalTime_s = Inf;
    provenInfeasibleReason = any(string(trialResult.TerminationReason) == [ ...
        "endpointBlocked", "goalTimeInfeasible"]);
    if provenInfeasibleReason
        evaluationStatus = "provenInfeasible";
    end
    return;
end
if plannerOptions.GoalTimeMode == "earliestarrival"
    arrivalTime_s = trialResult.goalLineInterceptTime_s;
    residual_s = arrivalTime_s - candidateTime_s;
else
    arrivalTime_s = candidateTime_s;
    residual_s = -max(options.SearchTimeTolerance_s, ...
        eps(candidateTime_s));
end
evaluationStatus = "feasible";
end

function goalTimeMode = candidatePlannerGoalTimeMode( ...
        obstacles, initialState, options)
% PURPOSE
%   - Select the safe residual method for one intercept search.
canUseAnalyticResidual = isempty(obstacles) && ...
    ~options.MatchTargetVelocity && ...
    norm(initialState.velocity_deg_s, Inf) <= 1e-12 && ...
    norm(initialState.acceleration_deg_s2, Inf) <= 1e-12;
if canUseAnalyticResidual
    goalTimeMode = "analyticresttorest";
elseif isempty(obstacles)
    goalTimeMode = "earliestarrival";
else
    goalTimeMode = "fixedarrival";
end
end

function count = plannerEvaluationCount(evaluationCount, evaluationMode)
% PURPOSE
%   - Report zero planner calls when the analytic residual is used.
if evaluationMode == "analyticresttorest"
    count = 0;
else
    count = evaluationCount;
end
end

function [lowerTime_s, lowerResidual_s, lowerArrivalTime_s, ...
        upperTime_s, upperResidual_s, upperArrivalTime_s, ...
        scanCount, bisectionCount, unknownCount, boundaryFound] = ...
        resolveAnalyticHs3Boundary( ...
        analyticLowerTime_s, analyticUpperTime_s, searchEndTime_s, ...
        obstacles, initialState, targetMotion, limits, options)
% PURPOSE
%   - Refine an analytic lower bracket with fixed-arrival HS-3 feasibility.
lowerTime_s = analyticLowerTime_s;
lowerResidual_s = max(options.SearchTimeTolerance_s, eps);
lowerArrivalTime_s = Inf;
upperTime_s = searchEndTime_s;
upperResidual_s = Inf;
upperArrivalTime_s = Inf;
scanCount = 0;
bisectionCount = 0;
unknownCount = 0;
boundaryFound = false;
trialTime_s = analyticUpperTime_s;
while trialTime_s <= searchEndTime_s + eps(searchEndTime_s)
    [trialResidual_s, trialArrivalTime_s, trialStatus] = ...
        fixedArrivalHs3Residual(trialTime_s, obstacles, initialState, ...
        targetMotion, limits, options);
    scanCount = scanCount + 1;
    if trialStatus == "feasible"
        upperTime_s = trialTime_s;
        upperResidual_s = trialResidual_s;
        upperArrivalTime_s = trialArrivalTime_s;
        boundaryFound = true;
        break;
    elseif trialStatus == "provenTranscriptionInfeasible"
        lowerTime_s = trialTime_s;
        lowerResidual_s = trialResidual_s;
        lowerArrivalTime_s = trialArrivalTime_s;
    else
        unknownCount = unknownCount + 1;
    end
    nextTime_s = min(searchEndTime_s, ...
        trialTime_s + options.InitialSearchStep_s);
    if nextTime_s <= trialTime_s
        break;
    end
    trialTime_s = nextTime_s;
end
if ~boundaryFound
    return;
end

while upperTime_s - lowerTime_s > options.SearchTimeTolerance_s && ...
        bisectionCount < options.MaximumSearchIterations
    middleTime_s = 0.5 * (lowerTime_s + upperTime_s);
    [middleResidual_s, middleArrivalTime_s, middleStatus] = ...
        fixedArrivalHs3Residual(middleTime_s, obstacles, initialState, ...
        targetMotion, limits, options);
    bisectionCount = bisectionCount + 1;
    if middleStatus == "feasible"
        upperTime_s = middleTime_s;
        upperResidual_s = middleResidual_s;
        upperArrivalTime_s = middleArrivalTime_s;
    elseif middleStatus == "provenTranscriptionInfeasible"
        lowerTime_s = middleTime_s;
        lowerResidual_s = middleResidual_s;
        lowerArrivalTime_s = middleArrivalTime_s;
    else
        unknownCount = unknownCount + 1;
        break;
    end
end
end

function [residual_s, arrivalTime_s, status] = ...
        fixedArrivalHs3Residual( ...
        candidateTime_s, obstacles, initialState, targetMotion, ...
        limits, options)
% PURPOSE
%   - Test one analytic bracket time in the bounded HS-3 transcription.
targetPosition_deg = evaluateTargetMotion(targetMotion, candidateTime_s);
goalState = struct( ...
    "time_s", candidateTime_s, ...
    "position_deg", targetPosition_deg, ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);
plannerOptions = options.PlannerOptions;
plannerOptions.GoalTimeMode = "fixedarrival";
plannerOptions.Verbose = false;
plannerOptions = requireMeshRefinementPasses(plannerOptions, 2);
trialResult = planAzElMotion( ...
    obstacles, initialState, goalState, limits, plannerOptions);
if trialResult.Success
    residual_s = -max(options.SearchTimeTolerance_s, ...
        eps(candidateTime_s));
    arrivalTime_s = candidateTime_s;
    status = "feasible";
    return;
end
residual_s = Inf;
arrivalTime_s = Inf;
terminationReason = string(trialResult.TerminationReason);
provedLinearSeedInfeasible = ...
    terminationReason == "initialGuessInfeasible" && ...
    contains(string(trialResult.Message), "Exit flag: -2");
provenReason = any(terminationReason == [ ...
    "goalTimeInfeasible", "endpointBlocked"]) || ...
    provedLinearSeedInfeasible;
if provenReason
    status = "provenTranscriptionInfeasible";
else
    status = "numericallyUnknown";
end
end

function plannerOptions = requireMeshRefinementPasses( ...
        plannerOptions, minimumPassCount)
% PURPOSE
%   - Apply an internal minimum without requiring a complete override.
plannerDefaults = planAzElMotion();
configuredPassCount = plannerDefaults.MaximumMeshRefinementPasses;
if isfield(plannerOptions, "MaximumMeshRefinementPasses") && ...
        ~isempty(plannerOptions.MaximumMeshRefinementPasses)
    configuredPassCount = ...
        plannerOptions.MaximumMeshRefinementPasses;
end
plannerOptions.MaximumMeshRefinementPasses = max( ...
    minimumPassCount, configuredPassCount);
end

function duration_s = minimumRestToRestDuration( ...
        displacement_deg, limits)
% PURPOSE
%   - Compute the exact independent-axis S-curve duration from rest to rest.
velocityLimit = expandTwoAxisLimit(limits.maxVelocity_deg_s);
accelerationLimit = expandTwoAxisLimit(limits.maxAcceleration_deg_s2);
jerkLimit = expandTwoAxisLimit(limits.maxJerk_deg_s3);
axisDuration_s = zeros(1, 2);
for axisIndex = 1:2
    distance_deg = abs(displacement_deg(axisIndex));
    if distance_deg <= eps
        continue;
    end
    maximumVelocity = velocityLimit(axisIndex);
    maximumAcceleration = accelerationLimit(axisIndex);
    maximumJerk = jerkLimit(axisIndex);
    triangularJerkTime_s = (distance_deg / ...
        (2 * maximumJerk)) ^ (1 / 3);
    triangularAcceleration = maximumJerk * triangularJerkTime_s;
    triangularVelocity = maximumJerk * triangularJerkTime_s ^ 2;
    if triangularAcceleration <= maximumAcceleration && ...
            triangularVelocity <= maximumVelocity
        axisDuration_s(axisIndex) = 4 * triangularJerkTime_s;
        continue;
    end

    jerkTime_s = maximumAcceleration / maximumJerk;
    accelerationPlateau_s = 0.5 * (sqrt( ...
        jerkTime_s ^ 2 + 4 * distance_deg / maximumAcceleration) - ...
        3 * jerkTime_s);
    peakVelocity = maximumAcceleration * ...
        (jerkTime_s + accelerationPlateau_s);
    if accelerationPlateau_s >= 0 && peakVelocity <= maximumVelocity
        axisDuration_s(axisIndex) = ...
            4 * jerkTime_s + 2 * accelerationPlateau_s;
        continue;
    end

    if maximumVelocity <= maximumAcceleration ^ 2 / maximumJerk
        jerkTime_s = sqrt(maximumVelocity / maximumJerk);
        accelerationPlateau_s = 0;
        rampDistance_deg = 2 * maximumJerk * jerkTime_s ^ 3;
    else
        jerkTime_s = maximumAcceleration / maximumJerk;
        accelerationPlateau_s = maximumVelocity / ...
            maximumAcceleration - jerkTime_s;
        rampDistance_deg = maximumAcceleration * ( ...
            2 * jerkTime_s ^ 2 + ...
            3 * jerkTime_s * accelerationPlateau_s + ...
            accelerationPlateau_s ^ 2);
    end
    cruiseTime_s = max(0, ...
        (distance_deg - rampDistance_deg) / maximumVelocity);
    axisDuration_s(axisIndex) = 4 * jerkTime_s + ...
        2 * accelerationPlateau_s + cruiseTime_s;
end
duration_s = max(axisDuration_s);
end

function limit = expandTwoAxisLimit(limit)
% PURPOSE
%   - Expand one scalar limit to the two planner axes.
limit = reshape(double(limit), 1, []);
if isscalar(limit)
    limit = repmat(limit, 1, 2);
end
end

function diagnostics = emptyInterceptSearchDiagnostics()
% PURPOSE
%   - Define the stable intercept-search diagnostics schema.
diagnostics = struct( ...
    "Success", true, ...
    "Message", "", ...
    "TerminationReason", "notRun", ...
    "LowerInfeasibleTime_s", NaN, ...
    "UpperFeasibleTime_s", NaN, ...
    "LowerResidual_s", NaN, ...
    "UpperResidual_s", NaN, ...
    "LowerMinimumArrivalTime_s", NaN, ...
    "UpperMinimumArrivalTime_s", NaN, ...
    "BracketEvaluationCount", 0, ...
    "BisectionIterationCount", 0, ...
    "KinematicPrefilterApplied", false, ...
    "KinematicPrefilterEvaluationCount", 0, ...
    "KinematicPrunedCandidateCount", 0, ...
    "InterpolatedCandidateTime_s", NaN, ...
    "TargetCandidateCount", 0, ...
    "FullValidationCandidateCount", 0, ...
    "FullPlannerEvaluationCount", 0, ...
    "NumericallyUnknownEvaluationCount", 0, ...
    "CandidatePlannerGoalTimeMode", "notRun", ...
    "EarliestCertified", true, ...
    "EarliestNumericallyResolved", true, ...
    "SearchMethod", "specifiedTime", ...
    "SearchStartTime_s", NaN, ...
    "SearchEndTime_s", NaN, ...
    "MaximumScanStep_s", NaN, ...
    "ElapsedPlanningTime_s", 0);
end
