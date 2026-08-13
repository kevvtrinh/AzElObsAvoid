function result = planAzElMovingTargetIntercept( ...
        initialState, targetMotion, limits, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = planAzElMovingTargetIntercept()
%   result = planAzElMovingTargetIntercept( ...
%       initialState, targetMotion, limits, options)
%**************************************************************************
% PURPOSE
%   - Resolve a sampled or constant-velocity target into the earliest
%     feasible or a specified intercept state, then use planAzElMotion.
%   - Support obstacle-free azimuth/elevation intercept demonstrations
%     without introducing a second retiming implementation.
%**************************************************************************
% INPUTS
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
%   - options (scalar struct, optional)
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
%     bisects the first detected feasibility transition. EarliestCertified
%     is true only for the legacy receding-ray model; arbitrary-path results
%     report EarliestNumericallyResolved instead. A stationary initial
%     state is required because fixed-arrival slack may use an initial hold.
%     Velocity matching requires the instantaneous target velocity to be
%     tangent to the direct obstacle-free route; use false for a
%     position-only intercept of a general path.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Normal planner result augmented with target, search, intercept,
%       validation, and target-track plot diagnostics. TargetMotion echoes
%       the normalized model; TargetVelocityAtIntercept_deg_s is evaluated
%       from its selected interpolation. Expected planner infeasibility is
%       returned by planAzElMotion; invalid target contracts throw.
%**************************************************************************
% UNITS
%   - Angles are degrees; time is seconds; velocity is deg/s.

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
    return;
end
if nargin < 4 || isempty(optionOverrides)
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

%% Section 1: Resolve The Concrete Intercept State
interceptMode = lower(string(options.InterceptMode));
searchDiagnostics = emptyInterceptSearchDiagnostics();
if interceptMode == "specifiedtime"
    interceptTime_s = options.SpecifiedInterceptTime_s;
    validateattributes(interceptTime_s, {'numeric'}, ...
        {'real', 'finite', 'scalar', '>', initialState.time_s});
    searchDiagnostics.Message = ...
        "The caller supplied the intercept time; no search was required.";
else
    validateEarliestInitialVelocity(initialState);
    targetAtInitial_deg = evaluateTargetMotion( ...
        targetMotion, initialState.time_s);
    if norm(targetAtInitial_deg - initialState.position_deg) <= 1e-10
        error("planAzElMovingTargetIntercept:InitialCoincidence", ...
            "The target is already at initialState.position_deg.");
    end
    [interceptTime_s, searchDiagnostics] = ...
        searchEarliestInterceptTime(initialState, targetMotion, ...
        limits, options);
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

%% Section 2: Run The Maintained Obstacle-Free Planner
plannerOptions = options.PlannerOptions;
plannerDefaultOptions = planAzElMotion();
animationRequested = logical(plannerDisplayOption( ...
    plannerOptions, plannerDefaultOptions, "ShowAnimation"));
figureVisible = plannerDisplayOption( ...
    plannerOptions, plannerDefaultOptions, "FigureVisible");
animationFrameStride = plannerDisplayOption( ...
    plannerOptions, plannerDefaultOptions, "AnimationFrameStride");
animationPause_s = plannerDisplayOption( ...
    plannerOptions, plannerDefaultOptions, "AnimationPause_s");
showSweptSurfaces = plannerDisplayOption( ...
    plannerOptions, plannerDefaultOptions, "ShowSweptSurfaces");
maximumDisplayedSlices = plannerDisplayOption( ...
    plannerOptions, plannerDefaultOptions, ...
    "MaximumDisplayedSlicesPerObstacle");
plannerTitle = string(plannerDisplayOption( ...
    plannerOptions, plannerDefaultOptions, "Title"));
plannerOptions.GoalTimeMode = "fixedArrival";
% The wrapper supplies the final animation itself so the target and the
% pursuer advance on the same timeline.
plannerOptions.ShowAnimation = false;
result = planAzElMotion([], initialState, goalState, limits, ...
    plannerOptions);
plannerSucceeded = result.Success;

trackTime_s = targetTrackTimes(targetMotion, initialState.time_s, ...
    interceptTime_s, options.TargetTrackSampleCount);
if plannerSucceeded
    trackTime_s = unique([trackTime_s; result.timedSlopePath.time_s]);
end
trackPosition_deg = evaluateTargetMotion(targetMotion, trackTime_s);
if plannerSucceeded && animationRequested
    if lower(string(figureVisible)) == "off"
        animationPause_s = 0;
    end
    result.animation = animateAzElTimedSlopePath( ...
        result.timedSlopePath, result.obstacleField, struct( ...
        "FigureVisible", figureVisible, ...
        "FrameStride", animationFrameStride, ...
        "Pause_s", animationPause_s, ...
        "ShowSweptSurfaces", showSweptSurfaces, ...
        "MaximumDisplayedSlicesPerObstacle", ...
        maximumDisplayedSlices, ...
        "TargetTime_s", trackTime_s, ...
        "TargetPosition_deg", trackPosition_deg, ...
        "TargetLabel", "Moving target", ...
        "Title", plannerTitle + " animation"));
end
result.Options.ShowAnimation = animationRequested;

%% Section 3: Validate The Intercept Contract
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
    isfield(result, "timedSlopePath") && ...
    ~isempty(result.timedSlopePath.position_deg) && ...
    ~isempty(result.timedSlopePath.velocity_deg_s) && ...
    isfinite(result.goalLineInterceptTime_s);
if hasTimedEndpoint
    [targetPositionAtIntercept_deg, actualTargetVelocity_deg_s] = ...
        evaluateTargetMotion( ...
        targetMotion, result.goalLineInterceptTime_s);
    actualTerminalPosition_deg = ...
        result.timedSlopePath.position_deg(end, :);
    actualTerminalVelocity_deg_s = ...
        result.timedSlopePath.velocity_deg_s(end, :);
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
if ~interceptPassed
    result.Message = string(result.Message) + " " + validationMessage;
end

%% Section 4: Draw The Moving Target World Line
targetTrackHandles = struct( ...
    "TimeSpace", gobjects(0, 1), ...
    "Intercept", gobjects(0, 1), ...
    "Animation3D", gobjects(0, 1), ...
    "Animation2D", gobjects(0, 1));
if isfield(result, "spaceView") && ...
        isgraphics(result.spaceView.Axes)
    targetTrackHandles.TimeSpace = plot3(result.spaceView.Axes, ...
        trackPosition_deg(:, 1), trackPosition_deg(:, 2), trackTime_s, ...
        "-.", "Color", [0.72 0.10 0.72], "LineWidth", 2.3, ...
        "DisplayName", "Moving target");
    targetTrackHandles.Intercept = plot3(result.spaceView.Axes, ...
        interceptPosition_deg(1), interceptPosition_deg(2), ...
        interceptTime_s, "p", "MarkerSize", 13, ...
        "MarkerFaceColor", [0.95 0.30 0.82], ...
        "MarkerEdgeColor", "k", "DisplayName", "Intercept");
    legend(result.spaceView.Axes, "Location", "best");
end
if isfield(result, "animation") && isstruct(result.animation) && ...
        isfield(result.animation, "Axes3D") && ...
        isgraphics(result.animation.Axes3D)
    targetTrackHandles.Animation3D = ...
        result.animation.TargetWorldLine3D;
    targetTrackHandles.Animation2D = ...
        result.animation.TargetWorldLine2D;
end

result.InterceptMode = options.InterceptMode;
result.TargetMotion = targetMotion;
result.RequestedInterceptTime_s = options.SpecifiedInterceptTime_s;
result.InterceptTime_s = interceptTime_s;
result.InterceptPosition_deg = interceptPosition_deg;
result.TargetPositionAtIntercept_deg = targetPositionAtIntercept_deg;
result.TargetVelocityAtIntercept_deg_s = targetVelocityAtIntercept_deg_s;
result.SearchDiagnostics = searchDiagnostics;
result.InterceptValidation = interceptValidation;
result.TargetTrackTime_s = trackTime_s;
result.TargetTrackPosition_deg = trackPosition_deg;
result.TargetTrackHandles = targetTrackHandles;
result.InterceptOptions = options;
end

function options = resolveInterceptOptions(defaultOptions, overrides)
%% Section 0: Header & Readme
% Resolve and validate public intercept controls.
if ~isstruct(overrides) || ~isscalar(overrides)
    error("planAzElMovingTargetIntercept:InvalidOptions", ...
        "options must be a scalar struct.");
end
unknownFields = setdiff( ...
    fieldnames(overrides), fieldnames(defaultOptions), "stable");
if ~isempty(unknownFields)
    warning("planAzElMovingTargetIntercept:UnknownOptions", ...
        "Ignoring unknown option fields: %s.", ...
        strjoin(string(unknownFields), ", "));
    overrides = rmfield(overrides, unknownFields);
end
options = defaultOptions;
overrideFields = fieldnames(overrides);
for fieldIndex = 1:numel(overrideFields)
    fieldName = overrideFields{fieldIndex};
    if ~isempty(overrides.(fieldName))
        options.(fieldName) = overrides.(fieldName);
    end
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
validateattributes(options.MatchTargetVelocity, ...
    {'logical', 'numeric'}, {'scalar'});
options.MatchTargetVelocity = logical(options.MatchTargetVelocity);
if ~isstruct(options.PlannerOptions) || ~isscalar(options.PlannerOptions)
    error("planAzElMovingTargetIntercept:InvalidPlannerOptions", ...
        "PlannerOptions must be a scalar struct.");
end
end

function initialState = validateInterceptInitialState(initialState)
%% Section 0: Header & Readme
% Validate the state fields needed before the maintained planner runs.
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
%% Section 0: Header & Readme
% SYNTAX
%   targetMotion = validateTargetMotion(targetMotion)
%**************************************************************************
% PURPOSE
%   - Normalize sampled trajectories and the compatible linear model.
%**************************************************************************
% INPUTS
%   - targetMotion (scalar struct)
%       Sampled time_s/position_deg or constant-velocity reference fields.
%**************************************************************************
% OUTPUTS
%   - targetMotion (scalar struct)
%       Normalized model with ModelType and interpolation provenance.
%**************************************************************************
% UNITS
%   - Time is seconds; position is degrees; velocity is deg/s.
%**************************************************************************
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
%% Section 0: Header & Readme
% SYNTAX
%   position_deg = evaluateTargetMotion(targetMotion, queryTime_s)
%   [position_deg, velocity_deg_s] = ...
%       evaluateTargetMotion(targetMotion, queryTime_s)
%**************************************************************************
% PURPOSE
%   - Evaluate target position and instantaneous velocity without extrapolating.
%**************************************************************************
% INPUTS
%   - targetMotion (normalized scalar struct)
%       Sampled trajectory or constant-velocity model.
%   - queryTime_s (finite numeric array)
%       Times to evaluate.
%**************************************************************************
% OUTPUTS
%   - position_deg (N-by-2 numeric matrix)
%   - velocity_deg_s (N-by-2 numeric matrix)
%**************************************************************************
% UNITS
%   - Time is seconds; position is degrees; velocity is deg/s.
%**************************************************************************
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
%% Section 0: Header & Readme
% SYNTAX
%   derivativePolynomial = differentiatePiecewisePolynomial(polynomial)
%**************************************************************************
% PURPOSE
%   - Differentiate a scalar MATLAB piecewise polynomial without a toolbox.
%**************************************************************************
% INPUTS
%   - polynomial (scalar pp-form struct)
%       Piecewise polynomial returned by pchip.
%**************************************************************************
% OUTPUTS
%   - derivativePolynomial (scalar pp-form struct)
%       Analytic first derivative in the same break representation.
%**************************************************************************
% UNITS
%   - Coefficient units are inherited and divided by the x-axis unit.
%**************************************************************************
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
%% Section 0: Header & Readme
% SYNTAX
%   certifiable = targetHasCertifiedRecedingRay(initialState, targetMotion)
%**************************************************************************
% PURPOSE
%   - Recognize the legacy monotone geometry proved by bisection.
%**************************************************************************
% INPUTS
%   - initialState (normalized scalar struct)
%   - targetMotion (normalized scalar struct)
%**************************************************************************
% OUTPUTS
%   - certifiable (logical scalar)
%       True only for a constant-velocity target receding on one ray.
%**************************************************************************
% UNITS
%   - State positions are degrees and target velocity is deg/s.
%**************************************************************************
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
%% Section 0: Header & Readme
% SYNTAX
%   trackTime_s = targetTrackTimes( ...
%       targetMotion, startTime_s, endTime_s, sampleCount)
%**************************************************************************
% PURPOSE
%   - Retain both regular display samples and supplied trajectory knots.
%**************************************************************************
% INPUTS
%   - targetMotion (normalized scalar struct)
%   - startTime_s, endTime_s (ordered finite scalars)
%   - sampleCount (integer scalar at least two)
%**************************************************************************
% OUTPUTS
%   - trackTime_s (strictly increasing numeric column)
%**************************************************************************
% UNITS
%   - All time values are seconds.
%**************************************************************************
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
%% Section 0: Header & Readme
% SYNTAX
%   candidateTimes_s = recedingRaySearchTimes( ...
%       startTime_s, endTime_s, initialStep_s)
%**************************************************************************
% PURPOSE
%   - Create exponentially expanding candidates for a monotone residual.
%**************************************************************************
% INPUTS
%   - startTime_s, endTime_s (ordered finite scalars)
%   - initialStep_s (positive finite scalar)
%**************************************************************************
% OUTPUTS
%   - candidateTimes_s (strictly increasing numeric column)
%**************************************************************************
% UNITS
%   - All inputs and outputs are seconds.
%**************************************************************************
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
%% Section 0: Header & Readme
% SYNTAX
%   candidateTimes_s = arbitraryTargetSearchTimes( ...
%       targetMotion, startTime_s, endTime_s, maximumStep_s)
%**************************************************************************
% PURPOSE
%   - Scan a general path at every supplied knot and a bounded time step.
%**************************************************************************
% INPUTS
%   - targetMotion (normalized scalar struct)
%   - startTime_s, endTime_s (ordered finite scalars)
%   - maximumStep_s (positive finite scalar)
%**************************************************************************
% OUTPUTS
%   - candidateTimes_s (strictly increasing numeric column)
%**************************************************************************
% UNITS
%   - All time quantities are seconds.
%**************************************************************************
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
%% Section 0: Header & Readme
% SYNTAX
%   message = earliestSearchMessage(certifiedRecedingRay)
%**************************************************************************
% PURPOSE
%   - Describe whether a first bracket has an analytic monotonicity proof.
%**************************************************************************
% INPUTS
%   - certifiedRecedingRay (logical scalar)
%**************************************************************************
% OUTPUTS
%   - message (string scalar)
%**************************************************************************
% UNITS
%   - The returned diagnostic text has no physical units.
%**************************************************************************
if certifiedRecedingRay
    message = "Earliest feasible intercept was certified by a monotone " + ...
        "receding-ray bracket and bisection.";
else
    message = "First sampled feasibility transition on the arbitrary " + ...
        "target path was bracketed and bisected numerically.";
end
end

function method = conditionalSearchMethod(certifiedRecedingRay)
%% Section 0: Header & Readme
% SYNTAX
%   method = conditionalSearchMethod(certifiedRecedingRay)
%**************************************************************************
% PURPOSE
%   - Return a machine-readable earliest-search method identifier.
%**************************************************************************
% INPUTS
%   - certifiedRecedingRay (logical scalar)
%**************************************************************************
% OUTPUTS
%   - method (string scalar)
%**************************************************************************
% UNITS
%   - The identifier has no physical units.
%**************************************************************************
if certifiedRecedingRay
    method = "certifiedRecedingRayBisection";
else
    method = "sampledPathFirstTransitionBisection";
end
end

function validateEarliestInitialVelocity(initialState)
%% Section 0: Header & Readme
% Expose the current fixed-arrival retimer's scheduling limitation early.
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
%% Section 0: Header & Readme
% A direct obstacle-free route can match only a tangent terminal velocity.
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
        initialState, targetMotion, limits, options)
%% Section 0: Header & Readme
% Scan arbitrary paths in chronological order, retaining exponential
% bracketing only for the receding-ray case where monotonicity is proved.
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
certifiedRecedingRay = targetHasCertifiedRecedingRay( ...
    initialState, targetMotion);
if certifiedRecedingRay
    candidateTimes_s = recedingRaySearchTimes( ...
        searchStartTime_s, searchEndTime_s, options.InitialSearchStep_s);
else
    candidateTimes_s = arbitraryTargetSearchTimes( ...
        targetMotion, searchStartTime_s, searchEndTime_s, ...
        options.InitialSearchStep_s);
end
lowerTime_s = candidateTimes_s(1);
[lowerResidual_s, lowerArrivalTime_s] = interceptResidual( ...
    lowerTime_s, searchEndTime_s, initialState, targetMotion, ...
    limits, options);
upperTime_s = lowerTime_s;
upperResidual_s = lowerResidual_s;
upperArrivalTime_s = lowerArrivalTime_s;
bracketEvaluationCount = 1;
for candidateIndex = 2:numel(candidateTimes_s)
    if upperResidual_s <= 0
        break;
    end
    upperTime_s = candidateTimes_s(candidateIndex);
    [upperResidual_s, upperArrivalTime_s] = interceptResidual( ...
        upperTime_s, searchEndTime_s, initialState, targetMotion, ...
        limits, options);
    bracketEvaluationCount = bracketEvaluationCount + 1;
    if upperResidual_s > 0
        lowerTime_s = upperTime_s;
        lowerResidual_s = upperResidual_s;
        lowerArrivalTime_s = upperArrivalTime_s;
    end
end
if upperResidual_s > 0
    error("planAzElMovingTargetIntercept:NoInterceptInHorizon", ...
        "No feasible intercept was found within %.6g seconds.", ...
        options.MaximumSearchDuration_s);
end

iterationCount = 0;
while upperTime_s - lowerTime_s > ...
        options.SearchTimeTolerance_s && ...
        iterationCount < options.MaximumSearchIterations
    iterationCount = iterationCount + 1;
    middleTime_s = 0.5 * (lowerTime_s + upperTime_s);
    [middleResidual_s, middleArrivalTime_s] = interceptResidual( ...
        middleTime_s, searchEndTime_s, initialState, targetMotion, ...
        limits, options);
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
interceptTime_s = upperTime_s;
diagnostics = struct( ...
    "Success", lowerResidual_s > 0 && upperResidual_s <= 0 && ...
    upperTime_s - lowerTime_s <= options.SearchTimeTolerance_s, ...
    "Message", earliestSearchMessage(certifiedRecedingRay), ...
    "LowerInfeasibleTime_s", lowerTime_s, ...
    "UpperFeasibleTime_s", upperTime_s, ...
    "LowerResidual_s", lowerResidual_s, ...
    "UpperResidual_s", upperResidual_s, ...
    "LowerMinimumArrivalTime_s", lowerArrivalTime_s, ...
    "UpperMinimumArrivalTime_s", upperArrivalTime_s, ...
    "BracketEvaluationCount", bracketEvaluationCount, ...
    "BisectionIterationCount", iterationCount, ...
    "EarliestCertified", certifiedRecedingRay && ...
    lowerResidual_s > 0 && ...
    upperResidual_s <= 0 && upperTime_s - lowerTime_s <= ...
    options.SearchTimeTolerance_s, ...
    "EarliestNumericallyResolved", lowerResidual_s > 0 && ...
    upperResidual_s <= 0 && upperTime_s - lowerTime_s <= ...
    options.SearchTimeTolerance_s, ...
    "SearchMethod", conditionalSearchMethod(certifiedRecedingRay), ...
    "SearchStartTime_s", searchStartTime_s, ...
    "SearchEndTime_s", searchEndTime_s, ...
    "MaximumScanStep_s", options.InitialSearchStep_s);
end

function [residual_s, arrivalTime_s] = interceptResidual( ...
        candidateTime_s, searchEndTime_s, initialState, targetMotion, ...
        limits, options)
%% Section 0: Header & Readme
% Evaluate minimum planner arrival minus the candidate target time.
goalVelocity_deg_s = [0 0];
[targetPosition_deg, targetVelocity_deg_s] = evaluateTargetMotion( ...
    targetMotion, candidateTime_s);
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
            return;
        end
        rethrow(exception);
    end
    goalVelocity_deg_s = targetVelocity_deg_s;
end
goalState = struct( ...
    "time_s", searchEndTime_s + max(1000, ...
    100 * options.MaximumSearchDuration_s), ...
    "position_deg", targetPosition_deg, ...
    "velocity_deg_s", goalVelocity_deg_s, ...
    "acceleration_deg_s2", [0 0]);
plannerOptions = options.PlannerOptions;
plannerOptions.GoalTimeMode = "earliestArrival";
plannerOptions.FigureVisible = "off";
plannerOptions.ShowAnimation = false;
plannerOptions.ShowKinematicPlot = false;
plannerOptions.ShowSweptSurfaces = false;
plannerOptions.Verbose = false;
trialResult = planAzElMotion( ...
    [], initialState, goalState, limits, plannerOptions);
deletePlannerFigures(trialResult);
if ~trialResult.Success
    residual_s = Inf;
    arrivalTime_s = Inf;
    return;
end
arrivalTime_s = trialResult.goalLineInterceptTime_s;
residual_s = arrivalTime_s - candidateTime_s;
end

function value = plannerDisplayOption(overrides, defaults, fieldName)
%% Section 0: Header & Readme
% Resolve one planner display option without duplicating planner parsing.
value = defaults.(fieldName);
if isfield(overrides, fieldName) && ~isempty(overrides.(fieldName))
    value = overrides.(fieldName);
end
end

function deletePlannerFigures(plannerResult)
%% Section 0: Header & Readme
% Close every hidden figure produced by one feasibility trial.
containerNames = ["spaceView" "animation" "kinematicPlot"];
for containerIndex = 1:numel(containerNames)
    containerName = containerNames(containerIndex);
    if ~isfield(plannerResult, containerName)
        continue;
    end
    container = plannerResult.(containerName);
    if isstruct(container) && isfield(container, "Figure") && ...
            isgraphics(container.Figure)
        delete(container.Figure);
    end
end
end

function diagnostics = emptyInterceptSearchDiagnostics()
%% Section 0: Header & Readme
% Return the stable search schema for specified-time interception.
diagnostics = struct( ...
    "Success", true, ...
    "Message", "", ...
    "LowerInfeasibleTime_s", NaN, ...
    "UpperFeasibleTime_s", NaN, ...
    "LowerResidual_s", NaN, ...
    "UpperResidual_s", NaN, ...
    "LowerMinimumArrivalTime_s", NaN, ...
    "UpperMinimumArrivalTime_s", NaN, ...
    "BracketEvaluationCount", 0, ...
    "BisectionIterationCount", 0, ...
    "EarliestCertified", true, ...
    "EarliestNumericallyResolved", true, ...
    "SearchMethod", "specifiedTime", ...
    "SearchStartTime_s", NaN, ...
    "SearchEndTime_s", NaN, ...
    "MaximumScanStep_s", NaN);
end
