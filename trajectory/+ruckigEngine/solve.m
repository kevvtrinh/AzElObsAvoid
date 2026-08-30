function result = solve( ...
        initialState, terminalState, limits, options, pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   options = ruckigEngine.solve()
%   result = ruckigEngine.solve(initialState, terminalState, limits)
%   result = ruckigEngine.solve( ...
%       initialState, terminalState, limits, options)
%   result = ruckigEngine.solve( ...
%       initialState, terminalState, limits, options, pathConstraints)
%**************************************************************************
% PURPOSE
%   - Create certified state-to-state trajectories with the extracted
%     Ruckig-derived exact switching equations.
%**************************************************************************
% INPUTS
%   - initialState, terminalState, limits (scalar structs)
%       Dimension-neutral boundary state and limit inputs for this engine.
%   - options (scalar struct, optional; default struct())
%       Direct Ruckig-engine overrides. Empty fields use engine defaults.
%   - pathConstraints (scalar struct, optional; default empty)
%       Affine rows continuously certify the constructed exact profile. They
%       reject a violating profile but do not steer profile construction.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Certified success or identified unsupported/failure record.
%       Invalid requirements throw identified errors.
%**************************************************************************
% UNITS
%   - Units are caller-defined and must be consistent across derivatives.
%**************************************************************************
% REFERENCE
%   - L. Berscheid and T. Kroeger, "Jerk-limited Real-time Trajectory
%     Generation with Arbitrary Target States," Robotics: Science and
%     Systems XVII, 2021. https://doi.org/10.15607/RSS.2021.XVII.015
%**************************************************************************

if nargin == 0
    result = ruckigEngine.defaultOptions();
    return;
end
if nargin < 4 || isempty(options)
    options = struct();
end
if nargin < 5 || isempty(pathConstraints)
    pathConstraints = struct();
end

%% Section 1: Normalize And Check Eligibility

[initialState, terminalState, limits, pathConstraints] = ...
    ruckigEngine.internal.normalizeRequest( ...
    initialState, terminalState, limits, pathConstraints);
options = normalizeEngineOptions(options);
result = ruckigEngine.internal.createEmptyResult( ...
    initialState, terminalState, limits, options, pathConstraints);
eligibility = ruckigEngine.checkEligibility( ...
    initialState, terminalState, limits, options, pathConstraints);
result.Diagnostics = struct("Eligibility", eligibility);
if ~eligibility.Supported
    result.Message = eligibility.Message;
    result.TerminationReason = eligibility.TerminationReason;
    return;
end

[hasBoundaryInfeasibility, boundaryMessage] = ...
    detectBoundaryKinematicInfeasibility( ...
    initialState, terminalState, limits);
if hasBoundaryInfeasibility
    result.Message = boundaryMessage;
    result.TerminationReason = "kinematicallyInfeasibleBoundaryState";
    return;
end

%% Section 2: Create An Exact Switching Profile

requestedFinalTime = [];
if options.TimeMode == "fixed"
    requestedFinalTime = options.FinalTime;
    if isempty(requestedFinalTime)
        requestedFinalTime = terminalState.maximumTime;
    end
end
solveTimer = tic;
profile = struct();
[hasDirectProgress, progressInitialState, progressTerminalState, ...
    progressLimits, displacement] = createDirectProgressProblem( ...
    initialState, terminalState, limits, options);
if hasDirectProgress
    progressProfile = ruckigEngine.createRestToRestJerkProfile( ...
        progressInitialState, progressTerminalState, progressLimits, ...
        requestedFinalTime);
    if progressProfile.Success
        profile = liftDirectProfile( ...
            progressProfile, displacement, initialState);
    end
end
if isempty(fieldnames(profile))
    profile = ruckigEngine.createSynchronizedJerkProfile( ...
        initialState, terminalState, limits, requestedFinalTime);
end
elapsedTime = toc(solveTimer);
result.Diagnostics.Profile = profile;
result.Diagnostics.ElapsedTime = elapsedTime;
if ~profile.Success
    [reason, message] = classifyProfileFailure( ...
        profile, initialState, options, requestedFinalTime);
    result.Message = message;
    result.TerminationReason = reason;
    return;
end
if options.TimeMode == "earliestArrival" && ...
        profile.FinalTime >= terminalState.maximumTime - ...
        options.ArrivalTimeTolerance
    result.Message = ...
        "The certified switching profile does not fit inside maximumTime.";
    result.TerminationReason = "infeasibleTimeHorizon";
    return;
end

%% Section 3: Reconstruct And Validate The Result

result.FinalTime = profile.FinalTime;
result.Duration = profile.FinalTime - initialState.time;
result.ControlJerk = profile.ControlJerk;
result.Polynomial = profile.Polynomial;
result.IntegratedSquaredJerk = profile.IntegratedSquaredJerk;
uniformTime = (initialState.time:options.SampleTime:profile.FinalTime).';
sampleTime = unique([uniformTime; ...
    profile.Polynomial.SegmentStartTime; profile.FinalTime]);
[sampleTime, position, velocity, acceleration, jerk] = ...
    ruckigEngine.internal.evaluatePolynomial( ...
    profile.Polynomial, sampleTime);
result.time = sampleTime;
result.position = position;
result.velocity = velocity;
result.acceleration = acceleration;
result.jerk = jerk;
[inequality, equality] = ...
    ruckigEngine.internal.evaluatePolynomialConstraints( ...
    profile.Polynomial, terminalState, limits, pathConstraints);
result.MaximumConstraintViolation = max([ ...
    0; inequality(:); abs(equality(:))]);
result.Validation = ruckigEngine.internal.validateResult(result);
result.Success = result.Validation.Passed;
if result.Success
    result.Message = ...
        "A kinematically constrained trajectory was found and independently validated.";
    result.TerminationReason = "goalReached";
else
    if ~isempty(pathConstraints.Tau)
        result.Message = "The exact switching profile violates an affine " + ...
            "path constraint. " + result.Validation.Message;
        result.TerminationReason = "pathConstraintViolation";
    else
        result.Message = result.Validation.Message;
        result.TerminationReason = "exactProfileValidationFailed";
    end
end
end

%% Section 4: Local Functions

function options = normalizeEngineOptions(options)
% Resolve only direct Ruckig settings; routing decisions belong to callers.
if ~isstruct(options) || ~isscalar(options)
    error("ruckigEngine:InvalidOptions", ...
        "options must be a scalar struct or empty.");
end
resolvedOptions = ruckigEngine.defaultOptions();
unknownNames = setdiff(string(fieldnames(options)), ...
    string(fieldnames(resolvedOptions)), "stable");
if ~isempty(unknownNames)
    warning("ruckigEngine:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
for fieldName = string(fieldnames(resolvedOptions)).'
    if isfield(options, fieldName) && ~isempty(options.(fieldName))
        resolvedOptions.(fieldName) = options.(fieldName);
    end
end
resolvedOptions.TimeMode = string(resolvedOptions.TimeMode);
if ~isscalar(resolvedOptions.TimeMode) || ...
        ~any(resolvedOptions.TimeMode == ["fixed", "earliestArrival"])
    error("ruckigEngine:InvalidTimeMode", ...
        "TimeMode must be 'fixed' or 'earliestArrival'.");
end
if ~isempty(resolvedOptions.FinalTime)
    validateattributes(resolvedOptions.FinalTime, {'numeric'}, ...
        {'real', 'finite', 'scalar'});
    resolvedOptions.FinalTime = double(resolvedOptions.FinalTime);
end
for fieldName = ["SampleTime", "ConstraintTolerance", ...
        "ArrivalTimeTolerance"]
    validateattributes(resolvedOptions.(fieldName), {'numeric'}, ...
        {'real', 'finite', 'scalar', 'positive'});
    resolvedOptions.(fieldName) = double(resolvedOptions.(fieldName));
end
isLogical = islogical(resolvedOptions.Verbose) && ...
    isscalar(resolvedOptions.Verbose);
isBinaryNumeric = isnumeric(resolvedOptions.Verbose) && ...
    isscalar(resolvedOptions.Verbose) && ...
    isfinite(resolvedOptions.Verbose) && ...
    any(resolvedOptions.Verbose == [0, 1]);
if ~isLogical && ~isBinaryNumeric
    error("ruckigEngine:InvalidVerbose", ...
        "Verbose must be a scalar logical or binary numeric value.");
end
resolvedOptions.Verbose = logical(resolvedOptions.Verbose);
options = resolvedOptions;
end

function [isEligible, progressInitialState, progressTerminalState, ...
        progressLimits, displacement] = createDirectProgressProblem( ...
        initialState, terminalState, limits, options)
% Reduce eligible rest-to-rest motion to one scalar straight-line progress.
dimensionCount = numel(initialState.position);
displacement = terminalState.position - initialState.position;
progressInitialState = struct();
progressTerminalState = struct();
progressLimits = struct();
endpointDerivatives = [ ...
    initialState.velocity, terminalState.velocity, ...
    initialState.acceleration, terminalState.acceleration];
derivativeTolerance = 256 * eps(max([1, abs(endpointDerivatives)]));
isEligible = dimensionCount > 1 && any(displacement ~= 0) && ...
    all(abs(endpointDerivatives) <= derivativeTolerance);
if ~isEligible
    return;
end
activeCoordinate = displacement ~= 0;
displacementMagnitude = abs(displacement(activeCoordinate));
normalizedLimits = [ ...
    limits.maximumVelocity(activeCoordinate) ./ displacementMagnitude; ...
    limits.maximumAcceleration(activeCoordinate) ./ displacementMagnitude; ...
    limits.maximumJerk(activeCoordinate) ./ displacementMagnitude];
if options.TimeMode == "earliestArrival"
    minimumLimit = min(normalizedLimits, [], 2);
    tieTolerance = 256 * eps(max([1; normalizedLimits(:)]));
    oneAxisOwnsAllLimits = any(all( ...
        normalizedLimits <= minimumLimit + tieTolerance, 1));
    if ~oneAxisOwnsAllLimits
        isEligible = false;
        return;
    end
end
progressMaximum = min(normalizedLimits, [], 2);
progressInitialState = struct( ...
    "time", initialState.time, ...
    "position", 0, "velocity", 0, "acceleration", 0);
progressTerminalState = struct( ...
    "position", 1, "velocity", 0, "acceleration", 0, ...
    "maximumTime", terminalState.maximumTime);
progressLimits = struct( ...
    "maximumVelocity", progressMaximum(1), ...
    "maximumAcceleration", progressMaximum(2), ...
    "maximumJerk", progressMaximum(3));
end

function profile = liftDirectProfile( ...
        progressProfile, displacement, initialState)
% Lift one scalar switching polynomial into every requested coordinate.
scalarPolynomial = progressProfile.Polynomial;
dimensionCount = numel(displacement);
displacementScale = reshape(displacement, 1, dimensionCount, 1);
polynomial = scalarPolynomial;
polynomial.positionPower = ...
    scalarPolynomial.positionPower .* displacementScale;
polynomial.positionPower(:, :, 1) = ...
    polynomial.positionPower(:, :, 1) + initialState.position;
polynomial.velocityPower = ...
    scalarPolynomial.velocityPower .* displacementScale;
polynomial.accelerationPower = ...
    scalarPolynomial.accelerationPower .* displacementScale;
polynomial.jerkPower = ...
    scalarPolynomial.jerkPower .* displacementScale;
scalarTerminal = scalarPolynomial.TerminalState;
polynomial.TerminalState = struct( ...
    "position", initialState.position + ...
    scalarTerminal.position * displacement, ...
    "velocity", scalarTerminal.velocity * displacement, ...
    "acceleration", scalarTerminal.acceleration * displacement);
profile = progressProfile;
profile.Polynomial = polynomial;
profile.ControlJerk = progressProfile.ControlJerk * displacement;
profile.IntegratedSquaredJerk = ...
    progressProfile.IntegratedSquaredJerk * sum(displacement .^ 2);
end

function [reason, message] = classifyProfileFailure( ...
        profile, initialState, options, requestedFinalTime)
% Distinguish a too-short fixed request from an unsupported switching family.
reason = "unsupportedSwitchingFamily";
message = string(profile.Message);
if options.TimeMode ~= "fixed" || isempty(requestedFinalTime)
    return;
end
minimumFinalTime = NaN;
if isfield(profile, "MinimumFinalTime") && ...
        isfinite(profile.MinimumFinalTime)
    minimumFinalTime = profile.MinimumFinalTime;
end
if ~isfinite(minimumFinalTime) && ...
        isfield(profile, "MinimumAxisDuration") && ...
        all(isfinite(profile.MinimumAxisDuration))
    minimumFinalTime = initialState.time + ...
        max(profile.MinimumAxisDuration);
end
if isfinite(minimumFinalTime) && requestedFinalTime < minimumFinalTime
    reason = "fixedTimeBelowMinimum";
    message = sprintf( ...
        "Requested final time %.12g is below the certified minimum %.12g.", ...
        requestedFinalTime, minimumFinalTime);
end
end

function [value, message] = detectBoundaryKinematicInfeasibility( ...
        initialState, terminalState, limits)
% Prove boundary states that must cross a velocity bound immediately.
velocityScale = max([1, abs(limits.velocityLower), ...
    abs(limits.velocityUpper)], [], 2);
accelerationScale = max([1, abs(limits.accelerationLower), ...
    abs(limits.accelerationUpper)], [], 2);
velocityTolerance = 128 * eps(velocityScale);
accelerationTolerance = 128 * eps(accelerationScale);
initialBelow = initialState.velocity <= ...
    limits.velocityLower + velocityTolerance;
initialAbove = initialState.velocity >= ...
    limits.velocityUpper - velocityTolerance;
terminalBelow = terminalState.velocity <= ...
    limits.velocityLower + velocityTolerance;
terminalAbove = terminalState.velocity >= ...
    limits.velocityUpper - velocityTolerance;
initialOutward = (initialBelow & ...
    initialState.acceleration < -accelerationTolerance) | ...
    (initialAbove & ...
    initialState.acceleration > accelerationTolerance);
terminalOutward = (terminalBelow & ...
    terminalState.acceleration > accelerationTolerance) | ...
    (terminalAbove & ...
    terminalState.acceleration < -accelerationTolerance);
value = any(initialOutward) || any(terminalOutward);
if any(initialOutward)
    dimensionIndex = find(initialOutward, 1);
    message = sprintf( ...
        "Initial axis %d is at a velocity bound with outward acceleration.", ...
        dimensionIndex);
elseif any(terminalOutward)
    dimensionIndex = find(terminalOutward, 1);
    message = sprintf( ...
        "Terminal axis %d requires an out-of-bound predecessor velocity.", ...
        dimensionIndex);
else
    message = "";
end
end
