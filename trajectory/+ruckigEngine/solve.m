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

%% Section 2: Create And Synchronize Exact Axis Profiles

% Independent axis profiles generally finish at different times. Create the
% fastest eligible profiles and synchronize them before evaluation so the
% returned vector motion has one physical clock and unchanged boundary states.
profileAttempt = ruckigEngine.createSynchronizedMotion( ...
    initialState, terminalState, limits, options);
result.Diagnostics.Profile = profileAttempt.Profile;
result.Diagnostics.ElapsedTime = profileAttempt.ElapsedTime;
if ~profileAttempt.Success
    result.Message = profileAttempt.Message;
    result.TerminationReason = profileAttempt.TerminationReason;
    return;
end

%% Section 3: Evaluate The Synchronized Motion

% Synchronization is itself a transformation, so evaluate the final polynomial
% at uniform and switching times before checking the returned vector motion.
result = ruckigEngine.evaluateSynchronizedMotion( ...
    result, profileAttempt.Profile, initialState, options);

%% Section 4: Check And Classify The Returned Motion

% Axis constructors and synchronization cannot approve the assembled result.
% Re-evaluate continuous limits and optional affine rows, then classify the
% engine outcome. This dimension-neutral check cannot approve obstacle safety;
% the calling planner's full trajectory validator retains that responsibility.
result = ruckigEngine.internal.checkResult( ...
    result, profileAttempt.Profile, terminalState, limits, pathConstraints);
end

%% Section 5: Local Functions

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
