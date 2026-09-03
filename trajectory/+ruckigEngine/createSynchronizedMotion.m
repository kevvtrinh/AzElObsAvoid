function attempt = createSynchronizedMotion( ...
        initialState, terminalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   attempt = ruckigEngine.createSynchronizedMotion( ...
%       initialState, terminalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Create the exact fastest profile and synchronize every motion axis.
%   - Retain direct-progress use, profile failure, and solve timing explicitly.
%**************************************************************************
% INPUTS
%   - initialState, terminalState, limits (normalized scalar structs)
%       Dimension-neutral boundary states and derivative limits.
%   - options (resolved scalar struct)
%       Fixed or earliest-arrival time policy and tolerances.
%**************************************************************************
% OUTPUTS
%   - attempt (scalar struct)
%       Profile, requested final time, elapsed time, success, message, and
%       termination reason.
%**************************************************************************
% UNITS
%   - Units are caller-defined and consistent across derivatives; time is in
%     the caller's time unit.
%**************************************************************************

%% Section 1: Resolve The Requested Final Time

% Axis profiles are first created from the boundary states. A fixed request
% supplies one shared final time; earliest arrival leaves synchronization free
% to use the slowest exact axis profile.

requestedFinalTime = [];
if options.TimeMode == "fixed"
    requestedFinalTime = options.FinalTime;
    if isempty(requestedFinalTime)
        requestedFinalTime = terminalState.maximumTime;
    end
end
%% Section 2: Create And Synchronize Exact Profiles

% Rest-to-rest collinear motion has one scalar progress law, which avoids
% independent-axis numerical disagreement. Other eligible states use the
% general synchronized jerk-profile constructor.

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

%% Section 3: Classify The Profile Outcome

success = profile.Success;
message = "";
terminationReason = "";
if ~profile.Success
    [terminationReason, message] = classifyProfileFailure( ...
        profile, initialState, options, requestedFinalTime);
elseif options.TimeMode == "earliestArrival" && ...
        profile.FinalTime > terminalState.maximumTime + ...
        options.ArrivalTimeTolerance
    success = false;
    message = ...
        "The certified switching profile does not fit inside maximumTime.";
    terminationReason = "infeasibleTimeHorizon";
end
attempt = struct( ...
    "Success", success, ...
    "Message", message, ...
    "TerminationReason", terminationReason, ...
    "RequestedFinalTime", requestedFinalTime, ...
    "Profile", profile, ...
    "ElapsedTime", elapsedTime, ...
    "UsedDirectProgress", hasDirectProgress);
end

%% Section 4: Local Functions

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
