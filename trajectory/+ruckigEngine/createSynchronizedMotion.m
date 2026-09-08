function attempt = createSynchronizedMotion(initialState, terminalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX: attempt = ruckigEngine.createSynchronizedMotion(initialState, terminalState, limits, options)
% PURPOSE: Create and synchronize exact fastest profiles across all motion axes.
% INPUTS: Normalized dimension-neutral states, derivative limits, and resolved
%   fixed/earliest-arrival options and tolerances.
% OUTPUTS: Profile, requested final time, timing, status, and termination reason.
% UNITS: Caller-defined position and time units consistent across derivatives.

%% Section 1: Resolve The Requested Final Time

% Synchronize axes at the requested fixed time or the earliest feasible time.

requestedFinalTime = [];
if options.TimeMode == "fixed"
    requestedFinalTime = options.FinalTime;
    if isempty(requestedFinalTime)
        requestedFinalTime = terminalState.maximumTime;
    end
end
%% Section 2: Create And Synchronize Exact Profiles

% Use scalar progress for eligible rest-to-rest straight-line motion.
% Otherwise construct synchronized axis profiles.

solveTimer = tic;
profile    = struct();
[hasDirectProgress, progressInitialState, progressTerminalState, ...
    progressLimits, displacement] = createDirectProgressProblem(initialState, terminalState, limits, options);
if hasDirectProgress
    progressProfile = ruckigEngine.createRestToRestJerkProfile(progressInitialState, progressTerminalState, progressLimits, requestedFinalTime);
    if progressProfile.Success
        profile = liftDirectProfile(progressProfile, displacement, initialState);
    end
end
if isempty(fieldnames(profile))
    profile = ruckigEngine.createSynchronizedJerkProfile(initialState, terminalState, limits, requestedFinalTime);
end
elapsedTime = toc(solveTimer);

%% Section 3: Classify The Profile Outcome

success           = profile.Success;
message           = "";
terminationReason = "";
if ~profile.Success
    [terminationReason, message] = classifyProfileFailure(profile, initialState, options, requestedFinalTime);
elseif options.TimeMode == "earliestArrival" && profile.FinalTime > terminalState.maximumTime + options.ArrivalTimeTolerance
    success           = false;
    message           = "The certified switching profile does not fit inside maximumTime.";
    terminationReason = "infeasibleTimeHorizon";
end
attempt = struct("Success", success, ...
    "Message", message, ...
    "TerminationReason", terminationReason, ...
    "RequestedFinalTime", requestedFinalTime, ...
    "Profile", profile, ...
    "ElapsedTime", elapsedTime, ...
    "UsedDirectProgress", hasDirectProgress);
end

%% Section 4: Local Functions

function [isEligible, progressInitialState, progressTerminalState, progressLimits, displacement] = createDirectProgressProblem(initialState, terminalState, limits, options)
    % Reduce eligible straight-line motion to scalar progress.
    dimensionCount        = numel(initialState.position);
    displacement          = terminalState.position - initialState.position;
    progressInitialState  = struct();
    progressTerminalState = struct();
    progressLimits        = struct();
    endpointDerivatives   = [ ...
        initialState.velocity, terminalState.velocity, ...
        initialState.acceleration, terminalState.acceleration];
    derivativeTolerance = 256 * eps(max([1, abs(endpointDerivatives)]));
    isEligible          = dimensionCount > 1 && any(displacement ~= 0) && all(abs(endpointDerivatives) <= derivativeTolerance);
    if ~isEligible
        return;
    end
    activeCoordinate      = displacement ~= 0;
    displacementMagnitude = abs(displacement(activeCoordinate));
    normalizedLimits      = [ ...
        limits.maximumVelocity(activeCoordinate) ./ displacementMagnitude; limits.maximumAcceleration(activeCoordinate) ./ displacementMagnitude; limits.maximumJerk(activeCoordinate) ./ displacementMagnitude];
    if options.TimeMode == "earliestArrival"
        minimumLimit         = min(normalizedLimits, [], 2);
        tieTolerance         = 256 * eps(max([1; normalizedLimits(:)]));
        oneAxisOwnsAllLimits = any(all(normalizedLimits <= minimumLimit + tieTolerance, 1));
        if ~oneAxisOwnsAllLimits
            isEligible = false;
            return;
        end
    end
    progressMaximum      = min(normalizedLimits, [], 2);
    progressInitialState = struct("time", initialState.time, ...
        "position", 0, "velocity", 0, "acceleration", 0);
    progressTerminalState = struct("position", 1, "velocity", 0, "acceleration", 0, ...
        "maximumTime", terminalState.maximumTime);
    progressLimits = struct("maximumVelocity", progressMaximum(1), ...
        "maximumAcceleration", progressMaximum(2), ...
        "maximumJerk", progressMaximum(3));
end

function profile = liftDirectProfile(progressProfile, displacement, initialState)
    % Map scalar progress to each coordinate.
    scalarPolynomial  = progressProfile.Polynomial;
    dimensionCount    = numel(displacement);
    displacementScale = reshape(displacement, 1, dimensionCount, 1);
    polynomial        = scalarPolynomial;
    polynomial.positionPower = scalarPolynomial.positionPower .* displacementScale;
    polynomial.positionPower(:, :, 1) = polynomial.positionPower(:, :, 1) + initialState.position;
    polynomial.velocityPower     = scalarPolynomial.velocityPower .* displacementScale;
    polynomial.accelerationPower = scalarPolynomial.accelerationPower .* displacementScale;
    polynomial.jerkPower         = scalarPolynomial.jerkPower .* displacementScale;
    scalarTerminal = scalarPolynomial.TerminalState;
    polynomial.TerminalState = struct("position", initialState.position + ...
        scalarTerminal.position * displacement, ...
        "velocity", scalarTerminal.velocity * displacement, ...
        "acceleration", scalarTerminal.acceleration * displacement);
    profile = progressProfile;
    profile.Polynomial            = polynomial;
    profile.ControlJerk           = progressProfile.ControlJerk * displacement;
    profile.IntegratedSquaredJerk = progressProfile.IntegratedSquaredJerk * sum(displacement .^ 2);
end

function [reason, message] = classifyProfileFailure(profile, initialState, options, requestedFinalTime)
    % Distinguish a too-short duration from an unsupported switching family.
    reason  = "unsupportedSwitchingFamily";
    message = string(profile.Message);
    if options.TimeMode ~= "fixed" || isempty(requestedFinalTime)
        return;
    end
    minimumFinalTime = NaN;
    if isfield(profile, "MinimumFinalTime") && isfinite(profile.MinimumFinalTime)
        minimumFinalTime = profile.MinimumFinalTime;
    end
    if ~isfinite(minimumFinalTime) && isfield(profile, "MinimumAxisDuration") && all(isfinite(profile.MinimumAxisDuration))
        minimumFinalTime = initialState.time + max(profile.MinimumAxisDuration);
    end
    if isfinite(minimumFinalTime) && requestedFinalTime < minimumFinalTime
        reason  = "fixedTimeBelowMinimum";
        message = sprintf("Requested final time %.12g is below the certified minimum %.12g.", requestedFinalTime, minimumFinalTime);
    end
end
