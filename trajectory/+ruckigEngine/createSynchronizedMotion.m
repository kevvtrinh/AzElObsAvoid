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
    progressProfile = createRestToRestProfile(progressInitialState, progressTerminalState, progressLimits, requestedFinalTime);
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
    polynomial.positionPower_units = scalarPolynomial.positionPower_units .* displacementScale;
    polynomial.positionPower_units(:, :, 1) = polynomial.positionPower_units(:, :, 1) + initialState.position;
    polynomial.velocityPower_units_s     = scalarPolynomial.velocityPower_units_s .* displacementScale;
    polynomial.accelerationPower_units_s2 = scalarPolynomial.accelerationPower_units_s2 .* displacementScale;
    polynomial.jerkPower_units_s3         = scalarPolynomial.jerkPower_units_s3 .* displacementScale;
    scalarTerminal = scalarPolynomial.TerminalState;
    polynomial.TerminalState = struct("position_units", initialState.position + ...
        scalarTerminal.position_units * displacement, ...
        "velocity_units_s", scalarTerminal.velocity_units_s * displacement, ...
        "acceleration_units_s2", scalarTerminal.acceleration_units_s2 * displacement);
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

function profile = createRestToRestProfile(initialState, terminalState, limits, requestedFinalTime)
    % Preserve Ruckig timing reserves while sharing the analytic law and exporter.
    velocityLimit     = limits.maximumVelocity;
    accelerationLimit = limits.maximumAcceleration;
    jerkLimit         = limits.maximumJerk;
    [phaseDuration, phaseJerk] = motionCore.createRestToRestLaw(1, velocityLimit, accelerationLimit, jerkLimit);

    % Add roundoff slack so analytic peaks remain within their limits.
    guardScale    = 1 + 64 * eps;
    phaseDuration = guardScale * phaseDuration.';
    phaseJerk = phaseJerk.' / guardScale^3;
    retainedPhase    = phaseDuration > 64 * eps;
    phaseDuration    = phaseDuration(retainedPhase);
    phaseJerk        = phaseJerk(retainedPhase);
    minimumFinalTime = initialState.time + sum(phaseDuration);

    profile = struct('Success',false,'Message',"No jerk-switching profile was created.", ...
        'Polynomial',struct(),'ControlJerk',zeros(0,1),'FinalTime',NaN, ...
        'MinimumFinalTime',minimumFinalTime,'IntegratedSquaredJerk',Inf);
    if ~isempty(requestedFinalTime)
        requestedDuration = requestedFinalTime - initialState.time;
        minimumDuration   = minimumFinalTime - initialState.time;
        timeTolerance     = 256 * eps(max([1, requestedDuration, minimumDuration]));
        if requestedDuration < minimumDuration - timeTolerance
            profile.Message = "The requested final time is below the jerk-switching minimum.";
            return;
        end
        stretch       = max(1, requestedDuration / minimumDuration);
        phaseDuration = stretch * phaseDuration;
        phaseJerk     = phaseJerk / stretch^3;
    end
    state = struct('time_s',initialState.time,'position_units',initialState.position, ...
        'velocity_units_s',initialState.velocity,'acceleration_units_s2',initialState.acceleration);
    [polynomial, terminal] = motionCore.createJerkPolynomial(state,[0;cumsum(phaseDuration)],phaseJerk);
    position = terminal.position_units;
    velocity = terminal.velocity_units_s;
    acceleration = terminal.acceleration_units_s2;
    finalTime = polynomial.FinalTime_s;
    endpointTolerance = 256 * eps(max([1, abs(position)]));
    endpointError     = max(abs([ position - terminalState.position, velocity - terminalState.velocity, acceleration - terminalState.acceleration]));
    profile.Success               = endpointError <= endpointTolerance;
    profile.Message               = "The exact rest-to-rest jerk-switching profile was created.";
    profile.Polynomial            = polynomial;
    profile.ControlJerk           = phaseJerk;
    profile.FinalTime             = finalTime;
    profile.IntegratedSquaredJerk = sum(phaseJerk .^ 2 .* phaseDuration);
    if ~profile.Success
        profile.Message = sprintf("Jerk-switching endpoint error %.9g exceeds tolerance %.9g.", endpointError, endpointTolerance);
    end
end
