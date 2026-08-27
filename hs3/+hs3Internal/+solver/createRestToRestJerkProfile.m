function profile = createRestToRestJerkProfile( ...
        initialState, terminalState, limits, requestedFinalTime)
%% Section 0: Header & Readme
% SYNTAX
%   profile = hs3Internal.solver.createRestToRestJerkProfile( ...
%       initialState, terminalState, limits, requestedFinalTime)
%**************************************************************************
% PURPOSE
%   - Create the exact minimum-time monotone unit-displacement profile with
%     bounded velocity, acceleration, and piecewise-constant jerk.
%**************************************************************************
% INPUTS
%   - initialState (scalar normalized one-dimensional state struct)
%       Position, velocity, and acceleration must be zero.
%   - terminalState (scalar normalized one-dimensional state struct)
%       Position must be one and terminal velocity/acceleration must be zero.
%   - limits (scalar normalized one-dimensional limit struct)
%       Positive symmetric velocity, acceleration, and jerk maxima apply.
%   - requestedFinalTime (empty or finite scalar)
%       Empty selects minimum arrival. A supplied time stretches the minimum
%       profile when it is not earlier than the physical minimum.
%**************************************************************************
% OUTPUTS
%   - profile (scalar struct)
%       Success, message, variable-duration polynomial, phase jerk controls,
%       final time, minimum final time, and integrated squared jerk.
%**************************************************************************
% UNITS
%   - Time and coordinate units are caller-defined and must be consistent.
%**************************************************************************

%% Section 1: Create The Minimum-Time Phase Law

velocityLimit = limits.maximumVelocity;
accelerationLimit = limits.maximumAcceleration;
jerkLimit = limits.maximumJerk;
[jerkRampTime, constantAccelerationTime, cruiseTime] = ...
    profileDurations(velocityLimit, accelerationLimit, jerkLimit);

% The tiny scale guard keeps every analytic peak on the feasible side after
% floating-point evaluation without changing reported time at practical scale.
guardScale = 1 + 64 * eps;
phaseDuration = guardScale * [ ...
    jerkRampTime; constantAccelerationTime; jerkRampTime; ...
    cruiseTime; jerkRampTime; constantAccelerationTime; jerkRampTime];
phaseJerk = [1; 0; -1; 0; -1; 0; 1] * ...
    jerkLimit / guardScale^3;
retainedPhase = phaseDuration > 64 * eps;
phaseDuration = phaseDuration(retainedPhase);
phaseJerk = phaseJerk(retainedPhase);
minimumFinalTime = initialState.time + sum(phaseDuration);

profile = createEmptyProfile(minimumFinalTime);
if ~isempty(requestedFinalTime)
    requestedDuration = requestedFinalTime - initialState.time;
    minimumDuration = minimumFinalTime - initialState.time;
    timeTolerance = 256 * eps(max([1, requestedDuration, minimumDuration]));
    if requestedDuration < minimumDuration - timeTolerance
        profile.Message = ...
            "The requested final time is below the jerk-switching minimum.";
        return;
    end
    stretch = max(1, requestedDuration / minimumDuration);
    phaseDuration = stretch * phaseDuration;
    phaseJerk = phaseJerk / stretch^3;
end

%% Section 2: Integrate Every Constant-Jerk Phase Exactly

segmentCount = numel(phaseDuration);
positionPower = zeros(segmentCount, 1, 6);
velocityPower = zeros(segmentCount, 1, 5);
accelerationPower = zeros(segmentCount, 1, 4);
jerkPower = zeros(segmentCount, 1, 3);
position = initialState.position;
velocity = initialState.velocity;
acceleration = initialState.acceleration;

for segmentIndex = 1:segmentCount
    duration = phaseDuration(segmentIndex);
    jerk = phaseJerk(segmentIndex);
    positionCoefficient = [ ...
        position, velocity * duration, ...
        0.5 * acceleration * duration^2, jerk * duration^3 / 6, 0, 0];
    velocityCoefficient = [ ...
        velocity, acceleration * duration, ...
        0.5 * jerk * duration^2, 0, 0];
    accelerationCoefficient = [ ...
        acceleration, jerk * duration, 0, 0];
    positionPower(segmentIndex, 1, :) = positionCoefficient;
    velocityPower(segmentIndex, 1, :) = velocityCoefficient;
    accelerationPower(segmentIndex, 1, :) = accelerationCoefficient;
    jerkPower(segmentIndex, 1, :) = [jerk, 0, 0];
    position = sum(positionCoefficient);
    velocity = sum(velocityCoefficient);
    acceleration = sum(accelerationCoefficient);
end

segmentStartTime = initialState.time + ...
    [0; cumsum(phaseDuration(1:end - 1))];
finalTime = initialState.time + sum(phaseDuration);
polynomial = struct( ...
    "SegmentCount", segmentCount, ...
    "SegmentStartTime", segmentStartTime, ...
    "SegmentDuration", phaseDuration, ...
    "FinalTime", finalTime, ...
    "positionPower", positionPower, ...
    "velocityPower", velocityPower, ...
    "accelerationPower", accelerationPower, ...
    "jerkPower", jerkPower, ...
    "TerminalState", struct( ...
    "position", position, ...
    "velocity", velocity, ...
    "acceleration", acceleration));

%% Section 3: Assemble The Profile

endpointTolerance = 256 * eps(max([1, abs(position)]));
endpointError = max(abs([ ...
    position - terminalState.position, ...
    velocity - terminalState.velocity, ...
    acceleration - terminalState.acceleration]));
profile.Success = endpointError <= endpointTolerance;
profile.Message = "The exact rest-to-rest jerk-switching profile was created.";
profile.Polynomial = polynomial;
profile.ControlJerk = phaseJerk;
profile.FinalTime = finalTime;
profile.IntegratedSquaredJerk = sum(phaseJerk .^ 2 .* phaseDuration);
if ~profile.Success
    profile.Message = sprintf( ...
        "Jerk-switching endpoint error %.9g exceeds tolerance %.9g.", ...
        endpointError, endpointTolerance);
end
end

%% Section 4: Local Functions

function [jerkRampTime, constantAccelerationTime, cruiseTime] = ...
        profileDurations(velocityLimit, accelerationLimit, jerkLimit)
% Solve the symmetric unit-distance jerk-limited switching durations.
accelerationDistance = 2 * accelerationLimit^3 / jerkLimit^2;
if accelerationDistance >= 1
    jerkRampTime = nthroot(1 / (2 * jerkLimit), 3);
    constantAccelerationTime = 0;
else
    jerkRampTime = accelerationLimit / jerkLimit;
    constantAccelerationTime = 0.5 * ( ...
        sqrt(jerkRampTime^2 + 4 / accelerationLimit) - 3 * jerkRampTime);
end
peakVelocity = jerkLimit * jerkRampTime * ...
    (jerkRampTime + constantAccelerationTime);
cruiseTime = 0;
if peakVelocity <= velocityLimit
    return;
end
if velocityLimit <= accelerationLimit^2 / jerkLimit
    jerkRampTime = sqrt(velocityLimit / jerkLimit);
    constantAccelerationTime = 0;
else
    jerkRampTime = accelerationLimit / jerkLimit;
    constantAccelerationTime = velocityLimit / accelerationLimit - ...
        jerkRampTime;
end
minimumDistance = velocityLimit * ...
    (2 * jerkRampTime + constantAccelerationTime);
cruiseTime = max(0, (1 - minimumDistance) / velocityLimit);
end

function profile = createEmptyProfile(minimumFinalTime)
% Define stable fields for an accepted profile or a too-short fixed request.
profile = struct( ...
    "Success", false, ...
    "Message", "No jerk-switching profile was created.", ...
    "Polynomial", struct(), ...
    "ControlJerk", zeros(0, 1), ...
    "FinalTime", NaN, ...
    "MinimumFinalTime", minimumFinalTime, ...
    "IntegratedSquaredJerk", Inf);
end
