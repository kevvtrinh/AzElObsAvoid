function attempt = createSynchronizedAccelerationMotion( ...
        initialState, terminalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   attempt = ruckigEngine.createSynchronizedAccelerationMotion( ...
%       initialState, terminalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Create an exact second-order Ruckig position trajectory when
%     acceleration is the discontinuous control and jerk is unconstrained.
%**************************************************************************
% INPUTS
%   - initialState, terminalState, limits (normalized scalar structs)
%       Position and velocity boundary states with symmetric velocity and
%       acceleration bounds. Acceleration endpoint values are not imposed.
%   - options (resolved scalar struct)
%       Earliest-arrival or fixed-time policy and numerical tolerances.
%**************************************************************************
% OUTPUTS
%   - attempt (scalar struct)
%       Exact synchronized profile, solve status, reason, and elapsed time.
%**************************************************************************
% UNITS
%   - Units are caller-defined and consistent across position derivatives.
%**************************************************************************

% The switching equations are adapted from Ruckig v0.19.4 under its MIT
% license; see trajectory/THIRD_PARTY_NOTICES.txt.

%% Section 1: Create Independent Minimum-Time Profiles

solveTimer = tic;
dimensionCount = numel(initialState.position);
minimumProfiles = cell(dimensionCount, 1);
candidateSets = cell(dimensionCount, 1);
minimumDuration = NaN(1, dimensionCount);
for dimensionIndex = 1:dimensionCount
    [minimumProfiles{dimensionIndex}, candidateSets{dimensionIndex}] = ...
        createMinimumAxisProfile(initialState, terminalState, ...
        limits, dimensionIndex);
    if minimumProfiles{dimensionIndex}.Success
        minimumDuration(dimensionIndex) = ...
            minimumProfiles{dimensionIndex}.Duration;
    end
end
profile = createEmptyProfile(dimensionCount, minimumDuration);
if any(~isfinite(minimumDuration))
    profile.Message = ...
        "At least one axis has no certified acceleration profile.";
    attempt = createAttempt(profile, solveTimer, ...
        "unsupportedSwitchingFamily");
    return;
end

%% Section 2: Select And Solve The Shared Duration

minimumCommonDuration = max(minimumDuration);
if options.TimeMode == "fixed"
    commonDuration = options.FinalTime;
    if isempty(commonDuration)
        commonDuration = terminalState.maximumTime;
    end
    commonDuration = commonDuration - initialState.time;
    timeTolerance = 256 * eps(max([1, commonDuration, ...
        minimumCommonDuration]));
    if commonDuration < minimumCommonDuration - timeTolerance
        profile.MinimumFinalTime = initialState.time + minimumCommonDuration;
        profile.Message = ...
            "The requested final time is below an independent axis minimum.";
        attempt = createAttempt(profile, solveTimer, ...
            "fixedTimeBelowMinimum");
        return;
    end
else
    commonDuration = selectEarliestCommonDuration( ...
        candidateSets, minimumCommonDuration);
end

axisProfiles = cell(dimensionCount, 1);
for dimensionIndex = 1:dimensionCount
    timeTolerance = 256 * eps(max([1, commonDuration, ...
        minimumDuration(dimensionIndex)]));
    boundaryProfile = findCandidateAtDuration( ...
        candidateSets{dimensionIndex}, commonDuration, timeTolerance);
    if ~isempty(boundaryProfile)
        axisProfiles{dimensionIndex} = boundaryProfile;
    elseif abs(commonDuration - minimumDuration(dimensionIndex)) <= ...
            timeTolerance
        axisProfiles{dimensionIndex} = minimumProfiles{dimensionIndex};
    else
        axisProfiles{dimensionIndex} = createFixedAxisProfile( ...
            initialState, terminalState, limits, ...
            dimensionIndex, commonDuration);
    end
    if ~axisProfiles{dimensionIndex}.Success
        profile.Message = sprintf( ...
            "Axis %d could not be synchronized at %.12g time units.", ...
            dimensionIndex, commonDuration);
        attempt = createAttempt(profile, solveTimer, ...
            "unsupportedSwitchingFamily");
        return;
    end
end

%% Section 3: Create The Common Polynomial

[polynomial, controlAcceleration] = createPolynomial( ...
    initialState, terminalState, axisProfiles, commonDuration);
profile.Success = true;
profile.Message = ...
    "An exact synchronized acceleration-switching profile was created.";
profile.Polynomial = polynomial;
profile.ControlJerk = zeros(size(controlAcceleration));
profile.ControlAcceleration = controlAcceleration;
profile.FinalTime = initialState.time + commonDuration;
profile.MinimumFinalTime = initialState.time + minimumCommonDuration;
profile.IntegratedSquaredJerk = 0;
profile.MinimumAxisDuration = minimumDuration;
profile.AxisFamily = strings(1, dimensionCount);
profile.AxisPathLength = zeros(1, dimensionCount);
for dimensionIndex = 1:dimensionCount
    profile.AxisFamily(dimensionIndex) = axisProfiles{dimensionIndex}.Family;
    profile.AxisPathLength(dimensionIndex) = ...
        axisProfiles{dimensionIndex}.PathLength;
end
attempt = createAttempt(profile, solveTimer, "");
end

%% Section 4: Local Functions

function [profile, candidates] = createMinimumAxisProfile( ...
        initialState, terminalState, limits, dimensionIndex)
% Create all certified second-order Step-1 extremal profiles for one axis.
p0 = initialState.position(dimensionIndex);
pf = terminalState.position(dimensionIndex);
v0 = initialState.velocity(dimensionIndex);
vf = terminalState.velocity(dimensionIndex);
maximumVelocity = limits.maximumVelocity(dimensionIndex);
maximumAcceleration = limits.maximumAcceleration(dimensionIndex);
candidates = repmat(createEmptyAxisProfile(), 0, 1);
for direction = [1, -1]
    vMaximum = direction * maximumVelocity;
    aMaximum = direction * maximumAcceleration;
    aMinimum = -direction * maximumAcceleration;
    positionDifference = pf - p0;

    radical = (aMaximum * vf^2 - aMinimum * v0^2 - ...
        2 * aMaximum * aMinimum * positionDifference) / ...
        (aMaximum - aMinimum);
    if radical >= 0
        root = sqrt(radical);
        candidates = appendAxisCandidate(candidates, p0, v0, pf, vf, ...
            maximumVelocity, maximumAcceleration, ...
            [-(v0 + root) / aMaximum, 0, ...
            (vf + root) / aMinimum], ...
            [aMaximum, 0, aMinimum], direction, "acceleration");
        candidates = appendAxisCandidate(candidates, p0, v0, pf, vf, ...
            maximumVelocity, maximumAcceleration, ...
            [(-v0 + root) / aMaximum, 0, ...
            (vf - root) / aMinimum], ...
            [aMaximum, 0, aMinimum], direction, "acceleration");
    end

    phaseDuration = [ ...
        (-v0 + vMaximum) / aMaximum, ...
        (aMinimum * v0^2 - aMaximum * vf^2) / ...
        (2 * aMaximum * aMinimum * vMaximum) + ...
        vMaximum * (aMaximum - aMinimum) / ...
        (2 * aMaximum * aMinimum) + positionDifference / vMaximum, ...
        (vf - vMaximum) / aMinimum];
    candidates = appendAxisCandidate(candidates, p0, v0, pf, vf, ...
        maximumVelocity, maximumAcceleration, phaseDuration, ...
        [aMaximum, 0, aMinimum], direction, "accelerationVelocity");
end

profile = createEmptyAxisProfile();
if isempty(candidates)
    return;
end
[~, selectedIndex] = min([candidates.Duration]);
profile = candidates(selectedIndex);
end

function profile = createFixedAxisProfile( ...
        initialState, terminalState, limits, dimensionIndex, duration)
% Evaluate every official second-order Step-2 family at one duration.
p0 = initialState.position(dimensionIndex);
pf = terminalState.position(dimensionIndex);
v0 = initialState.velocity(dimensionIndex);
vf = terminalState.velocity(dimensionIndex);
maximumVelocity = limits.maximumVelocity(dimensionIndex);
maximumAcceleration = limits.maximumAcceleration(dimensionIndex);
candidates = repmat(createEmptyAxisProfile(), 0, 1);
positionDifference = pf - p0;
velocityDifference = vf - v0;
for direction = [1, -1]
    aMaximum = direction * maximumAcceleration;
    aMinimum = -direction * maximumAcceleration;

    radical = (2 * aMaximum * (positionDifference - duration * vf) - ...
        2 * aMinimum * (positionDifference - duration * v0) + ...
        velocityDifference^2) / (aMaximum * aMinimum) + duration^2;
    if radical >= 0
        root = sqrt(radical);
        phaseDuration = [ ...
            (aMaximum * velocityDifference - aMaximum * aMinimum * ...
            (duration - root)) / (aMaximum * (aMaximum - aMinimum)), ...
            root, 0];
        phaseDuration(3) = duration - sum(phaseDuration(1:2));
        candidates = appendAxisCandidate(candidates, p0, v0, pf, vf, ...
            maximumVelocity, maximumAcceleration, phaseDuration, ...
            [aMaximum, 0, aMinimum], direction, ...
            "synchronizedAccelerationVelocity");
    end

    denominator = -velocityDifference + aMaximum * duration;
    if denominator ~= 0
        firstDuration = -velocityDifference^2 / ...
            (2 * aMaximum * denominator) + ...
            (positionDifference - v0 * duration) / denominator;
        phaseDuration = [firstDuration, ...
            -velocityDifference / aMaximum + duration, 0, 0, 0, 0, 0];
        phaseDuration(7) = duration - sum(phaseDuration(1:6));
        candidates = appendAxisCandidate(candidates, p0, v0, pf, vf, ...
            maximumVelocity, maximumAcceleration, phaseDuration, ...
            [aMaximum, 0, aMinimum, 0, aMinimum, 0, aMaximum], ...
            direction, "synchronizedSameAcceleration");
    end

    phaseDuration = [0, ...
        -velocityDifference / aMaximum + duration, ...
        0, 0, 0, 0, velocityDifference / aMaximum];
    candidates = appendAxisCandidate(candidates, p0, v0, pf, vf, ...
        maximumVelocity, maximumAcceleration, phaseDuration, ...
        [aMaximum, 0, aMinimum, 0, aMinimum, 0, aMaximum], ...
        direction, "synchronizedTwoStepAcceleration");

    if velocityDifference ~= 0
        firstDuration = 2 * (vf * duration - positionDifference) / ...
            velocityDifference;
        acceleration = velocityDifference^2 / ...
            (2 * (vf * duration - positionDifference));
        if acceleration >= min(aMinimum, aMaximum) - 1e-12 && ...
                acceleration <= max(aMinimum, aMaximum) + 1e-12
            candidates = appendAxisCandidate(candidates, p0, v0, pf, vf, ...
                maximumVelocity, maximumAcceleration, ...
                [firstDuration, duration - firstDuration, 0], ...
                [acceleration, -acceleration, 0], direction, ...
                "synchronizedFreeAcceleration");
        end
    elseif abs(positionDifference - duration * v0) <= ...
            1e-10 * max(1, abs(positionDifference))
        candidates = appendAxisCandidate(candidates, p0, v0, pf, vf, ...
            maximumVelocity, maximumAcceleration, [0, duration, 0], ...
            [0, 0, 0], direction, "constantVelocity");
    end
end

profile = createEmptyAxisProfile();
if isempty(candidates)
    return;
end
[~, selectedIndex] = min([candidates.PathLength]);
profile = candidates(selectedIndex);
end

function candidates = appendAxisCandidate( ...
        candidates, p0, v0, pf, vf, maximumVelocity, ...
        maximumAcceleration, phaseDuration, phaseAcceleration, ...
        direction, family)
% Integrate and retain one complete finite profile within derivative bounds.
phaseDuration = double(phaseDuration(:).');
phaseAcceleration = double(phaseAcceleration(:).');
if numel(phaseDuration) ~= numel(phaseAcceleration) || ...
        any(~isfinite(phaseDuration)) || any(~isfinite(phaseAcceleration))
    return;
end
durationTolerance = 1e-10 * max(1, sum(abs(phaseDuration)));
if any(phaseDuration < -durationTolerance) || ...
        any(abs(phaseAcceleration) > maximumAcceleration + 1e-10)
    return;
end
phaseDuration = max(0, phaseDuration);
position = zeros(1, numel(phaseDuration) + 1);
velocity = position;
position(1) = p0;
velocity(1) = v0;
for phaseIndex = 1:numel(phaseDuration)
    duration = phaseDuration(phaseIndex);
    acceleration = phaseAcceleration(phaseIndex);
    position(phaseIndex + 1) = position(phaseIndex) + ...
        duration * (velocity(phaseIndex) + duration * acceleration / 2);
    velocity(phaseIndex + 1) = ...
        velocity(phaseIndex) + duration * acceleration;
end
positionTolerance = 1e-8 * max(1, max(abs([p0, pf])));
velocityTolerance = 1e-8 * max(1, maximumVelocity);
if abs(position(end) - pf) > positionTolerance || ...
        abs(velocity(end) - vf) > velocityTolerance || ...
        any(abs(velocity) > maximumVelocity + velocityTolerance)
    return;
end
candidate = createEmptyAxisProfile();
candidate.Success = true;
candidate.PhaseDuration = phaseDuration;
candidate.PhaseAcceleration = phaseAcceleration;
candidate.Position = position;
candidate.Velocity = velocity;
candidate.Acceleration = [phaseAcceleration, 0];
candidate.Duration = sum(phaseDuration);
candidate.Direction = direction;
candidate.Family = family;
candidate.PathLength = sum(abs(diff(position)));
candidates(end + 1, 1) = candidate;
end

function duration = selectEarliestCommonDuration(candidateSets, minimumDuration)
% Advance only to certified extremal boundaries when a minimum is blocked.
possibleDuration = minimumDuration;
for dimensionIndex = 1:numel(candidateSets)
    candidates = candidateSets{dimensionIndex};
    possibleDuration = [possibleDuration, [candidates.Duration]]; %#ok<AGROW>
end
possibleDuration = unique(sort(possibleDuration));
duration = minimumDuration;
for candidateDuration = possibleDuration(possibleDuration >= minimumDuration)
    allAxesFeasible = true;
    for dimensionIndex = 1:numel(candidateSets)
        candidates = candidateSets{dimensionIndex};
        shorterDuration = sort([candidates.Duration]);
        if numel(shorterDuration) >= 3 && ...
                candidateDuration > shorterDuration(2) && ...
                candidateDuration < shorterDuration(3)
            allAxesFeasible = false;
            break;
        end
    end
    if allAxesFeasible
        duration = candidateDuration;
        return;
    end
end
end

function profile = findCandidateAtDuration(candidates, duration, tolerance)
% Reuse an exact extremal profile at a synchronization boundary.
profile = [];
for candidateIndex = 1:numel(candidates)
    if abs(candidates(candidateIndex).Duration - duration) <= tolerance
        profile = candidates(candidateIndex);
        return;
    end
end
end

function [polynomial, controlAcceleration] = createPolynomial( ...
        initialState, terminalState, axisProfiles, commonDuration)
% Assemble every axis acceleration switch on one exact physical clock.
dimensionCount = numel(axisProfiles);
switchTime = [0, commonDuration];
for dimensionIndex = 1:dimensionCount
    axisTime = cumsum(axisProfiles{dimensionIndex}.PhaseDuration);
    axisTime(end) = commonDuration;
    switchTime = [switchTime, axisTime]; %#ok<AGROW>
end
switchTime = unique(sort(switchTime));
timeTolerance = 512 * eps(max(1, commonDuration));
switchTime = switchTime([true, diff(switchTime) > timeTolerance]);
switchTime(1) = 0;
switchTime(end) = commonDuration;
segmentDuration = diff(switchTime).';
segmentStartTime = initialState.time + switchTime(1:end - 1).';
segmentCount = numel(segmentDuration);
controlAcceleration = zeros(segmentCount, dimensionCount);
for dimensionIndex = 1:dimensionCount
    axisEndTime = cumsum(axisProfiles{dimensionIndex}.PhaseDuration);
    axisAcceleration = axisProfiles{dimensionIndex}.PhaseAcceleration;
    for segmentIndex = 1:segmentCount
        middleTime = 0.5 * ...
            (switchTime(segmentIndex) + switchTime(segmentIndex + 1));
        phaseIndex = find(middleTime < axisEndTime + 1e-12, 1);
        controlAcceleration(segmentIndex, dimensionIndex) = ...
            axisAcceleration(phaseIndex);
    end
end

positionPower = zeros(segmentCount, dimensionCount, 6);
velocityPower = zeros(segmentCount, dimensionCount, 5);
accelerationPower = zeros(segmentCount, dimensionCount, 4);
jerkPower = zeros(segmentCount, dimensionCount, 3);
position = initialState.position;
velocity = initialState.velocity;
for segmentIndex = 1:segmentCount
    duration = segmentDuration(segmentIndex);
    acceleration = controlAcceleration(segmentIndex, :);
    positionCoefficient = [position; velocity * duration; ...
        acceleration * duration^2 / 2; zeros(3, dimensionCount)].';
    velocityCoefficient = [velocity; acceleration * duration; ...
        zeros(3, dimensionCount)].';
    positionPower(segmentIndex, :, :) = positionCoefficient;
    velocityPower(segmentIndex, :, :) = velocityCoefficient;
    accelerationPower(segmentIndex, :, 1) = acceleration;
    position = sum(positionCoefficient, 2).';
    velocity = sum(velocityCoefficient, 2).';
end
polynomial = struct( ...
    "SegmentCount", segmentCount, ...
    "SegmentStartTime", segmentStartTime, ...
    "SegmentDuration", segmentDuration, ...
    "FinalTime", initialState.time + commonDuration, ...
    "positionPower", positionPower, ...
    "velocityPower", velocityPower, ...
    "accelerationPower", accelerationPower, ...
    "jerkPower", jerkPower, ...
    "TerminalState", struct( ...
    "position", position, "velocity", velocity, ...
    "acceleration", terminalState.acceleration));
end

function attempt = createAttempt(profile, solveTimer, reason)
% Assemble the same stage contract used by the third-order engine path.
success = profile.Success;
message = string(profile.Message);
if success
    reason = "";
end
attempt = struct( ...
    "Success", success, ...
    "Message", message, ...
    "TerminationReason", string(reason), ...
    "RequestedFinalTime", [], ...
    "Profile", profile, ...
    "ElapsedTime", toc(solveTimer), ...
    "UsedDirectProgress", false);
end

function profile = createEmptyAxisProfile()
% Define one stable scalar acceleration-profile record.
profile = struct( ...
    "Success", false, ...
    "PhaseDuration", zeros(1, 0), ...
    "PhaseAcceleration", zeros(1, 0), ...
    "Position", zeros(1, 0), ...
    "Velocity", zeros(1, 0), ...
    "Acceleration", zeros(1, 0), ...
    "Duration", NaN, ...
    "Direction", 0, ...
    "Family", "", ...
    "PathLength", Inf);
end

function profile = createEmptyProfile(dimensionCount, minimumDuration)
% Define the synchronized second-order profile contract on every exit.
profile = struct( ...
    "Success", false, ...
    "Message", "No synchronized acceleration profile was created.", ...
    "Polynomial", struct(), ...
    "ControlJerk", zeros(0, dimensionCount), ...
    "ControlAcceleration", zeros(0, dimensionCount), ...
    "FinalTime", NaN, ...
    "MinimumFinalTime", NaN, ...
    "IntegratedSquaredJerk", 0, ...
    "MinimumAxisDuration", minimumDuration, ...
    "AxisPathLength", NaN(1, dimensionCount), ...
    "AxisFamily", strings(1, dimensionCount));
end
