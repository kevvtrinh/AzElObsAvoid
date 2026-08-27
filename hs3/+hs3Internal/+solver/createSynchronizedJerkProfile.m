function profile = createSynchronizedJerkProfile( ...
        initialState, terminalState, limits, requestedFinalTime)
%% Section 0: Header & Readme
% SYNTAX
%   profile = hs3Internal.solver.createSynchronizedJerkProfile( ...
%       initialState, terminalState, limits, requestedFinalTime)
%**************************************************************************
% PURPOSE
%   - Create a dimension-neutral trajectory at the maximum independent
%     minimum axis time and synchronize faster axes without delaying arrival.
%**************************************************************************
% INPUTS
%   - initialState (scalar struct)
%       Scalar time and row-vector position, velocity, and acceleration.
%   - terminalState (scalar struct)
%       Row-vector position, velocity, and acceleration.
%   - limits (scalar struct)
%       Row-vector maximumVelocity, maximumAcceleration, and maximumJerk.
%   - requestedFinalTime (empty or finite scalar)
%       Empty selects minimum arrival. A supplied time requests fixed arrival.
%**************************************************************************
% OUTPUTS
%   - profile (scalar struct)
%       Success, message, variable-duration polynomial, phase jerk controls,
%       arrival time, independent axis minima, and path-length evidence.
%**************************************************************************
% UNITS
%   - Time and coordinate units are caller-defined and must be consistent.
%**************************************************************************

%% Section 1: Create Independent Minimum-Time Axis Profiles

dimensionCount = numel(initialState.position);
minimumProfiles = cell(dimensionCount, 1);
minimumDuration = NaN(1, dimensionCount);
for dimensionIndex = 1:dimensionCount
    [axisInitialState, axisTerminalState, axisLimits] = ...
        extractAxisProblem(initialState, terminalState, limits, dimensionIndex);
    minimumProfiles{dimensionIndex} = ...
        hs3Internal.solver.createMinimumTimeAxisProfile( ...
        axisInitialState, axisTerminalState, axisLimits);
    if minimumProfiles{dimensionIndex}.Success
        minimumDuration(dimensionIndex) = ...
            minimumProfiles{dimensionIndex}.Duration;
    end
end

profile = createEmptyProfile(dimensionCount, minimumDuration);
if any(~isfinite(minimumDuration))
    profile.Message = ...
        "At least one axis has no certified exact minimum-time profile.";
    return;
end

minimumCommonDuration = max(minimumDuration);
if isempty(requestedFinalTime)
    commonDuration = minimumCommonDuration;
else
    commonDuration = requestedFinalTime - initialState.time;
    timeTolerance = 256 * eps(max([1, commonDuration, minimumCommonDuration]));
    if commonDuration < minimumCommonDuration - timeTolerance
        profile.Message = ...
            "The requested final time is below an independent axis minimum.";
        return;
    end
end

%% Section 2: Synchronize Faster Axes At The Common Duration

axisProfiles = cell(dimensionCount, 1);
axisCandidateSets = cell(dimensionCount, 1);
for dimensionIndex = 1:dimensionCount
    timeTolerance = 256 * eps(max([ ...
        1, commonDuration, minimumDuration(dimensionIndex)]));
    if abs(minimumDuration(dimensionIndex) - commonDuration) <= timeTolerance
        axisProfiles{dimensionIndex} = minimumProfiles{dimensionIndex};
        axisCandidateSets{dimensionIndex} = minimumProfiles(dimensionIndex);
        continue;
    end
    [axisInitialState, axisTerminalState, axisLimits] = ...
        extractAxisProblem(initialState, terminalState, limits, dimensionIndex);
    [axisProfiles{dimensionIndex}, fixedCandidates] = ...
        hs3Internal.solver.createFixedTimeAxisProfile( ...
        axisInitialState, axisTerminalState, axisLimits, commonDuration);
    if ~axisProfiles{dimensionIndex}.Success
        profile.Message = sprintf( ...
            "Axis %d could not be synchronized at %.12g time units.", ...
            dimensionIndex, commonDuration);
        return;
    end
    axisCandidateSets{dimensionIndex} = num2cell(fixedCandidates);
end
axisProfiles = selectSpatiallyShortestProfiles( ...
    axisProfiles, axisCandidateSets, commonDuration);

%% Section 3: Create One Multidimensional Polynomial

switchTime = [0, commonDuration];
for dimensionIndex = 1:dimensionCount
    axisTime = cumsum(axisProfiles{dimensionIndex}.PhaseDuration);
    axisTime(end) = commonDuration;
    switchTime = [switchTime, axisTime]; %#ok<AGROW>
end
switchTime = unique(round(switchTime, 12));
switchTolerance = 1e-11 * max(1, commonDuration);
switchTime = switchTime(switchTime >= -switchTolerance & ...
    switchTime <= commonDuration + switchTolerance);
switchTime(1) = 0;
switchTime(end) = commonDuration;
segmentDuration = diff(switchTime).';
segmentStartTime = initialState.time + switchTime(1:end - 1).';
segmentCount = numel(segmentDuration);
controlJerk = zeros(segmentCount, dimensionCount);

for dimensionIndex = 1:dimensionCount
    axisDuration = axisProfiles{dimensionIndex}.PhaseDuration;
    axisEndTime = cumsum(axisDuration);
    axisJerk = axisProfiles{dimensionIndex}.PhaseJerk;
    for segmentIndex = 1:segmentCount
        segmentMiddleTime = 0.5 * ( ...
            switchTime(segmentIndex) + switchTime(segmentIndex + 1));
        phaseIndex = find(segmentMiddleTime < axisEndTime + 1e-12, 1);
        if isempty(phaseIndex)
            phaseIndex = numel(axisJerk);
        end
        controlJerk(segmentIndex, dimensionIndex) = axisJerk(phaseIndex);
    end
end

[polynomial, terminalPosition, terminalVelocity, terminalAcceleration] = ...
    createPolynomial(initialState, segmentStartTime, ...
    segmentDuration, controlJerk);
polynomial.TerminalState = struct( ...
    "position", terminalPosition, ...
    "velocity", terminalVelocity, ...
    "acceleration", terminalAcceleration);

%% Section 4: Assemble The Synchronized Profile

profile.Success = true;
profile.Message = "An exact synchronized jerk-switching profile was created.";
profile.Polynomial = polynomial;
profile.ControlJerk = controlJerk;
profile.FinalTime = initialState.time + commonDuration;
profile.MinimumFinalTime = initialState.time + minimumCommonDuration;
profile.IntegratedSquaredJerk = sum(sum( ...
    controlJerk .^ 2 .* segmentDuration));
profile.MinimumAxisDuration = minimumDuration;
profile.AxisPathLength = zeros(1, dimensionCount);
profile.AxisFamily = strings(1, dimensionCount);
for dimensionIndex = 1:dimensionCount
    profile.AxisPathLength(dimensionIndex) = ...
        axisProfiles{dimensionIndex}.PathLength;
    profile.AxisFamily(dimensionIndex) = axisProfiles{dimensionIndex}.Family;
end
end

%% Section 5: Local Functions

function [axisInitialState, axisTerminalState, axisLimits] = ...
        extractAxisProblem(initialState, terminalState, limits, dimensionIndex)
% Isolate one coordinate without changing its units or boundary meaning.
axisInitialState = struct( ...
    "time", initialState.time, ...
    "position", initialState.position(dimensionIndex), ...
    "velocity", initialState.velocity(dimensionIndex), ...
    "acceleration", initialState.acceleration(dimensionIndex));
axisTerminalState = struct( ...
    "position", terminalState.position(dimensionIndex), ...
    "velocity", terminalState.velocity(dimensionIndex), ...
    "acceleration", terminalState.acceleration(dimensionIndex));
axisLimits = struct( ...
    "maximumVelocity", limits.maximumVelocity(dimensionIndex), ...
    "maximumAcceleration", limits.maximumAcceleration(dimensionIndex), ...
    "maximumJerk", limits.maximumJerk(dimensionIndex));
end

function [polynomial, position, velocity, acceleration] = createPolynomial( ...
        initialState, segmentStartTime, segmentDuration, controlJerk)
% Integrate the union of every axis switch time into the HS3 polynomial schema.
segmentCount = numel(segmentDuration);
dimensionCount = numel(initialState.position);
positionPower = zeros(segmentCount, dimensionCount, 6);
velocityPower = zeros(segmentCount, dimensionCount, 5);
accelerationPower = zeros(segmentCount, dimensionCount, 4);
jerkPower = zeros(segmentCount, dimensionCount, 3);
position = initialState.position;
velocity = initialState.velocity;
acceleration = initialState.acceleration;

for segmentIndex = 1:segmentCount
    duration = segmentDuration(segmentIndex);
    jerk = controlJerk(segmentIndex, :);
    positionCoefficient = [ ...
        position; velocity * duration; ...
        0.5 * acceleration * duration^2; jerk * duration^3 / 6; ...
        zeros(2, dimensionCount)].';
    velocityCoefficient = [ ...
        velocity; acceleration * duration; ...
        0.5 * jerk * duration^2; zeros(2, dimensionCount)].';
    accelerationCoefficient = [ ...
        acceleration; jerk * duration; zeros(2, dimensionCount)].';
    positionPower(segmentIndex, :, :) = positionCoefficient;
    velocityPower(segmentIndex, :, :) = velocityCoefficient;
    accelerationPower(segmentIndex, :, :) = accelerationCoefficient;
    jerkPower(segmentIndex, :, :) = [jerk; zeros(2, dimensionCount)].';
    position = sum(positionCoefficient, 2).';
    velocity = sum(velocityCoefficient, 2).';
    acceleration = sum(accelerationCoefficient, 2).';
end

polynomial = struct( ...
    "SegmentCount", segmentCount, ...
    "SegmentStartTime", segmentStartTime, ...
    "SegmentDuration", segmentDuration, ...
    "FinalTime", segmentStartTime(1) + sum(segmentDuration), ...
    "positionPower", positionPower, ...
    "velocityPower", velocityPower, ...
    "accelerationPower", accelerationPower, ...
    "jerkPower", jerkPower, ...
    "TerminalState", struct());
end

function selectedProfiles = selectSpatiallyShortestProfiles( ...
        selectedProfiles, candidateSets, commonDuration)
% Coordinate descent chooses synchronized families by multidimensional path length.
dimensionCount = numel(selectedProfiles);
for sweepIndex = 1:2
    selectionChanged = false;
    for dimensionIndex = 1:dimensionCount
        candidates = candidateSets{dimensionIndex};
        if numel(candidates) <= 1
            continue;
        end
        bestCandidate = selectedProfiles{dimensionIndex};
        bestLength = sampledSpatialPathLength( ...
            selectedProfiles, commonDuration);
        for candidateIndex = 1:numel(candidates)
            trialProfiles = selectedProfiles;
            trialProfiles{dimensionIndex} = candidates{candidateIndex};
            trialLength = sampledSpatialPathLength( ...
                trialProfiles, commonDuration);
            comparisonTolerance = 1e-10 * max(1, bestLength);
            if trialLength < bestLength - comparisonTolerance
                bestCandidate = candidates{candidateIndex};
                bestLength = trialLength;
            end
        end
        if bestCandidate.Family ~= selectedProfiles{dimensionIndex}.Family
            selectionChanged = true;
        end
        selectedProfiles{dimensionIndex} = bestCandidate;
    end
    if ~selectionChanged
        break;
    end
end
end

function lengthValue = sampledSpatialPathLength(axisProfiles, commonDuration)
% Rank profile combinations on one shared dense time base without changing feasibility.
sampleTime = linspace(0, commonDuration, 1001).';
dimensionCount = numel(axisProfiles);
position = zeros(numel(sampleTime), dimensionCount);
for dimensionIndex = 1:dimensionCount
    axisProfile = axisProfiles{dimensionIndex};
    phaseStart = [0, cumsum(axisProfile.PhaseDuration(1:end - 1))];
    phaseEnd = cumsum(axisProfile.PhaseDuration);
    for phaseIndex = 1:numel(axisProfile.PhaseDuration)
        if phaseIndex == numel(axisProfile.PhaseDuration)
            isInPhase = sampleTime >= phaseStart(phaseIndex) & ...
                sampleTime <= phaseEnd(phaseIndex) + 1e-11;
        else
            isInPhase = sampleTime >= phaseStart(phaseIndex) & ...
                sampleTime < phaseEnd(phaseIndex);
        end
        localTime = sampleTime(isInPhase) - phaseStart(phaseIndex);
        position(isInPhase, dimensionIndex) = ...
            axisProfile.Position(phaseIndex) + localTime .* ...
            (axisProfile.Velocity(phaseIndex) + localTime .* ...
            (axisProfile.Acceleration(phaseIndex) / 2 + ...
            localTime * axisProfile.PhaseJerk(phaseIndex) / 6));
    end
end
lengthValue = sum(vecnorm(diff(position, 1, 1), 2, 2));
end

function profile = createEmptyProfile(dimensionCount, minimumDuration)
% Define stable fields for an accepted profile or an optimizer fallback.
profile = struct( ...
    "Success", false, ...
    "Message", "No synchronized jerk-switching profile was created.", ...
    "Polynomial", struct(), ...
    "ControlJerk", zeros(0, dimensionCount), ...
    "FinalTime", NaN, ...
    "MinimumFinalTime", NaN, ...
    "IntegratedSquaredJerk", Inf, ...
    "MinimumAxisDuration", minimumDuration, ...
    "AxisPathLength", NaN(1, dimensionCount), ...
    "AxisFamily", strings(1, dimensionCount));
end
