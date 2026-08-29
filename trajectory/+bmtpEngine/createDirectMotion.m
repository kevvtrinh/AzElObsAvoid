function candidate = createDirectMotion( ...
        initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   candidate = bmtpEngine.createDirectMotion( ...
%       initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Create an exact minimum-time synchronized rest-to-rest trajectory.
%   - Use analytic scalar jerk-switching laws and lossless time scaling.
%**************************************************************************
% INPUTS
%   - initialState, goalState (scalar structs)
%       Require scalar time_s and one-by-D position_deg. Omitted velocity
%       and acceleration fields mean zero.
%   - limits (scalar struct)
%       Require positive one-by-D maxVelocity_deg_s,
%       maxAcceleration_deg_s2, and maxJerk_deg_s3.
%   - options (scalar struct)
%       Require GoalTimeMode and positive SampleTime_s. An optional positive
%       ConstraintTolerance controls endpoint-rest acceptance.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       Stable success-or-failure record containing an exact piecewise
%       constant-jerk Polynomial compatible with the public validator.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s,
%     deg/s^2, and deg/s^3. Histories are N-by-D.
%**************************************************************************

%% Section 1: Normalize The Rest-To-Rest Request

if nargin ~= 4
    error("createDirectMotion:InvalidCall", ...
        "initialState, goalState, limits, and options are required.");
end
[initialTime_s, initialPosition_deg, initialVelocity_deg_s, ...
    initialAcceleration_deg_s2] = readState(initialState, "initialState", []);
dimensionCount = numel(initialPosition_deg);
[goalTime_s, goalPosition_deg, goalVelocity_deg_s, ...
    goalAcceleration_deg_s2] = readState(goalState, "goalState", dimensionCount);
if goalTime_s <= initialTime_s
    error("createDirectMotion:InvalidTimeHorizon", ...
        "goalState.time_s must be later than initialState.time_s.");
end
[maximumVelocity_deg_s, maximumAcceleration_deg_s2, ...
    maximumJerk_deg_s3] = readLimits(limits, dimensionCount);
[goalTimeMode, sampleStep_s, constraintTolerance] = readOptions(options);
motionInitialState = struct("time_s", initialTime_s, ...
    "position_deg", initialPosition_deg, ...
    "velocity_deg_s", initialVelocity_deg_s, ...
    "acceleration_deg_s2", initialAcceleration_deg_s2);
candidate = bmtpEngine.createMotionRecord( ...
    struct(), motionInitialState, [], [], sampleStep_s, "directRestToRest");
candidate.Message = "The compact direct kernel was not run.";
candidate.Polynomial = emptyPolynomial(dimensionCount);
endpointDerivative = [initialVelocity_deg_s, goalVelocity_deg_s, ...
    initialAcceleration_deg_s2, goalAcceleration_deg_s2];
if max(abs(endpointDerivative)) > constraintTolerance
    candidate.Message = ...
        "The compact direct kernel supports rest-to-rest endpoints only.";
    candidate.TerminationReason = "unsupportedEndpointDerivatives";
    return;
end

%% Section 2: Create And Synchronize Exact Scalar Profiles

displacement_deg = goalPosition_deg - initialPosition_deg;
phaseDuration_s = zeros(dimensionCount, 7);
phaseJerk_deg_s3 = zeros(dimensionCount, 7);
minimumAxisDuration_s = zeros(1, dimensionCount);
for axisIndex = 1:dimensionCount
    [phaseDuration_s(axisIndex, :), phaseJerk_deg_s3(axisIndex, :)] = ...
        minimumProfile(displacement_deg(axisIndex), ...
        maximumVelocity_deg_s(axisIndex), ...
        maximumAcceleration_deg_s2(axisIndex), ...
        maximumJerk_deg_s3(axisIndex));
    minimumAxisDuration_s(axisIndex) = sum(phaseDuration_s(axisIndex, :));
end
minimumDuration_s = max(minimumAxisDuration_s);
if minimumDuration_s <= 0
    candidate.Message = "Coincident rest endpoints require no positive-time slew.";
    candidate.TerminationReason = "zeroDisplacement";
    return;
end
requestedDuration_s = goalTime_s - initialTime_s;
duration_s = minimumDuration_s;
if goalTimeMode == "fixedArrival"
    duration_s = requestedDuration_s;
    if duration_s < minimumDuration_s - constraintTolerance
        candidate.Message = sprintf( ...
            "Requested duration %.12g s is below the exact %.12g s minimum.", ...
            duration_s, minimumDuration_s);
        candidate.TerminationReason = "fixedTimeBelowMinimum";
        candidate.MinimumAxisDuration_s = minimumAxisDuration_s;
        return;
    end
elseif duration_s > requestedDuration_s + constraintTolerance
    candidate.Message = "The exact direct minimum exceeds the arrival horizon.";
    candidate.TerminationReason = "infeasibleTimeHorizon";
    candidate.MinimumAxisDuration_s = minimumAxisDuration_s;
    return;
end

% A shared progress law preserves the straight path when that law can attain
% the common physical clock. Otherwise, stretch each exact axis minimum.
activeAxis = displacement_deg ~= 0;
[progressPhase_s, progressJerk_1_s3] = minimumProfile(1, ...
    min(maximumVelocity_deg_s(activeAxis) ./ abs(displacement_deg(activeAxis))), ...
    min(maximumAcceleration_deg_s2(activeAxis) ./ abs(displacement_deg(activeAxis))), ...
    min(maximumJerk_deg_s3(activeAxis) ./ abs(displacement_deg(activeAxis))));
straightMinimumDuration_s = sum(progressPhase_s);
usedStraightProgress = straightMinimumDuration_s <= ...
    duration_s + 256 * eps(max(1, duration_s));
if usedStraightProgress
    [progressPhase_s, progressJerk_1_s3] = ...
        stretchProfile(progressPhase_s, progressJerk_1_s3, duration_s);
    phaseDuration_s = repmat(progressPhase_s, dimensionCount, 1);
    phaseJerk_deg_s3 = displacement_deg(:) * progressJerk_1_s3;
else
    for axisIndex = 1:dimensionCount
        if minimumAxisDuration_s(axisIndex) == 0
            phaseDuration_s(axisIndex, 4) = duration_s;
        else
            [phaseDuration_s(axisIndex, :), phaseJerk_deg_s3(axisIndex, :)] = ...
                stretchProfile(phaseDuration_s(axisIndex, :), ...
                phaseJerk_deg_s3(axisIndex, :), duration_s);
        end
    end
end

%% Section 3: Reconstruct, Sample, And Assemble The Candidate

[relativeBreak_s, segmentJerk_deg_s3] = mergeProfiles( ...
    phaseDuration_s, phaseJerk_deg_s3, duration_s);
[candidate, terminalState] = ...
    bmtpEngine.createMotionRecord(candidate, ...
    motionInitialState, relativeBreak_s, segmentJerk_deg_s3, ...
    sampleStep_s, "directRestToRest");
endpointError = max(abs([terminalState.position_deg - goalPosition_deg, ...
    terminalState.velocity_deg_s, terminalState.acceleration_deg_s2]));
endpointTolerance = 1e-9 * max( ...
    [1, abs(initialPosition_deg), abs(goalPosition_deg)], [], "all");
if endpointError > endpointTolerance
    candidate.Message = sprintf( ...
        "Analytic reconstruction endpoint error %.9g exceeds %.9g.", ...
        endpointError, endpointTolerance);
    candidate.TerminationReason = "analyticReconstructionFailed";
    candidate.MaximumConstraintViolation = endpointError;
    return;
end
candidate.Success = true;
candidate.OptimizerFeasible = true;
candidate.Message = "An exact synchronized rest-to-rest profile was created.";
candidate.TerminationReason = "goalReached";
candidate.MinimumAxisDuration_s = minimumAxisDuration_s;
candidate.StraightProgressMinimumDuration_s = straightMinimumDuration_s;
candidate.UsedStraightProgress = usedStraightProgress;
candidate.MaximumConstraintViolation = endpointError;
end

%% Section 4: Local Functions

function [time_s, position_deg, velocity_deg_s, acceleration_deg_s2] = ...
        readState(state, stateName, dimensionCount)
% Normalize one finite state and resolve omitted derivatives to zero.
requiredFields = {'time_s', 'position_deg'};
if ~isstruct(state) || ~isscalar(state) || ...
        ~all(isfield(state, requiredFields))
    error("createDirectMotion:InvalidState", ...
        "%s requires scalar time_s and one-by-D position_deg.", stateName);
end
validateattributes(state.time_s, {'numeric'}, ...
    {'real', 'finite', 'scalar'}, mfilename, stateName + ".time_s");
validateattributes(state.position_deg, {'numeric'}, ...
    {'real', 'finite', 'vector', 'nonempty'}, mfilename, ...
    stateName + ".position_deg");
time_s = double(state.time_s);
position_deg = double(state.position_deg(:).');
if isempty(dimensionCount)
    dimensionCount = numel(position_deg);
elseif numel(position_deg) ~= dimensionCount
        error("createDirectMotion:DimensionMismatch", ...
        "Initial and goal positions must have the same dimension.");
end
velocity_deg_s = readDerivative(state, "velocity_deg_s", dimensionCount);
acceleration_deg_s2 = readDerivative( ...
    state, "acceleration_deg_s2", dimensionCount);
end

function value = readDerivative(state, fieldName, dimensionCount)
% Read an optional finite endpoint derivative with the required dimension.
value = zeros(1, dimensionCount);
if isfield(state, fieldName) && ~isempty(state.(fieldName))
    validateattributes(state.(fieldName), {'numeric'}, ...
        {'real', 'finite', 'vector', 'nonempty'}, mfilename, fieldName);
    value = double(state.(fieldName)(:).');
    if numel(value) ~= dimensionCount
        error("createDirectMotion:DerivativeDimensionMismatch", ...
            "%s must contain %d values.", fieldName, dimensionCount);
    end
end
end

function [maximumVelocity_deg_s, maximumAcceleration_deg_s2, ...
        maximumJerk_deg_s3] = readLimits(limits, dimensionCount)
% Read all positive per-axis physical limits without relaxing dimensions.
names = ["maxVelocity_deg_s", "maxAcceleration_deg_s2", "maxJerk_deg_s3"];
values = zeros(3, dimensionCount);
if ~isstruct(limits) || ~isscalar(limits) || ~all(isfield(limits, names))
    error("createDirectMotion:MissingLimit", ...
        "limits requires positive per-axis velocity, acceleration, and jerk.");
end
for limitIndex = 1:3
    value = limits.(names(limitIndex));
    validateattributes(value, {'numeric'}, ...
        {'real', 'finite', 'vector', 'nonempty'}, mfilename, ...
        "limits." + names(limitIndex));
    value = double(value(:).');
    if numel(value) ~= dimensionCount || any(value <= 0)
        error("createDirectMotion:InvalidLimit", ...
            "limits.%s must contain %d positive values.", ...
            names(limitIndex), dimensionCount);
    end
    values(limitIndex, :) = value;
end
maximumVelocity_deg_s = values(1, :);
maximumAcceleration_deg_s2 = values(2, :);
maximumJerk_deg_s3 = values(3, :);
end

function [goalTimeMode, sampleTime_s, tolerance] = readOptions(options)
% Normalize the two required options and optional endpoint-rest tolerance.
if ~isstruct(options) || ~isscalar(options) || ...
        ~all(isfield(options, {'GoalTimeMode', 'SampleTime_s'}))
    error("createDirectMotion:InvalidOptions", ...
        "options requires GoalTimeMode and SampleTime_s.");
end
goalTimeMode = string(options.GoalTimeMode);
if ~isscalar(goalTimeMode) || ...
        ~any(goalTimeMode == ["earliestArrival", "fixedArrival"])
    error("createDirectMotion:InvalidGoalTimeMode", ...
        "GoalTimeMode must be earliestArrival or fixedArrival.");
end
validateattributes(options.SampleTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'}, mfilename, "SampleTime_s");
sampleTime_s = double(options.SampleTime_s);
tolerance = 1e-7;
if isfield(options, "ConstraintTolerance") && ...
        ~isempty(options.ConstraintTolerance)
    validateattributes(options.ConstraintTolerance, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'positive'}, mfilename, ...
        "ConstraintTolerance");
    tolerance = double(options.ConstraintTolerance);
end
end

function [duration_s, jerk_deg_s3] = minimumProfile( ...
        displacement_deg, velocityLimit_deg_s, ...
        accelerationLimit_deg_s2, jerkLimit_deg_s3)
% Return the exact symmetric seven-phase scalar minimum-time law.
distance_deg = abs(displacement_deg);
duration_s = zeros(1, 7);
jerk_deg_s3 = zeros(1, 7);
if distance_deg == 0
    return;
end
ramp_s = accelerationLimit_deg_s2 / jerkLimit_deg_s3;
if 2 * accelerationLimit_deg_s2 ^ 3 / jerkLimit_deg_s3 ^ 2 >= distance_deg
    ramp_s = nthroot(distance_deg / (2 * jerkLimit_deg_s3), 3);
    plateau_s = 0;
else
    plateau_s = 0.5 * (sqrt(ramp_s ^ 2 + ...
        4 * distance_deg / accelerationLimit_deg_s2) - 3 * ramp_s);
end
peakVelocity_deg_s = jerkLimit_deg_s3 * ramp_s * (ramp_s + plateau_s);
cruise_s = 0;
if peakVelocity_deg_s > velocityLimit_deg_s
    if velocityLimit_deg_s <= accelerationLimit_deg_s2 ^ 2 / jerkLimit_deg_s3
        ramp_s = sqrt(velocityLimit_deg_s / jerkLimit_deg_s3);
        plateau_s = 0;
    else
        ramp_s = accelerationLimit_deg_s2 / jerkLimit_deg_s3;
        plateau_s = velocityLimit_deg_s / accelerationLimit_deg_s2 - ramp_s;
    end
    minimumDistance_deg = velocityLimit_deg_s * (2 * ramp_s + plateau_s);
    cruise_s = (distance_deg - minimumDistance_deg) / velocityLimit_deg_s;
end
duration_s = [ramp_s, plateau_s, ramp_s, max(0, cruise_s), ...
    ramp_s, plateau_s, ramp_s];
jerk_deg_s3 = sign(displacement_deg) * jerkLimit_deg_s3 * ...
    [1, 0, -1, 0, -1, 0, 1];
end

function [duration_s, jerk] = stretchProfile(duration_s, jerk, targetDuration_s)
% Uniform time scaling preserves endpoints while reducing every limit peak.
sourceDuration_s = sum(duration_s);
scale = targetDuration_s / sourceDuration_s;
cumulativeFraction = cumsum(duration_s / sourceDuration_s);
cumulativeFraction(end) = 1;
duration_s = diff([0, targetDuration_s * cumulativeFraction]);
jerk = jerk / scale ^ 3;
end

function [relativeBreak_s, segmentJerk_deg_s3] = mergeProfiles( ...
        phaseDuration_s, phaseJerk_deg_s3, duration_s)
% Merge per-axis switching events without changing the physical clock.
dimensionCount = size(phaseDuration_s, 1);
axisBreak_s = [zeros(dimensionCount, 1), cumsum(phaseDuration_s, 2)];
axisBreak_s(:, end) = duration_s;
relativeBreak_s = mergeBreaks([0; duration_s; axisBreak_s(:)], ...
    duration_s, 1024 * eps(max(1, duration_s)));
segmentCount = numel(relativeBreak_s) - 1;
segmentJerk_deg_s3 = zeros(segmentCount, dimensionCount);
for segmentIndex = 1:segmentCount
    midpoint_s = 0.5 * sum(relativeBreak_s(segmentIndex:segmentIndex + 1));
    for axisIndex = 1:dimensionCount
        phaseIndex = find(midpoint_s < axisBreak_s(axisIndex, 2:end), 1);
        if isempty(phaseIndex)
            phaseIndex = 7;
        end
        segmentJerk_deg_s3(segmentIndex, axisIndex) = ...
            phaseJerk_deg_s3(axisIndex, phaseIndex);
    end
end
end

function merged_s = mergeBreaks(values_s, duration_s, tolerance_s)
% Merge floating-point duplicate switches without creating tiny segments.
values_s = sort(min(duration_s, max(0, double(values_s(:)))));
merged_s = zeros(size(values_s));
mergedCount = 0;
for valueIndex = 1:numel(values_s)
    if mergedCount == 0 || ...
            values_s(valueIndex) - merged_s(mergedCount) > tolerance_s
        mergedCount = mergedCount + 1;
    end
    merged_s(mergedCount) = values_s(valueIndex);
end
merged_s = merged_s(1:mergedCount);
merged_s([1, end]) = [0; duration_s];
end

function polynomial = emptyPolynomial(dimensionCount)
% Preserve polynomial field shape on an expected construction failure.
polynomial = struct("Degree", 3, "SegmentCount", 0, ...
    "SegmentStartTime_s", zeros(0, 1), "SegmentDuration_s", zeros(0, 1), ...
    "SegmentBreakTau", zeros(0, 1), "FinalTime_s", NaN, ...
    "positionPower_deg", zeros(0, dimensionCount, 4), ...
    "velocityPower_deg_s", zeros(0, dimensionCount, 3), ...
    "accelerationPower_deg_s2", zeros(0, dimensionCount, 2), ...
    "jerkPower_deg_s3", zeros(0, dimensionCount, 1), ...
    "TerminalState", struct());
end
