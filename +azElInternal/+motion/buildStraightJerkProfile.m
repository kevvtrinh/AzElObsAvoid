function profile = buildStraightJerkProfile(route_deg, initialState, limits)
%% Section 0: Header & Readme
% SYNTAX
%   profile = azElInternal.motion.buildStraightJerkProfile( ...
%       route_deg, initialState, limits)
%**************************************************************************
% PURPOSE
%   - Build the exact coefficient record for a symmetric minimum-time
%     straight-path motion with bounded velocity, acceleration, and jerk.
%**************************************************************************
% INPUTS
%   - route_deg (2-by-2 double): straight start and goal positions.
%   - initialState (scalar struct): normalized state containing time_s.
%   - limits (scalar struct): normalized axis kinematic limits.
%**************************************************************************
% OUTPUTS
%   - profile (scalar struct): quintic-schema polynomial and phase data.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Scalar Path Limits And Phase Durations

% Write the two-axis line as start + scalarPosition * delta. Dividing each
% axis limit by its displacement converts the problem into one unit-distance
% scalar motion whose solution automatically respects both physical axes.
delta_deg = route_deg(2, :) - route_deg(1, :);
activeAxis = abs(delta_deg) > 1e-12;
if ~any(activeAxis)
    scalarVelocityLimit_s = Inf;
    scalarAccelerationLimit_s2 = Inf;
    scalarJerkLimit_s3 = Inf;
else
    scalarVelocityLimit_s = min( limits.maxVelocity_deg_s(activeAxis) ./ abs(delta_deg(activeAxis)));
    scalarAccelerationLimit_s2 = min( limits.maxAcceleration_deg_s2(activeAxis) ./ abs(delta_deg(activeAxis)));
    scalarJerkLimit_s3 = min( limits.maxJerk_deg_s3(activeAxis) ./ abs(delta_deg(activeAxis)));
end
[jerkRampTime_s, constantAccelerationTime_s, cruiseTime_s] = profileDurations( ...
    scalarVelocityLimit_s, scalarAccelerationLimit_s2, scalarJerkLimit_s3);
durationScale = 1 + 64 * eps;
phaseDuration_s = durationScale * [ ...
    jerkRampTime_s; constantAccelerationTime_s; jerkRampTime_s; ...
    cruiseTime_s; jerkRampTime_s; constantAccelerationTime_s; jerkRampTime_s];
phaseJerk_s3 = [1; 0; -1; 0; -1; 0; 1] * scalarJerkLimit_s3 / durationScale^3;
retainedPhase = phaseDuration_s > 1e-12;
phaseDuration_s = phaseDuration_s(retainedPhase);
phaseJerk_s3 = phaseJerk_s3(retainedPhase);

%% Section 2: Integrate Each Constant-Jerk Phase Exactly

% Each phase uses normalized local time tau in [0,1]. The coefficient arrays
% are stored in ascending powers because the common evaluator and continuous
% validator consume that representation directly.
segmentCount = numel(phaseDuration_s);
positionPower_deg = zeros(segmentCount, 2, 6);
velocityPower_deg_s = zeros(segmentCount, 2, 5);
accelerationPower_deg_s2 = zeros(segmentCount, 2, 4);
jerkPower_deg_s3 = zeros(segmentCount, 2, 3);
scalarPosition = 0;
scalarVelocity = 0;
scalarAcceleration = 0;

% Integrate the seven-phase scalar timing law and map each phase onto both axes.
for segmentIndex = 1:segmentCount
    duration_s = phaseDuration_s(segmentIndex);
    scalarJerk = phaseJerk_s3(segmentIndex);
    scalarPositionPower = [ ...
        scalarPosition, scalarVelocity * duration_s, ...
        0.5 * scalarAcceleration * duration_s^2, scalarJerk * duration_s^3 / 6, 0, 0];
    scalarVelocityPower = [ scalarVelocity, scalarAcceleration * duration_s, 0.5 * scalarJerk * duration_s^2, 0, 0];
    scalarAccelerationPower = [ scalarAcceleration, scalarJerk * duration_s, 0, 0];
    positionPower = delta_deg.' .* scalarPositionPower;
    positionPower(:, 1) = positionPower(:, 1) + route_deg(1, :).';
    positionPower_deg(segmentIndex, :, :) = positionPower;
    velocityPower_deg_s(segmentIndex, :, :) = delta_deg.' .* scalarVelocityPower;
    accelerationPower_deg_s2(segmentIndex, :, :) = delta_deg.' .* scalarAccelerationPower;
    jerkPower_deg_s3(segmentIndex, :, :) = delta_deg.' .* [scalarJerk 0 0];
    scalarPosition = sum(scalarPositionPower);
    scalarVelocity = sum(scalarVelocityPower);
    scalarAcceleration = sum(scalarAccelerationPower);
end
segmentStartTime_s = initialState.time_s + [0; cumsum(phaseDuration_s(1:end - 1))];
finalTime_s = initialState.time_s + sum(phaseDuration_s);
polynomial = struct( ...
    "SegmentCount", segmentCount, ...
    "SegmentStartTime_s", segmentStartTime_s, ...
    "SegmentDuration_s", phaseDuration_s, ...
    "FinalTime_s", finalTime_s, ...
    "positionPower_deg", positionPower_deg, ...
    "velocityPower_deg_s", velocityPower_deg_s, ...
    "accelerationPower_deg_s2", accelerationPower_deg_s2, ...
    "jerkPower_deg_s3", jerkPower_deg_s3, ...
    "TerminalState", struct( "position_deg", route_deg(2, :), "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]));
profile = struct( ...
    "Delta_deg", delta_deg, "PhaseDuration_s", phaseDuration_s, "PhaseJerk_s3", phaseJerk_s3, "Polynomial", polynomial);
end

%% Section 3: Local Functions

function [jerkRampTime_s, constantAccelerationTime_s, cruiseTime_s] = profileDurations(velocityLimit_s, accelerationLimit_s2, jerkLimit_s3)
% Solve the symmetric unit-distance jerk-limited timing law.
if ~isfinite(jerkLimit_s3)
    jerkRampTime_s = 1e-3;
    constantAccelerationTime_s = 0;
    cruiseTime_s = 0;
    return;
end
accelerationDistance = 2 * accelerationLimit_s2^3 / jerkLimit_s3^2;
if accelerationDistance >= 1
    jerkRampTime_s = nthroot(1 / (2 * jerkLimit_s3), 3);
    constantAccelerationTime_s = 0;
else
    jerkRampTime_s = accelerationLimit_s2 / jerkLimit_s3;
    constantAccelerationTime_s = 0.5 * ( sqrt(jerkRampTime_s^2 + 4 / accelerationLimit_s2) - 3 * jerkRampTime_s);
end
peakVelocity_s = jerkLimit_s3 * jerkRampTime_s * (jerkRampTime_s + constantAccelerationTime_s);
cruiseTime_s = 0;
if peakVelocity_s <= velocityLimit_s
    return;
end
if velocityLimit_s <= accelerationLimit_s2^2 / jerkLimit_s3
    jerkRampTime_s = sqrt(velocityLimit_s / jerkLimit_s3);
    constantAccelerationTime_s = 0;
else
    jerkRampTime_s = accelerationLimit_s2 / jerkLimit_s3;
    constantAccelerationTime_s = velocityLimit_s / accelerationLimit_s2 - jerkRampTime_s;
end
minimumDistance = velocityLimit_s * (2 * jerkRampTime_s + constantAccelerationTime_s);
cruiseTime_s = max(0, (1 - minimumDistance) / velocityLimit_s);
end
