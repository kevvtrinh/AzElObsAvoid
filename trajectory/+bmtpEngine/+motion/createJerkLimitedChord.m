function [controlPoint_units, segmentTime_s, motionPhases] = createJerkLimitedChord( ...
    start_units, goal_units, limits, degree)
%% Section 0: Header & Readme
% SYNTAX
%   [controlPoint_units, segmentTime_s, motionPhases] = ...
%       bmtpEngine.motion.createJerkLimitedChord(start_units, goal_units, limits, degree)
%**************************************************************************
% PURPOSE
%   - Create the minimum-time straight-line motion under the per-axis
%     speed, acceleration, and jerk limits. Start and finish with zero
%     velocity and acceleration; jerk is constant within each phase.
%**************************************************************************
% INPUTS
%   - start_units (1-by-2 numeric row)
%       Straight-line start position.
%   - goal_units (1-by-2 numeric row)
%       Goal position, different from the start.
%   - limits (scalar struct)
%       Positive per-axis velocity, acceleration, and jerk limits.
%   - degree (integer scalar)
%       Output Bezier degree; at least three.
%**************************************************************************
% OUTPUTS
%   - controlPoint_units (S-by-(degree+1)-by-2 numeric array)
%       Bezier controls for each motion segment; the third dimension is x/y.
%       Increasing the requested degree preserves the same cubic curve.
%   - segmentTime_s (positive numeric column)
%       Physical duration of each retained phase.
%   - motionPhases (scalar struct)
%       Start time, starting position/velocity/acceleration, duration, and
%       constant jerk for each retained phase.
%**************************************************************************
% UNITS
%   - Position is coordinate units and duration is seconds.
%**************************************************************************

%% Section 1: Choose Ramp, Acceleration-Hold, And Cruise Durations

% Progress runs from 0 to 1 along the straight path. Divide each axis limit
% by its travel distance and use the tighter limit. Zero travel gives Inf,
% so that axis does not restrict progress.
displacement_units = goal_units - start_units;
axisDistance_units = abs(displacement_units);
maximumProgressRate_s1         = min(limits.maxVelocity_units_s ./ axisDistance_units);
maximumProgressAcceleration_s2 = min(limits.maxAcceleration_units_s2 ./ axisDistance_units);
maximumProgressJerk_s3         = min(limits.maxJerk_units_s3 ./ axisDistance_units);

% A low speed limit may be reached without reaching maximum acceleration.
% In that case, use jerk ramps with no constant-acceleration hold.
rampTimeToAccelerationLimit_s = maximumProgressAcceleration_s2 / maximumProgressJerk_s3;
rampTimeToVelocityLimit_s     = sqrt(maximumProgressRate_s1 / maximumProgressJerk_s3);
if rampTimeToVelocityLimit_s <= rampTimeToAccelerationLimit_s
    jerkRampTime_s         = rampTimeToVelocityLimit_s;
    accelerationHoldTime_s = 0;
else
    jerkRampTime_s         = rampTimeToAccelerationLimit_s;
    accelerationHoldTime_s = (maximumProgressRate_s1 - maximumProgressJerk_s3 * jerkRampTime_s ^ 2) / ...
        (maximumProgressJerk_s3 * jerkRampTime_s);
end

% Calculate the progress needed to speed up to the limit and slow down.
% If that uses less than the full path (1), fill the rest with a cruise.
progressNeededToReachSpeedLimit = ...
    maximumProgressRate_s1 * (2 * jerkRampTime_s + accelerationHoldTime_s);
progressRoundingAllowance = 64 * eps(max(1, abs(progressNeededToReachSpeedLimit)));
if progressNeededToReachSpeedLimit < 1 - progressRoundingAllowance
    cruiseTime_s = (1 - progressNeededToReachSpeedLimit) / maximumProgressRate_s1;
elseif progressNeededToReachSpeedLimit <= 1 + progressRoundingAllowance
    cruiseTime_s = 0;
else
    % The path is too short to reach maximum speed. Recalculate the ramps
    % and any acceleration hold to cover exactly one unit of progress.
    cruiseTime_s          = 0;
    rampTimeForDistance_s = (1 / (2 * maximumProgressJerk_s3)) ^ (1 / 3);
    if rampTimeForDistance_s <= rampTimeToAccelerationLimit_s
        jerkRampTime_s         = rampTimeForDistance_s;
        accelerationHoldTime_s = 0;
    else
        jerkRampTime_s = rampTimeToAccelerationLimit_s;
        holdTimeRoot_s = sqrt(jerkRampTime_s ^ 2 + 4 / (maximumProgressJerk_s3 * jerkRampTime_s));
        % This form of the quadratic solution avoids subtracting nearly
        % equal values when the required hold is very short.
        accelerationHoldTime_s = 2 * (1 / (maximumProgressJerk_s3 * jerkRampTime_s) - 2 * jerkRampTime_s ^ 2) / ...
            (holdTimeRoot_s + 3 * jerkRampTime_s);
    end
end

%% Section 2: Keep The Positive-Duration Motion Phases

% Accelerate through three phases, cruise, then decelerate through three.
% Each three-phase half ramps acceleration away from zero, holds it,
% then brings it back to zero.
segmentTime_s          = [jerkRampTime_s; accelerationHoldTime_s; jerkRampTime_s; cruiseTime_s; ...
    jerkRampTime_s; accelerationHoldTime_s; jerkRampTime_s];
segmentProgressJerk_s3 = maximumProgressJerk_s3 * [1; 0; -1; 0; -1; 0; 1];

% Unused holds and cruises have duration 0. Keep every positive duration:
% a very short ramp can still be necessary beside a very long cruise.
phaseIsActive          = segmentTime_s > 0;
segmentTime_s          = segmentTime_s(phaseIsActive);
segmentProgressJerk_s3 = segmentProgressJerk_s3(phaseIsActive);

%% Section 3: Integrate Constant Jerk And Express Each Segment As Bezier Controls

% With constant jerk, p(t) = p0 + v0 x t + a0 x t^2 / 2 + j x t^3 / 6.
% Store the phase start state, convert its position curve to Bezier form,
% then carry the ending progress, rate, and acceleration into the next phase.
phaseCount         = numel(segmentTime_s);
controlPoint_units = zeros(phaseCount, degree + 1, 2);
motionPhases = struct( ...
    'StartTime_s',           [0; cumsum(segmentTime_s(1:end - 1))], ...
    'SegmentTime_s',         segmentTime_s, ...
    'Position_units',        zeros(phaseCount, 2), ...
    'Velocity_units_s',      zeros(phaseCount, 2), ...
    'Acceleration_units_s2', zeros(phaseCount, 2), ...
    'Jerk_units_s3',         segmentProgressJerk_s3 .* displacement_units);
progress               = 0;
progressRate_s1         = 0;
progressAcceleration_s2 = 0;
for phaseIndex = 1:phaseCount
    phaseDuration_s = segmentTime_s(phaseIndex);
    progressJerk_s3 = segmentProgressJerk_s3(phaseIndex);
    motionPhases.Position_units(phaseIndex, :)        = start_units + progress * displacement_units;
    motionPhases.Velocity_units_s(phaseIndex, :)      = progressRate_s1 * displacement_units;
    motionPhases.Acceleration_units_s2(phaseIndex, :) = progressAcceleration_s2 * displacement_units;
    progressCoefficients = [progress; progressRate_s1 * phaseDuration_s; ...
        progressAcceleration_s2 * phaseDuration_s ^ 2 / 2; progressJerk_s3 * phaseDuration_s ^ 3 / 6];
    controlPoint_units(phaseIndex, :, :) = start_units + ...
        bmtpEngine.motion.powerToBernstein(progressCoefficients, degree) .* displacement_units;
    progress               = sum(progressCoefficients);
    progressRate_s1         = progressRate_s1 + progressAcceleration_s2 * phaseDuration_s + ...
        progressJerk_s3 * phaseDuration_s ^ 2 / 2;
    progressAcceleration_s2 = progressAcceleration_s2 + progressJerk_s3 * phaseDuration_s;
end
end
