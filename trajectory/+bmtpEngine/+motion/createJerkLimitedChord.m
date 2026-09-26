function [controlPoint_units, segmentTime_s, motionPhases] = createJerkLimitedChord( ...
    start_units, goal_units, limits, degree)
%% Section 0: Header & Readme
% SYNTAX
%   [controlPoint_units, segmentTime_s, motionPhases] = ...
%       bmtpEngine.motion.createJerkLimitedChord(start_units, goal_units, limits, degree)
%**************************************************************************
% PURPOSE
%   - Move along a straight line in minimum time under the supplied speed,
%     acceleration, and jerk limits. Start and finish at rest with zero
%     acceleration. Jerk is constant within each motion phase.
%**************************************************************************
% INPUTS
%   - start_units (1-by-2 numeric row)
%       Starting [x, y] position.
%   - goal_units (1-by-2 numeric row)
%       Ending [x, y] position, different from the start.
%   - limits (scalar struct)
%       Positive maxVelocity_units_s, maxAcceleration_units_s2, and
%       maxJerk_units_s3 for each coordinate axis.
%   - degree (integer scalar)
%       Output Bezier degree, at least three. A higher degree represents
%       the same cubic position curve with more controls.
%**************************************************************************
% OUTPUTS
%   - controlPoint_units (S-by-(degree+1)-by-2 numeric array)
%       Bezier controls for each of S phases; the last dimension is x/y.
%   - segmentTime_s (positive numeric column)
%       Duration of each retained phase, in travel order.
%   - motionPhases (scalar struct)
%       StartTime_s, SegmentTime_s, and the Position_units,
%       Velocity_units_s, Acceleration_units_s2, and Jerk_units_s3 for
%       each phase. State values are taken at that phase's start.
%**************************************************************************
% UNITS
%   - Position and controls: coordinate units; time: seconds; velocity:
%     units/s; acceleration: units/s^2; jerk: units/s^3.
%**************************************************************************

%% Section 1: Choose Ramp, Acceleration-Hold, And Cruise Durations

% Progress goes from 0 at start to 1 at goal. Divide each axis limit by
% that axis's travel distance, then use the smaller progress limit. For
% 10 units of x travel at 2 units/s, progress is limited to 0.2 each
% second. An axis with no travel does not restrict progress.
displacement_units = goal_units - start_units;
axisDistance_units = abs(displacement_units);
maximumProgressRate_s1         = min(limits.maxVelocity_units_s ./ axisDistance_units);
maximumProgressAcceleration_s2 = min(limits.maxAcceleration_units_s2 ./ axisDistance_units);
maximumProgressJerk_s3         = min(limits.maxJerk_units_s3 ./ axisDistance_units);

% To reach the speed limit, first find how long jerk must increase
% acceleration. If speed is reached before acceleration reaches its
% limit, skip the constant-acceleration hold.
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

% The accelerating and braking halves together need
% progress = maximum speed x (2 x jerk-ramp time + acceleration-hold time).
% If this is below 1, travel the remaining fraction at constant speed.
progressNeededToReachSpeedLimit = ...
    maximumProgressRate_s1 * (2 * jerkRampTime_s + accelerationHoldTime_s);
progressRoundingAllowance = 64 * eps(max(1, abs(progressNeededToReachSpeedLimit)));
% Treat a difference within floating-point rounding as exactly full
% travel; a tiny numerical excess must not create a negative cruise.
if progressNeededToReachSpeedLimit < 1 - progressRoundingAllowance
    cruiseTime_s = (1 - progressNeededToReachSpeedLimit) / maximumProgressRate_s1;
elseif progressNeededToReachSpeedLimit <= 1 + progressRoundingAllowance
    cruiseTime_s = 0;
else
    % The path is too short to reach maximum speed. Shorten the ramps and
    % any acceleration hold so acceleration plus braking cover progress 1.
    cruiseTime_s          = 0;
    rampTimeForDistance_s = (1 / (2 * maximumProgressJerk_s3)) ^ (1 / 3);
    % Four jerk ramps alone cover progress = 2 x jerk limit x ramp time^3.
    % If that ramp would exceed the acceleration limit, hold acceleration
    % between the ramps instead.
    if rampTimeForDistance_s <= rampTimeToAccelerationLimit_s
        jerkRampTime_s         = rampTimeForDistance_s;
        accelerationHoldTime_s = 0;
    else
        jerkRampTime_s = rampTimeToAccelerationLimit_s;
        holdTimeRoot_s = sqrt(jerkRampTime_s ^ 2 + 4 / (maximumProgressJerk_s3 * jerkRampTime_s));
        % Solve for the needed hold without subtracting nearly equal
        % numbers when the hold approaches zero.
        accelerationHoldTime_s = 2 * (1 / (maximumProgressJerk_s3 * jerkRampTime_s) - 2 * jerkRampTime_s ^ 2) / ...
            (holdTimeRoot_s + 3 * jerkRampTime_s);
    end
end

%% Section 2: Keep The Positive-Duration Motion Phases

% Accelerate with positive jerk, an optional acceleration hold, and
% negative jerk. After an optional cruise, mirror those phases to brake.
segmentTime_s          = [jerkRampTime_s; accelerationHoldTime_s; jerkRampTime_s; cruiseTime_s; ...
    jerkRampTime_s; accelerationHoldTime_s; jerkRampTime_s];
segmentProgressJerk_s3 = maximumProgressJerk_s3 * [1; 0; -1; 0; -1; 0; 1];

% Remove zero-duration holds and cruises. Retain every positive duration:
% a short ramp is still needed even beside a much longer cruise.
phaseIsActive          = segmentTime_s > 0;
segmentTime_s          = segmentTime_s(phaseIsActive);
segmentProgressJerk_s3 = segmentProgressJerk_s3(phaseIsActive);

%% Section 3: Integrate Constant Jerk And Express Each Segment As Bezier Controls

% Constant jerk gives position p(t) = p0 + v0 x t + a0 x t^2 / 2
% + j x t^3 / 6. Record each phase's start state, express its cubic curve
% with Bezier controls, then carry its end state into the next phase.
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
    % These coefficients use local fraction u = elapsed time / phase time,
    % which runs from 0 to 1 over this phase.
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
