function [controlPoint_units, segmentTime_s, phases] = createJerkLimitedChord( ...
        start_units, goal_units, limits, degree)
%% Section 0: Header & Readme
% SYNTAX
%   [controlPoint_units, segmentTime_s, phases] = ...
%       bmtpEngine.createJerkLimitedChord(start_units, goal_units, limits, degree)
%**************************************************************************
% PURPOSE
%   - Construct exact minimum-time rest-to-rest progress along a chord.
%**************************************************************************
% INPUTS
%   - start_units (1-by-2 numeric row)
%       Chord start point.
%   - goal_units (1-by-2 numeric row)
%       Distinct chord endpoint.
%   - limits (scalar struct)
%       Positive per-axis velocity, acceleration, and jerk limits.
%   - degree (integer scalar)
%       Output Bezier degree; at least three.
%**************************************************************************
% OUTPUTS
%   - controlPoint_units (S-by-(degree+1)-by-2 numeric array)
%       Degree-elevated Bezier spans, one page per coordinate.
%   - segmentTime_s (positive numeric column)
%       Physical duration of each retained phase.
%   - phases (scalar struct)
%       Analytic phase position, velocity, acceleration, and jerk states.
%**************************************************************************
% UNITS
%   - Position is coordinate units and duration is seconds.
%**************************************************************************

%% Section 1: Solve The Scalar Jerk-Limited Profile
displacement_units      = goal_units - start_units;
distance_units          = abs(displacement_units);
progressRate_s1         = min(limits.maxVelocity_units_s ./ distance_units);
progressAcceleration_s2 = min(limits.maxAcceleration_units_s2 ./ distance_units);
progressJerk_s3         = min(limits.maxJerk_units_s3 ./ distance_units);
accelerationRampTime_s  = progressAcceleration_s2 / progressJerk_s3;
velocityRampTime_s      = sqrt(progressRate_s1 / progressJerk_s3);
if velocityRampTime_s <= accelerationRampTime_s
    rampTime_s = velocityRampTime_s;
    holdTime_s = 0;
else
    rampTime_s = accelerationRampTime_s;
    holdTime_s = (progressRate_s1 - progressJerk_s3 * rampTime_s ^ 2) / ...
        (progressJerk_s3 * rampTime_s);
end
noCruiseProgress = progressRate_s1 * (2 * rampTime_s + holdTime_s);
regimeTolerance  = 64 * eps(max(1, abs(noCruiseProgress)));
if noCruiseProgress < 1 - regimeTolerance
    cruiseTime_s = (1 - noCruiseProgress) / progressRate_s1;
elseif noCruiseProgress <= 1 + regimeTolerance
    cruiseTime_s = 0;
else
    cruiseTime_s           = 0;
    displacementRampTime_s = (1 / (2 * progressJerk_s3)) ^ (1 / 3);
    if displacementRampTime_s <= accelerationRampTime_s
        rampTime_s = displacementRampTime_s;
        holdTime_s = 0;
    else
        rampTime_s = accelerationRampTime_s;
        rootTerm_s = sqrt(rampTime_s ^ 2 + 4 / (progressJerk_s3 * rampTime_s));
        holdTime_s = 2 * (1 / (progressJerk_s3 * rampTime_s) - 2 * rampTime_s ^ 2) / ...
            (rootTerm_s + 3 * rampTime_s);
    end
end

%% Section 2: Retain The Phases The Regime Actually Uses
segmentTime_s          = [rampTime_s; holdTime_s; rampTime_s; cruiseTime_s; ...
    rampTime_s; holdTime_s; rampTime_s];
segmentProgressJerk_s3 = progressJerk_s3 * [1; 0; -1; 0; -1; 0; 1];
% Analytic regime selection assigns inactive phases exactly zero. Do not
% compare one phase with another: a short physical ramp can legitimately
% coexist with a very long cruise.
phaseIsActive          = segmentTime_s > 0;
segmentTime_s          = segmentTime_s(phaseIsActive);
segmentProgressJerk_s3 = segmentProgressJerk_s3(phaseIsActive);

%% Section 3: Integrate And Elevate Each Cubic Without Changing Its Curve
phaseCount         = numel(segmentTime_s);
controlPoint_units = zeros(phaseCount, degree + 1, 2);
phases = struct( ...
    'StartTime_s',           [0; cumsum(segmentTime_s(1:end - 1))], ...
    'SegmentTime_s',         segmentTime_s, ...
    'Position_units',        zeros(phaseCount, 2), ...
    'Velocity_units_s',      zeros(phaseCount, 2), ...
    'Acceleration_units_s2', zeros(phaseCount, 2), ...
    'Jerk_units_s3',         segmentProgressJerk_s3 .* displacement_units);
progress                = 0;
progressRate_s1         = 0;
progressAcceleration_s2 = 0;
for phaseIndex = 1:phaseCount
    duration_s      = segmentTime_s(phaseIndex);
    progressJerk_s3 = segmentProgressJerk_s3(phaseIndex);
    phases.Position_units(phaseIndex, :)        = start_units + progress * displacement_units;
    phases.Velocity_units_s(phaseIndex, :)      = progressRate_s1 * displacement_units;
    phases.Acceleration_units_s2(phaseIndex, :) = progressAcceleration_s2 * displacement_units;
    progressPowers = [progress; progressRate_s1 * duration_s; ...
        progressAcceleration_s2 * duration_s ^ 2 / 2; progressJerk_s3 * duration_s ^ 3 / 6];
    controlPoint_units(phaseIndex, :, :) = start_units + ...
        bmtpEngine.powerToBernstein(progressPowers, degree) .* displacement_units;
    progress                = sum(progressPowers);
    progressRate_s1         = progressRate_s1 + progressAcceleration_s2 * duration_s + ...
        progressJerk_s3 * duration_s ^ 2 / 2;
    progressAcceleration_s2 = progressAcceleration_s2 + progressJerk_s3 * duration_s;
end
end
