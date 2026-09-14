function [controlPoint_units, segmentTime_s, phases] = createJerkLimitedChord(start_units, goal_units, limits, degree)
%% Section 0: Header & Readme
% SYNTAX: [controls, durations] = bmtpEngine.createJerkLimitedChord(start, goal, limits, degree)
% PURPOSE: Exact minimum-time rest-to-rest scalar progress along a chord.
% INPUTS: Distinct 1-by-2 endpoints, positive axis limits, degree at least three.
% OUTPUTS: Degree-elevated cubic Bezier spans, physical durations, and optional analytic phase
%   position, velocity, acceleration, and jerk states.
% UNITS: Position in coordinate units; durations in seconds.

%% Section 1: Solve The Scalar Jerk-Limited Profile
displacement_units = goal_units-start_units;
distance_units = abs(displacement_units);
velocity_s1 = min(limits.maxVelocity_units_s./distance_units);
acceleration_s2 = min(limits.maxAcceleration_units_s2./distance_units);
jerk_s3 = min(limits.maxJerk_units_s3./distance_units);
accelerationRampTime_s = acceleration_s2/jerk_s3;
velocityRampTime_s = sqrt(velocity_s1/jerk_s3);
if velocityRampTime_s <= accelerationRampTime_s
    rampTime_s = velocityRampTime_s;
    holdTime_s = 0;
else
    rampTime_s = accelerationRampTime_s;
    holdTime_s = (velocity_s1-jerk_s3*rampTime_s^2)/ ...
        (jerk_s3*rampTime_s);
end
noCruiseDistance = velocity_s1*(2*rampTime_s+holdTime_s);
regimeTolerance = 64*eps(max(1,abs(noCruiseDistance)));
if noCruiseDistance < 1-regimeTolerance
    cruiseTime_s = (1-noCruiseDistance)/velocity_s1;
elseif noCruiseDistance <= 1+regimeTolerance
    cruiseTime_s = 0;
else
    cruiseTime_s = 0;
    displacementRampTime_s = (1/(2*jerk_s3))^(1/3);
    if displacementRampTime_s <= accelerationRampTime_s
        rampTime_s = displacementRampTime_s;
        holdTime_s = 0;
    else
        rampTime_s = accelerationRampTime_s;
        rootTerm_s = sqrt(rampTime_s^2+4/(jerk_s3*rampTime_s));
        holdTime_s = 2*(1/(jerk_s3*rampTime_s)-2*rampTime_s^2)/ ...
            (rootTerm_s+3*rampTime_s);
    end
end
segmentTime_s = [rampTime_s;holdTime_s;rampTime_s;cruiseTime_s;rampTime_s;holdTime_s;rampTime_s];
segmentJerk_s3 = jerk_s3*[1;0;-1;0;-1;0;1];
% Analytic regime selection assigns inactive phases exactly zero. Do not
% compare one phase with another: a short physical ramp can legitimately
% coexist with a very long cruise.
active = segmentTime_s>0;
segmentTime_s = segmentTime_s(active);
segmentJerk_s3 = segmentJerk_s3(active);

%% Section 2: Integrate And Elevate Each Cubic Without Changing Its Curve
controlPoint_units = zeros(numel(segmentTime_s),degree+1,2);
phases = struct('StartTime_s',[0;cumsum(segmentTime_s(1:end-1))], ...
    'SegmentTime_s',segmentTime_s,'Position_units',zeros(numel(segmentTime_s),2), ...
    'Velocity_units_s',zeros(numel(segmentTime_s),2),'Acceleration_units_s2',zeros(numel(segmentTime_s),2), ...
    'Jerk_units_s3',segmentJerk_s3.*displacement_units);
position = 0; velocity_s1 = 0; acceleration_s2 = 0;
for k = 1:numel(segmentTime_s)
    duration_s = segmentTime_s(k);
    jerk_s3 = segmentJerk_s3(k);
    phases.Position_units(k,:) = start_units+position*displacement_units;
    phases.Velocity_units_s(k,:) = velocity_s1*displacement_units;
    phases.Acceleration_units_s2(k,:) = acceleration_s2*displacement_units;
    powers = [position;velocity_s1*duration_s;acceleration_s2*duration_s^2/2;jerk_s3*duration_s^3/6];
    controlPoint_units(k,:,:) = start_units+bmtpEngine.powerToBernstein(powers,degree).*displacement_units;
    position = sum(powers);
    velocity_s1 = velocity_s1+acceleration_s2*duration_s+jerk_s3*duration_s^2/2;
    acceleration_s2 = acceleration_s2+jerk_s3*duration_s;
end
end
