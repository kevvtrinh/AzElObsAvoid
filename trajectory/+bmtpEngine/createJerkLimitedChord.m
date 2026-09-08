function [controlPoint_units, segmentTime_s] = createJerkLimitedChord(start_units, goal_units, limits, degree)
%% Section 0: Header & Readme
% SYNTAX: [controls, durations] = bmtpEngine.createJerkLimitedChord(start, goal, limits, degree)
% PURPOSE: Exact minimum-time rest-to-rest scalar progress along a chord.
% INPUTS: Distinct 1-by-2 endpoints, positive axis limits, degree at least three.
% OUTPUTS: Degree-elevated cubic Bezier spans and their physical durations.
% UNITS: Position in coordinate units; durations in seconds.

%% Section 1: Solve The Scalar Jerk-Limited Profile

displacement_units = goal_units-start_units;
distance_units = abs(displacement_units);
velocity_s1 = min(limits.maxVelocity_units_s./distance_units);
acceleration_s2 = min(limits.maxAcceleration_units_s2./distance_units);
jerk_s3 = min(limits.maxJerk_units_s3./distance_units);
rampTime_s = min(acceleration_s2/jerk_s3,sqrt(velocity_s1/jerk_s3));
holdTime_s = max(0,velocity_s1/(jerk_s3*rampTime_s)-rampTime_s);
cruiseTime_s = 1/velocity_s1-(2*rampTime_s+holdTime_s);
if cruiseTime_s < 0
    cruiseTime_s = 0;
    rampTime_s = min(acceleration_s2/jerk_s3,(1/(2*jerk_s3))^(1/3));
    holdTime_s = max(0,(-3*rampTime_s+sqrt(rampTime_s^2+4/(jerk_s3*rampTime_s)))/2);
end
segmentTime_s = [rampTime_s;holdTime_s;rampTime_s;cruiseTime_s;rampTime_s;holdTime_s;rampTime_s];
segmentJerk_s3 = jerk_s3*[1;0;-1;0;-1;0;1];
active = segmentTime_s>0;
segmentTime_s = segmentTime_s(active);
segmentJerk_s3 = segmentJerk_s3(active);

%% Section 2: Integrate And Elevate Each Cubic Without Changing Its Curve

conversion = zeros(degree+1,4);
for k = 0:degree
    for power = 0:min(k,3)
        conversion(k+1,power+1) = nchoosek(k,power)/nchoosek(degree,power);
    end
end
controlPoint_units = zeros(numel(segmentTime_s),degree+1,2);
position = 0; velocity_s1 = 0; acceleration_s2 = 0;
for k = 1:numel(segmentTime_s)
    duration_s = segmentTime_s(k);
    jerk_s3 = segmentJerk_s3(k);
    powers = [position;velocity_s1*duration_s;acceleration_s2*duration_s^2/2;jerk_s3*duration_s^3/6];
    controlPoint_units(k,:,:) = start_units+(conversion*powers).*displacement_units;
    position = sum(powers);
    velocity_s1 = velocity_s1+acceleration_s2*duration_s+jerk_s3*duration_s^2/2;
    acceleration_s2 = acceleration_s2+jerk_s3*duration_s;
end
end
