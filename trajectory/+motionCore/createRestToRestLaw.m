function [duration_s, jerk_units_s3] = createRestToRestLaw(displacement_units, velocityLimit_units_s, accelerationLimit_units_s2, jerkLimit_units_s3)
%% Section 0: Header & Readme
% SYNTAX: [duration_s, jerk_units_s3] = motionCore.createRestToRestLaw(displacement_units, velocityLimit_units_s, accelerationLimit_units_s2, jerkLimit_units_s3)
% PURPOSE: Solve the exact symmetric seven-phase scalar rest-to-rest law.
% INPUTS: Scalar displacement and positive finite derivative limits.
% OUTPUTS: Seven phase durations and signed jerks; zero displacement returns zeros.
% UNITS: Coordinate units, seconds, and physical derivatives.
%% Section 1: Solve The Physical Switching Events
distance_units = abs(displacement_units);
duration_s   = zeros(1, 7);
jerk_units_s3  = zeros(1, 7);
if distance_units == 0
    return;
end
ramp_s = accelerationLimit_units_s2 / jerkLimit_units_s3;
if 2 * accelerationLimit_units_s2 ^ 3 / jerkLimit_units_s3 ^ 2 >= distance_units
    ramp_s    = nthroot(distance_units / (2 * jerkLimit_units_s3), 3);
    plateau_s = 0;
else
    plateau_s = 0.5 * (sqrt(ramp_s ^ 2 + 4 * distance_units / accelerationLimit_units_s2) - 3 * ramp_s);
end
peakVelocity_units_s = jerkLimit_units_s3 * ramp_s * (ramp_s + plateau_s);
cruise_s           = 0;
if peakVelocity_units_s > velocityLimit_units_s
    if velocityLimit_units_s <= accelerationLimit_units_s2 ^ 2 / jerkLimit_units_s3
        ramp_s    = sqrt(velocityLimit_units_s / jerkLimit_units_s3);
        plateau_s = 0;
    else
        ramp_s    = accelerationLimit_units_s2 / jerkLimit_units_s3;
        plateau_s = velocityLimit_units_s / accelerationLimit_units_s2 - ramp_s;
    end
    minimumDistance_units = velocityLimit_units_s * (2 * ramp_s + plateau_s);
    cruise_s            = (distance_units - minimumDistance_units) / velocityLimit_units_s;
end
duration_s = [ramp_s, plateau_s, ramp_s, max(0, cruise_s), ...
    ramp_s, plateau_s, ramp_s];
jerk_units_s3 = sign(displacement_units) * jerkLimit_units_s3 * [1, 0, -1, 0, -1, 0, 1];
end
