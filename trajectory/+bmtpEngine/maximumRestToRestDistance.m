function distance_units = maximumRestToRestDistance(duration_s, velocityLimit_units_s, accelerationLimit_units_s2, jerkLimit_units_s3)
%% Section 0: Header & Readme
% SYNTAX
%   distance_units = bmtpEngine.maximumRestToRestDistance( ...
%       duration_s, velocityLimit_units_s, ...
%       accelerationLimit_units_s2, jerkLimit_units_s3)
%
% PURPOSE
%   - Return the exact maximum rest-to-rest scalar displacement in one clock.
%
% INPUTS
%   - duration_s (nonnegative finite scalar)
%       Available motion duration.
%   - velocityLimit_units_s (positive finite scalar)
%       Symmetric velocity magnitude limit.
%   - accelerationLimit_units_s2 (positive finite scalar)
%       Symmetric acceleration magnitude limit.
%   - jerkLimit_units_s3 (positive finite scalar)
%       Symmetric jerk magnitude limit.
%
% OUTPUTS
%   - distance_units (nonnegative scalar)
%       Maximum displacement attained by the exact symmetric switching law.
%
% UNITS
%   - Position is coordinate units and time is seconds. Derivative limits use units/s,
%     units/s^2, and units/s^3.
%

%% Section 1: Validate The Scalar Limits

validateattributes(duration_s, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(velocityLimit_units_s, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(accelerationLimit_units_s2, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
validateattributes(jerkLimit_units_s3, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});

%% Section 2: Invert The Exact Switching Regimes

accelerationRampTime_s       = accelerationLimit_units_s2 / jerkLimit_units_s3;
velocityWithoutPlateau_units_s = accelerationLimit_units_s2 ^ 2 / jerkLimit_units_s3;
if velocityLimit_units_s <= velocityWithoutPlateau_units_s
    accelerationRampTime_s     = sqrt(velocityLimit_units_s / jerkLimit_units_s3);
    constantAccelerationTime_s = 0;
else
    constantAccelerationTime_s = velocityLimit_units_s / accelerationLimit_units_s2 - accelerationRampTime_s;
end
accelerationLimitedDuration_s = 4 * accelerationRampTime_s;
if duration_s <= accelerationLimitedDuration_s
    distance_units = jerkLimit_units_s3 * duration_s ^ 3 / 32;
    return;
end
velocityLimitedDuration_s = 4 * accelerationRampTime_s + 2 * constantAccelerationTime_s;
if duration_s <= velocityLimitedDuration_s
    constantAccelerationTime_s = 0.5 * duration_s - 2 * accelerationRampTime_s;
    distance_units               = accelerationLimit_units_s2 * (accelerationRampTime_s + constantAccelerationTime_s) * (2 * accelerationRampTime_s + constantAccelerationTime_s);
else
    distance_units = velocityLimit_units_s * (duration_s - 2 * accelerationRampTime_s - constantAccelerationTime_s);
end
end
