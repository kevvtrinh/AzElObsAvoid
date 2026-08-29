function distance_deg = maximumRestToRestDistance( ...
        duration_s, velocityLimit_deg_s, ...
        accelerationLimit_deg_s2, jerkLimit_deg_s3)
%% Section 0: Header & Readme
% SYNTAX
%   distance_deg = bmtpEngine.maximumRestToRestDistance( ...
%       duration_s, velocityLimit_deg_s, ...
%       accelerationLimit_deg_s2, jerkLimit_deg_s3)
%**************************************************************************
% PURPOSE
%   - Return the exact maximum rest-to-rest scalar displacement in one clock.
%**************************************************************************
% INPUTS
%   - duration_s (nonnegative finite scalar)
%       Available motion duration.
%   - velocityLimit_deg_s (positive finite scalar)
%       Symmetric velocity magnitude limit.
%   - accelerationLimit_deg_s2 (positive finite scalar)
%       Symmetric acceleration magnitude limit.
%   - jerkLimit_deg_s3 (positive finite scalar)
%       Symmetric jerk magnitude limit.
%**************************************************************************
% OUTPUTS
%   - distance_deg (nonnegative scalar)
%       Maximum displacement attained by the exact symmetric switching law.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivative limits use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Validate The Scalar Limits

validateattributes(duration_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(velocityLimit_deg_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
validateattributes(accelerationLimit_deg_s2, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
validateattributes(jerkLimit_deg_s3, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});

%% Section 2: Invert The Exact Switching Regimes

accelerationRampTime_s = accelerationLimit_deg_s2 / jerkLimit_deg_s3;
velocityWithoutPlateau_deg_s = ...
    accelerationLimit_deg_s2 ^ 2 / jerkLimit_deg_s3;
if velocityLimit_deg_s <= velocityWithoutPlateau_deg_s
    accelerationRampTime_s = sqrt(velocityLimit_deg_s / jerkLimit_deg_s3);
    constantAccelerationTime_s = 0;
else
    constantAccelerationTime_s = ...
        velocityLimit_deg_s / accelerationLimit_deg_s2 - ...
        accelerationRampTime_s;
end
accelerationLimitedDuration_s = 4 * accelerationRampTime_s;
if duration_s <= accelerationLimitedDuration_s
    distance_deg = jerkLimit_deg_s3 * duration_s ^ 3 / 32;
    return;
end
velocityLimitedDuration_s = ...
    4 * accelerationRampTime_s + 2 * constantAccelerationTime_s;
if duration_s <= velocityLimitedDuration_s
    constantAccelerationTime_s = ...
        0.5 * duration_s - 2 * accelerationRampTime_s;
    distance_deg = accelerationLimit_deg_s2 * ...
        (accelerationRampTime_s + constantAccelerationTime_s) * ...
        (2 * accelerationRampTime_s + constantAccelerationTime_s);
else
    distance_deg = velocityLimit_deg_s * (duration_s - ...
        2 * accelerationRampTime_s - constantAccelerationTime_s);
end
end
