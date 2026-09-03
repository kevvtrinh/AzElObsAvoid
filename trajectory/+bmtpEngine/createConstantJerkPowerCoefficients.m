function [positionPower_deg, velocityPower_deg_s, ...
        accelerationPower_deg_s2] = createConstantJerkPowerCoefficients( ...
        position_deg, velocity_deg_s, acceleration_deg_s2, jerk_deg_s3, step_s)
%% Section 0: Header & Readme
% SYNTAX
%   [positionPower_deg, velocityPower_deg_s, accelerationPower_deg_s2] = ...
%       bmtpEngine.createConstantJerkPowerCoefficients( ...
%       position_deg, velocity_deg_s, acceleration_deg_s2, jerk_deg_s3, step_s)
%**************************************************************************
% PURPOSE
%   - Create normalized-segment power coefficients for one constant-jerk span.
%**************************************************************************
% INPUTS
%   - position_deg, velocity_deg_s, acceleration_deg_s2, jerk_deg_s3
%       Matching 1-by-D state rows at the start of the span.
%   - step_s (positive finite scalar)
%       Span duration used to normalize local time to [0, 1].
%**************************************************************************
% OUTPUTS
%   - positionPower_deg (1-by-D-by-4 numeric array)
%       Ascending local-time position coefficients.
%   - velocityPower_deg_s (1-by-D-by-3 numeric array)
%       Ascending local-time velocity coefficients.
%   - accelerationPower_deg_s2 (1-by-D-by-2 numeric array)
%       Ascending local-time acceleration coefficients.
%**************************************************************************
% UNITS
%   - Position is degrees, time is seconds, and derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Create The Normalized Constant-Jerk Coefficients

dimensionCount = numel(position_deg);
positionPower_deg = reshape([position_deg; ...
    velocity_deg_s * step_s; acceleration_deg_s2 * step_s ^ 2 / 2; ...
    jerk_deg_s3 * step_s ^ 3 / 6].', 1, dimensionCount, 4);
velocityPower_deg_s = reshape([velocity_deg_s; ...
    acceleration_deg_s2 * step_s; jerk_deg_s3 * step_s ^ 2 / 2].', ...
    1, dimensionCount, 3);
accelerationPower_deg_s2 = reshape([acceleration_deg_s2; ...
    jerk_deg_s3 * step_s].', 1, dimensionCount, 2);
end
