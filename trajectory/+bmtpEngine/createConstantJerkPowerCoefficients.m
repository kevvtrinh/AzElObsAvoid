function [positionPower_units, velocityPower_units_s, accelerationPower_units_s2] = createConstantJerkPowerCoefficients(position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3, step_s)
%% Section 0: Header & Readme
% SYNTAX
%   [positionPower_units, velocityPower_units_s, accelerationPower_units_s2] = ...
%       bmtpEngine.createConstantJerkPowerCoefficients( ...
%       position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3, step_s)
%
% PURPOSE
%   - Create normalized-segment power coefficients for one constant-jerk span.
%
% INPUTS
%   - position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3
%       Matching 1-by-D state rows at the start of the span.
%   - step_s (positive finite scalar)
%       Span duration used to normalize local time to [0, 1].
%
% OUTPUTS
%   - positionPower_units (1-by-D-by-4 numeric array)
%       Ascending local-time position coefficients.
%   - velocityPower_units_s (1-by-D-by-3 numeric array)
%       Ascending local-time velocity coefficients.
%   - accelerationPower_units_s2 (1-by-D-by-2 numeric array)
%       Ascending local-time acceleration coefficients.
%
% UNITS
%   - Position is coordinate units, time is seconds, and derivatives use units/s,
%     units/s^2, and units/s^3.
%

%% Section 1: Create The Normalized Constant-Jerk Coefficients

dimensionCount           = numel(position_units);
positionPower_units        = reshape([position_units; velocity_units_s * step_s; acceleration_units_s2 * step_s ^ 2 / 2; jerk_units_s3 * step_s ^ 3 / 6].', 1, dimensionCount, 4);
velocityPower_units_s      = reshape([velocity_units_s; acceleration_units_s2 * step_s; jerk_units_s3 * step_s ^ 2 / 2].', 1, dimensionCount, 3);
accelerationPower_units_s2 = reshape([acceleration_units_s2; jerk_units_s3 * step_s].', 1, dimensionCount, 2);
end
