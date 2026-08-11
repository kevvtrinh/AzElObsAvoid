function coefficients = quinticHermiteCoefficients(startState, endState, ...
        duration_s)
%% Section 0: Header & Readme
% SYNTAX
%   coefficients = quinticHermiteCoefficients( ...
%       startState, endState, duration_s)
%**************************************************************************
% PURPOSE
%   - Construct the unique quintic segment matching position, velocity,
%     and acceleration at both ends.
%**************************************************************************
% INPUTS
%   - startState (struct)
%       position_deg, velocity_deg_s, and acceleration_deg_s2 row vectors.
%   - endState (struct)
%       Terminal fields matching startState.
%   - duration_s (positive scalar)
%       Segment duration.
%**************************************************************************
% OUTPUTS
%   - coefficients (6-by-2 numeric)
%       Ascending-power coefficients for local time in seconds.
%**************************************************************************
% UNITS
%   - Position is degrees and local time is seconds.

validateattributes(duration_s, "numeric", ...
    ["scalar", "real", "finite", "positive"], mfilename, "duration_s");

position0_deg = reshape(double(startState.position_deg), 1, 2);
velocity0_deg_s = reshape(double(startState.velocity_deg_s), 1, 2);
acceleration0_deg_s2 = reshape( ...
    double(startState.acceleration_deg_s2), 1, 2);
position1_deg = reshape(double(endState.position_deg), 1, 2);
velocity1_deg_s = reshape(double(endState.velocity_deg_s), 1, 2);
acceleration1_deg_s2 = reshape( ...
    double(endState.acceleration_deg_s2), 1, 2);

coefficients = zeros(6, 2);
coefficients(1, :) = position0_deg;
coefficients(2, :) = velocity0_deg_s;
coefficients(3, :) = acceleration0_deg_s2 ./ 2;

terminalResidual = [ ...
    position1_deg - position0_deg - velocity0_deg_s .* duration_s - ...
        0.5 .* acceleration0_deg_s2 .* duration_s.^2; ...
    velocity1_deg_s - velocity0_deg_s - ...
        acceleration0_deg_s2 .* duration_s; ...
    acceleration1_deg_s2 - acceleration0_deg_s2];
durationMatrix = [ ...
    duration_s.^3, duration_s.^4, duration_s.^5; ...
    3 .* duration_s.^2, 4 .* duration_s.^3, ...
        5 .* duration_s.^4; ...
    6 .* duration_s, 12 .* duration_s.^2, ...
        20 .* duration_s.^3];
coefficients(4:6, :) = durationMatrix \ terminalResidual;
end
