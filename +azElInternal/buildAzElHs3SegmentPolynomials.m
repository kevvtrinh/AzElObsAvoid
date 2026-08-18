function [statePower, controlPower] = ...
        buildAzElHs3SegmentPolynomials( ...
        firstState, firstControl, midpointControl, lastControl, ...
        segmentDuration_s)
%% Section 0: Header & Readme
% SYNTAX
%   statePower = azElInternal.buildAzElHs3SegmentPolynomials( ...
%       firstState, firstControl, midpointControl, lastControl, ...
%       segmentDuration_s)
%   [statePower, controlPower] = ...
%       azElInternal.buildAzElHs3SegmentPolynomials( ...
%       firstState, firstControl, midpointControl, lastControl, ...
%       segmentDuration_s)
%**************************************************************************
% PURPOSE
%   - Build the single power-polynomial representation of one generalized
%     Hermite--Simpson segment with quadratic jerk control.
%**************************************************************************
% INPUTS
%   - firstState (1-by-6 finite numeric row)
%       Initial [azimuth elevation velocity acceleration] state.
%   - firstControl, midpointControl, lastControl (1-by-2 finite rows)
%       Jerk at normalized segment times 0, 1/2, and 1.
%   - segmentDuration_s (nonnegative finite scalar)
%       Physical segment duration.
%**************************************************************************
% OUTPUTS
%   - statePower (6-by-6 numeric matrix)
%       Ascending normalized-time coefficients for position, velocity, and
%       acceleration in [azimuth elevation] axis order.
%   - controlPower (3-by-2 numeric matrix)
%       Ascending normalized-time coefficients for quadratic jerk.
%**************************************************************************
% UNITS
%   - Position is degrees. Velocity, acceleration, and jerk use deg/s,
%     deg/s^2, and deg/s^3. Time is seconds.
%**************************************************************************

%% Section 1: Build Control And State Polynomials

% Moreno-Martin, Ros, and Celaya (2024), generalized HS-M Equations 51 and
% 57-60, are specialized here to M=3. The normalized-time coefficients use
% the corrected time powers from the Kelly Equation 4.13 erratum. See
% citation.md.
controlPower = [firstControl; ...
    -3 * firstControl + 4 * midpointControl - lastControl; ...
    2 * firstControl - 4 * midpointControl + 2 * lastControl];
jerk0 = controlPower(1, :);
jerk1 = controlPower(2, :);
jerk2 = controlPower(3, :);
h = segmentDuration_s;
statePower = zeros(6, 6);
statePower(1, :) = firstState;
statePower(2, 1:2) = h * firstState(3:4);
statePower(3, 1:2) = h ^ 2 * firstState(5:6) / 2;
statePower(4, 1:2) = h ^ 3 * jerk0 / 6;
statePower(5, 1:2) = h ^ 3 * jerk1 / 24;
statePower(6, 1:2) = h ^ 3 * jerk2 / 60;
statePower(2, 3:4) = h * firstState(5:6);
statePower(3, 3:4) = h ^ 2 * jerk0 / 2;
statePower(4, 3:4) = h ^ 2 * jerk1 / 6;
statePower(5, 3:4) = h ^ 2 * jerk2 / 12;
statePower(2, 5:6) = h * jerk0;
statePower(3, 5:6) = h * jerk1 / 2;
statePower(4, 5:6) = h * jerk2 / 3;
end
