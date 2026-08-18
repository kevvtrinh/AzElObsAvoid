function [state, control] = evaluateAzElHs3Segment( ...
        solution, segmentIndex, segmentDuration_s, localTau)
%% Section 0: Header & Readme
% SYNTAX
%   [state, control] = azElInternal.evaluateAzElHs3Segment( ...
%       solution, segmentIndex, segmentDuration_s, localTau)
%**************************************************************************
% PURPOSE
%   - Evaluate one dynamically consistent generalized HS-3 segment at
%     normalized local times.
%**************************************************************************
% INPUTS
%   - solution (scalar propagated HS-3 solution struct)
%       KnotState, KnotControl, and MidpointControl contain the segment.
%   - segmentIndex (positive integer scalar)
%       Segment to evaluate.
%   - segmentDuration_s (nonnegative finite scalar)
%       Physical duration of the selected segment.
%   - localTau (finite numeric vector)
%       Normalized evaluation times from zero to one.
%**************************************************************************
% OUTPUTS
%   - state (N-by-6 numeric matrix)
%       Position, velocity, and acceleration by axis.
%   - control (N-by-2 numeric matrix)
%       Quadratic jerk by axis.
%**************************************************************************
% UNITS
%   - Position is degrees. Derivatives use deg/s, deg/s^2, and deg/s^3.
%     Time is seconds. Axis order is [azimuth elevation].
%**************************************************************************

%% Section 1: Build And Evaluate The Segment Polynomials

[statePower, controlPower] = ...
    azElInternal.buildAzElHs3SegmentPolynomials( ...
    solution.KnotState(segmentIndex, :), ...
    solution.KnotControl(segmentIndex, :), ...
    solution.MidpointControl(segmentIndex, :), ...
    solution.KnotControl(segmentIndex + 1, :), segmentDuration_s);
localTau = localTau(:);
statePowerValue = localTau .^ (0:size(statePower, 1) - 1);
controlPowerValue = localTau .^ (0:size(controlPower, 1) - 1);
state = statePowerValue * statePower;
control = controlPowerValue * controlPower;
end
