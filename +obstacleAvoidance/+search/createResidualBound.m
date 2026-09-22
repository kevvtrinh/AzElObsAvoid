function edgeSideTolerance_units2 = createResidualBound(coordinateScale_units)
%% Section 0: Header & Readme
% SYNTAX
%   edgeSideTolerance_units2 = ...
%       obstacleAvoidance.search.createResidualBound(coordinateScale_units)
%**************************************************************************
% PURPOSE
%   - Set the shared rounding allowance for deciding which side of a moving
%     edge contains a point. Collision tests and their quick exclusion checks
%     need the same tolerance so they agree near obstacle boundaries.
%**************************************************************************
% INPUTS
%   - coordinateScale_units (numeric scalar or column)
%       Largest absolute coordinate used in each side calculation.
%**************************************************************************
% OUTPUTS
%   - edgeSideTolerance_units2 (same shape as the input)
%       Numerical tolerance for edge-vector x point-offset cross products.
%**************************************************************************
% UNITS
%   - Squared coordinate units.
%**************************************************************************

%% Section 1: Scale The Rounding Allowance

% A cross product multiplies two coordinate differences, giving squared
% coordinate units. eps measures the spacing of representable numbers at
% that scale; the shared factor allows for rounding across the calculation.
edgeSideTolerance_units2 = 4096 * eps(coordinateScale_units .^ 2);
end
