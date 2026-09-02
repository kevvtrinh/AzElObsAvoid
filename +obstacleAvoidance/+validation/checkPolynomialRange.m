function within = checkPolynomialRange( ...
        powerCoefficient, lowerBound, upperBound, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   within = obstacleAvoidance.validation.checkPolynomialRange( ...
%       powerCoefficient, lowerBound, upperBound, tolerance)
%**************************************************************************
% PURPOSE
%   - Check a scalar polynomial over normalized time against inclusive limits.
%   - Use certified Bernstein range tests and the reliable stationary-point
%     fallback owned by the established implementation.
%**************************************************************************
% INPUTS
%   - powerCoefficient (finite real numeric vector)
%       Ascending-power coefficients on normalized time [0, 1].
%   - lowerBound, upperBound (finite real numeric scalars)
%       Inclusive limits with lowerBound no greater than upperBound.
%   - tolerance (nonnegative finite scalar)
%       Absolute allowance applied once to both limits.
%**************************************************************************
% OUTPUTS
%   - within (scalar logical)
%       True only when the complete polynomial stays within the limits.
%**************************************************************************
% UNITS
%   - Coefficients, bounds, and tolerance share the caller's physical unit.
%**************************************************************************

%% Section 1: Run The Certified Range Check

% The implementation treats one outlying Bernstein coefficient as ambiguous,
% not as an exact rejection. It recursively subdivides easy intervals and
% retains stationary-point evaluation when the hull tests remain ambiguous.

within = obstacleAvoidance.validation.certifyPolynomialRange( ...
    powerCoefficient, lowerBound, upperBound, tolerance);
end
