function [minimumValue, maximumValue] = boundPolynomialRange(powerCoefficients)
%% Section 0: Header & Readme
% SYNTAX
%   [minimumValue, maximumValue] = bmtpEngine.validation.boundPolynomialRange(powerCoefficients)
%**************************************************************************
% PURPOSE
%   - Find the exact minimum and maximum of a scalar polynomial over one
%     segment, using its endpoints and places where its slope is zero.
%**************************************************************************
% INPUTS
%   - powerCoefficients (finite real numeric vector)
%       Constant term first: [2 3 4] represents 2 + 3u + 4u^2.
%**************************************************************************
% OUTPUTS
%   - minimumValue, maximumValue (real numeric scalars)
%       Smallest and largest polynomial values for 0 <= u <= 1.
%**************************************************************************
% UNITS
%   - Values have the coefficient's physical unit; segment fraction u is
%     dimensionless.
%**************************************************************************

%% Section 1: Check Coefficients

validateattributes(powerCoefficients, {'numeric'}, {'vector', 'nonempty', 'real', 'finite'});
powerCoefficients = double(powerCoefficients(:));

%% Section 2: Evaluate Endpoints And Stationary Fractions

% A zero derivative identifies a possible interior maximum or minimum.
% Include both endpoints. Clamp roots just beyond an endpoint by the small
% fraction tolerance; the polynomial values themselves have no tolerance.
derivativeCoefficients = (1:numel(powerCoefficients) - 1).' .* powerCoefficients(2:end);
lastDerivativeIndex    = find(derivativeCoefficients ~= 0, 1, "last");
candidateFractions     = [0; 1];
if ~isempty(lastDerivativeIndex)
    stationaryFractions = real(roots(flip(derivativeCoefficients(1:lastDerivativeIndex))));
    fractionTolerance   = 1e-9;
    stationaryFractions = stationaryFractions( ...
        stationaryFractions >= -fractionTolerance & stationaryFractions <= 1 + fractionTolerance);
    candidateFractions = [candidateFractions; min(max(stationaryFractions, 0), 1)];
end
candidateValues = polyval(flip(powerCoefficients), candidateFractions);
minimumValue    = min(candidateValues);
maximumValue    = max(candidateValues);
end
