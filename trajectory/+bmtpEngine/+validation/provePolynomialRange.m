function isWithinRange = provePolynomialRange(powerCoefficients, lowerBound, upperBound, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   isWithinRange = bmtpEngine.validation.provePolynomialRange( ...
%       powerCoefficients, lowerBound, upperBound, tolerance)
%**************************************************************************
% PURPOSE
%   - Check that a scalar polynomial stays within its allowed range over
%     the whole segment, where fraction 0 is the start and 1 is the end.
%   - Try bounds from its Bezier coefficients first. If those are inconclusive,
%     evaluate endpoints and locations where the polynomial slope is zero.
%**************************************************************************
% INPUTS
%   - powerCoefficients (finite real numeric vector)
%       Constant term first: [2 3 4] represents 2 + 3u + 4u^2. The validated
%       trajectory supplies this vector; empty vectors are unsupported.
%   - lowerBound (finite real numeric scalar)
%       Inclusive lower range limit.
%   - upperBound (finite real numeric scalar)
%       Inclusive upper range limit no less than lowerBound.
%   - tolerance (nonnegative finite real numeric scalar)
%       Absolute allowance applied once to both limits.
%**************************************************************************
% OUTPUTS
%   - isWithinRange (scalar logical)
%       True only when the complete polynomial is within the tolerated range.
%**************************************************************************
% UNITS
%   - Coefficients, bounds, and tolerance share the caller's physical unit.
%     Polynomial time is dimensionless normalized time on [0, 1].
%**************************************************************************

%% Section 1: Check Endpoints And Whole-Curve Coefficient Bounds

% Remove unused high powers and apply the tolerance once. All subsequent
% checks use these same allowed lower and upper limits.

powerCoefficients    = double(powerCoefficients(:));
lastCoefficientIndex = find(powerCoefficients ~= 0, 1, "last");
if isempty(lastCoefficientIndex)
    lastCoefficientIndex = 1;
end
powerCoefficients = powerCoefficients(1:lastCoefficientIndex);
allowedLowerBound = lowerBound - tolerance;
allowedUpperBound = upperBound + tolerance;

% An out-of-range endpoint rejects any polynomial. For a constant or
% straight line, the endpoint values also determine the complete range.
endpointValues      = [powerCoefficients(1); sum(powerCoefficients)];
endpointIsOutOfRange = any(endpointValues < allowedLowerBound | endpointValues > allowedUpperBound);
if endpointIsOutOfRange
    isWithinRange = false;
    return
end
if numel(powerCoefficients) <= 2
    isWithinRange = true;
    return
end

bernsteinCoefficients = convertPowerToBernstein(powerCoefficients);
% A Bezier polynomial stays between its smallest and largest coefficients.
% Splitting into smaller intervals can tighten those bounds without changing
% the polynomial. Try up to two subdivision levels before checking extrema.
maximumSubdivisionDepth = 2;
rangeDecision           = classifyBernsteinRange( ...
    bernsteinCoefficients, allowedLowerBound, allowedUpperBound, maximumSubdivisionDepth);
if rangeDecision ~= 0
    isWithinRange = rangeDecision > 0;
    return
end

%% Section 2: Check Extrema When Coefficient Bounds Are Inconclusive

% A polynomial reaches its minimum/maximum at an endpoint or where its
% derivative is zero. Those values settle cases the coefficient bounds could
% not decide; an inconclusive bound alone is not a failure.

isWithinRange = checkEndpointAndStationaryValues( ...
    powerCoefficients, allowedLowerBound, allowedUpperBound);
end

%% Section 3: Local Functions

function bernsteinCoefficients = convertPowerToBernstein(powerCoefficients)
    % Represent the same polynomial using Bezier coefficients on [0 1].
    % This changes the coefficients, not the curve. Reuse the conversion
    % matrix for later polynomials with the same coefficient count.
    persistent conversionMapsByCoefficientCount
    degree           = numel(powerCoefficients) - 1;
    coefficientCount = degree + 1;
    conversionMapIsMissing = isempty(conversionMapsByCoefficientCount) || ...
        numel(conversionMapsByCoefficientCount) < coefficientCount || ...
        isempty(conversionMapsByCoefficientCount{coefficientCount});
    if conversionMapIsMissing
        powerToBernsteinMap = zeros(coefficientCount);
        for bernsteinIndex = 0:degree
            for powerIndex = 0:bernsteinIndex
                powerToBernsteinMap(bernsteinIndex + 1, powerIndex + 1) = ...
                    nchoosek(bernsteinIndex, powerIndex) / nchoosek(degree, powerIndex);
            end
        end
        conversionMapsByCoefficientCount{coefficientCount} = powerToBernsteinMap;
    end
    bernsteinCoefficients = conversionMapsByCoefficientCount{coefficientCount} * powerCoefficients;
end

function rangeDecision = classifyBernsteinRange( ...
        bernsteinCoefficients, lowerBound, upperBound, remainingSubdivisions)
    % Return 1 for inside, -1 for outside, and 0 when the bounds cannot decide.
    % Coefficients bound the curve but are not samples on it: one coefficient
    % beyond a limit is not enough to reject the curve.
    coefficientBoundsAreInside  = all(bernsteinCoefficients >= lowerBound & bernsteinCoefficients <= upperBound);
    coefficientBoundsAreOutside = max(bernsteinCoefficients) < lowerBound || min(bernsteinCoefficients) > upperBound;
    if coefficientBoundsAreInside
        rangeDecision = 1;
        return
    end
    if coefficientBoundsAreOutside
        rangeDecision = -1;
        return
    end
    if remainingSubdivisions == 0
        rangeDecision = 0;
        return
    end

    % Check both halves of the same curve. Reject if either half is outside;
    % accept only if both halves pass. Otherwise keep the result undecided.
    leftHalfCoefficients  = bmtpEngine.motion.restrictBezier(bernsteinCoefficients, [0, 0.5]);
    rightHalfCoefficients = bmtpEngine.motion.restrictBezier(bernsteinCoefficients, [0.5, 1]);
    leftHalfDecision = classifyBernsteinRange( ...
        leftHalfCoefficients, lowerBound, upperBound, remainingSubdivisions - 1);
    if leftHalfDecision < 0
        rangeDecision = -1;
        return
    end
    rightHalfDecision = classifyBernsteinRange( ...
        rightHalfCoefficients, lowerBound, upperBound, remainingSubdivisions - 1);
    if rightHalfDecision < 0
        rangeDecision = -1;
    elseif leftHalfDecision > 0 && rightHalfDecision > 0
        rangeDecision = 1;
    else
        rangeDecision = 0;
    end
end

function isWithinRange = checkEndpointAndStationaryValues(powerCoefficients, lowerBound, upperBound)
    % A zero derivative identifies a possible interior maximum or minimum.
    % Include both endpoints, and keep the real parts of derivative roots
    % within [0 1]. Clamp roots just beyond an endpoint by the small fraction
    % tolerance; the polynomial-value limits are unchanged.
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
    isWithinRange   = all(candidateValues >= lowerBound & candidateValues <= upperBound);
end
