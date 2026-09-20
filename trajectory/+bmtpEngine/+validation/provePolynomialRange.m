function isWithinRange = provePolynomialRange(powerCoefficient, lowerBound, upperBound, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   isWithinRange = bmtpEngine.validation.provePolynomialRange( ...
%       powerCoefficient, lowerBound, upperBound, tolerance)
%**************************************************************************
% PURPOSE
%   - Prove a scalar polynomial range on normalized time [0, 1].
%   - Resolve easy intervals with Bernstein hulls before using stationary
%     points for cases that remain ambiguous after subdivision.
%**************************************************************************
% INPUTS
%   - powerCoefficient (finite real numeric vector)
%       Ascending-power coefficients supplied by the validated polynomial
%       trajectory path. Empty vectors are unsupported.
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

%% Section 1: Try Proven Bernstein Range Tests

powerCoefficient     = double(powerCoefficient(:));
lastCoefficientIndex = find(powerCoefficient ~= 0, 1, "last");
if isempty(lastCoefficientIndex)
    lastCoefficientIndex = 1;
end
powerCoefficient    = powerCoefficient(1:lastCoefficientIndex);
provenLowerBound = lowerBound - tolerance;
provenUpperBound = upperBound + tolerance;

endpointValues       = [powerCoefficient(1); sum(powerCoefficient)];
endpointIsOutOfRange = any(endpointValues < provenLowerBound | endpointValues > provenUpperBound);
if endpointIsOutOfRange
    isWithinRange = false;
    return
end
if numel(powerCoefficient) <= 2
    isWithinRange = true;
    return
end

bernsteinControl = convertPowerToBernstein(powerCoefficient);
% Try two subdivisions before falling back to polynomial extrema.
maximumSubdivisionDepth = 2;
decision                = classifyBernsteinRange( ...
    bernsteinControl, provenLowerBound, provenUpperBound, maximumSubdivisionDepth);
if decision ~= 0
    isWithinRange = decision > 0;
    return
end

%% Section 2: Resolve Ambiguity At Stationary Points

isWithinRange = stationaryPointsWithinBounds( ...
    powerCoefficient, provenLowerBound, provenUpperBound);
end

%% Section 3: Local Functions

function bernsteinControl = convertPowerToBernstein(powerCoefficient)
    % Convert ascending powers to same-degree Bernstein controls on [0, 1].
    % The degree-dependent basis conversion is cached across calls.
    persistent transformByCoefficientCount
    degree           = numel(powerCoefficient) - 1;
    coefficientCount = degree + 1;
    needsTransform   = isempty(transformByCoefficientCount) || ...
        numel(transformByCoefficientCount) < coefficientCount || ...
        isempty(transformByCoefficientCount{coefficientCount});
    if needsTransform
        transform = zeros(coefficientCount);
        for bernsteinIndex = 0:degree
            for powerIndex = 0:bernsteinIndex
                transform(bernsteinIndex + 1, powerIndex + 1) = ...
                    nchoosek(bernsteinIndex, powerIndex) / nchoosek(degree, powerIndex);
            end
        end
        transformByCoefficientCount{coefficientCount} = transform;
    end
    bernsteinControl = transformByCoefficientCount{coefficientCount} * powerCoefficient;
end

function decision = classifyBernsteinRange(control, lowerBound, upperBound, remainingDepth)
    % Return 1 for proven inside, -1 for proven outside, and 0 for ambiguous.
    % One outlying control is not a curve sample and cannot reject the interval.
    hullIsInside  = all(control >= lowerBound & control <= upperBound);
    hullIsOutside = max(control) < lowerBound || min(control) > upperBound;
    if hullIsInside
        decision = 1;
        return
    end
    if hullIsOutside
        decision = -1;
        return
    end
    if remainingDepth == 0
        decision = 0;
        return
    end

    % One de Casteljau restriction serves both halves of the midpoint split.
    leftControl  = bmtpEngine.motion.restrictBezier(control, [0, 0.5]);
    rightControl = bmtpEngine.motion.restrictBezier(control, [0.5, 1]);
    leftDecision = classifyBernsteinRange(leftControl, lowerBound, upperBound, remainingDepth - 1);
    if leftDecision < 0
        decision = -1;
        return
    end
    rightDecision = classifyBernsteinRange(rightControl, lowerBound, upperBound, remainingDepth - 1);
    if rightDecision < 0
        decision = -1;
    elseif leftDecision > 0 && rightDecision > 0
        decision = 1;
    else
        decision = 0;
    end
end

function isWithinRange = stationaryPointsWithinBounds(powerCoefficient, lowerBound, upperBound)
    % Fall back to endpoints and real stationary points.
    derivativeCoefficient = (1:numel(powerCoefficient) - 1).' .* powerCoefficient(2:end);
    lastDerivativeIndex   = find(derivativeCoefficient ~= 0, 1, "last");
    candidateTau          = [0; 1];
    if ~isempty(lastDerivativeIndex)
        stationaryTau  = real(roots(flip(derivativeCoefficient(1:lastDerivativeIndex))));
        rootTolerance  = 1e-9;
        stationaryTau  = stationaryTau(stationaryTau >= -rootTolerance & stationaryTau <= 1 + rootTolerance);
        candidateTau   = [candidateTau; min(max(stationaryTau, 0), 1)];
    end
    candidateValues = polyval(flip(powerCoefficient), candidateTau);
    isWithinRange   = all(candidateValues >= lowerBound & candidateValues <= upperBound);
end
