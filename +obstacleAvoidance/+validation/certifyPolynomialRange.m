function within = certifyPolynomialRange( ...
        powerCoefficient, lowerBound, upperBound, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   within = obstacleAvoidance.validation.certifyPolynomialRange( ...
%       powerCoefficient, lowerBound, upperBound, tolerance)
%**************************************************************************
% PURPOSE
%   - Certify a scalar polynomial range on normalized time [0, 1].
%   - Resolve easy intervals with Bernstein hulls before using stationary
%     points for cases that remain ambiguous after subdivision.
%**************************************************************************
% INPUTS
%   - powerCoefficient (finite real numeric vector)
%       Ascending-power coefficients supplied by the validated polynomial
%       trajectory path. Empty vectors are unsupported.
%   - lowerBound, upperBound (finite real numeric scalars)
%       Inclusive range limits with lowerBound no greater than upperBound.
%   - tolerance (nonnegative finite real numeric scalar)
%       Absolute allowance applied once to both limits.
%**************************************************************************
% OUTPUTS
%   - within (scalar logical)
%       True only when the complete polynomial is within the tolerated range.
%**************************************************************************
% UNITS
%   - Coefficients, bounds, and tolerance share the caller's physical unit.
%     Polynomial time is dimensionless normalized time on [0, 1].
%**************************************************************************

%% Section 1: Try Certified Bernstein Range Tests

powerCoefficient = double(powerCoefficient(:));
lastCoefficientIndex = find(powerCoefficient ~= 0, 1, "last");
if isempty(lastCoefficientIndex)
    lastCoefficientIndex = 1;
end
powerCoefficient = powerCoefficient(1:lastCoefficientIndex);
certifiedLowerBound = lowerBound - tolerance;
certifiedUpperBound = upperBound + tolerance;

endpointValue = [powerCoefficient(1); sum(powerCoefficient)];
if any(endpointValue < certifiedLowerBound | ...
        endpointValue > certifiedUpperBound)
    within = false;
    return;
end
if numel(powerCoefficient) <= 2
    within = true;
    return;
end

bernsteinControl = convertPowerToBernstein(powerCoefficient);
% Two bisections expose four tighter hulls while keeping an unsuccessful
% proof cheaper than allowing recursive work to compete with the fallback.
maximumSubdivisionDepth = 2;
decision = classifyBernsteinRange(bernsteinControl, ...
    certifiedLowerBound, certifiedUpperBound, maximumSubdivisionDepth);
if decision ~= 0
    within = decision > 0;
    return;
end

%% Section 2: Resolve Ambiguity At Stationary Points

within = stationaryPointsWithinBounds(powerCoefficient, ...
    certifiedLowerBound, certifiedUpperBound);
end

%% Section 3: Local Functions

function bernsteinControl = convertPowerToBernstein(powerCoefficient)
% Convert ascending powers to same-degree Bernstein controls on [0, 1].
degree = numel(powerCoefficient) - 1;
persistent transformByCoefficientCount
coefficientCount = degree + 1;
needsTransform = isempty(transformByCoefficientCount) || ...
    numel(transformByCoefficientCount) < coefficientCount || ...
    isempty(transformByCoefficientCount{coefficientCount});
if needsTransform
    % The basis map depends only on degree; rebuilding it for every trajectory
    % segment would make the proof path more expensive than root evaluation.
    transform = zeros(coefficientCount);
    for bernsteinIndex = 0:degree
        for powerIndex = 0:bernsteinIndex
            transform(bernsteinIndex + 1, powerIndex + 1) = ...
                nchoosek(bernsteinIndex, powerIndex) / ...
                nchoosek(degree, powerIndex);
        end
    end
    transformByCoefficientCount{coefficientCount} = transform;
end
bernsteinControl = ...
    transformByCoefficientCount{coefficientCount} * powerCoefficient;
end

function decision = classifyBernsteinRange( ...
        control, lowerBound, upperBound, remainingDepth)
% Return 1 for proven inside, -1 for proven outside, and 0 for ambiguous.
% One outlying control is not a curve sample and cannot reject the interval.
if all(control >= lowerBound & control <= upperBound)
    decision = 1;
    return;
end
if max(control) < lowerBound || min(control) > upperBound
    decision = -1;
    return;
end
if remainingDepth == 0
    decision = 0;
    return;
end

[leftControl, rightControl] = subdivideAtMidpoint(control);
leftDecision = classifyBernsteinRange( ...
    leftControl, lowerBound, upperBound, remainingDepth - 1);
if leftDecision < 0
    decision = -1;
    return;
end
rightDecision = classifyBernsteinRange( ...
    rightControl, lowerBound, upperBound, remainingDepth - 1);
if rightDecision < 0
    decision = -1;
elseif leftDecision > 0 && rightDecision > 0
    decision = 1;
else
    decision = 0;
end
end

function [leftControl, rightControl] = subdivideAtMidpoint(control)
% Apply de Casteljau subdivision without evaluating the power polynomial.
controlCount = numel(control);
leftControl = zeros(controlCount, 1);
rightControl = zeros(controlCount, 1);
work = control;
leftControl(1) = work(1);
rightControl(end) = work(end);
for levelIndex = 2:controlCount
    work = 0.5 * (work(1:end - 1) + work(2:end));
    leftControl(levelIndex) = work(1);
    rightControl(end - levelIndex + 1) = work(end);
end
end

function within = stationaryPointsWithinBounds( ...
        powerCoefficient, lowerBound, upperBound)
% Retain the established endpoint and finite stationary-point fallback.
derivativeCoefficient = (1:numel(powerCoefficient) - 1).' .* ...
    powerCoefficient(2:end);
lastDerivativeIndex = find(derivativeCoefficient ~= 0, 1, "last");
candidateTau = [0; 1];
if ~isempty(lastDerivativeIndex)
    stationaryTau = real(roots(flip( ...
        derivativeCoefficient(1:lastDerivativeIndex))));
    rootTolerance = 1e-9;
    stationaryTau = stationaryTau(stationaryTau >= -rootTolerance & ...
        stationaryTau <= 1 + rootTolerance);
    candidateTau = [candidateTau; min(max(stationaryTau, 0), 1)];
end
candidateValue = polyval(flip(powerCoefficient), candidateTau);
within = all(candidateValue >= lowerBound & candidateValue <= upperBound);
end
