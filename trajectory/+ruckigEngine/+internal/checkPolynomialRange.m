function [within, minimumValue, maximumValue] = checkPolynomialRange( ...
        powerCoefficient, lowerBound, upperBound, tolerance)
%% Section 0: Header & Readme
% SYNTAX
%   [within, minimumValue, maximumValue] = ...
%       ruckigEngine.internal.checkPolynomialRange( ...
%       powerCoefficient, lowerBound, upperBound, tolerance)
%**************************************************************************
% PURPOSE
%   - Check one scalar power polynomial over normalized time [0, 1].
%   - Use Bernstein hulls only as certificates and retain exact stationary
%     points when a coefficient hull remains ambiguous.
%**************************************************************************
% INPUTS
%   - powerCoefficient (finite real numeric vector)
%       Ascending-power coefficients on normalized time [0, 1].
%   - lowerBound, upperBound (real numeric scalars)
%       Inclusive limits; either limit may be infinite.
%   - tolerance (nonnegative finite numeric scalar)
%       Absolute allowance applied once to both limits.
%**************************************************************************
% OUTPUTS
%   - within (scalar logical)
%       True only when the complete polynomial stays within the limits.
%   - minimumValue, maximumValue (numeric scalars)
%       Certified enclosing values on a Bernstein fast path or exact extrema
%       from endpoints and stationary points on an ambiguous path.
%**************************************************************************
% UNITS
%   - Coefficients, bounds, tolerance, and returned values share units.
%**************************************************************************

%% Section 1: Try Certified Bernstein Range Tests

powerCoefficient = double(powerCoefficient(:));
lastCoefficientIndex = find(powerCoefficient ~= 0, 1, "last");
if isempty(lastCoefficientIndex)
    lastCoefficientIndex = 1;
end
powerCoefficient = powerCoefficient(1:lastCoefficientIndex);
checkedLowerBound = lowerBound - tolerance;
checkedUpperBound = upperBound + tolerance;

if numel(powerCoefficient) <= 2
    endpointValue = [powerCoefficient(1); sum(powerCoefficient)];
    minimumValue = min(endpointValue);
    maximumValue = max(endpointValue);
    within = minimumValue >= checkedLowerBound && ...
        maximumValue <= checkedUpperBound;
    return;
end

bernsteinControl = convertPowerToBernstein(powerCoefficient);
maximumSubdivisionDepth = 2;
[decision, certifiedMinimum, certifiedMaximum] = ...
    classifyBernsteinRange(bernsteinControl, ...
    checkedLowerBound, checkedUpperBound, maximumSubdivisionDepth);
if decision > 0
    minimumValue = certifiedMinimum;
    maximumValue = certifiedMaximum;
    within = true;
    return;
end

%% Section 2: Resolve Rejection Or Ambiguity At Stationary Points

% An outlying Bernstein coefficient is not a curve sample. Even when a hull
% cannot prove inclusion, evaluate all true extrema before rejecting it.
[minimumValue, maximumValue] = polynomialExtrema(powerCoefficient);
within = minimumValue >= checkedLowerBound && ...
    maximumValue <= checkedUpperBound;
end

%% Section 3: Local Functions

function bernsteinControl = convertPowerToBernstein(powerCoefficient)
% Convert ascending powers to same-degree Bernstein controls on [0, 1].
degree = numel(powerCoefficient) - 1;
coefficientCount = degree + 1;
persistent transformByCoefficientCount
needsTransform = isempty(transformByCoefficientCount) || ...
    numel(transformByCoefficientCount) < coefficientCount || ...
    isempty(transformByCoefficientCount{coefficientCount});
if needsTransform
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

function [decision, certifiedMinimum, certifiedMaximum] = ...
        classifyBernsteinRange( ...
        control, lowerBound, upperBound, remainingDepth)
% Return 1 for proven inside, -1 for proven outside, and 0 for ambiguous.
certifiedMinimum = min(control);
certifiedMaximum = max(control);
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
[leftDecision, leftMinimum, leftMaximum] = classifyBernsteinRange( ...
    leftControl, lowerBound, upperBound, remainingDepth - 1);
[rightDecision, rightMinimum, rightMaximum] = classifyBernsteinRange( ...
    rightControl, lowerBound, upperBound, remainingDepth - 1);
certifiedMinimum = min(leftMinimum, rightMinimum);
certifiedMaximum = max(leftMaximum, rightMaximum);
if leftDecision < 0 || rightDecision < 0
    decision = -1;
elseif leftDecision > 0 && rightDecision > 0
    decision = 1;
else
    decision = 0;
end
end

function [leftControl, rightControl] = subdivideAtMidpoint(control)
% Restrict one Bernstein polynomial to its two exact half intervals.
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

function [minimumValue, maximumValue] = polynomialExtrema(powerCoefficient)
% Evaluate endpoints and every finite real stationary point in [0, 1].
derivativeCoefficient = (1:numel(powerCoefficient) - 1).' .* ...
    powerCoefficient(2:end);
lastDerivativeIndex = find(derivativeCoefficient ~= 0, 1, "last");
candidateTau = [0; 1];
if ~isempty(lastDerivativeIndex)
    stationaryRoot = roots(flip( ...
        derivativeCoefficient(1:lastDerivativeIndex)));
    rootTolerance = 1e-9;
    isReal = abs(imag(stationaryRoot)) <= rootTolerance * ...
        max(1, abs(real(stationaryRoot)));
    stationaryTau = real(stationaryRoot(isReal));
    stationaryTau = stationaryTau( ...
        stationaryTau >= -rootTolerance & ...
        stationaryTau <= 1 + rootTolerance);
    candidateTau = [candidateTau; ...
        min(max(stationaryTau, 0), 1)];
end
candidateValue = polyval(flip(powerCoefficient), candidateTau);
minimumValue = min(candidateValue);
maximumValue = max(candidateValue);
end
