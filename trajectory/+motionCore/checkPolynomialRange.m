function [within, minimumValue, maximumValue] = checkPolynomialRange(powerCoefficient, lowerBound, upperBound, tolerance)
%% Section 0: Header & Readme
% SYNTAX: [within, minimumValue, maximumValue] = motionCore.checkPolynomialRange(powerCoefficient, lowerBound, upperBound, tolerance)
% PURPOSE: Independently bound a scalar polynomial on [0,1], using Bernstein
%   hulls and two subdivisions before resolving ambiguity at stationary points.
% INPUTS: Finite ascending coefficients, inclusive bounds (possibly infinite),
%   and a nonnegative absolute tolerance applied once to both bounds.
% OUTPUTS: Complete-interval verdict; optional certified enclosing or extremal range.
% UNITS: Coefficients and bounds share physical units; polynomial time is normalized.

%% Section 1: Try Certified Bernstein Range Tests

powerCoefficient     = double(powerCoefficient(:));
lastCoefficientIndex = find(powerCoefficient ~= 0, 1, "last");
if isempty(lastCoefficientIndex)
    lastCoefficientIndex = 1;
end
powerCoefficient  = powerCoefficient(1:lastCoefficientIndex);
checkedLowerBound = lowerBound - tolerance;
checkedUpperBound = upperBound + tolerance;
% Verdict-only callers can reject endpoints without constructing a basis.
if nargout < 2
    endpoints = [powerCoefficient(1);sum(powerCoefficient)];
    within = all(endpoints >= checkedLowerBound & endpoints <= checkedUpperBound);
    if ~within || numel(powerCoefficient) <= 2
        return;
    end
elseif numel(powerCoefficient) <= 2
    endpointValue = [powerCoefficient(1); sum(powerCoefficient)];
    minimumValue  = min(endpointValue);
    maximumValue  = max(endpointValue);
    within        = minimumValue >= checkedLowerBound && maximumValue <= checkedUpperBound;
    return;
end

bernsteinControl        = convertPowerToBernstein(powerCoefficient);
maximumSubdivisionDepth = 2;
[decision, certifiedMinimum, certifiedMaximum] = classifyBernsteinRange(bernsteinControl, checkedLowerBound, checkedUpperBound, maximumSubdivisionDepth, nargout > 1);
if decision > 0 || (decision < 0 && nargout < 2)
    minimumValue = certifiedMinimum;
    maximumValue = certifiedMaximum;
    within       = decision > 0;
    return;
end

%% Section 2: Resolve Rejection Or Ambiguity At Stationary Points

% A control point outside the limits does not prove a curve violation.
% Check actual extrema if the Bernstein hull is inconclusive.
[minimumValue, maximumValue] = polynomialExtrema(powerCoefficient);
within = minimumValue >= checkedLowerBound && maximumValue <= checkedUpperBound;
end

%% Section 3: Local Functions

function bernsteinControl = convertPowerToBernstein(powerCoefficient)
    % Convert ascending powers to same-degree Bernstein controls on [0, 1].
    degree           = numel(powerCoefficient) - 1;
    coefficientCount = degree + 1;
    persistent transformByCoefficientCount
    needsTransform = isempty(transformByCoefficientCount) || numel(transformByCoefficientCount) < coefficientCount || isempty(transformByCoefficientCount{coefficientCount});
    if needsTransform
        transform = zeros(coefficientCount);
        for bernsteinIndex = 0:degree
            for powerIndex = 0:bernsteinIndex
                transform(bernsteinIndex + 1, powerIndex + 1) = nchoosek(bernsteinIndex, powerIndex) / nchoosek(degree, powerIndex);
            end
        end
        transformByCoefficientCount{coefficientCount} = transform;
    end
    bernsteinControl = transformByCoefficientCount{coefficientCount} * powerCoefficient;
end

function [decision, certifiedMinimum, certifiedMaximum] = classifyBernsteinRange(control, lowerBound, upperBound, remainingDepth, includeRange)
    % Return 1 for proven inside, -1 for proven outside, and 0 for ambiguous.
    certifiedMinimum = min(control);
    certifiedMaximum = max(control);
    if all(control >= lowerBound & control <= upperBound)
        decision = 1;
        return;
    end
    if certifiedMaximum < lowerBound || certifiedMinimum > upperBound
        decision = -1;
        return;
    end
    if remainingDepth == 0
        decision = 0;
        return;
    end
    [leftControl, rightControl]                 = subdivideAtMidpoint(control);
    [leftDecision, leftMinimum, leftMaximum]    = classifyBernsteinRange(leftControl, lowerBound, upperBound, remainingDepth - 1, includeRange);
    if leftDecision < 0 && ~includeRange
        decision = -1;
        return;
    end
    [rightDecision, rightMinimum, rightMaximum] = classifyBernsteinRange(rightControl, lowerBound, upperBound, remainingDepth - 1, includeRange);
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
    leftControl  = zeros(controlCount, 1);
    rightControl = zeros(controlCount, 1);
    work         = control;
    leftControl(1) = work(1);
    rightControl(end) = work(end);
    % Repeat the level alternatives needed to refine the current solution.
    for levelIndex = 2:controlCount
        work = 0.5 * (work(1:end - 1) + work(2:end));
        leftControl(levelIndex) = work(1);
        rightControl(end - levelIndex + 1) = work(end);
    end
end

function [minimumValue, maximumValue] = polynomialExtrema(powerCoefficient)
    % Evaluate endpoints and every finite real stationary point in [0, 1].
    derivativeCoefficient = (1:numel(powerCoefficient) - 1).' .* powerCoefficient(2:end);
    lastDerivativeIndex   = find(derivativeCoefficient ~= 0, 1, "last");
    candidateTau          = [0; 1];
    if ~isempty(lastDerivativeIndex)
        stationaryRoot = roots(flip(derivativeCoefficient(1:lastDerivativeIndex)));
        rootTolerance  = 1e-9;
        % Preserve the public validator's conservative inclusion of real parts
        % of all roots. Extra in-interval evaluations cannot weaken its verdict.
        stationaryTau  = real(stationaryRoot);
        stationaryTau  = stationaryTau(stationaryTau >= -rootTolerance & stationaryTau <= 1 + rootTolerance);
        candidateTau   = [candidateTau; ...
            min(max(stationaryTau, 0), 1)];
    end
    candidateValue = polyval(flip(powerCoefficient), candidateTau);
    minimumValue   = min(candidateValue);
    maximumValue   = max(candidateValue);
end
