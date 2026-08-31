function [inequality, equality] = evaluatePolynomialConstraints( ...
        polynomial, terminalState, limits, pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   [inequality, equality] = ...
%       ruckigEngine.internal.evaluatePolynomialConstraints( ...
%       polynomial, terminalState, limits, pathConstraints)
%**************************************************************************
% PURPOSE
%   - Certify continuous derivative, position, affine path, and endpoint
%     constraints from the common polynomial format.
%**************************************************************************
% INPUTS
%   - polynomial (scalar struct)
%       Segment durations and ascending-power derivative coefficients.
%   - terminalState (scalar struct)
%       Required terminal position, velocity, and acceleration rows.
%   - limits (resolved scalar struct)
%       Position-through-jerk lower and upper coordinate bounds.
%   - pathConstraints (resolved scalar struct)
%       Affine point or single-segment interval inequalities.
%**************************************************************************
% OUTPUTS
%   - inequality (numeric column), feasible when every value is <= 0.
%   - equality (3D-by-1 numeric column), terminal state residuals.
%**************************************************************************
% UNITS
%   - Values retain the caller's consistent coordinate and time units.
%**************************************************************************

%% Section 1: Evaluate Complete Polynomial Bounds

inequality = continuousBoundConstraints(polynomial, limits);
if ~isempty(pathConstraints.Tau)
    inequality = [inequality; ...
        affinePathConstraints(polynomial, pathConstraints)];
end
terminal = polynomial.TerminalState;
equality = [ ...
    terminal.position - terminalState.position, ...
    terminal.velocity - terminalState.velocity, ...
    terminal.acceleration - terminalState.acceleration].';
end

%% Section 2: Local Functions

function inequality = continuousBoundConstraints(polynomial, limits)
% Bound every complete segment through its exact Bernstein convex hull.
coefficientFields = [ ...
    "positionPower", "velocityPower", ...
    "accelerationPower", "jerkPower"];
lowerFields = [ ...
    "positionLower", "velocityLower", ...
    "accelerationLower", "jerkLower"];
upperFields = [ ...
    "positionUpper", "velocityUpper", ...
    "accelerationUpper", "jerkUpper"];
dimensionCount = size(polynomial.positionPower, 2);
inequality = zeros(0, 1);
for dimensionIndex = 1:dimensionCount
    for quantityIndex = 1:numel(coefficientFields)
        coefficientArray = polynomial.(coefficientFields(quantityIndex));
        bernstein = coordinateBernstein( ...
            coefficientArray, dimensionIndex);
        upperBounds = limits.(upperFields(quantityIndex));
        lowerBounds = limits.(lowerFields(quantityIndex));
        upperBound = upperBounds(dimensionIndex);
        lowerBound = lowerBounds(dimensionIndex);
        if isfinite(upperBound)
            inequality = [inequality; bernstein - upperBound]; %#ok<AGROW>
        end
        if isfinite(lowerBound)
            inequality = [inequality; lowerBound - bernstein]; %#ok<AGROW>
        end
    end
end
end

function inequality = affinePathConstraints(polynomial, pathConstraints)
% Restrict each requested interval and certify its projected Bernstein hull.
coefficientCount = size(polynomial.positionPower, 3);
segmentCount = polynomial.SegmentCount;
inequality = zeros(0, 1);
for constraintIndex = 1:numel(pathConstraints.Tau)
    scaledStart = segmentCount * pathConstraints.Tau(constraintIndex);
    scaledEnd = segmentCount * pathConstraints.TauEnd(constraintIndex);
    firstSegmentIndex = min(segmentCount, floor(scaledStart) + 1);
    lastSegmentIndex = firstSegmentIndex;
    if scaledEnd > scaledStart
        lastSegmentIndex = min(segmentCount, ceil(scaledEnd));
    end
    for segmentIndex = firstSegmentIndex:lastSegmentIndex
        localStart = min(1, max(0, scaledStart - segmentIndex + 1));
        localEnd = min(1, max(0, scaledEnd - segmentIndex + 1));
        hullMap = createSubintervalBernsteinMap( ...
            localStart, localEnd, coefficientCount);
        segmentPower = reshape(polynomial.positionPower( ...
            segmentIndex, :, :), [], coefficientCount);
        projectedPower = ...
            pathConstraints.Normal(constraintIndex, :) * segmentPower;
        projectionHull = hullMap * projectedPower.';
        rowCount = 1 + (coefficientCount - 1) * (scaledEnd > scaledStart);
        inequality = [inequality; pathConstraints.LowerBound( ...
            constraintIndex) - projectionHull(1:rowCount)]; %#ok<AGROW>
    end
end
end

function bernstein = coordinateBernstein( ...
        coefficientArray, dimensionIndex)
% Convert every segment for one coordinate while preserving row order.
segmentCount = size(coefficientArray, 1);
coefficientCount = size(coefficientArray, 3);
powerMatrix = reshape(permute( ...
    coefficientArray(:, dimensionIndex, :), [3, 1, 2]), ...
    coefficientCount, segmentCount);
bernstein = convertPowerToBernstein(powerMatrix);
bernstein = bernstein(:);
end

function coefficient = convertPowerToBernstein(powerCoefficient)
% Deliberately independent from obstacle-side conversions so one arithmetic
% error cannot make motion production and certification agree.
% Convert ascending powers to same-degree Bernstein coefficients on [0,1].
powerCoefficient = double(powerCoefficient);
if isvector(powerCoefficient)
    powerCoefficient = powerCoefficient(:);
end
degree = size(powerCoefficient, 1) - 1;
persistent conversionMatrixByDegree
if numel(conversionMatrixByDegree) > degree && ...
        ~isempty(conversionMatrixByDegree{degree + 1})
    coefficient = ...
        conversionMatrixByDegree{degree + 1} * powerCoefficient;
    return;
end
conversionMatrix = pascal(degree + 1, 1);
conversionMatrix = conversionMatrix ./ conversionMatrix(end, :);
conversionMatrixByDegree{degree + 1} = conversionMatrix;
coefficient = conversionMatrix * powerCoefficient;
end

function hullMap = createSubintervalBernsteinMap( ...
        localStart, localEnd, coefficientCount)
% Combine interval restriction with power-to-Bernstein basis conversion.
localSpan = max(0, localEnd - localStart);
degree = coefficientCount - 1;
sourceExponent = 0:degree;
targetExponent = (0:degree).';
shiftExponent = sourceExponent - targetExponent;
binomialWeight = zeros(coefficientCount);
for targetIndex = 0:degree
    for sourceIndex = targetIndex:degree
        binomialWeight(targetIndex + 1, sourceIndex + 1) = ...
            nchoosek(sourceIndex, targetIndex);
    end
end
restriction = binomialWeight .* ...
    localStart .^ max(shiftExponent, 0) .* ...
    localSpan .^ targetExponent;
hullMap = convertPowerToBernstein(eye(coefficientCount)) * restriction;
end
