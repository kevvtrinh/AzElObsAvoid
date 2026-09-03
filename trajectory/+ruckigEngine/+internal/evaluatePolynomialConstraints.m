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
    terminal.velocity - terminalState.velocity].';
if limits.ControlOrder == 3
    equality = [equality; ...
        (terminal.acceleration - terminalState.acceleration).'];
end
end

%% Section 2: Local Functions

function inequality = continuousBoundConstraints(polynomial, limits)
% Check every scalar segment without treating hull controls as curve samples.
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
        upperBounds = limits.(upperFields(quantityIndex));
        lowerBounds = limits.(lowerFields(quantityIndex));
        upperBound = upperBounds(dimensionIndex);
        lowerBound = lowerBounds(dimensionIndex);
        for segmentIndex = 1:polynomial.SegmentCount
            powerCoefficient = reshape(coefficientArray( ...
                segmentIndex, dimensionIndex, :), [], 1);
            [~, minimumValue, maximumValue] = ...
                ruckigEngine.internal.checkPolynomialRange( ...
                powerCoefficient, lowerBound, upperBound, 0);
            if isfinite(upperBound)
                inequality(end + 1, 1) = ...
                    maximumValue - upperBound; %#ok<AGROW>
            end
            if isfinite(lowerBound)
                inequality(end + 1, 1) = ...
                    lowerBound - minimumValue; %#ok<AGROW>
            end
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
        restriction = createSubintervalPowerMap( ...
            localStart, localEnd, coefficientCount);
        segmentPower = reshape(polynomial.positionPower( ...
            segmentIndex, :, :), [], coefficientCount);
        projectedPower = ...
            pathConstraints.Normal(constraintIndex, :) * segmentPower;
        restrictedPower = restriction * projectedPower.';
        [~, minimumValue] = ...
            ruckigEngine.internal.checkPolynomialRange( ...
            restrictedPower, ...
            pathConstraints.LowerBound(constraintIndex), Inf, 0);
        inequality(end + 1, 1) = ...
            pathConstraints.LowerBound(constraintIndex) - ...
            minimumValue; %#ok<AGROW>
    end
end
end

function restriction = createSubintervalPowerMap( ...
        localStart, localEnd, coefficientCount)
% Express a source power polynomial on one normalized subinterval.
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
end
