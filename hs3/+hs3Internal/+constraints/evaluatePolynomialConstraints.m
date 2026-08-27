function [inequality, equality] = evaluatePolynomialConstraints( ...
        polynomial, terminalState, limits, pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   [inequality, equality] = ...
%       hs3Internal.constraints.evaluatePolynomialConstraints( ...
%       polynomial, terminalState, limits, pathConstraints)
%**************************************************************************
% PURPOSE
%   - Evaluate complete continuous kinematic, affine path, and terminal-state
%     constraints directly from a dimension-neutral polynomial record.
%**************************************************************************
% INPUTS
%   - polynomial (scalar HS3 polynomial struct)
%       Segment durations may be scalar or one positive value per segment.
%   - terminalState (scalar struct)
%       Required position, velocity, and acceleration are 1-by-D rows.
%   - limits (resolved scalar struct)
%       Position-through-jerk lower and upper bounds are 1-by-D rows.
%   - pathConstraints (resolved scalar struct)
%       Tau/TauEnd are M-by-1, Normal is M-by-D, and LowerBound is M-by-1.
%**************************************************************************
% OUTPUTS
%   - inequality (numeric column), feasible when every entry is <= 0.
%   - equality (3D-by-1 numeric column), terminal P/V/A residuals.
%**************************************************************************
% UNITS
%   - Values retain caller-defined consistent coordinate and time units.
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

function inequality = affinePathConstraints(polynomial, pathConstraints)
% Bound each projected point or interval through its Bernstein hull.
coefficientCount = size(polynomial.positionPower, 3);
[segmentIndex, hullMap] = ...
    hs3Internal.polynomial.createSubintervalBernsteinMap( ...
    pathConstraints.Tau, pathConstraints.TauEnd, ...
    polynomial.SegmentCount, coefficientCount);
isInterval = pathConstraints.TauEnd > pathConstraints.Tau;
rowCounts = 1 + (coefficientCount - 1) * isInterval;
inequality = zeros(sum(rowCounts), 1);
nextRow = 1;
for constraintIndex = 1:numel(pathConstraints.Tau)
    rowCount = rowCounts(constraintIndex);
    rows = nextRow:nextRow + rowCount - 1;
    nextRow = rows(end) + 1;
    segmentPower = reshape(polynomial.positionPower( ...
        segmentIndex(constraintIndex), :, :), [], coefficientCount);
    projectionHull = hullMap(:, :, constraintIndex) * ...
        (pathConstraints.Normal(constraintIndex, :) * segmentPower).';
    inequality(rows) = pathConstraints.LowerBound(constraintIndex) - ...
        projectionHull(1:rowCount);
end
end

function inequality = continuousBoundConstraints(polynomial, limits)
% Convert every complete-segment derivative bound into Bernstein residuals.
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

function bernstein = coordinateBernstein(coefficientArray, dimensionIndex)
% Convert every segment of one coordinate while preserving constraint order.
segmentCount = size(coefficientArray, 1);
coefficientCount = size(coefficientArray, 3);
powerMatrix = reshape(permute( ...
    coefficientArray(:, dimensionIndex, :), [3 1 2]), ...
    coefficientCount, segmentCount);
bernstein = hs3Internal.polynomial.convertPowerToBernstein(powerMatrix);
bernstein = bernstein(:);
end
