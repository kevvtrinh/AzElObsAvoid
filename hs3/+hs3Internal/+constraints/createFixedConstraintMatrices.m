function [inequalityMatrix, equalityMatrix] = ...
        createFixedConstraintMatrices( ...
        segmentCount, duration, dimensionCount, limits, pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   [inequalityMatrix, equalityMatrix] = ...
%       hs3Internal.constraints.createFixedConstraintMatrices(segmentCount, duration, ...
%       dimensionCount, limits, pathConstraints)
%**************************************************************************
% PURPOSE
%   - Assemble exact fixed-time HS3 jerk Jacobians for continuous kinematic
%     bounds, terminal states, and coordinate-space affine path constraints.
%**************************************************************************
% INPUTS
%   - segmentCount (positive integer scalar), equal-duration segments.
%   - duration (positive finite scalar), complete trajectory duration.
%   - dimensionCount (positive integer scalar), modeled coordinates.
%   - limits (resolved scalar limit struct), finite or infinite bounds.
%   - pathConstraints (scalar struct)
%       Normalized Tau/TauEnd/Normal/LowerBound point or interval rows.
%**************************************************************************
% OUTPUTS
%   - inequalityMatrix (M-by-V numeric), exact dc/djerk matrix.
%   - equalityMatrix ((3D)-by-V numeric), terminal P/V/A Jacobian.
%**************************************************************************
% UNITS
%   - Rows retain their derivative's caller-defined physical units.
%**************************************************************************

%% Section 1: Assemble Ordered Constraint Blocks

% For a fixed duration, position through jerk are affine functions of the
% jerk controls. This routine creates only the derivatives of those functions.
% evaluateConstraints supplies the constant offsets from the initial state and
% the limits. The row order must match the row order in evaluateConstraints.
% quadprog uses both parts to form A*x <= b and Aeq*x = beq.

sensitivity = hs3Internal.polynomial.createAffineSensitivityModel( ...
    segmentCount, duration, pathConstraints.Tau);
controlCount = sensitivity.ControlCount;
inequalityMatrix = zeros(0, dimensionCount * controlCount);
mapFields = [ ...
    "positionPowerMap", "velocityPowerMap", ...
    "accelerationPowerMap", "jerkPowerMap"];
lowerFields = [ ...
    "positionLower", "velocityLower", ...
    "accelerationLower", "jerkLower"];
upperFields = [ ...
    "positionUpper", "velocityUpper", ...
    "accelerationUpper", "jerkUpper"];
for dimensionIndex = 1:dimensionCount
    % Coordinates are independent until a path-plane normal combines them.
    % Place each scalar-coordinate map in its coordinate-major decision block.
    columnIndex = (dimensionIndex - 1) * controlCount + (1:controlCount);
    for quantityIndex = 1:numel(mapFields)
        bernsteinMap = mapToBernstein(sensitivity.(mapFields(quantityIndex)));
        % Every Bernstein coefficient lies on the same side of a bound only
        % if the complete polynomial segment does. This test can reject a
        % feasible polynomial. It prevents an unchecked limit crossing between
        % samples.
        upperBounds = limits.(upperFields(quantityIndex));
        lowerBounds = limits.(lowerFields(quantityIndex));
        upperBound = upperBounds(dimensionIndex);
        lowerBound = lowerBounds(dimensionIndex);
        if isfinite(upperBound)
            newRows = zeros(size(bernsteinMap, 1), ...
                dimensionCount * controlCount);
            newRows(:, columnIndex) = bernsteinMap;
            inequalityMatrix = [inequalityMatrix; newRows]; %#ok<AGROW>
        end
        if isfinite(lowerBound)
            newRows = zeros(size(bernsteinMap, 1), ...
                dimensionCount * controlCount);
            newRows(:, columnIndex) = -bernsteinMap;
            inequalityMatrix = [inequalityMatrix; newRows]; %#ok<AGROW>
        end
    end
end
pathMatrix = pathConstraintMatrix( ...
    sensitivity, pathConstraints, dimensionCount);
inequalityMatrix = [inequalityMatrix; pathMatrix];
terminalMap = sensitivity.terminalStateMap;
% Terminal position, velocity, and acceleration are equalities. Initial-state
% contributions are constants and therefore do not appear in this Jacobian.
equalityMatrix = zeros(3 * dimensionCount, dimensionCount * controlCount);
for dimensionIndex = 1:dimensionCount
    columnIndex = (dimensionIndex - 1) * controlCount + (1:controlCount);
    rowIndex = dimensionIndex + (0:2) * dimensionCount;
    equalityMatrix(rowIndex, columnIndex) = terminalMap;
end
end

%% Section 2: Local Functions

function bernsteinMap = mapToBernstein(powerMap)
% Convert coefficient sensitivities through the exact Bernstein basis.
% The first reshape batches every segment/control combination as a polynomial
% column. The final reshape restores one constraint row per Bernstein value.
coefficientCount = size(powerMap, 2);
segmentCount = size(powerMap, 1);
controlCount = size(powerMap, 3);
powerMatrix = reshape(permute(powerMap, [2 1 3]), ...
    coefficientCount, []);
bernsteinMatrix = hs3Internal.polynomial.convertPowerToBernstein(powerMatrix);
bernsteinMap = reshape( ...
    bernsteinMatrix, coefficientCount * segmentCount, controlCount);
end

function matrix = pathConstraintMatrix( ...
        sensitivity, pathConstraints, dimensionCount)
% Differentiate point or complete-interval affine path half-spaces.
% A point contributes one row. An interval contributes one row per Bernstein
% coefficient of the restricted position polynomial. The leading minus sign
% changes Normal*position >= LowerBound into the solver form c(x) <= 0.
controlCount = sensitivity.ControlCount;
coefficientCount = size(sensitivity.positionPowerMap, 2);
constraintCount = numel(pathConstraints.Tau);
if constraintCount == 0
    matrix = zeros(0, dimensionCount * controlCount);
    return;
end
[segmentIndex, hullMap] = hs3Internal.polynomial.createSubintervalBernsteinMap( ...
    pathConstraints.Tau, pathConstraints.TauEnd, ...
    size(sensitivity.positionPowerMap, 1), coefficientCount);
isInterval = pathConstraints.TauEnd > pathConstraints.Tau;
rowCounts = 1 + (coefficientCount - 1) * isInterval;
matrix = zeros(sum(rowCounts), dimensionCount * controlCount);
nextRow = 1;
for constraintIndex = 1:constraintCount
    rowCount = rowCounts(constraintIndex);
    rows = nextRow:nextRow + rowCount - 1;
    nextRow = rows(end) + 1;
    selectedPowerMap = reshape(sensitivity.positionPowerMap( ...
        segmentIndex(constraintIndex), :, :), ...
        coefficientCount, controlCount);
    restrictedMap = hullMap(:, :, constraintIndex) * selectedPowerMap;
    for dimensionIndex = 1:dimensionCount
        columns = (dimensionIndex - 1) * controlCount + ...
            (1:controlCount);
        matrix(rows, columns) = ...
            -pathConstraints.Normal(constraintIndex, dimensionIndex) * ...
            restrictedMap(1:rowCount, :);
    end
end
end
