function [inequalityMatrix, equalityMatrix] = ...
        buildFixedConstraintMatrices( ...
        segmentCount, duration, dimensionCount, limits, pathConstraints)
%% Section 0: Header & Readme
% SYNTAX
%   [inequalityMatrix, equalityMatrix] = ...
%       hs3Internal.buildFixedConstraintMatrices(segmentCount, duration, ...
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

sensitivity = hs3Internal.affineSensitivity( ...
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
    columnIndex = (dimensionIndex - 1) * controlCount + (1:controlCount);
    for quantityIndex = 1:numel(mapFields)
        bernsteinMap = mapToBernstein(sensitivity.(mapFields(quantityIndex)));
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
equalityMatrix = zeros(3 * dimensionCount, dimensionCount * controlCount);
for dimensionIndex = 1:dimensionCount
    columnIndex = (dimensionIndex - 1) * controlCount + (1:controlCount);
    rowIndex = dimensionIndex + (0:2) * dimensionCount;
    equalityMatrix(rowIndex, columnIndex) = terminalMap;
end
end

%% Section 2: Local Functions

function bernsteinMap = mapToBernstein(powerMap)
%% Section 0: Header & Readme
% SYNTAX
%   bernsteinMap = mapToBernstein(powerMap)
%**************************************************************************
% PURPOSE
%   - Convert coefficient sensitivities through the exact Bernstein basis.
%**************************************************************************
% INPUTS
%   - powerMap (segment-by-power-by-control numeric), scalar-coordinate map.
%**************************************************************************
% OUTPUTS
%   - bernsteinMap ((power*segment)-by-control numeric), coefficient-fast.
%**************************************************************************
% UNITS
%   - Values retain powerMap sensitivity units.
%**************************************************************************
coefficientCount = size(powerMap, 2);
segmentCount = size(powerMap, 1);
controlCount = size(powerMap, 3);
powerMatrix = reshape(permute(powerMap, [2 1 3]), ...
    coefficientCount, []);
bernsteinMatrix = hs3Internal.powerToBernstein(powerMatrix);
bernsteinMap = reshape( ...
    bernsteinMatrix, coefficientCount * segmentCount, controlCount);
end

function matrix = pathConstraintMatrix( ...
        sensitivity, pathConstraints, dimensionCount)
%% Section 0: Header & Readme
% SYNTAX
%   matrix = pathConstraintMatrix( ...
%       sensitivity, pathConstraints, dimensionCount)
%**************************************************************************
% PURPOSE
%   - Differentiate point or complete-interval affine path half-spaces.
%**************************************************************************
% INPUTS
%   - sensitivity (scalar struct), dimension-neutral HS3 affine maps.
%   - pathConstraints (normalized scalar struct), affine path rows.
%   - dimensionCount (positive integer scalar), modeled coordinates.
%**************************************************************************
% OUTPUTS
%   - matrix (M-by-(D*C) numeric), exact path-constraint jerk Jacobian.
%**************************************************************************
% UNITS
%   - Rows retain coordinate-per-jerk sensitivity units.
%**************************************************************************
controlCount = sensitivity.ControlCount;
coefficientCount = size(sensitivity.positionPowerMap, 2);
constraintCount = numel(pathConstraints.Tau);
if constraintCount == 0
    matrix = zeros(0, dimensionCount * controlCount);
    return;
end
[segmentIndex, hullMap] = hs3Internal.subintervalHullMap( ...
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
